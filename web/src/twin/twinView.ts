import * as THREE from 'three';

import type { ClassInfo, SceneDescription } from '../net/client';
import type { MapData } from '../map/mapData';
import {
  buildBuildingLayer,
  buildImageLayer,
  buildStylizedLayer,
  recolorMap,
} from '../map/mapLayers';
import type { WorldObject, WorldState } from '../net/schema';
import { alignAxis, FlowField, roadAxis } from './flowField';
import { angleDelta, WorldInterpolator } from './interpolation';
import {
  buildClassMesh,
  buildConflictHalo,
  buildContactShadow,
  buildSelectionRing,
  buildVruRing,
  disposeObject,
  makeMaterials,
  type ObjectLook,
  setFade,
} from './meshes';
import { distanceForHeight, OrbitRig, type ViewMode } from './orbitRig';
import { setTwinTheme, type Theme, twinPalette } from './palette';
import { TrailLayer } from './trails';

const GROUND_SIZE_M = 6000;
const GRID_MINOR_M = 10;
const GRID_MAJOR_M = 50;
const GRID_MINOR_EXTENT_M = 500;
const GRID_MAJOR_EXTENT_M = 1500;
const INITIAL_VIEW_M = 80;
const HALO_PULSE_HZ = 1.2;
const DEFAULT_CAMERA_HEIGHT_M = 6;
const VIEW_WEDGE_DEG = 54;
const ZONE_Z = 0.08;
// Sun: fixed direction, placed far enough to light (and shadow) even very tall buildings.
const SUN_DIRECTION = new THREE.Vector3(-0.38, -0.6, 0.7).normalize();
const SUN_DISTANCE_M = 900;

// Entering and leaving road users fade instead of popping.
const FADE_IN_S = 0.7;
const FADE_OUT_S = 0.9;
// Turn rates of the drawn heading (rad/s): vehicles cannot spin on the spot.
const VEHICLE_TURN_RATE = THREE.MathUtils.degToRad(75);
const VRU_TURN_RATE = THREE.MathUtils.degToRad(300);
// A track's own heading is trusted once it moves this fast with a small speed uncertainty.
const VEHICLE_MOVING_MPS = 1.2;
const VRU_MOVING_MPS = 0.5;
// Tracks younger than this follow the learned traffic direction when their own heading disagrees.
const YOUNG_TRACK_S = 1.5;
// Only confident, older vehicle tracks teach the flow field.
const LEARN_MIN_SPEED_MPS = 2;
const LEARN_MIN_AGE_S = 1;
const FLOW_SAVE_INTERVAL_MS = 15000;

type ZoneColor = 'crosswalk' | 'lane' | 'intersection';
const ZONE_STYLE: Record<string, { color: ZoneColor; fill: number; edge: number }> = {
  crosswalk: { color: 'crosswalk', fill: 0.16, edge: 0.85 },
  lane: { color: 'lane', fill: 0.05, edge: 0.35 },
  intersection: { color: 'intersection', fill: 0.05, edge: 0.4 },
};

export type BaseLayer = 'map' | 'ortho' | 'roadmap';

/** Placement of map layers given in their own east-north-up frame: scene = R(yaw) · map + (x, y). */
export interface MapTransform {
  yaw_deg: number;
  x_m: number;
  y_m: number;
}

interface TrackedMesh {
  objectClass: string;
  look: string;
  root: THREE.Group;
  heading: THREE.Group;
  model: THREE.Group;
  halo: THREE.Mesh | null;
  selection: THREE.Mesh | null;
  /** When the track appeared (ms) and, once gone, when it left. */
  born: number;
  lost: number | null;
  /** Drawn heading, turned towards the target heading at a limited rate. */
  drawnHeading: number;
  fade: number;
  last: WorldObject;
  x: number;
  y: number;
}

interface ZoneMesh {
  type: string;
  fill: THREE.MeshBasicMaterial;
  edge: THREE.LineBasicMaterial;
  base: number;
}

/** Ids of the objects in active conflict events and in any active event. */
export function eventParticipants(state: WorldState): { conflict: Set<number>; any: Set<number> } {
  const conflict = new Set<number>();
  const any = new Set<number>();
  for (const event of state.events) {
    if (event.state !== 'active') {
      continue;
    }
    for (const id of event.object_ids) {
      any.add(id);
      if (event.type === 'VRU_VEHICLE_CONFLICT') {
        conflict.add(id);
      }
    }
  }
  return { conflict, any };
}

/**
 * Centre of all zone corners; without zones the camera position, or the world origin (the geo
 * anchor) for scenes that are not calibrated yet.
 */
function initialCenter(scene: SceneDescription): { x: number; y: number } {
  const corners = scene.zones.features.flatMap((f) => f.geometry.coordinates[0] ?? []);
  if (corners.length === 0) {
    const [x, y] = scene.camera_ground_position_m ?? [0, 0];
    return { x, y };
  }
  const sum = corners.reduce((acc, [x = 0, y = 0]) => ({ x: acc.x + x, y: acc.y + y }), {
    x: 0,
    y: 0,
  });
  return { x: sum.x / corners.length, y: sum.y / corners.length };
}

/** Whether a track's own velocity is reliable enough to orient it. */
function moving(object: WorldObject): boolean {
  const threshold = object.vru ? VRU_MOVING_MPS : VEHICLE_MOVING_MPS;
  const std = object.speed_std_mps;
  return (
    object.speed_mps > threshold && (std === null || std < Math.max(0.6, 0.5 * object.speed_mps))
  );
}

function ownHeading(object: WorldObject): number {
  return object.heading_rad ?? Math.atan2(object.vy, object.vx);
}

/**
 * The digital twin: the map of the scene (stylized from OpenStreetMap, aerial photo or road map,
 * with LoD2 buildings), its zones and camera, and one low-poly mesh per road user, moved smoothly
 * between world states. Road users fade in and out and enter facing the learned direction of
 * traffic. Viewed top-down (2D) or as a tilted perspective (3D), in a night or day theme.
 */
export class TwinView {
  private readonly renderer: THREE.WebGLRenderer;
  private readonly scene = new THREE.Scene();
  private readonly rig: OrbitRig;
  private readonly sun = new THREE.DirectionalLight(0xfff2e0, 2);
  private readonly hemisphere = new THREE.HemisphereLight(0xffffff, 0x000000, 1);
  private readonly interpolator = new WorldInterpolator();
  private readonly trails = new TrailLayer();
  private readonly raycaster = new THREE.Raycaster();
  private readonly groundMaterial = new THREE.MeshStandardMaterial({ roughness: 1 });
  private readonly gridMaterials = [new THREE.LineBasicMaterial(), new THREE.LineBasicMaterial()];
  private readonly mapRoot = new THREE.Group();
  private grids: THREE.LineSegments[] = [];
  private stylized: THREE.Group | null = null;
  private buildings: THREE.Group | null = null;
  private ortho: THREE.Mesh | null = null;
  private roadmap: THREE.Mesh | null = null;
  private map: MapData | null = null;
  private layer: BaseLayer = 'map';
  private readonly meshes = new Map<number, TrackedMesh>();
  private readonly zones = new Map<string, ZoneMesh>();
  private readonly classes = new Map<string, ClassInfo>();
  private readonly fallbackClass: ClassInfo = {
    name: 'other',
    length_m: 2,
    width_m: 1,
    height_m: 1.5,
    vru: false,
  };
  private readonly flow: FlowField;
  private showTrails = true;
  private lastFrame = performance.now();
  private lastFlowSave = performance.now();
  private shadowExtent = 0;
  private hoverPointer: { ndc: THREE.Vector2; x: number; y: number } | null = null;
  private hoveredId: number | null = null;
  private lastState: WorldState | null = null;
  private transform: MapTransform = { yaw_deg: 0, x_m: 0, y_m: 0 };
  private aligning: { start: THREE.Vector3; rotate: boolean; transform: MapTransform } | null =
    null;
  private alignMode = false;

  /** Called with the object under the pointer (null: none) and the pointer position. */
  onHover: (id: number | null, x: number, y: number) => void = () => undefined;
  /** Called while the map is dragged in align mode. */
  onMapTransform: (transform: MapTransform) => void = () => undefined;

  constructor(
    private readonly canvas: HTMLCanvasElement,
    description: SceneDescription,
    options: { siteKey?: string; theme?: Theme } = {},
  ) {
    for (const info of description.classes) {
      this.classes.set(info.name, info);
    }
    setTwinTheme(options.theme ?? 'dark');
    this.flow = new FlowField(options.siteKey ? `jp-flow-${options.siteKey}` : null);
    // A logarithmic depth buffer keeps the flat map layers, centimetres apart, from fighting for
    // the same depth at any zoom.
    this.renderer = new THREE.WebGLRenderer({
      canvas,
      antialias: true,
      powerPreference: 'high-performance',
      logarithmicDepthBuffer: true,
    });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.05;
    this.scene.background = new THREE.Color();
    this.scene.fog = new THREE.Fog(0x000000, 200, 800);

    const center = initialCenter(description);
    const camera = description.camera_ground_position_m;
    this.rig = new OrbitRig(canvas, {
      targetX: center.x,
      targetY: center.y,
      distance: distanceForHeight(INITIAL_VIEW_M),
      // The camera's viewing direction points up on screen, as in the camera image.
      azimuth: camera ? Math.atan2(center.y - camera[1], center.x - camera[0]) - Math.PI / 2 : 0,
    });
    this.addLights();
    this.addGround(center);
    this.addZones(description);
    this.addCameraMarker(description, center);
    this.scene.add(this.mapRoot, this.trails.group);
    this.applyPalette();
    canvas.addEventListener('pointerdown', (e) => this.onAlignStart(e));
    canvas.addEventListener('pointermove', (e) => {
      this.hoverPointer = { ndc: this.rig.pointer(e), x: e.offsetX, y: e.offsetY };
      this.onAlignMove(e);
    });
    canvas.addEventListener('pointerup', () => (this.aligning = null));
    canvas.addEventListener('pointerleave', () => {
      this.hoverPointer = null;
    });
    window.addEventListener('beforeunload', () => this.flow.save());
    new ResizeObserver(() => this.resize()).observe(canvas);
    this.resize();
    this.renderer.setAnimationLoop(() => this.render());
  }

  /** Adds the map layers of the scene. */
  setMap(map: MapData): void {
    this.map = map;
    this.stylized = buildStylizedLayer(map);
    this.buildings = buildBuildingLayer(map);
    this.ortho = buildImageLayer(map, map.ortho);
    this.roadmap = buildImageLayer(map, map.roadmap);
    this.mapRoot.add(this.stylized, this.buildings);
    for (const image of [this.ortho, this.roadmap]) {
      if (image) {
        this.mapRoot.add(image);
      }
    }
    this.showLayer(this.layer);
  }

  /** Places map layers that are not in the scene frame (sensor-frame scenes). */
  setMapTransform(transform: MapTransform): void {
    this.transform = { ...transform };
    this.mapRoot.rotation.z = THREE.MathUtils.degToRad(transform.yaw_deg);
    this.mapRoot.position.set(transform.x_m, transform.y_m, 0);
  }

  get mapTransform(): MapTransform {
    return { ...this.transform };
  }

  /**
   * Align mode: dragging moves the map, right or shift drag rotates it about the point where the
   * drag started; the wheel still zooms the view.
   */
  setAlignMode(on: boolean): void {
    this.alignMode = on;
    this.rig.interactive = !on;
    this.canvas.classList.toggle('aligning', on);
  }

  /** Base layer: stylized map (with grid), aerial photo or road map. */
  showLayer(layer: BaseLayer): void {
    this.layer = layer;
    if (this.stylized) {
      this.stylized.visible = layer === 'map';
    }
    if (this.ortho) {
      this.ortho.visible = layer === 'ortho';
    }
    if (this.roadmap) {
      this.roadmap.visible = layer === 'roadmap';
    }
    for (const grid of this.grids) {
      grid.visible = layer === 'map';
    }
  }

  showBuildings(visible: boolean): void {
    if (this.buildings) {
      this.buildings.visible = visible;
    }
  }

  setTrails(visible: boolean): void {
    this.showTrails = visible;
    this.trails.group.visible = visible;
    if (!visible) {
      this.trails.clear();
    }
  }

  setViewMode(mode: ViewMode): void {
    this.rig.setMode(mode);
  }

  recenter(): void {
    this.rig.recenter();
  }

  /** Night (dark) or day (light) colours of the whole scene. */
  setTheme(theme: Theme): void {
    setTwinTheme(theme);
    this.applyPalette();
    // Road users are rebuilt in the new colours on the next frame.
    for (const tracked of this.meshes.values()) {
      tracked.look = '';
    }
  }

  /** Feeds a new world state. */
  push(state: WorldState): void {
    if (this.lastState && state.frame < this.lastState.frame) {
      // Seek or loop restart: no fading across the jump.
      this.trails.clear();
      for (const [id, tracked] of this.meshes) {
        this.scene.remove(tracked.root);
        disposeObject(tracked.root);
        this.meshes.delete(id);
      }
    }
    this.lastState = state;
    this.interpolator.push(state, performance.now());
  }

  private applyPalette(): void {
    const p = twinPalette();
    (this.scene.background as THREE.Color).setHex(p.background);
    (this.scene.fog as THREE.Fog).color.setHex(p.background);
    this.groundMaterial.color.setHex(p.ground);
    this.gridMaterials[0]?.color.setHex(p.gridMinor);
    this.gridMaterials[1]?.color.setHex(p.gridMajor);
    this.hemisphere.color.setHex(p.sky);
    this.hemisphere.groundColor.setHex(p.skyGround);
    this.hemisphere.intensity = p.hemisphere;
    this.sun.intensity = p.sun;
    for (const zone of this.zones.values()) {
      const style = ZONE_STYLE[zone.type] ?? ZONE_STYLE['lane'];
      if (style) {
        zone.fill.color.setHex(p[style.color]);
        zone.edge.color.setHex(p[style.color]);
      }
    }
    recolorMap(this.mapRoot, p);
  }

  private resize(): void {
    const { clientWidth: width, clientHeight: height } = this.canvas;
    if (width === 0 || height === 0) {
      return;
    }
    this.renderer.setSize(width, height, false);
    this.rig.resize();
  }

  private addLights(): void {
    this.scene.add(this.hemisphere);
    this.sun.castShadow = true;
    this.sun.shadow.mapSize.set(4096, 4096);
    this.sun.shadow.radius = 3;
    this.sun.shadow.bias = -0.0005;
    this.sun.shadow.normalBias = 0.05;
    this.scene.add(this.sun, this.sun.target);
  }

  /** Sun, shadow area and fog follow the view; the sun's direction stays fixed. */
  private updateEnvironment(): void {
    const { x, y } = this.rig.target;
    const d = this.rig.distance;
    this.sun.position.set(x, y, 0).addScaledVector(SUN_DIRECTION, SUN_DISTANCE_M);
    this.sun.target.position.set(x, y, 0);
    const extent = THREE.MathUtils.clamp(d * 0.9, 60, 400);
    if (Math.abs(extent - this.shadowExtent) > this.shadowExtent * 0.15) {
      this.shadowExtent = extent;
      Object.assign(this.sun.shadow.camera, {
        left: -extent,
        right: extent,
        top: extent,
        bottom: -extent,
        near: 10,
        far: SUN_DISTANCE_M * 2,
      });
      this.sun.shadow.camera.updateProjectionMatrix();
    }
    const fog = this.scene.fog as THREE.Fog;
    fog.near = d * 1.6;
    fog.far = d * 5.5;
  }

  private addGround(center: { x: number; y: number }): void {
    const ground = new THREE.Mesh(
      new THREE.PlaneGeometry(GROUND_SIZE_M, GROUND_SIZE_M),
      this.groundMaterial,
    );
    ground.position.set(center.x, center.y, 0);
    ground.receiveShadow = true;
    this.scene.add(ground);
    this.grids = [
      this.grid(center, GRID_MINOR_M, GRID_MINOR_EXTENT_M, 0, 0.005),
      this.grid(center, GRID_MAJOR_M, GRID_MAJOR_EXTENT_M, 1, 0.01),
    ];
    this.scene.add(...this.grids);
  }

  private grid(
    center: { x: number; y: number },
    spacing: number,
    extent: number,
    material: number,
    z: number,
  ): THREE.LineSegments {
    const points: THREE.Vector3[] = [];
    const x0 = Math.round(center.x / spacing) * spacing;
    const y0 = Math.round(center.y / spacing) * spacing;
    for (let d = -extent; d <= extent; d += spacing) {
      points.push(new THREE.Vector3(x0 + d, y0 - extent, z));
      points.push(new THREE.Vector3(x0 + d, y0 + extent, z));
      points.push(new THREE.Vector3(x0 - extent, y0 + d, z));
      points.push(new THREE.Vector3(x0 + extent, y0 + d, z));
    }
    return new THREE.LineSegments(
      new THREE.BufferGeometry().setFromPoints(points),
      this.gridMaterials[material],
    );
  }

  private addZones(description: SceneDescription): void {
    for (const feature of description.zones.features) {
      const ring = feature.geometry.coordinates[0] ?? [];
      const type = feature.properties.type;
      const style = ZONE_STYLE[type] ?? ZONE_STYLE['lane'];
      if (!style) {
        continue;
      }
      const shape = new THREE.Shape(ring.map(([x = 0, y = 0]) => new THREE.Vector2(x, y)));
      const fillMaterial = new THREE.MeshBasicMaterial({
        transparent: true,
        opacity: style.fill,
        depthWrite: false,
      });
      const fill = new THREE.Mesh(new THREE.ShapeGeometry(shape), fillMaterial);
      fill.position.z = ZONE_Z;
      const edgeMaterial = new THREE.LineBasicMaterial({ transparent: true, opacity: style.edge });
      const edge = new THREE.LineLoop(
        new THREE.BufferGeometry().setFromPoints(
          ring.map(([x = 0, y = 0]) => new THREE.Vector3(x, y, ZONE_Z + 0.01)),
        ),
        edgeMaterial,
      );
      this.scene.add(fill, edge);
      this.zones.set(feature.properties.id, {
        type,
        fill: fillMaterial,
        edge: edgeMaterial,
        base: style.fill,
      });
    }
  }

  /** Pole at the camera position with the camera head and its viewing direction on the ground. */
  private addCameraMarker(description: SceneDescription, center: { x: number; y: number }): void {
    if (!description.camera_ground_position_m) {
      return;
    }
    const p = twinPalette();
    const [x, y] = description.camera_ground_position_m;
    const height = description.camera_height_m ?? DEFAULT_CAMERA_HEIGHT_M;
    const direction = Math.atan2(center.y - y, center.x - x);
    const group = new THREE.Group();
    group.position.set(x, y, 0);

    const metal = new THREE.MeshStandardMaterial({
      color: 0x8a97ab,
      roughness: 0.4,
      metalness: 0.6,
    });
    const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.16, height, 12), metal);
    pole.rotation.x = Math.PI / 2;
    pole.position.z = height / 2;
    pole.castShadow = true;
    const head = new THREE.Mesh(
      new THREE.BoxGeometry(0.9, 0.38, 0.38),
      new THREE.MeshStandardMaterial({ color: p.camera, roughness: 0.35, metalness: 0.2 }),
    );
    head.position.set(0.3 * Math.cos(direction), 0.3 * Math.sin(direction), height + 0.1);
    head.rotation.z = direction;
    head.castShadow = true;
    const lens = new THREE.Mesh(
      new THREE.CircleGeometry(0.13, 16),
      new THREE.MeshBasicMaterial({ color: p.cameraView }),
    );
    lens.rotation.y = Math.PI / 2;
    lens.position.set(0.46, 0, 0);
    head.add(lens);

    // Translucent wedge on the ground towards the observed area.
    const reach = THREE.MathUtils.clamp(Math.hypot(center.x - x, center.y - y) * 0.8, 15, 60);
    const half = THREE.MathUtils.degToRad(VIEW_WEDGE_DEG / 2);
    const wedgeShape = new THREE.Shape();
    wedgeShape.moveTo(0, 0);
    wedgeShape.absarc(0, 0, reach, direction - half, direction + half, false);
    wedgeShape.lineTo(0, 0);
    const wedge = new THREE.Mesh(
      new THREE.ShapeGeometry(wedgeShape, 24),
      new THREE.MeshBasicMaterial({
        color: p.cameraView,
        transparent: true,
        opacity: 0.05,
        depthWrite: false,
      }),
    );
    wedge.position.z = ZONE_Z - 0.01;
    const outline = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(
        wedgeShape.getPoints(24).map((q) => new THREE.Vector3(q.x, q.y, ZONE_Z)),
      ),
      new THREE.LineBasicMaterial({ color: p.cameraView, transparent: true, opacity: 0.35 }),
    );
    const base = new THREE.Mesh(
      new THREE.RingGeometry(0.5, 0.8, 24),
      new THREE.MeshBasicMaterial({ color: p.cameraView, transparent: true, opacity: 0.8 }),
    );
    base.position.z = ZONE_Z + 0.02;
    group.add(pole, head, wedge, outline, base);
    this.scene.add(group);
  }

  private classInfo(name: string): ClassInfo {
    return this.classes.get(name) ?? this.fallbackClass;
  }

  /**
   * Heading a new road user enters with: the learned traffic direction where it appears, else
   * the direction of a one-way road (or the road axis turned to its own motion), else its own.
   */
  private entryHeading(object: WorldObject, fallback: number): number {
    const own = moving(object) ? ownHeading(object) : null;
    if (!object.vru) {
      const flow = this.flow.direction(object.x, object.y);
      if (flow !== null) {
        return flow;
      }
      const road = roadAxis(this.map?.osm ?? null, object.x, object.y);
      if (road) {
        return road.oneway ? road.angle : alignAxis(road.angle, own ?? fallback);
      }
    }
    return own ?? fallback;
  }

  /** Heading the drawn heading turns towards. */
  private targetHeading(object: WorldObject, tracked: TrackedMesh, now: number): number {
    if (!moving(object)) {
      // Standing road users keep the way they face.
      return tracked.drawnHeading;
    }
    const own = ownHeading(object);
    const age = (now - tracked.born) / 1000;
    const flow = object.vru ? null : this.flow.direction(object.x, object.y);
    // A young track's heading is still settling: where traffic has a clear direction and the
    // track disagrees, the traffic direction wins until the track proves itself.
    if (flow !== null && age < YOUNG_TRACK_S && Math.abs(angleDelta(flow, own)) > Math.PI / 2) {
      return flow;
    }
    return own;
  }

  private ensureMesh(
    object: WorldObject,
    look: ObjectLook,
    now: number,
    heading: number,
  ): TrackedMesh {
    const key = `${Number(look.highlighted)}${Number(look.conflict)}${Number(look.validated)}`;
    const existing = this.meshes.get(object.id);
    if (existing && existing.objectClass === object.class && existing.look === key) {
      return existing;
    }
    const info = this.classInfo(object.class);
    const model = buildClassMesh(info, makeMaterials(object.class, look));
    model.userData['id'] = object.id;
    if (existing && existing.objectClass === object.class) {
      existing.heading.remove(existing.model);
      disposeObject(existing.model);
      existing.heading.add(model);
      existing.model = model;
      existing.look = key;
      if (existing.fade < 1) {
        setFade(model, existing.fade);
      }
      return existing;
    }
    if (existing) {
      this.scene.remove(existing.root);
      disposeObject(existing.root);
    }
    const root = new THREE.Group();
    const headingGroup = new THREE.Group();
    headingGroup.add(buildContactShadow(info), model);
    root.add(headingGroup);
    if (info.vru) {
      root.add(buildVruRing(info));
    }
    this.scene.add(root);
    const tracked: TrackedMesh = {
      objectClass: object.class,
      look: key,
      root,
      heading: headingGroup,
      model,
      halo: null,
      selection: null,
      born: existing?.born ?? now,
      lost: null,
      drawnHeading: existing?.drawnHeading ?? this.entryHeading(object, heading),
      fade: existing?.fade ?? 0,
      last: object,
      x: object.x,
      y: object.y,
    };
    setFade(root, tracked.fade);
    this.meshes.set(object.id, tracked);
    return tracked;
  }

  private updateObjects(now: number, dt: number): void {
    const state = this.interpolator.current();
    if (!state) {
      return;
    }
    const poses = this.interpolator.poses(now);
    const { conflict, any } = eventParticipants(state);
    const pulse = 0.35 + 0.25 * Math.sin((now / 1000) * 2 * Math.PI * HALO_PULSE_HZ);
    const trailPositions = new Map<number, { x: number; y: number; className: string }>();
    for (const object of state.objects) {
      const pose = poses.get(object.id);
      if (!pose) {
        continue;
      }
      const tracked = this.ensureMesh(
        object,
        {
          highlighted: any.has(object.id),
          conflict: conflict.has(object.id),
          validated: object.validated,
        },
        now,
        pose.heading,
      );
      tracked.lost = null;
      tracked.last = object;
      tracked.x = pose.x;
      tracked.y = pose.y;
      tracked.root.position.set(pose.x, pose.y, 0);

      // Heading: turn towards the target at a limited rate.
      const target = this.targetHeading(object, tracked, now);
      const rate = object.vru ? VRU_TURN_RATE : VEHICLE_TURN_RATE;
      const delta = angleDelta(tracked.drawnHeading, target);
      tracked.drawnHeading += THREE.MathUtils.clamp(delta, -rate * dt, rate * dt);
      tracked.heading.rotation.z = tracked.drawnHeading;

      if (tracked.fade < 1) {
        tracked.fade = Math.min(1, tracked.fade + dt / FADE_IN_S);
        setFade(tracked.root, tracked.fade);
      }
      if (
        !object.vru &&
        moving(object) &&
        object.speed_mps > LEARN_MIN_SPEED_MPS &&
        (now - tracked.born) / 1000 > LEARN_MIN_AGE_S
      ) {
        this.flow.learn(object.x, object.y, object.vx, object.vy);
      }

      if (conflict.has(object.id) && !tracked.halo) {
        tracked.halo = buildConflictHalo(this.classInfo(object.class));
        tracked.root.add(tracked.halo);
      } else if (!conflict.has(object.id) && tracked.halo) {
        tracked.root.remove(tracked.halo);
        disposeObject(tracked.halo);
        tracked.halo = null;
      }
      if (tracked.halo) {
        (tracked.halo.material as THREE.MeshBasicMaterial).opacity = pulse;
        tracked.halo.scale.setScalar(
          1 + 0.08 * Math.sin((now / 1000) * 2 * Math.PI * HALO_PULSE_HZ),
        );
      }
      if (this.showTrails && tracked.fade >= 1) {
        trailPositions.set(object.id, { x: pose.x, y: pose.y, className: object.class });
      }
    }

    // Road users that left keep going along their last velocity while they fade out.
    const present = new Set(state.objects.map((o) => o.id));
    for (const [id, tracked] of this.meshes) {
      if (present.has(id)) {
        continue;
      }
      tracked.lost ??= now;
      tracked.x += tracked.last.vx * dt;
      tracked.y += tracked.last.vy * dt;
      tracked.root.position.set(tracked.x, tracked.y, 0);
      tracked.fade = Math.max(0, tracked.fade - dt / FADE_OUT_S);
      setFade(tracked.root, tracked.fade);
      if (tracked.fade <= 0) {
        this.scene.remove(tracked.root);
        disposeObject(tracked.root);
        this.meshes.delete(id);
      }
    }
    if (this.showTrails) {
      this.trails.update(trailPositions);
    }

    // Crosswalks with a road user on them glow.
    const occupied = new Set(
      state.events.filter((e) => e.state === 'active' && e.zone !== null).map((e) => e.zone),
    );
    for (const [id, zone] of this.zones) {
      const active = occupied.has(id);
      zone.fill.opacity = active ? zone.base * 2.2 + 0.08 * Math.sin(now / 160) : zone.base;
      zone.edge.opacity = active ? 1 : Math.min(zone.base * 5.3, 0.85);
    }
    if (now - this.lastFlowSave > FLOW_SAVE_INTERVAL_MS) {
      this.lastFlowSave = now;
      this.flow.save();
    }
  }

  /** The road user under the pointer gets a ring; meshes are not rebuilt, so nothing flickers. */
  private updateHover(): void {
    let id: number | null = null;
    if (this.hoverPointer && !this.rig.dragging && !this.alignMode) {
      this.scene.updateMatrixWorld();
      this.raycaster.setFromCamera(this.hoverPointer.ndc, this.rig.camera);
      const models = [...this.meshes.values()].filter((m) => m.lost === null).map((m) => m.model);
      for (const hit of this.raycaster.intersectObjects(models, true)) {
        let node: THREE.Object3D | null = hit.object;
        while (node && node.userData['id'] === undefined) {
          node = node.parent;
        }
        if (node) {
          id = node.userData['id'] as number;
          break;
        }
      }
    }
    if (id !== this.hoveredId) {
      for (const [objectId, tracked] of this.meshes) {
        const selected = objectId === id;
        if (selected && !tracked.selection) {
          tracked.selection = buildSelectionRing(this.classInfo(tracked.objectClass));
          tracked.root.add(tracked.selection);
        } else if (!selected && tracked.selection) {
          tracked.root.remove(tracked.selection);
          disposeObject(tracked.selection);
          tracked.selection = null;
        }
      }
      this.hoveredId = id;
    }
    this.onHover(id, this.hoverPointer?.x ?? 0, this.hoverPointer?.y ?? 0);
    this.canvas.classList.toggle('hovering', id !== null);
  }

  private onAlignStart(e: PointerEvent): void {
    if (!this.alignMode) {
      return;
    }
    const start = this.rig.groundAt(e.offsetX, e.offsetY);
    if (start) {
      this.aligning = {
        start,
        rotate: e.button === 2 || e.shiftKey,
        transform: { ...this.transform },
      };
      this.canvas.setPointerCapture(e.pointerId);
    }
  }

  private onAlignMove(e: PointerEvent): void {
    if (!this.aligning) {
      return;
    }
    const { start, transform } = this.aligning;
    if (this.aligning.rotate) {
      // Rotate about the start point by the horizontal drag.
      const angle = (e.movementX / this.canvas.clientWidth) * Math.PI;
      const cos = Math.cos(angle);
      const sin = Math.sin(angle);
      const ox = this.transform.x_m - start.x;
      const oy = this.transform.y_m - start.y;
      this.setMapTransform({
        yaw_deg: this.transform.yaw_deg + THREE.MathUtils.radToDeg(angle),
        x_m: start.x + ox * cos - oy * sin,
        y_m: start.y + ox * sin + oy * cos,
      });
    } else {
      const now = this.rig.groundAt(e.offsetX, e.offsetY);
      if (!now) {
        return;
      }
      this.setMapTransform({
        yaw_deg: transform.yaw_deg,
        x_m: transform.x_m + now.x - start.x,
        y_m: transform.y_m + now.y - start.y,
      });
    }
    this.onMapTransform(this.mapTransform);
  }

  private render(): void {
    const now = performance.now();
    const dt = Math.min((now - this.lastFrame) / 1000, 0.1);
    this.lastFrame = now;
    this.rig.tick(dt);
    this.updateEnvironment();
    this.updateObjects(now, dt);
    this.updateHover();
    this.renderer.render(this.scene, this.rig.camera);
  }
}
