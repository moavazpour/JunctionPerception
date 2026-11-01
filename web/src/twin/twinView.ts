import * as THREE from 'three';

import type { ClassInfo, SceneDescription } from '../net/client';
import type { MapData } from '../map/mapData';
import { buildBuildingLayer, buildOrthoLayer, buildStylizedLayer } from '../map/mapLayers';
import type { WorldState } from '../net/schema';
import { WorldInterpolator } from './interpolation';
import {
  buildClassMesh,
  buildConflictHalo,
  buildContactShadow,
  buildVruRing,
  disposeObject,
  makeMaterials,
  type ObjectLook,
} from './meshes';
import { distanceForHeight, OrbitRig, type ViewMode } from './orbitRig';
import { TWIN } from './palette';
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

const ZONE_STYLE: Record<string, { color: number; fill: number; edge: number }> = {
  crosswalk: { color: TWIN.crosswalk, fill: 0.16, edge: 0.85 },
  lane: { color: TWIN.lane, fill: 0.05, edge: 0.35 },
  intersection: { color: TWIN.intersection, fill: 0.05, edge: 0.4 },
};

export type BaseLayer = 'map' | 'ortho';

interface TrackedMesh {
  objectClass: string;
  look: string;
  root: THREE.Group;
  heading: THREE.Group;
  model: THREE.Group;
  halo: THREE.Mesh | null;
}

interface ZoneMesh {
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

/**
 * The digital twin: the map of the scene (stylized from OpenStreetMap or the orthophoto, with
 * LoD2 buildings), its zones and camera, and one low-poly mesh per road user, moved smoothly
 * between world states. Viewed top-down (2D) or as a tilted perspective (3D).
 */
export class TwinView {
  private readonly renderer: THREE.WebGLRenderer;
  private readonly scene = new THREE.Scene();
  private readonly rig: OrbitRig;
  private readonly sun = new THREE.DirectionalLight(0xfff2e0, 2.4);
  private readonly interpolator = new WorldInterpolator();
  private readonly trails = new TrailLayer();
  private readonly raycaster = new THREE.Raycaster();
  private grids: THREE.LineSegments[] = [];
  private stylized: THREE.Group | null = null;
  private buildings: THREE.Group | null = null;
  private ortho: THREE.Mesh | null = null;
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
  private showTrails = true;
  private lastFrame = performance.now();
  private shadowExtent = 0;
  private hoverPointer: { ndc: THREE.Vector2; x: number; y: number } | null = null;
  private hoveredId: number | null = null;
  private lastState: WorldState | null = null;

  /** Called when the object under the pointer changes (null: none). */
  onHover: (id: number | null, x: number, y: number) => void = () => undefined;

  constructor(
    private readonly canvas: HTMLCanvasElement,
    description: SceneDescription,
  ) {
    for (const info of description.classes) {
      this.classes.set(info.name, info);
    }
    this.renderer = new THREE.WebGLRenderer({
      canvas,
      antialias: true,
      powerPreference: 'high-performance',
    });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.05;
    this.scene.background = new THREE.Color(TWIN.background);
    this.scene.fog = new THREE.Fog(TWIN.background, 200, 800);

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
    this.scene.add(this.trails.group);
    canvas.addEventListener('pointermove', (e) => {
      this.hoverPointer = { ndc: this.rig.pointer(e), x: e.offsetX, y: e.offsetY };
    });
    canvas.addEventListener('pointerleave', () => {
      this.hoverPointer = null;
    });
    new ResizeObserver(() => this.resize()).observe(canvas);
    this.resize();
    this.renderer.setAnimationLoop(() => this.render());
  }

  /** Adds the map layers of the scene; the stylized map is shown first. */
  setMap(map: MapData): void {
    this.stylized = buildStylizedLayer(map);
    this.buildings = buildBuildingLayer(map);
    this.ortho = buildOrthoLayer(map);
    this.scene.add(this.stylized, this.buildings);
    if (this.ortho) {
      this.scene.add(this.ortho);
    }
    this.showLayer('map');
  }

  /** Base layer: stylized map (with grid) or orthophoto. */
  showLayer(layer: BaseLayer): void {
    if (this.stylized) {
      this.stylized.visible = layer === 'map';
    }
    if (this.ortho) {
      this.ortho.visible = layer === 'ortho';
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

  /** Feeds a new world state. */
  push(state: WorldState): void {
    if (this.lastState && state.frame < this.lastState.frame) {
      this.trails.clear();
    }
    this.lastState = state;
    this.interpolator.push(state, performance.now());
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
    this.scene.add(new THREE.HemisphereLight(TWIN.sky, TWIN.ground, 0.9));
    this.sun.castShadow = true;
    this.sun.shadow.mapSize.set(4096, 4096);
    this.sun.shadow.radius = 3;
    this.sun.shadow.bias = -0.0004;
    this.sun.shadow.normalBias = 0.02;
    this.scene.add(this.sun, this.sun.target);
  }

  /** Sun, shadow area and fog follow the view. */
  private updateEnvironment(): void {
    const { x, y } = this.rig.target;
    const d = this.rig.distance;
    this.sun.position.set(x - 70, y - 110, 180);
    this.sun.target.position.set(x, y, 0);
    const extent = THREE.MathUtils.clamp(d * 0.9, 60, 400);
    if (Math.abs(extent - this.shadowExtent) > this.shadowExtent * 0.15) {
      this.shadowExtent = extent;
      Object.assign(this.sun.shadow.camera, {
        left: -extent,
        right: extent,
        top: extent,
        bottom: -extent,
        near: 1,
        far: 600,
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
      new THREE.MeshStandardMaterial({ color: TWIN.ground, roughness: 1 }),
    );
    ground.position.set(center.x, center.y, 0);
    ground.receiveShadow = true;
    this.scene.add(ground);
    this.grids = [
      this.grid(center, GRID_MINOR_M, GRID_MINOR_EXTENT_M, TWIN.gridMinor, 0.004),
      this.grid(center, GRID_MAJOR_M, GRID_MAJOR_EXTENT_M, TWIN.gridMajor, 0.005),
    ];
    this.scene.add(...this.grids);
  }

  private grid(
    center: { x: number; y: number },
    spacing: number,
    extent: number,
    color: number,
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
      new THREE.LineBasicMaterial({ color }),
    );
  }

  private addZones(description: SceneDescription): void {
    for (const feature of description.zones.features) {
      const ring = feature.geometry.coordinates[0] ?? [];
      const style = ZONE_STYLE[feature.properties.type] ?? ZONE_STYLE['lane'];
      if (!style) {
        continue;
      }
      const shape = new THREE.Shape(ring.map(([x = 0, y = 0]) => new THREE.Vector2(x, y)));
      const fillMaterial = new THREE.MeshBasicMaterial({
        color: style.color,
        transparent: true,
        opacity: style.fill,
        depthWrite: false,
      });
      const fill = new THREE.Mesh(new THREE.ShapeGeometry(shape), fillMaterial);
      fill.position.z = 0.015;
      const edgeMaterial = new THREE.LineBasicMaterial({
        color: style.color,
        transparent: true,
        opacity: style.edge,
      });
      const edge = new THREE.LineLoop(
        new THREE.BufferGeometry().setFromPoints(
          ring.map(([x = 0, y = 0]) => new THREE.Vector3(x, y, 0.03)),
        ),
        edgeMaterial,
      );
      this.scene.add(fill, edge);
      this.zones.set(feature.properties.id, {
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
      new THREE.MeshStandardMaterial({ color: TWIN.camera, roughness: 0.35, metalness: 0.2 }),
    );
    head.position.set(0.3 * Math.cos(direction), 0.3 * Math.sin(direction), height + 0.1);
    head.rotation.z = direction;
    head.castShadow = true;
    const lens = new THREE.Mesh(
      new THREE.CircleGeometry(0.13, 16),
      new THREE.MeshBasicMaterial({ color: TWIN.cameraView }),
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
        color: TWIN.cameraView,
        transparent: true,
        opacity: 0.05,
        depthWrite: false,
      }),
    );
    wedge.position.z = 0.012;
    const outline = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(
        wedgeShape.getPoints(24).map((p) => new THREE.Vector3(p.x, p.y, 0.035)),
      ),
      new THREE.LineBasicMaterial({ color: TWIN.cameraView, transparent: true, opacity: 0.35 }),
    );
    const base = new THREE.Mesh(
      new THREE.RingGeometry(0.5, 0.8, 24),
      new THREE.MeshBasicMaterial({ color: TWIN.cameraView, transparent: true, opacity: 0.8 }),
    );
    base.position.z = 0.04;
    group.add(pole, head, wedge, outline, base);
    this.scene.add(group);
  }

  private classInfo(name: string): ClassInfo {
    return this.classes.get(name) ?? this.fallbackClass;
  }

  private ensureMesh(id: number, objectClass: string, look: ObjectLook): TrackedMesh {
    const key = `${Number(look.highlighted)}${Number(look.conflict)}${Number(look.validated)}`;
    const existing = this.meshes.get(id);
    if (existing && existing.objectClass === objectClass && existing.look === key) {
      return existing;
    }
    const info = this.classInfo(objectClass);
    const model = buildClassMesh(info, makeMaterials(objectClass, look));
    model.userData['id'] = id;
    if (existing && existing.objectClass === objectClass) {
      existing.heading.remove(existing.model);
      disposeObject(existing.model);
      existing.heading.add(model);
      existing.model = model;
      existing.look = key;
      return existing;
    }
    if (existing) {
      this.scene.remove(existing.root);
      disposeObject(existing.root);
    }
    const root = new THREE.Group();
    const heading = new THREE.Group();
    heading.add(buildContactShadow(info), model);
    root.add(heading);
    if (info.vru) {
      root.add(buildVruRing(info));
    }
    this.scene.add(root);
    const tracked = { objectClass, look: key, root, heading, model, halo: null };
    this.meshes.set(id, tracked);
    return tracked;
  }

  private updateObjects(now: number): void {
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
      const tracked = this.ensureMesh(object.id, object.class, {
        highlighted: any.has(object.id) || object.id === this.hoveredId,
        conflict: conflict.has(object.id),
        validated: object.validated,
      });
      tracked.root.position.set(pose.x, pose.y, 0);
      tracked.heading.rotation.z = pose.heading;
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
      if (this.showTrails) {
        trailPositions.set(object.id, { x: pose.x, y: pose.y, className: object.class });
      }
    }
    const present = new Set(state.objects.map((o) => o.id));
    for (const [id, tracked] of this.meshes) {
      if (!present.has(id)) {
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
  }

  private updateHover(): void {
    let id: number | null = null;
    if (this.hoverPointer && !this.rig.dragging) {
      this.raycaster.setFromCamera(this.hoverPointer.ndc, this.rig.camera);
      const models = [...this.meshes.values()].map((m) => m.model);
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
    if (id !== this.hoveredId || id !== null) {
      this.hoveredId = id;
      this.onHover(id, this.hoverPointer?.x ?? 0, this.hoverPointer?.y ?? 0);
    }
    this.canvas.classList.toggle('hovering', id !== null);
  }

  private render(): void {
    const now = performance.now();
    const dt = Math.min((now - this.lastFrame) / 1000, 0.1);
    this.lastFrame = now;
    this.rig.tick(dt);
    this.updateEnvironment();
    this.updateObjects(now);
    this.updateHover();
    this.renderer.render(this.scene, this.rig.camera);
  }
}
