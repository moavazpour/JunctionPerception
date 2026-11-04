import * as THREE from 'three';

import { perf } from '../bev/perf';
import type { ClassInfo, SceneDescription } from '../net/client';
import type { MapData } from '../map/mapData';
import {
  buildBuildingLayer,
  buildImageLayer,
  buildStylizedLayer,
  recolorMap,
} from '../map/mapLayers';
import type { WorldObject, WorldState } from '../net/schema';
import { alignAxis, FlowField, laneDirection, type RoadAxis, roadAxis } from './flowField';
import { type CameraTrack, CameraFusion } from './fusion';
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

const GROUND_MARGIN_M = 3000;
const GRID_MINOR_M = 10;
const GRID_MAJOR_M = 50;
const GRID_MINOR_EXTENT_M = 500;
const GRID_MAJOR_EXTENT_M = 1500;
const INITIAL_VIEW_M = 80;
const INTERSECTION_VIEW_M = 140;
const HALO_PULSE_HZ = 1.2;
const DEFAULT_CAMERA_HEIGHT_M = 6;
const VIEW_WEDGE_DEG = 54;
const ZONE_Z = 0.08;
// Sun: fixed direction, placed far enough to light (and shadow) even very tall buildings.
const SUN_DIRECTION = new THREE.Vector3(-0.38, -0.6, 0.7).normalize();
const SUN_DISTANCE_M = 900;
// A camera whose last world state is older than this shows no road users (stopped, lost).
const STALE_AFTER_MS = 2500;

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
const EARTH_RADIUS_M = 6378137;

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

/** A site (intersection) of the twin. */
export interface SiteSpec {
  id: string;
  name: string;
  /** Where the origin of the site's world frame lies in the twin (east, north in metres). */
  offset: { x: number; y: number };
  /** Map placement of a sensor-frame site; null for sites in east-north-up. */
  transform: MapTransform | null;
}

/** A road user as drawn: the fused tracks of the cameras of its site. */
/** A traffic light of a camera, placed in its site's frame (see Regions in calibrationApi). */
export interface SignalDef {
  id: string;
  name: string;
  position: [number, number] | null;
  /** Direction the lamps face, from +x counter-clockwise. */
  heading_deg: number;
  height_m: number;
}

type Lamp = 'red' | 'yellow' | 'green';
const LAMP_COLORS: Record<Lamp, number> = { red: 0xff3b30, yellow: 0xffb000, green: 0x2bd96b };
// Lamps lit per reported state.
const LIT: Record<string, Lamp[]> = {
  red: ['red'],
  yellow: ['yellow'],
  red_yellow: ['red', 'yellow'],
  green: ['green'],
};

interface SignalLamp {
  material: THREE.MeshStandardMaterial;
  /** Glow facing the viewer, so the colour shows from afar and from above. */
  glow: THREE.Sprite;
}

interface SignalMesh {
  camera: string;
  def: SignalDef;
  root: THREE.Group;
  lamps: Record<Lamp, SignalLamp>;
  shown: string;
}

let glowTexture: THREE.Texture | null = null;

/** Soft round spot for the lamp glows (white; tinted per lamp). */
function lampGlow(): THREE.Texture {
  if (!glowTexture) {
    const size = 64;
    const canvas = document.createElement('canvas');
    canvas.width = size;
    canvas.height = size;
    const c = canvas.getContext('2d');
    if (c) {
      const g = c.createRadialGradient(size / 2, size / 2, 0, size / 2, size / 2, size / 2);
      g.addColorStop(0, 'rgba(255,255,255,1)');
      g.addColorStop(0.35, 'rgba(255,255,255,0.75)');
      g.addColorStop(1, 'rgba(255,255,255,0)');
      c.fillStyle = g;
      c.fillRect(0, 0, size, size);
    }
    glowTexture = new THREE.CanvasTexture(canvas);
  }
  return glowTexture;
}

/** White three-lamp signal head without a pole, facing +x (the group is turned to the heading). */
function buildSignal(def: SignalDef): { root: THREE.Group; lamps: Record<Lamp, SignalLamp> } {
  const root = new THREE.Group();
  const head = new THREE.Mesh(
    new THREE.BoxGeometry(0.3, 0.42, 1.05),
    new THREE.MeshStandardMaterial({ color: 0xf2f3f5, roughness: 0.5 }),
  );
  head.position.set(0.12, 0, def.height_m + 0.35);
  head.castShadow = true;
  const lamps = {} as Record<Lamp, SignalLamp>;
  (['red', 'yellow', 'green'] as Lamp[]).forEach((lamp, i) => {
    const material = new THREE.MeshStandardMaterial({
      color: 0x22252a,
      emissive: LAMP_COLORS[lamp],
      emissiveIntensity: 0,
      roughness: 0.3,
    });
    const z = def.height_m + 0.35 + 0.32 - i * 0.32;
    const disc = new THREE.Mesh(new THREE.CircleGeometry(0.12, 20), material);
    disc.rotation.y = Math.PI / 2;
    disc.position.set(0.28, 0, z);
    const glow = new THREE.Sprite(
      new THREE.SpriteMaterial({
        map: lampGlow(),
        color: LAMP_COLORS[lamp],
        blending: THREE.AdditiveBlending,
        depthWrite: false,
        transparent: true,
      }),
    );
    glow.scale.setScalar(1.6);
    glow.position.set(0.32, 0, z);
    glow.visible = false;
    root.add(disc, glow);
    lamps[lamp] = { material, glow };
  });
  root.add(head);
  return { root, lamps };
}

export interface DrawnObject {
  key: string;
  site: string;
  cameras: string[];
  object: WorldObject;
}

/** East and north metres of a geographic position from an origin (local tangent plane). */
export function geoOffset(
  origin: { lat_deg: number; lon_deg: number },
  point: { lat_deg: number; lon_deg: number },
): { x: number; y: number } {
  const rad = Math.PI / 180;
  return {
    x: (point.lon_deg - origin.lon_deg) * rad * EARTH_RADIUS_M * Math.cos(origin.lat_deg * rad),
    y: (point.lat_deg - origin.lat_deg) * rad * EARTH_RADIUS_M,
  };
}

interface TrackedMesh {
  objectClass: string;
  look: string;
  root: THREE.Group;
  heading: THREE.Group;
  model: THREE.Group;
  halo: THREE.Mesh | null;
  selection: THREE.Mesh | null;
  /** When the road user appeared (ms) and, once gone, when it left. */
  born: number;
  lost: number | null;
  /** Drawn heading, turned towards the target heading at a limited rate. */
  drawnHeading: number;
  fade: number;
  last: WorldObject;
  cameras: string[];
  x: number;
  y: number;
}

interface ZoneMesh {
  type: string;
  fill: THREE.MeshBasicMaterial;
  edge: THREE.LineBasicMaterial;
  base: number;
}

interface CameraFeed {
  id: string;
  interpolator: WorldInterpolator;
  latest: WorldState | null;
  arrival: number;
  marker: THREE.Group | null;
}

/** Everything of one site, drawn in its own frame inside `group`. */
interface SiteLayer {
  spec: SiteSpec;
  group: THREE.Group;
  /** Map layers in their own east-north-up frame, placed by the map transform. */
  mapRoot: THREE.Group;
  trails: TrailLayer;
  map: MapData | null;
  stylized: THREE.Group | null;
  buildings: THREE.Group | null;
  ortho: THREE.Mesh | null;
  roadmap: THREE.Mesh | null;
  zones: Map<string, ZoneMesh>;
  zonesLoaded: boolean;
  /** Mean of the zone corners and camera positions (site frame), where the view looks. */
  center: { x: number; y: number } | null;
  cameras: Map<string, CameraFeed>;
  fusion: CameraFusion;
  flow: FlowField;
  meshes: Map<string, TrackedMesh>;
  drawn: Map<string, DrawnObject>;
  /** Traffic lights by "camera:id". */
  signals: Map<string, SignalMesh>;
  transform: MapTransform;
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

/** Zone corners of a scene and its camera position: the points its view should cover. */
function scenePoints(scene: SceneDescription): [number, number][] {
  const corners = scene.zones.features.flatMap((f) =>
    (f.geometry.coordinates[0] ?? []).map(([x = 0, y = 0]): [number, number] => [x, y]),
  );
  if (scene.camera_ground_position_m) {
    corners.push(scene.camera_ground_position_m);
  }
  return corners;
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
 * The digital twin of all sites: each intersection at its geographic position with its map
 * (stylized from OpenStreetMap, aerial photo or road map, with LoD2 buildings), zones and
 * cameras, and one low-poly mesh per road user. The tracks of the cameras of an intersection are
 * merged where their views overlap, moved smoothly between world states, fade in and out and
 * enter facing the learned direction of traffic. Viewed top-down (2D) or as a tilted perspective
 * (3D), in a night or day theme.
 */
export class TwinView {
  private readonly renderer: THREE.WebGLRenderer;
  private readonly scene = new THREE.Scene();
  private readonly rig: OrbitRig;
  private readonly sun = new THREE.DirectionalLight(0xfff2e0, 2);
  private readonly hemisphere = new THREE.HemisphereLight(0xffffff, 0x000000, 1);
  private readonly raycaster = new THREE.Raycaster();
  private readonly ground: THREE.Mesh;
  private readonly groundMaterial = new THREE.MeshStandardMaterial({ roughness: 1 });
  private readonly gridMaterials = [new THREE.LineBasicMaterial(), new THREE.LineBasicMaterial()];
  private grids: THREE.LineSegments[] = [];
  private layer: BaseLayer = 'map';
  private buildingsVisible = true;
  private readonly sites = new Map<string, SiteLayer>();
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
  private lastFlowSave = performance.now();
  private shadowExtent = 0;
  private hoverPointer: { ndc: THREE.Vector2; x: number; y: number } | null = null;
  private hovered: string | null = null;
  private aligning: {
    site: SiteLayer;
    start: THREE.Vector3;
    rotate: boolean;
    transform: MapTransform;
  } | null = null;
  private alignSite: SiteLayer | null = null;
  private placing: {
    site: SiteLayer;
    camera: string;
    id: string;
    position: THREE.Vector3 | null;
    marker: THREE.Mesh | null;
  } | null = null;

  /** Called with the road user under the pointer (null: none) and the pointer position. */
  onHover: (object: DrawnObject | null, x: number, y: number) => void = () => undefined;
  /** Called while the map is dragged in align mode. */
  onMapTransform: (transform: MapTransform) => void = () => undefined;
  /** Called when a traffic light was placed: position (site frame, m) and facing direction. */
  onSignalPlaced: (
    site: string,
    camera: string,
    id: string,
    position: [number, number],
    headingDeg: number,
  ) => void = () => undefined;

  constructor(
    private readonly canvas: HTMLCanvasElement,
    classes: ClassInfo[],
    options: { theme?: Theme } = {},
  ) {
    for (const info of classes) {
      this.classes.set(info.name, info);
    }
    setTwinTheme(options.theme ?? 'dark');
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

    this.rig = new OrbitRig(canvas, {
      targetX: 0,
      targetY: 0,
      distance: distanceForHeight(INITIAL_VIEW_M),
      azimuth: 0,
    });
    this.addLights();
    this.ground = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), this.groundMaterial);
    this.ground.receiveShadow = true;
    this.scene.add(this.ground);
    this.layoutGround();
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
    window.addEventListener('beforeunload', () => this.saveFlows());
    new ResizeObserver(() => this.resize()).observe(canvas);
    this.resize();
    this.renderer.setAnimationLoop(() => this.render());
  }

  /** Adds a site; its cameras, map and zones follow. */
  addSite(spec: SiteSpec): void {
    if (this.sites.has(spec.id)) {
      return;
    }
    const group = new THREE.Group();
    const mapRoot = new THREE.Group();
    const trails = new TrailLayer();
    trails.group.visible = this.showTrails;
    group.add(mapRoot, trails.group);
    const site: SiteLayer = {
      spec,
      group,
      mapRoot,
      trails,
      map: null,
      stylized: null,
      buildings: null,
      ortho: null,
      roadmap: null,
      zones: new Map(),
      zonesLoaded: false,
      center: null,
      cameras: new Map(),
      fusion: new CameraFusion(),
      flow: new FlowField(`jp-flow-${spec.id}`),
      meshes: new Map(),
      drawn: new Map(),
      signals: new Map(),
      transform: spec.transform ?? { yaw_deg: 0, x_m: 0, y_m: 0 },
    };
    this.sites.set(spec.id, site);
    this.scene.add(group);
    this.placeSite(site);
    this.layoutGround();
  }

  /** Adds the map layers of a site. */
  setMap(siteId: string, map: MapData): void {
    const site = this.sites.get(siteId);
    if (!site) {
      return;
    }
    site.map = map;
    site.stylized = buildStylizedLayer(map);
    site.buildings = buildBuildingLayer(map);
    site.buildings.visible = this.buildingsVisible;
    site.ortho = buildImageLayer(map, map.ortho);
    site.roadmap = buildImageLayer(map, map.roadmap);
    site.mapRoot.add(site.stylized, site.buildings);
    for (const image of [site.ortho, site.roadmap]) {
      if (image) {
        site.mapRoot.add(image);
      }
    }
    recolorMap(site.mapRoot, twinPalette());
    this.showLayer(this.layer);
  }

  /** Adds a camera of a site: its marker and, from the first camera, the site's zones. */
  addCamera(siteId: string, cameraId: string, description: SceneDescription): void {
    const site = this.sites.get(siteId);
    if (!site || site.cameras.has(cameraId)) {
      return;
    }
    for (const info of description.classes) {
      this.classes.set(info.name, info);
    }
    if (!site.zonesLoaded) {
      this.addZones(site, description);
      site.zonesLoaded = true;
    }
    site.center = this.mergeCenter(site, description);
    site.cameras.set(cameraId, {
      id: cameraId,
      interpolator: new WorldInterpolator(),
      latest: null,
      arrival: 0,
      marker: this.addCameraMarker(site, description),
    });
  }

  /** The traffic lights of a camera (replacing those it had); unplaced ones are not drawn. */
  setSignals(siteId: string, cameraId: string, defs: SignalDef[]): void {
    const site = this.sites.get(siteId);
    if (!site) {
      return;
    }
    for (const [key, signal] of site.signals) {
      if (signal.camera === cameraId) {
        site.group.remove(signal.root);
        disposeObject(signal.root);
        site.signals.delete(key);
      }
    }
    for (const def of defs) {
      if (!def.position) {
        continue;
      }
      const { root, lamps } = buildSignal(def);
      root.position.set(def.position[0], def.position[1], 0);
      root.rotation.z = THREE.MathUtils.degToRad(def.heading_deg);
      site.group.add(root);
      site.signals.set(`${cameraId}:${def.id}`, { camera: cameraId, def, root, lamps, shown: '' });
    }
  }

  /**
   * Placement of a traffic light: the next click on the ground sets where it stands, the one
   * after that the direction it faces (towards the traffic it controls). null cancels.
   */
  setSignalPlacement(siteId: string | null, cameraId = '', signalId = ''): void {
    if (this.placing?.marker) {
      this.placing.site.group.remove(this.placing.marker);
      disposeObject(this.placing.marker);
    }
    const site = siteId ? this.sites.get(siteId) : undefined;
    this.placing = site
      ? { site, camera: cameraId, id: signalId, position: null, marker: null }
      : null;
    this.rig.interactive = this.alignSite === null && this.placing === null;
    this.canvas.classList.toggle('aligning', this.placing !== null || this.alignSite !== null);
  }

  /** Feeds a new world state of a camera. */
  push(siteId: string, cameraId: string, state: WorldState): void {
    const feed = this.sites.get(siteId)?.cameras.get(cameraId);
    const site = this.sites.get(siteId);
    if (!feed || !site) {
      return;
    }
    if (feed.latest && state.frame < feed.latest.frame) {
      // Seek or loop restart: the camera's road users go at once, without fading.
      site.trails.clear();
      for (const [key, tracked] of site.meshes) {
        if (tracked.cameras.includes(cameraId)) {
          site.group.remove(tracked.root);
          disposeObject(tracked.root);
          site.meshes.delete(key);
        }
      }
    }
    feed.latest = state;
    feed.arrival = performance.now();
    feed.interpolator.push(state, feed.arrival);
  }

  /** Centre of a site in the twin's world frame. */
  siteCenter(siteId: string): { x: number; y: number } | null {
    const site = this.sites.get(siteId);
    if (!site) {
      return null;
    }
    const local = site.center ?? { x: 0, y: 0 };
    const world = new THREE.Vector3(local.x, local.y, 0).applyMatrix4(site.group.matrixWorld);
    return { x: world.x, y: world.y };
  }

  /**
   * Flies the view to a site: a single camera looks up the screen as in its image, an
   * intersection with several cameras is shown north up.
   */
  focus(siteId: string): void {
    const site = this.sites.get(siteId);
    const center = this.siteCenter(siteId);
    if (!site || !center) {
      return;
    }
    let azimuth = 0;
    const feeds = [...site.cameras.values()];
    const only = feeds.length === 1 ? feeds[0] : undefined;
    if (only?.marker) {
      const camera = only.marker.getWorldPosition(new THREE.Vector3());
      azimuth = Math.atan2(center.y - camera.y, center.x - camera.x) - Math.PI / 2;
    }
    this.rig.flyTo({
      targetX: center.x,
      targetY: center.y,
      distance: distanceForHeight(feeds.length > 1 ? INTERSECTION_VIEW_M : INITIAL_VIEW_M),
      azimuth,
    });
  }

  /** Shows all sites at once. */
  overview(): void {
    const centers = [...this.sites.keys()]
      .map((id) => this.siteCenter(id))
      .filter((c): c is { x: number; y: number } => c !== null);
    if (centers.length === 0) {
      return;
    }
    const xs = centers.map((c) => c.x);
    const ys = centers.map((c) => c.y);
    const span = Math.max(Math.max(...xs) - Math.min(...xs), Math.max(...ys) - Math.min(...ys));
    this.rig.flyTo({
      targetX: (Math.max(...xs) + Math.min(...xs)) / 2,
      targetY: (Math.max(...ys) + Math.min(...ys)) / 2,
      distance: distanceForHeight(Math.max(span * 1.4, INTERSECTION_VIEW_M)),
      azimuth: 0,
    });
  }

  /** Places the map layers of a sensor-frame site (scene = R(yaw) · map + t). */
  setMapTransform(siteId: string, transform: MapTransform): void {
    const site = this.sites.get(siteId);
    if (!site) {
      return;
    }
    site.transform = { ...transform };
    site.mapRoot.rotation.z = THREE.MathUtils.degToRad(transform.yaw_deg);
    site.mapRoot.position.set(transform.x_m, transform.y_m, 0);
    if (this.alignSite !== site) {
      this.placeSite(site);
    }
  }

  mapTransform(siteId: string): MapTransform {
    return { ...(this.sites.get(siteId)?.transform ?? { yaw_deg: 0, x_m: 0, y_m: 0 }) };
  }

  /**
   * Align mode for a site: dragging moves its map, right or shift drag rotates it about the point
   * where the drag started; the wheel still zooms the view. null ends the mode.
   */
  setAlignMode(siteId: string | null): void {
    const previous = this.alignSite;
    this.alignSite = siteId ? (this.sites.get(siteId) ?? null) : null;
    this.rig.interactive = this.alignSite === null;
    this.canvas.classList.toggle('aligning', this.alignSite !== null);
    if (previous && previous !== this.alignSite) {
      // The site is placed by its (new) map position again: its road users move, the map stays.
      const before = this.siteCenter(previous.spec.id);
      this.placeSite(previous);
      const after = this.siteCenter(previous.spec.id);
      if (before && after) {
        this.rig.shift(after.x - before.x, after.y - before.y);
      }
    }
  }

  /** Base layer: stylized map (with grid), aerial photo or road map. */
  showLayer(layer: BaseLayer): void {
    this.layer = layer;
    for (const site of this.sites.values()) {
      if (site.stylized) {
        site.stylized.visible = layer === 'map';
      }
      if (site.ortho) {
        site.ortho.visible = layer === 'ortho';
      }
      if (site.roadmap) {
        site.roadmap.visible = layer === 'roadmap';
      }
    }
    for (const grid of this.grids) {
      grid.visible = layer === 'map';
    }
  }

  showBuildings(visible: boolean): void {
    this.buildingsVisible = visible;
    for (const site of this.sites.values()) {
      if (site.buildings) {
        site.buildings.visible = visible;
      }
    }
  }

  setTrails(visible: boolean): void {
    this.showTrails = visible;
    for (const site of this.sites.values()) {
      site.trails.group.visible = visible;
      if (!visible) {
        site.trails.clear();
      }
    }
  }

  /** Draws (true) or pauses drawing while another view is shown; world states are still taken. */
  setActive(active: boolean): void {
    if (active) {
      this.lastFrame = performance.now();
      this.renderer.setAnimationLoop(() => this.render());
    } else {
      this.renderer.setAnimationLoop(null);
      perf.twin.pause();
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
    for (const site of this.sites.values()) {
      for (const tracked of site.meshes.values()) {
        tracked.look = '';
      }
    }
  }

  /** The road users as drawn now, per site. */
  objects(siteId?: string): DrawnObject[] {
    const layers = siteId ? [this.sites.get(siteId)] : [...this.sites.values()];
    return layers.flatMap((site) => (site ? [...site.drawn.values()] : []));
  }

  /** Latest world states of a site's cameras that are not stale. */
  states(siteId?: string): { site: string; camera: string; state: WorldState }[] {
    const now = performance.now();
    const layers = siteId ? [this.sites.get(siteId)] : [...this.sites.values()];
    return layers.flatMap((site) =>
      site
        ? [...site.cameras.values()]
            .filter((f) => f.latest && now - f.arrival < STALE_AFTER_MS)
            .map((f) => ({ site: site.spec.id, camera: f.id, state: f.latest as WorldState }))
        : [],
    );
  }

  /** The site frame of a site in the twin: offset, and for sensor-frame sites the inverse of
   * the map transform (the map is then where its geo anchor says). */
  private placeSite(site: SiteLayer): void {
    const { offset } = site.spec;
    if (site.spec.transform === null) {
      site.group.rotation.z = 0;
      site.group.position.set(offset.x, offset.y, 0);
    } else {
      // world = offset + map, scene = R · map + t  =>  world = offset + R⁻¹ · (scene − t).
      const yaw = THREE.MathUtils.degToRad(site.transform.yaw_deg);
      const cos = Math.cos(-yaw);
      const sin = Math.sin(-yaw);
      const tx = -site.transform.x_m;
      const ty = -site.transform.y_m;
      site.group.rotation.z = -yaw;
      site.group.position.set(offset.x + cos * tx - sin * ty, offset.y + sin * tx + cos * ty, 0);
    }
    site.group.updateMatrixWorld(true);
  }

  /** Ground and grid cover all sites. */
  private layoutGround(): void {
    const centers = [...this.sites.values()].map((s) => s.spec.offset);
    const xs = [0, ...centers.map((c) => c.x)];
    const ys = [0, ...centers.map((c) => c.y)];
    const cx = (Math.max(...xs) + Math.min(...xs)) / 2;
    const cy = (Math.max(...ys) + Math.min(...ys)) / 2;
    const size =
      Math.max(Math.max(...xs) - Math.min(...xs), Math.max(...ys) - Math.min(...ys)) +
      2 * GROUND_MARGIN_M;
    this.ground.scale.set(size, size, 1);
    this.ground.position.set(cx, cy, 0);
    this.scene.remove(...this.grids);
    for (const grid of this.grids) {
      grid.geometry.dispose();
    }
    const focus = this.rig.target;
    this.grids = [
      this.grid(focus, GRID_MINOR_M, GRID_MINOR_EXTENT_M, 0, 0.005),
      this.grid(focus, GRID_MAJOR_M, GRID_MAJOR_EXTENT_M, 1, 0.01),
    ];
    for (const grid of this.grids) {
      grid.visible = this.layer === 'map';
    }
    this.scene.add(...this.grids);
  }

  private mergeCenter(site: SiteLayer, description: SceneDescription): { x: number; y: number } {
    const points = scenePoints(description);
    const zoneCorners = site.zones.size > 0;
    const previous = site.center;
    if (points.length === 0) {
      return previous ?? { x: 0, y: 0 };
    }
    const mean = points.reduce((a, [x, y]) => ({ x: a.x + x, y: a.y + y }), { x: 0, y: 0 });
    const own = { x: mean.x / points.length, y: mean.y / points.length };
    if (!previous) {
      return own;
    }
    // With zones the first camera's centre (that of the zones) stays.
    return zoneCorners ? previous : { x: (previous.x + own.x) / 2, y: (previous.y + own.y) / 2 };
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
    for (const site of this.sites.values()) {
      for (const zone of site.zones.values()) {
        const style = ZONE_STYLE[zone.type] ?? ZONE_STYLE['lane'];
        if (style) {
          zone.fill.color.setHex(p[style.color]);
          zone.edge.color.setHex(p[style.color]);
        }
      }
      recolorMap(site.mapRoot, p);
    }
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

  /** Sun, shadow area, grid and fog follow the view; the sun's direction stays fixed. */
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
    // The grid moves with the view in whole major cells, so it covers any site.
    for (const grid of this.grids) {
      grid.position.set(
        Math.round(x / GRID_MAJOR_M) * GRID_MAJOR_M - (grid.userData['x0'] as number),
        Math.round(y / GRID_MAJOR_M) * GRID_MAJOR_M - (grid.userData['y0'] as number),
        0,
      );
    }
    const fog = this.scene.fog as THREE.Fog;
    fog.near = d * 1.6;
    fog.far = d * 5.5;
  }

  private grid(
    center: { x: number; y: number },
    spacing: number,
    extent: number,
    material: number,
    z: number,
  ): THREE.LineSegments {
    const points: THREE.Vector3[] = [];
    const x0 = Math.round(center.x / GRID_MAJOR_M) * GRID_MAJOR_M;
    const y0 = Math.round(center.y / GRID_MAJOR_M) * GRID_MAJOR_M;
    for (let d = -extent; d <= extent; d += spacing) {
      points.push(new THREE.Vector3(x0 + d, y0 - extent, z));
      points.push(new THREE.Vector3(x0 + d, y0 + extent, z));
      points.push(new THREE.Vector3(x0 - extent, y0 + d, z));
      points.push(new THREE.Vector3(x0 + extent, y0 + d, z));
    }
    const grid = new THREE.LineSegments(
      new THREE.BufferGeometry().setFromPoints(points),
      this.gridMaterials[material],
    );
    grid.userData['x0'] = x0;
    grid.userData['y0'] = y0;
    return grid;
  }

  private addZones(site: SiteLayer, description: SceneDescription): void {
    for (const feature of description.zones.features) {
      const ring = feature.geometry.coordinates[0] ?? [];
      const type = feature.properties.type;
      const style = ZONE_STYLE[type] ?? ZONE_STYLE['lane'];
      if (!style) {
        continue;
      }
      const p = twinPalette();
      const shape = new THREE.Shape(ring.map(([x = 0, y = 0]) => new THREE.Vector2(x, y)));
      const fillMaterial = new THREE.MeshBasicMaterial({
        color: p[style.color],
        transparent: true,
        opacity: style.fill,
        depthWrite: false,
      });
      const fill = new THREE.Mesh(new THREE.ShapeGeometry(shape), fillMaterial);
      fill.position.z = ZONE_Z;
      const edgeMaterial = new THREE.LineBasicMaterial({
        color: p[style.color],
        transparent: true,
        opacity: style.edge,
      });
      const edge = new THREE.LineLoop(
        new THREE.BufferGeometry().setFromPoints(
          ring.map(([x = 0, y = 0]) => new THREE.Vector3(x, y, ZONE_Z + 0.01)),
        ),
        edgeMaterial,
      );
      site.group.add(fill, edge);
      site.zones.set(feature.properties.id, {
        type,
        fill: fillMaterial,
        edge: edgeMaterial,
        base: style.fill,
      });
    }
  }

  /** Pole at the camera position with the camera head and its viewing direction on the ground. */
  private addCameraMarker(site: SiteLayer, description: SceneDescription): THREE.Group | null {
    if (!description.camera_ground_position_m) {
      return null;
    }
    const p = twinPalette();
    const [x, y] = description.camera_ground_position_m;
    // The camera looks at the middle of the zones it shares with the site, else at the centre.
    const own = scenePoints(description).filter(([px, py]) => px !== x || py !== y);
    const target =
      own.length > 0
        ? own.reduce((a, [px, py]) => ({ x: a.x + px / own.length, y: a.y + py / own.length }), {
            x: 0,
            y: 0,
          })
        : (site.center ?? { x: 0, y: 0 });
    const height = description.camera_height_m ?? DEFAULT_CAMERA_HEIGHT_M;
    const direction = Math.atan2(target.y - y, target.x - x);
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
    const reach = THREE.MathUtils.clamp(Math.hypot(target.x - x, target.y - y) * 0.8, 15, 60);
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
    site.group.add(group);
    return group;
  }

  private classInfo(name: string): ClassInfo {
    return this.classes.get(name) ?? this.fallbackClass;
  }

  /** Road axis of the site's map at a point of the site frame. */
  private road(site: SiteLayer, x: number, y: number): RoadAxis | null {
    if (!site.map?.osm) {
      return null;
    }
    // The map lies in its own frame for sensor-frame sites: scene = R · map + t.
    const yaw = THREE.MathUtils.degToRad(site.transform.yaw_deg);
    const dx = x - site.transform.x_m;
    const dy = y - site.transform.y_m;
    const mx = Math.cos(-yaw) * dx - Math.sin(-yaw) * dy;
    const my = Math.sin(-yaw) * dx + Math.cos(-yaw) * dy;
    const axis = roadAxis(site.map.osm, mx, my);
    // Rotation keeps the side of the centre line.
    return axis ? { ...axis, angle: axis.angle + yaw } : null;
  }

  /**
   * Heading a new road user enters with: the learned traffic direction where it appears, else
   * the road: turned to its own motion when it moves, else the direction of its lane. Only
   * without either it takes its own heading or the fallback.
   */
  private entryHeading(
    site: SiteLayer,
    object: WorldObject,
    x: number,
    y: number,
    fallback: number,
  ) {
    const own = moving(object) ? ownHeading(object) : null;
    if (!object.vru) {
      const flow = site.flow.direction(x, y);
      if (flow !== null) {
        return flow;
      }
      const road = this.road(site, x, y);
      if (road) {
        if (own !== null) {
          return road.oneway ? road.angle : alignAxis(road.angle, own);
        }
        const lane = laneDirection(road);
        // A standing vehicle that already has a heading keeps the way of the road it agrees with.
        return lane ?? alignAxis(road.angle, object.heading_rad ?? fallback);
      }
    }
    return own ?? fallback;
  }

  /** Heading the drawn heading turns towards. */
  private targetHeading(
    site: SiteLayer,
    object: WorldObject,
    tracked: TrackedMesh,
    now: number,
  ): number {
    if (!moving(object)) {
      // Standing road users keep the way they face.
      return tracked.drawnHeading;
    }
    const own = ownHeading(object);
    const age = (now - tracked.born) / 1000;
    const flow = object.vru ? null : site.flow.direction(tracked.x, tracked.y);
    // A young track's heading is still settling: where traffic has a clear direction and the
    // track disagrees, the traffic direction wins until the track proves itself.
    if (flow !== null && age < YOUNG_TRACK_S && Math.abs(angleDelta(flow, own)) > Math.PI / 2) {
      return flow;
    }
    return own;
  }

  private ensureMesh(
    site: SiteLayer,
    key: string,
    object: WorldObject,
    look: ObjectLook,
    now: number,
    pose: { x: number; y: number; heading: number },
  ): TrackedMesh {
    const lookKey = `${Number(look.highlighted)}${Number(look.conflict)}${Number(look.validated)}`;
    const existing = site.meshes.get(key);
    if (existing && existing.objectClass === object.class && existing.look === lookKey) {
      return existing;
    }
    const info = this.classInfo(object.class);
    const model = buildClassMesh(info, makeMaterials(object.class, look));
    model.userData['key'] = key;
    model.userData['site'] = site.spec.id;
    if (existing && existing.objectClass === object.class) {
      existing.heading.remove(existing.model);
      disposeObject(existing.model);
      existing.heading.add(model);
      existing.model = model;
      existing.look = lookKey;
      if (existing.fade < 1) {
        setFade(model, existing.fade);
      }
      return existing;
    }
    if (existing) {
      site.group.remove(existing.root);
      disposeObject(existing.root);
    }
    const root = new THREE.Group();
    const headingGroup = new THREE.Group();
    headingGroup.add(buildContactShadow(info), model);
    root.add(headingGroup);
    if (info.vru) {
      root.add(buildVruRing(info));
    }
    site.group.add(root);
    const tracked: TrackedMesh = {
      objectClass: object.class,
      look: lookKey,
      root,
      heading: headingGroup,
      model,
      halo: null,
      selection: null,
      born: existing?.born ?? now,
      lost: null,
      drawnHeading:
        existing?.drawnHeading ?? this.entryHeading(site, object, pose.x, pose.y, pose.heading),
      fade: existing?.fade ?? 0,
      last: object,
      cameras: [],
      x: pose.x,
      y: pose.y,
    };
    setFade(root, tracked.fade);
    site.meshes.set(key, tracked);
    return tracked;
  }

  private updateSite(site: SiteLayer, now: number, dt: number, pulse: number): void {
    // Tracks of all live cameras at their interpolated positions, merged across cameras.
    const tracks: CameraTrack[] = [];
    const live: { camera: string; state: WorldState }[] = [];
    for (const feed of site.cameras.values()) {
      const state = feed.interpolator.current();
      if (!state || now - feed.arrival > STALE_AFTER_MS) {
        continue;
      }
      live.push({ camera: feed.id, state });
      const poses = feed.interpolator.poses(now);
      for (const object of state.objects) {
        const pose = poses.get(object.id);
        if (pose) {
          tracks.push({ camera: feed.id, object, ...pose });
        }
      }
    }
    const fused = site.fusion.fuse(tracks);
    const conflict = new Set<string>();
    const any = new Set<string>();
    const occupied = new Set<string>();
    for (const { camera, state } of live) {
      for (const event of state.events) {
        if (event.state !== 'active') {
          continue;
        }
        if (event.zone !== null) {
          occupied.add(event.zone);
        }
        for (const id of event.object_ids) {
          const key = site.fusion.keyOf(camera, id);
          any.add(key);
          if (event.type === 'VRU_VEHICLE_CONFLICT') {
            conflict.add(key);
          }
        }
      }
    }

    const trailPositions = new Map<string, { x: number; y: number; className: string }>();
    site.drawn.clear();
    for (const track of fused) {
      const { key, object } = track;
      const tracked = this.ensureMesh(
        site,
        key,
        object,
        { highlighted: any.has(key), conflict: conflict.has(key), validated: object.validated },
        now,
        track,
      );
      tracked.lost = null;
      tracked.last = object;
      tracked.cameras = track.cameras;
      tracked.x = track.x;
      tracked.y = track.y;
      tracked.root.position.set(track.x, track.y, 0);
      site.drawn.set(key, { key, site: site.spec.id, cameras: track.cameras, object });

      // Heading: turn towards the target at a limited rate.
      const target = this.targetHeading(site, object, tracked, now);
      const rate = object.vru ? VRU_TURN_RATE : VEHICLE_TURN_RATE;
      const delta = angleDelta(tracked.drawnHeading, target);
      tracked.drawnHeading += THREE.MathUtils.clamp(delta, -rate * dt, rate * dt);
      tracked.heading.rotation.z = tracked.drawnHeading;
      // Keypoint models measure the footprint: the class model is stretched to it.
      const info = this.classInfo(object.class);
      const stretch = (measured: number | null | undefined, nominal: number): number =>
        measured ? THREE.MathUtils.clamp(measured / nominal, 0.5, 2) : 1;
      tracked.model.scale.set(
        stretch(object.length_m, info.length_m),
        stretch(object.width_m, info.width_m),
        1,
      );

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
        site.flow.learn(track.x, track.y, object.vx, object.vy);
      }

      if (conflict.has(key) && !tracked.halo) {
        tracked.halo = buildConflictHalo(this.classInfo(object.class));
        tracked.root.add(tracked.halo);
      } else if (!conflict.has(key) && tracked.halo) {
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
        trailPositions.set(key, { x: track.x, y: track.y, className: object.class });
      }
    }

    // Road users that left keep going along their last velocity while they fade out.
    for (const [key, tracked] of site.meshes) {
      if (site.drawn.has(key)) {
        continue;
      }
      tracked.lost ??= now;
      tracked.x += tracked.last.vx * dt;
      tracked.y += tracked.last.vy * dt;
      tracked.root.position.set(tracked.x, tracked.y, 0);
      tracked.fade = Math.max(0, tracked.fade - dt / FADE_OUT_S);
      setFade(tracked.root, tracked.fade);
      if (tracked.fade <= 0) {
        site.group.remove(tracked.root);
        disposeObject(tracked.root);
        site.meshes.delete(key);
      }
    }
    if (this.showTrails) {
      site.trails.update(trailPositions);
    }

    // Traffic lights show the colour their camera reads; dark while it does not run.
    for (const signal of site.signals.values()) {
      const feed = site.cameras.get(signal.camera);
      const state =
        feed?.latest && now - feed.arrival < STALE_AFTER_MS
          ? (feed.latest.signals?.find((s) => s.id === signal.def.id)?.state ?? 'unknown')
          : 'unknown';
      if (state !== signal.shown) {
        signal.shown = state;
        const lit = LIT[state] ?? [];
        for (const lamp of ['red', 'yellow', 'green'] as Lamp[]) {
          const on = lit.includes(lamp);
          signal.lamps[lamp].material.emissiveIntensity = on ? 3 : 0;
          signal.lamps[lamp].glow.visible = on;
        }
      }
    }

    // Crosswalks with a road user on them glow.
    for (const [id, zone] of site.zones) {
      const active = occupied.has(id);
      zone.fill.opacity = active ? zone.base * 2.2 + 0.08 * Math.sin(now / 160) : zone.base;
      zone.edge.opacity = active ? 1 : Math.min(zone.base * 5.3, 0.85);
    }
  }

  private saveFlows(): void {
    for (const site of this.sites.values()) {
      site.flow.save();
    }
  }

  /** The road user under the pointer gets a ring; meshes are not rebuilt, so nothing flickers. */
  private updateHover(): void {
    let hit: { site: SiteLayer; key: string } | null = null;
    if (this.hoverPointer && !this.rig.dragging && !this.alignSite) {
      this.scene.updateMatrixWorld();
      this.raycaster.setFromCamera(this.hoverPointer.ndc, this.rig.camera);
      const models = [...this.sites.values()].flatMap((s) =>
        [...s.meshes.values()].filter((m) => m.lost === null).map((m) => m.model),
      );
      for (const intersection of this.raycaster.intersectObjects(models, true)) {
        let node: THREE.Object3D | null = intersection.object;
        while (node && node.userData['key'] === undefined) {
          node = node.parent;
        }
        const site = node ? this.sites.get(node.userData['site'] as string) : undefined;
        if (node && site) {
          hit = { site, key: node.userData['key'] as string };
          break;
        }
      }
    }
    const id = hit ? `${hit.site.spec.id}|${hit.key}` : null;
    if (id !== this.hovered) {
      for (const site of this.sites.values()) {
        for (const [key, tracked] of site.meshes) {
          const selected = hit !== null && hit.site === site && hit.key === key;
          if (selected && !tracked.selection) {
            tracked.selection = buildSelectionRing(this.classInfo(tracked.objectClass));
            tracked.root.add(tracked.selection);
          } else if (!selected && tracked.selection) {
            tracked.root.remove(tracked.selection);
            disposeObject(tracked.selection);
            tracked.selection = null;
          }
        }
      }
      this.hovered = id;
    }
    const object = hit ? (hit.site.drawn.get(hit.key) ?? null) : null;
    this.onHover(object, this.hoverPointer?.x ?? 0, this.hoverPointer?.y ?? 0);
    this.canvas.classList.toggle('hovering', object !== null);
  }

  private onSignalClick(e: PointerEvent): boolean {
    const placing = this.placing;
    if (!placing || e.button !== 0) {
      return false;
    }
    const ground = this.rig.groundAt(e.offsetX, e.offsetY);
    if (!ground) {
      return true;
    }
    const local = placing.site.group.worldToLocal(ground.clone());
    if (!placing.position) {
      placing.position = local;
      placing.marker = new THREE.Mesh(
        new THREE.RingGeometry(0.4, 0.7, 24),
        new THREE.MeshBasicMaterial({ color: LAMP_COLORS.red, transparent: true, opacity: 0.9 }),
      );
      placing.marker.position.set(local.x, local.y, ZONE_Z + 0.03);
      placing.site.group.add(placing.marker);
      return true;
    }
    const heading = Math.atan2(local.y - placing.position.y, local.x - placing.position.x);
    const { site, camera, id, position } = placing;
    this.setSignalPlacement(null);
    this.onSignalPlaced(
      site.spec.id,
      camera,
      id,
      [position.x, position.y],
      THREE.MathUtils.radToDeg(heading),
    );
    return true;
  }

  private onAlignStart(e: PointerEvent): void {
    if (this.onSignalClick(e)) {
      return;
    }
    const site = this.alignSite;
    if (!site) {
      return;
    }
    const start = this.rig.groundAt(e.offsetX, e.offsetY);
    if (start) {
      this.aligning = {
        site,
        start: site.group.worldToLocal(start.clone()),
        rotate: e.button === 2 || e.shiftKey,
        transform: { ...site.transform },
      };
      this.canvas.setPointerCapture(e.pointerId);
    }
  }

  private onAlignMove(e: PointerEvent): void {
    if (!this.aligning) {
      return;
    }
    const { site, start, transform } = this.aligning;
    const id = site.spec.id;
    if (this.aligning.rotate) {
      // Rotate about the start point by the horizontal drag.
      const angle = (e.movementX / this.canvas.clientWidth) * Math.PI;
      const cos = Math.cos(angle);
      const sin = Math.sin(angle);
      const ox = site.transform.x_m - start.x;
      const oy = site.transform.y_m - start.y;
      this.setMapTransform(id, {
        yaw_deg: site.transform.yaw_deg + THREE.MathUtils.radToDeg(angle),
        x_m: start.x + ox * cos - oy * sin,
        y_m: start.y + ox * sin + oy * cos,
      });
    } else {
      const ground = this.rig.groundAt(e.offsetX, e.offsetY);
      if (!ground) {
        return;
      }
      const now = site.group.worldToLocal(ground.clone());
      this.setMapTransform(id, {
        yaw_deg: transform.yaw_deg,
        x_m: transform.x_m + now.x - start.x,
        y_m: transform.y_m + now.y - start.y,
      });
    }
    this.onMapTransform(this.mapTransform(id));
  }

  private render(): void {
    const now = performance.now();
    const dt = Math.min((now - this.lastFrame) / 1000, 0.1);
    this.lastFrame = now;
    this.rig.tick(dt);
    this.updateEnvironment();
    const pulse = 0.35 + 0.25 * Math.sin((now / 1000) * 2 * Math.PI * HALO_PULSE_HZ);
    for (const site of this.sites.values()) {
      this.updateSite(site, now, dt, pulse);
    }
    if (now - this.lastFlowSave > FLOW_SAVE_INTERVAL_MS) {
      this.lastFlowSave = now;
      this.saveFlows();
    }
    this.updateHover();
    this.renderer.render(this.scene, this.rig.camera);
    perf.twin.record(now, performance.now() - now);
  }
}
