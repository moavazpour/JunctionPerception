import type { ClassInfo, SceneDescription } from '../net/client';
import type { MapData } from '../map/mapData';
import type { WorldState } from '../net/schema';
import { roadAxis } from '../twin/flowField';
import { classCss, twinPalette } from '../twin/palette';
import type { DrawnObject, MapTransform, SignalDef, SiteSpec } from '../twin/twinView';
import { type BevTrack, BevWorld, type DuplicatePair, duplicateCandidates } from './bevModel';
import { covarianceEllipse, footprintCorners, homography, type Vec } from './geometry';
import { type LaneStatus, LaneAnalytics, type SiteStatus } from './laneAnalytics';
import { perf } from './perf';

// A camera whose last world state is older than this shows no road users (as in the 3D twin).
const STALE_AFTER_MS = 2500;
// Lane analysis at most this often per site (it runs on every world state otherwise).
const ANALYTICS_INTERVAL_MS = 100;
// Tracks of two cameras further apart in time than this are not merged, only flagged.
export const MAX_MERGE_SKEW_S = 0.25;
// Ground coverage of a camera: image border through the homography, cut at this range.
const COVERAGE_RANGE_M = 160;
const COVERAGE_SAMPLES = 24;
const MIN_SCALE = 0.05;
const MAX_SCALE = 80;
const LABEL_MIN_SCALE = 3.5;
const PICK_RADIUS_PX = 10;
const VELOCITY_HORIZON_S = 1;

/** Optional layers and the way tracks of several cameras are shown. */
export interface BevOptions {
  aerial: boolean;
  trails: boolean;
  uncertainty: boolean;
  labels: boolean;
  /** "cameras": every camera's tracks, duplicates linked; "merged": duplicates hidden. */
  tracks: 'cameras' | 'merged';
}

interface CameraLayer {
  id: string;
  description: SceneDescription;
  position: Vec | null;
  coverage: Vec[];
  lines: { name: string; points: Vec[] }[];
  latest: WorldState | null;
  arrival: number;
}

interface BevSite {
  spec: SiteSpec;
  transform: MapTransform;
  map: MapData | null;
  aerial: HTMLImageElement | null;
  analytics: LaneAnalytics | null;
  cameras: Map<string, CameraLayer>;
  signals: Map<string, { camera: string; def: SignalDef }>;
  lastAnalytics: number;
  tracks: BevTrack[];
  shown: BevTrack[];
  duplicates: DuplicatePair[];
}

interface Colors {
  bg: string;
  text: string;
  muted: string;
  border: string;
  accent: string;
  ok: string;
  warn: string;
  danger: string;
  violet: string;
  cyan: string;
  road: string;
  path: string;
  marking: string;
  building: string;
}

const hex = (value: number): string => `#${value.toString(16).padStart(6, '0')}`;

function cssVar(name: string, fallback: string): string {
  const value = getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  return value || fallback;
}

/** Ground coverage of a calibrated camera: its image border projected onto the ground. */
export function cameraCoverage(
  h: number[][],
  size: [number, number],
  camera: Vec | null,
  rangeM = COVERAGE_RANGE_M,
): Vec[] {
  const [w, hgt] = size;
  const border: Vec[] = [];
  const n = COVERAGE_SAMPLES;
  for (let i = 0; i <= n; i++) border.push([(w * i) / n, hgt]);
  for (let i = 0; i <= n; i++) border.push([w, hgt - (hgt * i) / n]);
  for (let i = 0; i <= n; i++) border.push([w - (w * i) / n, 0]);
  for (let i = 0; i <= n; i++) border.push([0, (hgt * i) / n]);
  const ground: Vec[] = [];
  for (const [u, v] of border) {
    const p = homography(h, u, v);
    if (!p) {
      continue;
    }
    if (camera) {
      const dx = p[0] - camera[0];
      const dy = p[1] - camera[1];
      const d = Math.hypot(dx, dy);
      if (d > rangeM) {
        ground.push([camera[0] + (dx / d) * rangeM, camera[1] + (dy / d) * rangeM]);
        continue;
      }
    }
    ground.push(p);
  }
  return ground;
}

/** Keeps the more confident track of each duplicate pair that is close in time. */
export function deduplicate(tracks: BevTrack[], pairs: DuplicatePair[]): BevTrack[] {
  const dropped = new Set<string>();
  for (const { a, b, dtS } of pairs) {
    if (Math.abs(dtS) <= MAX_MERGE_SKEW_S) {
      dropped.add(a.object.confidence >= b.object.confidence ? b.key : a.key);
    }
  }
  return tracks.filter((t) => !dropped.has(t.key));
}

/**
 * Bird's-eye view of the sites: a metric, north-up top-down drawing on a 2D canvas. Road users
 * are drawn where their camera's latest world state puts them, with their measured or class-size
 * footprint and a heading that says where it comes from. Nothing is interpolated or smoothed: a
 * jump on screen is a jump in the data.
 */
export class BevView {
  readonly world = new BevWorld();
  private readonly ctx: CanvasRenderingContext2D;
  private readonly sites = new Map<string, BevSite>();
  private colors: Colors = this.readColors();
  private view = { cx: 0, cy: 0, scale: 4 };
  private dirty = true;
  private active = false;
  private frameRequest = 0;
  private drag: { x: number; y: number; moved: boolean; cx: number; cy: number } | null = null;
  private selected: { site: string; key: string } | null = null;
  private highlightedLane: { site: string; id: string } | null = null;
  options: BevOptions = {
    aerial: false,
    trails: true,
    uncertainty: true,
    labels: true,
    tracks: 'cameras',
  };
  /** Redraw every animation frame (for measurements) instead of only on new data. */
  continuous = false;

  onHover: (object: DrawnObject | null, x: number, y: number) => void = () => undefined;
  onSelect: (track: BevTrack | null) => void = () => undefined;
  /** Called after the lane analysis of a site ran. */
  onStatus: (site: string) => void = () => undefined;

  constructor(
    private readonly canvas: HTMLCanvasElement,
    classes: ClassInfo[] = [],
  ) {
    const ctx = canvas.getContext('2d');
    if (!ctx) {
      throw new Error('2D canvas not available');
    }
    this.ctx = ctx;
    this.world.setClasses(classes);
    this.world.laneHeading = (site, x, y) =>
      this.sites.get(site)?.analytics?.directionAt(x, y) ?? null;
    canvas.addEventListener('pointerdown', (e) => this.onPointerDown(e));
    canvas.addEventListener('pointermove', (e) => this.onPointerMove(e));
    canvas.addEventListener('pointerup', (e) => this.onPointerUp(e));
    canvas.addEventListener('pointerleave', () => {
      this.onHover(null, 0, 0);
    });
    canvas.addEventListener('wheel', (e) => this.onWheel(e), { passive: false });
    canvas.addEventListener('dblclick', () => this.recenter());
    new ResizeObserver(() => this.invalidate()).observe(canvas);
  }

  /** Shows or hides the view; hidden, it draws nothing. */
  setActive(active: boolean): void {
    this.active = active;
    cancelAnimationFrame(this.frameRequest);
    if (active) {
      this.dirty = true;
      this.frameRequest = requestAnimationFrame(() => this.loop());
    } else {
      perf.bev.pause();
    }
  }

  addSite(spec: SiteSpec): void {
    if (this.sites.has(spec.id)) {
      return;
    }
    this.sites.set(spec.id, {
      spec,
      transform: spec.transform ?? { yaw_deg: 0, x_m: 0, y_m: 0 },
      map: null,
      aerial: null,
      analytics: null,
      cameras: new Map(),
      signals: new Map(),
      lastAnalytics: 0,
      tracks: [],
      shown: [],
      duplicates: [],
    });
  }

  setMapTransform(siteId: string, transform: MapTransform): void {
    const site = this.sites.get(siteId);
    if (site) {
      site.transform = { ...transform };
      this.invalidate();
    }
  }

  setMap(siteId: string, map: MapData): void {
    const site = this.sites.get(siteId);
    if (!site) {
      return;
    }
    site.map = map;
    if (map.ortho) {
      const image = new Image();
      image.onload = () => this.invalidate();
      image.src = map.base + map.ortho.image;
      site.aerial = image;
    }
    site.analytics?.setRoadLookup((x, y) => this.road(site, x, y));
    this.invalidate();
  }

  addCamera(siteId: string, cameraId: string, description: SceneDescription): void {
    const site = this.sites.get(siteId);
    if (!site || site.cameras.has(cameraId)) {
      return;
    }
    this.world.setClasses(description.classes);
    if (!site.analytics) {
      site.analytics = new LaneAnalytics(description.zones, `jp-bev-lanes-${siteId}`);
      site.analytics.setRoadLookup((x, y) => this.road(site, x, y));
    }
    const position = description.camera_ground_position_m;
    const h = description.image_to_ground ?? null;
    site.cameras.set(cameraId, {
      id: cameraId,
      description,
      position,
      coverage: h ? cameraCoverage(h, description.image_size_px, position) : [],
      lines: [],
      latest: null,
      arrival: 0,
    });
    this.invalidate();
  }

  /** Counting lines of a camera (image pixels), drawn on the ground through its homography. */
  setLines(siteId: string, cameraId: string, lines: { name: string; points: Vec[] }[]): void {
    const camera = this.sites.get(siteId)?.cameras.get(cameraId);
    const h = camera?.description.image_to_ground;
    if (!camera || !h) {
      return;
    }
    camera.lines = lines
      .map((line) => ({
        name: line.name,
        points: line.points
          .map(([u, v]) => homography(h, u, v))
          .filter((p): p is Vec => p !== null),
      }))
      .filter((line) => line.points.length >= 2);
    this.invalidate();
  }

  setSignals(siteId: string, cameraId: string, defs: SignalDef[]): void {
    const site = this.sites.get(siteId);
    if (!site) {
      return;
    }
    for (const [key, signal] of site.signals) {
      if (signal.camera === cameraId) {
        site.signals.delete(key);
      }
    }
    for (const def of defs) {
      site.signals.set(`${cameraId}:${def.id}`, { camera: cameraId, def });
    }
    this.invalidate();
  }

  /** A new world state of a camera: the same objects the 3D twin receives. */
  push(siteId: string, cameraId: string, state: WorldState): void {
    const site = this.sites.get(siteId);
    const camera = site?.cameras.get(cameraId);
    if (!site || !camera) {
      return;
    }
    camera.latest = state;
    camera.arrival = performance.now();
    this.world.push(siteId, cameraId, state);
    this.refresh(site, false);
    this.dirty = true;
  }

  /** Tracks, duplicates and lane status of a site from its cameras' latest states. */
  private refresh(site: BevSite, force: boolean): void {
    const now = performance.now();
    const live = new Set(
      [...site.cameras.values()]
        .filter((c) => c.latest && now - c.arrival < STALE_AFTER_MS)
        .map((c) => c.id),
    );
    site.tracks = this.world.tracks(site.spec.id, live);
    site.duplicates = duplicateCandidates(site.tracks);
    const unique = deduplicate(site.tracks, site.duplicates);
    site.shown = this.options.tracks === 'merged' ? unique : site.tracks;
    if (site.analytics && (force || now - site.lastAnalytics >= ANALYTICS_INTERVAL_MS)) {
      site.lastAnalytics = now;
      const times = site.tracks.map((t) => t.timestampNs);
      const latest = [...site.cameras.values()].map((c) => c.latest?.timestamp_ns ?? 0);
      site.analytics.update(unique, Math.max(0, ...times, ...latest));
      this.onStatus(site.spec.id);
    }
  }

  /** Latest world state of each live camera of a site, with its age at the browser. */
  cameraStates(siteId: string): { camera: string; state: WorldState; ageMs: number }[] {
    const now = performance.now();
    return [...(this.sites.get(siteId)?.cameras.values() ?? [])]
      .filter((c) => c.latest && now - c.arrival < STALE_AFTER_MS)
      .map((c) => ({ camera: c.id, state: c.latest as WorldState, ageMs: now - c.arrival }));
  }

  status(siteId: string): SiteStatus | null {
    return this.sites.get(siteId)?.analytics?.status() ?? null;
  }

  /** Tracks of a site as shown, and the duplicate pairs among all tracks. */
  snapshot(siteId: string): { tracks: BevTrack[]; shown: BevTrack[]; duplicates: DuplicatePair[] } {
    const site = this.sites.get(siteId);
    return {
      tracks: site?.tracks ?? [],
      shown: site?.shown ?? [],
      duplicates: site?.duplicates ?? [],
    };
  }

  /** Road users as shown, in the form the side panel takes. */
  objects(siteId?: string): DrawnObject[] {
    const sites = siteId ? [this.sites.get(siteId)] : [...this.sites.values()];
    return sites.flatMap((site) =>
      (site?.shown ?? []).map((t) => ({
        key: t.key,
        site: t.site,
        cameras: [t.camera],
        object: t.object,
      })),
    );
  }

  selectedTrack(): BevTrack | null {
    if (!this.selected) {
      return null;
    }
    const { site, key } = this.selected;
    return this.sites.get(site)?.tracks.find((t) => t.key === key) ?? null;
  }

  highlightLane(site: string | null, id: string | null): void {
    this.highlightedLane = site && id ? { site, id } : null;
    this.invalidate();
  }

  setOptions(options: Partial<BevOptions>): void {
    this.options = { ...this.options, ...options };
    for (const site of this.sites.values()) {
      this.refresh(site, true);
    }
    this.invalidate();
  }

  setTheme(): void {
    this.colors = this.readColors();
    this.invalidate();
  }

  invalidate(): void {
    this.dirty = true;
  }

  /** Looks at a site: its zones, cameras and coverage fill the canvas. */
  focus(siteId: string): void {
    const site = this.sites.get(siteId);
    if (!site) {
      return;
    }
    const points: Vec[] = [];
    for (const camera of site.cameras.values()) {
      for (const f of camera.description.zones.features) {
        for (const [x = 0, y = 0] of f.geometry.coordinates[0] ?? []) {
          points.push([x, y]);
        }
      }
      if (camera.position) {
        points.push(camera.position);
      }
    }
    const twin = points.map(([x, y]) => this.toTwin(site, x, y));
    if (twin.length === 0) {
      const o = site.spec.offset;
      this.view = { cx: o.x, cy: o.y, scale: this.fitScale(80, 80) };
    } else {
      const xs = twin.map((p) => p[0]);
      const ys = twin.map((p) => p[1]);
      const w = Math.max(Math.max(...xs) - Math.min(...xs), 30);
      const hgt = Math.max(Math.max(...ys) - Math.min(...ys), 30);
      this.view = {
        cx: (Math.max(...xs) + Math.min(...xs)) / 2,
        cy: (Math.max(...ys) + Math.min(...ys)) / 2,
        scale: this.fitScale(w * 1.15, hgt * 1.15),
      };
    }
    this.invalidate();
  }

  overview(): void {
    const centers = [...this.sites.values()].map((s) => s.spec.offset);
    if (centers.length === 0) {
      return;
    }
    const xs = centers.map((c) => c.x);
    const ys = centers.map((c) => c.y);
    this.view = {
      cx: (Math.max(...xs) + Math.min(...xs)) / 2,
      cy: (Math.max(...ys) + Math.min(...ys)) / 2,
      scale: this.fitScale(
        Math.max(Math.max(...xs) - Math.min(...xs), 140) * 1.4,
        Math.max(Math.max(...ys) - Math.min(...ys), 140) * 1.4,
      ),
    };
    this.invalidate();
  }

  private focusId: string | null = null;

  /** The site the view returns to on a double click (null: all sites). */
  setHome(siteId: string | null): void {
    this.focusId = siteId;
  }

  recenter(): void {
    if (this.focusId) {
      this.focus(this.focusId);
    } else {
      this.overview();
    }
  }

  /** Site nearest to the view centre. */
  centralSite(): string | null {
    let best: string | null = null;
    let distance = Infinity;
    for (const site of this.sites.values()) {
      const d = Math.hypot(site.spec.offset.x - this.view.cx, site.spec.offset.y - this.view.cy);
      if (d < distance) {
        distance = d;
        best = site.spec.id;
      }
    }
    return best;
  }

  /** Metres per CSS pixel of the current view. */
  metresPerPixel(): number {
    return 1 / this.view.scale;
  }

  private fitScale(widthM: number, heightM: number): number {
    const { clientWidth: w, clientHeight: h } = this.canvas;
    if (w === 0 || h === 0) {
      return 4;
    }
    return clamp(Math.min(w / widthM, h / heightM), MIN_SCALE, MAX_SCALE);
  }

  private readColors(): Colors {
    const p = twinPalette();
    return {
      bg: cssVar('--bg', '#080d17'),
      text: cssVar('--text', '#e7eef8'),
      muted: cssVar('--text-3', '#6b7a92'),
      border: cssVar('--border-strong', '#2a3a57'),
      accent: cssVar('--accent', '#3d9bff'),
      ok: cssVar('--ok', '#2fcf8a'),
      warn: cssVar('--warn', '#f6b443'),
      danger: cssVar('--danger', '#ff5d6c'),
      violet: cssVar('--violet', '#a68bfa'),
      cyan: cssVar('--cyan', '#2fd4f0'),
      road: hex(p.road),
      path: hex(p.path),
      marking: hex(p.marking),
      building: hex(p.building),
    };
  }

  /** Site frame -> twin frame (east and north metres of the first site). */
  private toTwin(site: BevSite, x: number, y: number): Vec {
    const { offset } = site.spec;
    if (site.spec.transform === null) {
      return [offset.x + x, offset.y + y];
    }
    // world = offset + R(-yaw) · (scene - t), the inverse of the map placement.
    const yaw = (-site.transform.yaw_deg * Math.PI) / 180;
    const dx = x - site.transform.x_m;
    const dy = y - site.transform.y_m;
    return [
      offset.x + Math.cos(yaw) * dx - Math.sin(yaw) * dy,
      offset.y + Math.sin(yaw) * dx + Math.cos(yaw) * dy,
    ];
  }

  private fromTwin(site: BevSite, x: number, y: number): Vec {
    const { offset } = site.spec;
    if (site.spec.transform === null) {
      return [x - offset.x, y - offset.y];
    }
    const yaw = (site.transform.yaw_deg * Math.PI) / 180;
    const dx = x - offset.x;
    const dy = y - offset.y;
    return [
      Math.cos(yaw) * dx - Math.sin(yaw) * dy + site.transform.x_m,
      Math.sin(yaw) * dx + Math.cos(yaw) * dy + site.transform.y_m,
    ];
  }

  private toScreen(site: BevSite, x: number, y: number): Vec {
    const [tx, ty] = this.toTwin(site, x, y);
    return [
      this.canvas.clientWidth / 2 + (tx - this.view.cx) * this.view.scale,
      this.canvas.clientHeight / 2 - (ty - this.view.cy) * this.view.scale,
    ];
  }

  private screenToTwin(px: number, py: number): Vec {
    return [
      this.view.cx + (px - this.canvas.clientWidth / 2) / this.view.scale,
      this.view.cy - (py - this.canvas.clientHeight / 2) / this.view.scale,
    ];
  }

  /** Road axis of the site's OSM map at a point of the site frame. */
  private road(site: BevSite, x: number, y: number): { angle: number; oneway: boolean } | null {
    if (!site.map?.osm) {
      return null;
    }
    // The map lies in its own frame for sensor-frame sites: scene = R · map + t.
    const yaw = (site.transform.yaw_deg * Math.PI) / 180;
    const dx = x - site.transform.x_m;
    const dy = y - site.transform.y_m;
    const mx = Math.cos(-yaw) * dx - Math.sin(-yaw) * dy;
    const my = Math.sin(-yaw) * dx + Math.cos(-yaw) * dy;
    const axis = roadAxis(site.map.osm, mx, my);
    return axis ? { angle: axis.angle + yaw, oneway: axis.oneway } : null;
  }

  // --- Interaction -------------------------------------------------------------------------

  private onPointerDown(e: PointerEvent): void {
    this.drag = { x: e.clientX, y: e.clientY, moved: false, cx: this.view.cx, cy: this.view.cy };
    this.canvas.setPointerCapture(e.pointerId);
  }

  private onPointerMove(e: PointerEvent): void {
    if (this.drag) {
      const dx = e.clientX - this.drag.x;
      const dy = e.clientY - this.drag.y;
      if (Math.hypot(dx, dy) > 4) {
        this.drag.moved = true;
      }
      if (this.drag.moved) {
        this.view.cx = this.drag.cx - dx / this.view.scale;
        this.view.cy = this.drag.cy + dy / this.view.scale;
        this.invalidate();
        return;
      }
    }
    const hit = this.pick(e.offsetX, e.offsetY);
    this.canvas.classList.toggle('hovering', hit !== null);
    this.onHover(
      hit ? { key: hit.key, site: hit.site, cameras: [hit.camera], object: hit.object } : null,
      e.offsetX,
      e.offsetY,
    );
  }

  private onPointerUp(e: PointerEvent): void {
    const drag = this.drag;
    this.drag = null;
    if (drag && !drag.moved) {
      const hit = this.pick(e.offsetX, e.offsetY);
      this.selected = hit ? { site: hit.site, key: hit.key } : null;
      this.onSelect(hit);
      this.invalidate();
    }
  }

  private onWheel(e: WheelEvent): void {
    e.preventDefault();
    const before = this.screenToTwin(e.offsetX, e.offsetY);
    this.view.scale = clamp(this.view.scale * Math.exp(-e.deltaY * 0.0015), MIN_SCALE, MAX_SCALE);
    const after = this.screenToTwin(e.offsetX, e.offsetY);
    this.view.cx += before[0] - after[0];
    this.view.cy += before[1] - after[1];
    this.invalidate();
  }

  /** The shown track under a screen point: inside its footprint, or within a few pixels. */
  private pick(px: number, py: number): BevTrack | null {
    let best: BevTrack | null = null;
    let distance = PICK_RADIUS_PX;
    for (const site of this.sites.values()) {
      for (const t of site.shown) {
        const [sx, sy] = this.toScreen(site, t.object.x, t.object.y);
        const reach = Math.max((Math.max(t.length, t.width) / 2) * this.view.scale, 0);
        const d = Math.max(Math.hypot(sx - px, sy - py) - reach, 0);
        if (d < distance || (d === 0 && best === null)) {
          distance = d;
          best = t;
        }
      }
    }
    return best;
  }

  // --- Drawing -----------------------------------------------------------------------------

  private loop(): void {
    if (!this.active) {
      return;
    }
    if (this.dirty || this.continuous) {
      const start = performance.now();
      // Stale cameras drop out even without new data.
      for (const site of this.sites.values()) {
        this.refresh(site, false);
      }
      this.draw();
      this.dirty = false;
      perf.bev.record(start, performance.now() - start);
    }
    this.frameRequest = requestAnimationFrame(() => this.loop());
  }

  private draw(): void {
    const { ctx, canvas } = this;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    if (w === 0 || h === 0) {
      return;
    }
    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
      canvas.width = Math.round(w * dpr);
      canvas.height = Math.round(h * dpr);
    }
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.fillStyle = this.colors.bg;
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    const s = this.view.scale;
    const base = (): void =>
      ctx.setTransform(
        dpr * s,
        0,
        0,
        -dpr * s,
        dpr * (w / 2 - this.view.cx * s),
        dpr * (h / 2 + this.view.cy * s),
      );
    for (const site of this.sites.values()) {
      base();
      ctx.translate(site.spec.offset.x, site.spec.offset.y);
      this.drawMap(site);
      base();
      this.siteFrame(site);
      this.drawScene(site);
    }
    base();
    this.drawGrid(w, h);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    for (const site of this.sites.values()) {
      this.drawLabels(site);
    }
    this.drawScale(w, h);
  }

  /** Sets the canvas transform from the site frame (on top of the view transform). */
  private siteFrame(site: BevSite): void {
    const { offset } = site.spec;
    if (site.spec.transform === null) {
      this.ctx.translate(offset.x, offset.y);
      return;
    }
    const yaw = (-site.transform.yaw_deg * Math.PI) / 180;
    this.ctx.translate(offset.x, offset.y);
    this.ctx.rotate(yaw);
    this.ctx.translate(-site.transform.x_m, -site.transform.y_m);
  }

  private px(pixels: number): number {
    return pixels / this.view.scale;
  }

  /** Map layers in their own frame (site offset applied): aerial, roads, crossings, buildings. */
  private drawMap(site: BevSite): void {
    const { ctx } = this;
    const map = site.map;
    if (!map) {
      return;
    }
    if (this.options.aerial && site.aerial?.complete && map.ortho) {
      const [tl, tr, , bl] = map.ortho.corners;
      const iw = site.aerial.naturalWidth;
      const ih = site.aerial.naturalHeight;
      if (iw > 0 && ih > 0) {
        ctx.save();
        ctx.transform(
          (tr[0] - tl[0]) / iw,
          (tr[1] - tl[1]) / iw,
          (bl[0] - tl[0]) / ih,
          (bl[1] - tl[1]) / ih,
          tl[0],
          tl[1],
        );
        ctx.globalAlpha = 0.75;
        ctx.drawImage(site.aerial, 0, 0);
        ctx.restore();
      }
    }
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    const alpha = this.options.aerial ? 0.35 : 1;
    ctx.globalAlpha = alpha;
    for (const building of map.buildings) {
      this.path(building.footprint);
      ctx.closePath();
      ctx.fillStyle = this.colors.building;
      ctx.fill();
    }
    const osm = map.osm;
    if (osm) {
      for (const [lines, color] of [
        [osm.paths, this.colors.path],
        [osm.roads, this.colors.road],
      ] as const) {
        ctx.strokeStyle = color;
        for (const line of lines) {
          ctx.lineWidth = line.width_m;
          this.path(line.points);
          ctx.stroke();
        }
      }
      ctx.strokeStyle = this.colors.marking;
      ctx.globalAlpha = alpha * 0.55;
      ctx.setLineDash([0.5, 0.5]);
      for (const crossing of osm.crossings) {
        if (crossing.kind === 'way' && crossing.points) {
          ctx.lineWidth = crossing.width_m ?? 3;
          this.path(crossing.points);
          ctx.stroke();
        }
      }
      ctx.setLineDash([]);
    }
    ctx.globalAlpha = 1;
  }

  private path(points: Vec[]): void {
    const { ctx } = this;
    ctx.beginPath();
    points.forEach(([x, y], i) => (i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y)));
  }

  /** Everything in the site frame: coverage, zones, lanes, lines, signals and road users. */
  private drawScene(site: BevSite): void {
    const { ctx, colors } = this;
    // Ground coverage of the cameras: no road users outside it does not mean there are none.
    for (const camera of site.cameras.values()) {
      if (camera.coverage.length >= 3) {
        this.path(camera.coverage);
        ctx.closePath();
        ctx.fillStyle = colors.accent;
        ctx.globalAlpha = 0.04;
        ctx.fill();
        ctx.globalAlpha = 0.35;
        ctx.strokeStyle = colors.accent;
        ctx.lineWidth = this.px(1);
        ctx.setLineDash([this.px(6), this.px(4)]);
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.globalAlpha = 1;
      }
    }
    const status = site.analytics?.status();
    const lanes = new Map((status?.lanes ?? []).map((l) => [l.id, l]));
    const areas = new Map((status?.areas ?? []).map((a) => [a.id, a]));
    const first = [...site.cameras.values()][0];
    for (const f of first?.description.zones.features ?? []) {
      const ring = (f.geometry.coordinates[0] ?? []).map(([x = 0, y = 0]): Vec => [x, y]);
      const lane = lanes.get(f.properties.id);
      const area = areas.get(f.properties.id);
      const highlighted =
        this.highlightedLane?.site === site.spec.id && this.highlightedLane.id === f.properties.id;
      let color = colors.accent;
      let fill = 0.06;
      if (f.properties.type === 'crosswalk') {
        color = colors.cyan;
        fill = area && area.vrus > 0 ? 0.3 : 0.12;
      } else if (f.properties.type === 'intersection') {
        color = area?.blocked ? colors.danger : colors.violet;
        fill = area?.blocked ? 0.16 : 0.05;
      } else if (lane) {
        color = laneColor(lane, colors);
        fill = lane.state === 'empty' ? 0.05 : 0.14;
      }
      this.path(ring);
      ctx.closePath();
      ctx.fillStyle = color;
      ctx.globalAlpha = fill;
      ctx.fill();
      ctx.globalAlpha = highlighted ? 1 : 0.7;
      ctx.strokeStyle = color;
      ctx.lineWidth = this.px(highlighted ? 3 : 1.2);
      ctx.stroke();
      ctx.globalAlpha = 1;
      if (lane) {
        this.drawLaneDirection(lane);
      }
    }
    // Counting lines drawn in the console, on the ground.
    for (const camera of site.cameras.values()) {
      for (const line of camera.lines) {
        ctx.strokeStyle = colors.warn;
        ctx.lineWidth = this.px(2);
        ctx.setLineDash([this.px(8), this.px(4)]);
        this.path(line.points);
        ctx.stroke();
        ctx.setLineDash([]);
      }
    }
    // Queues: from the lane end upstream.
    for (const lane of lanes.values()) {
      if (lane.queueLine) {
        ctx.strokeStyle = (lane.queueM ?? 0) > 40 ? colors.danger : colors.warn;
        ctx.lineWidth = Math.max(this.px(5), 0.8);
        ctx.lineCap = 'butt';
        this.path(lane.queueLine);
        ctx.stroke();
        ctx.lineCap = 'round';
      }
    }
    this.drawCameras(site);
    this.drawSignals(site);
    if (this.options.trails) {
      this.drawTrails(site);
    }
    this.drawDuplicates(site);
    const conflict = new Set<string>();
    for (const camera of site.cameras.values()) {
      for (const event of camera.latest?.events ?? []) {
        if (event.state === 'active' && event.type === 'VRU_VEHICLE_CONFLICT') {
          for (const id of event.object_ids) {
            conflict.add(`${camera.id}:${id}`);
          }
        }
      }
    }
    for (const t of site.shown) {
      this.drawTrack(site, t, conflict.has(t.key));
    }
  }

  /** Chevrons along a lane with a known direction (dashed when it comes from OSM). */
  private drawLaneDirection(lane: LaneStatus): void {
    const { ctx, colors } = this;
    const { axis, direction } = lane;
    if (!axis || !direction) {
      return;
    }
    const length = axis.max - axis.min;
    const size = Math.min(axis.width * 0.35, 2);
    const ux = Math.cos(direction.angle);
    const uy = Math.sin(direction.angle);
    ctx.strokeStyle = colors.text;
    ctx.globalAlpha = 0.55;
    ctx.lineWidth = this.px(1.5);
    if (direction.source === 'osm') {
      ctx.setLineDash([this.px(3), this.px(3)]);
    }
    const count = Math.max(1, Math.min(4, Math.floor(length / 15)));
    for (let i = 1; i <= count; i++) {
      const d = axis.min + (length * i) / (count + 1);
      const x = axis.cx + axis.ux * d;
      const y = axis.cy + axis.uy * d;
      ctx.beginPath();
      ctx.moveTo(x - ux * size - uy * size, y - uy * size + ux * size);
      ctx.lineTo(x, y);
      ctx.lineTo(x - ux * size + uy * size, y - uy * size - ux * size);
      ctx.stroke();
    }
    ctx.setLineDash([]);
    ctx.globalAlpha = 1;
  }

  private drawCameras(site: BevSite): void {
    const { ctx, colors } = this;
    for (const camera of site.cameras.values()) {
      if (!camera.position) {
        continue;
      }
      const [x, y] = camera.position;
      ctx.fillStyle = colors.text;
      ctx.beginPath();
      ctx.arc(x, y, this.px(4), 0, 2 * Math.PI);
      ctx.fill();
      ctx.strokeStyle = colors.accent;
      ctx.lineWidth = this.px(2);
      ctx.beginPath();
      ctx.arc(x, y, this.px(7), 0, 2 * Math.PI);
      ctx.stroke();
    }
  }

  private signalState(
    site: BevSite,
    camera: string,
    id: string,
  ): { color: string; seconds: number | null } {
    const state = site.cameras.get(camera)?.latest;
    const signal = state?.signals?.find((s) => s.id === id);
    const fresh =
      state && performance.now() - (site.cameras.get(camera)?.arrival ?? 0) < STALE_AFTER_MS;
    if (!signal || !fresh || !state) {
      return { color: 'unknown', seconds: null };
    }
    return {
      color: signal.state,
      seconds: Math.max(0, (state.timestamp_ns - signal.since_ns) / 1e9),
    };
  }

  private drawSignals(site: BevSite): void {
    const { ctx, colors } = this;
    for (const { camera, def } of site.signals.values()) {
      if (!def.position) {
        continue;
      }
      const [x, y] = def.position;
      const { color } = this.signalState(site, camera, def.id);
      const heading = (def.heading_deg * Math.PI) / 180;
      ctx.strokeStyle = colors.text;
      ctx.lineWidth = this.px(2);
      ctx.beginPath();
      ctx.moveTo(x, y);
      ctx.lineTo(x + Math.cos(heading) * this.px(14), y + Math.sin(heading) * this.px(14));
      ctx.stroke();
      ctx.fillStyle = signalCss(color, colors);
      ctx.beginPath();
      ctx.arc(x, y, Math.max(this.px(6), 0.6), 0, 2 * Math.PI);
      ctx.fill();
      ctx.stroke();
    }
  }

  private drawTrails(site: BevSite): void {
    const { ctx } = this;
    ctx.lineWidth = this.px(1.5);
    for (const t of site.shown) {
      if (t.trail.length < 2) {
        continue;
      }
      ctx.strokeStyle = classCss(t.object.class);
      ctx.globalAlpha = 0.55;
      this.path(t.trail.map((p): Vec => [p.x, p.y]));
      ctx.stroke();
    }
    ctx.globalAlpha = 1;
  }

  private drawDuplicates(site: BevSite): void {
    if (this.options.tracks !== 'cameras') {
      return;
    }
    const { ctx, colors } = this;
    ctx.strokeStyle = colors.violet;
    ctx.lineWidth = this.px(2);
    ctx.setLineDash([this.px(4), this.px(3)]);
    for (const { a, b } of site.duplicates) {
      ctx.beginPath();
      ctx.moveTo(a.object.x, a.object.y);
      ctx.lineTo(b.object.x, b.object.y);
      ctx.stroke();
    }
    ctx.setLineDash([]);
  }

  private drawTrack(site: BevSite, t: BevTrack, conflict: boolean): void {
    const { ctx, colors } = this;
    const o = t.object;
    const color = classCss(o.class);
    const selected = this.selected?.site === site.spec.id && this.selected.key === t.key;
    ctx.globalAlpha = o.validated ? 1 : 0.45;

    if (this.options.uncertainty && o.position_cov_m2) {
      const ellipse = covarianceEllipse(o.position_cov_m2, 2);
      if (ellipse && ellipse.major > 0) {
        ctx.strokeStyle = color;
        ctx.lineWidth = this.px(1);
        ctx.globalAlpha *= 0.7;
        ctx.beginPath();
        ctx.ellipse(
          o.x,
          o.y,
          ellipse.major,
          Math.max(ellipse.minor, 0.01),
          ellipse.angle,
          0,
          2 * Math.PI,
        );
        ctx.stroke();
        ctx.globalAlpha = o.validated ? 1 : 0.45;
      }
    }

    // Velocity over the next second, only from a speed the filter knows.
    if (t.motion !== 'stopped' && o.speed_mps > 0.2) {
      ctx.strokeStyle = t.motion === 'moving' ? colors.text : colors.muted;
      ctx.lineWidth = this.px(1.5);
      if (t.motion === 'uncertain') {
        ctx.setLineDash([this.px(3), this.px(3)]);
      }
      ctx.beginPath();
      ctx.moveTo(o.x, o.y);
      ctx.lineTo(o.x + o.vx * VELOCITY_HORIZON_S, o.y + o.vy * VELOCITY_HORIZON_S);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    const minimum = this.px(3);
    const length = Math.max(t.length, minimum * 2);
    const width = Math.max(t.width, minimum * 2);
    if (t.heading === null) {
      // No heading: an orientation-free disc of the footprint's size, never a guessed box.
      const r = o.vru ? Math.max(width / 2, minimum) : length / 2;
      ctx.beginPath();
      ctx.arc(o.x, o.y, r, 0, 2 * Math.PI);
      ctx.fillStyle = color;
      ctx.globalAlpha *= o.vru ? 0.9 : 0.35;
      ctx.fill();
      ctx.globalAlpha = o.validated ? 1 : 0.45;
      ctx.strokeStyle = color;
      ctx.lineWidth = this.px(1.5);
      if (!o.vru) {
        ctx.setLineDash([this.px(4), this.px(3)]);
      }
      ctx.stroke();
      ctx.setLineDash([]);
    } else {
      const corners =
        o.class === 'pedestrian' ? null : footprintCorners(o.x, o.y, t.heading, length, width);
      if (corners) {
        this.path(corners);
        ctx.closePath();
        ctx.fillStyle = color;
        ctx.globalAlpha *= t.headingStatus === 'lane' ? 0.45 : 0.85;
        ctx.fill();
        ctx.globalAlpha = o.validated ? 1 : 0.45;
        ctx.strokeStyle = t.sizeMeasured ? colors.text : color;
        ctx.lineWidth = this.px(1.2);
        if (!t.sizeMeasured) {
          ctx.setLineDash([this.px(4), this.px(2)]);
        }
        ctx.stroke();
        ctx.setLineDash([]);
        this.drawFront(t, length, width);
      } else {
        ctx.beginPath();
        ctx.arc(o.x, o.y, Math.max(0.35, minimum), 0, 2 * Math.PI);
        ctx.fillStyle = color;
        ctx.fill();
        this.drawFront(t, 0.9, 0.5);
      }
    }
    if (conflict) {
      ctx.strokeStyle = colors.danger;
      ctx.lineWidth = this.px(2.5);
      ctx.beginPath();
      ctx.arc(o.x, o.y, Math.max(length, width) / 2 + this.px(5), 0, 2 * Math.PI);
      ctx.stroke();
    }
    if (t.possibleIdSwitch && t.ageS < 3) {
      ctx.strokeStyle = colors.warn;
      ctx.lineWidth = this.px(1.5);
      ctx.setLineDash([this.px(2), this.px(2)]);
      ctx.beginPath();
      ctx.arc(o.x, o.y, Math.max(length, width) / 2 + this.px(9), 0, 2 * Math.PI);
      ctx.stroke();
      ctx.setLineDash([]);
    }
    if (selected) {
      ctx.strokeStyle = colors.accent;
      ctx.lineWidth = this.px(2.5);
      ctx.beginPath();
      ctx.arc(o.x, o.y, Math.max(length, width) / 2 + this.px(7), 0, 2 * Math.PI);
      ctx.stroke();
    }
    ctx.globalAlpha = 1;
  }

  /** Front of a road user: filled (measured), hollow (held) or dotted (lane direction). */
  private drawFront(t: BevTrack, length: number, width: number): void {
    const { ctx, colors } = this;
    const heading = t.heading as number;
    const c = Math.cos(heading);
    const s = Math.sin(heading);
    const tip = length / 2;
    const back = tip - Math.min(length * 0.35, width * 0.9);
    const half = width * 0.35;
    const { x, y } = t.object;
    ctx.beginPath();
    ctx.moveTo(x + c * tip, y + s * tip);
    ctx.lineTo(x + c * back - s * half, y + s * back + c * half);
    ctx.lineTo(x + c * back + s * half, y + s * back - c * half);
    ctx.closePath();
    ctx.lineWidth = this.px(1.2);
    ctx.strokeStyle = colors.bg;
    if (t.headingStatus === 'measured') {
      ctx.fillStyle = colors.text;
      ctx.fill();
    } else {
      if (t.headingStatus === 'lane') {
        ctx.setLineDash([this.px(2), this.px(2)]);
      }
      ctx.strokeStyle = colors.text;
      ctx.stroke();
      ctx.setLineDash([]);
    }
  }

  /** Screen-space text: road user labels, lane names and signal times. */
  private drawLabels(site: BevSite): void {
    const { ctx, colors } = this;
    const scale = this.view.scale;
    ctx.font = '11px system-ui, -apple-system, "Segoe UI", sans-serif';
    ctx.textBaseline = 'middle';
    const status = site.analytics?.status();
    if (scale >= 1.5) {
      for (const lane of status?.lanes ?? []) {
        if (!lane.axis) {
          continue;
        }
        const [x, y] = this.toScreen(site, lane.axis.cx, lane.axis.cy);
        const text = lane.direction ? lane.id : `${lane.id} · direction ?`;
        this.text(text, x, y, colors.muted, 'center');
        if (lane.queueM && lane.queueLine) {
          const [qx, qy] = this.toScreen(site, ...lane.queueLine[1]);
          this.text(`Q ${lane.queueM.toFixed(0)} m`, qx, qy - 10, colors.warn, 'center', true);
        }
      }
    }
    for (const { camera, def } of site.signals.values()) {
      if (!def.position) {
        continue;
      }
      const { seconds } = this.signalState(site, camera, def.id);
      const [x, y] = this.toScreen(site, def.position[0], def.position[1]);
      this.text(
        `${def.name}${seconds !== null ? ` ${seconds.toFixed(0)} s` : ''}`,
        x + 10,
        y - 10,
        colors.text,
        'left',
        true,
      );
    }
    if (this.options.labels) {
      for (const t of site.shown) {
        const selected = this.selected?.site === site.spec.id && this.selected.key === t.key;
        if (scale < LABEL_MIN_SCALE && !selected) {
          continue;
        }
        const [x, y] = this.toScreen(site, t.object.x, t.object.y);
        const r = (Math.max(t.length, t.width) / 2) * scale;
        const speed =
          t.motion === 'uncertain' ? '? km/h' : `${(t.object.speed_mps * 3.6).toFixed(0)} km/h`;
        const camera = site.cameras.size > 1 ? `${t.camera} ` : '';
        this.text(
          `${camera}#${t.object.id} ${speed}`,
          x + r + 4,
          y - r - 2,
          colors.text,
          'left',
          true,
        );
      }
    }
    if (this.options.tracks === 'cameras' && scale >= LABEL_MIN_SCALE) {
      for (const { a, b, dtS } of site.duplicates) {
        const [x, y] = this.toScreen(
          site,
          (a.object.x + b.object.x) / 2,
          (a.object.y + b.object.y) / 2,
        );
        this.text(`dup Δt ${(dtS * 1000).toFixed(0)} ms`, x, y + 12, colors.violet, 'center', true);
      }
    }
  }

  private text(
    text: string,
    x: number,
    y: number,
    color: string,
    align: CanvasTextAlign,
    halo = false,
  ): void {
    const { ctx } = this;
    ctx.textAlign = align;
    if (halo) {
      ctx.lineWidth = 3;
      ctx.strokeStyle = this.colors.bg;
      ctx.globalAlpha = 0.85;
      ctx.strokeText(text, x, y);
      ctx.globalAlpha = 1;
    }
    ctx.fillStyle = color;
    ctx.fillText(text, x, y);
  }

  private drawGrid(w: number, h: number): void {
    const { ctx, colors } = this;
    const s = this.view.scale;
    if (s < 1) {
      return;
    }
    const step = s >= 6 ? 10 : 50;
    const [x0, y1] = this.screenToTwin(0, 0);
    const [x1, y0] = this.screenToTwin(w, h);
    ctx.strokeStyle = colors.border;
    ctx.globalAlpha = 0.35;
    ctx.lineWidth = this.px(1);
    ctx.beginPath();
    for (let x = Math.floor(x0 / step) * step; x <= x1; x += step) {
      ctx.moveTo(x, y0);
      ctx.lineTo(x, y1);
    }
    for (let y = Math.floor(y0 / step) * step; y <= y1; y += step) {
      ctx.moveTo(x0, y);
      ctx.lineTo(x1, y);
    }
    ctx.stroke();
    ctx.globalAlpha = 1;
  }

  /** Scale bar and north arrow (the view is always north up). */
  private drawScale(w: number, h: number): void {
    const { ctx, colors } = this;
    const target = 120 / this.view.scale;
    const magnitude = 10 ** Math.floor(Math.log10(target));
    const metres =
      [1, 2, 5, 10]
        .map((k) => k * magnitude)
        .filter((m) => m <= target)
        .pop() ?? magnitude;
    const length = metres * this.view.scale;
    const x = w - 24 - length;
    const y = h - 84;
    ctx.strokeStyle = colors.text;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(x, y - 5);
    ctx.lineTo(x, y);
    ctx.lineTo(x + length, y);
    ctx.lineTo(x + length, y - 5);
    ctx.stroke();
    this.text(
      `${metres >= 1 ? metres.toFixed(0) : metres.toFixed(1)} m`,
      x + length / 2,
      y - 12,
      colors.text,
      'center',
      true,
    );
    const nx = w - 30;
    const ny = y - 52;
    ctx.fillStyle = colors.text;
    ctx.beginPath();
    ctx.moveTo(nx, ny - 12);
    ctx.lineTo(nx - 6, ny + 6);
    ctx.lineTo(nx, ny + 2);
    ctx.lineTo(nx + 6, ny + 6);
    ctx.closePath();
    ctx.fill();
    this.text('N', nx, ny - 22, colors.text, 'center', true);
  }

  /** Site frame position of a point of the twin frame, for tests and tools. */
  siteAt(siteId: string, x: number, y: number): Vec | null {
    const site = this.sites.get(siteId);
    return site ? this.fromTwin(site, x, y) : null;
  }
}

function clamp(value: number, lo: number, hi: number): number {
  return Math.min(Math.max(value, lo), hi);
}

function laneColor(lane: LaneStatus, colors: Colors): string {
  switch (lane.state) {
    case 'queue':
      return (lane.queueM ?? 0) > 40 ? colors.danger : colors.warn;
    case 'flowing':
      return colors.ok;
    case 'unknown':
      return colors.muted;
    default:
      return colors.accent;
  }
}

export function signalCss(state: string, colors: { muted: string }): string {
  switch (state) {
    case 'red':
      return '#ff3b30';
    case 'yellow':
      return '#ffb000';
    case 'red_yellow':
      return '#ff7a18';
    case 'green':
      return '#2bd96b';
    default:
      return colors.muted;
  }
}
