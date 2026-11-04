import type { MapData } from '../map/mapData';
import type { SceneDescription } from '../net/client';
import type { RoadUser, SiteState } from '../net/siteState';
import { covarianceEllipse, footprintCorners, groundHomography, homography } from './geometry';
import { classCss, mapPalette } from './palette';
import {
  type MapTransform,
  rotatePlacement,
  shiftPlacement,
  type SignalDef,
  siteToView,
  type SiteSpec,
  type Vec,
  viewToSite,
} from './siteFrame';

// A site state older than this at the browser is not drawn: the stream stopped.
const STALE_AFTER_MS = 3000;
// Ground coverage of a camera: image border through the homography, cut at this range.
const COVERAGE_RANGE_M = 160;
const COVERAGE_SAMPLES = 24;
const MIN_SCALE = 0.05;
const MAX_SCALE = 80;
const LABEL_MIN_SCALE = 3.5;
const PICK_RADIUS_PX = 10;
const VELOCITY_HORIZON_S = 1;
// Drawn trail: the reported positions of the last seconds (site time).
const TRAIL_S = 8;
// Size of a road user the backend reports no size for (m): a marker, not a claim.
const UNKNOWN_SIZE_M = 1;

/** Optional layers. */
export interface BevOptions {
  aerial: boolean;
  trails: boolean;
  uncertainty: boolean;
  labels: boolean;
}

/** A road user under the pointer or selected, with its site. */
export interface Picked {
  site: string;
  user: RoadUser;
}

interface CameraLayer {
  description: SceneDescription;
  /** Image-to-ground homography oriented with groundHomography; null if not calibrated. */
  ground: number[][] | null;
  position: Vec | null;
  coverage: Vec[];
  lines: Vec[][];
  signals: SignalDef[];
}

interface Trail {
  run: number;
  points: { t: number; x: number; y: number }[];
}

interface BevSite {
  spec: SiteSpec;
  transform: MapTransform;
  map: MapData | null;
  aerial: HTMLImageElement | null;
  cameras: Map<string, CameraLayer>;
  state: SiteState | null;
  arrival: number;
  trails: Map<number, Trail>;
}

interface Colors {
  bg: string;
  text: string;
  muted: string;
  border: string;
  accent: string;
  violet: string;
  cyan: string;
}

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

/** Whether the heading of a road user was measured now (not held from earlier). */
export function headingMeasured(user: RoadUser): boolean {
  return user.heading_source !== null && user.heading_source !== 'held';
}

/**
 * Bird's-eye view of the sites: a metric, north-up top-down drawing on a 2D canvas of the site
 * states the backend publishes. Every road user is drawn where its state puts it, with the size
 * and heading it reports and their provenance; nothing is fused, smoothed or invented here.
 */
export class BevView {
  private readonly ctx: CanvasRenderingContext2D;
  private readonly sites = new Map<string, BevSite>();
  private colors: Colors = this.readColors();
  private view = { cx: 0, cy: 0, scale: 4 };
  private dirty = true;
  private fresh = '';
  private frameRequest = 0;
  private drag: { x: number; y: number; moved: boolean; cx: number; cy: number } | null = null;
  private selected: { site: string; id: number } | null = null;
  private home: string | null = null;
  // Map alignment of a sensor-frame site, and traffic light placement.
  private aligning: { site: BevSite; rotate: boolean; last: Vec } | null = null;
  private alignSite: string | null = null;
  private placing: { site: string; camera: string; id: string; position: Vec | null } | null = null;
  options: BevOptions = { aerial: false, trails: true, uncertainty: true, labels: false };

  onHover: (picked: Picked | null, x: number, y: number) => void = () => undefined;
  onSelect: (picked: Picked | null) => void = () => undefined;
  onMapTransform: (site: string, transform: MapTransform) => void = () => undefined;
  onSignalPlaced: (
    site: string,
    camera: string,
    id: string,
    position: Vec,
    headingDeg: number,
  ) => void = () => undefined;

  constructor(private readonly canvas: HTMLCanvasElement) {
    const ctx = canvas.getContext('2d');
    if (!ctx) {
      throw new Error('2D canvas not available');
    }
    this.ctx = ctx;
    canvas.addEventListener('pointerdown', (e) => this.onPointerDown(e));
    canvas.addEventListener('pointermove', (e) => this.onPointerMove(e));
    canvas.addEventListener('pointerup', (e) => this.onPointerUp(e));
    canvas.addEventListener('pointerleave', () => this.onHover(null, 0, 0));
    canvas.addEventListener('contextmenu', (e) => {
      if (this.alignSite) {
        e.preventDefault();
      }
    });
    canvas.addEventListener('wheel', (e) => this.onWheel(e), { passive: false });
    canvas.addEventListener('dblclick', () => this.recenter());
    new ResizeObserver(() => this.invalidate()).observe(canvas);
    this.frameRequest = requestAnimationFrame(() => this.loop());
  }

  dispose(): void {
    cancelAnimationFrame(this.frameRequest);
  }

  // --- Data ----------------------------------------------------------------------------------

  addSite(spec: SiteSpec): void {
    if (this.sites.has(spec.id)) {
      return;
    }
    this.sites.set(spec.id, {
      spec,
      transform: spec.transform ?? { yaw_deg: 0, x_m: 0, y_m: 0 },
      map: null,
      aerial: null,
      cameras: new Map(),
      state: null,
      arrival: 0,
      trails: new Map(),
    });
    this.invalidate();
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
    this.invalidate();
  }

  setMapTransform(siteId: string, transform: MapTransform): void {
    const site = this.sites.get(siteId);
    if (site) {
      site.transform = { ...transform };
      this.invalidate();
    }
  }

  mapTransform(siteId: string): MapTransform {
    return { ...(this.sites.get(siteId)?.transform ?? { yaw_deg: 0, x_m: 0, y_m: 0 }) };
  }

  /** A camera of a site: its position, ground coverage and zones (from its scene). */
  addCamera(siteId: string, cameraId: string, description: SceneDescription): void {
    const site = this.sites.get(siteId);
    if (!site || site.cameras.has(cameraId)) {
      return;
    }
    const position = description.camera_ground_position_m;
    const h = description.image_to_ground
      ? groundHomography(description.image_to_ground, description.image_size_px)
      : null;
    site.cameras.set(cameraId, {
      description,
      ground: h,
      position,
      coverage: h ? cameraCoverage(h, description.image_size_px, position) : [],
      lines: [],
      signals: [],
    });
    this.invalidate();
  }

  /** Detection lines of a camera (image pixels), drawn on the ground through its homography. */
  setLines(siteId: string, cameraId: string, lines: { points: Vec[] }[]): void {
    const camera = this.sites.get(siteId)?.cameras.get(cameraId);
    const h = camera?.ground;
    if (!camera || !h) {
      return;
    }
    camera.lines = lines
      .map((line) =>
        line.points.map(([u, v]) => homography(h, u, v)).filter((p): p is Vec => p !== null),
      )
      .filter((points) => points.length >= 2);
    this.invalidate();
  }

  setSignals(siteId: string, cameraId: string, defs: SignalDef[]): void {
    const camera = this.sites.get(siteId)?.cameras.get(cameraId);
    if (camera) {
      camera.signals = defs;
      this.invalidate();
    }
  }

  /** The latest site state from the backend. */
  setState(siteId: string, state: SiteState): void {
    const site = this.sites.get(siteId);
    if (!site) {
      return;
    }
    site.state = state;
    site.arrival = performance.now();
    const t = state.t_ns / 1e9;
    for (const user of state.road_users) {
      let trail = site.trails.get(user.id);
      if (!trail || trail.run !== state.run) {
        trail = { run: state.run, points: [] };
        site.trails.set(user.id, trail);
      }
      const last = trail.points[trail.points.length - 1];
      if (!last || last.t < t) {
        trail.points.push({ t, x: user.x, y: user.y });
      }
      while ((trail.points[0]?.t ?? t) < t - TRAIL_S) {
        trail.points.shift();
      }
    }
    const present = new Set(state.road_users.map((u) => u.id));
    for (const [id, trail] of site.trails) {
      if (!present.has(id) || trail.run !== state.run) {
        site.trails.delete(id);
      }
    }
    this.dirty = true;
  }

  /** The latest site state if it is fresh, else null. */
  state(siteId: string): SiteState | null {
    const site = this.sites.get(siteId);
    if (!site?.state || performance.now() - site.arrival > STALE_AFTER_MS) {
      return null;
    }
    return site.state;
  }

  selectedRoadUser(): Picked | null {
    if (!this.selected) {
      return null;
    }
    const { site, id } = this.selected;
    const user = this.state(site)?.road_users.find((u) => u.id === id);
    return user ? { site, user } : null;
  }

  setOptions(options: Partial<BevOptions>): void {
    this.options = { ...this.options, ...options };
    this.invalidate();
  }

  setTheme(): void {
    this.colors = this.readColors();
    this.invalidate();
  }

  invalidate(): void {
    this.dirty = true;
  }

  // --- Configuration modes -------------------------------------------------------------------
  /** The site in align mode, if any. */
  alignedSite(): string | null {
    return this.alignSite;
  }

  /**
   * Align mode for a sensor-frame site: dragging moves the site over its map, right or shift
   * drag turns it about the pointer; the wheel still zooms. null ends the mode.
   */
  setAlignMode(siteId: string | null): void {
    this.alignSite = siteId;
    this.aligning = null;
    this.canvas.classList.toggle('aligning', siteId !== null || this.placing !== null);
  }

  /**
   * Placement of a traffic light: the next click sets where it stands, the one after that the
   * direction it faces (towards the traffic it controls). null cancels.
   */
  setSignalPlacement(siteId: string | null, cameraId = '', signalId = ''): void {
    this.placing = siteId ? { site: siteId, camera: cameraId, id: signalId, position: null } : null;
    this.canvas.classList.toggle('aligning', this.placing !== null || this.alignSite !== null);
    this.invalidate();
  }

  // --- View ----------------------------------------------------------------------------------

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
    const view = points.map(([x, y]) => this.toView(site, x, y));
    if (view.length === 0) {
      const o = site.spec.offset;
      this.view = { cx: o.x, cy: o.y, scale: this.fitScale(80, 80) };
    } else {
      const xs = view.map((p) => p[0]);
      const ys = view.map((p) => p[1]);
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

  /** Shows all sites at once. */
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

  /** The site the view returns to on a double click (null: all sites). */
  setHome(siteId: string | null): void {
    this.home = siteId;
  }

  recenter(): void {
    if (this.home) {
      this.focus(this.home);
    } else {
      this.overview();
    }
  }

  private fitScale(widthM: number, heightM: number): number {
    const { clientWidth: w, clientHeight: h } = this.canvas;
    if (w === 0 || h === 0) {
      return 4;
    }
    return clamp(Math.min(w / widthM, h / heightM), MIN_SCALE, MAX_SCALE);
  }

  private readColors(): Colors {
    return {
      bg: cssVar('--bg', '#080d17'),
      text: cssVar('--text', '#e7eef8'),
      muted: cssVar('--text-3', '#6b7a92'),
      border: cssVar('--border-strong', '#2a3a57'),
      accent: cssVar('--accent', '#3d9bff'),
      violet: cssVar('--violet', '#a68bfa'),
      cyan: cssVar('--cyan', '#2fd4f0'),
    };
  }

  private toView(site: BevSite, x: number, y: number): Vec {
    return siteToView(site.spec, site.transform, x, y);
  }

  private toScreen(site: BevSite, x: number, y: number): Vec {
    const [vx, vy] = this.toView(site, x, y);
    return [
      this.canvas.clientWidth / 2 + (vx - this.view.cx) * this.view.scale,
      this.canvas.clientHeight / 2 - (vy - this.view.cy) * this.view.scale,
    ];
  }

  private screenToView(px: number, py: number): Vec {
    return [
      this.view.cx + (px - this.canvas.clientWidth / 2) / this.view.scale,
      this.view.cy - (py - this.canvas.clientHeight / 2) / this.view.scale,
    ];
  }

  // --- Interaction ---------------------------------------------------------------------------

  private onPointerDown(e: PointerEvent): void {
    const align = this.alignSite ? this.sites.get(this.alignSite) : undefined;
    if (align && !this.placing) {
      this.aligning = {
        site: align,
        rotate: e.button === 2 || e.shiftKey,
        last: this.screenToView(e.offsetX, e.offsetY),
      };
      this.canvas.setPointerCapture(e.pointerId);
      return;
    }
    this.drag = { x: e.clientX, y: e.clientY, moved: false, cx: this.view.cx, cy: this.view.cy };
    this.canvas.setPointerCapture(e.pointerId);
  }

  private onPointerMove(e: PointerEvent): void {
    if (this.aligning) {
      const { site, rotate, last } = this.aligning;
      const now = this.screenToView(e.offsetX, e.offsetY);
      site.transform = rotate
        ? rotatePlacement(site.spec, site.transform, last, (e.movementX / 200) * (Math.PI / 4))
        : shiftPlacement(site.transform, now[0] - last[0], now[1] - last[1]);
      if (!rotate) {
        this.aligning.last = now;
      }
      this.onMapTransform(site.spec.id, { ...site.transform });
      this.invalidate();
      return;
    }
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
    this.onHover(hit, e.offsetX, e.offsetY);
  }

  private onPointerUp(e: PointerEvent): void {
    if (this.aligning) {
      this.aligning = null;
      return;
    }
    const drag = this.drag;
    this.drag = null;
    if (!drag || drag.moved) {
      return;
    }
    if (this.placing) {
      this.placeSignal(e);
      return;
    }
    const hit = this.pick(e.offsetX, e.offsetY);
    this.selected = hit ? { site: hit.site, id: hit.user.id } : null;
    this.onSelect(hit);
    this.invalidate();
  }

  private placeSignal(e: PointerEvent): void {
    const placing = this.placing;
    const site = placing ? this.sites.get(placing.site) : undefined;
    if (!placing || !site) {
      return;
    }
    const [vx, vy] = this.screenToView(e.offsetX, e.offsetY);
    const point = viewToSite(site.spec, site.transform, vx, vy);
    if (!placing.position) {
      placing.position = point;
      this.invalidate();
      return;
    }
    const [x0, y0] = placing.position;
    const heading = (Math.atan2(point[1] - y0, point[0] - x0) * 180) / Math.PI;
    this.setSignalPlacement(null);
    this.onSignalPlaced(placing.site, placing.camera, placing.id, [x0, y0], heading);
  }

  private onWheel(e: WheelEvent): void {
    e.preventDefault();
    const before = this.screenToView(e.offsetX, e.offsetY);
    this.view.scale = clamp(this.view.scale * Math.exp(-e.deltaY * 0.0015), MIN_SCALE, MAX_SCALE);
    const after = this.screenToView(e.offsetX, e.offsetY);
    this.view.cx += before[0] - after[0];
    this.view.cy += before[1] - after[1];
    this.invalidate();
  }

  /** The road user under a screen point: inside its footprint, or within a few pixels. */
  private pick(px: number, py: number): Picked | null {
    let best: Picked | null = null;
    let distance = PICK_RADIUS_PX;
    for (const site of this.sites.values()) {
      for (const user of this.state(site.spec.id)?.road_users ?? []) {
        const [sx, sy] = this.toScreen(site, user.x, user.y);
        const [length, width] = drawnSize(user);
        const reach = (Math.max(length, width) / 2) * this.view.scale;
        const d = Math.max(Math.hypot(sx - px, sy - py) - reach, 0);
        if (d < distance || (d === 0 && best === null)) {
          distance = d;
          best = { site: site.spec.id, user };
        }
      }
    }
    return best;
  }

  // --- Drawing -------------------------------------------------------------------------------

  private loop(): void {
    // Stale states drop out even without new data.
    const fresh = [...this.sites.keys()].filter((id) => this.state(id) !== null).join('|');
    if (fresh !== this.fresh) {
      this.fresh = fresh;
      this.dirty = true;
    }
    if (this.dirty) {
      this.draw();
      this.dirty = false;
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
    this.ctx.translate(offset.x, offset.y);
    if (site.spec.transform !== null) {
      this.ctx.rotate((-site.transform.yaw_deg * Math.PI) / 180);
      this.ctx.translate(-site.transform.x_m, -site.transform.y_m);
    }
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
    const colors = mapPalette();
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
      ctx.fillStyle = colors.building;
      ctx.fill();
    }
    const osm = map.osm;
    if (osm) {
      for (const [lines, color] of [
        [osm.paths, colors.path],
        [osm.roads, colors.road],
      ] as const) {
        ctx.strokeStyle = color;
        for (const line of lines) {
          ctx.lineWidth = line.width_m;
          this.path(line.points);
          ctx.stroke();
        }
      }
      ctx.strokeStyle = colors.marking;
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

  /** Everything in the site frame: coverage, zones, lines, cameras, signals and road users. */
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
    const first = [...site.cameras.values()][0];
    for (const f of first?.description.zones.features ?? []) {
      const ring = (f.geometry.coordinates[0] ?? []).map(([x = 0, y = 0]): Vec => [x, y]);
      const color =
        f.properties.type === 'crosswalk'
          ? colors.cyan
          : f.properties.type === 'intersection'
            ? colors.violet
            : colors.accent;
      this.path(ring);
      ctx.closePath();
      ctx.fillStyle = color;
      ctx.globalAlpha = 0.06;
      ctx.fill();
      ctx.globalAlpha = 0.6;
      ctx.strokeStyle = color;
      ctx.lineWidth = this.px(1.2);
      ctx.stroke();
      ctx.globalAlpha = 1;
    }
    for (const camera of site.cameras.values()) {
      for (const line of camera.lines) {
        ctx.strokeStyle = colors.muted;
        ctx.lineWidth = this.px(2);
        ctx.setLineDash([this.px(8), this.px(4)]);
        this.path(line);
        ctx.stroke();
        ctx.setLineDash([]);
      }
    }
    this.drawCameras(site);
    this.drawSignals(site);
    const state = this.state(site.spec.id);
    if (!state) {
      return;
    }
    if (this.options.trails) {
      this.drawTrails(site, state);
    }
    for (const user of state.road_users) {
      this.drawRoadUser(site, user);
    }
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

  /** Colour and duration of a traffic light, as the backend reports it. */
  signalState(
    siteId: string,
    camera: string,
    id: string,
  ): { color: string; seconds: number | null } {
    const state = this.state(siteId);
    const signal = state?.signals.find((s) => s.camera === camera && s.id === id);
    if (!state || !signal) {
      return { color: 'unknown', seconds: null };
    }
    return { color: signal.state, seconds: Math.max(0, (state.t_ns - signal.since_ns) / 1e9) };
  }

  private drawSignals(site: BevSite): void {
    const { ctx, colors } = this;
    for (const [cameraId, camera] of site.cameras) {
      for (const def of camera.signals) {
        if (!def.position) {
          continue;
        }
        const [x, y] = def.position;
        const { color } = this.signalState(site.spec.id, cameraId, def.id);
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
    const placing = this.placing;
    if (placing?.site === site.spec.id && placing.position) {
      ctx.strokeStyle = colors.accent;
      ctx.lineWidth = this.px(2);
      ctx.beginPath();
      ctx.arc(placing.position[0], placing.position[1], this.px(9), 0, 2 * Math.PI);
      ctx.stroke();
    }
  }

  private drawTrails(site: BevSite, state: SiteState): void {
    const { ctx } = this;
    ctx.lineWidth = this.px(1.5);
    for (const user of state.road_users) {
      const trail = site.trails.get(user.id);
      if (!trail || trail.points.length < 2) {
        continue;
      }
      ctx.strokeStyle = classCss(user.class);
      ctx.globalAlpha = 0.55;
      this.path(trail.points.map((p): Vec => [p.x, p.y]));
      ctx.stroke();
    }
    ctx.globalAlpha = 1;
  }

  private drawRoadUser(site: BevSite, user: RoadUser): void {
    const { ctx, colors } = this;
    const color = classCss(user.class);
    const coasting = user.status === 'coasting';
    const alpha = (user.validated ? 1 : 0.45) * (coasting ? 0.5 : 1);
    const selected = this.selected?.site === site.spec.id && this.selected.id === user.id;
    const { x, y } = user;
    ctx.globalAlpha = alpha;

    if (this.options.uncertainty) {
      const ellipse = covarianceEllipse(user.position_cov_m2, 2);
      if (ellipse && ellipse.major > 0) {
        ctx.strokeStyle = color;
        ctx.lineWidth = this.px(1);
        ctx.globalAlpha = alpha * 0.7;
        ctx.beginPath();
        ctx.ellipse(
          x,
          y,
          ellipse.major,
          Math.max(ellipse.minor, 0.01),
          ellipse.angle,
          0,
          2 * Math.PI,
        );
        ctx.stroke();
        ctx.globalAlpha = alpha;
      }
    }

    // Velocity over the next second; dashed while the speed is not clearly above its noise.
    if (user.speed_mps > 0.2) {
      const certain = user.speed_std_mps !== null && user.speed_mps > 2 * user.speed_std_mps;
      ctx.strokeStyle = certain ? colors.text : colors.muted;
      ctx.lineWidth = this.px(1.5);
      if (!certain) {
        ctx.setLineDash([this.px(3), this.px(3)]);
      }
      ctx.beginPath();
      ctx.moveTo(x, y);
      ctx.lineTo(x + user.vx * VELOCITY_HORIZON_S, y + user.vy * VELOCITY_HORIZON_S);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    const minimum = this.px(3);
    const [rawLength, rawWidth] = drawnSize(user);
    const length = Math.max(rawLength, minimum * 2);
    const width = Math.max(rawWidth, minimum * 2);
    const outline = (): void => {
      ctx.globalAlpha = alpha;
      ctx.strokeStyle = user.size_source === 'measured' ? colors.text : color;
      ctx.lineWidth = this.px(1.2);
      if (user.size_source !== 'measured' || coasting) {
        ctx.setLineDash([this.px(4), this.px(2)]);
      }
      ctx.stroke();
      ctx.setLineDash([]);
    };
    if (user.heading_rad === null || user.vru) {
      // No heading (or a person): an orientation-free disc of the footprint's size.
      ctx.beginPath();
      ctx.arc(x, y, user.vru ? Math.max(width / 2, minimum) : length / 2, 0, 2 * Math.PI);
      ctx.fillStyle = color;
      ctx.globalAlpha = alpha * (user.vru ? 0.9 : 0.35);
      ctx.fill();
      outline();
      if (user.heading_rad !== null) {
        this.drawFront(user, Math.max(width, 0.9), Math.max(width * 0.6, 0.5));
      }
    } else {
      this.path(footprintCorners(x, y, user.heading_rad, length, width));
      ctx.closePath();
      ctx.fillStyle = color;
      ctx.globalAlpha = alpha * 0.85;
      ctx.fill();
      outline();
      this.drawFront(user, length, width);
    }
    if (selected) {
      ctx.globalAlpha = 1;
      ctx.strokeStyle = colors.accent;
      ctx.lineWidth = this.px(2.5);
      ctx.beginPath();
      ctx.arc(x, y, Math.max(length, width) / 2 + this.px(7), 0, 2 * Math.PI);
      ctx.stroke();
    }
    ctx.globalAlpha = 1;
  }

  /** Front of a road user: filled if measured now, hollow if held from earlier. */
  private drawFront(user: RoadUser, length: number, width: number): void {
    const { ctx, colors } = this;
    const heading = user.heading_rad as number;
    const c = Math.cos(heading);
    const s = Math.sin(heading);
    const tip = length / 2;
    const back = tip - Math.min(length * 0.35, width * 0.9);
    const half = width * 0.35;
    const { x, y } = user;
    ctx.beginPath();
    ctx.moveTo(x + c * tip, y + s * tip);
    ctx.lineTo(x + c * back - s * half, y + s * back + c * half);
    ctx.lineTo(x + c * back + s * half, y + s * back - c * half);
    ctx.closePath();
    ctx.lineWidth = this.px(1.2);
    if (headingMeasured(user)) {
      ctx.fillStyle = colors.text;
      ctx.fill();
    } else {
      ctx.strokeStyle = colors.text;
      ctx.stroke();
    }
  }

  /** Screen-space text: road user labels and signal times. */
  private drawLabels(site: BevSite): void {
    const { colors } = this;
    const scale = this.view.scale;
    this.ctx.font = '11px system-ui, -apple-system, "Segoe UI", sans-serif';
    this.ctx.textBaseline = 'middle';
    for (const [cameraId, camera] of site.cameras) {
      for (const def of camera.signals) {
        if (!def.position) {
          continue;
        }
        const { seconds } = this.signalState(site.spec.id, cameraId, def.id);
        const [x, y] = this.toScreen(site, def.position[0], def.position[1]);
        const time = seconds !== null ? ` ${seconds.toFixed(0)} s` : '';
        this.text(`${def.name}${time}`, x + 10, y - 10, colors.text, 'left', true);
      }
    }
    if (!this.options.labels) {
      return;
    }
    for (const user of this.state(site.spec.id)?.road_users ?? []) {
      const selected = this.selected?.site === site.spec.id && this.selected.id === user.id;
      if (scale < LABEL_MIN_SCALE && !selected) {
        continue;
      }
      const [x, y] = this.toScreen(site, user.x, user.y);
      const r = (Math.max(...drawnSize(user)) / 2) * scale;
      const speed = `${(user.speed_mps * 3.6).toFixed(0)} km/h`;
      this.text(`#${user.id} ${speed}`, x + r + 4, y - r - 2, colors.text, 'left', true);
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
    const [x0, y1] = this.screenToView(0, 0);
    const [x1, y0] = this.screenToView(w, h);
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
    const label = `${metres >= 1 ? metres.toFixed(0) : metres.toFixed(1)} m`;
    this.text(label, x + length / 2, y - 12, colors.text, 'center', true);
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
}

/** Length and width drawn for a road user: its reported size, else a small marker. */
export function drawnSize(user: RoadUser): [number, number] {
  return [user.length_m ?? UNKNOWN_SIZE_M, user.width_m ?? UNKNOWN_SIZE_M];
}

function clamp(value: number, lo: number, hi: number): number {
  return Math.min(Math.max(value, lo), hi);
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
