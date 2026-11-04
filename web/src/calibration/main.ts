import './calibration.css';

import {
  attributionLines,
  crossingSuggestions,
  loadMapData,
  type MapData,
  type Point,
} from '../map/mapData';
import {
  applyHomography,
  type ControlPoint,
  fetchControlPoints,
  fitControlPoints,
  type FitResult,
  fetchRegions,
  invert3,
  type Regions,
  requestRestart,
  saveCalibration,
  saveRegions,
  saveZones,
} from '../net/calibrationApi';
import {
  cameraBase,
  fetchScene,
  fetchStatus,
  type SceneDescription,
  sendPlayback,
  setSite,
  siteUrl,
  subscribeVideo,
} from '../net/client';
import { APPROACH_LABELS, cameraAction, findCamera, getSite, sitePlayback } from '../net/manager';
import { h, param } from '../ui/dom';
import { toast } from '../ui/feedback';
import { icon } from '../ui/icons';
import { links, mountShell } from '../ui/shell';
import { waitForCamera } from '../ui/siteGate';
import { PanZoomCanvas, type View2D } from './panZoom';

type Mode = 'points' | 'zones' | 'regions' | 'camera';

const ZONE_TYPES = ['crosswalk', 'lane'] as const;
type ZoneType = (typeof ZONE_TYPES)[number];

interface EditableZone {
  id: string;
  type: ZoneType;
  polygon: Point[];
}

const COLORS = {
  pending: '#f6b443',
  fit: '#3d9bff',
  check: '#2fcf8a',
  outlier: '#ff5d6c',
  zone: 'rgba(47, 212, 240, 0.2)',
  zoneEdge: '#2fd4f0',
  drawing: '#f6b443',
  camera: '#ffffff',
  road: 'rgba(255, 255, 255, 0.55)',
  label: 'rgba(8, 13, 23, 0.8)',
  area: 'rgba(61, 155, 255, 0.16)',
  areaEdge: '#3d9bff',
  line: '#f6b443',
  signal: '#ff5d6c',
};
const RESTART_RELOAD_MS = 6000;
// Servers of the site's other running cameras (they share the zones).
let siblingCameras: string[] = [];
const MARKER_RADIUS = 6;

function element<T extends HTMLElement>(selector: string): T {
  const found = document.querySelector<T>(selector);
  if (!found) {
    throw new Error(`missing element ${selector}`);
  }
  return found;
}

/**
 * Image pixels -> world through the fitted homography, and back. Ground beyond the
 * camera's horizon has no image; the homography would mirror it onto the far side.
 */
class Projection {
  constructor(
    readonly imageToGround: number[][],
    private readonly groundToImage: number[][] | null,
    /** Sign of the ground-to-image w on the side the camera sees. */
    private readonly side: number,
    /** Smallest signed w kept, a margin short of the horizon. */
    private readonly minDepth: number,
  ) {}

  /** `groundPixel` is any image pixel known to show the ground, e.g. a control point. */
  static from(fit: FitResult | null, groundPixel: Point): Projection | null {
    if (!fit) {
      return null;
    }
    // H_gi = H_ig^-1, so w_gi at the ground point of pixel q is 1 / w_ig(q).
    const [g = 0, k = 0, l = 1] = fit.image_to_ground[2] ?? [];
    const w = g * groundPixel[0] + k * groundPixel[1] + l;
    const reference = Math.abs(w) > 1e-12 ? 1 / Math.abs(w) : 1;
    return new Projection(
      fit.image_to_ground,
      invert3(fit.image_to_ground),
      Math.sign(w) || 1,
      0.01 * reference,
    );
  }

  /** Signed distance from the horizon, positive where the camera sees the ground. */
  private depth(x: number, y: number): number {
    const [g = 0, k = 0, l = 1] = this.groundToImage?.[2] ?? [];
    return this.side * (g * x + k * y + l) - this.minDepth;
  }

  toImage(x: number, y: number): [number, number] | null {
    if (!this.groundToImage || this.depth(x, y) <= 0) {
      return null;
    }
    return applyHomography(this.groundToImage, x, y);
  }

  /** The part of a ground polygon in front of the horizon, in image pixels. */
  polygonToImage(polygon: Point[]): [number, number][] {
    const clipped: Point[] = [];
    polygon.forEach((a, i) => {
      const b = polygon[(i + 1) % polygon.length] as Point;
      const da = this.depth(a[0], a[1]);
      const db = this.depth(b[0], b[1]);
      if (da > 0) {
        clipped.push(a);
      }
      if (da > 0 !== db > 0) {
        const t = da / (da - db);
        clipped.push([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])]);
      }
    });
    return clipped
      .map(([x, y]) => (this.groundToImage ? applyHomography(this.groundToImage, x, y) : null))
      .filter((p): p is [number, number] => p !== null);
  }
}

class CalibrationApp {
  private mode: Mode = 'points';
  private points: ControlPoint[] = [];
  private pendingPixel: [number, number] | null = null;
  private fit: FitResult | null = null;
  private projection: Projection | null = null;
  private cameraOverride: [number, number] | null = null;
  private cameraHeight: number | null = null;
  private zones: EditableZone[] = [];
  private drawing: Point[] = [];
  // Detection areas and counting lines of the camera, and the one being drawn (image px).
  private regions: Regions = { areas: [], lines: [], signals: [] };
  private drawingRegion: [number, number][] = [];
  private image = new Image();
  private orthoImage = new Image();
  private readonly imageCanvas: PanZoomCanvas;
  private readonly mapCanvas: PanZoomCanvas;
  private nextPoint = 1;
  private readonly calibrated: boolean;

  constructor(
    scene: SceneDescription,
    private readonly map: MapData,
  ) {
    this.calibrated = scene.calibrated;
    this.imageCanvas = new PanZoomCanvas(
      element('#image'),
      false,
      (c, v) => this.drawImage(c, v),
      (x, y) => this.onImageClick(x, y),
    );
    this.mapCanvas = new PanZoomCanvas(
      element('#map'),
      true,
      (c, v) => this.drawMap(c, v),
      (x, y) => this.onMapClick(x, y),
    );
    this.zones = scene.zones.features.map((f) => ({
      id: f.properties.id,
      type: (ZONE_TYPES as readonly string[]).includes(f.properties.type)
        ? (f.properties.type as ZoneType)
        : 'crosswalk',
      polygon: (f.geometry.coordinates[0] ?? []).slice(0, -1).map(([x = 0, y = 0]) => [x, y]),
    }));
    // A camera pose set by hand earlier stays until it is changed.
    if (scene.camera_ground_position_m && scene.camera_height_m && scene.camera_height_m > 0) {
      this.cameraOverride = scene.camera_ground_position_m;
      this.cameraHeight = scene.camera_height_m;
    }
    this.bindControls();
    this.loadImages();
  }

  async start(): Promise<void> {
    this.points = await fetchControlPoints();
    this.regions = await fetchRegions().catch(() => ({ areas: [], lines: [], signals: [] }));
    this.nextPoint = this.points.length + 1;
    await this.refit();
    // Without map layers there is nothing to click control points on.
    this.setMode(this.map.ortho || this.map.osm ? 'points' : 'regions');
  }

  private bindControls(): void {
    for (const button of document.querySelectorAll<HTMLButtonElement>('.tabs button')) {
      button.addEventListener('click', () => this.setMode(button.dataset['mode'] as Mode));
    }
    element('#refresh-frame').addEventListener('click', () => this.loadFrame());
    element('#save').addEventListener('click', () => void this.save());
    element('#apply').addEventListener('click', () => void this.apply());
    element('#suggest').addEventListener('click', () => this.suggestCrossings());
    element('#finish-zone').addEventListener('click', () => this.finishZone());
    element('#finish-area').addEventListener('click', () => this.finishRegion('area'));
    element('#finish-line').addEventListener('click', () => this.finishRegion('line'));
    element('#finish-signal').addEventListener('click', () => this.finishRegion('signal'));
    const height = element<HTMLInputElement>('#camera-height input');
    if (this.cameraHeight !== null) {
      height.value = String(this.cameraHeight);
    }
    height.addEventListener('change', () => {
      const value = Number.parseFloat(height.value);
      this.cameraHeight = Number.isFinite(value) && value > 0 ? value : null;
    });
    window.addEventListener('keydown', (e) => {
      if (e.key === 'Enter' && this.mode === 'zones') {
        this.finishZone();
      } else if (e.key === 'Escape') {
        this.pendingPixel = null;
        this.drawing = [];
        this.drawingRegion = [];
        this.redraw();
      }
    });
    element('#attribution').replaceChildren(
      ...attributionLines(this.map).map(({ text, url }) => {
        const a = document.createElement('a');
        a.href = url;
        a.target = '_blank';
        a.rel = 'noopener';
        a.textContent = text;
        return a;
      }),
    );
  }

  private loadImages(): void {
    this.loadFrame();
    if (this.map.ortho) {
      this.orthoImage.onload = () => this.fitMapView();
      this.orthoImage.src = this.map.base + this.map.ortho.image;
    }
  }

  /** Loads the latest camera frame of the site. */
  loadFrame(): void {
    this.image.onload = () => {
      const canvas = this.imageCanvas.canvas;
      this.imageCanvas.view.fit(
        0,
        0,
        this.image.width,
        this.image.height,
        canvas.clientWidth,
        canvas.clientHeight,
      );
      this.imageCanvas.redraw();
    };
    this.image.onerror = () => this.message('Waiting for the first camera frame…');
    this.image.src = siteUrl(`/api/frame?t=${Date.now()}`);
  }

  private fitMapView(): void {
    const canvas = this.mapCanvas.canvas;
    const r = this.map.manifest.radius_m;
    this.mapCanvas.view.fit(-r, -r, r, r, canvas.clientWidth, canvas.clientHeight);
    this.mapCanvas.redraw();
  }

  private setMode(mode: Mode): void {
    this.mode = mode;
    for (const button of document.querySelectorAll<HTMLButtonElement>('.tabs button')) {
      button.classList.toggle('active', button.dataset['mode'] === mode);
    }
    element('#zone-actions').hidden = mode !== 'zones';
    element('#region-actions').hidden = mode !== 'regions';
    element('#camera-height').hidden = mode !== 'camera';
    const hints: Record<Mode, string> = {
      points:
        'Click a point in the camera image, then the same point on the map (road markings, kerb corners). Use at least 8 points spread over the scene; every fourth point is a check point. Wheel: zoom, drag: pan.',
      zones:
        'Click the corners of a zone on the map, then "Finish zone" (or Enter). Zones (lanes, crosswalks) are drawn in the twin.',
      regions:
        'Click points in the camera image, then "Finish ROI" (3+ points), "Finish line" (2+ points) or "Finish signal" (two corners around a traffic light head). With an ROI (region of interest), only road users inside it are detected and shown. Lines with a band keep only road users that close to the line. Signals are read for their colour and shown in the twin once placed there. Save applies them at once.',
      camera:
        'The camera pose is estimated from the control points. Where they cannot constrain it (long lenses, points along one road), click the camera position on the map and enter its height above the road.',
    };
    element('#hint').textContent = hints[mode];
    this.renderList();
    this.redraw();
  }

  private onImageClick(u: number, v: number): void {
    if (this.mode === 'regions') {
      this.drawingRegion.push([u, v]);
      this.redraw();
      return;
    }
    if (this.mode !== 'points') {
      return;
    }
    this.pendingPixel = [u, v];
    this.redraw();
  }

  private finishRegion(kind: 'area' | 'line' | 'signal'): void {
    const points = this.drawingRegion;
    if (points.length < (kind === 'area' ? 3 : 2)) {
      this.message(
        kind === 'area'
          ? 'An ROI needs at least three points.'
          : kind === 'line'
            ? 'A line needs two points.'
            : 'Click two opposite corners of the signal head (or its outline).',
      );
      return;
    }
    const taken = new Set(
      [...this.regions.areas, ...this.regions.lines, ...this.regions.signals].map((r) => r.id),
    );
    let n = 1;
    while (taken.has(`${kind}_${n}`)) {
      n++;
    }
    const id = `${kind}_${n}`;
    if (kind === 'area') {
      this.regions.areas.push({ id, name: `ROI ${n}`, points });
    } else if (kind === 'line') {
      this.regions.lines.push({ id, name: `Line ${n}`, points, band_m: 0, speed_limit_kmh: 50 });
    } else {
      this.regions.signals.push({
        id,
        name: `Signal ${n}`,
        points,
        position: null,
        heading_deg: 0,
        height_m: 4.5,
      });
      this.message('Signal added: place it in the live twin ("Traffic lights") to show it there.');
    }
    this.drawingRegion = [];
    this.renderList();
    this.redraw();
  }

  private regionItem(kind: 'area' | 'line' | 'signal', index: number): HTMLLIElement {
    const item = document.createElement('li');
    item.className = 'region-item';
    const list: { id: string; name: string }[] =
      kind === 'area'
        ? this.regions.areas
        : kind === 'line'
          ? this.regions.lines
          : this.regions.signals;
    const region = list[index];
    if (!region) {
      return item;
    }
    const tag = document.createElement('span');
    tag.className = `region-tag ${kind}`;
    tag.textContent = { area: 'ROI', line: 'Line', signal: 'Signal' }[kind];
    if (kind === 'signal') {
      const signal = region as Regions['signals'][number];
      tag.title = signal.position
        ? 'Placed in the twin'
        : 'Not placed yet: place it in the live twin (Traffic lights)';
      if (!signal.position) {
        tag.textContent = 'Signal · not placed';
      }
    }
    const name = document.createElement('input');
    name.value = region.name;
    name.title = region.id;
    name.addEventListener('change', () => (region.name = name.value.trim() || region.id));
    const remove = document.createElement('button');
    remove.type = 'button';
    remove.textContent = 'Remove';
    remove.addEventListener('click', () => {
      list.splice(index, 1);
      this.renderList();
      this.redraw();
    });
    item.append(tag, name);
    if (kind === 'line') {
      const line = region as Regions['lines'][number];
      const number = (label: string, value: number, title: string, set: (v: number) => void) => {
        const wrap = document.createElement('label');
        wrap.title = title;
        const input = document.createElement('input');
        input.type = 'number';
        input.min = '0';
        input.step = '0.5';
        input.value = String(value);
        input.addEventListener('change', () =>
          set(Math.max(0, Number.parseFloat(input.value) || 0)),
        );
        wrap.append(label, input);
        return wrap;
      };
      item.append(
        number(
          'band m',
          line.band_m,
          'Only road users this close to the line are detected (0: the line only counts)',
          (v) => {
            line.band_m = v;
          },
        ),
        number('limit km/h', line.speed_limit_kmh, 'Speed limit for the statistics', (v) => {
          line.speed_limit_kmh = v;
        }),
      );
    }
    item.append(remove);
    return item;
  }

  /** Arrow at the middle of a line towards the side a direction-A crossing goes to. */
  private directionArrow(line: [number, number][]): [[number, number], [number, number]] | null {
    const projection = this.projection;
    const [a, b] = line;
    if (!projection || !a || !b) {
      return null;
    }
    const mid: [number, number] = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
    const length = Math.hypot(b[0] - a[0], b[1] - a[1]) || 1;
    const normal: [number, number] = [-(b[1] - a[1]) / length, (b[0] - a[0]) / length];
    const toGround = (u: number, v: number) => applyHomography(projection.imageToGround, u, v);
    const ga = toGround(...a);
    const gb = toGround(...b);
    const probe = toGround(mid[0] + normal[0] * 10, mid[1] + normal[1] * 10);
    if (!ga || !gb || !probe) {
      return null;
    }
    // Direction A crosses from the left of the line (as drawn, on the ground) to its right.
    const left = (gb[0] - ga[0]) * (probe[1] - ga[1]) - (gb[1] - ga[1]) * (probe[0] - ga[0]) > 0;
    const sign = left ? -1 : 1;
    const reach = Math.min(60, length * 0.25);
    return [mid, [mid[0] + normal[0] * reach * sign, mid[1] + normal[1] * reach * sign]];
  }

  private drawRegions(c: CanvasRenderingContext2D, view: View2D): void {
    for (const signal of this.regions.signals) {
      const [a, b] = signal.points;
      const corners: [number, number][] =
        signal.points.length === 2 && a && b ? [a, [b[0], a[1]], b, [a[0], b[1]]] : signal.points;
      const screen = corners.map(([u, v]) => view.toScreen(u, v));
      this.polygon(c, screen, null, COLORS.signal);
      const [first] = screen;
      if (first) {
        this.marker(c, first[0], first[1], COLORS.signal, signal.name);
      }
    }
    for (const area of this.regions.areas) {
      this.polygon(
        c,
        area.points.map(([u, v]) => view.toScreen(u, v)),
        COLORS.area,
        COLORS.areaEdge,
      );
    }
    for (const line of this.regions.lines) {
      const screen = line.points.map(([u, v]) => view.toScreen(u, v));
      c.beginPath();
      screen.forEach(([x, y], i) => (i === 0 ? c.moveTo(x, y) : c.lineTo(x, y)));
      c.strokeStyle = COLORS.line;
      c.lineWidth = 3;
      c.stroke();
      const arrow = this.directionArrow(line.points);
      if (arrow) {
        const [x0, y0] = view.toScreen(...arrow[0]);
        const [x1, y1] = view.toScreen(...arrow[1]);
        c.beginPath();
        c.moveTo(x0, y0);
        c.lineTo(x1, y1);
        c.lineWidth = 2;
        c.stroke();
        this.marker(c, x1, y1, COLORS.line, 'A');
      }
      const [first] = screen;
      if (first) {
        this.marker(c, first[0], first[1], COLORS.line, line.name);
      }
    }
    if (this.drawingRegion.length > 0) {
      c.setLineDash([4, 4]);
      c.beginPath();
      this.drawingRegion.forEach(([u, v], i) => {
        const [x, y] = view.toScreen(u, v);
        if (i === 0) {
          c.moveTo(x, y);
        } else {
          c.lineTo(x, y);
        }
      });
      c.strokeStyle = COLORS.drawing;
      c.lineWidth = 2;
      c.stroke();
      c.setLineDash([]);
      for (const [u, v] of this.drawingRegion) {
        const [x, y] = view.toScreen(u, v);
        c.beginPath();
        c.arc(x, y, 3, 0, 2 * Math.PI);
        c.fillStyle = COLORS.drawing;
        c.fill();
      }
    }
  }

  private onMapClick(x: number, y: number): void {
    if (this.mode === 'zones') {
      this.drawing.push([x, y]);
    } else if (this.mode === 'camera') {
      this.cameraOverride = [x, y];
      this.message('Camera position set by hand.');
    } else if (this.pendingPixel) {
      const [u, v] = this.pendingPixel;
      const id = `p${this.nextPoint++}`;
      this.points.push({ id, u, v, x, y, check: this.points.length % 4 === 3 });
      this.pendingPixel = null;
      void this.refit();
    } else {
      this.message('Click the point in the camera image first.');
    }
    this.renderList();
    this.redraw();
  }

  private async refit(): Promise<void> {
    if (this.points.filter((p) => !p.check).length < 4) {
      this.fit = null;
      this.projection = null;
      this.renderSummary();
      return;
    }
    try {
      this.fit = await fitControlPoints(this.points);
      const anchor = this.points.find((p) => !p.check);
      this.projection = Projection.from(this.fit, anchor ? [anchor.u, anchor.v] : [0, 0]);
      this.message('');
    } catch (error) {
      this.fit = null;
      this.projection = null;
      this.message((error as Error).message);
    }
    this.renderSummary();
    this.renderList();
    this.redraw();
  }

  private residual(id: string) {
    return this.fit?.residuals.find((r) => r.id === id);
  }

  private pointColor(point: ControlPoint): string {
    const residual = this.residual(point.id);
    if (residual && !residual.check && !residual.inlier) {
      return COLORS.outlier;
    }
    return point.check ? COLORS.check : COLORS.fit;
  }

  private renderSummary(): void {
    const summary = element('#summary');
    const fit = this.fit;
    if (!fit) {
      summary.textContent =
        this.points.length === 0 && this.calibrated
          ? 'Calibrated without control points (e.g. from the dataset); new points would replace it'
          : `${this.points.length} points · need at least 4 fit points`;
      return;
    }
    const check =
      fit.check_mean_m !== null && fit.check_max_m !== null
        ? `check mean ${fit.check_mean_m.toFixed(2)} m, max ${fit.check_max_m.toFixed(2)} m`
        : 'no check points';
    const camera =
      fit.camera_position_m && fit.camera_height_m !== null
        ? `camera at (${fit.camera_position_m[0].toFixed(1)}, ${fit.camera_position_m[1].toFixed(1)}) m, ${fit.camera_height_m.toFixed(1)} m high`
        : 'camera position not recoverable: set it by hand';
    summary.textContent = `${fit.inliers}/${fit.fit_points} fit points kept · RMS ${fit.fit_rms_m.toFixed(2)} m · ${check} · ${camera}`;
  }

  private renderList(): void {
    const list = element<HTMLUListElement>('#items');
    if (this.mode === 'zones') {
      list.replaceChildren(...this.zones.map((zone, index) => this.zoneItem(zone, index)));
      return;
    }
    if (this.mode === 'regions') {
      list.replaceChildren(
        ...this.regions.areas.map((_, index) => this.regionItem('area', index)),
        ...this.regions.lines.map((_, index) => this.regionItem('line', index)),
        ...this.regions.signals.map((_, index) => this.regionItem('signal', index)),
      );
      return;
    }
    list.replaceChildren(...this.points.map((point, index) => this.pointItem(point, index)));
  }

  private pointItem(point: ControlPoint, index: number): HTMLLIElement {
    const item = document.createElement('li');
    const residual = this.residual(point.id);
    const label = document.createElement('span');
    label.style.color = this.pointColor(point);
    label.textContent = `${point.id} · ${point.check ? 'check' : 'fit'}${
      residual ? ` · ${residual.error_m.toFixed(2)} m` : ''
    }${residual && !residual.check && !residual.inlier ? ' · outlier' : ''}`;
    const role = document.createElement('button');
    role.type = 'button';
    role.textContent = point.check ? 'Use for fit' : 'Use as check';
    role.addEventListener('click', () => {
      point.check = !point.check;
      void this.refit();
    });
    const remove = document.createElement('button');
    remove.type = 'button';
    remove.textContent = 'Remove';
    remove.addEventListener('click', () => {
      this.points.splice(index, 1);
      void this.refit();
      this.renderList();
    });
    item.append(label, role, remove);
    return item;
  }

  private zoneItem(zone: EditableZone, index: number): HTMLLIElement {
    const item = document.createElement('li');
    const id = document.createElement('input');
    id.value = zone.id;
    id.addEventListener('change', () => (zone.id = id.value.trim() || zone.id));
    const type = document.createElement('select');
    for (const value of ZONE_TYPES) {
      const option = document.createElement('option');
      option.value = value;
      option.textContent = value;
      option.selected = zone.type === value;
      type.append(option);
    }
    type.addEventListener('change', () => (zone.type = type.value as ZoneType));
    const remove = document.createElement('button');
    remove.type = 'button';
    remove.textContent = 'Remove';
    remove.addEventListener('click', () => {
      this.zones.splice(index, 1);
      this.renderList();
      this.redraw();
    });
    item.append(id, type, remove);
    return item;
  }

  private finishZone(): void {
    if (this.drawing.length < 3) {
      this.message('A zone needs at least three corners.');
      return;
    }
    this.zones.push({ id: this.uniqueZoneId('zone'), type: 'crosswalk', polygon: this.drawing });
    this.drawing = [];
    this.renderList();
    this.redraw();
  }

  private uniqueZoneId(prefix: string): string {
    let n = 1;
    while (this.zones.some((z) => z.id === `${prefix}_${n}`)) {
      n++;
    }
    return `${prefix}_${n}`;
  }

  private suggestCrossings(): void {
    if (!this.map.osm) {
      return;
    }
    const suggestions = crossingSuggestions(this.map.osm);
    for (const polygon of suggestions) {
      this.zones.push({ id: this.uniqueZoneId('crosswalk_osm'), type: 'crosswalk', polygon });
    }
    this.message(`${suggestions.length} crossings added from OpenStreetMap; check and edit them.`);
    this.renderList();
    this.redraw();
  }

  /** Saves the zones and, once enough fit points exist, the calibration. */
  private async save(): Promise<void> {
    const saved: string[] = [];
    try {
      const zones = {
        type: 'FeatureCollection' as const,
        coordinate_frame: 'scene_ground',
        features: this.zones.map((z) => ({
          properties: { id: z.id, type: z.type },
          geometry: {
            type: 'Polygon' as const,
            coordinates: [[...z.polygon, z.polygon[0] as Point]],
          },
        })),
      };
      await saveZones(zones);
      // Zones belong to the intersection: the other running cameras take them over at once.
      await Promise.allSettled(siblingCameras.map((base) => saveZones(zones, base)));
      saved.push(`${this.zones.length} zones`);
      await saveRegions(this.regions);
      saved.push(`${this.regions.areas.length} ROIs and ${this.regions.lines.length} lines`);
      if (this.fit) {
        const fit = await saveCalibration(this.points, this.cameraOverride, this.cameraHeight);
        saved.push(`calibration (RMS ${fit.fit_rms_m.toFixed(2)} m); Apply to restart with it`);
      }
      this.message(`Saved ${saved.join(' and ')}.`);
      toast(`Saved ${saved.join(' and ')}.`, 'ok');
    } catch (error) {
      this.message(
        `${saved.length > 0 ? `Saved ${saved.join(', ')}; then: ` : ''}${(error as Error).message}`,
      );
    }
  }

  private async apply(): Promise<void> {
    await requestRestart();
    this.message('Restarting the server…');
    setTimeout(() => location.reload(), RESTART_RELOAD_MS);
  }

  private message(text: string): void {
    element('#message').textContent = text;
  }

  private redraw(): void {
    this.imageCanvas.redraw();
    this.mapCanvas.redraw();
  }

  private marker(c: CanvasRenderingContext2D, x: number, y: number, color: string, label: string) {
    c.beginPath();
    c.arc(x, y, MARKER_RADIUS, 0, 2 * Math.PI);
    c.lineWidth = 2;
    c.strokeStyle = color;
    c.stroke();
    c.font = '600 12px Inter, "Segoe UI", system-ui, sans-serif';
    // Dark outline keeps labels readable on the photo and the camera image.
    c.lineWidth = 3;
    c.strokeStyle = COLORS.label;
    c.strokeText(label, x + MARKER_RADIUS + 3, y - MARKER_RADIUS);
    c.fillStyle = color;
    c.fillText(label, x + MARKER_RADIUS + 3, y - MARKER_RADIUS);
  }

  private polygon(
    c: CanvasRenderingContext2D,
    screen: [number, number][],
    fill: string | null,
    edge: string = COLORS.zoneEdge,
  ) {
    if (screen.length < 2) {
      return;
    }
    c.beginPath();
    screen.forEach(([x, y], i) => (i === 0 ? c.moveTo(x, y) : c.lineTo(x, y)));
    c.closePath();
    if (fill) {
      c.fillStyle = fill;
      c.fill();
    }
    c.strokeStyle = edge;
    c.lineWidth = 1.5;
    c.stroke();
  }

  private drawImage(c: CanvasRenderingContext2D, view: View2D): void {
    if (this.image.complete && this.image.naturalWidth > 0) {
      const [x, y] = view.toScreen(0, 0);
      c.drawImage(this.image, x, y, this.image.width * view.scale, this.image.height * view.scale);
    }
    // Zones and map points projected into the image with the current fit; the zones belong to
    // the whole site, so only the part inside this camera's frame is drawn.
    const projection = this.projection;
    if (projection) {
      c.save();
      if (this.image.complete && this.image.naturalWidth > 0) {
        const [x, y] = view.toScreen(0, 0);
        c.beginPath();
        c.rect(x, y, this.image.width * view.scale, this.image.height * view.scale);
        c.clip();
      }
      for (const zone of this.zones) {
        const corners = projection
          .polygonToImage(zone.polygon)
          .map(([u, v]) => view.toScreen(u, v));
        if (corners.length >= 3) {
          this.polygon(c, corners, COLORS.zone);
        }
      }
      c.restore();
    }
    for (const point of this.points) {
      const [sx, sy] = view.toScreen(point.u, point.v);
      const mapped = projection?.toImage(point.x, point.y);
      if (mapped) {
        const [mx, my] = view.toScreen(mapped[0], mapped[1]);
        c.beginPath();
        c.moveTo(sx, sy);
        c.lineTo(mx, my);
        c.strokeStyle = this.pointColor(point);
        c.stroke();
      }
      this.marker(c, sx, sy, this.pointColor(point), point.id);
    }
    if (this.pendingPixel) {
      const [sx, sy] = view.toScreen(...this.pendingPixel);
      this.marker(c, sx, sy, COLORS.pending, 'new');
    }
    this.drawRegions(c, view);
  }

  private drawMap(c: CanvasRenderingContext2D, view: View2D): void {
    const ortho = this.map.ortho;
    if (ortho && this.orthoImage.complete && this.orthoImage.naturalWidth > 0) {
      // Affine image -> screen from three corners (top-left, top-right, bottom-left).
      const [tl, tr, , bl] = ortho.corners.map(([x, y]) => view.toScreen(x, y));
      if (tl && tr && bl) {
        const w = this.orthoImage.naturalWidth;
        const h = this.orthoImage.naturalHeight;
        c.save();
        c.transform(
          (tr[0] - tl[0]) / w,
          (tr[1] - tl[1]) / w,
          (bl[0] - tl[0]) / h,
          (bl[1] - tl[1]) / h,
          tl[0],
          tl[1],
        );
        c.drawImage(this.orthoImage, 0, 0);
        c.restore();
      }
    }
    for (const road of this.map.osm?.roads ?? []) {
      c.beginPath();
      road.points.forEach(([x, y], i) => {
        const [sx, sy] = view.toScreen(x, y);
        if (i === 0) {
          c.moveTo(sx, sy);
        } else {
          c.lineTo(sx, sy);
        }
      });
      c.strokeStyle = COLORS.road;
      c.lineWidth = 1;
      c.stroke();
    }
    for (const zone of this.zones) {
      this.polygon(
        c,
        zone.polygon.map(([x, y]) => view.toScreen(x, y)),
        COLORS.zone,
      );
    }
    if (this.drawing.length > 0) {
      c.setLineDash([4, 4]);
      this.polygon(
        c,
        this.drawing.map(([x, y]) => view.toScreen(x, y)),
        null,
      );
      c.setLineDash([]);
    }
    for (const point of this.points) {
      const [sx, sy] = view.toScreen(point.x, point.y);
      this.marker(c, sx, sy, this.pointColor(point), point.id);
    }
    const camera = this.cameraOverride ?? this.fit?.camera_position_m ?? null;
    if (camera) {
      const [sx, sy] = view.toScreen(camera[0], camera[1]);
      c.fillStyle = COLORS.camera;
      c.beginPath();
      c.moveTo(sx, sy - 9);
      c.lineTo(sx + 7, sy + 6);
      c.lineTo(sx - 7, sy + 6);
      c.closePath();
      c.fill();
    }
  }
}

function layout(): HTMLElement {
  const tab = (mode: Mode, label: string, active = false) =>
    h('button', { type: 'button', 'data-mode': mode, class: active ? 'active' : '' }, label);
  return h(
    'div',
    { class: 'calibration' },
    h(
      'div',
      { class: 'calib-toolbar' },
      h(
        'nav',
        { class: 'tabs segmented' },
        tab('points', 'Control points', true),
        tab('zones', 'Zones'),
        tab('regions', 'ROI and lines'),
        tab('camera', 'Camera position'),
      ),
      h('span', { class: 'spacer' }),
      h(
        'button',
        { id: 'refresh-frame', type: 'button', class: 'btn small' },
        icon('image'),
        'New camera frame',
      ),
      h(
        'button',
        { id: 'save', type: 'button', class: 'btn small primary' },
        icon('check'),
        'Save',
      ),
      h(
        'button',
        {
          id: 'apply',
          type: 'button',
          class: 'btn small',
          title: 'Restart the site with the saved calibration',
        },
        icon('restart'),
        'Apply',
      ),
    ),
    h(
      'div',
      { class: 'workspace' },
      h(
        'section',
        { class: 'pane' },
        h('h2', {}, icon('video'), 'Camera image', h('span', { id: 'scene-id', class: 'note' })),
        h('canvas', { id: 'image' }),
      ),
      h(
        'section',
        { class: 'pane' },
        h('h2', {}, icon('map'), 'Map', h('span', { id: 'map-note', class: 'note' })),
        h('canvas', { id: 'map' }),
      ),
      h(
        'aside',
        { class: 'calib-sidebar' },
        h('p', { id: 'hint', class: 'hint' }),
        h('div', { id: 'summary', class: 'summary' }),
        h('ul', { id: 'items', class: 'items' }),
        h(
          'label',
          { id: 'camera-height', class: 'field', hidden: true },
          h('span', {}, 'Camera height above the road (m)'),
          h('input', { type: 'number', min: 0.5, step: 0.1, class: 'input' }),
        ),
        h(
          'div',
          { id: 'zone-actions', class: 'actions', hidden: true },
          h(
            'button',
            { id: 'suggest', type: 'button', class: 'btn small' },
            icon('map'),
            'Add OSM crossings',
          ),
          h(
            'button',
            { id: 'finish-zone', type: 'button', class: 'btn small primary' },
            icon('check'),
            'Finish zone',
          ),
        ),
        h(
          'div',
          { id: 'region-actions', class: 'actions', hidden: true },
          h(
            'button',
            { id: 'finish-area', type: 'button', class: 'btn small' },
            icon('layers'),
            'Finish ROI',
          ),
          h(
            'button',
            { id: 'finish-line', type: 'button', class: 'btn small primary' },
            icon('route'),
            'Finish line',
          ),
          h(
            'button',
            { id: 'finish-signal', type: 'button', class: 'btn small' },
            icon('pulse'),
            'Finish signal',
          ),
        ),
        h('p', { id: 'message', class: 'message' }),
        h('p', { id: 'attribution', class: 'attribution' }),
      ),
    ),
  );
}

/**
 * The calibration needs a current camera frame: keep the site's camera images flowing (the
 * server encodes frames only for viewers) and resume paused playback.
 */
async function ensureFrames(onFirstFrame: () => void, siteId: string | null): Promise<void> {
  let first = true;
  subscribeVideo(() => {
    if (first) {
      first = false;
      onFirstFrame();
    }
  });
  try {
    const status = await fetchStatus();
    if (status.playing === false) {
      // Behind the manager the site's cameras play together.
      await (siteId ? sitePlayback(siteId, { action: 'play' }) : sendPlayback({ action: 'play' }));
      toast('Playback resumed to get a camera frame.', 'info');
    }
  } catch {
    // The site reports its status once it runs; the frame arrives with the first image.
  }
}

async function start(): Promise<void> {
  const siteId = param('site');
  const shell = await mountShell({
    page: 'calibration',
    siteId,
    title: 'Calibration and zones',
    fullBleed: true,
  });
  if (shell.managed && !siteId) {
    location.replace('/');
    return;
  }
  setSite(null);
  let cameraSwitch: HTMLElement | null = null;
  if (shell.managed && siteId) {
    shell.setActions(
      h('a', { class: 'btn small', href: links.twin(siteId) }, icon('twin'), 'Live twin'),
      h('a', { class: 'btn small', href: links.settings(siteId) }, icon('settings'), 'Settings'),
    );
    const requested = param('camera');
    const before = await getSite(siteId).catch(() => null);
    const wanted = before ? findCamera(before, requested) : null;
    if (before && wanted && (!before.enabled || !wanted.enabled)) {
      await cameraAction(siteId, wanted.id, 'start')
        .then(() => toast(`${wanted.name} is starting for the calibration.`, 'info'))
        .catch((error: unknown) => toast((error as Error).message, 'error'));
    }
    const { site, camera } = await waitForCamera(
      shell.content,
      siteId,
      requested,
      'calibrate it against the camera image',
    );
    setSite(siteId, camera.id);
    shell.setTitle(`Calibration · ${camera.name}`);
    siblingCameras = site.cameras
      .filter((c) => c.id !== camera.id && c.runtime.state === 'running')
      .map((c) => cameraBase(siteId, c.id));
    if (site.cameras.length > 1) {
      cameraSwitch = h(
        'nav',
        { class: 'segmented camera-switch', title: 'Camera to calibrate' },
        site.cameras.map((c) =>
          h(
            'a',
            {
              class: c.id === camera.id ? 'active' : '',
              href: links.calibration(siteId, c.id),
              title: APPROACH_LABELS[c.approach],
            },
            icon('video'),
            c.name,
            c.calibrated ? null : h('i', { class: 'todo', title: 'Not calibrated yet' }),
          ),
        ),
      );
    }
  }
  shell.content.append(layout());
  if (cameraSwitch) {
    element('.calib-toolbar').prepend(cameraSwitch);
  }
  const scene = await fetchScene();
  element('#scene-id').textContent = ` · ${scene.image_size_px[0]} × ${scene.image_size_px[1]} px`;
  let map: MapData;
  if (scene.map && scene.geo_anchor) {
    element('#map-note').textContent = ' · aerial image, OpenStreetMap roads';
    map = await loadMapData(siteUrl(scene.map));
  } else {
    // Without map layers in the scene frame (not geo-referenced, e.g. a dataset or sensor frame)
    // the camera image still takes ROIs, lines and signals; control points need the map.
    const text = scene.calibrated
      ? 'This scene is not geo-referenced, so no map is shown: the camera keeps its calibration, ROIs, lines and signals can be drawn on the camera image.'
      : siteId
        ? 'This site has no geographic anchor or map layers yet: set its location and prepare the map under Settings to calibrate the camera.'
        : 'This scene has no geo anchor or map layers; run tools/prepare_map_data.py for it first.';
    element('#map-note').textContent = ' · not geo-referenced';
    toast(text, scene.calibrated ? 'info' : 'error');
    map = {
      base: '',
      manifest: { scene_id: scene.scene_id, radius_m: 100, layers: {}, attribution: {} },
      ortho: null,
      roadmap: null,
      osm: null,
      buildings: [],
    };
  }
  const app = new CalibrationApp(scene, map);
  await app.start();
  await ensureFrames(() => app.loadFrame(), shell.managed ? siteId : null);
}

void start();
