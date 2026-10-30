import '../style.css';
import './calibration.css';

import { crossingSuggestions, loadMapData, type MapData, type Point } from '../map/mapData';
import { attributionLines } from '../map/mapLayers';
import {
  applyHomography,
  type ControlPoint,
  fetchControlPoints,
  fitControlPoints,
  type FitResult,
  invert3,
  requestRestart,
  saveCalibration,
  saveZones,
} from '../net/calibrationApi';
import { fetchScene, type SceneDescription } from '../net/client';
import { PanZoomCanvas, type View2D } from './panZoom';

type Mode = 'points' | 'zones' | 'camera';

interface EditableZone {
  id: string;
  type: 'crosswalk' | 'intersection';
  polygon: Point[];
}

const COLORS = {
  pending: '#f59e0b',
  fit: '#2f7cf6',
  check: '#16a34a',
  outlier: '#e5484d',
  zone: 'rgba(47, 124, 246, 0.25)',
  zoneEdge: '#2f7cf6',
  drawing: '#f59e0b',
  camera: '#2b2f35',
  road: 'rgba(80, 80, 80, 0.35)',
};
const RESTART_RELOAD_MS = 6000;
const MARKER_RADIUS = 6;

function element<T extends HTMLElement>(selector: string): T {
  const found = document.querySelector<T>(selector);
  if (!found) {
    throw new Error(`missing element ${selector}`);
  }
  return found;
}

/** Image pixels -> world through the fitted homography, and back. */
class Projection {
  constructor(
    readonly imageToGround: number[][],
    private readonly groundToImage: number[][] | null,
  ) {}

  static from(fit: FitResult | null): Projection | null {
    return fit ? new Projection(fit.image_to_ground, invert3(fit.image_to_ground)) : null;
  }

  toImage(x: number, y: number): [number, number] | null {
    return this.groundToImage ? applyHomography(this.groundToImage, x, y) : null;
  }
}

class CalibrationApp {
  private mode: Mode = 'points';
  private points: ControlPoint[] = [];
  private pendingPixel: [number, number] | null = null;
  private fit: FitResult | null = null;
  private projection: Projection | null = null;
  private cameraOverride: [number, number] | null = null;
  private zones: EditableZone[] = [];
  private drawing: Point[] = [];
  private image = new Image();
  private orthoImage = new Image();
  private readonly imageCanvas: PanZoomCanvas;
  private readonly mapCanvas: PanZoomCanvas;
  private nextPoint = 1;

  constructor(
    scene: SceneDescription,
    private readonly map: MapData,
  ) {
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
      type: f.properties.type === 'intersection' ? 'intersection' : 'crosswalk',
      polygon: (f.geometry.coordinates[0] ?? []).slice(0, -1).map(([x = 0, y = 0]) => [x, y]),
    }));
    this.bindControls();
    this.loadImages();
  }

  async start(): Promise<void> {
    this.points = await fetchControlPoints();
    this.nextPoint = this.points.length + 1;
    await this.refit();
    this.setMode('points');
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
    window.addEventListener('keydown', (e) => {
      if (e.key === 'Enter' && this.mode === 'zones') {
        this.finishZone();
      } else if (e.key === 'Escape') {
        this.pendingPixel = null;
        this.drawing = [];
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

  private loadFrame(): void {
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
    this.image.onerror = () => this.message('No camera image yet: start playback in the twin.');
    this.image.src = `/api/frame?t=${Date.now()}`;
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
    const hints: Record<Mode, string> = {
      points:
        'Click a point in the camera image, then the same point on the map (road markings, kerb corners). Use at least 8 points spread over the scene; every fourth point is a check point. Wheel: zoom, drag: pan.',
      zones:
        'Click the corners of a zone on the map, then "Finish zone" (or Enter). Crosswalk zones trigger VRU events.',
      camera:
        'The camera position is estimated from the control points. Click on the map to set it by hand.',
    };
    element('#hint').textContent = hints[mode];
    this.renderList();
    this.redraw();
  }

  private onImageClick(u: number, v: number): void {
    if (this.mode !== 'points') {
      return;
    }
    this.pendingPixel = [u, v];
    this.redraw();
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
      this.projection = Projection.from(this.fit);
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
      summary.textContent = `${this.points.length} points · need at least 4 fit points`;
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
    for (const value of ['crosswalk', 'intersection']) {
      const option = document.createElement('option');
      option.value = value;
      option.textContent = value;
      option.selected = zone.type === value;
      type.append(option);
    }
    type.addEventListener('change', () => (zone.type = type.value as EditableZone['type']));
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

  private async save(): Promise<void> {
    try {
      if (this.mode === 'zones') {
        await saveZones({
          type: 'FeatureCollection',
          coordinate_frame: 'scene_ground',
          features: this.zones.map((z) => ({
            properties: { id: z.id, type: z.type },
            geometry: { type: 'Polygon', coordinates: [[...z.polygon, z.polygon[0] as Point]] },
          })),
        });
        this.message(`Saved ${this.zones.length} zones; events use them right away.`);
      } else {
        const fit = await saveCalibration(this.points, this.cameraOverride);
        this.message(
          `Calibration saved (RMS ${fit.fit_rms_m.toFixed(2)} m). Apply to restart the server with it.`,
        );
      }
    } catch (error) {
      this.message((error as Error).message);
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
    c.font = '12px system-ui, sans-serif';
    c.fillStyle = color;
    c.fillText(label, x + MARKER_RADIUS + 2, y - MARKER_RADIUS);
  }

  private polygon(c: CanvasRenderingContext2D, screen: [number, number][], fill: string | null) {
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
    c.strokeStyle = COLORS.zoneEdge;
    c.lineWidth = 1.5;
    c.stroke();
  }

  private drawImage(c: CanvasRenderingContext2D, view: View2D): void {
    if (this.image.complete && this.image.naturalWidth > 0) {
      const [x, y] = view.toScreen(0, 0);
      c.drawImage(this.image, x, y, this.image.width * view.scale, this.image.height * view.scale);
    }
    // Zones and map points projected into the image with the current fit.
    const projection = this.projection;
    if (projection) {
      for (const zone of this.zones) {
        const corners = zone.polygon
          .map(([x, y]) => projection.toImage(x, y))
          .filter((p): p is [number, number] => p !== null)
          .map(([u, v]) => view.toScreen(u, v));
        this.polygon(c, corners, COLORS.zone);
      }
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

async function start(): Promise<void> {
  const scene = await fetchScene();
  element('#scene-id').textContent = scene.scene_id;
  if (!scene.map || !scene.geo_anchor) {
    element('#message').textContent =
      'This scene has no geo anchor or map layers; run tools/prepare_map_data.py for it first.';
    return;
  }
  element('#map-note').textContent = 'orthophoto, OpenStreetMap roads';
  const app = new CalibrationApp(scene, await loadMapData(scene.map));
  await app.start();
}

void start();
