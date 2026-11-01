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
import {
  fetchScene,
  fetchStatus,
  type SceneDescription,
  sendPlayback,
  setSite,
  siteUrl,
  subscribeVideo,
} from '../net/client';
import { getSite, siteAction } from '../net/manager';
import { h, param } from '../ui/dom';
import { toast } from '../ui/feedback';
import { icon } from '../ui/icons';
import { links, mountShell } from '../ui/shell';
import { waitForSite } from '../ui/siteGate';
import { PanZoomCanvas, type View2D } from './panZoom';

type Mode = 'points' | 'zones' | 'camera';

const ZONE_TYPES = ['crosswalk', 'lane', 'intersection'] as const;
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
  private cameraHeight: number | null = null;
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
    element('#camera-height').hidden = mode !== 'camera';
    const hints: Record<Mode, string> = {
      points:
        'Click a point in the camera image, then the same point on the map (road markings, kerb corners). Use at least 8 points spread over the scene; every fourth point is a check point. Wheel: zoom, drag: pan.',
      zones:
        'Click the corners of a zone on the map, then "Finish zone" (or Enter). Crosswalk zones trigger VRU events; lanes and intersections are drawn in the twin.',
      camera:
        'The camera pose is estimated from the control points. Where they cannot constrain it (long lenses, points along one road), click the camera position on the map and enter its height above the road.',
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
      await saveZones({
        type: 'FeatureCollection',
        coordinate_frame: 'scene_ground',
        features: this.zones.map((z) => ({
          properties: { id: z.id, type: z.type },
          geometry: { type: 'Polygon', coordinates: [[...z.polygon, z.polygon[0] as Point]] },
        })),
      });
      saved.push(`${this.zones.length} zones`);
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
async function ensureFrames(onFirstFrame: () => void): Promise<void> {
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
      await sendPlayback({ action: 'play' });
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
  setSite(shell.managed ? siteId : null);
  if (shell.managed && siteId) {
    shell.setActions(
      h('a', { class: 'btn small', href: links.twin(siteId) }, icon('twin'), 'Live twin'),
      h('a', { class: 'btn small', href: links.settings(siteId) }, icon('settings'), 'Settings'),
    );
    const site = await getSite(siteId).catch(() => null);
    if (site && !site.enabled) {
      await siteAction(siteId, 'start')
        .then(() => toast(`${site.name} is starting for the calibration.`, 'info'))
        .catch((error: unknown) => toast((error as Error).message, 'error'));
    }
    await waitForSite(shell.content, siteId, 'calibrate it against the camera image');
  }
  shell.content.append(layout());
  const scene = await fetchScene();
  element('#scene-id').textContent = ` · ${scene.image_size_px[0]} × ${scene.image_size_px[1]} px`;
  if (!scene.map || !scene.geo_anchor) {
    const text =
      scene.kind === 'benchmark'
        ? 'Benchmark scenes are calibrated from their labels. To show the map, set the location under Settings, prepare the map and align it in the live twin.'
        : siteId
          ? 'This site has no geographic anchor or map layers yet: set its location and prepare the map under Settings.'
          : 'This scene has no geo anchor or map layers; run tools/prepare_map_data.py for it first.';
    element('#message').textContent = text;
    toast(text, 'error');
    return;
  }
  element('#map-note').textContent = ' · aerial image, OpenStreetMap roads';
  const app = new CalibrationApp(scene, await loadMapData(siteUrl(scene.map)));
  await app.start();
  await ensureFrames(() => app.loadFrame());
}

void start();
