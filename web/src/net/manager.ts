/** Client of the site manager (manager/jp_manager.py): sites, cameras, processes, logs, host. */
import type { MapTransform } from '../bev/siteFrame';
import type { FusionMode } from './siteState';
import type { PlaybackCommand, ServerStatus } from './client';

export type SourceMode = 'live' | 'oracle' | 'replay';
export type RunState = 'stopped' | 'starting' | 'running' | 'stopping' | 'backoff' | 'failed';
/** State of a site: that of its cameras, "partial" while only some of them run. */
export type SiteRunState = RunState | 'partial';

export interface SiteSource {
  mode: SourceMode;
  /** Video or sequence under the data root, stream URL, or replay file in the repository. */
  input: string;
}

export interface GeoAnchor {
  lat_deg: number;
  lon_deg: number;
  alt_m: number;
}

export interface MapJob {
  state: 'running' | 'done' | 'failed';
  message: string;
  started_at: number;
  finished_at?: number;
}

/** Arm of the intersection a camera watches. */
export type Approach =
  | 'north'
  | 'north-east'
  | 'east'
  | 'south-east'
  | 'south'
  | 'south-west'
  | 'west'
  | 'north-west'
  | 'centre';

export const APPROACH_LABELS: Record<Approach, string> = {
  north: 'North arm',
  'north-east': 'North-east arm',
  east: 'East arm',
  'south-east': 'South-east arm',
  south: 'South arm',
  'south-west': 'South-west arm',
  west: 'West arm',
  'north-west': 'North-west arm',
  centre: 'Centre (overview)',
};

/** Compass bearing of an approach (degrees clockwise from north); null for the centre. */
export const APPROACH_BEARING: Record<Approach, number | null> = {
  north: 0,
  'north-east': 45,
  east: 90,
  'south-east': 135,
  south: 180,
  'south-west': 225,
  west: 270,
  'north-west': 315,
  centre: null,
};

export interface CameraRuntime {
  state: RunState;
  pid: number | null;
  port: number | null;
  started_at: number | null;
  uptime_s: number | null;
  restarts: number;
  last_exit: number | null;
  error: string | null;
  status: ServerStatus | null;
}

export interface PlaybackSettings {
  loop: boolean;
  speed: number;
}

export interface VideoSettings {
  max_fps: number;
  width_px: number;
  jpeg_quality: number;
}

/** One camera of a site, with its own source, calibration and server. */
export interface Camera {
  id: string;
  name: string;
  approach: Approach;
  image_size_px: [number, number] | null;
  enabled: boolean;
  calibrated: boolean;
  control_points: boolean;
  /** Detection areas and counting lines drawn on the camera image (regions.json). */
  regions: {
    areas: number;
    lines: { id: string; name: string }[];
    signals: { id: string; name: string; placed: boolean }[];
  };
  source: SiteSource;
  playback: PlaybackSettings;
  video: VideoSettings;
  /** Added to the camera's timestamps on the site clock (ms): aligns recordings that did not
   * start together, or the latency of a live stream. */
  time_offset_ms: number;
  runtime: CameraRuntime;
}

/** A site: one intersection with up to four cameras in a shared world frame. */
export interface Site {
  id: string;
  name: string;
  description: string;
  enabled: boolean;
  kind: string;
  /** "intersection": cameras below cameras/; "single": an older single-camera scene folder. */
  layout: 'intersection' | 'single';
  /** Geographic position: the intersection centre, or for sensor-frame scenes the map anchor. */
  geo_anchor: GeoAnchor | null;
  /** World frame is east-north-up at the geo anchor (false: a sensor frame, e.g. benchmark). */
  georeferenced: boolean;
  map_radius_m: number | null;
  /** Placement of the map in a sensor-frame scene, aligned in the twin. */
  map_transform: MapTransform | null;
  has_map: boolean;
  zones: boolean;
  max_cameras: number;
  cameras: Camera[];
  runtime: {
    state: SiteRunState;
    error: string | null;
    map_job: MapJob | null;
    /** Latest state of the site world model: fusion mode and road users (null before one). */
    world: { mode: FusionMode; road_users: number; t_ns: number } | null;
  };
}

export interface SitePatch {
  name?: string;
  description?: string;
  geo_anchor?: GeoAnchor | null;
  map_radius_m?: number;
  map_transform?: MapTransform;
}

export interface CameraPatch {
  name?: string;
  approach?: Approach;
  image_size_px?: [number, number];
  source?: SiteSource;
  playback?: PlaybackSettings;
  video?: VideoSettings;
  time_offset_ms?: number;
}

export interface NewCamera {
  name: string;
  approach: Approach;
  image_size_px: [number, number];
  source: SiteSource;
}

export interface NewSite {
  id: string;
  name: string;
  description: string;
  geo_anchor: GeoAnchor;
  map_radius_m: number;
  enabled: boolean;
  prepare_map: boolean;
  cameras: NewCamera[];
}

export interface GpuInfo {
  name: string;
  utilization: number | null;
  memory_used_mib: number | null;
  memory_total_mib: number | null;
  temperature_c: number | null;
  power_w: number | null;
}

export interface SystemInfo {
  version: string;
  started_at: number;
  uptime_s: number;
  backend_on: boolean;
  sites_total: number;
  sites_enabled: number;
  sites_running: number;
  cameras_total: number;
  cameras_running: number;
  max_running: number;
  data_root: string;
  server_binary: string;
  server_binary_found: boolean;
  host: {
    cpu_count: number | null;
    load: [number, number, number] | null;
    memory: { total: number; used: number } | null;
    disk: { total: number; used: number } | null;
  };
  gpus: GpuInfo[] | null;
}

export interface InputFile {
  path: string;
  size: number | null;
}

export interface Inputs {
  videos: InputFile[];
  sequences: InputFile[];
  replays: InputFile[];
}

export type LogLevel = 'trace' | 'debug' | 'info' | 'warning' | 'error' | 'critical';

export interface LogLine {
  t: number;
  level: LogLevel;
  source: string;
  msg: string;
}

export type SystemAction = 'start-backend' | 'stop-backend' | 'restart-backend' | 'restart-manager';
export type SiteAction = 'start' | 'stop' | 'restart' | 'prepare-map';
export type CameraAction = 'start' | 'stop' | 'restart';

const API = '/manager/api';

async function request<T>(method: string, path: string, body?: unknown): Promise<T> {
  const response = await fetch(API + path, {
    method,
    headers: body === undefined ? {} : { 'Content-Type': 'application/json' },
    body: body === undefined ? undefined : JSON.stringify(body),
  });
  const text = await response.text();
  let parsed: unknown = null;
  try {
    parsed = text ? JSON.parse(text) : null;
  } catch {
    parsed = null;
  }
  if (!response.ok) {
    const message = (parsed as { error?: string } | null)?.error;
    throw new Error(message ?? `${method} ${path} failed (${response.status})`);
  }
  return parsed as T;
}

/** True when the console is served by the manager (not by a single jp_server). */
export async function managerAvailable(): Promise<boolean> {
  try {
    const response = await fetch(`${API}/health`);
    return response.ok && (response.headers.get('Content-Type') ?? '').includes('json');
  } catch {
    return false;
  }
}

export async function listSites(): Promise<Site[]> {
  return (await request<{ sites: Site[] }>('GET', '/sites')).sites;
}

export const getSite = (id: string): Promise<Site> =>
  request('GET', `/sites/${encodeURIComponent(id)}`);

export const createSite = (site: NewSite): Promise<Site> => request('POST', '/sites', site);

export const updateSite = (id: string, patch: SitePatch): Promise<Site> =>
  request('PATCH', `/sites/${encodeURIComponent(id)}`, patch);

export const deleteSite = (id: string): Promise<unknown> =>
  request('DELETE', `/sites/${encodeURIComponent(id)}`);

export const siteAction = (id: string, action: SiteAction): Promise<Site> =>
  request('POST', `/sites/${encodeURIComponent(id)}/${action}`);

/**
 * Play, pause or seek of all cameras of a site together; a seek names the camera whose frame
 * index it gives.
 */
export const sitePlayback = (
  id: string,
  command: PlaybackCommand,
  camera?: string,
): Promise<{ synced: boolean }> =>
  request('POST', `/sites/${encodeURIComponent(id)}/playback`, { ...command, camera });

const cameraPath = (site: string, camera: string): string =>
  `/sites/${encodeURIComponent(site)}/cameras/${encodeURIComponent(camera)}`;

/** Adds a camera; returns the site with it. */
export const addCamera = (site: string, camera: NewCamera): Promise<Site> =>
  request('POST', `/sites/${encodeURIComponent(site)}/cameras`, camera);

export const updateCamera = (site: string, camera: string, patch: CameraPatch): Promise<Site> =>
  request('PATCH', cameraPath(site, camera), patch);

export const deleteCamera = (site: string, camera: string): Promise<Site> =>
  request('DELETE', cameraPath(site, camera));

export const cameraAction = (site: string, camera: string, action: CameraAction): Promise<Site> =>
  request('POST', `${cameraPath(site, camera)}/${action}`);

/** Live figures of a site: road users from its world model, rates averaged over its cameras. */
export interface SiteFigures {
  running: number;
  /** Road users of the site's latest state; null while the world model has none. */
  objects: number | null;
  fps: number | null;
  latency_ms: number | null;
}

export function siteFigures(site: Site): SiteFigures {
  const statuses = site.cameras
    .filter((c) => c.runtime.state === 'running' && c.runtime.status)
    .map((c) => c.runtime.status as ServerStatus);
  const mean = (values: (number | undefined)[]): number | null => {
    const known = values.filter((v): v is number => v !== undefined);
    return known.length > 0 ? known.reduce((a, b) => a + b, 0) / known.length : null;
  };
  return {
    running: site.cameras.filter((c) => c.runtime.state === 'running').length,
    objects: site.runtime.world?.road_users ?? null,
    fps: mean(statuses.map((s) => s.fps)),
    latency_ms: mean(statuses.map((s) => s.latency_ms)),
  };
}

/** The camera of a site with this id, else its first camera. */
export function findCamera(site: Site, id: string | null): Camera | null {
  return site.cameras.find((c) => c.id === id) ?? site.cameras[0] ?? null;
}

export const systemInfo = (): Promise<SystemInfo> => request('GET', '/system');

export const systemAction = (action: SystemAction): Promise<unknown> =>
  request('POST', `/system/${action}`);

export const listInputs = (): Promise<Inputs> => request('GET', '/inputs');

export async function fetchLogs(source: string, lines = 1000): Promise<LogLine[]> {
  const query = new URLSearchParams({ source, lines: String(lines) });
  return (await request<{ lines: LogLine[] }>('GET', `/logs?${query}`)).lines;
}

/** Live log lines of a source ("*" for all) until the returned function is called. */
export function streamLogs(source: string, onLine: (line: LogLine) => void): () => void {
  const events = new EventSource(`${API}/logs/stream?${new URLSearchParams({ source })}`);
  events.onmessage = (event: MessageEvent<string>) => onLine(JSON.parse(event.data) as LogLine);
  return () => events.close();
}

export type Tone = 'ok' | 'warn' | 'danger' | '';

/** Display state of a site: label and colour class of its status pill. */
export function siteState(site: Site): { label: string; tone: Tone } {
  if (site.runtime.state === 'partial') {
    const running = site.cameras.filter((c) => c.runtime.state === 'running').length;
    return { label: `${running} of ${site.cameras.length} cameras`, tone: 'warn' };
  }
  if (site.cameras.length === 0) {
    return { label: 'No cameras', tone: '' };
  }
  return runState(site.runtime.state, site.enabled);
}

/** Display state of a camera. */
export function cameraState(site: Site, camera: Camera): { label: string; tone: Tone } {
  return runState(camera.runtime.state, site.enabled && camera.enabled);
}

function runState(state: RunState, enabled: boolean): { label: string; tone: Tone } {
  switch (state) {
    case 'running':
      return { label: 'Running', tone: 'ok' };
    case 'starting':
      return { label: 'Starting', tone: 'warn' };
    case 'stopping':
      return { label: 'Stopping', tone: 'warn' };
    case 'backoff':
      return { label: 'Restarting', tone: 'danger' };
    case 'failed':
      return { label: 'Failed', tone: 'danger' };
    default:
      return { label: enabled ? 'Waiting' : 'Inactive', tone: '' };
  }
}

/** Kind of camera source of a site, as chosen in the console. */
export type SourceKind = 'file' | 'stream' | 'oracle' | 'replay';

export function sourceKind(source: SiteSource): SourceKind {
  if (source.mode === 'oracle' || source.mode === 'replay') {
    return source.mode;
  }
  return source.input.includes('://') ? 'stream' : 'file';
}

export const SOURCE_LABELS: Record<SourceKind, string> = {
  file: 'Video file',
  stream: 'Camera stream',
  oracle: 'Benchmark labels',
  replay: 'Recorded replay',
};

/** What jp_server --probe reports about a video or stream. */
export interface ProbeResult {
  width: number;
  height: number;
  fps: number;
  frames: number | null;
}

export const probeInput = (input: string): Promise<ProbeResult> =>
  request('GET', `/probe?${new URLSearchParams({ input })}`);

/** Uploads a video below the data root; reports the sent fraction (0..1) while uploading. */
export function uploadVideo(
  file: File,
  onProgress: (fraction: number) => void,
): Promise<{ path: string; size: number }> {
  return new Promise((resolve, reject) => {
    const xhr = new XMLHttpRequest();
    xhr.open('POST', `${API}/videos`);
    xhr.upload.onprogress = (e) => {
      if (e.lengthComputable) {
        onProgress(e.loaded / e.total);
      }
    };
    xhr.onload = () => {
      let body: { path?: string; size?: number; error?: string } = {};
      try {
        body = JSON.parse(xhr.responseText) as typeof body;
      } catch {
        // Fall through to the status check.
      }
      if (xhr.status >= 200 && xhr.status < 300 && body.path) {
        resolve({ path: body.path, size: body.size ?? file.size });
      } else {
        reject(new Error(body.error ?? `upload failed (${xhr.status})`));
      }
    };
    xhr.onerror = () => reject(new Error('upload failed: connection lost'));
    const form = new FormData();
    form.append('file', file);
    xhr.send(form);
  });
}
