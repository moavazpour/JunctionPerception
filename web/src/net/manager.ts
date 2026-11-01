/** Client of the site manager (manager/jp_manager.py): sites, processes, logs and the host. */
import type { MapTransform } from '../twin/twinView';
import type { ServerStatus } from './client';

export type SourceMode = 'live' | 'oracle' | 'replay';
export type RunState = 'stopped' | 'starting' | 'running' | 'stopping' | 'backoff' | 'failed';

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

export interface SiteRuntime {
  state: RunState;
  pid: number | null;
  port: number | null;
  started_at: number | null;
  uptime_s: number | null;
  restarts: number;
  last_exit: number | null;
  error: string | null;
  status: ServerStatus | null;
  map_job: MapJob | null;
}

export interface Site {
  id: string;
  name: string;
  description: string;
  enabled: boolean;
  kind: string;
  image_size_px: [number, number] | null;
  /** Geographic position: the geo anchor, or for sensor-frame scenes the anchor of the map. */
  geo_anchor: GeoAnchor | null;
  /** World frame is east-north-up at the geo anchor (false: a sensor frame, e.g. benchmark). */
  georeferenced: boolean;
  map_radius_m: number | null;
  /** Placement of the map in a sensor-frame scene, aligned in the twin. */
  map_transform: MapTransform | null;
  calibrated: boolean;
  has_map: boolean;
  zones: boolean;
  source: SiteSource;
  playback: { loop: boolean; speed: number };
  video: { max_fps: number; width_px: number; jpeg_quality: number };
  runtime: SiteRuntime;
}

export interface SitePatch {
  name?: string;
  description?: string;
  source?: SiteSource;
  playback?: Site['playback'];
  video?: Site['video'];
  image_size_px?: [number, number];
  geo_anchor?: GeoAnchor | null;
  map_radius_m?: number;
  map_transform?: MapTransform;
}

export interface NewSite extends SitePatch {
  id: string;
  name: string;
  source: SiteSource;
  image_size_px: [number, number];
  enabled: boolean;
  prepare_map: boolean;
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

/** Display state of a site: label and colour class of its status pill. */
export function siteState(site: Site): { label: string; tone: 'ok' | 'warn' | 'danger' | '' } {
  switch (site.runtime.state) {
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
      return { label: site.enabled ? 'Waiting' : 'Inactive', tone: '' };
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
