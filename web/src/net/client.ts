import { isSiteState, type SiteState } from './siteState';

/** Twin class as described by the server (configs/classes.yaml). */
export interface ClassInfo {
  name: string;
  length_m: number;
  width_m: number;
  height_m: number;
  vru: boolean;
}

/** Zones of a scene (GeoJSON FeatureCollection of polygons in the scene ground frame). */
export interface ZoneCollection {
  type: 'FeatureCollection';
  coordinate_frame?: string;
  features: {
    properties: { id: string; type: string };
    geometry: { type: 'Polygon'; coordinates: number[][][] };
  }[];
}

export interface SceneDescription {
  scene_id: string;
  /** Camera folder of an intersection; empty for single-camera scenes. */
  camera_id?: string;
  kind: string;
  image_size_px: [number, number];
  calibrated: boolean;
  camera_ground_position_m: [number, number] | null;
  /** Image-to-ground homography (row-major 3 x 3) of a calibrated camera; absent in older servers. */
  image_to_ground?: number[][] | null;
  camera_height_m: number | null;
  /** The pose above was entered by hand, else estimated from the control points (absent: older servers). */
  camera_position_entered?: boolean;
  camera_height_entered?: boolean;
  geo_anchor: { lat_deg: number; lon_deg: number; alt_m: number } | null;
  /** URL of the map manifest, or null if the scene has no map layers. */
  map: string | null;
  zones: ZoneCollection;
  classes: ClassInfo[];
}

export interface ServerStatus {
  scene_id: string;
  camera_id?: string;
  source: string;
  frame_count: number;
  seekable: boolean;
  playing?: boolean;
  frame?: number;
  timestamp_ns?: number;
  objects?: number;
  fps?: number;
  latency_ms?: number;
  clients?: number;
  calibrated?: boolean;
}

export type PlaybackCommand =
  | { action: 'play' }
  | { action: 'pause' }
  | { action: 'seek'; frame: number };

/** Camera image of the side panel: frame index and JPEG bytes. */
export interface VideoFrame {
  frame: number;
  jpeg: Blob;
}

/** Splits a /ws/video message: 8-byte little-endian frame index, then the JPEG. */
export function parseVideoMessage(data: ArrayBuffer): VideoFrame {
  if (data.byteLength < 8) {
    throw new RangeError('video message shorter than its header');
  }
  const frame = Number(new DataView(data).getBigUint64(0, true));
  return { frame, jpeg: new Blob([data.slice(8)], { type: 'image/jpeg' }) };
}

// Behind the site manager every camera is served under /sites/<id>/cameras/<camera>/; a single
// jp_server serves at /.
let siteBase = '';

/** Path prefix of a camera's server behind the manager. */
export function cameraBase(siteId: string, cameraId: string): string {
  return `/sites/${encodeURIComponent(siteId)}/cameras/${encodeURIComponent(cameraId)}`;
}

/**
 * Selects the camera whose server the API calls and streams go to (null: the serving host);
 * without a camera, the site's first one.
 */
export function setSite(siteId: string | null, cameraId: string | null = null): void {
  if (!siteId) {
    siteBase = '';
  } else {
    siteBase = cameraId ? cameraBase(siteId, cameraId) : `/sites/${encodeURIComponent(siteId)}`;
  }
}

/** URL of a path on the selected camera's server (or the one at `base`). */
export function siteUrl(path: string, base: string = siteBase): string {
  return base + path;
}

export async function fetchScene(base: string = siteBase): Promise<SceneDescription> {
  const response = await fetch(siteUrl('/api/scene', base));
  if (!response.ok) {
    throw new Error(`GET /api/scene failed: ${response.status}`);
  }
  return (await response.json()) as SceneDescription;
}

export async function fetchStatus(base: string = siteBase): Promise<ServerStatus> {
  const response = await fetch(siteUrl('/api/status', base));
  if (!response.ok) {
    throw new Error(`GET /api/status failed: ${response.status}`);
  }
  return (await response.json()) as ServerStatus;
}

export async function sendPlayback(
  command: PlaybackCommand,
  base: string = siteBase,
): Promise<void> {
  await fetch(siteUrl('/api/playback', base), { method: 'POST', body: JSON.stringify(command) });
}

const RECONNECT_DELAY_MS = 1000;
// A connection without messages for this long is replaced; this also recovers connections to a
// restarted server that were never reported as closed. Paused playback just reconnects quietly.
const STALE_AFTER_MS = 5000;

/** Stops a subscription and its reconnects. */
export type Unsubscribe = () => void;

/** WebSocket that reconnects after the connection drops or goes silent, until closed. */
function openSocket(
  url: string,
  binary: boolean,
  onMessage: (data: string | ArrayBuffer) => void,
  onConnection: (connected: boolean) => void,
): Unsubscribe {
  const address = `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}${url}`;
  let closed = false;
  let current: WebSocket | null = null;
  let retry = 0;
  const connect = (): void => {
    const socket = new WebSocket(address);
    current = socket;
    if (binary) {
      socket.binaryType = 'arraybuffer';
    }
    let watchdog = 0;
    const arm = (): void => {
      clearTimeout(watchdog);
      watchdog = window.setTimeout(() => socket.close(), STALE_AFTER_MS);
    };
    socket.onopen = () => {
      onConnection(true);
      arm();
    };
    socket.onmessage = (event: MessageEvent<string | ArrayBuffer>) => {
      arm();
      onMessage(event.data);
    };
    socket.onclose = () => {
      clearTimeout(watchdog);
      onConnection(false);
      if (!closed) {
        retry = window.setTimeout(connect, RECONNECT_DELAY_MS);
      }
    };
  };
  connect();
  return () => {
    closed = true;
    clearTimeout(retry);
    current?.close();
  };
}

/** Subscribes to the site state of the manager's world model; malformed messages are dropped. */
export function subscribeSite(
  siteId: string,
  onState: (state: SiteState) => void,
  onConnection: (connected: boolean) => void = () => undefined,
): Unsubscribe {
  return openSocket(
    `/sites/${encodeURIComponent(siteId)}/ws/site`,
    false,
    (data) => {
      if (typeof data !== 'string') {
        return;
      }
      const parsed: unknown = JSON.parse(data);
      if (isSiteState(parsed)) {
        onState(parsed);
      }
    },
    onConnection,
  );
}

/** Subscribes to the camera image stream. */
export function subscribeVideo(
  onFrame: (frame: VideoFrame) => void,
  onConnection: (connected: boolean) => void = () => undefined,
  base: string = siteBase,
): Unsubscribe {
  return openSocket(
    siteUrl('/ws/video', base),
    true,
    (data) => {
      if (data instanceof ArrayBuffer) {
        onFrame(parseVideoMessage(data));
      }
    },
    onConnection,
  );
}
