import { isWorldState, type WorldState } from './schema';

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
  kind: string;
  image_size_px: [number, number];
  calibrated: boolean;
  camera_ground_position_m: [number, number] | null;
  camera_height_m: number | null;
  geo_anchor: { lat_deg: number; lon_deg: number; alt_m: number } | null;
  /** URL of the map manifest, or null if the scene has no map layers. */
  map: string | null;
  zones: ZoneCollection;
  classes: ClassInfo[];
}

export interface ServerStatus {
  scene_id: string;
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

export async function fetchScene(): Promise<SceneDescription> {
  const response = await fetch('/api/scene');
  if (!response.ok) {
    throw new Error(`GET /api/scene failed: ${response.status}`);
  }
  return (await response.json()) as SceneDescription;
}

export async function fetchStatus(): Promise<ServerStatus> {
  const response = await fetch('/api/status');
  if (!response.ok) {
    throw new Error(`GET /api/status failed: ${response.status}`);
  }
  return (await response.json()) as ServerStatus;
}

export async function sendPlayback(command: PlaybackCommand): Promise<void> {
  await fetch('/api/playback', { method: 'POST', body: JSON.stringify(command) });
}

const RECONNECT_DELAY_MS = 1000;
// A connection without messages for this long is replaced; this also recovers connections to a
// restarted server that were never reported as closed. Paused playback just reconnects quietly.
const STALE_AFTER_MS = 5000;

/** WebSocket that reconnects after the connection drops or goes silent. */
function openSocket(
  path: string,
  binary: boolean,
  onMessage: (data: string | ArrayBuffer) => void,
  onConnection: (connected: boolean) => void,
): void {
  const url = `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}${path}`;
  const connect = (): void => {
    const socket = new WebSocket(url);
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
      setTimeout(connect, RECONNECT_DELAY_MS);
    };
  };
  connect();
}

/** Subscribes to the world-state stream; malformed messages are dropped. */
export function subscribeWorld(
  onState: (state: WorldState) => void,
  onConnection: (connected: boolean) => void,
): void {
  openSocket(
    '/ws/world',
    false,
    (data) => {
      if (typeof data !== 'string') {
        return;
      }
      const parsed: unknown = JSON.parse(data);
      if (isWorldState(parsed)) {
        onState(parsed);
      }
    },
    onConnection,
  );
}

/** Subscribes to the camera image stream. */
export function subscribeVideo(onFrame: (frame: VideoFrame) => void): void {
  openSocket(
    '/ws/video',
    true,
    (data) => {
      if (data instanceof ArrayBuffer) {
        onFrame(parseVideoMessage(data));
      }
    },
    () => undefined,
  );
}
