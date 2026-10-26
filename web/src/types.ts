// Message schema "junction-perception/1.0" (see docs/INTERFACES.md).

export type ObjectClass = "car" | "bus" | "truck" | "trailer" | "pedestrian" | "bicycle" | "escooter" | "motorcycle";

export interface Header {
  schema: string;
  type: string;
  site: string;
  camera: string;
  ts: number;
}

export interface RoadUser {
  id: number;
  class: ObjectClass;
  state: "tracked" | "predicted" | "tentative";
  x: number;
  y: number;
  lat: number;
  lon: number;
  heading: number;
  yaw: number;
  speed: number;
  speed_kmh: number;
  accel: number;
  yaw_rate: number;
  length: number;
  width: number;
  height: number;
  sigma: { pos: number; speed: number; accel: number };
  confidence: number;
  range: number;
  age: number;
  occluded: boolean;
  lane?: string;
  approach?: string;
  lane_s?: number;
  dist_to_stop?: number;
  eta?: number;
  eta_cv?: number;
  eta_sigma?: number;
  stopping?: boolean;
  queued?: boolean;
}

export interface ObjectsMsg extends Header {
  frame: number;
  objects: RoadUser[];
}

export interface Lane {
  id: string;
  approach: string;
  to_junction: boolean;
  width: number;
  source: string;
  centerline: [number, number][];
  stop_s?: number;
}

export interface Zone {
  id: string;
  type: string;
  lane: string;
  polygon: [number, number][];
}

export interface SiteMap {
  name: string;
  lanes: Lane[];
  zones: Zone[];
  crosswalks: [number, number][][];
  roi: [number, number][];
}

export interface CameraJson {
  width: number;
  height: number;
  K: number[][];
  plane: number[];
  bearing_deg: number;
  pitch_deg: number;
  roll_deg: number;
  height_m: number;
  anchor?: { lat: number; lon: number; alt: number };
}

export interface MapMsg extends Header {
  map: SiteMap;
  camera_model: CameraJson;
}

export interface ZoneState {
  id: string;
  type: string;
  lane: string;
  occupied: boolean;
  occupancy: number;
  count: number;
  entered: number;
  mean_speed: number;
  users: number[];
}

export interface LaneState {
  id: string;
  users: number;
  queue_length: number;
  queue_count: number;
  crossings: number;
  mean_speed: number;
}

export interface DetectorsMsg extends Header {
  zones: ZoneState[];
  lanes: LaneState[];
}

export interface Health {
  level: "ok" | "degraded" | "fault";
  confidence: number;
  reasons: string[];
  calibrated: boolean;
  pipeline: { fps: number; latency_ms?: number; detector_ms: number; frames?: number; dropped?: number };
  stability?: { status: string; jitter_px: number; drift_deg: number; track_ratio: number; compensating: boolean };
  visibility?: {
    score: number;
    conditions: string[];
    brightness: number;
    contrast: number;
    sharpness_ratio: number;
    haze: number;
    far_contrast_ratio: number;
    obstructed_fraction: number;
    effective_range_m: number;
    baseline_range_m: number;
    fragmentation: number;
  };
  calibration?: {
    status: string;
    score: number;
    size_ratio: number;
    size_slope: number;
    heading_error_deg: number;
    speed_p95: number;
    samples: number;
  };
  coverage?: { zone: string; visible: number; static_occluded: number; dynamic_occluded: number; unobservable: number }[];
}

export interface HealthMsg extends Header {
  health: Health;
}

export interface NearMiss {
  event_id: number;
  kind: string;
  severity: "critical" | "warning";
  finished: boolean;
  start: number;
  end: number;
  duration: number;
  a: { id: number; class: ObjectClass; max_speed: number };
  b?: { id: number; class: ObjectClass; max_speed: number };
  x: number;
  y: number;
  lat: number;
  lon: number;
  metrics: { ttc?: number; mttc?: number; pet?: number; drac?: number; min_accel?: number };
}

export interface SafetyMsg extends Header {
  event: NearMiss;
}

export interface OcclusionMsg extends Header {
  occlusion: {
    origin: [number, number];
    cell: number;
    cols: number;
    rows: number;
    legend: string[];
    rle: number[];
    segments: { zone: string; s0: number; s1: number; cause: string }[];
  };
}

export interface CalibrationMsg extends Header {
  calibration: CameraJson & {
    source: string;
    confidence?: number;
    box_residual?: number;
    speed_drift?: number;
    tracks_used?: number;
    vanishing_point?: [number, number];
  };
}

export interface SnapshotMsg extends Header {
  width: number;
  height: number;
  scale: number;
  frame_width?: number;
  frame_height?: number;
  anchor?: { lat: number; lon: number };
  jpeg_base64: string;
}

export interface ManualResultMsg extends Header {
  result: { action: string; ok: boolean; message?: string; rms_px?: number; max_px?: number };
}
