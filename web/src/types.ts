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
  decel_required?: number;
  stopping?: boolean;
  queued?: boolean;
  speed_valid?: boolean;
  heading_valid?: boolean;
  heading_source?: "lane" | "motion" | "hold";
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
  compute?: ComputeInfo;
  source?: { live: boolean; loop: boolean };
  preview_clients?: number;
  calibration_confidence?: number;
  version?: { version: string; commit: string };
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
  result: SolveReport;
}

// ------------------------------------------------------------------ operator topics (panel <-> runtime)

export interface Settings {
  confidence: number;
  keypoint_confidence: number;
  max_range_m: number;
  max_fps: number;
  classes: ObjectClass[];
  roi_filter: boolean;
  preview_overlay: boolean;
  preview_fps: number;
  model: string;
  imgsz: number;
  backend: string;
  max_det: number;
  crop_to_roi: boolean;
}

export interface ComputeInfo {
  backend: "cuda" | "tensorrt" | "cpu";
  dynamic_input?: boolean;
  crop?: [number, number, number, number] | null;
  models?: string[];
  error?: string;
  device: string;
  precision: string;
  model: string;
  input: [number, number];
  available: string[];
  segmentation: boolean;
}

export interface ConfigMsg extends Header {
  site_name: string;
  settings: Settings;
  compute: ComputeInfo;
  source: { uri: string; live: boolean; loop: boolean; fps: number; width: number; height: number };
  placement: { lat: number; lon: number; height_m: number; height_sigma: number; bearing_deg: number; lens: string };
  preview: { enabled: boolean; port: number };
  calibration_mode: "manual" | "auto";
}

// Installer-drawn layout (WGS84 [lat, lon]).
export interface UserLane {
  id: string;
  name?: string;
  points: [number, number][];
  width: number;
  enabled?: boolean; // false: kept in the layout, not used (default true)
}
export interface SignalHead {
  id: string;
  box: [number, number, number, number]; // frame pixels u0 v0 u1 v1
  stop_line: number; // index into stop_lines, -1 = none
}
export interface UserMap {
  lanes: UserLane[];
  stop_lines: [number, number][][];
  crosswalks: [number, number][][];
  roi: [number, number][];
  signals?: SignalHead[];
}
export type SignalStateName = "red" | "red_amber" | "amber" | "green" | "off" | "unknown";
export interface SignalsMsg extends Header {
  signals: { id: string; state: SignalStateName; confidence: number; stop_line: number }[];
}
export interface UserMapMsg extends Header {
  map: UserMap;
}

export interface CalibPoint {
  u: number;
  v: number;
  lat: number;
  lon: number;
  align?: string; // set for alignment pairs made in the check view
}
export interface CalibLine {
  px: [number, number][];
  geo: [number, number][];
}
export interface CalibInputs {
  points: CalibPoint[];
  lines: CalibLine[];
  verticals: [number, number][][];
  height_m?: number | null;
  height_sigma?: number;
  lens?: string;
}
export interface CalibInputsMsg extends Header {
  inputs: CalibInputs;
}

export interface SolveReport {
  action: string;
  ok: boolean;
  plausible?: boolean;
  applied?: boolean;
  message?: string;
  quality?: "good" | "fair" | "poor";
  rms_px?: number;
  max_px?: number;
  ground_rms_m?: number;
  ground_max_m?: number;
  focal_px?: number;
  fov_deg?: number;
  pitch_deg?: number;
  roll_deg?: number;
  height_m?: number;
  bearing_deg?: number;
  warnings?: string[];
  residuals?: { kind: "point" | "line" | "vertical"; index: number; px: number; m: number; norm?: number }[];
  fit?: number;
  accuracy?: { range_m: number; sigma_m: number }[];
  camera?: CameraJson;
}

export interface RefineState {
  status: "idle" | "collecting" | "solving" | "proposal" | "consistent" | "rejected";
  progress: number;
  tracks: number;
  spread_deg: number;
  proposal?: {
    spread_before_deg: number;
    spread_after_deg: number;
    points_before_m: number;
    points_after_m: number;
    d_pitch_deg: number;
    d_roll_deg: number;
    d_yaw_deg: number;
    d_focal_rel: number;
    tracks: number;
    camera: CameraJson;
  };
}
export interface RefineMsg extends Header {
  refine: RefineState;
}
