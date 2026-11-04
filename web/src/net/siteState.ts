/**
 * Site state, schema version 1: the authoritative road users of one site, produced by the site
 * world model of the manager (manager/world_model.py) with the lane traffic of its site feed
 * (manager/traffic.py), streamed on /sites/<id>/ws/site. The frontend only draws it; it never
 * fuses, re-identifies or re-estimates road users, nor counts or measures traffic.
 * Contract fixture: tests/fixtures/site_state_v1.json (tested here and in the manager).
 */
export const SITE_STATE_SCHEMA = 'site_state';
export const SITE_STATE_SCHEMA_VERSION = 1;

export type HeadingSource = 'motion' | 'keypoints' | 'held' | 'label';
export type FusionMode = 'single' | 'fused' | 'unfused';
export type SignalColor = 'red' | 'yellow' | 'red_yellow' | 'green' | 'off' | 'unknown';

export interface Observation {
  camera: string;
  /** Track ID of that camera. */
  track_id: number;
  /** Site time of the camera state the observation comes from. */
  t_ns: number;
  /** How far it was predicted to the site time (ms). */
  dt_ms: number;
}

export interface RoadUser {
  /** Global ID, unique within a run of the site. */
  id: number;
  class: string;
  vru: boolean;
  rider: boolean;
  confidence: number;
  /** "tracked": seen by a camera now; "coasting": no camera sees it, position predicted. */
  status: 'tracked' | 'coasting';
  /** Position in the site frame (m) and its covariance xx, xy, yy (m²). */
  x: number;
  y: number;
  position_cov_m2: [number, number, number];
  vx: number;
  vy: number;
  speed_mps: number;
  speed_std_mps: number | null;
  /** Counter-clockwise from +x; null while unknown (never guessed). */
  heading_rad: number | null;
  heading_source: HeadingSource | null;
  length_m: number | null;
  width_m: number | null;
  /** "measured" (keypoint footprint) or "class" (class size); null without a size. */
  size_source: 'measured' | 'class' | null;
  validated: boolean;
  first_seen_ns: number;
  last_observed_ns: number;
  observed_by: Observation[];
}

export interface SiteCamera {
  id: string;
  /** "ok": used in this state; "stale": too far behind the others; "offline": no state yet. */
  state: 'ok' | 'stale' | 'offline';
  time_source: 'media' | 'capture' | 'arrival' | null;
  offset_ms: number;
  last_t_ns: number | null;
  lag_ms: number | null;
  frame: number | null;
  objects: number | null;
}

export interface SiteSignal {
  camera: string;
  id: string;
  name: string;
  state: SignalColor;
  confidence: number;
  since_ns: number;
}

/** Measures of a lane a camera outlines (manager/traffic.py), on the global road-user IDs. */
export interface LaneTraffic {
  /** "<camera>/<line id>". */
  id: string;
  camera: string;
  name: string;
  /** Without a direction of travel set by the user, nothing is counted as passing. */
  direction_set: boolean;
  /** Vehicles in the lane now, and those of them slower than the queue speed. */
  vehicles: number;
  queue: number;
  /** Vehicles that drove through the lane in its direction since `since_ns`, and by class. */
  passed: number;
  passed_by_class: Record<string, number>;
}

/** Lane traffic of the site, added to the state by the manager's site feed. */
export interface SiteTraffic {
  /** Site time the counts started at: the start of this run. */
  since_ns: number;
  present_by_class: Record<string, number>;
  lanes: LaneTraffic[];
}

export interface SiteState {
  schema: typeof SITE_STATE_SCHEMA;
  schema_version: number;
  site_id: string;
  seq: number;
  /** Changes on a seek, loop or restart: road-user IDs start over. */
  run: number;
  t_ns: number;
  clock: 'media' | 'unix' | 'mixed';
  coordinate_frame: 'site';
  fusion: { mode: FusionMode; reason: string | null };
  cameras: SiteCamera[];
  road_users: RoadUser[];
  signals: SiteSignal[];
  traffic: SiteTraffic;
}

type Check = (value: unknown) => boolean;

const isNumber: Check = (v) => typeof v === 'number';
const isString: Check = (v) => typeof v === 'string';
const isBoolean: Check = (v) => typeof v === 'boolean';
const nullable =
  (check: Check): Check =>
  (v) =>
    v === null || check(v);
const oneOf =
  (...values: unknown[]): Check =>
  (v) =>
    values.includes(v);
const arrayOf =
  (check: Check): Check =>
  (v) =>
    Array.isArray(v) && v.every(check);
const triple: Check = (v) => Array.isArray(v) && v.length === 3 && v.every(isNumber);
const counts: Check = (v) =>
  typeof v === 'object' &&
  v !== null &&
  !Array.isArray(v) &&
  Object.values(v).every((n) => Number.isInteger(n) && (n as number) >= 0);

/** Exactly these fields, each passing its check. */
function record<T>(fields: Record<keyof T, Check>): Check {
  return (value) => {
    if (typeof value !== 'object' || value === null || Array.isArray(value)) {
      return false;
    }
    const r = value as Record<string, unknown>;
    const keys = Object.keys(fields);
    return (
      Object.keys(r).length === keys.length &&
      keys.every((k) => k in r && fields[k as keyof T](r[k]))
    );
  };
}

const observation = record<Observation>({
  camera: isString,
  track_id: isNumber,
  t_ns: isNumber,
  dt_ms: isNumber,
});

const roadUser = record<RoadUser>({
  id: isNumber,
  class: isString,
  vru: isBoolean,
  rider: isBoolean,
  confidence: isNumber,
  status: oneOf('tracked', 'coasting'),
  x: isNumber,
  y: isNumber,
  position_cov_m2: triple,
  vx: isNumber,
  vy: isNumber,
  speed_mps: isNumber,
  speed_std_mps: nullable(isNumber),
  heading_rad: nullable(isNumber),
  heading_source: nullable(oneOf('motion', 'keypoints', 'held', 'label')),
  length_m: nullable(isNumber),
  width_m: nullable(isNumber),
  size_source: oneOf('measured', 'class', null),
  validated: isBoolean,
  first_seen_ns: isNumber,
  last_observed_ns: isNumber,
  observed_by: arrayOf(observation),
});

const siteCamera = record<SiteCamera>({
  id: isString,
  state: oneOf('ok', 'stale', 'offline'),
  time_source: oneOf('media', 'capture', 'arrival', null),
  offset_ms: isNumber,
  last_t_ns: nullable(isNumber),
  lag_ms: nullable(isNumber),
  frame: nullable(isNumber),
  objects: nullable(isNumber),
});

const siteSignal = record<SiteSignal>({
  camera: isString,
  id: isString,
  name: isString,
  state: oneOf('red', 'yellow', 'red_yellow', 'green', 'off', 'unknown'),
  confidence: isNumber,
  since_ns: isNumber,
});

const laneTraffic = record<LaneTraffic>({
  id: isString,
  camera: isString,
  name: isString,
  direction_set: isBoolean,
  vehicles: isNumber,
  queue: isNumber,
  passed: isNumber,
  passed_by_class: counts,
});

const siteState = record<SiteState>({
  schema: oneOf(SITE_STATE_SCHEMA),
  schema_version: oneOf(SITE_STATE_SCHEMA_VERSION),
  site_id: isString,
  seq: isNumber,
  run: isNumber,
  t_ns: isNumber,
  clock: oneOf('media', 'unix', 'mixed'),
  coordinate_frame: oneOf('site'),
  fusion: record<SiteState['fusion']>({
    mode: oneOf('single', 'fused', 'unfused'),
    reason: nullable(isString),
  }),
  cameras: arrayOf(siteCamera),
  road_users: arrayOf(roadUser),
  signals: arrayOf(siteSignal),
  traffic: record<SiteTraffic>({
    since_ns: isNumber,
    present_by_class: counts,
    lanes: arrayOf(laneTraffic),
  }),
});

/** Runtime check that a parsed message is a site state of the supported version. */
export function isSiteState(value: unknown): value is SiteState {
  return siteState(value);
}
