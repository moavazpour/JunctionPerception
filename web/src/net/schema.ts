/** World-state schema version 1 (docs/architecture.md). */
export const WORLD_STATE_SCHEMA_VERSION = 1;

export type WorldSource = 'live' | 'oracle' | 'replay';
export type EventType = 'VRU_ON_CROSSWALK' | 'VRU_VEHICLE_CONFLICT';
export type EventState = 'active' | 'ended';
export type VehicleState = 'moving' | 'stopped' | 'starting';

export interface WorldObject {
  id: number;
  class: string;
  vru: boolean;
  rider: boolean;
  x: number;
  y: number;
  vx: number;
  vy: number;
  heading_rad: number | null;
  speed_mps: number;
  /** Speed uncertainty of the tracking filter; null if not estimated (e.g. ground truth). */
  speed_std_mps: number | null;
  confidence: number;
  zones: string[];
  turning: boolean;
  /** Inside the region validated for safety events; objects outside take part in none. */
  validated: boolean;
}

export interface WorldEvent {
  id: number;
  type: EventType;
  state: EventState;
  object_ids: number[];
  min_distance_m: number | null;
  time_to_closest_s: number | null;
  closing_speed_mps: number | null;
  vehicle_state: VehicleState | null;
  zone: string | null;
}

export interface WorldState {
  schema_version: number;
  scene_id: string;
  frame: number;
  timestamp_ns: number;
  source: WorldSource;
  objects: WorldObject[];
  events: WorldEvent[];
}

type FieldCheck = (value: unknown) => boolean;

const isNumber: FieldCheck = (v) => typeof v === 'number';
const isString: FieldCheck = (v) => typeof v === 'string';
const isBoolean: FieldCheck = (v) => typeof v === 'boolean';
const nullable =
  (check: FieldCheck): FieldCheck =>
  (v) =>
    v === null || check(v);
const arrayOf =
  (check: FieldCheck): FieldCheck =>
  (v) =>
    Array.isArray(v) && v.every(check);
const oneOf =
  (...values: string[]): FieldCheck =>
  (v) =>
    typeof v === 'string' && values.includes(v);

const objectFields: Record<keyof WorldObject, FieldCheck> = {
  id: isNumber,
  class: isString,
  vru: isBoolean,
  rider: isBoolean,
  x: isNumber,
  y: isNumber,
  vx: isNumber,
  vy: isNumber,
  heading_rad: nullable(isNumber),
  speed_mps: isNumber,
  speed_std_mps: nullable(isNumber),
  confidence: isNumber,
  zones: arrayOf(isString),
  turning: isBoolean,
  validated: isBoolean,
};

const eventFields: Record<keyof WorldEvent, FieldCheck> = {
  id: isNumber,
  type: oneOf('VRU_ON_CROSSWALK', 'VRU_VEHICLE_CONFLICT'),
  state: oneOf('active', 'ended'),
  object_ids: arrayOf(isNumber),
  min_distance_m: nullable(isNumber),
  time_to_closest_s: nullable(isNumber),
  closing_speed_mps: nullable(isNumber),
  vehicle_state: nullable(oneOf('moving', 'stopped', 'starting')),
  zone: nullable(isString),
};

function hasExactFields<T>(value: unknown, fields: Record<keyof T, FieldCheck>): value is T {
  if (typeof value !== 'object' || value === null || Array.isArray(value)) {
    return false;
  }
  const record = value as Record<string, unknown>;
  const expected = Object.keys(fields);
  return (
    Object.keys(record).length === expected.length &&
    expected.every((key) => key in record && fields[key as keyof T](record[key]))
  );
}

const stateFields: Record<keyof WorldState, FieldCheck> = {
  schema_version: (v) => v === WORLD_STATE_SCHEMA_VERSION,
  scene_id: isString,
  frame: isNumber,
  timestamp_ns: isNumber,
  source: oneOf('live', 'oracle', 'replay'),
  objects: arrayOf((o) => hasExactFields<WorldObject>(o, objectFields)),
  events: arrayOf((e) => hasExactFields<WorldEvent>(e, eventFields)),
};

/** Runtime check that a parsed message is a world state of the supported schema version. */
export function isWorldState(value: unknown): value is WorldState {
  return hasExactFields<WorldState>(value, stateFields);
}
