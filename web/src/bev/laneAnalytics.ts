import type { ZoneCollection } from '../net/client';
import type { BevTrack } from './bevModel';
import {
  along,
  angleBetween,
  pointInPolygon,
  polygonAxis,
  type PolygonAxis,
  type Vec,
} from './geometry';

/**
 * Live state of the lanes, the intersection and the crosswalks of a site, for the operator of
 * the signals. Every number says what it rests on:
 * - measured: counted from tracks (vehicles in a lane, stopped vehicles, VRUs on a crosswalk);
 * - estimate: derived with assumptions (queue length, waiting time, arrivals, occupancy);
 * - a lane direction is known only from observed traffic or a one-way OpenStreetMap road, never
 *   guessed; without it there is no queue (no downstream end to measure it from).
 */

// Lane direction from observed traffic: tracks that moved clearly along the lane.
const DIRECTION_MIN_SPEED_MPS = 2.5;
const DIRECTION_MAX_STD_MPS = 0.8;
const DIRECTION_MAX_ANGLE = Math.PI / 6;
const DIRECTION_MIN_TRACKS = 5;
const DIRECTION_MIN_AGREEMENT = 0.8;
// A one-way road counts for a lane whose axis it follows within this angle.
const OSM_MAX_ANGLE = (25 * Math.PI) / 180;
// Queue: the first stopped vehicle within this distance of the lane end, then gaps of at most
// this much between stopped vehicles (rear of one to front of the next).
const QUEUE_FIRST_GAP_M = 12;
const QUEUE_JAM_GAP_M = 10;
// Arrivals are counted over this window of camera time.
const ARRIVAL_WINDOW_S = 60;
// A vehicle standing this long inside the intersection blocks it.
const BLOCKING_S = 5;

const NS = 1e9;

export type DirectionSource = 'observed' | 'osm';

export interface LaneDirection {
  /** Direction of travel (rad, counter-clockwise from +x). */
  angle: number;
  source: DirectionSource;
  /** Observed: tracks that agree, out of all that moved along the lane. */
  agree: number;
  total: number;
}

export type LaneState = 'empty' | 'flowing' | 'queue' | 'unknown';

export interface LaneStatus {
  id: string;
  ring: Vec[];
  axis: PolygonAxis | null;
  direction: LaneDirection | null;
  /** Measured. */
  vehicles: number;
  stopped: number;
  /** Vehicles whose speed is too uncertain to say whether they stand. */
  uncertain: number;
  meanSpeedKmh: number | null;
  /** Estimates (null where the lane direction is not known). */
  queueM: number | null;
  queueVehicles: number;
  maxWaitS: number | null;
  /** Share of the lane length covered by vehicle footprints. */
  occupancy: number;
  /** Vehicles that entered the lane per minute, and the camera time that rate covers. */
  arrivalsPerMin: number | null;
  arrivalWindowS: number;
  state: LaneState;
  /** Queue line on the map: from the lane end upstream (site frame). */
  queueLine: [Vec, Vec] | null;
}

export interface AreaStatus {
  id: string;
  type: 'intersection' | 'crosswalk';
  ring: Vec[];
  vehicles: number;
  stoppedVehicles: number;
  vrus: number;
  /** Longest a vehicle has stood inside (s); intersection blocking above BLOCKING_S. */
  longestStopS: number | null;
  blocked: boolean;
}

export interface SiteStatus {
  lanes: LaneStatus[];
  areas: AreaStatus[];
}

/** Axis of the nearest OSM road at a point (site frame), as roadAxis in flowField. */
export type RoadLookup = (x: number, y: number) => { angle: number; oneway: boolean } | null;

interface LaneMemory {
  forward: Set<string>;
  backward: Set<string>;
  /** Track key -> camera time it entered the lane. */
  entered: Map<string, number>;
  inside: Set<string>;
  firstNs: number | null;
  /** Direction counts of earlier visits (forward, backward). */
  stored: [number, number];
}

interface Zone {
  id: string;
  type: string;
  ring: Vec[];
  axis: PolygonAxis | null;
}

function axisAngle(axis: PolygonAxis): number {
  return Math.atan2(axis.uy, axis.ux);
}

/** Lane and area status of one site, updated with the tracks of every new world state. */
export class LaneAnalytics {
  private readonly zones: Zone[];
  private readonly memory = new Map<string, LaneMemory>();
  private road: RoadLookup = () => null;
  private latest: SiteStatus = { lanes: [], areas: [] };

  constructor(
    zones: ZoneCollection,
    private readonly storageKey: string | null = null,
  ) {
    this.zones = zones.features.map((f) => {
      const ring = (f.geometry.coordinates[0] ?? []).map(([x = 0, y = 0]): Vec => [x, y]);
      return { id: f.properties.id, type: f.properties.type, ring, axis: polygonAxis(ring) };
    });
    this.load();
  }

  setRoadLookup(road: RoadLookup): void {
    this.road = road;
  }

  status(): SiteStatus {
    return this.latest;
  }

  /** Direction of a lane, if known reliably. */
  direction(id: string): LaneDirection | null {
    const zone = this.zones.find((z) => z.id === id && z.type === 'lane');
    return zone ? this.laneDirection(zone) : null;
  }

  /** Reliable direction of the lane containing a point, for vehicles without heading. */
  directionAt(x: number, y: number): { heading: number; lane: string; source: string } | null {
    for (const zone of this.zones) {
      if (zone.type === 'lane' && pointInPolygon(x, y, zone.ring)) {
        const direction = this.laneDirection(zone);
        return direction
          ? { heading: direction.angle, lane: zone.id, source: direction.source }
          : null;
      }
    }
    return null;
  }

  /** Takes the (deduplicated) tracks of the site at camera time `nowNs`. */
  update(tracks: BevTrack[], nowNs: number): SiteStatus {
    const lanes: LaneStatus[] = [];
    const areas: AreaStatus[] = [];
    for (const zone of this.zones) {
      const inside = tracks.filter((t) => pointInPolygon(t.object.x, t.object.y, zone.ring));
      if (zone.type === 'lane') {
        lanes.push(
          this.updateLane(
            zone,
            inside.filter((t) => !t.object.vru),
            nowNs,
          ),
        );
      } else if (zone.type === 'intersection' || zone.type === 'crosswalk') {
        const cars = inside.filter((t) => !t.object.vru);
        const stops = cars.map((t) => t.stoppedS).filter((s): s is number => s !== null);
        const longestStopS = stops.length > 0 ? Math.max(...stops) : null;
        areas.push({
          id: zone.id,
          type: zone.type,
          ring: zone.ring,
          vehicles: cars.length,
          stoppedVehicles: cars.filter((t) => t.motion === 'stopped').length,
          vrus: inside.length - cars.length,
          longestStopS,
          blocked:
            zone.type === 'intersection' && longestStopS !== null && longestStopS >= BLOCKING_S,
        });
      }
    }
    this.latest = { lanes, areas };
    return this.latest;
  }

  private laneMemory(id: string): LaneMemory {
    let m = this.memory.get(id);
    if (!m) {
      m = {
        forward: new Set(),
        backward: new Set(),
        entered: new Map(),
        inside: new Set(),
        firstNs: null,
        stored: [0, 0],
      };
      this.memory.set(id, m);
    }
    return m;
  }

  private laneDirection(zone: Zone): LaneDirection | null {
    if (!zone.axis) {
      return null;
    }
    const m = this.laneMemory(zone.id);
    const forward = m.forward.size + m.stored[0];
    const backward = m.backward.size + m.stored[1];
    const total = forward + backward;
    const agree = Math.max(forward, backward);
    const base = axisAngle(zone.axis);
    if (total >= DIRECTION_MIN_TRACKS && agree / total >= DIRECTION_MIN_AGREEMENT) {
      const angle = forward >= backward ? base : base + Math.PI;
      return { angle, source: 'observed', agree, total };
    }
    const road = this.road(zone.axis.cx, zone.axis.cy);
    if (road?.oneway) {
      const delta = angleBetween(road.angle, base);
      if (delta < OSM_MAX_ANGLE || delta > Math.PI - OSM_MAX_ANGLE) {
        return { angle: road.angle, source: 'osm', agree, total };
      }
    }
    return null;
  }

  private updateLane(zone: Zone, inside: BevTrack[], nowNs: number): LaneStatus {
    const m = this.laneMemory(zone.id);
    m.firstNs ??= nowNs;
    const axis = zone.axis;
    // Learn the direction from tracks that clearly move along the lane.
    if (axis) {
      const base = axisAngle(axis);
      for (const t of inside) {
        const o = t.object;
        if (
          o.speed_mps > DIRECTION_MIN_SPEED_MPS &&
          o.speed_std_mps !== null &&
          o.speed_std_mps < DIRECTION_MAX_STD_MPS
        ) {
          const motion = Math.atan2(o.vy, o.vx);
          const delta = angleBetween(motion, base);
          if (delta < DIRECTION_MAX_ANGLE) {
            m.forward.add(t.key);
            m.backward.delete(t.key);
          } else if (delta > Math.PI - DIRECTION_MAX_ANGLE) {
            m.backward.add(t.key);
            m.forward.delete(t.key);
          }
        }
      }
      this.save();
    }
    // Arrivals: tracks entering the lane, within the window.
    const now = new Set(inside.map((t) => t.key));
    for (const key of now) {
      if (!m.inside.has(key) && !m.entered.has(key)) {
        m.entered.set(key, nowNs);
      }
    }
    m.inside = now;
    for (const [key, t] of m.entered) {
      if ((nowNs - t) / NS > ARRIVAL_WINDOW_S) {
        m.entered.delete(key);
      }
    }
    const windowS = Math.min(ARRIVAL_WINDOW_S, (nowNs - m.firstNs) / NS);

    const stopped = inside.filter((t) => t.motion === 'stopped');
    const uncertain = inside.filter((t) => t.motion === 'uncertain').length;
    const speeds = inside.filter((t) => t.motion !== 'uncertain').map((t) => t.object.speed_mps);
    const direction = this.laneDirection(zone);
    const lengthM = axis ? axis.max - axis.min : 0;
    const occupied = inside.reduce((s, t) => s + t.length, 0);

    let queueM: number | null = null;
    let queueVehicles = 0;
    let maxWaitS: number | null = null;
    let queueLine: [Vec, Vec] | null = null;
    if (axis && direction) {
      const queue = measureQueue(axis, direction.angle, stopped);
      queueM = queue.lengthM;
      queueVehicles = queue.members.length;
      const waits = queue.members.map((t) => t.stoppedS).filter((s): s is number => s !== null);
      maxWaitS = waits.length > 0 ? Math.max(...waits) : null;
      if (queueM > 0) {
        const sign = Math.cos(direction.angle - axisAngle(axis)) >= 0 ? 1 : -1;
        const end = sign > 0 ? axis.max : axis.min;
        const start = end - sign * queueM;
        const at = (d: number): Vec => [axis.cx + axis.ux * d, axis.cy + axis.uy * d];
        queueLine = [at(end), at(start)];
      }
    }
    let state: LaneState = 'empty';
    if (queueVehicles > 0) {
      state = 'queue';
    } else if (stopped.length > 0 && !direction) {
      state = 'unknown';
    } else if (inside.length > 0) {
      state = 'flowing';
    }
    return {
      id: zone.id,
      ring: zone.ring,
      axis,
      direction,
      vehicles: inside.length,
      stopped: stopped.length,
      uncertain,
      meanSpeedKmh:
        speeds.length > 0 ? (speeds.reduce((s, v) => s + v, 0) / speeds.length) * 3.6 : null,
      queueM,
      queueVehicles,
      maxWaitS,
      occupancy: lengthM > 0 ? Math.min(occupied / lengthM, 1) : 0,
      arrivalsPerMin: windowS >= 10 ? (m.entered.size / windowS) * 60 : null,
      arrivalWindowS: windowS,
      state,
      queueLine,
    };
  }

  private save(): void {
    if (!this.storageKey) {
      return;
    }
    try {
      const data = Object.fromEntries(
        [...this.memory].map(([id, m]) => [
          id,
          [m.forward.size + m.stored[0], m.backward.size + m.stored[1]],
        ]),
      );
      localStorage.setItem(this.storageKey, JSON.stringify(data));
    } catch {
      // Blocked storage: the directions are learned again from live traffic.
    }
  }

  /** Direction counts of earlier visits of this browser. */
  private load(): void {
    if (!this.storageKey) {
      return;
    }
    try {
      const raw = localStorage.getItem(this.storageKey);
      const data = raw ? (JSON.parse(raw) as Record<string, [number, number]>) : {};
      for (const [id, [forward, backward]] of Object.entries(data)) {
        this.laneMemory(id).stored = [forward, backward];
      }
    } catch {
      this.memory.clear();
    }
  }
}

/**
 * Queue of a lane: stopped vehicles chained from the downstream lane end against the direction
 * of travel. The first must stand near the end, each next one close behind the previous one.
 * Its length runs from the lane end to the rear of the last vehicle.
 */
export function measureQueue(
  axis: PolygonAxis,
  directionAngle: number,
  stopped: BevTrack[],
): { lengthM: number; members: BevTrack[] } {
  const sign = Math.cos(directionAngle - Math.atan2(axis.uy, axis.ux)) >= 0 ? 1 : -1;
  const end = sign > 0 ? axis.max : -axis.min;
  const placed = stopped
    .map((t) => {
      const d = sign * along(axis, t.object.x, t.object.y);
      const front = end - (d + t.length / 2);
      return { t, front, rear: front + t.length };
    })
    .sort((a, b) => a.front - b.front);
  const members: BevTrack[] = [];
  let rear = 0;
  for (const p of placed) {
    const gap = members.length === 0 ? QUEUE_FIRST_GAP_M : QUEUE_JAM_GAP_M;
    if (p.front - rear > gap) {
      break;
    }
    members.push(p.t);
    rear = Math.max(rear, p.rear);
  }
  return { lengthM: members.length > 0 ? rear : 0, members };
}
