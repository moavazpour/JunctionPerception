import type { ClassInfo } from '../net/client';
import type { WorldObject, WorldState } from '../net/schema';
import { angleBetween } from './geometry';

/**
 * How trustworthy a drawn heading is:
 * - measured: from this track's motion, its box corners, or a label;
 * - held: the last measured heading, kept while the road user is too slow to measure it;
 * - lane: no heading of its own; the direction of the lane it stands in (observed or OSM);
 * - unknown: no heading at all. It is never replaced by 0°.
 */
export type HeadingStatus = 'measured' | 'held' | 'lane' | 'unknown';

/** Motion of a track from its filtered speed and the speed's uncertainty. */
export type Motion = 'moving' | 'stopped' | 'uncertain';

// Below this speed a road user counts as stopped (5 km/h, the usual queue criterion).
export const STOPPED_SPEED_MPS = 1.4;
// Speed estimates less certain than this say nothing about stopping (new or re-anchored tracks).
export const MAX_SPEED_STD_MPS = 1.0;
// A heading reported without its source counts as measured from motion when the track moves
// clearly in that direction.
const INFERRED_MOTION_MPS = 2;
const INFERRED_MAX_ANGLE = Math.PI / 6;
// Raw trajectories: how long they are kept, and when a track not seen any more is forgotten.
const TRAIL_S = 30;
const FORGET_S = 5;
// A new track this close to one lost this recently, on the same camera, is a likely ID switch.
const SWITCH_DISTANCE_M = 2.5;
const SWITCH_WINDOW_S = 1.0;
// Tracks of two cameras this close (vehicles, VRUs) are candidates for the same road user.
const DUPLICATE_GATE_M = { vehicle: 3.0, vru: 1.5 };
const MAX_DUPLICATE_DV_MPS = 4;

const NS = 1e9;

export interface TrailPoint {
  x: number;
  y: number;
  t: number;
}

/** A track of one camera as the BEV draws it: raw position, no smoothing or interpolation. */
export interface BevTrack {
  /** "camera:id" within its site. */
  key: string;
  site: string;
  camera: string;
  object: WorldObject;
  frame: number;
  timestampNs: number;
  /** Seconds since the track first appeared (camera time). */
  ageS: number;
  heading: number | null;
  headingStatus: HeadingStatus;
  /** Where the heading comes from, in words. */
  headingNote: string;
  length: number;
  width: number;
  /** Footprint measured from the box corners (else the class size). */
  sizeMeasured: boolean;
  motion: Motion;
  /** Camera time since the track stands (null while it moves or its motion is uncertain). */
  stoppedS: number | null;
  trail: TrailPoint[];
  /** The track appeared where another one of its camera was just lost. */
  possibleIdSwitch: boolean;
}

/** Two tracks of different cameras that are probably the same road user. */
export interface DuplicatePair {
  a: BevTrack;
  b: BevTrack;
  distanceM: number;
  /** Difference of the two cameras' frame timestamps (s). */
  dtS: number;
}

/** Lane direction lookup for vehicles without heading (only reliable lane directions). */
export type LaneHeading = (
  site: string,
  x: number,
  y: number,
) => { heading: number; lane: string; source: string } | null;

interface Memory {
  firstNs: number;
  lastNs: number;
  stoppedSinceNs: number | null;
  trail: TrailPoint[];
  switched: boolean;
  /** Not in the latest state (already listed as lost). */
  lost: boolean;
  last: { x: number; y: number; vru: boolean };
}

/** Quality counters of one camera feed. */
export interface FeedQuality {
  camera: string;
  frame: number;
  timestampNs: number;
  objects: number;
  noHeading: number;
  classSize: number;
  idSwitches: number;
  /** Tracks started so far (to put ID switches into proportion). */
  tracksStarted: number;
}

class Feed {
  latest: WorldState | null = null;
  memory = new Map<number, Memory>();
  lost: { id: number; x: number; y: number; vru: boolean; t: number }[] = [];
  idSwitches = 0;
  tracksStarted = 0;

  push(state: WorldState): void {
    if (this.latest && state.frame < this.latest.frame) {
      // Seek or loop: the camera's history no longer applies.
      this.memory.clear();
      this.lost = [];
    }
    this.latest = state;
    const t = state.timestamp_ns;
    // Tracks that ended since the last state first, so a new track can be matched to them.
    const present = new Set(state.objects.map((o) => o.id));
    for (const [id, m] of this.memory) {
      if (present.has(id)) {
        continue;
      }
      if (!m.lost) {
        m.lost = true;
        this.lost.push({ id, ...m.last, t: m.lastNs });
      }
      if ((t - m.lastNs) / NS > FORGET_S) {
        this.memory.delete(id);
      }
    }
    this.lost = this.lost.filter((l) => (t - l.t) / NS <= SWITCH_WINDOW_S && !present.has(l.id));
    for (const o of state.objects) {
      let m = this.memory.get(o.id);
      if (!m) {
        // Each lost track explains at most one new track.
        const index = this.lost.findIndex(
          (l) => l.vru === o.vru && Math.hypot(l.x - o.x, l.y - o.y) <= SWITCH_DISTANCE_M,
        );
        const switched = index >= 0;
        if (switched) {
          this.lost.splice(index, 1);
          this.idSwitches++;
        }
        this.tracksStarted++;
        m = {
          firstNs: t,
          lastNs: t,
          stoppedSinceNs: null,
          trail: [],
          switched,
          lost: false,
          last: { x: o.x, y: o.y, vru: o.vru },
        };
        this.memory.set(o.id, m);
      }
      m.lost = false;
      m.lastNs = t;
      m.last = { x: o.x, y: o.y, vru: o.vru };
      m.trail.push({ x: o.x, y: o.y, t });
      while (m.trail.length > 0 && t - (m.trail[0] as TrailPoint).t > TRAIL_S * NS) {
        m.trail.shift();
      }
      const motion = motionOf(o);
      if (motion === 'stopped') {
        m.stoppedSinceNs ??= t;
      } else if (motion === 'moving') {
        m.stoppedSinceNs = null;
      }
    }
  }
}

/** Motion of a road user: an uncertain speed says nothing, whatever its value. */
export function motionOf(o: WorldObject): Motion {
  if (o.speed_std_mps !== null && o.speed_std_mps > MAX_SPEED_STD_MPS) {
    return 'uncertain';
  }
  return o.speed_mps < STOPPED_SPEED_MPS ? 'stopped' : 'moving';
}

/** Heading status of an object from what the source reports (and, for older sources, infers). */
export function headingOf(o: WorldObject): {
  heading: number | null;
  status: HeadingStatus;
  note: string;
} {
  if (o.heading_rad === null) {
    return { heading: null, status: 'unknown', note: 'not measured yet' };
  }
  switch (o.heading_source) {
    case 'motion':
      return { heading: o.heading_rad, status: 'measured', note: 'direction of travel' };
    case 'keypoints':
      return { heading: o.heading_rad, status: 'measured', note: 'box corners' };
    case 'label':
      return { heading: o.heading_rad, status: 'measured', note: 'ground-truth label' };
    case 'held':
      return { heading: o.heading_rad, status: 'held', note: 'last measured, held while slow' };
    default: {
      // Older servers do not say: trust it only where the track clearly moves that way.
      const moving =
        o.speed_mps > INFERRED_MOTION_MPS &&
        (o.speed_std_mps === null || o.speed_std_mps < MAX_SPEED_STD_MPS) &&
        angleBetween(Math.atan2(o.vy, o.vx), o.heading_rad) < INFERRED_MAX_ANGLE;
      return moving
        ? { heading: o.heading_rad, status: 'measured', note: 'agrees with motion (source n/a)' }
        : { heading: o.heading_rad, status: 'held', note: 'source not reported' };
    }
  }
}

/**
 * The world as the bird's-eye view shows it: the latest world state of every camera, as sent,
 * with raw trajectories and the quality of each feed. Nothing is interpolated, smoothed or
 * oriented by the frontend; vehicles without heading take a lane direction only where the lane
 * analysis knows it reliably, and are marked so.
 */
export class BevWorld {
  private readonly feeds = new Map<string, Map<string, Feed>>();
  private readonly classes = new Map<string, ClassInfo>();
  laneHeading: LaneHeading = () => null;

  setClasses(classes: ClassInfo[]): void {
    for (const c of classes) {
      this.classes.set(c.name, c);
    }
  }

  push(site: string, camera: string, state: WorldState): void {
    let cameras = this.feeds.get(site);
    if (!cameras) {
      cameras = new Map();
      this.feeds.set(site, cameras);
    }
    let feed = cameras.get(camera);
    if (!feed) {
      feed = new Feed();
      cameras.set(camera, feed);
    }
    feed.push(state);
  }

  /** Forgets a camera (stopped, or its stream went stale). */
  drop(site: string, camera: string): void {
    this.feeds.get(site)?.delete(camera);
  }

  states(site: string): { camera: string; state: WorldState }[] {
    return [...(this.feeds.get(site) ?? new Map<string, Feed>()).entries()]
      .filter(([, f]) => f.latest)
      .map(([camera, f]) => ({ camera, state: f.latest as WorldState }));
  }

  sites(): string[] {
    return [...this.feeds.keys()];
  }

  /** Tracks of a site's cameras, as reported in each camera's latest state. */
  tracks(site: string, cameras?: Set<string>): BevTrack[] {
    const result: BevTrack[] = [];
    for (const [camera, feed] of this.feeds.get(site) ?? []) {
      if (!feed.latest || (cameras && !cameras.has(camera))) {
        continue;
      }
      const t = feed.latest.timestamp_ns;
      for (const o of feed.latest.objects) {
        const m = feed.memory.get(o.id);
        const info = this.classes.get(o.class);
        let { heading, status, note } = headingOf(o);
        if (heading === null && !o.vru) {
          const lane = this.laneHeading(site, o.x, o.y);
          if (lane) {
            heading = lane.heading;
            status = 'lane';
            note = `direction of ${lane.lane} (${lane.source})`;
          }
        }
        const sizeMeasured = Boolean(o.length_m && o.width_m);
        const motion = motionOf(o);
        result.push({
          key: `${camera}:${o.id}`,
          site,
          camera,
          object: o,
          frame: feed.latest.frame,
          timestampNs: t,
          ageS: m ? (t - m.firstNs) / NS : 0,
          heading,
          headingStatus: status,
          headingNote: note,
          length: sizeMeasured ? (o.length_m as number) : (info?.length_m ?? 2),
          width: sizeMeasured ? (o.width_m as number) : (info?.width_m ?? 1),
          sizeMeasured,
          motion,
          stoppedS:
            motion === 'stopped' && m?.stoppedSinceNs != null ? (t - m.stoppedSinceNs) / NS : null,
          trail: m?.trail ?? [],
          possibleIdSwitch: m?.switched ?? false,
        });
      }
    }
    return result;
  }

  quality(site: string): FeedQuality[] {
    return [...(this.feeds.get(site) ?? [])]
      .filter(([, f]) => f.latest)
      .map(([camera, f]) => {
        const objects = (f.latest as WorldState).objects;
        return {
          camera,
          frame: (f.latest as WorldState).frame,
          timestampNs: (f.latest as WorldState).timestamp_ns,
          objects: objects.length,
          noHeading: objects.filter((o) => o.heading_rad === null).length,
          classSize: objects.filter((o) => !(o.length_m && o.width_m)).length,
          idSwitches: f.idSwitches,
          tracksStarted: f.tracksStarted,
        };
      });
  }
}

/**
 * Pairs of tracks of different cameras that are close, of the same kind and move alike: candidates
 * for one road user seen twice. Each track pairs with its nearest candidate only.
 */
export function duplicateCandidates(tracks: BevTrack[]): DuplicatePair[] {
  const pairs: DuplicatePair[] = [];
  for (let i = 0; i < tracks.length; i++) {
    for (let j = i + 1; j < tracks.length; j++) {
      const a = tracks[i] as BevTrack;
      const b = tracks[j] as BevTrack;
      if (a.camera === b.camera || a.object.vru !== b.object.vru) {
        continue;
      }
      const gate = a.object.vru ? DUPLICATE_GATE_M.vru : DUPLICATE_GATE_M.vehicle;
      const distanceM = Math.hypot(a.object.x - b.object.x, a.object.y - b.object.y);
      const dv = Math.hypot(a.object.vx - b.object.vx, a.object.vy - b.object.vy);
      if (distanceM < gate && dv < MAX_DUPLICATE_DV_MPS) {
        pairs.push({ a, b, distanceM, dtS: (a.timestampNs - b.timestampNs) / NS });
      }
    }
  }
  pairs.sort((p, q) => p.distanceM - q.distanceM);
  const used = new Set<string>();
  return pairs.filter((p) => {
    if (used.has(p.a.key) || used.has(p.b.key)) {
      return false;
    }
    used.add(p.a.key);
    used.add(p.b.key);
    return true;
  });
}
