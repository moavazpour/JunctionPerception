import type { WorldObject } from '../net/schema';

// Distance within which tracks of two cameras count as the same road user, and the larger
// distance that keeps an established match (hysteresis).
const VEHICLE_GATE_M = 3.0;
const VRU_GATE_M = 1.5;
const KEEP_FACTOR = 1.6;
// Tracks moving this differently (m/s) are not the same road user.
const MAX_VELOCITY_DIFFERENCE_MPS = 4;

/** A track of one camera at its drawn (interpolated) position in the site frame. */
export interface CameraTrack {
  camera: string;
  object: WorldObject;
  x: number;
  y: number;
  heading: number;
}

/** A road user of the site: one or more camera tracks of it, merged. */
export interface FusedTrack {
  /** Stable key, the track key ("camera:id") of the camera that saw it first. */
  key: string;
  /** The most confident track, at the merged position. */
  object: WorldObject;
  x: number;
  y: number;
  heading: number;
  cameras: string[];
}

export function trackKey(camera: string, id: number): string {
  return `${camera}:${id}`;
}

/**
 * Merges the tracks of the cameras of one intersection, whose fields of view overlap: tracks of
 * different cameras that are close, of the same kind (vehicle or VRU) and move alike become one
 * road user. Matches are kept while the tracks stay a little further apart, and a merged road
 * user keeps its key while any of its tracks lives, so it neither flickers nor jumps.
 */
export class CameraFusion {
  // Track key -> key of the road user it belongs to.
  private readonly alias = new Map<string, string>();

  fuse(tracks: CameraTrack[]): FusedTrack[] {
    const keys = tracks.map((t) => trackKey(t.camera, t.object.id));
    const parent = tracks.map((_, i) => i);
    const find = (i: number): number => {
      while (parent[i] !== i) {
        i = parent[i] = parent[parent[i] as number] as number;
      }
      return i;
    };
    // Cameras in each group: a group never holds two tracks of the same camera.
    const cameras = tracks.map((t) => new Set([t.camera]));

    const pairs: { a: number; b: number; cost: number }[] = [];
    for (let a = 0; a < tracks.length; a++) {
      for (let b = a + 1; b < tracks.length; b++) {
        const ta = tracks[a] as CameraTrack;
        const tb = tracks[b] as CameraTrack;
        if (ta.camera === tb.camera || ta.object.vru !== tb.object.vru) {
          continue;
        }
        const known =
          this.alias.get(keys[a] as string) !== undefined &&
          this.alias.get(keys[a] as string) === this.alias.get(keys[b] as string);
        const gate = (ta.object.vru ? VRU_GATE_M : VEHICLE_GATE_M) * (known ? KEEP_FACTOR : 1);
        const distance = Math.hypot(ta.x - tb.x, ta.y - tb.y);
        const dv = Math.hypot(ta.object.vx - tb.object.vx, ta.object.vy - tb.object.vy);
        if (distance < gate && dv < MAX_VELOCITY_DIFFERENCE_MPS) {
          // Established matches go first.
          pairs.push({ a, b, cost: distance / gate - (known ? 1 : 0) });
        }
      }
    }
    pairs.sort((p, q) => p.cost - q.cost);
    for (const { a, b } of pairs) {
      const ra = find(a);
      const rb = find(b);
      if (ra === rb) {
        continue;
      }
      const ca = cameras[ra] as Set<string>;
      const cb = cameras[rb] as Set<string>;
      if ([...cb].some((c) => ca.has(c))) {
        continue;
      }
      parent[rb] = ra;
      for (const c of cb) {
        ca.add(c);
      }
    }

    const groups = new Map<number, number[]>();
    tracks.forEach((_, i) => {
      const root = find(i);
      groups.set(root, [...(groups.get(root) ?? []), i]);
    });
    const used = new Set<string>();
    const fused: FusedTrack[] = [];
    const present = new Set(keys);
    for (const members of groups.values()) {
      // Keep the key the group had before (that of most of its tracks), if still free.
      const votes = new Map<string, number>();
      for (const i of members) {
        const previous = this.alias.get(keys[i] as string);
        if (previous !== undefined) {
          votes.set(previous, (votes.get(previous) ?? 0) + 1);
        }
      }
      const candidates = [
        ...[...votes.entries()]
          .sort((p, q) => q[1] - p[1] || (p[0] < q[0] ? -1 : 1))
          .map(([k]) => k),
        ...members.map((i) => keys[i] as string).sort(),
      ];
      let key = candidates.find((k) => !used.has(k));
      if (key === undefined) {
        key = `${keys[members[0] as number]}#${used.size}`;
      }
      used.add(key);
      let weight = 0;
      let x = 0;
      let y = 0;
      let best = members[0] as number;
      for (const i of members) {
        const t = tracks[i] as CameraTrack;
        const w = Math.max(t.object.confidence, 0.05);
        weight += w;
        x += t.x * w;
        y += t.y * w;
        if (t.object.confidence > (tracks[best] as CameraTrack).object.confidence) {
          best = i;
        }
        this.alias.set(keys[i] as string, key);
      }
      const top = tracks[best] as CameraTrack;
      fused.push({
        key,
        object: top.object,
        x: x / weight,
        y: y / weight,
        heading: top.heading,
        cameras: members.map((i) => (tracks[i] as CameraTrack).camera),
      });
    }
    for (const k of [...this.alias.keys()]) {
      if (!present.has(k)) {
        this.alias.delete(k);
      }
    }
    return fused;
  }

  /** Key of the road user a camera track belongs to (after the latest fuse). */
  keyOf(camera: string, id: number): string {
    const key = trackKey(camera, id);
    return this.alias.get(key) ?? key;
  }
}
