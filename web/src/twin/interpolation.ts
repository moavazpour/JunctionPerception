import type { WorldObject, WorldState } from '../net/schema';

/** Angle difference b - a wrapped to (-pi, pi]. */
export function angleDelta(a: number, b: number): number {
  const d = (b - a) % (2 * Math.PI);
  if (d > Math.PI) {
    return d - 2 * Math.PI;
  }
  if (d <= -Math.PI) {
    return d + 2 * Math.PI;
  }
  return d;
}

/** Interpolates angles along the shorter way round. */
export function lerpAngle(a: number, b: number, t: number): number {
  return a + angleDelta(a, b) * t;
}

/** Pose of an object for rendering. */
export interface Pose {
  x: number;
  y: number;
  heading: number;
}

/**
 * Smooths world states for rendering: positions and headings are interpolated between the two
 * latest states over the time the latest one took to arrive, so the display runs one frame
 * behind and never jumps. Headings are held while the source reports none.
 */
export class WorldInterpolator {
  private previous: WorldState | null = null;
  private latest: WorldState | null = null;
  private latestArrival = 0;
  private interval = 100;
  private readonly headings = new Map<number, number>();

  push(state: WorldState, nowMs: number): void {
    if (this.latest && state.frame < this.latest.frame) {
      // A seek or loop restart: start over without interpolating across the jump.
      this.previous = null;
    } else {
      this.previous = this.latest;
    }
    if (this.latest) {
      this.interval = Math.min(Math.max(nowMs - this.latestArrival, 1), 1000);
    }
    this.latest = state;
    this.latestArrival = nowMs;
    for (const object of state.objects) {
      if (object.heading_rad !== null) {
        this.headings.set(object.id, object.heading_rad);
      }
    }
  }

  /** The latest state, or null before the first message. */
  current(): WorldState | null {
    return this.latest;
  }

  /** Interpolated pose of every object of the latest state. */
  poses(nowMs: number): Map<number, Pose> {
    const result = new Map<number, Pose>();
    if (!this.latest) {
      return result;
    }
    const t = Math.min(Math.max((nowMs - this.latestArrival) / this.interval, 0), 1);
    const before = new Map<number, WorldObject>();
    for (const object of this.previous?.objects ?? []) {
      before.set(object.id, object);
    }
    for (const object of this.latest.objects) {
      const heading = this.headings.get(object.id) ?? 0;
      const from = before.get(object.id);
      if (!from) {
        result.set(object.id, { x: object.x, y: object.y, heading });
        continue;
      }
      const fromHeading = from.heading_rad ?? heading;
      result.set(object.id, {
        x: from.x + (object.x - from.x) * t,
        y: from.y + (object.y - from.y) * t,
        heading: lerpAngle(fromHeading, heading, t),
      });
    }
    return result;
  }
}
