import type { OsmLayer, Point } from '../map/mapData';
import { angleDelta } from './interpolation';

const CELL_M = 2.5;
// Weight cap per cell: old observations fade out as new ones arrive.
const MAX_WEIGHT = 120;
const MIN_WEIGHT = 4;
// Mean resultant length below which a cell has no clear direction (two-way or turning traffic).
const MIN_COHERENCE = 0.6;
const MAX_CELLS = 30000;
const ROAD_SEARCH_M = 7;

interface Cell {
  x: number;
  y: number;
  w: number;
}

/**
 * Direction of traffic on the ground, learned from the vehicles the twin has seen: each grid cell
 * keeps the mean direction of confident, moving tracks through it. New road users take this
 * direction at once instead of turning into it, and it outlives page reloads (kept per site in
 * the browser).
 */
export class FlowField {
  private cells = new Map<string, Cell>();
  private dirty = false;

  constructor(private readonly storageKey: string | null) {
    this.load();
  }

  /** Adds an observation of a vehicle moving with velocity (vx, vy) at (x, y). */
  learn(x: number, y: number, vx: number, vy: number): void {
    const speed = Math.hypot(vx, vy);
    if (speed === 0) {
      return;
    }
    const key = this.key(x, y);
    let cell = this.cells.get(key);
    if (!cell) {
      if (this.cells.size >= MAX_CELLS) {
        return;
      }
      cell = { x: 0, y: 0, w: 0 };
      this.cells.set(key, cell);
    }
    cell.x += vx / speed;
    cell.y += vy / speed;
    cell.w += 1;
    if (cell.w > MAX_WEIGHT) {
      const k = MAX_WEIGHT / cell.w;
      cell.x *= k;
      cell.y *= k;
      cell.w = MAX_WEIGHT;
    }
    this.dirty = true;
  }

  /** Learned direction of traffic at (x, y) in radians, or null where it is not clear. */
  direction(x: number, y: number): number | null {
    const cx = Math.floor(x / CELL_M);
    const cy = Math.floor(y / CELL_M);
    let sx = 0;
    let sy = 0;
    let w = 0;
    for (let dx = -1; dx <= 1; dx++) {
      for (let dy = -1; dy <= 1; dy++) {
        const cell = this.cells.get(`${cx + dx},${cy + dy}`);
        if (cell) {
          // The centre cell counts double: lanes next to each other may run opposite ways.
          const k = dx === 0 && dy === 0 ? 2 : 1;
          sx += cell.x * k;
          sy += cell.y * k;
          w += cell.w * k;
        }
      }
    }
    if (w < MIN_WEIGHT || Math.hypot(sx, sy) / w < MIN_COHERENCE) {
      return null;
    }
    return Math.atan2(sy, sx);
  }

  /** Stores the field in the browser (called now and then). */
  save(): void {
    if (!this.dirty || !this.storageKey) {
      return;
    }
    const compact: [string, number, number, number][] = [];
    for (const [key, c] of this.cells) {
      compact.push([
        key,
        Math.round(c.x * 100) / 100,
        Math.round(c.y * 100) / 100,
        Math.round(c.w),
      ]);
    }
    try {
      localStorage.setItem(this.storageKey, JSON.stringify(compact));
      this.dirty = false;
    } catch {
      // Storage full or blocked: the field is rebuilt from the live traffic.
    }
  }

  private load(): void {
    if (!this.storageKey) {
      return;
    }
    try {
      const raw = localStorage.getItem(this.storageKey);
      for (const [key, x, y, w] of raw
        ? (JSON.parse(raw) as [string, number, number, number][])
        : []) {
        this.cells.set(key, { x, y, w });
      }
    } catch {
      this.cells.clear();
    }
  }

  private key(x: number, y: number): string {
    return `${Math.floor(x / CELL_M)},${Math.floor(y / CELL_M)}`;
  }
}

/** Axis of a road near a point. */
export interface RoadAxis {
  /** Direction of the road's centre line (of travel on a one-way road). */
  angle: number;
  oneway: boolean;
  /** Signed distance of the point from the centre line: positive on its left. */
  offset: number;
}

// Farther than this from the centre line of a two-way road, a vehicle is in a lane of one
// direction (right-hand traffic); closer, the side is not clear.
const LANE_SIDE_M = 0.75;

/**
 * Axis of the OpenStreetMap road nearest to (x, y), within a few metres: its direction, whether
 * it is one-way (then the direction is that of travel) and the side of the point.
 */
export function roadAxis(osm: OsmLayer | null, x: number, y: number): RoadAxis | null {
  if (!osm) {
    return null;
  }
  let best: RoadAxis | null = null;
  let bestDistance = ROAD_SEARCH_M;
  for (const road of osm.roads) {
    for (let i = 1; i < road.points.length; i++) {
      const a = road.points[i - 1] as Point;
      const b = road.points[i] as Point;
      const dx = b[0] - a[0];
      const dy = b[1] - a[1];
      const lengthSq = dx * dx + dy * dy;
      if (lengthSq === 0) {
        continue;
      }
      const t = Math.min(Math.max(((x - a[0]) * dx + (y - a[1]) * dy) / lengthSq, 0), 1);
      const distance = Math.hypot(a[0] + t * dx - x, a[1] + t * dy - y);
      if (distance < bestDistance) {
        bestDistance = distance;
        const side = Math.sign(dx * (y - a[1]) - dy * (x - a[0]));
        best = { angle: Math.atan2(dy, dx), oneway: road.oneway === true, offset: side * distance };
      }
    }
  }
  return best;
}

/**
 * Direction of travel of the lane a standing vehicle is in: that of a one-way road, else (right-
 * hand traffic) along the centre line on its right side and against it on its left; null where
 * the side is not clear.
 */
export function laneDirection(road: RoadAxis): number | null {
  if (road.oneway) {
    return road.angle;
  }
  if (Math.abs(road.offset) < LANE_SIDE_M) {
    return null;
  }
  return road.offset < 0 ? road.angle : road.angle + Math.PI;
}

/** The direction of an axis (angle or angle + π) closest to a reference direction. */
export function alignAxis(axis: number, reference: number): number {
  return Math.abs(angleDelta(reference, axis)) <= Math.PI / 2 ? axis : axis + Math.PI;
}
