/** Plane geometry of the bird's-eye view, in metres of a site's world frame (x east, y north). */

export type Vec = [number, number];

/** Corners of a footprint rectangle centred at (x, y), its length along `heading`. */
export function footprintCorners(
  x: number,
  y: number,
  heading: number,
  length: number,
  width: number,
): Vec[] {
  const c = Math.cos(heading);
  const s = Math.sin(heading);
  const l = length / 2;
  const w = width / 2;
  return [
    [x + c * l - s * w, y + s * l + c * w],
    [x + c * l + s * w, y + s * l - c * w],
    [x - c * l + s * w, y - s * l - c * w],
    [x - c * l - s * w, y - s * l + c * w],
  ];
}

/** Ellipse of a 2 x 2 covariance (xx, xy, yy) at `sigma` standard deviations. */
export interface Ellipse {
  /** Semi-axes (m), major first. */
  major: number;
  minor: number;
  /** Direction of the major axis (rad, counter-clockwise from +x). */
  angle: number;
}

export function covarianceEllipse(cov: [number, number, number], sigma = 2): Ellipse | null {
  const [a, b, d] = cov;
  if (![a, b, d].every(Number.isFinite) || a < 0 || d < 0) {
    return null;
  }
  const mean = (a + d) / 2;
  const root = Math.sqrt(((a - d) / 2) ** 2 + b * b);
  const l1 = mean + root;
  const l2 = Math.max(mean - root, 0);
  return {
    major: sigma * Math.sqrt(l1),
    minor: sigma * Math.sqrt(l2),
    angle: 0.5 * Math.atan2(2 * b, a - d),
  };
}

/** Ray casting: whether (x, y) lies inside the polygon. */
export function pointInPolygon(x: number, y: number, ring: Vec[]): boolean {
  let inside = false;
  for (let i = 0, j = ring.length - 1; i < ring.length; j = i++) {
    const [xi, yi] = ring[i] as Vec;
    const [xj, yj] = ring[j] as Vec;
    if (yi > y !== yj > y && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi) {
      inside = !inside;
    }
  }
  return inside;
}

/** Signed area (shoelace; positive counter-clockwise) and centroid of a polygon. */
export function polygonCentroid(ring: Vec[]): { area: number; x: number; y: number } {
  let area = 0;
  let cx = 0;
  let cy = 0;
  for (let i = 0, j = ring.length - 1; i < ring.length; j = i++) {
    const [xi, yi] = ring[i] as Vec;
    const [xj, yj] = ring[j] as Vec;
    const f = xj * yi - xi * yj;
    area += f;
    cx += (xi + xj) * f;
    cy += (yi + yj) * f;
  }
  area /= 2;
  if (Math.abs(area) < 1e-9) {
    const n = Math.max(ring.length, 1);
    return {
      area: 0,
      x: ring.reduce((s, p) => s + p[0], 0) / n,
      y: ring.reduce((s, p) => s + p[1], 0) / n,
    };
  }
  return { area, x: cx / (6 * area), y: cy / (6 * area) };
}

/** Long axis of a polygon (a lane strip): its direction and extent along and across it. */
export interface PolygonAxis {
  cx: number;
  cy: number;
  /** Unit direction of the long axis (sign arbitrary). */
  ux: number;
  uy: number;
  /** Extent along the axis, relative to the centroid. */
  min: number;
  max: number;
  /** Extent across the axis (width). */
  width: number;
}

/**
 * Long axis of a polygon from the second moments of its outline (corners weighted by the edges
 * they span), so a lane drawn with a few or many corners gets the same axis.
 */
export function polygonAxis(ring: Vec[]): PolygonAxis | null {
  if (ring.length < 3) {
    return null;
  }
  const { x: cx, y: cy } = polygonCentroid(ring);
  let sxx = 0;
  let sxy = 0;
  let syy = 0;
  for (let i = 0, j = ring.length - 1; i < ring.length; j = i++) {
    const [ax, ay] = ring[j] as Vec;
    const [bx, by] = ring[i] as Vec;
    const length = Math.hypot(bx - ax, by - ay);
    // Second moment of the edge segment about the centroid.
    const px = ax - cx;
    const py = ay - cy;
    const qx = bx - cx;
    const qy = by - cy;
    sxx += (length * (px * px + px * qx + qx * qx)) / 3;
    syy += (length * (py * py + py * qy + qy * qy)) / 3;
    sxy += (length * (2 * px * py + px * qy + qx * py + 2 * qx * qy)) / 6;
  }
  const angle = 0.5 * Math.atan2(2 * sxy, sxx - syy);
  const ux = Math.cos(angle);
  const uy = Math.sin(angle);
  let min = Infinity;
  let max = -Infinity;
  let lo = Infinity;
  let hi = -Infinity;
  for (const [x, y] of ring) {
    const along = (x - cx) * ux + (y - cy) * uy;
    const across = -(x - cx) * uy + (y - cy) * ux;
    min = Math.min(min, along);
    max = Math.max(max, along);
    lo = Math.min(lo, across);
    hi = Math.max(hi, across);
  }
  return { cx, cy, ux, uy, min, max, width: hi - lo };
}

/** Position of a point along an axis (m from its centroid). */
export function along(axis: PolygonAxis, x: number, y: number): number {
  return (x - axis.cx) * axis.ux + (y - axis.cy) * axis.uy;
}

/** Intersection parameter t in [0, 1] of segments p0-p1 and q0-q1, or null. */
export function segmentIntersection(p0: Vec, p1: Vec, q0: Vec, q1: Vec): Vec | null {
  const rx = p1[0] - p0[0];
  const ry = p1[1] - p0[1];
  const sx = q1[0] - q0[0];
  const sy = q1[1] - q0[1];
  const denom = rx * sy - ry * sx;
  if (Math.abs(denom) < 1e-12) {
    return null;
  }
  const t = ((q0[0] - p0[0]) * sy - (q0[1] - p0[1]) * sx) / denom;
  const u = ((q0[0] - p0[0]) * ry - (q0[1] - p0[1]) * rx) / denom;
  if (t < 0 || t > 1 || u < 0 || u > 1) {
    return null;
  }
  return [p0[0] + t * rx, p0[1] + t * ry];
}

/** Smallest absolute difference of two directions (rad, 0..π). */
export function angleBetween(a: number, b: number): number {
  const d = Math.abs(((((b - a) % (2 * Math.PI)) + 3 * Math.PI) % (2 * Math.PI)) - Math.PI);
  return d;
}

/** Applies a row-major 3 x 3 homography to a point; null at or behind the horizon. */
export function homography(h: number[][], u: number, v: number): Vec | null {
  const r0 = h[0] ?? [];
  const r1 = h[1] ?? [];
  const r2 = h[2] ?? [];
  const w = (r2[0] ?? 0) * u + (r2[1] ?? 0) * v + (r2[2] ?? 1);
  if (!(w > 1e-9)) {
    return null;
  }
  return [
    ((r0[0] ?? 0) * u + (r0[1] ?? 0) * v + (r0[2] ?? 0)) / w,
    ((r1[0] ?? 0) * u + (r1[1] ?? 0) * v + (r1[2] ?? 0)) / w,
  ];
}
