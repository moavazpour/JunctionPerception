/** Plane geometry of the bird's-eye view, in metres of a site's world frame (x east, y north). */

import type { Vec } from './siteFrame';

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
