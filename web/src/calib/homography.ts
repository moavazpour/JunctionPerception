// Ground plane -> image homography fitted to the clicked point pairs. Used to show map geometry in the
// camera image before the first solve (with four or more pairs); after a solve the solved camera is used.
import { geoToEnu, type GroundCamera } from "../geo";
import type { PairDraft } from "./draft";

export type GeoProjector = (lat: number, lon: number) => [number, number] | null;

// Solves A x = b (n x n) by Gaussian elimination with partial pivoting; null when singular.
function solve(A: number[][], b: number[]): number[] | null {
  const n = b.length;
  const M = A.map((r, i) => [...r, b[i]]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    if (Math.abs(M[p][c]) < 1e-12) return null;
    [M[c], M[p]] = [M[p], M[c]];
    for (let r = 0; r < n; r++) {
      if (r === c) continue;
      const f = M[r][c] / M[c][c];
      for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k];
    }
  }
  return M.map((r, i) => r[n] / r[i]);
}

// Translate to the centroid and scale to unit mean distance (conditioning for the DLT).
function normalizer(p: [number, number][]) {
  const cx = p.reduce((a, q) => a + q[0], 0) / p.length, cy = p.reduce((a, q) => a + q[1], 0) / p.length;
  const s = Math.SQRT2 / (p.reduce((a, q) => a + Math.hypot(q[0] - cx, q[1] - cy), 0) / p.length || 1);
  return { cx, cy, s, f: (q: [number, number]): [number, number] => [(q[0] - cx) * s, (q[1] - cy) * s] };
}

export function fitHomography(points: PairDraft[]): GeoProjector | null {
  const pairs = points.filter((p) => p.u !== undefined && p.v !== undefined && p.lat !== undefined && p.lon !== undefined);
  if (pairs.length < 4) return null;
  const anchor = { lat: pairs[0].lat!, lon: pairs[0].lon! };
  const g = pairs.map((p) => geoToEnu(anchor, p.lat!, p.lon!) as [number, number]);
  const im = pairs.map((p) => [p.u!, p.v!] as [number, number]);
  const ng = normalizer(g), ni = normalizer(im);
  // Least squares with h33 = 1 via the normal equations.
  const AtA = Array.from({ length: 8 }, () => new Array(8).fill(0));
  const Atb = new Array(8).fill(0);
  const add = (row: number[], rhs: number) => {
    for (let i = 0; i < 8; i++) {
      Atb[i] += row[i] * rhs;
      for (let j = 0; j < 8; j++) AtA[i][j] += row[i] * row[j];
    }
  };
  g.forEach((q, k) => {
    const [x, y] = ng.f(q), [u, v] = ni.f(im[k]);
    add([x, y, 1, 0, 0, 0, -u * x, -u * y], u);
    add([0, 0, 0, x, y, 1, -v * x, -v * y], v);
  });
  const h = solve(AtA, Atb);
  if (!h) return null;
  const H = [...h, 1];
  const w = (x: number, y: number) => H[6] * x + H[7] * y + H[8];
  // Points on the camera's side of the horizon share the sign of w with the clicked pairs.
  const [x0, y0] = ng.f(g[0]);
  const side = Math.sign(w(x0, y0));
  return (lat, lon) => {
    const [x, y] = ng.f(geoToEnu(anchor, lat, lon) as [number, number]);
    const ww = w(x, y);
    if (Math.sign(ww) !== side || Math.abs(ww) < 1e-9) return null;
    const u = (H[0] * x + H[1] * y + H[2]) / ww, v = (H[3] * x + H[4] * y + H[5]) / ww;
    return [u / ni.s + ni.cx, v / ni.s + ni.cy];
  };
}

// Best available map -> image projection: the solved camera, else the homography of the point pairs.
export function geoProjector(camera: GroundCamera | undefined, points: PairDraft[]): GeoProjector | null {
  if (camera?.anchor) return (lat, lon) => camera.projectGeo(lat, lon);
  return fitHomography(points);
}
