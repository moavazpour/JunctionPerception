// Local tangent-plane conversion (mirrors core geoToEnu) and the pinhole ground camera (mirrors CameraModel).
import type { CameraJson } from "./types";

const A = 6378137.0, E2 = 6.69437999014e-3, DEG = Math.PI / 180;

export type V2 = [number, number];

function radii(lat0: number) {
  const s = Math.sin(lat0 * DEG);
  const M = (A * (1 - E2)) / Math.pow(1 - E2 * s * s, 1.5);
  const N = A / Math.sqrt(1 - E2 * s * s);
  return { M, N, c: Math.cos(lat0 * DEG) };
}

// [lat, lon] -> [east, north] metres around an anchor.
export function geoToEnu(anchor: { lat: number; lon: number }, lat: number, lon: number): V2 {
  const { M, N, c } = radii(anchor.lat);
  return [(lon - anchor.lon) * DEG * N * c, (lat - anchor.lat) * DEG * M];
}

export function enuToGeo(anchor: { lat: number; lon: number }, e: number, n: number): V2 {
  const { M, N, c } = radii(anchor.lat);
  return [anchor.lat + n / (M * DEG), anchor.lon + e / (N * c * DEG)];
}

export function bearingDeg(a: { lat: number; lng: number }, b: { lat: number; lng: number }) {
  const f1 = a.lat * DEG, f2 = b.lat * DEG, dl = (b.lng - a.lng) * DEG;
  const y = Math.sin(dl) * Math.cos(f2), x = Math.cos(f1) * Math.sin(f2) - Math.sin(f1) * Math.cos(f2) * Math.cos(dl);
  return ((Math.atan2(y, x) / DEG) + 360) % 360;
}

type M3 = number[][];
const mul = (m: M3, v: number[]) => [0, 1, 2].map((i) => m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2]);
const tr = (m: M3): M3 => [0, 1, 2].map((i) => [0, 1, 2].map((j) => m[j][i]));
const norm = (v: number[]) => {
  const l = Math.hypot(v[0], v[1], v[2]) || 1;
  return v.map((x) => x / l);
};
const cross = (a: number[], b: number[]) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];

// Ground frame: ENU with the origin on the ground below the camera (the camera's anchor).
export class GroundCamera {
  readonly width: number;
  readonly height: number;
  readonly anchor?: { lat: number; lon: number };
  private K: M3;
  private n: number[];
  private d: number;
  private Rgc: M3;

  constructor(j: CameraJson) {
    this.width = j.width;
    this.height = j.height;
    this.anchor = j.anchor;
    this.K = j.K;
    const l = Math.hypot(j.plane[0], j.plane[1], j.plane[2]);
    this.n = [j.plane[0] / l, j.plane[1] / l, j.plane[2] / l];
    this.d = j.plane[3] / l;
    // Same construction as CameraModel::build: horizontal forward f, left l = n x f, up n.
    const ez = [0, 0, 1];
    const dot = this.n[2];
    const f = norm([ez[0] - dot * this.n[0], ez[1] - dot * this.n[1], ez[2] - dot * this.n[2]]);
    const lv = cross(this.n, f);
    const C: M3 = [0, 1, 2].map((i) => [f[i], lv[i], this.n[i]]);
    const b = j.bearing_deg * DEG;
    const W: M3 = [
      [Math.sin(b), -Math.cos(b), 0],
      [Math.cos(b), Math.sin(b), 0],
      [0, 0, 1],
    ];
    const Ct = tr(C);
    this.Rgc = [0, 1, 2].map((i) => [0, 1, 2].map((k) => W[i][0] * Ct[0][k] + W[i][1] * Ct[1][k] + W[i][2] * Ct[2][k]));
  }

  get heightM() {
    return this.d;
  }

  // Ground point (x east, y north, z up) -> pixel; null behind the camera.
  project(x: number, y: number, z = 0): V2 | null {
    const pc = mul(tr(this.Rgc), [x, y, z - this.d]);
    if (pc[2] < 0.1) return null;
    const uv = mul(this.K, pc);
    return [uv[0] / uv[2], uv[1] / uv[2]];
  }

  // Pixel -> ground point; null above the horizon.
  unproject(u: number, v: number): V2 | null {
    const K = this.K;
    const ry = (v - K[1][2]) / K[1][1];
    const rx = (u - K[0][2] - K[0][1] * ry) / K[0][0];
    const r = [rx, ry, 1];
    const den = r[0] * this.n[0] + r[1] * this.n[1] + r[2] * this.n[2];
    if (Math.abs(den) < 1e-12) return null;
    const t = -this.d / den;
    if (t <= 0 || !isFinite(t) || t > 3000) return null;
    const g = mul(this.Rgc, [r[0] * t, r[1] * t, r[2] * t]);
    return [g[0], g[1]];
  }

  enu(lat: number, lon: number): V2 {
    return this.anchor ? geoToEnu(this.anchor, lat, lon) : [0, 0];
  }

  projectGeo(lat: number, lon: number): V2 | null {
    const [x, y] = this.enu(lat, lon);
    return this.project(x, y);
  }
}
