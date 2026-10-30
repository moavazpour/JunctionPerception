import type { ZoneCollection } from './client';

/** A control point: image position (px) and map position (world frame, m). */
export interface ControlPoint {
  id: string;
  u: number;
  v: number;
  x: number;
  y: number;
  check: boolean;
}

export interface PointResidual {
  id: string;
  check: boolean;
  inlier: boolean;
  error_m: number;
}

export interface FitResult {
  residuals: PointResidual[];
  /** Row-major 3x3 homography from image pixels to the world frame. */
  image_to_ground: number[][];
  fit_points: number;
  inliers: number;
  fit_rms_m: number;
  check_mean_m: number | null;
  check_max_m: number | null;
  camera_position_m: [number, number] | null;
  camera_height_m: number | null;
}

async function post<T>(url: string, body: unknown): Promise<T> {
  const response = await fetch(url, { method: 'POST', body: JSON.stringify(body) });
  const parsed = (await response.json()) as T | { error: string };
  if (!response.ok) {
    throw new Error((parsed as { error: string }).error ?? `POST ${url} failed`);
  }
  return parsed as T;
}

export async function fetchControlPoints(): Promise<ControlPoint[]> {
  const response = await fetch('/api/calibration');
  return ((await response.json()) as { points: ControlPoint[] }).points;
}

export function fitControlPoints(points: ControlPoint[]): Promise<FitResult> {
  return post<FitResult>('/api/calibration/fit', { points });
}

export function saveCalibration(
  points: ControlPoint[],
  cameraPosition: [number, number] | null,
): Promise<FitResult> {
  return post<FitResult>('/api/calibration/save', { points, camera_position_m: cameraPosition });
}

export function saveZones(zones: ZoneCollection): Promise<ZoneCollection> {
  return post<ZoneCollection>('/api/zones', zones);
}

export async function requestRestart(): Promise<void> {
  await fetch('/api/restart', { method: 'POST' });
}

/** Applies a row-major 3x3 homography to a 2D point; null at infinity. */
export function applyHomography(h: number[][], x: number, y: number): [number, number] | null {
  const row = (i: number): number[] => h[i] ?? [0, 0, 0];
  const [a = 0, b = 0, c = 0] = row(0);
  const [d = 0, e = 0, f = 0] = row(1);
  const [g = 0, k = 0, l = 1] = row(2);
  const w = g * x + k * y + l;
  if (Math.abs(w) < 1e-12) {
    return null;
  }
  return [(a * x + b * y + c) / w, (d * x + e * y + f) / w];
}

/** Inverse of a 3x3 matrix (row-major), or null if singular. */
export function invert3(m: number[][]): number[][] | null {
  const [[a = 0, b = 0, c = 0] = [], [d = 0, e = 0, f = 0] = [], [g = 0, h = 0, i = 0] = []] = m;
  const det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
  if (Math.abs(det) < 1e-15) {
    return null;
  }
  return [
    [(e * i - f * h) / det, (c * h - b * i) / det, (b * f - c * e) / det],
    [(f * g - d * i) / det, (a * i - c * g) / det, (c * d - a * f) / det],
    [(d * h - e * g) / det, (b * g - a * h) / det, (a * e - b * d) / det],
  ];
}
