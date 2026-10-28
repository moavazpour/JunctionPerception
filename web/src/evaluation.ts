// Accuracy of the twin against recorded ground truth (replayed dataset sites): road users matched per source
// video frame, and the site's calibration compared with the dataset's reference calibration.
import { create } from "zustand";
import { api, json, type GroundTruth, type GtObject } from "./api";
import { enuToGeo, geoToEnu, GroundCamera } from "./geo";
import { useStore, type State } from "./store";
import type { CameraJson, RoadUser } from "./types";

export const GATE_M = 3.0; // a detection within this distance of a ground-truth road user of its kind matches it
export const MAX_RANGE_M = 100; // evaluated area: within this distance of a camera
export const RANGES = [30, 60, 100] as const; // error bins: 0-30, 30-60, 60-100 m from the nearest camera

const KIND: Record<string, string> = {
  car: "vehicle", van: "vehicle", truck: "vehicle", bus: "vehicle", trailer: "vehicle",
  bicycle: "two_wheel", motorcycle: "two_wheel", escooter: "two_wheel", pedestrian: "pedestrian",
};
const kind = (c: string) => KIND[c] ?? c;

export interface Match {
  range: number;
  pos: number; // m
  heading?: number; // deg, both moving
  speed?: number; // m/s, signed (ours - truth)
  length?: number; // m, signed
}

export interface EvalState {
  frames: number;
  tp: number;
  fp: number;
  fn: number;
  matches: Match[];
  lastFrame: number;
  loops: number;
  reset: () => void;
}

const EMPTY = { frames: 0, tp: 0, fp: 0, fn: 0, matches: [] as Match[], lastFrame: -1 };
export const useEval = create<EvalState>((set) => ({ ...EMPTY, loops: 0, reset: () => set({ ...EMPTY }) }));

// Cameras whose view the shown road users cover: every camera while the fused list arrives, else the selected one.
export function coveredCameras(st: State): string[] {
  const fused = st.fusedAt > 0 && performance.now() / 1000 - st.fusedAt < 1.5;
  return fused && st.site ? st.camerasSeen[st.site] ?? [st.camera] : [st.camera];
}

// Ground-truth road users of the current frame that the covered cameras see.
export function gtFrame(gt: GroundTruth, frame: number | undefined, cams: string[]): GtObject[] {
  if (frame === undefined || frame < 0 || frame >= gt.frames.length) return [];
  return gt.frames[frame].filter((o) => o[9].some((c) => cams.includes(c)));
}

// Switches the site's twin source between the perception output and the recording (kept on the site manager).
export async function setGtTwin(site: string, on: boolean) {
  await api(`/sites/${site}/groundtruth/twin`, json("PUT", { on }));
  const st = useStore.getState();
  if (st.gt) st.set({ gt: { ...st.gt, twin: on }, objects: [], events: [], selected: null, sourceFrame: undefined });
  useEval.getState().reset();
}

// Numeric ids for the recording's track ids, stable for the session.
const gtIds = new Map<string, number>();

// The recording's road users of one frame as twin road users (twin source "ground truth"): every road user some
// camera of the site sees, in the frame of `anchor`. A missing speed comes from the previous frame.
export function gtRoadUsers(gt: GroundTruth, frame: number, anchor: { lat: number; lon: number }): RoadUser[] {
  if (frame < 0 || frame >= gt.frames.length) return [];
  const prev = new Map(frame > 0 ? gt.frames[frame - 1].map((o) => [o[7], o] as const) : []);
  return gt.frames[frame]
    .filter((o) => o[9].length > 0)
    .map((o) => {
      const [x, y] = geoToEnu(anchor, o[0], o[1]);
      let id = gtIds.get(o[7]);
      if (id === undefined) gtIds.set(o[7], (id = gtIds.size + 1));
      const p = prev.get(o[7]);
      const speed = o[8] ?? (p ? Math.hypot(...geoToEnu({ lat: p[0], lon: p[1] }, o[0], o[1])) * gt.fps : 0);
      return {
        id, class: o[6] as RoadUser["class"], state: "tracked", x, y, lat: o[0], lon: o[1],
        heading: o[2], yaw: ((90 - o[2]) * Math.PI) / 180, speed, speed_kmh: speed * 3.6, accel: 0, yaw_rate: 0,
        length: o[3], width: o[4], height: o[5], sigma: { pos: 0, speed: 0, accel: 0 }, confidence: 1,
        range: Math.hypot(x, y), age: 0, occluded: false, speed_valid: true, heading_valid: true,
      };
    });
}

const headingDiff = (a: number, b: number) => Math.abs((((a - b) % 360) + 540) % 360 - 180);

function evaluate(st: State) {
  const gt = st.gt;
  const f = st.sourceFrame;
  const ev = useEval.getState();
  if (!gt || f === undefined || f === ev.lastFrame) return;
  // The video restarted: a new loop starts a new evaluation.
  if (f < ev.lastFrame) {
    useEval.setState({ ...EMPTY, loops: ev.loops + 1 });
    return;
  }
  const cams = coveredCameras(st);
  const anchors = cams.map((c) => st.cams[c]?.calibration?.anchor).filter((a): a is NonNullable<typeof a> => !!a);
  if (!anchors.length) return;
  const o0 = anchors[0];
  const range = (lat: number, lon: number) =>
    Math.min(...anchors.map((a) => Math.hypot(...geoToEnu(a, lat, lon))));
  const truth = gtFrame(gt, f, cams)
    .map((o) => ({ o, p: geoToEnu(o0, o[0], o[1]), r: range(o[0], o[1]) }))
    .filter((t) => t.r <= MAX_RANGE_M);
  const ours = st.objects
    .filter((u: RoadUser) => u.state !== "tentative")
    .map((u) => ({ u, p: geoToEnu(o0, u.lat, u.lon), r: range(u.lat, u.lon) }))
    .filter((t) => t.r <= MAX_RANGE_M);
  // Greedy matching on the closest pairs of the same kind.
  const pairs: [number, number, number][] = [];
  truth.forEach((t, i) =>
    ours.forEach((d, j) => {
      if (kind(t.o[6]) !== kind(d.u.class)) return;
      const dist = Math.hypot(t.p[0] - d.p[0], t.p[1] - d.p[1]);
      if (dist <= GATE_M) pairs.push([dist, i, j]);
    }),
  );
  pairs.sort((a, b) => a[0] - b[0]);
  const ti = new Set<number>(), oj = new Set<number>();
  const matches: Match[] = [];
  for (const [dist, i, j] of pairs) {
    if (ti.has(i) || oj.has(j)) continue;
    ti.add(i);
    oj.add(j);
    const t = truth[i].o, u = ours[j].u;
    const moving = (t[8] ?? 0) > 1 && u.speed > 1;
    matches.push({
      range: truth[i].r,
      pos: dist,
      heading: moving ? headingDiff(u.heading, t[2]) : undefined,
      speed: t[8] !== null && u.speed_valid !== false ? u.speed - t[8] : undefined,
      length: u.length - t[3],
    });
  }
  useEval.setState({
    frames: ev.frames + 1,
    tp: ev.tp + matches.length,
    fp: ev.fp + ours.length - matches.length,
    fn: ev.fn + truth.length - matches.length,
    matches: ev.matches.concat(matches),
    lastFrame: f,
  });
}

let started = false;
// Evaluates every new source frame while the site has ground truth.
export function startEvaluation() {
  if (started) return;
  started = true;
  useStore.subscribe((cur, prev) => {
    if (cur.site !== prev.site) useEval.getState().reset();
    // A twin drawn from the ground truth itself has nothing to be measured against.
    if (cur.gt && !cur.gt.twin && cur.sourceFrame !== prev.sourceFrame) evaluate(cur);
  });
}

export function quantile(v: number[], q: number) {
  if (!v.length) return undefined;
  const s = [...v].sort((a, b) => a - b);
  return s[Math.min(s.length - 1, Math.floor(q * s.length))];
}

// The site's calibration against the reference: camera pose and focal length, and how far a ground point
// seen by the reference camera at 20 / 50 / 80 m along its view lands with ours.
export function calibrationDiff(ours: CameraJson, ref: CameraJson) {
  const a = new GroundCamera(ours), b = new GroundCamera(ref);
  if (!a.anchor || !b.anchor) return null;
  const position = Math.hypot(...geoToEnu(b.anchor, a.anchor.lat, a.anchor.lon));
  const ground = [20, 50, 80].map((r) => {
    const br = (ref.bearing_deg * Math.PI) / 180;
    const [lat, lon] = enuToGeo(b.anchor!, Math.sin(br) * r, Math.cos(br) * r);
    const px = b.projectGeo(lat, lon);
    const g = px && a.unproject(px[0], px[1]);
    if (!g) return { range: r, error: undefined };
    const [la, lo] = enuToGeo(a.anchor!, g[0], g[1]);
    return { range: r, error: Math.hypot(...geoToEnu({ lat, lon }, la, lo)) };
  });
  const wrap = (d: number) => ((d + 540) % 360) - 180;
  return {
    position,
    height: ours.height_m - ref.height_m,
    bearing: wrap(ours.bearing_deg - ref.bearing_deg),
    pitch: ours.pitch_deg - ref.pitch_deg,
    roll: ours.roll_deg - ref.roll_deg,
    focal: (ours.K[0][0] / ref.K[0][0] - 1) * 100,
    ground,
  };
}
