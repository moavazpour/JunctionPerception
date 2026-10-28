// Application state: everything the runtime publishes for the selected site, plus UI preferences.
// MQTT messages land here (see mqtt.ts); components subscribe with useStore(selector).
import { create } from "zustand";
import type {
  CalibInputs, CalibrationMsg, ConfigMsg, DetectorsMsg, Health, MapMsg, NearMiss, OcclusionMsg, RefineState, RoadUser,
  SignalsMsg, SnapshotMsg, SolveReport, UserMap,
} from "./types";
import type { GroundTruth } from "./api";

export type ThemePref = "system" | "light" | "dark";
export type MapMode = "stylized" | "aerial";
export type OffLane = "show" | "ghost" | "hide";

const load = <T,>(key: string, fallback: T): T => {
  try {
    const v = localStorage.getItem(key);
    return v ? (JSON.parse(v) as T) : fallback;
  } catch {
    return fallback;
  }
};
export const save = (key: string, v: unknown) => {
  try {
    localStorage.setItem(key, JSON.stringify(v));
  } catch {
    /* storage unavailable */
  }
};

// What one camera publishes. The flat fields of State hold the selected camera's copy; `cams` keeps every
// camera of the site, so switching camera is instant.
export const CAMERA_FIELDS = [
  "objects", "objectsAt", "sourceFrame", "health", "healthAt", "calibration", "map", "detectors", "occlusion", "snapshot", "config",
  "refine", "signals", "inputs", "lastReport",
] as const;
export type CameraState = Partial<Pick<State, (typeof CAMERA_FIELDS)[number]>>;

export interface State {
  connected: boolean;
  site: string | null;
  camera: string;
  sitesSeen: string[];
  camerasSeen: Record<string, string[]>; // per site, from the MQTT topics
  cams: Record<string, CameraState>;
  fusedAt: number; // last fused road-user list (multi-camera sites); 0 = none
  sourceFrame?: number; // source video frame of the shown road users (to line up ground truth)
  gt?: GroundTruth | null; // ground truth of a replayed dataset site (null: none)
  gtVisible: boolean; // ground-truth boxes and the evaluation panel shown
  objects: RoadUser[];
  objectsAt: number;
  health?: Health;
  healthAt: number;
  calibration?: CalibrationMsg["calibration"];
  map?: MapMsg;
  detectors?: DetectorsMsg;
  occlusion?: OcclusionMsg;
  snapshot?: SnapshotMsg;
  config?: ConfigMsg;
  refine?: RefineState;
  userMap?: UserMap;
  signals?: SignalsMsg;
  inputs?: CalibInputs;
  lastReport?: SolveReport;
  events: NearMiss[];
  selected: number | null;
  // Preferences (per browser).
  theme: ThemePref;
  mapMode: MapMode;
  offLane: OffLane; // road users off the drawn lanes, in the 3D view
  focusLanes: boolean; // fade the map outside the drawn lanes
  labels: boolean;
  occlusionLayer: boolean;
  videoCorner: "br" | "bl" | "tr" | "tl";
  videoSize: "s" | "m" | "l";
  videoVisible: boolean;
  set: (p: Partial<State>) => void;
}

export const useStore = create<State>((set) => ({
  connected: false,
  site: new URLSearchParams(location.search).get("site"),
  camera: "cam1",
  sitesSeen: [],
  camerasSeen: {},
  cams: {},
  fusedAt: 0,
  gtVisible: false,
  objects: [],
  objectsAt: 0,
  healthAt: 0,
  events: [],
  selected: null,
  theme: load<ThemePref>("jp-theme", "system"),
  mapMode: load<MapMode>("jp-mapmode", "aerial"),
  offLane: load<OffLane>("jp-offlane", "ghost"),
  focusLanes: load("jp-focus-lanes", true),
  labels: load("jp-labels", true),
  occlusionLayer: load("jp-occ", false),
  videoCorner: load("jp-video-corner", "br"),
  videoSize: load("jp-video-size", "m"),
  videoVisible: load("jp-video", true),
  set: (p) => set(p),
}));

// Persist preferences when they change.
useStore.subscribe((s, prev) => {
  if (s.theme !== prev.theme) save("jp-theme", s.theme);
  if (s.mapMode !== prev.mapMode) save("jp-mapmode", s.mapMode);
  if (s.offLane !== prev.offLane) save("jp-offlane", s.offLane);
  if (s.focusLanes !== prev.focusLanes) save("jp-focus-lanes", s.focusLanes);
  if (s.labels !== prev.labels) save("jp-labels", s.labels);
  if (s.occlusionLayer !== prev.occlusionLayer) save("jp-occ", s.occlusionLayer);
  if (s.videoCorner !== prev.videoCorner) save("jp-video-corner", s.videoCorner);
  if (s.videoSize !== prev.videoSize) save("jp-video-size", s.videoSize);
  if (s.videoVisible !== prev.videoVisible) save("jp-video", s.videoVisible);
});

// Resolved theme (system preference followed live).
export function resolvedTheme(pref: ThemePref): "light" | "dark" {
  if (pref !== "system") return pref;
  return window.matchMedia?.("(prefers-color-scheme: dark)").matches ? "dark" : "light";
}

export const CLASS_LABEL: Record<string, string> = {
  car: "Car",
  bus: "Bus",
  truck: "Truck",
  trailer: "Trailer",
  pedestrian: "Pedestrian",
  bicycle: "Bicycle",
  escooter: "E-scooter",
  motorcycle: "Motorcycle",
};
// Same colours as the live view overlay drawn by the runtime.
export const CLASS_COLOR: Record<string, string> = {
  car: "#4c8dff",
  bus: "#ffd43b",
  truck: "#ff9f43",
  trailer: "#c0a080",
  pedestrian: "#51cf66",
  bicycle: "#b197fc",
  escooter: "#3cc8e6",
  motorcycle: "#f06595",
};
export const LANE_COLORS = ["#4c8dff", "#ff9f43", "#51cf66", "#f06595", "#b197fc", "#ffd43b", "#3cc8e6", "#ff6b6b"];
