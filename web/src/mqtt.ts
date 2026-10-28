// MQTT over WebSocket (through the panel's own origin at /mqtt). One connection for the whole app.
// Topics: <prefix>/<site>/<camera>/<leaf>. A site can have several cameras; their state is kept per camera
// (store.cams) and the selected camera's copy is mirrored into the flat store fields. Multi-camera sites
// also publish <prefix>/<site>/fused/objects (the site manager's fusion), which then feeds the twin.
import mqtt from "mqtt";
import { geoToEnu } from "./geo";
import { CAMERA_FIELDS, useStore, type CameraState, type State } from "./store";
import type { NearMiss, RoadUser } from "./types";

const params = new URLSearchParams(location.search);
const wsUrl = params.get("mqtt") ?? `${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/mqtt`;
export const prefix = params.get("prefix") ?? "jp";
export const FUSED = "fused";
// Fused road users replace the selected camera's while they keep coming.
const FUSED_FRESH_S = 1.5;

const client = mqtt.connect(wsUrl, { reconnectPeriod: 2000, connectTimeout: 5000 });
type Waiter = (result: any) => void;
const waiters: Waiter[] = [];

client.on("connect", () => {
  useStore.getState().set({ connected: true });
  client.subscribe(`${prefix}/+/+/#`);
});
client.on("close", () => useStore.getState().set({ connected: false }));

// Topics the panel itself publishes (commands), never state.
const OWN = new Set(["calibration/manual", "map/user/set", "config/set"]);

const now = () => performance.now() / 1000;

// Fused road users in the twin's frame (the selected camera's ground anchor).
function toTwinFrame(objects: RoadUser[], st: State): RoadUser[] {
  const a = st.map?.camera_model?.anchor;
  if (!a) return objects;
  return objects.map((o) => {
    const [x, y] = geoToEnu(a, o.lat, o.lon);
    return { ...o, x, y };
  });
}

// Store patch of one camera message (undefined: not a per-camera field).
function cameraPatch(leaf: string, m: any): CameraState | undefined {
  switch (leaf) {
    case "objects":
      return { objects: m.objects, objectsAt: now(), sourceFrame: m.source_frame };
    case "health":
      return { health: m.health, healthAt: now() };
    case "calibration":
      return { calibration: m.calibration };
    case "map":
      return { map: m };
    case "detectors":
      return { detectors: m };
    case "occlusion":
      return { occlusion: m };
    case "snapshot":
      return { snapshot: m };
    case "config":
      return { config: m };
    case "calibration/refine":
      return { refine: m.refine };
    case "signals":
      return { signals: m };
    case "calibration/inputs":
      return { inputs: m.inputs };
    case "calibration/manual/result":
      return { lastReport: m.result };
  }
  return undefined;
}

client.on("message", (topic, payload) => {
  const parts = topic.split("/");
  if (parts.length < 4) return;
  const [, site, camera] = parts;
  const leaf = parts.slice(3).join("/");
  let st = useStore.getState();
  if (!st.sitesSeen.includes(site)) st.set({ sitesSeen: [...st.sitesSeen, site] });
  if (camera !== FUSED && !(st.camerasSeen[site] ?? []).includes(camera))
    st.set({ camerasSeen: { ...st.camerasSeen, [site]: [...(st.camerasSeen[site] ?? []), camera].sort() } });
  if (!st.site && camera !== FUSED) st.set({ site, camera });
  st = useStore.getState();
  if (st.site !== site) return;
  if (OWN.has(leaf) || leaf.startsWith("detector/")) return;
  const text = payload.toString();
  const selected = camera === st.camera;
  if (!text) {
    // Cleared retained topic.
    const cleared: CameraState | undefined =
      leaf === "calibration" ? { calibration: undefined } : leaf === "map" ? { map: undefined }
        : leaf === "detectors" ? { detectors: undefined } : leaf === "signals" ? { signals: undefined } : undefined;
    if (cleared) {
      st.set({ cams: { ...st.cams, [camera]: { ...st.cams[camera], ...cleared } } });
      if (selected) st.set(cleared);
    }
    return;
  }
  let m: any;
  try {
    m = JSON.parse(text);
  } catch {
    return;
  }
  if (camera === FUSED) {
    if (leaf === "objects") st.set({ objects: toTwinFrame(m.objects, st), objectsAt: now(), fusedAt: now(), sourceFrame: m.source_frame ?? undefined });
    return;
  }
  // Site-wide: the drawn layout (the same on every camera) and safety events from any camera.
  if (leaf === "map/user") return st.set({ userMap: m.map });
  if (leaf === "events/safety") {
    const e = m.event as NearMiss;
    return st.set({ events: [e, ...st.events.filter((x) => x.event_id !== e.event_id)].slice(0, 50) });
  }
  const patch = cameraPatch(leaf, m);
  if (!patch) return;
  st.set({ cams: { ...st.cams, [camera]: { ...st.cams[camera], ...patch } } });
  if (!selected) return;
  // While fused road users arrive they drive the twin, not the selected camera's own list.
  if (leaf === "objects" && st.fusedAt > 0 && now() - st.fusedAt < FUSED_FRESH_S) return;
  st.set(patch);
  if (leaf === "calibration/manual/result") waiters.splice(0).forEach((w) => w(m.result));
});

// Leaves that belong to the whole site (every camera gets them): the drawn layout and the detection settings.
const SITE_WIDE = new Set(["map/user/set", "config/set"]);

// Commands go to the selected camera, site-wide ones to all cameras of the site.
export function publish(leaf: string, body: object) {
  const { site, camera, camerasSeen } = useStore.getState();
  if (!site) return false;
  const targets = SITE_WIDE.has(leaf) ? camerasSeen[site] ?? [camera] : [camera];
  for (const cam of targets) client.publish(`${prefix}/${site}/${cam}/${leaf}`, JSON.stringify(body), { qos: 1 });
  return true;
}

// Calibration command; resolves with the runtime's report (or rejects after a timeout).
export function calibrationCommand(body: object, timeoutMs = 60000): Promise<any> {
  return new Promise((resolve, reject) => {
    if (!publish("calibration/manual", body)) return reject(new Error("no site selected"));
    const t = setTimeout(() => {
      const i = waiters.indexOf(w);
      if (i >= 0) waiters.splice(i, 1);
      reject(new Error("the runtime did not answer (is the site running?)"));
    }, timeoutMs);
    const w: Waiter = (r) => {
      clearTimeout(t);
      resolve(r);
    };
    waiters.push(w);
  });
}

const EMPTY_CAMERA: CameraState = Object.fromEntries(CAMERA_FIELDS.map((k) => [k, undefined]));

// Switching camera within the site: its cached state becomes the selected one (calibration, video, map).
export function selectCamera(camera: string) {
  const st = useStore.getState();
  const fused = st.fusedAt > 0 && now() - st.fusedAt < FUSED_FRESH_S;
  const { objects, objectsAt, sourceFrame, ...rest }: CameraState = { ...EMPTY_CAMERA, objects: [], objectsAt: 0, healthAt: 0, ...st.cams[camera] };
  st.set({ ...rest, ...(fused ? {} : { objects, objectsAt, sourceFrame }), camera });
}

// Switching site: drop the previous site's state; retained topics refill it.
export function selectSite(site: string) {
  const url = new URL(location.href);
  url.searchParams.set("site", site);
  history.replaceState(null, "", url);
  useStore.getState().set({
    site, camera: "cam1", cams: {}, fusedAt: 0, sourceFrame: undefined, gt: undefined, objects: [], health: undefined, calibration: undefined, map: undefined,
    detectors: undefined, occlusion: undefined, snapshot: undefined, config: undefined, refine: undefined, userMap: undefined,
    signals: undefined, inputs: undefined, lastReport: undefined, events: [], selected: null,
  });
  client.unsubscribe(`${prefix}/+/+/#`, () => client.subscribe(`${prefix}/+/+/#`));
}
