// MQTT over WebSocket (through the panel's own origin at /mqtt). One connection for the whole app.
import mqtt from "mqtt";
import { useStore } from "./store";
import type { NearMiss } from "./types";

const params = new URLSearchParams(location.search);
const wsUrl = params.get("mqtt") ?? `${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/mqtt`;
export const prefix = params.get("prefix") ?? "jp";

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

client.on("message", (topic, payload) => {
  const parts = topic.split("/");
  if (parts.length < 4) return;
  const [, site, camera] = parts;
  const leaf = parts.slice(3).join("/");
  const st = useStore.getState();
  if (!st.sitesSeen.includes(site)) st.set({ sitesSeen: [...st.sitesSeen, site] });
  if (!st.site) st.set({ site, camera });
  if (useStore.getState().site !== site) return;
  if (OWN.has(leaf) || leaf.startsWith("detector/")) return;
  const text = payload.toString();
  if (!text) {
    // Cleared retained topic.
    if (leaf === "calibration") st.set({ calibration: undefined });
    else if (leaf === "map") st.set({ map: undefined });
    else if (leaf === "detectors") st.set({ detectors: undefined });
    else if (leaf === "signals") st.set({ signals: undefined });
    return;
  }
  let m: any;
  try {
    m = JSON.parse(text);
  } catch {
    return;
  }
  const set = st.set;
  switch (leaf) {
    case "objects":
      set({ objects: m.objects, objectsAt: performance.now() / 1000 });
      break;
    case "health":
      set({ health: m.health, healthAt: performance.now() / 1000 });
      break;
    case "calibration":
      set({ calibration: m.calibration });
      break;
    case "map":
      set({ map: m });
      break;
    case "detectors":
      set({ detectors: m });
      break;
    case "occlusion":
      set({ occlusion: m });
      break;
    case "snapshot":
      set({ snapshot: m });
      break;
    case "config":
      set({ config: m, camera });
      break;
    case "calibration/refine":
      set({ refine: m.refine });
      break;
    case "map/user":
      set({ userMap: m.map });
      break;
    case "signals":
      set({ signals: m });
      break;
    case "calibration/inputs":
      set({ inputs: m.inputs });
      break;
    case "calibration/manual/result":
      set({ lastReport: m.result });
      waiters.splice(0).forEach((w) => w(m.result));
      break;
    case "events/safety": {
      const e = m.event as NearMiss;
      const events = [e, ...st.events.filter((x) => x.event_id !== e.event_id)].slice(0, 50);
      set({ events });
      break;
    }
  }
});

export function publish(leaf: string, body: object) {
  const { site, camera } = useStore.getState();
  if (!site) return false;
  client.publish(`${prefix}/${site}/${camera}/${leaf}`, JSON.stringify(body), { qos: 1 });
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

// Switching site: drop the previous site's state; retained topics refill it.
export function selectSite(site: string) {
  const url = new URL(location.href);
  url.searchParams.set("site", site);
  history.replaceState(null, "", url);
  useStore.getState().set({
    site, objects: [], health: undefined, calibration: undefined, map: undefined, detectors: undefined, occlusion: undefined,
    snapshot: undefined, config: undefined, refine: undefined, userMap: undefined, signals: undefined, inputs: undefined, lastReport: undefined,
    events: [], selected: null,
  });
  client.unsubscribe(`${prefix}/+/+/#`, () => client.subscribe(`${prefix}/+/+/#`));
}
