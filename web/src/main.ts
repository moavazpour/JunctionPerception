import mqtt from "mqtt";
import "./style.css";
import { Scene3D } from "./scene3d";
import { renderCalibration } from "./calib";
import type {
  CalibrationMsg, DetectorsMsg, HealthMsg, MapMsg, NearMiss, ObjectsMsg, OcclusionMsg, RoadUser, SafetyMsg, SnapshotMsg,
} from "./types";

const $ = <T extends HTMLElement>(sel: string) => document.querySelector(sel) as T;

const params = new URLSearchParams(location.search);
const wsUrl =
  params.get("mqtt") ?? `${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/mqtt`;
const prefix = params.get("prefix") ?? "jp";

const scene = new Scene3D($("#view"));
let users: RoadUser[] = [];
let mapMsg: MapMsg | null = null;
let lastOcc: OcclusionMsg | null = null;
let showOcc = true;
let lastDetectors: DetectorsMsg | null = null;
let calibMsg: CalibrationMsg | null = null;
let snapshot: SnapshotMsg | null = null;
const events: NearMiss[] = [];
let site = "", cameraId = "";
let firstMap = true;
let lastObjectsAt = 0;

// ------------------------------------------------------------------ MQTT

const client = mqtt.connect(wsUrl, { reconnectPeriod: 2000, connectTimeout: 5000 });
client.on("connect", () => {
  setConn(true);
  client.subscribe(`${prefix}/+/+/#`);
});
client.on("close", () => setConn(false));
client.on("message", (topic, payload) => {
  const leaf = topic.split("/").slice(3).join("/");
  if (leaf.startsWith("detector/")) return;
  let msg: any;
  try {
    msg = JSON.parse(payload.toString());
  } catch {
    return;
  }
  const t = performance.now() / 1000;
  switch (leaf) {
    case "objects":
      users = (msg as ObjectsMsg).objects;
      site = msg.site;
      cameraId = msg.camera;
      lastObjectsAt = t;
      scene.updateObjects(users, t);
      renderUsers();
      break;
    case "map":
      mapMsg = msg as MapMsg;
      scene.setMap(mapMsg.map, mapMsg.camera_model);
      if (firstMap) {
        scene.view("orbit");
        firstMap = false;
      }
      $("#site").textContent = mapMsg.map.name || mapMsg.site;
      break;
    case "detectors":
      lastDetectors = msg as DetectorsMsg;
      scene.setDetectors(lastDetectors);
      renderLanes();
      break;
    case "health":
      renderHealth(msg as HealthMsg);
      break;
    case "occlusion":
      lastOcc = msg as OcclusionMsg;
      scene.setOcclusion(lastOcc, showOcc);
      break;
    case "events/safety": {
      const e = (msg as SafetyMsg).event;
      const i = events.findIndex((x) => x.event_id === e.event_id);
      if (i >= 0) events[i] = e;
      else {
        events.unshift(e);
        scene.flashEvent(e);
      }
      events.splice(50);
      renderEvents();
      break;
    }
    case "calibration":
      calibMsg = msg as CalibrationMsg;
      renderCalib();
      break;
    case "snapshot":
      snapshot = msg as SnapshotMsg;
      renderCalib();
      break;
  }
});

function setConn(ok: boolean) {
  const el = $("#conn");
  el.textContent = ok ? "live" : "offline";
  el.className = `pill ${ok ? "ok" : "fault"}`;
}
setInterval(() => {
  if (performance.now() / 1000 - lastObjectsAt > 3 && client.connected) {
    $("#conn").textContent = "no data";
    $("#conn").className = "pill degraded";
  }
}, 1000);

// ------------------------------------------------------------------ panels

const icons: Record<string, string> = {
  car: "🚗", bus: "🚌", truck: "🚚", trailer: "🚛", pedestrian: "🚶", bicycle: "🚲", escooter: "🛴", motorcycle: "🏍️",
};
const fmt = (v: number | undefined, d = 1) => (v === undefined || v === null ? "–" : v.toFixed(d));

function renderUsers() {
  const sorted = [...users].sort((a, b) => (a.eta ?? 1e9) - (b.eta ?? 1e9) || a.range - b.range);
  $("#count").textContent = String(users.length);
  const byClass = new Map<string, number>();
  for (const u of users) byClass.set(u.class, (byClass.get(u.class) ?? 0) + 1);
  $("#classes").innerHTML = [...byClass].map(([c, n]) => `<span class="chip">${icons[c] ?? ""} ${n}</span>`).join("");
  $("#users tbody").innerHTML = sorted
    .slice(0, 60)
    .map((u) => {
      const sel = scene.selected === u.id ? "sel" : "";
      const eta = u.dist_to_stop !== undefined && u.dist_to_stop > 0 ? (u.eta !== undefined ? fmt(u.eta) + "s" : u.stopping ? "stop" : "–") : u.dist_to_stop !== undefined ? "past" : "";
      return `<tr data-id="${u.id}" class="${sel} ${u.state}">
        <td>${icons[u.class] ?? ""}</td><td>#${u.id}</td><td>${fmt(u.speed_kmh, 0)}</td>
        <td class="${u.accel < -3 ? "bad" : ""}">${fmt(u.accel)}</td><td>${u.lane ?? ""}</td>
        <td>${u.dist_to_stop !== undefined ? fmt(u.dist_to_stop, 0) : ""}</td><td>${eta}</td></tr>`;
    })
    .join("");
  renderSelected();
}

function renderSelected() {
  const u = users.find((x) => x.id === scene.selected);
  const el = $("#selected");
  if (!u) {
    el.innerHTML = `<div class="muted">Click a road user for details</div>`;
    return;
  }
  el.innerHTML = `
    <div class="sel-head">${icons[u.class]} <b>${u.class}</b> #${u.id} <span class="pill ${u.state === "tracked" ? "ok" : "degraded"}">${u.state}</span></div>
    <div class="grid2">
      <div>Speed</div><div>${fmt(u.speed_kmh)} km/h ± ${fmt(u.sigma.speed * 3.6)}</div>
      <div>Acceleration</div><div>${fmt(u.accel, 2)} m/s² ± ${fmt(u.sigma.accel, 2)}</div>
      <div>Heading</div><div>${fmt(u.heading, 0)}°</div>
      <div>Size</div><div>${fmt(u.length)} × ${fmt(u.width)} × ${fmt(u.height)} m</div>
      <div>Position</div><div>${u.lat.toFixed(6)}, ${u.lon.toFixed(6)} (±${fmt(u.sigma.pos, 2)} m)</div>
      <div>Range</div><div>${fmt(u.range, 0)} m</div>
      <div>Lane</div><div>${u.lane ?? "–"} ${u.approach ? "(" + u.approach + ")" : ""}</div>
      <div>To stop line</div><div>${u.dist_to_stop !== undefined ? fmt(u.dist_to_stop) + " m" : "–"}</div>
      <div>ETA (kinematic)</div><div>${u.eta !== undefined ? fmt(u.eta, 2) + " s ± " + fmt(u.eta_sigma, 2) : u.stopping ? "stopping before line" : "–"}</div>
      <div>ETA (const. v)</div><div>${u.eta_cv !== undefined ? fmt(u.eta_cv, 2) + " s" : "–"}</div>
      <div>Confidence</div><div>${fmt(u.confidence * 100, 0)}% ${u.occluded ? "· occluded" : ""}</div>
      <div>Tracked for</div><div>${fmt(u.age)} s</div>
    </div>`;
}

$("#users").addEventListener("click", (e) => {
  const tr = (e.target as HTMLElement).closest("tr");
  if (tr?.dataset.id) scene.select(Number(tr.dataset.id));
});
scene.onSelect = () => renderUsers();

function renderLanes() {
  if (!lastDetectors) return;
  $("#lanes tbody").innerHTML = lastDetectors.lanes
    .map((l) => `<tr><td>${l.id}</td><td>${l.users}</td><td>${l.queue_count}</td><td>${fmt(l.queue_length, 0)} m</td><td>${l.crossings}</td><td>${fmt(l.mean_speed * 3.6, 0)}</td></tr>`)
    .join("");
  $("#zones").innerHTML = lastDetectors.zones
    .map((z) => `<span class="zone ${z.occupied ? "on" : ""}" title="occupancy ${(z.occupancy * 100).toFixed(0)}% · count ${z.count}">${z.id}</span>`)
    .join("");
}

function renderHealth(m: HealthMsg) {
  const h = m.health;
  const lvl = $("#level");
  lvl.textContent = h.level.toUpperCase();
  lvl.className = `pill big ${h.level}`;
  $("#confidence").textContent = `${Math.round(h.confidence * 100)}%`;
  ($("#confbar") as HTMLDivElement).style.width = `${Math.round(h.confidence * 100)}%`;
  $("#reasons").innerHTML = h.reasons.length ? h.reasons.map((r) => `<li>${r}</li>`).join("") : `<li class="muted">All systems nominal</li>`;
  const v = h.visibility, s = h.stability, c = h.calibration;
  $("#metrics").innerHTML = `
    <div>Pipeline</div><div>${fmt(h.pipeline.fps)} Hz · ${fmt(h.pipeline.latency_ms, 0)} ms · det ${fmt(h.pipeline.detector_ms)} ms</div>
    <div>Camera</div><div>${s ? `${s.status} · jitter ${fmt(s.jitter_px)} px · drift ${fmt(s.drift_deg, 2)}°` : "–"}</div>
    <div>Visibility</div><div>${v ? `${Math.round(v.score * 100)}% ${v.conditions.join(", ")}` : "–"}</div>
    <div>Detection range</div><div>${v ? `${fmt(v.effective_range_m, 0)} m (baseline ${fmt(v.baseline_range_m, 0)} m)` : "–"}</div>
    <div>Calibration</div><div>${c ? `${c.status} · size ${fmt(c.size_ratio, 2)} · ${c.samples} samples` : h.calibrated ? "ok" : "in progress"}</div>`;
  $("#coverage").innerHTML = (h.coverage ?? [])
    .map((z) => {
      const pct = (x: number) => `${Math.round(x * 100)}%`;
      return `<div class="cov"><span>${z.zone}</span><div class="bar">
        <i style="width:${pct(z.visible)}" class="v"></i><i style="width:${pct(z.dynamic_occluded)}" class="d"></i>
        <i style="width:${pct(z.static_occluded)}" class="s"></i><i style="width:${pct(z.unobservable)}" class="u"></i></div></div>`;
    })
    .join("");
}

function renderEvents() {
  $("#events").innerHTML = events
    .slice(0, 25)
    .map((e) => {
      const m = e.metrics;
      const parts = [
        m.ttc !== undefined ? `TTC ${fmt(m.ttc)}s` : "",
        m.mttc !== undefined ? `MTTC ${fmt(m.mttc)}s` : "",
        m.pet !== undefined ? `PET ${fmt(m.pet)}s` : "",
        m.drac ? `DRAC ${fmt(m.drac)}m/s²` : "",
        m.min_accel !== undefined ? `a ${fmt(m.min_accel)}m/s²` : "",
      ].filter(Boolean);
      const who = e.b ? `${icons[e.a.class]}#${e.a.id} ↔ ${icons[e.b.class]}#${e.b.id}` : `${icons[e.a.class]}#${e.a.id}`;
      const time = new Date(e.start * 1000).toLocaleTimeString();
      return `<li class="${e.severity}"><b>${e.kind.replace("_", " ")}</b> ${who}<br><span class="muted">${time} · ${parts.join(" · ")}</span></li>`;
    })
    .join("");
}

function renderCalib() {
  if (!calibMsg) return;
  renderCalibration($("#calib"), calibMsg, snapshot, mapMsg);
}

// ------------------------------------------------------------------ controls

for (const b of document.querySelectorAll<HTMLButtonElement>("[data-view]"))
  b.addEventListener("click", () => scene.view(b.dataset.view as "orbit" | "top" | "camera"));
$("#occ").addEventListener("change", (e) => {
  showOcc = (e.target as HTMLInputElement).checked;
  scene.setOcclusion(lastOcc, showOcc);
});
$("#labels").addEventListener("change", (e) => (scene.showAllLabels = (e.target as HTMLInputElement).checked));
for (const t of document.querySelectorAll<HTMLButtonElement>("[data-tab]"))
  t.addEventListener("click", () => {
    document.querySelectorAll(".tab").forEach((x) => x.classList.remove("active"));
    document.querySelectorAll("[data-tab]").forEach((x) => x.classList.remove("active"));
    $(`#tab-${t.dataset.tab}`).classList.add("active");
    t.classList.add("active");
    if (t.dataset.tab === "calib") renderCalib();
  });
setInterval(() => {
  $("#clock").textContent = new Date().toLocaleTimeString();
  $("#cam").textContent = site ? `${site} / ${cameraId}` : "";
}, 500);
