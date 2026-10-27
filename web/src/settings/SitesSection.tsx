import L from "leaflet";
import { Eye, Play, Plus, Search, Square, Trash2 } from "lucide-react";
import { useEffect, useRef, useState } from "react";
import { navigate } from "../App";
import { api, geocode, json, managerHealth, uploadVideo, type ManagerHealth, type SiteInfo } from "../api";
import { useBaseMap, pin } from "../components/leaflet";
import { bearingDeg } from "../geo";
import { selectSite } from "../mqtt";
import { useStore } from "../store";
import { Page } from "./SettingsPage";

export const LENSES: { v: string; label: string; hint: string }[] = [
  { v: "phone_wide", label: "Phone, main camera (1×)", hint: "~66° field of view" },
  { v: "phone_2x", label: "Phone, 2× zoom", hint: "~37°" },
  { v: "phone_3x", label: "Phone, 3× / telephoto", hint: "~25°" },
  { v: "cctv", label: "Traffic / CCTV camera", hint: "varifocal, wide range" },
  { v: "unknown", label: "Unknown", hint: "the calibration finds it" },
];

// Camera position + viewing direction on the aerial map: first click = camera, second = a point it looks at.
export function CameraPlacer({ value, onChange }: {
  value: { lat: number; lon: number; bearing: number } | null;
  onChange: (v: { lat: number; lon: number; bearing: number } | null) => void;
}) {
  const ref = useRef<HTMLDivElement>(null);
  const [start] = useState<[number, number]>(() => (value ? [value.lat, value.lon] : [48.1372, 11.5756]));
  const map = useBaseMap(ref, start, value ? 19 : 13);
  const layer = useRef(L.layerGroup());
  const stage = useRef<{ cam: L.LatLng | null; look: L.LatLng | null }>({ cam: value ? L.latLng(value.lat, value.lon) : null, look: null });
  const [q, setQ] = useState("");
  const [msg, setMsg] = useState("");
  const draw = () => {
    const l = layer.current;
    l.clearLayers();
    const { cam, look } = stage.current;
    if (!cam) return;
    L.marker(cam, { icon: pin("#3e6ae1", "", 16) }).bindTooltip("camera").addTo(l);
    const b = look ? bearingDeg(cam, look) : value?.bearing;
    if (b !== undefined) {
      const r = 60, rad = (b * Math.PI) / 180;
      const tip = L.latLng(cam.lat + (r * Math.cos(rad)) / 111320, cam.lng + (r * Math.sin(rad)) / (111320 * Math.cos((cam.lat * Math.PI) / 180)));
      L.polyline([cam, tip], { color: "#ff9f43", weight: 3, dashArray: "6 6" }).addTo(l);
    }
  };
  useEffect(() => {
    if (!map) return;
    layer.current.addTo(map);
    draw();
    map.on("click", (e: L.LeafletMouseEvent) => {
      const s = stage.current;
      if (!s.cam || s.look) {
        s.cam = e.latlng;
        s.look = null;
        onChange(null);
      } else {
        s.look = e.latlng;
        onChange({ lat: s.cam.lat, lon: s.cam.lng, bearing: bearingDeg(s.cam, s.look) });
      }
      draw();
    });
    return () => {
      map.off("click");
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [map]);
  const search = async () => {
    if (!q.trim() || !map) return;
    try {
      const hit = await geocode(q);
      if (!hit) return setMsg(`Nothing found for “${q}”.`);
      map.setView([hit.lat, hit.lon], 18);
      setMsg(hit.name);
    } catch {
      setMsg("Address search is not reachable; zoom the map by hand.");
    }
  };
  return (
    <div className="space-y-2">
      <div className="flex gap-2">
        <input className="input" placeholder="Search an address or place…" value={q} onChange={(e) => setQ(e.target.value)} onKeyDown={(e) => e.key === "Enter" && search()} />
        <button className="btn" onClick={search}>
          <Search size={14} /> Find
        </button>
      </div>
      <div ref={ref} className="h-80 overflow-hidden rounded-lg border border-line cursor-crosshair" />
      <div className="text-xs text-muted">
        {msg ||
          (stage.current.cam && !stage.current.look && !value
            ? "Now click a point the camera looks at (e.g. the middle of the junction)."
            : "Click where the camera is, then a point it looks at.")}
        {value && ` · camera ${value.lat.toFixed(6)}, ${value.lon.toFixed(6)} · looking ${value.bearing.toFixed(0)}°`}
      </div>
    </div>
  );
}

function NewSite({ mgr, onDone }: { mgr: ManagerHealth; onDone: () => void }) {
  const [name, setName] = useState("");
  const [kind, setKind] = useState<"video" | "stream">("video");
  const [file, setFile] = useState<File | null>(null);
  const [url, setUrl] = useState("");
  const [height, setHeight] = useState(7);
  const [lens, setLens] = useState("phone_wide");
  const [device, setDevice] = useState<"auto" | "gpu" | "cpu">("auto");
  const [place, setPlace] = useState<{ lat: number; lon: number; bearing: number } | null>(null);
  const [msg, setMsg] = useState<{ t: string; bad?: boolean } | null>(null);
  const [busy, setBusy] = useState(false);
  const create = async () => {
    if (!name.trim()) return setMsg({ t: "Give the site a name.", bad: true });
    if (!place) return setMsg({ t: "Place the camera on the map (camera, then a point it looks at).", bad: true });
    const body: Record<string, unknown> = { name: name.trim(), lat: place.lat, lon: place.lon, height_m: height, bearing_deg: place.bearing, lens, device };
    setBusy(true);
    try {
      if (kind === "video") {
        if (!file) throw new Error("Choose a video file.");
        body.video = await uploadVideo(file, (p) => setMsg({ t: `Uploading ${p} %…` }));
      } else {
        if (!/^(rtsp|rtsps|http|https):\/\//.test(url.trim())) throw new Error("The stream URL must start with rtsp:// or http(s)://");
        body.stream = url.trim();
      }
      setMsg({ t: "Starting the site… (the first GPU start builds the TensorRT engine, about a minute)" });
      const s = await api<SiteInfo>("/sites", json("POST", body));
      selectSite(s.id);
      onDone();
      navigate("settings/calibration");
    } catch (e) {
      setMsg({ t: (e as Error).message, bad: true });
    } finally {
      setBusy(false);
    }
  };
  return (
    <section className="card space-y-4 p-5">
      <div className="text-base font-semibold">New site</div>
      <div className="grid gap-4 md:grid-cols-2">
        <div>
          <label className="label">Name</label>
          <input className="input" placeholder="e.g. Georg-Brauchle-Ring east" value={name} onChange={(e) => setName(e.target.value)} />
        </div>
        <div>
          <label className="label">Source</label>
          <div className="seg">
            <button data-on={kind === "video"} onClick={() => setKind("video")}>Video file</button>
            <button data-on={kind === "stream"} onClick={() => setKind("stream")}>Live stream (RTSP)</button>
          </div>
        </div>
        <div className="md:col-span-2">
          {kind === "video" ? (
            <input type="file" accept="video/*" className="input" onChange={(e) => setFile(e.target.files?.[0] ?? null)} />
          ) : (
            <input className="input" placeholder="rtsp://user:password@camera-ip:554/stream1" value={url} onChange={(e) => setUrl(e.target.value)} />
          )}
          <div className="mt-1 text-xs text-muted">
            {kind === "video" ? "A recorded video plays in a loop, like a live camera." : "The site reconnects automatically if the stream drops."}
          </div>
        </div>
        <div>
          <label className="label">Camera height above the road (estimate)</label>
          <div className="flex items-center gap-2">
            <input type="number" className="input w-28" min={1} max={100} step={0.5} value={height} onChange={(e) => setHeight(Number(e.target.value) || 7)} />
            <span className="text-sm text-muted">m</span>
          </div>
        </div>
        <div>
          <label className="label">Lens</label>
          <select className="input" value={lens} onChange={(e) => setLens(e.target.value)}>
            {LENSES.map((l) => (
              <option key={l.v} value={l.v}>
                {l.label} ({l.hint})
              </option>
            ))}
          </select>
        </div>
        <div>
          <label className="label">Compute</label>
          <div className="seg">
            {(["auto", "gpu", "cpu"] as const).map((d) => (
              <button key={d} data-on={device === d} disabled={d === "gpu" && !mgr.gpu} onClick={() => setDevice(d)}>
                {d === "auto" ? "Auto" : d.toUpperCase()}
              </button>
            ))}
          </div>
        </div>
      </div>
      <div>
        <label className="label">Camera position and viewing direction</label>
        <CameraPlacer value={place} onChange={setPlace} />
      </div>
      <div className="flex items-center gap-3">
        <button className="btn-primary" disabled={busy} onClick={create}>
          <Plus size={15} /> Create and start
        </button>
        {msg && <span className={`text-xs ${msg.bad ? "text-bad" : "text-muted"}`}>{msg.t}</span>}
      </div>
    </section>
  );
}

export function SitesSection() {
  const current = useStore((s) => s.site);
  const [mgr, setMgr] = useState<ManagerHealth | null | undefined>(undefined);
  const [sites, setSites] = useState<SiteInfo[]>([]);
  const [adding, setAdding] = useState(false);
  const [err, setErr] = useState("");
  const refresh = () => api<SiteInfo[]>("/sites").then(setSites).catch((e) => setErr(e.message));
  useEffect(() => {
    managerHealth().then((h) => {
      setMgr(h);
      if (h) refresh();
    });
  }, []);
  if (mgr === undefined) return <Page title="Sites"><div className="text-sm text-muted">Loading…</div></Page>;
  if (mgr === null)
    return (
      <Page title="Sites">
        <div className="card p-5 text-sm text-muted">This installation runs a single camera configured on the device; site management is not available.</div>
      </Page>
    );
  const act = async (s: SiteInfo, a: string) => {
    try {
      if (a === "delete") {
        if (!confirm(`Delete site “${s.name}” with its video, calibration and drawn map?`)) return;
        await api(`/sites/${s.id}`, { method: "DELETE" });
      } else await api(`/sites/${s.id}/${a}`, { method: "POST" });
      await refresh();
    } catch (e) {
      setErr((e as Error).message);
    }
  };
  return (
    <Page title="Sites" desc={`Each site is one camera. Up to ${mgr.max_sites} can run at the same time.`}>
      <section className="card overflow-hidden">
        <table className="w-full text-sm">
          <thead className="bg-panel2 text-xs text-muted">
            <tr className="text-left">
              <th className="px-4 py-2 font-medium">Name</th>
              <th className="font-medium">Source</th>
              <th className="font-medium">State</th>
              <th className="font-medium">Calibrated</th>
              <th className="font-medium">Compute</th>
              <th />
            </tr>
          </thead>
          <tbody>
            {sites.map((s) => (
              <tr key={s.id} className={`border-t border-line ${s.id === current ? "bg-accent/5" : ""}`}>
                <td className="px-4 py-2.5 font-medium">{s.name}</td>
                <td className="max-w-[220px] truncate text-xs text-muted" title={s.source}>
                  {s.kind === "video" ? "Video · " : "Stream · "}
                  {s.source}
                </td>
                <td>
                  <span className={`chip ${s.status === "running" ? "border-ok/30 bg-ok/10 text-ok" : ""}`}>{s.status}</span>
                </td>
                <td className="text-xs">{s.calibrated ? "yes" : <span className="text-warn">no</span>}</td>
                <td className="text-xs uppercase text-muted">{s.device}</td>
                <td className="whitespace-nowrap pr-3 text-right">
                  <button className="btn-ghost px-2" title="Open" onClick={() => { selectSite(s.id); navigate("live"); }}>
                    <Eye size={15} />
                  </button>
                  {s.status === "running" ? (
                    <button className="btn-ghost px-2" title="Stop" onClick={() => act(s, "stop")}>
                      <Square size={15} />
                    </button>
                  ) : (
                    <button className="btn-ghost px-2" title="Start" onClick={() => act(s, "start")}>
                      <Play size={15} />
                    </button>
                  )}
                  <button className="btn-ghost px-2 text-bad" title="Delete" onClick={() => act(s, "delete")}>
                    <Trash2 size={15} />
                  </button>
                </td>
              </tr>
            ))}
            {!sites.length && (
              <tr>
                <td colSpan={6} className="px-4 py-6 text-center text-muted">No sites yet.</td>
              </tr>
            )}
          </tbody>
        </table>
      </section>
      {err && <div className="text-xs text-bad">{err}</div>}
      {adding ? (
        <NewSite mgr={mgr} onDone={() => { setAdding(false); refresh(); }} />
      ) : (
        <button className="btn-primary" onClick={() => setAdding(true)}>
          <Plus size={15} /> New site
        </button>
      )}
    </Page>
  );
}
