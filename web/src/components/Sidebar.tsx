import { AlertTriangle, Gauge, Navigation, X } from "lucide-react";
import { useMemo } from "react";
import { CLASS_COLOR, CLASS_LABEL, useStore } from "../store";
import type { RoadUser } from "../types";

const COMPASS = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"];
const compass = (deg: number) => COMPASS[Math.round((((deg % 360) + 360) % 360) / 45) % 8];
const f1 = (v?: number) => (v === undefined || !isFinite(v) ? "–" : v.toFixed(1));
const f0 = (v?: number) => (v === undefined || !isFinite(v) ? "–" : v.toFixed(0));

function Dot({ cls }: { cls: string }) {
  return <span className="inline-block h-2 w-2 shrink-0 rounded-full" style={{ background: CLASS_COLOR[cls] ?? "#999" }} />;
}

function Stat({ label, value, unit, sub }: { label: string; value: string; unit?: string; sub?: string }) {
  return (
    <div className="min-w-0">
      <div className="text-[11px] font-medium text-muted">{label}</div>
      <div className="flex items-baseline gap-1">
        <span className="text-2xl font-semibold tabular-nums tracking-tight">{value}</span>
        {unit && <span className="text-xs text-muted">{unit}</span>}
      </div>
      {sub && <div className="truncate text-[11px] text-muted">{sub}</div>}
    </div>
  );
}

function Overview({ users }: { users: RoadUser[] }) {
  const byClass = useMemo(() => {
    const m = new Map<string, number>();
    for (const u of users) m.set(u.class, (m.get(u.class) ?? 0) + 1);
    return [...m.entries()].sort((a, b) => b[1] - a[1]);
  }, [users]);
  const moving = users.filter((u) => Math.abs(u.speed) > 1.0 && u.class !== "pedestrian");
  const avg = moving.length ? (moving.reduce((s, u) => s + Math.abs(u.speed_kmh), 0) / moving.length) : undefined;
  const max = moving.length ? Math.max(...moving.map((u) => Math.abs(u.speed_kmh))) : undefined;
  const queued = users.filter((u) => u.queued).length;
  return (
    <section className="card p-4">
      <div className="card-title mb-3">Right now</div>
      <div className="grid grid-cols-3 gap-3">
        <Stat label="Road users" value={String(users.length)} sub={`${moving.length} moving`} />
        <Stat label="Avg. speed" value={f0(avg)} unit="km/h" sub={max !== undefined ? `max ${f0(max)} km/h` : "—"} />
        <Stat label="Queued" value={String(queued)} sub="at stop lines" />
      </div>
      {byClass.length > 0 && (
        <div className="mt-3 flex flex-wrap gap-1.5">
          {byClass.map(([c, n]) => (
            <span key={c} className="chip text-fg">
              <Dot cls={c} /> {CLASS_LABEL[c] ?? c} <b className="tabular-nums">{n}</b>
            </span>
          ))}
        </div>
      )}
    </section>
  );
}

function Lanes() {
  const lanes = useStore((s) => s.detectors?.lanes);
  const map = useStore((s) => s.map?.map);
  if (!lanes?.length || !map) return null;
  const approach = new Map(map?.lanes.map((l) => [l.id, l]) ?? []);
  const inbound = lanes.filter((l) => approach.get(l.id)?.to_junction !== false);
  return (
    <section className="card p-4">
      <div className="card-title mb-2">Lanes</div>
      <table className="w-full text-xs">
        <thead className="text-muted">
          <tr className="text-left">
            <th className="py-1 font-medium">Lane</th>
            <th className="font-medium">Users</th>
            <th className="font-medium">Queue</th>
            <th className="font-medium">km/h</th>
            <th className="text-right font-medium">Crossed</th>
          </tr>
        </thead>
        <tbody className="tabular-nums">
          {inbound.map((l) => (
            <tr key={l.id} className="border-t border-line">
              <td className="py-1.5">
                {l.id}
                <span className="ml-1 text-muted">{approach.get(l.id)?.approach}</span>
              </td>
              <td>{l.users}</td>
              <td>{l.queue_count ? `${l.queue_count} · ${l.queue_length.toFixed(0)} m` : "–"}</td>
              <td>{l.users ? f0(l.mean_speed * 3.6) : "–"}</td>
              <td className="text-right">{l.crossings}</td>
            </tr>
          ))}
        </tbody>
      </table>
    </section>
  );
}

function Selected({ u }: { u: RoadUser }) {
  const set = useStore((s) => s.set);
  const decel = u.decel_required;
  const hard = decel !== undefined && decel > 3.5 && !u.stopping;
  const rows: [string, string][] = [
    ["Speed", `${f0(Math.abs(u.speed_kmh))} km/h`],
    ["Acceleration", `${u.accel >= 0 ? "+" : ""}${u.accel.toFixed(2)} m/s²`],
    ["Heading", `${f0(u.heading)}° ${compass(u.heading)}`],
    ["Size (L×W×H)", `${f1(u.length)} × ${f1(u.width)} × ${f1(u.height)} m`],
    ["Lane", u.lane ? `${u.lane}${u.approach ? ` · ${u.approach}` : ""}` : "–"],
    ["To stop line", u.dist_to_stop !== undefined ? `${f1(u.dist_to_stop)} m` : "–"],
    ["Arrives in", u.eta !== undefined && (u.dist_to_stop ?? 0) > 0 ? `${f1(u.eta)} s` : "–"],
    ["Distance to camera", `${f0(u.range)} m`],
    ["Position", `${u.lat.toFixed(6)}, ${u.lon.toFixed(6)}`],
    ["Tracked for", `${f0(u.age)} s · ${u.state}`],
    ["Confidence", `${Math.round(u.confidence * 100)} %`],
  ];
  return (
    <section className="card overflow-hidden">
      <div className="flex items-center gap-2 border-b border-line bg-panel2 px-4 py-2.5">
        <Dot cls={u.class} />
        <span className="font-semibold">{CLASS_LABEL[u.class]}</span>
        <span className="text-muted">#{u.id}</span>
        <div className="flex-1" />
        <button className="btn-ghost px-1.5 py-1" aria-label="Close" onClick={() => set({ selected: null })}>
          <X size={14} />
        </button>
      </div>
      <div className="grid grid-cols-2 gap-3 px-4 pt-3">
        <Stat label="Speed" value={f0(Math.abs(u.speed_kmh))} unit="km/h" sub={`± ${f1(u.sigma.speed * 3.6)} km/h`} />
        <Stat label="Acceleration" value={`${u.accel >= 0 ? "+" : ""}${u.accel.toFixed(1)}`} unit="m/s²" sub={`± ${f1(u.sigma.accel)}`} />
      </div>
      {decel !== undefined && (
        <div className={`mx-4 mt-3 flex items-center gap-2 rounded-lg px-3 py-2 text-xs ${hard ? "bg-bad/10 text-bad" : "bg-panel2 text-muted"}`}>
          {hard ? <AlertTriangle size={14} /> : <Gauge size={14} />}
          Needs {decel.toFixed(1)} m/s² to stop at the line{u.stopping ? " · braking" : hard ? " · unlikely to stop" : ""}
        </div>
      )}
      <dl className="grid grid-cols-[auto_1fr] gap-x-4 gap-y-1.5 px-4 py-3 text-xs">
        {rows.map(([k, v]) => (
          <div key={k} className="contents">
            <dt className="text-muted">{k}</dt>
            <dd className="text-right tabular-nums">{v}</dd>
          </div>
        ))}
      </dl>
    </section>
  );
}

function ObjectList({ users }: { users: RoadUser[] }) {
  const selected = useStore((s) => s.selected);
  const set = useStore((s) => s.set);
  const sorted = useMemo(() => [...users].sort((a, b) => a.range - b.range), [users]);
  return (
    <section className="card flex min-h-0 flex-col">
      <div className="flex items-center justify-between px-4 pb-2 pt-3">
        <span className="card-title">Road users</span>
        <span className="text-[11px] text-muted">click to follow</span>
      </div>
      <div className="min-h-0 flex-1 overflow-auto">
        <table className="w-full text-xs">
          <thead className="sticky top-0 bg-panel text-muted">
            <tr className="text-left">
              <th className="py-1 pl-4 font-medium">Type</th>
              <th className="font-medium">km/h</th>
              <th className="font-medium">m/s²</th>
              <th className="font-medium">Lane</th>
              <th className="pr-4 text-right font-medium">Stop line</th>
            </tr>
          </thead>
          <tbody className="tabular-nums">
            {sorted.map((u) => (
              <tr
                key={u.id}
                className={`cursor-pointer border-t border-line hover:bg-panel2 ${selected === u.id ? "bg-accent/10" : ""} ${u.state === "predicted" ? "opacity-60" : ""}`}
                onClick={() => set({ selected: selected === u.id ? null : u.id })}
              >
                <td className="py-1.5 pl-4">
                  <span className="flex items-center gap-1.5">
                    <Dot cls={u.class} />
                    {CLASS_LABEL[u.class]}
                    <span className="text-muted">#{u.id}</span>
                  </span>
                </td>
                <td>{f0(Math.abs(u.speed_kmh))}</td>
                <td className={u.accel < -2 ? "text-warn" : ""}>{u.accel.toFixed(1)}</td>
                <td>{u.lane ?? "–"}</td>
                <td className="pr-4 text-right">
                  {u.dist_to_stop !== undefined && u.dist_to_stop > 0 ? `${f0(u.dist_to_stop)} m${u.eta !== undefined ? ` · ${f1(u.eta)} s` : ""}` : "–"}
                </td>
              </tr>
            ))}
            {!sorted.length && (
              <tr>
                <td colSpan={5} className="px-4 py-6 text-center text-muted">
                  No road users in view
                </td>
              </tr>
            )}
          </tbody>
        </table>
      </div>
    </section>
  );
}

function Events() {
  const events = useStore((s) => s.events);
  if (!events.length) return null;
  return (
    <section className="card p-4">
      <div className="card-title mb-2">Safety events</div>
      <ul className="space-y-1.5 text-xs">
        {events.slice(0, 6).map((e) => (
          <li key={e.event_id} className="flex items-center gap-2">
            <span className={`h-2 w-2 rounded-full ${e.severity === "critical" ? "bg-bad" : "bg-warn"}`} />
            <span className="font-medium">{e.kind.replace(/_/g, " ")}</span>
            <span className="text-muted">
              #{e.a.id}
              {e.b ? ` × #${e.b.id}` : ""}
            </span>
            <span className="ml-auto text-muted tabular-nums">
              {e.metrics.ttc !== undefined ? `TTC ${e.metrics.ttc.toFixed(1)} s` : e.metrics.pet !== undefined ? `PET ${e.metrics.pet.toFixed(1)} s` : ""}
            </span>
          </li>
        ))}
      </ul>
    </section>
  );
}

export function Sidebar() {
  const objects = useStore((s) => s.objects);
  const selected = useStore((s) => s.selected);
  const calibrated = useStore((s) => !!s.calibration);
  const users = useMemo(() => objects.filter((o) => o.state !== "tentative"), [objects]);
  const sel = users.find((u) => u.id === selected);
  return (
    <aside className="flex min-h-0 w-full flex-col gap-3 overflow-y-auto border-l border-line bg-bg p-3 lg:w-[380px]">
      {!calibrated && (
        <a href="#/settings/calibration" className="card flex items-center gap-3 border-warn/40 bg-warn/5 p-3 text-xs">
          <Navigation size={16} className="text-warn" />
          <span>
            <b className="block text-fg">Calibrate the camera</b>
            <span className="text-muted">Detections appear in the twin once the camera is placed on the map.</span>
          </span>
        </a>
      )}
      <Overview users={users} />
      {sel && <Selected u={sel} />}
      <Lanes />
      <ObjectList users={users} />
      <Events />
    </aside>
  );
}
