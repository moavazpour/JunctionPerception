// Step 3: reference points, lines and verticals matched between the camera image and the aerial map.
import L from "leaflet";
import { Camera, Crosshair, Minus, RefreshCw, Spline, TrafficCone, Trash2, Undo2 } from "lucide-react";
import { useEffect, useMemo, useRef } from "react";
import { pin, useBaseMap } from "../components/leaflet";
import { enuToGeo, GroundCamera } from "../geo";
import { calibrationCommand, publish } from "../mqtt";
import { useStore } from "../store";
import type { SignalHead, SolveReport, UserMap } from "../types";
import { useDraft, type Draft, type Tool } from "./draft";
import { ImageCanvas, type Handle, type Poly } from "./ImageCanvas";
import { drawLayout } from "./LayoutEditor";

export const PAIR_COLORS = ["#ff4d4d", "#3ddc84", "#4e8cff", "#ffb020", "#c86bff", "#36d6e7", "#ff7ab6", "#b8e04a", "#ffffff", "#ff9f43"];
export const pc = (i: number) => PAIR_COLORS[i % PAIR_COLORS.length];
export const LINE_COLOR = "#ffd43b";
export const VERT_COLOR = "#36d6e7";

// Colour of a residual relative to the precision clicking can reach at that distance (1 = as expected).
export function errColor(norm?: number) {
  if (norm === undefined) return "#9aa1aa";
  return norm <= 1.5 ? "#34c759" : norm <= 3 ? "#ffb020" : "#ff453a";
}

// Residuals of the last solve, by kind and position in the *complete* lists sent to the device.
export function residualIndex(d: Draft, r?: SolveReport) {
  const pts = new Map<number, { px: number; m: number; norm?: number }>();
  const lines = new Map<number, { px: number; m: number; norm?: number }>();
  if (!r?.residuals) return { pts, lines };
  const ci: number[] = [];
  d.points.forEach((p, i) => p.u !== undefined && p.lat !== undefined && ci.push(i));
  const li: number[] = [];
  d.lines.forEach((l, i) => l.px.length >= 2 && l.geo.length === 2 && li.push(i));
  for (const e of r.residuals) {
    if (e.kind === "point" && ci[e.index] !== undefined) pts.set(ci[e.index], e);
    if (e.kind === "line" && li[e.index] !== undefined) lines.set(li[e.index], e);
  }
  return { pts, lines };
}

function guidance(d: Draft): string {
  if (d.tool === "light") return "Drag a box around a traffic light head in the image, then pick the stop line it controls below. Its colour is read live from the image.";
  if (d.tool === "point") {
    const last = d.points[d.points.length - 1];
    if (last && last.u !== undefined && last.lat === undefined) return `Now click the same spot on the map (point ${d.points.length}).`;
    return "Click a sharp spot on the road surface in the camera image: a corner of a lane marking, an end of the stop line, a kerb corner.";
  }
  if (d.tool === "line") {
    const l = d.openLine !== null ? d.lines[d.openLine] : null;
    if (l && l.geo.length === 0 && l.px.length < 2) return "Keep clicking along the same straight marking in the image (two or more points, as far apart as possible).";
    if (l && l.geo.length < 2) return `Now click two points of the same marking on the map (${l.geo.length}/2). Image clicks still add points to the line.`;
    return "Click on a long straight marking in the image (lane line, kerb, stop line) to start a line.";
  }
  const v = d.openVertical !== null ? d.verticals[d.openVertical] : null;
  return v && v.length === 1 ? "Now click the top of the same pole or edge." : "Click the bottom of a vertical pole or building edge, then its top.";
}

export function PairsEditor({ camera, report }: { camera?: GroundCamera; report?: SolveReport }) {
  const d = useDraft();
  const snapshot = useStore((s) => s.snapshot);
  const config = useStore((s) => s.config);
  const userMap = useStore((s) => s.userMap);
  const res = residualIndex(d, report);

  // ------------------------------------------------------------------ image side
  const onPlace = (u: number, v: number) => {
    const s = useDraft.getState();
    if (s.tool === "point") {
      const pts = [...s.points];
      const last = pts[pts.length - 1];
      if (last && last.u !== undefined && last.lat === undefined) pts[pts.length - 1] = { ...last, u, v };
      else pts.push({ u, v });
      s.set({ points: pts });
    } else if (s.tool === "line") {
      const lines = s.lines.map((l) => ({ px: [...l.px], geo: [...l.geo] }));
      if (s.openLine !== null && lines[s.openLine] && lines[s.openLine].geo.length < 2) {
        lines[s.openLine].px.push([u, v]);
        s.set({ lines });
      } else {
        lines.push({ px: [[u, v]], geo: [] });
        s.set({ lines, openLine: lines.length - 1 });
      }
    } else {
      const vs = s.verticals.map((x) => [...x]);
      if (s.openVertical !== null && vs[s.openVertical]?.length === 1) {
        vs[s.openVertical].push([u, v]);
        s.set({ verticals: vs, openVertical: null });
      } else {
        vs.push([[u, v]]);
        s.set({ verticals: vs, openVertical: vs.length - 1 });
      }
    }
  };
  const onMove = (id: string, u: number, v: number) => {
    const s = useDraft.getState();
    const [k, a, b] = id.split(":");
    const i = Number(a), j = Number(b);
    if (k === "p") s.set({ points: s.points.map((p, n) => (n === i ? { ...p, u, v } : p)) });
    if (k === "l") s.set({ lines: s.lines.map((l, n) => (n === i ? { ...l, px: l.px.map((q, m) => (m === j ? [u, v] : q)) } : l)) });
    if (k === "v") s.set({ verticals: s.verticals.map((x, n) => (n === i ? x.map((q, m) => (m === j ? [u, v] : q)) : x)) });
  };

  const { handles, polys } = useMemo(() => {
    const h: Handle[] = [];
    const p: Poly[] = [];
    d.points.forEach((q, i) => {
      if (q.u === undefined || q.v === undefined) return;
      h.push({ id: `p:${i}`, u: q.u, v: q.v, color: pc(i), label: String(i + 1), ring: q.lat === undefined });
      // Where the solved camera sees the map point: a short residual line.
      if (camera && q.lat !== undefined && q.lon !== undefined) {
        const pr = camera.projectGeo(q.lat, q.lon);
        if (pr) p.push({ pts: [[q.u, q.v], pr], color: errColor(res.pts.get(i)?.norm), width: 2 });
      }
    });
    d.lines.forEach((l, i) => {
      p.push({ pts: l.px, color: LINE_COLOR, width: 2.5, dashed: l.geo.length < 2 });
      l.px.forEach(([u, v], j) => h.push({ id: `l:${i}:${j}`, u, v, color: LINE_COLOR, ring: l.geo.length < 2 }));
      if (camera && l.geo.length === 2) {
        const a = camera.projectGeo(l.geo[0][0], l.geo[0][1]), b = camera.projectGeo(l.geo[1][0], l.geo[1][1]);
        if (a && b) {
          // The map line extended across the image.
          const dx = b[0] - a[0], dy = b[1] - a[1];
          p.push({ pts: [[a[0] - dx * 3, a[1] - dy * 3], [b[0] + dx * 3, b[1] + dy * 3]], color: errColor(res.lines.get(i)?.norm), width: 1, dashed: true });
        }
      }
    });
    d.verticals.forEach((v, i) => {
      if (v.length === 2) p.push({ pts: v as [number, number][], color: VERT_COLOR, width: 2.5 });
      v.forEach(([a, b], j) => h.push({ id: `v:${i}:${j}`, u: a, v: b, color: VERT_COLOR, ring: v.length < 2 }));
    });
    return { handles: h, polys: p };
  }, [d.points, d.lines, d.verticals, camera, report]);

  // ------------------------------------------------------------------ map side
  const ref = useRef<HTMLDivElement>(null);
  const center = useMemo<[number, number] | null>(() => (config ? [config.placement.lat, config.placement.lon] : null), [config?.placement.lat, config?.placement.lon]);
  const map = useBaseMap(ref, center, 19);
  const layer = useRef(L.layerGroup());
  const context = useRef(L.layerGroup());
  useEffect(() => {
    if (!map) return;
    map.doubleClickZoom.disable();
    context.current.addTo(map);
    layer.current.addTo(map);
    map.on("click", (e: L.LeafletMouseEvent) => {
      const s = useDraft.getState();
      const g: [number, number] = [e.latlng.lat, e.latlng.lng];
      if (s.tool === "point") {
        const pts = [...s.points];
        const last = pts[pts.length - 1];
        if (last && last.u !== undefined && last.lat === undefined) {
          pts[pts.length - 1] = { ...last, lat: g[0], lon: g[1] };
          s.set({ points: pts });
        }
      } else if (s.tool === "line" && s.openLine !== null) {
        const lines = s.lines.map((l) => ({ px: [...l.px], geo: [...l.geo] }));
        const l = lines[s.openLine];
        if (l && l.px.length >= 2 && l.geo.length < 2) {
          l.geo.push(g);
          s.set({ lines, openLine: l.geo.length === 2 ? null : s.openLine });
        }
      }
    });
    return () => {
      map.off("click");
    };
  }, [map]);
  useEffect(() => {
    context.current.clearLayers();
    if (userMap) drawLayout(userMap, context.current, { faint: true });
  }, [userMap, map]);
  useEffect(() => {
    const lg = layer.current;
    lg.clearLayers();
    d.points.forEach((q, i) => {
      if (q.lat === undefined || q.lon === undefined) return;
      const mk = L.marker([q.lat, q.lon], { icon: pin(pc(i), i + 1), draggable: true }).addTo(lg);
      mk.on("dragend", () => {
        const ll = mk.getLatLng();
        const s = useDraft.getState();
        s.set({ points: s.points.map((p, n) => (n === i ? { ...p, lat: ll.lat, lon: ll.lng } : p)) });
      });
      // Where the clicked image point lands on the ground under the solved camera.
      if (camera && q.u !== undefined && q.v !== undefined && camera.anchor) {
        const g = camera.unproject(q.u, q.v);
        if (g) {
          const ll = L.latLng(...enu2ll(camera, g));
          L.polyline([[q.lat, q.lon], ll], { color: errColor(res.pts.get(i)?.norm), weight: 2 }).addTo(lg);
          L.circleMarker(ll, { radius: 4, color: errColor(res.pts.get(i)?.norm), weight: 2, fillOpacity: 0 }).addTo(lg);
        }
      }
    });
    d.lines.forEach((l, i) => {
      if (l.geo.length) L.polyline(l.geo, { color: LINE_COLOR, weight: 3, dashArray: l.geo.length < 2 ? "4 4" : undefined }).addTo(lg);
      l.geo.forEach((g, j) => {
        const mk = L.marker(g, { icon: pin(LINE_COLOR, "", 12), draggable: true }).addTo(lg);
        mk.on("dragend", () => {
          const ll = mk.getLatLng();
          const s = useDraft.getState();
          s.set({ lines: s.lines.map((x, n) => (n === i ? { ...x, geo: x.geo.map((q, m) => (m === j ? [ll.lat, ll.lng] : q)) } : x)) });
        });
      });
    });
  }, [d.points, d.lines, camera, report, map]);

  // ------------------------------------------------------------------ controls
  const undo = () => {
    const s = useDraft.getState();
    if (s.tool === "point" && s.points.length) {
      const last = s.points[s.points.length - 1];
      if (last.lat !== undefined) s.set({ points: [...s.points.slice(0, -1), { u: last.u, v: last.v }] });
      else s.set({ points: s.points.slice(0, -1) });
    } else if (s.tool === "line" && s.lines.length) {
      const i = s.openLine ?? s.lines.length - 1;
      const l = s.lines[i];
      const nl = l.geo.length ? { ...l, geo: l.geo.slice(0, -1) } : { ...l, px: l.px.slice(0, -1) };
      const lines = nl.px.length ? s.lines.map((x, n) => (n === i ? nl : x)) : s.lines.filter((_, n) => n !== i);
      s.set({ lines, openLine: nl.px.length ? i : null });
    } else if (s.tool === "vertical" && s.verticals.length) {
      s.set({ verticals: s.verticals.slice(0, -1), openVertical: null });
    }
  };
  const TOOLS: { t: Tool; label: string; icon: JSX.Element }[] = [
    { t: "point", label: "Point pair", icon: <Crosshair size={14} /> },
    { t: "line", label: "Line", icon: <Spline size={14} /> },
    { t: "vertical", label: "Vertical", icon: <Minus size={14} className="rotate-90" /> },
    { t: "light", label: "Traffic light", icon: <TrafficCone size={14} /> },
  ];
  return (
    <div className="space-y-3">
      <div className="flex flex-wrap items-center gap-2">
        <div className="seg">
          {TOOLS.map((x) => (
            <button key={x.t} data-on={d.tool === x.t} onClick={() => d.set({ tool: x.t })}>
              <span className="flex items-center gap-1">{x.icon} {x.label}</span>
            </button>
          ))}
        </div>
        <button className="btn" onClick={undo}>
          <Undo2 size={14} /> Undo
        </button>
        <button className="btn" onClick={() => calibrationCommand({ action: "snapshot" }).catch(() => {})} title="Take a new camera image">
          <Camera size={14} /> New snapshot
        </button>
        <button
          className="btn-ghost text-bad"
          onClick={() => confirm("Remove all points, lines and verticals?") && d.set({ points: [], lines: [], verticals: [], openLine: null, openVertical: null })}
        >
          <Trash2 size={14} /> Clear
        </button>
        <span className="ml-auto text-xs text-muted">Wheel: zoom · drag: pan · drag a marker to correct it</span>
      </div>
      <div className="rounded-lg border border-accent/30 bg-accent/5 px-3 py-2 text-sm">{guidance(d)}</div>
      <div className="grid gap-3 lg:grid-cols-2">
        <ImageCanvas
          snapshot={snapshot}
          handles={d.tool === "light" ? [] : handles}
          polys={[...polys, ...signalPolys(userMap)]}
          onPlace={d.tool === "light" ? undefined : onPlace}
          onMove={d.tool === "light" ? undefined : onMove}
          onRect={d.tool === "light" ? addSignal : undefined}
          className="h-[58vh] min-h-[360px]"
        />
        <div ref={ref} className="h-[58vh] min-h-[360px] overflow-hidden rounded-lg border border-line cursor-crosshair" />
      </div>
      <SignalList />
      <PairList res={res} />
    </div>
  );
}

function enu2ll(cam: GroundCamera, g: [number, number]): [number, number] {
  return enuToGeo(cam.anchor!, g[0], g[1]);
}

function PairList({ res }: { res: ReturnType<typeof residualIndex> }) {
  const d = useDraft();
  const fmt = (e?: { px: number; m: number; norm?: number }) =>
    e ? (
      <span style={{ color: errColor(e.norm) }} className="font-medium tabular-nums">
        {e.m.toFixed(2)} m · {e.px.toFixed(1)} px
      </span>
    ) : (
      <span className="text-muted">–</span>
    );
  const rows = [
    ...d.points.map((p, i) => ({
      key: `p${i}`,
      sw: pc(i),
      name: `Point ${i + 1}`,
      img: p.u !== undefined,
      mapOk: p.lat !== undefined,
      err: res.pts.get(i),
      del: () => d.set({ points: d.points.filter((_, n) => n !== i) }),
    })),
    ...d.lines.map((l, i) => ({
      key: `l${i}`,
      sw: LINE_COLOR,
      name: `Line ${i + 1}`,
      img: l.px.length >= 2,
      mapOk: l.geo.length === 2,
      err: res.lines.get(i),
      del: () => d.set({ lines: d.lines.filter((_, n) => n !== i), openLine: null }),
    })),
    ...d.verticals.map((v, i) => ({
      key: `v${i}`,
      sw: VERT_COLOR,
      name: `Vertical ${i + 1}`,
      img: v.length === 2,
      mapOk: true,
      err: undefined,
      del: () => d.set({ verticals: d.verticals.filter((_, n) => n !== i), openVertical: null }),
    })),
  ];
  if (!rows.length) return null;
  return (
    <div className="card overflow-hidden">
      <table className="w-full text-xs">
        <thead className="bg-panel2 text-muted">
          <tr className="text-left">
            <th className="px-4 py-1.5 font-medium">Item</th>
            <th className="font-medium">Image</th>
            <th className="font-medium">Map</th>
            <th className="font-medium">Error after solving (ground · image)</th>
            <th />
          </tr>
        </thead>
        <tbody>
          {rows.map((r) => (
            <tr key={r.key} className="border-t border-line">
              <td className="px-4 py-1.5">
                <span className="flex items-center gap-2">
                  <span className="h-2.5 w-2.5 rounded-full" style={{ background: r.sw }} /> {r.name}
                </span>
              </td>
              <td>{r.img ? "✓" : <span className="text-warn">missing</span>}</td>
              <td>{r.mapOk ? "✓" : <span className="text-warn">missing</span>}</td>
              <td>{fmt(r.err)}</td>
              <td className="pr-2 text-right">
                <button className="btn-ghost px-1.5 py-0.5 text-bad" onClick={r.del}>
                  <Trash2 size={12} />
                </button>
              </td>
            </tr>
          ))}
        </tbody>
      </table>
      <div className="flex items-center gap-2 border-t border-line px-4 py-2 text-[11px] text-muted">
        <RefreshCw size={11} /> Errors refresh after each solve. Colours compare the error with what clicking can achieve at that distance (far points may be off by more metres): green as expected, amber larger, red: check that both clicks show the same spot.
      </div>
    </div>
  );
}

// ------------------------------------------------------------------ traffic lights

const EMPTY_MAP: UserMap = { lanes: [], stop_lines: [], crosswalks: [], roi: [], signals: [] };

function saveSignals(signals: SignalHead[]) {
  const st = useStore.getState();
  const m: UserMap = { ...(st.userMap ?? EMPTY_MAP), signals };
  st.set({ userMap: m });
  publish("map/user/set", { map: m });
}

function addSignal(u0: number, v0: number, u1: number, v1: number) {
  const m = useStore.getState().userMap ?? EMPTY_MAP;
  const cur = m.signals ?? [];
  let k = cur.length + 1;
  while (cur.some((s) => s.id === `S${k}`)) k++;
  saveSignals([...cur, { id: `S${k}`, box: [u0, v0, u1, v1], stop_line: m.stop_lines.length ? 0 : -1 }]);
}

function signalPolys(m?: UserMap): Poly[] {
  return (m?.signals ?? []).map((s) => {
    const [u0, v0, u1, v1] = s.box;
    return { pts: [[u0, v0], [u1, v0], [u1, v1], [u0, v1]], color: "#ffb020", width: 2, closed: true };
  });
}

function SignalList() {
  const m = useStore((s) => s.userMap);
  const states = useStore((s) => s.signals?.signals);
  const sig = m?.signals ?? [];
  if (!sig.length) return null;
  return (
    <div className="card p-4">
      <div className="card-title mb-2">Traffic lights</div>
      <ul className="space-y-2 text-sm">
        {sig.map((s, i) => (
          <li key={s.id} className="flex flex-wrap items-center gap-2">
            <b className="w-10">{s.id}</b>
            <span className="text-xs text-muted">controls</span>
            <select
              className="input w-44 py-1"
              value={s.stop_line}
              onChange={(e) => saveSignals(sig.map((x, j) => (j === i ? { ...x, stop_line: Number(e.target.value) } : x)))}
            >
              <option value={-1}>no stop line</option>
              {m!.stop_lines.map((_, j) => (
                <option key={j} value={j}>
                  Stop line {j + 1}
                </option>
              ))}
            </select>
            <span className="text-xs text-muted">now: {states?.find((x) => x.id === s.id)?.state ?? "–"}</span>
            <button className="btn-ghost ml-auto px-1.5 py-1 text-bad" onClick={() => saveSignals(sig.filter((_, j) => j !== i))}>
              <Trash2 size={13} />
            </button>
          </li>
        ))}
      </ul>
      {!m!.stop_lines.length && <div className="mt-2 text-xs text-warn">Draw the stop lines in step 2 to link the lights to them.</div>}
    </div>
  );
}
