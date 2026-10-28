// Road layout drawn on the aerial map: lanes (centreline in the direction of travel + width), stop lines,
// crosswalks and the road area. Saved on the device (map/user/set) and used by the twin, the lane
// statistics (queues, distance to the stop line, arrival time) and the calibration check.
import L from "leaflet";
import { Check, Eraser, Eye, EyeOff, Footprints, MoveRight, Save, Square, Trash2, Undo2 } from "lucide-react";
import { useEffect, useMemo, useRef, useState } from "react";
import { useBaseMap } from "../components/leaflet";
import { enuToGeo, geoToEnu } from "../geo";
import { publish } from "../mqtt";
import { LANE_COLORS, useStore } from "../store";
import type { UserMap } from "../types";

type Tool = "lane" | "stop" | "crosswalk" | "roi" | "erase";
const EMPTY: UserMap = { lanes: [], stop_lines: [], crosswalks: [], roi: [] };

// Polygon of a lane ribbon (centreline +/- width/2), in lat/lon.
export function laneRibbon(pts: [number, number][], width: number): [number, number][] {
  if (pts.length < 2) return [];
  const a = { lat: pts[0][0], lon: pts[0][1] };
  const e = pts.map(([la, lo]) => geoToEnu(a, la, lo));
  const off = (d: number) =>
    e.map((p, i) => {
      const p0 = e[Math.max(i - 1, 0)], p1 = e[Math.min(i + 1, e.length - 1)];
      const tx = p1[0] - p0[0], ty = p1[1] - p0[1], l = Math.hypot(tx, ty) || 1;
      return enuToGeo(a, p[0] - (ty / l) * d, p[1] + (tx / l) * d);
    });
  return [...off(width / 2), ...off(-width / 2).reverse()];
}

function arrows(pts: [number, number][], color: string, layer: L.LayerGroup) {
  const a = { lat: pts[0][0], lon: pts[0][1] };
  const e = pts.map(([la, lo]) => geoToEnu(a, la, lo));
  let acc = 0;
  for (let i = 1; i < e.length; i++) {
    const L0 = Math.hypot(e[i][0] - e[i - 1][0], e[i][1] - e[i - 1][1]);
    const tx = (e[i][0] - e[i - 1][0]) / (L0 || 1), ty = (e[i][1] - e[i - 1][1]) / (L0 || 1);
    for (let s = (12 - (acc % 12)) % 12; s < L0; s += 12) {
      const x = e[i - 1][0] + tx * s, y = e[i - 1][1] + ty * s;
      const tri = [
        enuToGeo(a, x + tx * 1.2, y + ty * 1.2),
        enuToGeo(a, x - tx * 0.8 - ty * 0.9, y - ty * 0.8 + tx * 0.9),
        enuToGeo(a, x - tx * 0.8 + ty * 0.9, y - ty * 0.8 - tx * 0.9),
      ];
      L.polygon(tri, { color, weight: 0, fillOpacity: 0.95, interactive: false }).addTo(layer);
    }
    acc += L0;
  }
}

export function drawLayout(m: UserMap, layer: L.LayerGroup, opts: { faint?: boolean } = {}) {
  m.roi.length >= 3 && L.polygon(m.roi, { color: "#3e6ae1", weight: 2, dashArray: "6 6", fillOpacity: 0.04, interactive: false }).addTo(layer);
  m.lanes.forEach((l, i) => {
    const c = LANE_COLORS[i % LANE_COLORS.length];
    const on = l.enabled !== false;
    const rib = laneRibbon(l.points, l.width);
    // A disabled lane: outline only.
    if (rib.length)
      L.polygon(rib, { color: c, weight: 1, dashArray: on ? undefined : "4 4", fillColor: c, fillOpacity: !on ? 0.04 : opts.faint ? 0.18 : 0.3, interactive: false }).addTo(layer);
    if (on && !opts.faint && l.points.length >= 2) arrows(l.points, c, layer);
    if (!opts.faint && l.points.length)
      L.tooltip({ permanent: true, direction: "center", className: "jp-tip" }).setLatLng(l.points[0]).setContent(l.id).addTo(layer);
  });
  m.stop_lines.forEach((s) => L.polyline(s as [number, number][], { color: "#ffffff", weight: 5, opacity: 0.95, interactive: false }).addTo(layer));
  m.crosswalks.forEach((c) => L.polygon(c, { color: "#ffffff", weight: 2, dashArray: "3 4", fillColor: "#ffffff", fillOpacity: 0.25, interactive: false }).addTo(layer));
}

// The drawn feature under a click (within `tol` screen px of its line or inside its area), drawn last first.
function featureAt(map: L.Map, m: UserMap, at: L.Point, tol = 12): { kind: "lane" | "stop" | "crosswalk" | "roi"; i: number } | null {
  const px = (pts: [number, number][]) => pts.map((p) => map.latLngToContainerPoint(p));
  const near = (pts: L.Point[], closed: boolean) => {
    const n = closed ? pts.length : pts.length - 1;
    for (let k = 0; k < n; k++) if (L.LineUtil.pointToSegmentDistance(at, pts[k], pts[(k + 1) % pts.length]) <= tol) return true;
    return false;
  };
  const inside = (pts: L.Point[]) => {
    let c = false;
    for (let a = 0, b = pts.length - 1; a < pts.length; b = a++)
      if (pts[a].y > at.y !== pts[b].y > at.y && at.x < ((pts[b].x - pts[a].x) * (at.y - pts[a].y)) / (pts[b].y - pts[a].y) + pts[a].x) c = !c;
    return c;
  };
  for (let i = m.stop_lines.length - 1; i >= 0; i--) if (near(px(m.stop_lines[i] as [number, number][]), false)) return { kind: "stop", i };
  for (let i = m.lanes.length - 1; i >= 0; i--) {
    const rib = px(laneRibbon(m.lanes[i].points, m.lanes[i].width));
    if (near(px(m.lanes[i].points), false) || (rib.length >= 3 && inside(rib))) return { kind: "lane", i };
  }
  for (let i = m.crosswalks.length - 1; i >= 0; i--) {
    const c = px(m.crosswalks[i]);
    if (near(c, true) || inside(c)) return { kind: "crosswalk", i };
  }
  // The road area only by its outline: it covers everything else.
  if (m.roi.length >= 3 && near(px(m.roi), true)) return { kind: "roi", i: 0 };
  return null;
}

export function LayoutEditor() {
  const remote = useStore((s) => s.userMap);
  const config = useStore((s) => s.config);
  const ref = useRef<HTMLDivElement>(null);
  const center = useMemo<[number, number] | null>(() => (config ? [config.placement.lat, config.placement.lon] : null), [config?.placement.lat, config?.placement.lon]);
  const map = useBaseMap(ref, center, 19);
  const [m, setM] = useState<UserMap>(remote ?? EMPTY);
  const [tool, setTool] = useState<Tool>("lane");
  const [width, setWidth] = useState(3.25);
  const [pending, setPending] = useState<[number, number][]>([]);
  const [dirty, setDirty] = useState(false);
  const [saved, setSaved] = useState("");
  const layer = useRef(L.layerGroup());
  const draftLayer = useRef(L.layerGroup());
  const state = useRef({ tool, pending, width, m });
  state.current = { tool, pending, width, m };

  useEffect(() => {
    if (remote && !dirty) setM(remote);
  }, [remote, dirty]);

  const finish = (raw = state.current.pending) => {
    const t = state.current.tool;
    // A double-click also delivers two clicks: drop repeated points.
    const pts = raw.filter((p, i) => {
      if (!i) return true;
      const [x, y] = geoToEnu({ lat: raw[i - 1][0], lon: raw[i - 1][1] }, p[0], p[1]);
      return Math.hypot(x, y) > 0.5;
    });
    if (t === "lane" && pts.length >= 2) {
      setM((mm) => {
        let k = mm.lanes.length + 1;
        while (mm.lanes.some((l) => l.id === `L${k}`)) k++;
        return { ...mm, lanes: [...mm.lanes, { id: `L${k}`, points: pts, width: state.current.width }] };
      });
    } else if (t === "stop" && pts.length >= 2) setM((mm) => ({ ...mm, stop_lines: [...mm.stop_lines, pts.slice(0, 2)] }));
    else if (t === "crosswalk" && pts.length >= 3) setM((mm) => ({ ...mm, crosswalks: [...mm.crosswalks, pts] }));
    else if (t === "roi" && pts.length >= 3) setM((mm) => ({ ...mm, roi: pts }));
    else return setPending([]);
    setDirty(true);
    setPending([]);
  };

  useEffect(() => {
    if (!map) return;
    map.doubleClickZoom.disable();
    layer.current.addTo(map);
    draftLayer.current.addTo(map);
    map.on("click", (e: L.LeafletMouseEvent) => {
      if (state.current.tool === "erase") {
        const hit = featureAt(map, state.current.m, e.containerPoint);
        if (!hit) return;
        setM((mm) => {
          if (hit.kind === "lane") return { ...mm, lanes: mm.lanes.filter((_, j) => j !== hit.i) };
          if (hit.kind === "stop") return { ...mm, stop_lines: mm.stop_lines.filter((_, j) => j !== hit.i) };
          if (hit.kind === "crosswalk") return { ...mm, crosswalks: mm.crosswalks.filter((_, j) => j !== hit.i) };
          return { ...mm, roi: [] };
        });
        setDirty(true);
        return;
      }
      const p: [number, number] = [e.latlng.lat, e.latlng.lng];
      const next = [...state.current.pending, p];
      if (state.current.tool === "stop" && next.length === 2) return finish(next);
      setPending(next);
    });
    map.on("dblclick", () => finish());
    return () => {
      map.off("click");
      map.off("dblclick");
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [map]);

  // Enter finishes the shape being drawn, Escape drops it.
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if ((e.target as HTMLElement)?.tagName === "INPUT") return;
      if (e.key === "Enter") finish();
      if (e.key === "Escape") setPending([]);
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);
  useEffect(() => {
    layer.current.clearLayers();
    drawLayout(m, layer.current);
  }, [m, map]);
  useEffect(() => {
    const d = draftLayer.current;
    d.clearLayers();
    if (!pending.length) return;
    const color = tool === "lane" ? LANE_COLORS[m.lanes.length % LANE_COLORS.length] : tool === "roi" ? "#3e6ae1" : "#ffffff";
    if (tool === "lane" && pending.length >= 2) L.polygon(laneRibbon(pending, width), { color, weight: 1, fillOpacity: 0.3 }).addTo(d);
    L.polyline(pending, { color, weight: 2, dashArray: "4 4" }).addTo(d);
    pending.forEach((p) => L.circleMarker(p, { radius: 4, color: "white", weight: 2, fillColor: color, fillOpacity: 1 }).addTo(d));
  }, [pending, tool, width, m.lanes.length]);

  const save = () => {
    if (publish("map/user/set", { map: m })) {
      setDirty(false);
      setSaved("Saved on the device.");
      setTimeout(() => setSaved(""), 3000);
    }
  };
  const TOOLS: { t: Tool; label: string; icon: JSX.Element; hint: string }[] = [
    { t: "lane", label: "Lane", icon: <MoveRight size={14} />, hint: "Click along the middle of one lane in the direction of travel, through the stop line. Double-click or Enter to finish, Esc to cancel." },
    { t: "stop", label: "Stop line", icon: <Square size={14} />, hint: "Click both ends of a stop line, across all its lanes." },
    { t: "crosswalk", label: "Crosswalk", icon: <Footprints size={14} />, hint: "Click the corners of the crossing. Double-click to finish." },
    { t: "roi", label: "Road area", icon: <Check size={14} />, hint: "Outline the road surface the system should watch (optional). Objects outside are ignored. Double-click to finish." },
    { t: "erase", label: "Erase", icon: <Eraser size={14} />, hint: "Click a lane, stop line, crossing or the outline of the road area to delete it. Save to keep the change." },
  ];
  const cur = TOOLS.find((x) => x.t === tool)!;
  return (
    <div className="grid gap-4 xl:grid-cols-[1fr_320px]">
      <div className="space-y-2">
        <div className="flex flex-wrap items-center gap-2">
          <div className="seg">
            {TOOLS.map((x) => (
              <button key={x.t} data-on={tool === x.t} onClick={() => { setTool(x.t); setPending([]); }}>
                <span className="flex items-center gap-1">{x.icon} {x.label}</span>
              </button>
            ))}
          </div>
          {tool === "lane" && (
            <label className="flex items-center gap-2 text-xs text-muted">
              Width
              <input type="number" className="input w-20 py-1" min={2} max={6} step={0.05} value={width} onChange={(e) => setWidth(Number(e.target.value) || 3.25)} /> m
            </label>
          )}
          <button className="btn" disabled={!pending.length} onClick={() => setPending(pending.slice(0, -1))}>
            <Undo2 size={14} /> Undo point
          </button>
          <button className="btn" disabled={pending.length < (tool === "lane" || tool === "stop" ? 2 : 3)} onClick={() => finish()}>
            <Check size={14} /> Finish
          </button>
        </div>
        <div className="text-xs text-muted">{cur.hint}</div>
        <div className={tool === "erase" ? "jp-erase" : "cursor-crosshair"}>
          <div ref={ref} className="h-[560px] overflow-hidden rounded-xl border border-line" />
        </div>
      </div>
      <div className="space-y-3">
        <div className="card p-4">
          <div className="card-title mb-2">Lanes ({m.lanes.length})</div>
          <ul className="space-y-1.5">
            {m.lanes.map((l, i) => (
              <li key={l.id} className={`flex items-center gap-2 text-sm ${l.enabled === false ? "opacity-50" : ""}`}>
                <button
                  className="btn-ghost px-1 py-1"
                  title={l.enabled === false ? "Disabled: kept in the layout but not used. Click to enable." : "Enabled. Click to disable this lane."}
                  onClick={() => { setM({ ...m, lanes: m.lanes.map((x, j) => (j === i ? { ...x, enabled: x.enabled === false } : x)) }); setDirty(true); }}
                >
                  {l.enabled === false ? <EyeOff size={13} /> : <Eye size={13} />}
                </button>
                <span className="h-3 w-3 rounded" style={{ background: LANE_COLORS[i % LANE_COLORS.length] }} />
                <input
                  className="input w-20 py-0.5 text-xs"
                  value={l.id}
                  onChange={(e) => { const id = e.target.value.replace(/\s/g, "").slice(0, 12); setM({ ...m, lanes: m.lanes.map((x, j) => (j === i ? { ...x, id } : x)) }); setDirty(true); }}
                />
                <input
                  type="number"
                  className="input w-16 py-0.5 text-xs"
                  step={0.05}
                  value={l.width}
                  onChange={(e) => { setM({ ...m, lanes: m.lanes.map((x, j) => (j === i ? { ...x, width: Number(e.target.value) || 3.25 } : x)) }); setDirty(true); }}
                />
                <span className="text-xs text-muted">m</span>
                <button className="btn-ghost ml-auto px-1.5 py-1 text-bad" onClick={() => { setM({ ...m, lanes: m.lanes.filter((_, j) => j !== i) }); setDirty(true); }}>
                  <Trash2 size={13} />
                </button>
              </li>
            ))}
            {!m.lanes.length && <li className="text-xs text-muted">None yet. Without drawn lanes the system learns them from traffic.</li>}
          </ul>
        </div>
        <div className="card space-y-2 p-4 text-sm">
          <div className="card-title">Other</div>
          {(
            [
              ["Stop lines", m.stop_lines.length, () => setM({ ...m, stop_lines: m.stop_lines.slice(0, -1) })],
              ["Crosswalks", m.crosswalks.length, () => setM({ ...m, crosswalks: m.crosswalks.slice(0, -1) })],
              ["Road area", m.roi.length ? 1 : 0, () => setM({ ...m, roi: [] })],
            ] as const
          ).map(([label, n, del]) => (
            <div key={label} className="flex items-center justify-between">
              <span>{label}</span>
              <span className="flex items-center gap-2">
                <b className="tabular-nums">{n}</b>
                <button className="btn-ghost px-1.5 py-1 text-bad" disabled={!n} title="Remove the last one" onClick={() => { del(); setDirty(true); }}>
                  <Trash2 size={13} />
                </button>
              </span>
            </div>
          ))}
        </div>
        <button className="btn-primary w-full" disabled={!dirty} onClick={save}>
          <Save size={15} /> Save layout
        </button>
        <div className="text-xs text-muted">{saved || (dirty ? "Unsaved changes." : "Lanes that cross a stop line become inbound lanes with queue and arrival-time statistics.")}</div>
      </div>
    </div>
  );
}
