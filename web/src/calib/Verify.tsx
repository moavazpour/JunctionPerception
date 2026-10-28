// Step 4: does the calibration fit? Map features and a 5 m grid drawn into the camera image, the camera
// image projected onto the map, the expected accuracy by distance, and the traffic check.
import L from "leaflet";
import { useEffect, useMemo, useRef, useState } from "react";
import { useBaseMap } from "../components/leaflet";
import { enuToGeo, GroundCamera, type V2 } from "../geo";
import { calibrationCommand } from "../mqtt";
import { LANE_COLORS, useStore } from "../store";
import type { RefineState, SnapshotMsg, SolveReport, UserMap } from "../types";
import { solveDraft, useDraft, type AlignPair } from "./draft";
import { ImageCanvas, type Handle, type Poly } from "./ImageCanvas";
import { drawLayout, laneRibbon } from "./LayoutEditor";

// Ground polyline (ENU) -> image polylines, split where it leaves the view.
function projectPath(cam: GroundCamera, pts: V2[], step = 1.0): [number, number][][] {
  const out: [number, number][][] = [];
  let cur: [number, number][] = [];
  const W = cam.width, H = cam.height;
  const push = (x: number, y: number) => {
    const p = cam.project(x, y);
    const ok = p && p[0] > -W && p[0] < 2 * W && p[1] > -H && p[1] < 2 * H && Math.hypot(x, y) < 400;
    if (ok) cur.push(p!);
    else if (cur.length) {
      if (cur.length > 1) out.push(cur);
      cur = [];
    }
  };
  for (let i = 0; i + 1 < pts.length; i++) {
    const [ax, ay] = pts[i], [bx, by] = pts[i + 1];
    const n = Math.max(1, Math.ceil(Math.hypot(bx - ax, by - ay) / step));
    for (let k = 0; k <= n; k++) push(ax + ((bx - ax) * k) / n, ay + ((by - ay) * k) / n);
  }
  if (cur.length > 1) out.push(cur);
  return out;
}

export function overlayPolys(cam: GroundCamera, m: UserMap | undefined, grid: boolean, layout: boolean): Poly[] {
  const polys: Poly[] = [];
  if (grid) {
    // 5 m grid in the camera's ground frame, out to 150 m.
    const R = 150;
    for (let x = -R; x <= R; x += 5)
      for (const seg of projectPath(cam, [[x, -R], [x, R]], 2)) polys.push({ pts: seg, color: x % 25 === 0 ? "rgba(255,255,255,0.55)" : "rgba(255,255,255,0.22)", width: 1 });
    for (let y = -R; y <= R; y += 5)
      for (const seg of projectPath(cam, [[-R, y], [R, y]], 2)) polys.push({ pts: seg, color: y % 25 === 0 ? "rgba(255,255,255,0.55)" : "rgba(255,255,255,0.22)", width: 1 });
  }
  if (layout && m && cam.anchor) {
    const enu = (p: [number, number]) => cam.enu(p[0], p[1]);
    m.lanes.forEach((l, i) => {
      const rib = laneRibbon(l.points, l.width);
      const n = rib.length / 2;
      const c = LANE_COLORS[i % LANE_COLORS.length];
      for (const edge of [rib.slice(0, n), rib.slice(n)]) for (const seg of projectPath(cam, edge.map(enu))) polys.push({ pts: seg, color: c, width: 1.5, dashed: true });
    });
    // Stop lines, crosswalks and the road area are drawn through their draggable dots (handlePolys).
  }
  return polys;
}

// The camera image re-projected onto the ground (bird's-eye view) as a map overlay.
async function birdsEye(cam: GroundCamera, snap: SnapshotMsg): Promise<{ url: string; bounds: L.LatLngBoundsExpression } | null> {
  if (!cam.anchor) return null;
  const img = new Image();
  img.src = `data:image/jpeg;base64,${snap.jpeg_base64}`;
  await img.decode();
  const src = document.createElement("canvas");
  src.width = img.width;
  src.height = img.height;
  const sctx = src.getContext("2d")!;
  sctx.drawImage(img, 0, 0);
  const sd = sctx.getImageData(0, 0, img.width, img.height).data;
  // Ground area: what the image sees within 160 m.
  let x0 = 1e9, x1 = -1e9, y0 = 1e9, y1 = -1e9;
  for (let v = cam.height - 1; v >= 0; v -= cam.height / 40)
    for (let u = 0; u <= cam.width; u += cam.width / 40) {
      const g = cam.unproject(u, v);
      if (!g || Math.hypot(g[0], g[1]) > 160) continue;
      x0 = Math.min(x0, g[0]);
      x1 = Math.max(x1, g[0]);
      y0 = Math.min(y0, g[1]);
      y1 = Math.max(y1, g[1]);
    }
  if (x1 <= x0 || y1 <= y0) return null;
  const res = Math.max((x1 - x0) / 700, (y1 - y0) / 700, 0.1);
  const W = Math.ceil((x1 - x0) / res), H = Math.ceil((y1 - y0) / res);
  const out = document.createElement("canvas");
  out.width = W;
  out.height = H;
  const octx = out.getContext("2d")!;
  const od = octx.createImageData(W, H);
  const s = snap.scale;
  for (let j = 0; j < H; j++) {
    const y = y1 - (j + 0.5) * res;
    for (let i = 0; i < W; i++) {
      const x = x0 + (i + 0.5) * res;
      const p = cam.project(x, y);
      if (!p || Math.hypot(x, y) > 160) continue;
      const u = Math.floor(p[0] * s), v = Math.floor(p[1] * s);
      if (u < 0 || v < 0 || u >= img.width || v >= img.height) continue;
      const a = (v * img.width + u) * 4, b = (j * W + i) * 4;
      od.data[b] = sd[a];
      od.data[b + 1] = sd[a + 1];
      od.data[b + 2] = sd[a + 2];
      od.data[b + 3] = 255;
    }
  }
  octx.putImageData(od, 0, 0);
  const sw = enuToGeo(cam.anchor, x0, y0), ne = enuToGeo(cam.anchor, x1, y1);
  return { url: out.toDataURL("image/png"), bounds: [sw, ne] };
}

export function RefineCard({ refine }: { refine?: RefineState }) {
  const [busy, setBusy] = useState(false);
  if (!refine || refine.status === "idle") return null;
  const p = refine.proposal;
  const act = async (action: string) => {
    setBusy(true);
    try {
      await calibrationCommand({ action });
    } finally {
      setBusy(false);
    }
  };
  return (
    <section className="card p-5">
      <div className="mb-1 text-sm font-semibold">Traffic check</div>
      <div className="mb-3 text-xs text-muted">
        Vehicles on a straight road drive parallel. With a slightly wrong camera angle their paths fan out on the map; the
        system measures that and can propose a small correction. Nothing changes without your approval.
      </div>
      {refine.status === "collecting" && (
        <div>
          <div className="mb-1 flex justify-between text-xs">
            <span>Collecting vehicle paths</span>
            <span className="tabular-nums text-muted">{refine.tracks} paths</span>
          </div>
          <div className="h-2 overflow-hidden rounded-full bg-panel2">
            <div className="h-full bg-accent transition-all" style={{ width: `${Math.round(refine.progress * 100)}%` }} />
          </div>
        </div>
      )}
      {refine.status === "solving" && <div className="text-sm text-muted">Analysing the paths…</div>}
      {refine.status === "consistent" && (
        <div className="rounded-lg bg-ok/10 px-3 py-2 text-sm text-ok">
          The traffic agrees with the calibration (paths within {refine.spread_deg.toFixed(1)}° of their direction).
        </div>
      )}
      {refine.status === "rejected" && <div className="text-sm text-muted">Correction rejected; the calibration is unchanged.</div>}
      {refine.status === "proposal" && p && (
        <div className="space-y-3">
          <div className="grid grid-cols-2 gap-3 text-sm md:grid-cols-4">
            <div><div className="text-[11px] text-muted">Path spread</div><b>{p.spread_before_deg.toFixed(2)}° → {p.spread_after_deg.toFixed(2)}°</b></div>
            <div><div className="text-[11px] text-muted">Your points</div><b>{p.points_before_m.toFixed(2)} → {p.points_after_m.toFixed(2)} m</b></div>
            <div><div className="text-[11px] text-muted">Pitch / roll</div><b>{p.d_pitch_deg >= 0 ? "+" : ""}{p.d_pitch_deg.toFixed(2)}° / {p.d_roll_deg >= 0 ? "+" : ""}{p.d_roll_deg.toFixed(2)}°</b></div>
            <div><div className="text-[11px] text-muted">Focal length</div><b>{p.d_focal_rel >= 0 ? "+" : ""}{(p.d_focal_rel * 100).toFixed(1)} %</b></div>
          </div>
          <div className="flex gap-2">
            <button className="btn-primary" disabled={busy} onClick={() => act("refine_accept")}>Apply correction</button>
            <button className="btn" disabled={busy} onClick={() => act("refine_reject")}>Keep my calibration</button>
          </div>
        </div>
      )}
    </section>
  );
}

export function AccuracyTable({ report }: { report?: SolveReport }) {
  if (!report?.accuracy?.length) return null;
  return (
    <section className="card p-5">
      <div className="mb-1 text-sm font-semibold">Expected position accuracy</div>
      <div className="mb-3 text-xs text-muted">From the calibration alone (1 σ), along the viewing direction. Detection adds roughly the same again far away.</div>
      <div className="flex flex-wrap gap-2">
        {report.accuracy.map((a) => (
          <div key={a.range_m} className="rounded-lg bg-panel2 px-3 py-2 text-center">
            <div className="text-[11px] text-muted">{a.range_m} m</div>
            <div className="text-sm font-semibold tabular-nums" style={{ color: a.sigma_m <= 0.5 ? "rgb(var(--ok))" : a.sigma_m <= 1.5 ? "rgb(var(--warn))" : "rgb(var(--bad))" }}>
              ± {a.sigma_m < 10 ? a.sigma_m.toFixed(2) : a.sigma_m.toFixed(0)} m
            </div>
          </div>
        ))}
      </div>
    </section>
  );
}

export function Verify({ camera, report }: { camera?: GroundCamera; report?: SolveReport }) {
  const snapshot = useStore((s) => s.snapshot);
  const userMap = useStore((s) => s.userMap);
  const refine = useStore((s) => s.refine);
  const [grid, setGrid] = useState(true);
  const [layout, setLayout] = useState(true);
  const [bev, setBev] = useState(true);
  const polys = useMemo(() => (camera ? overlayPolys(camera, userMap, grid, layout) : []), [camera, userMap, grid, layout]);
  // Drag-to-align: moving a drawn vertex in the image pairs its map position with that pixel and re-solves.
  const align = useDraft((s) => s.align);
  const showLayout = useDraft((s) => s.showLayout);
  const handles = useMemo(() => (camera && layout ? alignHandles(camera, userMap, align) : []), [camera, userMap, align, layout]);
  const allPolys = useMemo(() => [...polys, ...handlePolys(handles)], [polys, handles]);
  const [solving, setSolving] = useState(false);
  const [msg, setMsg] = useState("");
  const timer = useRef<number>();
  const resolve = () => {
    clearTimeout(timer.current);
    timer.current = window.setTimeout(async () => {
      setSolving(true);
      try {
        const r = await solveDraft(false);
        setMsg(r.plausible ? "" : r.message ?? "no solution");
      } catch (e) {
        setMsg((e as Error).message);
      } finally {
        setSolving(false);
      }
    }, 350);
  };
  const onMove = (id: string, u: number, v: number) => {
    const h = handles.find((x) => x.id === id);
    if (!h) return;
    const d = useDraft.getState();
    d.set({ align: { ...d.align, [id]: { u, v, lat: h.lat, lon: h.lon } } });
    resolve();
  };
  const apply = async () => {
    const ok = report?.ok || confirm("The points fit poorly. Apply this calibration anyway?");
    if (!ok) return;
    setSolving(true);
    try {
      const r = await solveDraft(true, !report?.ok);
      setMsg(r.applied ? "Applied." : r.message ?? "not applied");
    } finally {
      setSolving(false);
    }
  };
  const ref = useRef<HTMLDivElement>(null);
  // The map stays centred on the configured camera position: the solved one moves with every re-solve.
  const config = useStore((s) => s.config);
  const center = useMemo<[number, number] | null>(() => (config ? [config.placement.lat, config.placement.lon] : null), [config?.placement.lat, config?.placement.lon]);
  const map = useBaseMap(ref, center, 18);
  useEffect(() => {
    if (!map) return;
    // A fresh layer per run, removed on cleanup: nothing is ever added to a map that is gone.
    const lg = L.layerGroup().addTo(map);
    let cancelled = false;
    if (bev && camera && snapshot)
      birdsEye(camera, snapshot).then((o) => {
        if (cancelled) return;
        if (o) L.imageOverlay(o.url, o.bounds, { opacity: 0.75 }).addTo(lg);
        if (userMap && showLayout) drawLayout(userMap, lg, { faint: true });
      });
    else if (userMap && showLayout) drawLayout(userMap, lg, { faint: true });
    if (camera?.anchor) L.circleMarker([camera.anchor.lat, camera.anchor.lon], { radius: 6, color: "#3e6ae1", weight: 3, fillOpacity: 1 }).bindTooltip("camera").addTo(lg);
    return () => {
      cancelled = true;
      lg.remove();
    };
  }, [map, camera, snapshot, bev, userMap, showLayout]);
  return (
    <div className="space-y-4">
      <div className="flex flex-wrap items-center gap-2">
        <div className="seg">
          <button data-on={grid} onClick={() => setGrid(!grid)}>5 m grid</button>
          <button data-on={layout} onClick={() => setLayout(!layout)}>Lanes & stop lines</button>
          <button data-on={bev} onClick={() => setBev(!bev)}>Image on map</button>
          <button data-on={showLayout} onClick={() => useDraft.getState().set({ showLayout: !showLayout })}>Layout on map</button>
        </div>
        <span className="flex-1 text-xs text-muted">
          The drawn lanes should sit on the painted lanes in the image. If they do not, <b className="text-fg">drag the white dots</b> in
          the image to where that point really is: the camera is solved again as you drag, and the image on the map follows. A few
          decimetres off is normal (aerial imagery itself is not perfect).
        </span>
        {solving && <span className="text-xs text-accent">solving…</span>}
        {Object.keys(align).length > 0 && (
          <button className="btn" onClick={() => { useDraft.getState().set({ align: {} }); resolve(); }}>
            Reset {Object.keys(align).length} dragged
          </button>
        )}
        <button className="btn-primary" disabled={solving || !report?.plausible} onClick={apply}>
          Apply calibration
        </button>
      </div>
      {!camera && <div className="card p-6 text-sm text-muted">Solve the calibration first (step 3).</div>}
      <div className="grid gap-3 lg:grid-cols-2">
        <ImageCanvas snapshot={snapshot} handles={handles} polys={allPolys} onMove={onMove} className="h-[58vh] min-h-[360px]" />
        <div ref={ref} className="h-[58vh] min-h-[360px] overflow-hidden rounded-lg border border-line" />
      </div>
      {msg && <div className="text-sm text-warn">{msg}</div>}
      <AccuracyTable report={report} />
      <RefineCard refine={refine} />
    </div>
  );
}

export const STOP_COLORS = ["#ffffff", "#ff6b6b", "#fcc419", "#63e6be", "#da77f2"];
const ROI_COLOR = "#5c7cfa", CW_COLOR = "#e9ecef";

export interface AlignHandle extends Handle {
  lat: number;
  lon: number;
  feature: string; // vertices of one feature share it (drawn joined)
  closed?: boolean;
}

// Draggable vertices of the drawn layout, at their projection (or where the installer put them), one colour
// per feature, joined in the image so it is clear which line a dot belongs to.
export function alignHandles(cam: GroundCamera, m: UserMap | undefined, align: Record<string, AlignPair>) {
  const out: AlignHandle[] = [];
  if (!m) return out;
  const add = (id: string, g: [number, number], color: string, feature: string, hint: string, closed = false) => {
    const a = align[id];
    const p = a ? ([a.u, a.v] as V2) : cam.projectGeo(g[0], g[1]);
    if (!p || p[0] < -0.1 * cam.width || p[0] > 1.1 * cam.width || p[1] < -0.1 * cam.height || p[1] > 1.1 * cam.height) return;
    out.push({ id, u: p[0], v: p[1], color, ring: !a, lat: g[0], lon: g[1], feature, hint: `${hint}${a ? " (moved)" : ""}`, closed });
  };
  m.lanes.forEach((l, i) =>
    l.points.forEach((g, j) => add(`lane:${l.id}:${j}`, g, LANE_COLORS[i % LANE_COLORS.length], `lane:${l.id}`, `Lane ${l.id} · point ${j + 1}`)));
  m.stop_lines.forEach((s, i) =>
    s.forEach((g, j) => add(`stop:${i}:${j}`, g as [number, number], STOP_COLORS[i % STOP_COLORS.length], `stop:${i}`, `Stop line ${i + 1} · ${j ? "end B" : "end A"}`)));
  m.crosswalks.forEach((c, i) => c.forEach((g, j) => add(`cw:${i}:${j}`, g, CW_COLOR, `cw:${i}`, `Crosswalk ${i + 1} · corner ${j + 1}`, true)));
  m.roi.forEach((g, j) => add(`roi:${j}`, g, ROI_COLOR, "roi", `Road area · corner ${j + 1}`, true));
  return out;
}

// Each feature drawn through its dots (solid), so moving a dot visibly moves its line.
export function handlePolys(handles: AlignHandle[]): Poly[] {
  const groups = new Map<string, AlignHandle[]>();
  for (const h of handles) groups.set(h.feature, [...(groups.get(h.feature) ?? []), h]);
  const out: Poly[] = [];
  for (const [feature, hs] of groups) {
    if (hs.length < 2) continue;
    const stop = feature.startsWith("stop:");
    out.push({ pts: hs.map((h) => [h.u, h.v]), color: hs[0].color, width: stop ? 4 : 2.5, closed: hs[0].closed, dashed: feature === "roi" });
  }
  return out;
}
