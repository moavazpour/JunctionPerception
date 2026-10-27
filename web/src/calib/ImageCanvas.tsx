// Camera snapshot with zoom (wheel), pan (drag the background), draggable handles and a magnifier.
// All coordinates in and out are frame pixels (the snapshot is a downscaled copy: frame = snapshot / scale).
import { Minus, Plus, Scan } from "lucide-react";
import { useCallback, useEffect, useRef, useState } from "react";
import type { SnapshotMsg } from "../types";

export interface Handle {
  id: string;
  u: number;
  v: number;
  color: string;
  label?: string;
  ring?: boolean; // open (waiting for its map counterpart)
}
export interface Poly {
  pts: [number, number][];
  color: string;
  width?: number;
  dashed?: boolean;
  closed?: boolean;
  fill?: string;
}

interface Props {
  snapshot?: SnapshotMsg;
  handles: Handle[];
  polys: Poly[];
  onPlace?: (u: number, v: number) => void;
  onMove?: (id: string, u: number, v: number) => void;
  className?: string;
}

export function ImageCanvas({ snapshot, handles, polys, onPlace, onMove, className }: Props) {
  const wrap = useRef<HTMLDivElement>(null);
  const canvas = useRef<HTMLCanvasElement>(null);
  const [img, setImg] = useState<HTMLImageElement | null>(null);
  const view = useRef({ zoom: 1, ox: 0, oy: 0 }); // zoom relative to fit, offset in css px
  const drag = useRef<{ kind: "pan" | "handle"; id?: string; x: number; y: number; moved: boolean } | null>(null);
  const mouse = useRef<{ x: number; y: number } | null>(null);
  const [, redraw] = useState(0);

  useEffect(() => {
    if (!snapshot) return;
    const i = new Image();
    i.onload = () => setImg(i);
    i.src = `data:image/jpeg;base64,${snapshot.jpeg_base64}`;
  }, [snapshot]);

  const s = snapshot?.scale ?? 1;
  // css px <-> frame px
  const geom = useCallback(() => {
    const W = wrap.current?.clientWidth ?? 1, H = wrap.current?.clientHeight ?? 1;
    const iw = img?.width ?? 960, ih = img?.height ?? 540;
    const fit = Math.min(W / iw, H / ih);
    const k = fit * view.current.zoom;
    const x0 = (W - iw * k) / 2 + view.current.ox, y0 = (H - ih * k) / 2 + view.current.oy;
    return { W, H, k, x0, y0, iw, ih };
  }, [img]);
  const toCss = (u: number, v: number) => {
    const g = geom();
    return [g.x0 + u * s * g.k, g.y0 + v * s * g.k] as const;
  };
  const toFrame = (x: number, y: number) => {
    const g = geom();
    return [(x - g.x0) / g.k / s, (y - g.y0) / g.k / s] as const;
  };

  const draw = useCallback(() => {
    const c = canvas.current, w = wrap.current;
    if (!c || !w) return;
    const dpr = window.devicePixelRatio || 1;
    const g = geom();
    c.width = g.W * dpr;
    c.height = g.H * dpr;
    c.style.width = `${g.W}px`;
    c.style.height = `${g.H}px`;
    const ctx = c.getContext("2d")!;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = "#0b0d10";
    ctx.fillRect(0, 0, g.W, g.H);
    if (img) ctx.drawImage(img, g.x0, g.y0, g.iw * g.k, g.ih * g.k);
    const P = (u: number, v: number) => [g.x0 + u * s * g.k, g.y0 + v * s * g.k] as const;
    // Overlays stay inside the image.
    ctx.save();
    ctx.beginPath();
    ctx.rect(g.x0, g.y0, g.iw * g.k, g.ih * g.k);
    ctx.clip();
    for (const p of polys) {
      if (p.pts.length < 2) continue;
      ctx.beginPath();
      p.pts.forEach(([u, v], i) => {
        const [x, y] = P(u, v);
        if (i) ctx.lineTo(x, y);
        else ctx.moveTo(x, y);
      });
      if (p.closed) ctx.closePath();
      if (p.fill) {
        ctx.fillStyle = p.fill;
        ctx.fill();
      }
      ctx.strokeStyle = p.color;
      ctx.lineWidth = p.width ?? 2;
      ctx.setLineDash(p.dashed ? [6, 5] : []);
      ctx.stroke();
    }
    ctx.restore();
    ctx.setLineDash([]);
    for (const h of handles) {
      const [x, y] = P(h.u, h.v);
      ctx.beginPath();
      ctx.arc(x, y, h.ring ? 9 : 7, 0, Math.PI * 2);
      ctx.fillStyle = h.ring ? "rgba(0,0,0,0.35)" : h.color;
      ctx.fill();
      ctx.lineWidth = 2;
      ctx.strokeStyle = h.ring ? h.color : "white";
      ctx.stroke();
      if (h.label) {
        ctx.fillStyle = "white";
        ctx.font = "700 10px Inter, sans-serif";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(h.label, x, y + 0.5);
      }
    }
    // Magnifier: 4x around the cursor, in the corner away from it.
    const m = mouse.current;
    if (img && m && !drag.current?.moved) {
      const R = 70, Z = 4;
      const cx = m.x < g.W / 2 ? g.W - R - 10 : R + 10, cy = R + 10;
      ctx.save();
      ctx.beginPath();
      ctx.arc(cx, cy, R, 0, Math.PI * 2);
      ctx.clip();
      ctx.fillStyle = "#000";
      ctx.fillRect(cx - R, cy - R, 2 * R, 2 * R);
      const sx = (m.x - g.x0) / g.k, sy = (m.y - g.y0) / g.k; // snapshot px under the cursor
      const half = R / (g.k * Z);
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(img, sx - half, sy - half, 2 * half, 2 * half, cx - R, cy - R, 2 * R, 2 * R);
      ctx.restore();
      ctx.beginPath();
      ctx.arc(cx, cy, R, 0, Math.PI * 2);
      ctx.strokeStyle = "white";
      ctx.lineWidth = 2;
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(cx - 10, cy);
      ctx.lineTo(cx + 10, cy);
      ctx.moveTo(cx, cy - 10);
      ctx.lineTo(cx, cy + 10);
      ctx.strokeStyle = "#ff4d4d";
      ctx.lineWidth = 1.5;
      ctx.stroke();
    }
  }, [geom, handles, polys, img, s]);

  useEffect(() => {
    draw();
  });
  useEffect(() => {
    const ro = new ResizeObserver(() => draw());
    if (wrap.current) ro.observe(wrap.current);
    return () => ro.disconnect();
  }, [draw]);

  const local = (e: React.PointerEvent | React.WheelEvent) => {
    const r = canvas.current!.getBoundingClientRect();
    return { x: e.clientX - r.left, y: e.clientY - r.top };
  };
  const hit = (x: number, y: number) => {
    for (let i = handles.length - 1; i >= 0; i--) {
      const [hx, hy] = toCss(handles[i].u, handles[i].v);
      if (Math.hypot(hx - x, hy - y) < 10) return handles[i];
    }
    return null;
  };
  const onDown = (e: React.PointerEvent) => {
    const p = local(e);
    (e.target as HTMLElement).setPointerCapture(e.pointerId);
    const h = e.button === 0 ? hit(p.x, p.y) : null;
    drag.current = h && onMove ? { kind: "handle", id: h.id, ...p, moved: false } : { kind: "pan", ...p, moved: false };
  };
  const onMoveEv = (e: React.PointerEvent) => {
    const p = local(e);
    mouse.current = p;
    const d = drag.current;
    if (d) {
      if (Math.hypot(p.x - d.x, p.y - d.y) > 3) d.moved = true;
      if (d.moved && d.kind === "pan") {
        view.current.ox += p.x - d.x;
        view.current.oy += p.y - d.y;
        d.x = p.x;
        d.y = p.y;
      } else if (d.moved && d.kind === "handle" && d.id) {
        const [u, v] = toFrame(p.x, p.y);
        onMove?.(d.id, u, v);
      }
    }
    redraw((x) => x + 1);
  };
  const onUp = (e: React.PointerEvent) => {
    const d = drag.current;
    drag.current = null;
    if (d && !d.moved && e.button === 0 && img) {
      const p = local(e);
      const [u, v] = toFrame(p.x, p.y);
      const fw = img.width / s, fh = img.height / s;
      if (u >= 0 && v >= 0 && u < fw && v < fh) onPlace?.(u, v);
    }
    redraw((x) => x + 1);
  };
  const onWheel = (e: React.WheelEvent) => {
    const p = local(e);
    const g = geom();
    const z0 = view.current.zoom;
    const z1 = Math.min(12, Math.max(1, z0 * Math.exp(-e.deltaY * 0.0015)));
    // Keep the point under the cursor fixed.
    const fx = (p.x - g.x0) / g.k, fy = (p.y - g.y0) / g.k;
    view.current.zoom = z1;
    const g2 = geom();
    view.current.ox += p.x - (g2.x0 + fx * g2.k);
    view.current.oy += p.y - (g2.y0 + fy * g2.k);
    if (z1 === 1) view.current.ox = view.current.oy = 0;
    redraw((x) => x + 1);
  };
  const zoomBy = (f: number) => {
    view.current.zoom = Math.min(12, Math.max(1, view.current.zoom * f));
    if (view.current.zoom === 1) view.current.ox = view.current.oy = 0;
    redraw((x) => x + 1);
  };
  // Wheel listener must be non-passive to stop the page from scrolling.
  useEffect(() => {
    const c = canvas.current;
    if (!c) return;
    const stop = (e: WheelEvent) => e.preventDefault();
    c.addEventListener("wheel", stop, { passive: false });
    return () => c.removeEventListener("wheel", stop);
  }, []);

  return (
    <div ref={wrap} className={`relative overflow-hidden rounded-lg bg-black ${className ?? ""}`}>
      <canvas
        ref={canvas}
        className="block cursor-crosshair touch-none"
        onPointerDown={onDown}
        onPointerMove={onMoveEv}
        onPointerUp={onUp}
        onPointerLeave={() => {
          mouse.current = null;
          redraw((x) => x + 1);
        }}
        onWheel={onWheel}
        onContextMenu={(e) => e.preventDefault()}
      />
      {!snapshot && (
        <div className="absolute inset-0 flex items-center justify-center text-sm text-white/60">Waiting for the camera snapshot…</div>
      )}
      <div className="absolute bottom-2 right-2 flex gap-1">
        <button className="flex h-7 w-7 items-center justify-center rounded-md bg-black/60 text-white" onClick={() => zoomBy(1.5)} title="Zoom in">
          <Plus size={14} />
        </button>
        <button className="flex h-7 w-7 items-center justify-center rounded-md bg-black/60 text-white" onClick={() => zoomBy(1 / 1.5)} title="Zoom out">
          <Minus size={14} />
        </button>
        <button className="flex h-7 w-7 items-center justify-center rounded-md bg-black/60 text-white" onClick={() => zoomBy(0)} title="Fit">
          <Scan size={14} />
        </button>
      </div>
    </div>
  );
}
