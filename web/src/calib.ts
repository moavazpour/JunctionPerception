// Calibration view: the (pixelated) snapshot with the calibrated ground grid, horizon and lanes
// projected through the camera model, so an installer can verify the auto-calibration at a glance.
import type { CalibrationMsg, CameraJson, MapMsg, SnapshotMsg } from "./types";

type V3 = [number, number, number];
const sub = (a: V3, b: V3): V3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a: V3, b: V3) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a: V3, b: V3): V3 => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const scale = (a: V3, s: number): V3 => [a[0] * s, a[1] * s, a[2] * s];
const norm = (a: V3): V3 => scale(a, 1 / Math.hypot(...a));

// Same construction as jp::CameraModel::build().
function projector(cam: CameraJson) {
  const n = norm(cam.plane.slice(0, 3) as V3);
  const d = cam.plane[3];
  const ez: V3 = [0, 0, 1];
  const f = norm(sub(ez, scale(n, dot(ez, n))));
  const l = cross(n, f);
  const b = (cam.bearing_deg * Math.PI) / 180;
  const W: V3[] = [[Math.sin(b), Math.cos(b), 0], [-Math.cos(b), Math.sin(b), 0], [0, 0, 1]];
  // R_gc = W * C^T, so R_cg = C * W^T: camera vector = f*(W0.g) + l*(W1.g) + n*(W2.g).
  const K = cam.K;
  return (g: V3): [number, number] | null => {
    const q: V3 = [g[0], g[1], g[2] - d];
    const a = dot(W[0], q), bb = dot(W[1], q), c = dot(W[2], q);
    const pc: V3 = [f[0] * a + l[0] * bb + n[0] * c, f[1] * a + l[1] * bb + n[1] * c, f[2] * a + l[2] * bb + n[2] * c];
    if (pc[2] < 0.5) return null;
    return [K[0][0] * (pc[0] / pc[2]) + K[0][2], K[1][1] * (pc[1] / pc[2]) + K[1][2]];
  };
}

export function renderCalibration(host: HTMLElement, cal: CalibrationMsg, snap: SnapshotMsg | null, map: MapMsg | null) {
  const c = cal.calibration;
  host.innerHTML = `
    <div class="grid2">
      <div>Source</div><div>${c.source}</div>
      <div>Focal length</div><div>${c.K[0][0].toFixed(0)} px (HFOV ${((2 * Math.atan(c.width / 2 / c.K[0][0]) * 180) / Math.PI).toFixed(1)}°)</div>
      <div>Height</div><div>${c.height_m.toFixed(2)} m</div>
      <div>Pitch / roll</div><div>${c.pitch_deg.toFixed(2)}° / ${c.roll_deg.toFixed(2)}°</div>
      <div>Bearing</div><div>${c.bearing_deg.toFixed(1)}°</div>
      ${c.confidence !== undefined ? `<div>Confidence</div><div>${Math.round(c.confidence * 100)}% (box ${c.box_residual?.toFixed(3)}, speed drift ${c.speed_drift?.toFixed(3)}, ${c.tracks_used} tracks)</div>` : ""}
    </div>
    <canvas id="calcanvas"></canvas>
    <div class="muted small">Snapshot is published only in calibration mode; every road user is pixelated on the device.</div>`;
  const canvas = host.querySelector("canvas")!;
  const w = snap ? snap.width : 960, h = snap ? snap.height : Math.round((960 * c.height) / c.width);
  canvas.width = w;
  canvas.height = h;
  const ctx = canvas.getContext("2d")!;
  const s = w / c.width;
  const draw = () => {
    const P = projector(c);
    const line = (pts: [number, number, number][], color: string, width = 1.5) => {
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.beginPath();
      let pen = false;
      for (const p of pts) {
        const q = P(p);
        if (!q) {
          pen = false;
          continue;
        }
        if (!pen) ctx.moveTo(q[0] * s, q[1] * s);
        else ctx.lineTo(q[0] * s, q[1] * s);
        pen = true;
      }
      ctx.stroke();
    };
    // Range rings every 25 m.
    const b = (c.bearing_deg * Math.PI) / 180;
    for (let r = 25; r <= 200; r += 25) {
      const pts: [number, number, number][] = [];
      for (let a = -60; a <= 60; a += 2) {
        const t = b + (a * Math.PI) / 180;
        pts.push([Math.sin(t) * r, Math.cos(t) * r, 0]);
      }
      line(pts, "rgba(78,140,255,0.55)", 1);
      const q = P([Math.sin(b) * r, Math.cos(b) * r, 0]);
      if (q) {
        ctx.fillStyle = "rgba(160,190,255,0.9)";
        ctx.font = "12px system-ui";
        ctx.fillText(`${r} m`, q[0] * s + 4, q[1] * s - 4);
      }
    }
    for (const lane of map?.map.lanes ?? []) {
      line(lane.centerline.map(([x, y]) => [x, y, 0]), lane.to_junction ? "rgba(61,220,132,0.9)" : "rgba(255,200,80,0.9)", 2);
    }
  };
  if (snap) {
    const img = new Image();
    img.onload = () => {
      ctx.drawImage(img, 0, 0, w, h);
      draw();
    };
    img.src = `data:image/jpeg;base64,${snap.jpeg_base64}`;
  } else {
    ctx.fillStyle = "#111";
    ctx.fillRect(0, 0, w, h);
    draw();
  }
}
