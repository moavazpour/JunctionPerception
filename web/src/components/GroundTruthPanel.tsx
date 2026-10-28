// Accuracy against recorded ground truth (replayed dataset sites): detection rate, position / heading / speed
// errors by distance, and the calibration compared with the dataset's reference calibration.
import { Download, RotateCcw, X } from "lucide-react";
import { useEffect } from "react";
import { api, type GroundTruth } from "../api";
import { calibrationDiff, GATE_M, MAX_RANGE_M, quantile, RANGES, startEvaluation, useEval } from "../evaluation";
import { useStore } from "../store";

// Loads the site's ground truth (if it has one) and keeps the evaluation running.
export function useGroundTruth() {
  const site = useStore((s) => s.site);
  const set = useStore((s) => s.set);
  useEffect(() => {
    startEvaluation();
    if (!site) return;
    let live = true;
    api<GroundTruth>(`/sites/${site}/groundtruth`)
      // A ground-truth twin replaces what the perception sent before the recording arrived.
      .then((gt) => live && set({ gt, ...(gt.twin ? { objects: [], events: [], selected: null } : {}) }))
      .catch(() => live && set({ gt: null }));
    return () => {
      live = false;
    };
  }, [site, set]);
}

const f1 = (v: number | undefined, unit = "") => (v === undefined ? "–" : `${v.toFixed(v < 10 ? 2 : 1)}${unit}`);

function Stat({ label, value, hint }: { label: string; value: string; hint?: string }) {
  return (
    <div title={hint}>
      <div className="text-[10px] uppercase tracking-wide text-muted">{label}</div>
      <div className="text-base font-semibold tabular-nums">{value}</div>
    </div>
  );
}

export function GroundTruthPanel() {
  const gt = useStore((s) => s.gt);
  const visible = useStore((s) => s.gtVisible);
  const camera = useStore((s) => s.camera);
  const calib = useStore((s) => s.calibration);
  const set = useStore((s) => s.set);
  const ev = useEval();
  if (!gt || !visible) return null;
  const recall = ev.tp + ev.fn ? ev.tp / (ev.tp + ev.fn) : undefined;
  const precision = ev.tp + ev.fp ? ev.tp / (ev.tp + ev.fp) : undefined;
  const bins = RANGES.map((hi, i) => {
    const lo = i ? RANGES[i - 1] : 0;
    const m = ev.matches.filter((x) => x.range >= lo && x.range < hi).map((x) => x.pos);
    return { lo, hi, n: m.length, med: quantile(m, 0.5), p90: quantile(m, 0.9) };
  });
  const heading = ev.matches.flatMap((m) => (m.heading === undefined ? [] : [m.heading]));
  const speed = ev.matches.flatMap((m) => (m.speed === undefined ? [] : [Math.abs(m.speed)]));
  const length = ev.matches.flatMap((m) => (m.length === undefined ? [] : [Math.abs(m.length)]));
  const ref = gt.calibration[camera];
  const diff = calib && ref ? calibrationDiff(calib, ref) : null;
  const report = () => {
    const body = {
      dataset: gt.dataset, frames: ev.frames, loops: ev.loops, gate_m: GATE_M, max_range_m: MAX_RANGE_M,
      tp: ev.tp, fp: ev.fp, fn: ev.fn, recall, precision,
      position_by_range: bins, heading_deg: { median: quantile(heading, 0.5), p90: quantile(heading, 0.9) },
      speed_ms: { median: quantile(speed, 0.5), p90: quantile(speed, 0.9) }, length_m: { median: quantile(length, 0.5) },
      calibration: { camera, ...diff },
    };
    const a = document.createElement("a");
    a.href = URL.createObjectURL(new Blob([JSON.stringify(body, null, 1)], { type: "application/json" }));
    a.download = `evaluation_${useStore.getState().site}.json`;
    a.click();
  };
  return (
    <div className="w-[340px] space-y-3 rounded-xl bg-panel/95 p-3 text-xs shadow-card backdrop-blur">
      <div className="flex items-center gap-2">
        <div className="flex-1">
          <div className="text-sm font-semibold">Accuracy vs ground truth</div>
          <div className="text-[10px] text-muted">{gt.dataset}</div>
        </div>
        <button className="btn-ghost px-1.5 py-1" title="Download as JSON" onClick={report}>
          <Download size={13} />
        </button>
        <button className="btn-ghost px-1.5 py-1" title="Start over" onClick={ev.reset}>
          <RotateCcw size={13} />
        </button>
        <button className="btn-ghost px-1.5 py-1" title="Close" onClick={() => set({ gtVisible: false })}>
          <X size={13} />
        </button>
      </div>
      <div className="grid grid-cols-3 gap-2">
        <Stat label="Frames" value={String(ev.frames)} hint="Source video frames evaluated in this loop" />
        <Stat label="Found" value={recall === undefined ? "–" : `${Math.round(recall * 100)} %`} hint={`Ground-truth road users within ${MAX_RANGE_M} m matched by a detection within ${GATE_M} m (recall)`} />
        <Stat label="Correct" value={precision === undefined ? "–" : `${Math.round(precision * 100)} %`} hint="Detections that match a ground-truth road user (precision)" />
      </div>
      <table className="w-full tabular-nums">
        <thead className="text-[10px] text-muted">
          <tr className="text-left">
            <th className="font-medium">Position error</th>
            <th className="font-medium">median</th>
            <th className="font-medium">p90</th>
            <th className="font-medium">n</th>
          </tr>
        </thead>
        <tbody>
          {bins.map((b) => (
            <tr key={b.hi}>
              <td>{b.lo}–{b.hi} m</td>
              <td>{f1(b.med, " m")}</td>
              <td>{f1(b.p90, " m")}</td>
              <td className="text-muted">{b.n}</td>
            </tr>
          ))}
        </tbody>
      </table>
      <div className="grid grid-cols-3 gap-2">
        <Stat label="Heading" value={f1(quantile(heading, 0.5), "°")} hint="Median heading error of moving road users" />
        <Stat label="Speed" value={f1(quantile(speed, 0.5), " m/s")} hint="Median absolute speed error" />
        <Stat label="Length" value={f1(quantile(length, 0.5), " m")} hint="Median absolute length error" />
      </div>
      {ref && (
        <div className="border-t border-line pt-2">
          <div className="mb-1 font-medium">Calibration of {camera} vs the dataset's</div>
          {diff ? (
            <div className="grid grid-cols-3 gap-x-2 gap-y-1 tabular-nums">
              <span>position {f1(diff.position, " m")}</span>
              <span>height {f1(diff.height, " m")}</span>
              <span>focal {diff.focal.toFixed(1)} %</span>
              <span>bearing {diff.bearing.toFixed(1)}°</span>
              <span>tilt {diff.pitch.toFixed(1)}°</span>
              <span>roll {diff.roll.toFixed(1)}°</span>
              {diff.ground.map((g) => (
                <span key={g.range}>at {g.range} m: {f1(g.error, " m")}</span>
              ))}
            </div>
          ) : (
            <div className="text-muted">Not calibrated yet.</div>
          )}
        </div>
      )}
      <div className="text-[10px] text-muted">
        Grey boxes in the twin are the ground truth. Errors restart when the video loops.
      </div>
    </div>
  );
}
