// Quick detection settings on the live view. A new model, input size or backend restarts the site with it (every
// camera, from the first frame); thresholds apply live. Watch the effect on the twin, the camera view and (on
// dataset sites) the accuracy panel.
import { X } from "lucide-react";
import type { ReactNode } from "react";
import { useEval } from "../evaluation";
import { BACKENDS, INPUT_SIZES, RestartButton, Slider, useModelSettings } from "../settings/modelSettings";
import { Toggle } from "../settings/SettingsPage";
import { useStore } from "../store";

function Field({ label, children }: { label: string; children: ReactNode }) {
  return (
    <div className="space-y-1">
      <div className="text-[10px] uppercase tracking-wide text-muted">{label}</div>
      {children}
    </div>
  );
}

export function ModelPanel() {
  const visible = useStore((s) => s.modelPanel);
  const set = useStore((s) => s.set);
  const health = useStore((s) => s.health?.pipeline);
  const users = useStore((s) => s.objects.length);
  const hasGt = useStore((s) => !!s.gt);
  const ev = useEval();
  const { s, compute, change, switching } = useModelSettings();
  const busy = !!switching && !switching.error;
  if (!visible || !s) return null;
  const recall = ev.tp + ev.fn ? Math.round((100 * ev.tp) / (ev.tp + ev.fn)) : undefined;
  const precision = ev.tp + ev.fp ? Math.round((100 * ev.tp) / (ev.tp + ev.fp)) : undefined;
  return (
    <div className="w-[340px] space-y-3 rounded-xl bg-panel/95 p-3 text-xs shadow-card backdrop-blur">
      <div className="flex items-center">
        <div className="flex-1 text-sm font-semibold">Detection model</div>
        <button className="btn-ghost px-1.5 py-1" title="Close" onClick={() => set({ modelPanel: false })}>
          <X size={13} />
        </button>
      </div>
      <Field label="Model">
        <select className="input py-1" disabled={busy} value={s.model} onChange={(e) => change({ model: e.target.value })}>
          {(compute?.models ?? [s.model]).map((m) => (
            <option key={m} value={m}>{m}</option>
          ))}
        </select>
      </Field>
      <div className="grid grid-cols-2 gap-2">
        <Field label="Input size">
          <select className="input py-1" disabled={busy || !compute?.dynamic_input} value={s.imgsz} onChange={(e) => change({ imgsz: Number(e.target.value) })}>
            {INPUT_SIZES.map((v) => (
              <option key={v} value={v}>{v} × {v}</option>
            ))}
          </select>
        </Field>
        <Field label="Backend">
          <select className="input py-1" disabled={busy} value={s.backend} onChange={(e) => change({ backend: e.target.value })}>
            {(Object.keys(BACKENDS) as (keyof typeof BACKENDS)[]).map((b) => (
              <option key={b} value={b} disabled={b !== "auto" && !compute?.available.includes(b)}>{BACKENDS[b]}</option>
            ))}
          </select>
        </Field>
      </div>
      <Field label="Confidence">
        <Slider value={s.confidence} min={0.1} max={0.9} step={0.01} onChange={(v) => change({ confidence: v })} fmt={(v) => v.toFixed(2)} />
      </Field>
      <Field label="Keypoint confidence">
        <Slider value={s.keypoint_confidence} min={0.05} max={0.9} step={0.01} onChange={(v) => change({ keypoint_confidence: v })} fmt={(v) => v.toFixed(2)} />
      </Field>
      <div className="flex items-center justify-between">
        <span>Detect in the road area only</span>
        <Toggle on={s.crop_to_roi} onChange={(v) => change({ crop_to_roi: v })} />
      </div>
      {compute?.error && !switching && <div className="text-bad">Last model change failed: {compute.error}</div>}
      <div className="grid grid-cols-3 gap-2 border-t border-line pt-2 tabular-nums">
        <div title="Network input now">
          <div className="text-[10px] text-muted">input</div>
          {compute?.input?.join("×") ?? "–"}
        </div>
        <div title="Frames per second and detector time">
          <div className="text-[10px] text-muted">speed</div>
          {health ? `${health.fps.toFixed(0)} fps · ${health.detector_ms?.toFixed(0)} ms` : "–"}
        </div>
        <div title={hasGt ? "Found / correct against the ground truth (this loop)" : "Road users now"}>
          <div className="text-[10px] text-muted">{hasGt ? "found · correct" : "road users"}</div>
          {hasGt ? `${recall ?? "–"} % · ${precision ?? "–"} %` : users}
        </div>
      </div>
      <RestartButton className="btn w-full justify-center" />
    </div>
  );
}
