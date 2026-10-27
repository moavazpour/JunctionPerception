import { RotateCcw } from "lucide-react";
import { useEffect, useRef, useState } from "react";
import { publish } from "../mqtt";
import { CLASS_COLOR, CLASS_LABEL, useStore } from "../store";
import type { ObjectClass, Settings } from "../types";
import { NoSite, Page, Row, Toggle } from "./SettingsPage";

const ALL: ObjectClass[] = ["car", "bus", "truck", "trailer", "motorcycle", "bicycle", "escooter", "pedestrian"];
const DEFAULTS: Partial<Settings> = { confidence: 0.45, keypoint_confidence: 0.35, max_range_m: 250, max_fps: 15, preview_fps: 12 };

function Slider({ value, min, max, step, unit, onChange, fmt }: {
  value: number; min: number; max: number; step: number; unit?: string; onChange: (v: number) => void; fmt?: (v: number) => string;
}) {
  return (
    <div className="flex items-center gap-3">
      <input type="range" className="flex-1" min={min} max={max} step={step} value={value} onChange={(e) => onChange(Number(e.target.value))} />
      <span className="w-20 text-right text-sm tabular-nums">
        {fmt ? fmt(value) : value}
        {unit && <span className="text-muted"> {unit}</span>}
      </span>
    </div>
  );
}

// Model settings applied by the running site within a frame (topic config/set), persisted on the device.
export function InferenceSection() {
  const remote = useStore((s) => s.config?.settings);
  const [s, setS] = useState<Settings | undefined>(remote);
  const timer = useRef<number>();
  const pending = useRef<Partial<Settings>>({});
  useEffect(() => setS(remote), [remote]);
  if (!s) return <Page title="Detection"><NoSite /></Page>;
  const change = (p: Partial<Settings>) => {
    setS({ ...s, ...p });
    pending.current = { ...pending.current, ...p };
    clearTimeout(timer.current);
    timer.current = window.setTimeout(() => {
      publish("config/set", { settings: pending.current });
      pending.current = {};
    }, 150);
  };
  return (
    <Page title="Detection" desc="Changes apply to the running site immediately and are kept across restarts.">
      <section className="card px-5 py-2">
        <Row label="Detection confidence" hint="Score a detection needs to start a track. Lower finds more far objects but more false ones.">
          <Slider value={s.confidence} min={0.1} max={0.9} step={0.01} onChange={(v) => change({ confidence: v })} fmt={(v) => v.toFixed(2)} />
        </Row>
        <Row label="Keypoint confidence" hint="Cuboid corners below this are not used for the 3D box.">
          <Slider value={s.keypoint_confidence} min={0.05} max={0.9} step={0.01} onChange={(v) => change({ keypoint_confidence: v })} fmt={(v) => v.toFixed(2)} />
        </Row>
        <Row label="Maximum range" hint="Objects farther from the camera are ignored.">
          <Slider value={s.max_range_m} min={30} max={500} step={10} unit="m" onChange={(v) => change({ max_range_m: v })} />
        </Row>
        <Row label="Processing rate" hint="Frames per second analysed (0 = every frame).">
          <Slider value={s.max_fps} min={0} max={30} step={1} unit="fps" onChange={(v) => change({ max_fps: v })} fmt={(v) => (v ? String(v) : "all")} />
        </Row>
        <Row label="Road area only" hint="Ignore objects outside the road area drawn during calibration (parked cars, yards).">
          <Toggle on={s.roi_filter} onChange={(v) => change({ roi_filter: v })} />
        </Row>
      </section>
      <section className="card px-5 py-4">
        <div className="mb-1 text-sm font-medium">Reported classes</div>
        <div className="mb-3 text-xs text-muted">Unchecked classes are dropped before tracking.</div>
        <div className="flex flex-wrap gap-2">
          {ALL.map((c) => {
            const on = s.classes.includes(c);
            return (
              <button
                key={c}
                className={`chip px-3 py-1 text-xs ${on ? "border-accent/40 bg-accent/10 text-fg" : "opacity-60"}`}
                onClick={() => change({ classes: on ? s.classes.filter((x) => x !== c) : [...s.classes, c] })}
              >
                <span className="h-2 w-2 rounded-full" style={{ background: CLASS_COLOR[c] }} /> {CLASS_LABEL[c]}
              </button>
            );
          })}
        </div>
      </section>
      <section className="card px-5 py-2">
        <Row label="Boxes in the camera view" hint="Draw the tracked 3D boxes on the live camera image.">
          <Toggle on={s.preview_overlay} onChange={(v) => change({ preview_overlay: v })} />
        </Row>
        <Row label="Camera view frame rate">
          <Slider value={s.preview_fps} min={2} max={25} step={1} unit="fps" onChange={(v) => change({ preview_fps: v })} />
        </Row>
      </section>
      <button className="btn" onClick={() => change({ ...DEFAULTS, classes: ALL, roi_filter: true, preview_overlay: true })}>
        <RotateCcw size={14} /> Restore defaults
      </button>
    </Page>
  );
}
