// Detection settings shared by the settings page and the quick panel on the live view: the device's settings
// (config/set, applied within a frame on every camera of the site) and a restart of the site from the start.
import { Loader2, RotateCcw } from "lucide-react";
import { useEffect, useRef, useState } from "react";
import { api, managerHealth } from "../api";
import { useEval } from "../evaluation";
import { publish } from "../mqtt";
import { useStore } from "../store";
import type { Settings } from "../types";

export const INPUT_SIZES = [128, 160, 192, 256, 320, 416, 480, 640, 768, 960, 1120, 1280, 1440, 1600, 1920];
export const BACKENDS = { auto: "Auto", cuda: "GPU (FP32)", tensorrt: "TensorRT (FP16)", cpu: "CPU" } as const;

export function Slider({ value, min, max, step, unit, onChange, fmt }: {
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

// The device's settings with local edits; changes are sent after a short pause (slider drags).
export function useModelSettings() {
  const remote = useStore((s) => s.config?.settings);
  const compute = useStore((s) => s.config?.compute);
  const [s, setS] = useState<Settings | undefined>(remote);
  const timer = useRef<number>();
  const pending = useRef<Partial<Settings>>({});
  useEffect(() => setS(remote), [remote]);
  const change = (p: Partial<Settings>) => {
    setS((cur) => (cur ? { ...cur, ...p } : cur));
    pending.current = { ...pending.current, ...p };
    clearTimeout(timer.current);
    timer.current = window.setTimeout(() => {
      publish("config/set", { settings: pending.current });
      pending.current = {};
    }, 150);
  };
  return { s, compute, change };
}

// Restarts every camera of the site through the site manager: the video plays from its first frame, tracks,
// learned statistics and the accuracy counters start over (the model loads again, a few seconds).
export function RestartButton({ className = "btn" }: { className?: string }) {
  const site = useStore((s) => s.site);
  const [mgr, setMgr] = useState(false);
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState("");
  useEffect(() => {
    managerHealth().then((h) => setMgr(!!h));
  }, []);
  if (!mgr || !site) return null;
  const restart = async () => {
    setBusy(true);
    setErr("");
    try {
      await api(`/sites/${site}/start`, { method: "POST" });
      useEval.getState().reset();
      useStore.getState().set({ objects: [], events: [], selected: null });
    } catch (e) {
      setErr((e as Error).message);
    } finally {
      // The runtime needs a moment to load the model and open the video again.
      setTimeout(() => setBusy(false), 4000);
    }
  };
  return (
    <span className="inline-flex items-center gap-2">
      <button className={className} disabled={busy} onClick={restart} title="Restart all cameras of the site: the video plays from the start, tracks and statistics are cleared">
        {busy ? <Loader2 size={14} className="animate-spin" /> : <RotateCcw size={14} />} Restart from the beginning
      </button>
      {err && <span className="text-xs text-bad">{err}</span>}
    </span>
  );
}
