// Detection settings shared by the settings page and the quick panel on the live view: the device's settings
// (config/set, applied within a frame on every camera of the site) and a restart of the site from the start.
import { Loader2, RotateCcw } from "lucide-react";
import { useEffect, useRef, useState } from "react";
import { api, json, managerHealth } from "../api";
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

const SWITCH_TIMEOUT_S = 300;
const now = () => performance.now() / 1000;
let manager: Promise<boolean> | undefined;
const hasManager = () => (manager ??= managerHealth().then((h) => !!h));

// Model, input size and backend change through the site manager: it restarts every camera of the site with the
// new choice, from the first frame and in lock-step. Mid-stream the old model's tracks would carry on into the new
// model's picture and the cameras would switch at different moments. Road users are hidden until every camera
// runs the new model (store.switching).
async function switchModel(site: string, p: Partial<Pick<Settings, "model" | "imgsz" | "backend">>) {
  const st = useStore.getState();
  st.set({
    switching: { model: p.model ?? st.config?.settings.model ?? "", imgsz: p.imgsz, backend: p.backend, since: Infinity },
    objects: [], events: [], selected: null,
  });
  useEval.getState().reset();
  try {
    await api(`/sites/${site}/model`, json("POST", p));
    // The old containers are gone once the call returns: health from here on is the new model's.
    const sw = useStore.getState().switching;
    if (sw) useStore.getState().set({ switching: { ...sw, since: now() } });
  } catch (e) {
    const sw = useStore.getState().switching;
    if (sw) useStore.getState().set({ switching: { ...sw, error: (e as Error).message } });
  }
}

// The switch is over when every camera of the site reports the new model and is processing frames again.
useStore.subscribe((st) => {
  const sw = st.switching;
  if (!sw || sw.error || !st.site) return;
  const cams = st.camerasSeen[st.site] ?? [st.camera];
  const name = sw.model.split("/").pop();
  const ready = cams.every((c) => {
    const cs = st.cams[c];
    const cfg = cs?.config;
    if (!cfg || (cs.healthAt ?? 0) < sw.since || cfg.compute.model !== name) return false;
    return !sw.backend || cfg.settings.backend === sw.backend;
  });
  const failed = cams.map((c) => st.cams[c]?.config?.compute.error).find((e) => e);
  if (ready) st.set({ switching: null, objects: [] });
  else if (failed && (st.cams[cams[0]]?.healthAt ?? 0) > sw.since) st.set({ switching: { ...sw, error: failed } });
  else if (now() - sw.since > SWITCH_TIMEOUT_S) st.set({ switching: { ...sw, error: "the cameras did not come back with the new model" } });
});

// The device's settings with local edits; changes are sent after a short pause (slider drags).
export function useModelSettings() {
  const remote = useStore((s) => s.config?.settings);
  const compute = useStore((s) => s.config?.compute);
  const site = useStore((s) => s.site);
  const switching = useStore((s) => s.switching);
  const [s, setS] = useState<Settings | undefined>(remote);
  const timer = useRef<number>();
  const pending = useRef<Partial<Settings>>({});
  useEffect(() => setS(remote), [remote]);
  const change = async (p: Partial<Settings>) => {
    const { model, imgsz, backend, ...live } = p;
    const inference = { ...(model !== undefined && { model }), ...(imgsz !== undefined && { imgsz }), ...(backend !== undefined && { backend }) };
    if (Object.keys(inference).length) {
      if (site && (await hasManager())) {
        setS((cur) => (cur ? { ...cur, ...inference } : cur));
        await switchModel(site, inference);
        if (!Object.keys(live).length) return;
      } else {
        Object.assign(live, inference);
      }
    }
    setS((cur) => (cur ? { ...cur, ...live } : cur));
    pending.current = { ...pending.current, ...live };
    clearTimeout(timer.current);
    timer.current = window.setTimeout(() => {
      publish("config/set", { settings: pending.current });
      pending.current = {};
    }, 150);
  };
  return { s, compute, change, switching };
}

// Progress of a model switch (none: nothing shown).
export function SwitchStatus({ className = "" }: { className?: string }) {
  const sw = useStore((s) => s.switching);
  const set = useStore((s) => s.set);
  if (!sw) return null;
  if (sw.error)
    return (
      <div className={`flex items-center gap-2 text-bad ${className}`}>
        <span className="flex-1">Model switch failed: {sw.error}</span>
        <button className="btn-ghost px-1.5 py-0.5" onClick={() => set({ switching: null })}>Dismiss</button>
      </div>
    );
  return (
    <div className={`flex items-center gap-2 ${className}`}>
      <Loader2 size={14} className="animate-spin text-accent" />
      <span>
        Loading <b>{sw.model.split("/").pop()}</b>{sw.imgsz ? ` at ${sw.imgsz} px` : ""} on every camera; the video starts over
        when all are ready
      </span>
    </div>
  );
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
