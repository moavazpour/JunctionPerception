import { Cpu, Zap } from "lucide-react";
import { useEffect, useState } from "react";
import { api, json, managerHealth, type ManagerHealth, type SiteInfo } from "../api";
import { useStore } from "../store";
import { NoSite, Page, Row } from "./SettingsPage";

export function ComputeSection() {
  const site = useStore((s) => s.site);
  const c = useStore((s) => s.health?.compute ?? s.config?.compute);
  const pipe = useStore((s) => s.health?.pipeline);
  const [mgr, setMgr] = useState<ManagerHealth | null>(null);
  const [info, setInfo] = useState<SiteInfo | null>(null);
  const [msg, setMsg] = useState("");
  useEffect(() => {
    managerHealth().then(setMgr);
    if (site) api<SiteInfo>(`/sites/${site}`).then(setInfo).catch(() => setInfo(null));
  }, [site]);
  if (!c) return <Page title="Compute"><NoSite /></Page>;
  const gpu = c.backend !== "cpu";
  const setDevice = async (device: string) => {
    if (!site) return;
    setMsg("Restarting the site on the new device…");
    try {
      setInfo(await api<SiteInfo>(`/sites/${site}`, json("PATCH", { device })));
      setMsg(device === "gpu" || (device === "auto" && mgr?.gpu) ? "Restarted. The first GPU start builds the TensorRT engine (about a minute)." : "Restarted on the CPU.");
    } catch (e) {
      setMsg((e as Error).message);
    }
  };
  return (
    <Page title="Compute" desc="Where the neural networks run for this site.">
      <section className="card flex items-center gap-4 p-5">
        <div className={`flex h-12 w-12 items-center justify-center rounded-xl ${gpu ? "bg-ok/10 text-ok" : "bg-warn/10 text-warn"}`}>
          {gpu ? <Zap size={22} /> : <Cpu size={22} />}
        </div>
        <div className="min-w-0 flex-1">
          <div className="text-lg font-semibold">{c.backend === "tensorrt" ? "GPU · NVIDIA TensorRT (FP16)" : gpu ? "GPU · ONNX Runtime CUDA (FP32, model as trained)" : "CPU · ONNX Runtime"}</div>
          <div className="truncate text-sm text-muted">{c.device}</div>
        </div>
        {pipe && (
          <div className="text-right text-sm tabular-nums">
            <div className="text-2xl font-semibold">{pipe.fps.toFixed(1)}</div>
            <div className="text-xs text-muted">frames / s</div>
          </div>
        )}
      </section>
      <section className="card px-5 py-2">
        <Row label="Detector">{c.model} · {c.input.join(" × ")} px · {c.precision.toUpperCase()}</Row>
        <Row label="Detector latency">{pipe ? `${pipe.detector_ms.toFixed(1)} ms per frame` : "–"}</Row>
        <Row label="End-to-end latency" hint="Frame capture to published result.">{pipe?.latency_ms !== undefined ? `${pipe.latency_ms.toFixed(0)} ms` : "–"}</Row>
        <Row label="Segmentation" hint="Static occlusion (buildings, trees); runs every few seconds in the background.">{c.segmentation ? "on" : "off"}</Row>
        <Row label="Backends in this build">{c.available.map((b) => ({ cuda: "ONNX Runtime (GPU)", tensorrt: "TensorRT (GPU)", cpu: "ONNX Runtime (CPU)" } as Record<string, string>)[b] ?? b).join(", ")}</Row>
      </section>
      {mgr && info && (
        <section className="card px-5 py-2">
          <Row label="Run this site on" hint={mgr.gpu ? "Auto uses the GPU when one is available." : "This server has no GPU runtime: sites run on the CPU."}>
            <div className="seg">
              {(["auto", "gpu", "cpu"] as const).map((d) => (
                <button key={d} data-on={info.device === d} disabled={d === "gpu" && !mgr.gpu} onClick={() => setDevice(d)}>
                  {d === "auto" ? "Auto" : d.toUpperCase()}
                </button>
              ))}
            </div>
          </Row>
          {msg && <div className="pb-3 text-xs text-muted">{msg}</div>}
        </section>
      )}
      {!gpu && (
        <div className="rounded-lg border border-warn/40 bg-warn/5 p-3 text-xs text-muted">
          On the CPU the 960 px detector runs at a few frames per second. Tracking and speeds stay correct, but fast
          objects are sampled less often. Lower “Processing rate” under Detection if the CPU is overloaded.
        </div>
      )}
    </Page>
  );
}
