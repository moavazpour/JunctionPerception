import { RefreshCw } from "lucide-react";
import { useEffect, useState } from "react";
import { api, managerHealth } from "../api";
import { useStore } from "../store";
import { NoSite, Page, Row } from "./SettingsPage";

const pct = (v?: number) => (v === undefined ? "–" : `${Math.round(v * 100)} %`);

export function SystemSection() {
  const site = useStore((s) => s.site);
  const h = useStore((s) => s.health);
  const connected = useStore((s) => s.connected);
  const [logs, setLogs] = useState<string | null>(null);
  const [hasMgr, setHasMgr] = useState(false);
  const loadLogs = () => site && api<{ logs: string }>(`/sites/${site}/logs?lines=200`).then((r) => setLogs(r.logs)).catch(() => setLogs(null));
  useEffect(() => {
    managerHealth().then((m) => {
      setHasMgr(!!m);
      if (m) loadLogs();
    });
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [site]);
  if (!h) return <Page title="System"><NoSite /></Page>;
  const level = { ok: "text-ok", degraded: "text-warn", fault: "text-bad" }[h.level];
  return (
    <Page title="System" desc="Self-monitoring of the running site.">
      <section className="card px-5 py-2">
        <Row label="Status">
          <span className={`font-semibold uppercase ${level}`}>{h.level}</span>
          <span className="ml-2 text-muted">confidence {pct(h.confidence)}</span>
        </Row>
        {h.reasons.length > 0 && (
          <Row label="Notes">
            <ul className="list-disc space-y-0.5 pl-4 text-xs text-muted">
              {h.reasons.map((r) => <li key={r}>{r}</li>)}
            </ul>
          </Row>
        )}
        <Row label="Broker">{connected ? "connected" : "offline"}</Row>
        <Row label="Frames">{h.pipeline.frames ?? "–"} processed · {h.pipeline.dropped ?? 0} skipped (real-time)</Row>
        {h.stability && <Row label="Camera stability" hint="Shake and drift of the image.">{h.stability.status} · jitter {h.stability.jitter_px.toFixed(1)} px · drift {h.stability.drift_deg.toFixed(2)}°</Row>}
        {h.visibility && (
          <Row label="Visibility" hint="Weather, glare, dirt on the lens.">
            score {pct(h.visibility.score)} · {h.visibility.conditions.join(", ") || "clear"} · range {h.visibility.effective_range_m.toFixed(0)} m
          </Row>
        )}
        {h.calibration && (
          <Row label="Calibration check" hint="Do vehicle sizes, headings and speeds look plausible?">
            {h.calibration.status} · size ratio {h.calibration.size_ratio.toFixed(2)} · heading error {h.calibration.heading_error_deg.toFixed(1)}°
          </Row>
        )}
      </section>
      {hasMgr && (
        <section className="card p-5">
          <div className="mb-2 flex items-center justify-between">
            <div className="text-sm font-semibold">Runtime log</div>
            <button className="btn" onClick={loadLogs}>
              <RefreshCw size={14} /> Refresh
            </button>
          </div>
          <pre className="max-h-[420px] overflow-auto rounded-lg bg-panel2 p-3 font-mono text-[11px] leading-relaxed text-muted">{logs ?? "–"}</pre>
        </section>
      )}
    </Page>
  );
}
