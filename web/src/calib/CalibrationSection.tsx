// Manual calibration: the installer places the camera in the world by matching the camera image with the
// aerial map. Four steps: camera hints, road layout, reference points/lines, check.
import { AlertTriangle, CheckCircle2, ChevronRight, CircleHelp, Loader2, RotateCcw, Wand2 } from "lucide-react";
import { useEffect, useMemo, useState } from "react";
import { navigate } from "../App";
import { GroundCamera } from "../geo";
import { calibrationCommand } from "../mqtt";
import { NoSite } from "../settings/SettingsPage";
import { LENSES } from "../settings/SitesSection";
import { useStore } from "../store";
import type { SolveReport } from "../types";
import { completeLines, completePoints, completeVerticals, loadDraft, useDraft } from "./draft";
import { LayoutEditor } from "./LayoutEditor";
import { PairsEditor } from "./PairsEditor";
import { RefineCard, Verify } from "./Verify";

const STEPS = ["Camera", "Road layout", "Match image and map", "Check"];

function Stepper({ step, setStep }: { step: number; setStep: (n: number) => void }) {
  return (
    <ol className="flex flex-wrap items-center gap-1">
      {STEPS.map((s, i) => (
        <li key={s} className="flex items-center gap-1">
          <button
            onClick={() => setStep(i)}
            className={`flex items-center gap-2 rounded-full px-3 py-1.5 text-sm font-medium transition ${
              i === step ? "bg-accent text-white" : i < step ? "bg-accent/10 text-accent" : "bg-panel2 text-muted hover:text-fg"
            }`}
          >
            <span className={`flex h-5 w-5 items-center justify-center rounded-full text-[11px] ${i === step ? "bg-white/25" : "bg-black/5"}`}>{i + 1}</span>
            {s}
          </button>
          {i < STEPS.length - 1 && <ChevronRight size={14} className="text-muted" />}
        </li>
      ))}
    </ol>
  );
}

function StatusBanner() {
  const cal = useStore((s) => s.calibration);
  const [busy, setBusy] = useState(false);
  if (!cal)
    return (
      <div className="flex items-center gap-3 rounded-xl border border-warn/40 bg-warn/5 p-4 text-sm">
        <AlertTriangle size={18} className="text-warn" />
        <div>
          <b>Not calibrated.</b> <span className="text-muted">The model already detects road users, but cannot place them in the twin yet.</span>
        </div>
      </div>
    );
  const m = (cal as any).manual as SolveReport | undefined;
  return (
    <div className="flex flex-wrap items-center gap-3 rounded-xl border border-ok/30 bg-ok/5 p-4 text-sm">
      <CheckCircle2 size={18} className="text-ok" />
      <div className="min-w-0 flex-1">
        <b>Calibrated</b>
        <span className="text-muted">
          {" "}
          ({cal.source}) · height {cal.height_m.toFixed(1)} m · pitch {cal.pitch_deg.toFixed(1)}° · looking {cal.bearing_deg.toFixed(0)}°
          {m?.ground_rms_m !== undefined && ` · points fit ${m.ground_rms_m.toFixed(2)} m`}
        </span>
      </div>
      <button
        className="btn-ghost text-bad"
        disabled={busy}
        onClick={async () => {
          if (!confirm("Discard the calibration? The twin stays empty until you calibrate again.")) return;
          setBusy(true);
          await calibrationCommand({ action: "auto" }).catch(() => {});
          setBusy(false);
        }}
      >
        <RotateCcw size={14} /> Clear
      </button>
    </div>
  );
}

function Guide() {
  return (
    <section className="card p-5 text-sm">
      <div className="mb-2 flex items-center gap-2 font-semibold">
        <CircleHelp size={16} className="text-accent" /> What you will do
      </div>
      <ol className="list-decimal space-y-1.5 pl-5 text-muted">
        <li><b className="text-fg">Camera:</b> say roughly how high the camera is and which lens filmed the video. Rough is fine; it only rules out impossible solutions.</li>
        <li><b className="text-fg">Road layout:</b> on the aerial map, draw each lane (in its driving direction) and the stop lines. This becomes the lanes of the digital twin.</li>
        <li>
          <b className="text-fg">Match image and map:</b> click the same spot in the camera image and on the map, 6–8 times. Good spots: corners of dashed lane
          markings, ends of stop lines, kerb corners. Spread them near and far, left and right. Add 2–3 <i>lines</i> (a long lane marking traced in the image, two points on the map) for best accuracy.
        </li>
        <li><b className="text-fg">Check:</b> the drawn lanes and a 5 m grid appear in the camera image. When they sit on the road, apply.</li>
      </ol>
      <div className="mt-3 rounded-lg bg-panel2 p-3 text-xs text-muted">
        The camera can be anywhere and zoomed in or out: focal length, tilt, height and the exact camera position are all solved from your clicks. Points must
        lie on the road surface (not on cars, signs or roofs).
      </div>
    </section>
  );
}

function CameraStep({ next }: { next: () => void }) {
  const d = useDraft();
  const config = useStore((s) => s.config);
  const snapshot = useStore((s) => s.snapshot);
  return (
    <div className="grid gap-4 lg:grid-cols-[1fr_1fr]">
      <div className="space-y-4">
        <Guide />
        <section className="card space-y-4 p-5">
          <div>
            <label className="label">Lens that recorded the video</label>
            <select className="input" value={d.lens} onChange={(e) => d.set({ lens: e.target.value })}>
              {LENSES.map((l) => (
                <option key={l.v} value={l.v}>
                  {l.label} ({l.hint})
                </option>
              ))}
            </select>
          </div>
          <div className="grid grid-cols-2 gap-4">
            <div>
              <label className="label">Camera height above the road</label>
              <div className="flex items-center gap-2">
                <input type="number" className="input" min={1} max={150} step={0.5} value={d.height} onChange={(e) => d.set({ height: Number(e.target.value) || 7 })} />
                <span className="text-sm text-muted">m</span>
              </div>
            </div>
            <div>
              <label className="label">How sure (±)</label>
              <div className="flex items-center gap-2">
                <input type="number" className="input" min={0.2} max={20} step={0.5} value={d.heightSigma} onChange={(e) => d.set({ heightSigma: Number(e.target.value) || 1.5 })} />
                <span className="text-sm text-muted">m</span>
              </div>
            </div>
          </div>
          <div className="text-xs text-muted">
            Camera position: {config ? `${config.placement.lat.toFixed(6)}, ${config.placement.lon.toFixed(6)}, looking ${config.placement.bearing_deg.toFixed(0)}°` : "–"} ·{" "}
            <button className="text-accent hover:underline" onClick={() => navigate("settings/source")}>change</button>
          </div>
        </section>
        <button className="btn-primary" onClick={next}>
          Next: road layout <ChevronRight size={15} />
        </button>
      </div>
      <section className="card overflow-hidden">
        {snapshot ? (
          <img className="w-full" src={`data:image/jpeg;base64,${snapshot.jpeg_base64}`} alt="Camera snapshot" />
        ) : (
          <div className="p-8 text-center text-sm text-muted">Waiting for the camera snapshot…</div>
        )}
        <div className="px-4 py-2 text-xs text-muted">Calibration snapshot (road users are blurred).</div>
      </section>
    </div>
  );
}

function SolvePanel({ report, onReport, toCheck }: { report?: SolveReport; onReport: (r: SolveReport) => void; toCheck: () => void }) {
  const d = useDraft();
  const [busy, setBusy] = useState<"" | "solve" | "apply">("");
  const [err, setErr] = useState("");
  const pts = completePoints(d), lines = completeLines(d), verts = completeVerticals(d);
  const ready = pts.length >= 4;
  const run = async (apply: boolean, force = false) => {
    setBusy(apply ? "apply" : "solve");
    setErr("");
    try {
      const r = (await calibrationCommand({
        action: "solve",
        apply,
        force,
        points: pts.map((p) => ({ u: p.u, v: p.v, lat: p.lat, lon: p.lon })),
        lines,
        verticals: verts,
        height_m: d.height,
        height_sigma: d.heightSigma,
        lens: d.lens,
      })) as SolveReport;
      onReport(r);
      if (apply && r.applied) toCheck();
    } catch (e) {
      setErr((e as Error).message);
    } finally {
      setBusy("");
    }
  };
  const q = report?.quality;
  const qc = q === "good" ? "text-ok bg-ok/10" : q === "fair" ? "text-warn bg-warn/10" : "text-bad bg-bad/10";
  return (
    <section className="card p-5">
      <div className="flex flex-wrap items-center gap-3">
        <div className="text-sm">
          <b className="tabular-nums">{pts.length}</b> points · <b className="tabular-nums">{lines.length}</b> lines · <b className="tabular-nums">{verts.length}</b> verticals
          {pts.length < 4 && <span className="ml-2 text-warn">(at least 4 points needed)</span>}
          {pts.length >= 4 && pts.length < 6 && !lines.length && <span className="ml-2 text-muted">(6+ points or a few lines make it robust)</span>}
        </div>
        <div className="flex-1" />
        <button className="btn" disabled={!ready || !!busy} onClick={() => run(false)}>
          {busy === "solve" ? <Loader2 size={14} className="animate-spin" /> : <Wand2 size={14} />} Solve
        </button>
        <button
          className="btn-primary"
          disabled={!ready || !!busy || !report?.plausible}
          onClick={() => {
            if (report?.ok) run(true);
            else if (confirm("The points fit poorly. Apply this calibration anyway?")) run(true, true);
          }}
        >
          {busy === "apply" && <Loader2 size={14} className="animate-spin" />} Apply calibration
        </button>
      </div>
      {err && <div className="mt-3 text-sm text-bad">{err}</div>}
      {report && report.action === "solve" && (
        <div className="mt-4 space-y-3">
          <div className="flex flex-wrap items-center gap-2">
            {q && <span className={`rounded-full px-2.5 py-0.5 text-xs font-semibold uppercase ${qc}`}>{q}</span>}
            {report.ground_rms_m !== undefined && (
              <span className="text-sm">
                Points fit <b>{report.ground_rms_m.toFixed(2)} m</b> on the ground (worst {report.ground_max_m?.toFixed(2)} m), {report.rms_px?.toFixed(1)} px in the image
              </span>
            )}
            {report.applied && <span className="chip border-ok/30 bg-ok/10 text-ok">applied</span>}
          </div>
          {report.plausible && (
            <div className="grid grid-cols-2 gap-2 text-xs sm:grid-cols-5">
              {(
                [
                  ["Field of view", `${report.fov_deg?.toFixed(0)}°`],
                  ["Focal length", `${report.focal_px?.toFixed(0)} px`],
                  ["Height", `${report.height_m?.toFixed(2)} m`],
                  ["Tilt (pitch)", `${report.pitch_deg?.toFixed(2)}°`],
                  ["Looking", `${report.bearing_deg?.toFixed(1)}°`],
                ] as const
              ).map(([k, v]) => (
                <div key={k} className="rounded-lg bg-panel2 px-3 py-2">
                  <div className="text-muted">{k}</div>
                  <div className="text-sm font-semibold tabular-nums">{v}</div>
                </div>
              ))}
            </div>
          )}
          {!report.plausible && <div className="text-sm text-bad">{report.message}</div>}
          {!!report.warnings?.length && (
            <ul className="space-y-1 text-xs">
              {report.warnings.map((w) => (
                <li key={w} className="flex gap-2 text-warn">
                  <AlertTriangle size={13} className="mt-0.5 shrink-0" /> {w}
                </li>
              ))}
            </ul>
          )}
          {report.plausible && (
            <button className="btn" onClick={toCheck}>
              Check the fit <ChevronRight size={14} />
            </button>
          )}
        </div>
      )}
    </section>
  );
}

export function CalibrationSection() {
  const site = useStore((s) => s.site);
  const config = useStore((s) => s.config);
  const inputs = useStore((s) => s.inputs);
  const calibration = useStore((s) => s.calibration);
  const lastReport = useStore((s) => s.lastReport);
  const refine = useStore((s) => s.refine);
  const [step, setStep] = useState(0);
  const [report, setReport] = useState<SolveReport | undefined>(undefined);
  const draftSite = useDraft((s) => s.site);
  useEffect(() => {
    if (site && config && draftSite !== site)
      loadDraft(site, inputs, { lens: config.placement.lens === "unknown" ? "phone_wide" : config.placement.lens, height: config.placement.height_m, heightSigma: config.placement.height_sigma });
  }, [site, config, inputs, draftSite]);
  useEffect(() => {
    if (lastReport?.action === "solve") setReport(lastReport);
  }, [lastReport]);
  // Camera for the overlays: the last solve (even before it is applied), else the active calibration.
  const camera = useMemo(() => {
    if (report?.camera) return new GroundCamera(report.camera);
    if (calibration?.anchor) return new GroundCamera(calibration);
    return undefined;
  }, [report, calibration]);
  const shownReport = report ?? ((calibration as any)?.manual as SolveReport | undefined);
  if (!site || !config) return <div className="p-6"><NoSite /></div>;
  return (
    <div className="mx-auto w-full max-w-[1500px] space-y-4 p-4 md:p-6">
      <div className="flex flex-wrap items-end justify-between gap-3">
        <div>
          <h1 className="text-xl font-semibold tracking-tight">Calibration</h1>
          <p className="mt-1 text-sm text-muted">Place the camera in the world so every detection lands at its real position, with real speeds.</p>
        </div>
        <Stepper step={step} setStep={setStep} />
      </div>
      <StatusBanner />
      {step === 0 && <CameraStep next={() => setStep(1)} />}
      {step === 1 && (
        <>
          <LayoutEditor />
          <button className="btn-primary" onClick={() => setStep(2)}>
            Next: match image and map <ChevronRight size={15} />
          </button>
        </>
      )}
      {step === 2 && (
        <>
          <PairsEditor camera={camera} report={shownReport} />
          <SolvePanel report={report} onReport={setReport} toCheck={() => setStep(3)} />
        </>
      )}
      {step === 3 && <Verify camera={camera} report={shownReport} />}
      {step !== 3 && calibration && <RefineCard refine={refine} />}
    </div>
  );
}
