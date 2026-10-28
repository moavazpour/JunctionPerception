import { ArrowLeft, Cpu, Layers, Monitor, Moon, Radio, Repeat, Settings, Sun, TriangleAlert, Zap } from "lucide-react";
import { useEffect, useState } from "react";
import { navigate, type Route } from "../App";
import { api, type SiteInfo } from "../api";
import { selectCamera, selectSite } from "../mqtt";
import { useStore, type ThemePref } from "../store";

function Logo() {
  return (
    <svg width="26" height="26" viewBox="0 0 32 32" aria-hidden>
      <rect width="32" height="32" rx="8" className="fill-accent" />
      <path d="M8 22 L16 8 L24 22" stroke="white" strokeWidth="2.6" fill="none" strokeLinecap="round" strokeLinejoin="round" />
      <path d="M11.5 17 H20.5" stroke="white" strokeWidth="2.2" strokeLinecap="round" opacity="0.8" />
    </svg>
  );
}

// Panel version (and commit); the runtime's own version shows on hover, flagged when it differs.
function Version() {
  const rt = useStore((s) => s.health?.version);
  const panel = `v${__APP_VERSION__}${__GIT_SHA__ ? ` (${__GIT_SHA__})` : ""}`;
  const runtime = rt ? `v${rt.version} (${rt.commit})` : "";
  const differs = !!rt && rt.version !== __APP_VERSION__;
  return (
    <span
      className={`hidden whitespace-nowrap text-[11px] tabular-nums md:inline ${differs ? "text-warn" : "text-muted"}`}
      title={`Panel ${panel}${runtime ? `\nRuntime ${runtime}` : ""}`}
    >
      {panel}
      {differs && ` · runtime v${rt!.version}`}
    </span>
  );
}

function SourceBadge() {
  const config = useStore((s) => s.config);
  const health = useStore((s) => s.health);
  const src = config?.source ?? (health?.source ? { ...health.source } : undefined);
  if (!src) return null;
  if (src.live)
    return (
      <span className="chip border-bad/30 bg-bad/10 text-bad">
        <span className="pulse h-1.5 w-1.5 rounded-full bg-bad" /> LIVE
      </span>
    );
  return (
    <span className="chip" title="Recorded video, played in a loop">
      <Repeat size={11} /> REPLAY{src.loop ? " · loop" : ""}
    </span>
  );
}

function ComputeBadge() {
  const c = useStore((s) => s.health?.compute ?? s.config?.compute);
  const fps = useStore((s) => s.health?.pipeline.fps);
  const lat = useStore((s) => s.health?.pipeline.latency_ms);
  const det = useStore((s) => s.health?.pipeline.detector_ms);
  if (!c) return null;
  const gpu = c.backend !== "cpu";
  const engine = c.backend === "tensorrt" ? "TensorRT" : "ONNX Runtime";
  return (
    <div className="hidden items-center gap-2 text-xs text-muted lg:flex">
      <span
        className={`chip ${gpu ? "border-ok/30 bg-ok/10 text-ok" : "border-warn/40 bg-warn/10 text-warn"}`}
        title={`${c.device} · ${c.model} ${c.input?.join("×")}`}
      >
        {gpu ? <Zap size={11} /> : <Cpu size={11} />}
        {gpu ? "GPU" : "CPU"} · {engine} {c.precision.toUpperCase()}
      </span>
      <span className="hidden max-w-[180px] truncate xl:inline" title={c.device}>
        {c.device}
      </span>
      {fps !== undefined && (
        <span className="whitespace-nowrap tabular-nums">
          {fps.toFixed(0)} fps · {det?.toFixed(0)} ms{lat !== undefined ? ` · ${lat.toFixed(0)} ms e2e` : ""}
        </span>
      )}
    </div>
  );
}

const THEMES: { v: ThemePref; icon: JSX.Element; label: string }[] = [
  { v: "light", icon: <Sun size={14} />, label: "Light" },
  { v: "dark", icon: <Moon size={14} />, label: "Dark" },
  { v: "system", icon: <Monitor size={14} />, label: "System" },
];

function ThemeSwitch() {
  const theme = useStore((s) => s.theme);
  const set = useStore((s) => s.set);
  return (
    <div className="seg" role="radiogroup" aria-label="Theme">
      {THEMES.map((t) => (
        <button key={t.v} data-on={theme === t.v} title={t.label} aria-label={t.label} onClick={() => set({ theme: t.v })}>
          {t.icon}
        </button>
      ))}
    </div>
  );
}

function SitePicker() {
  const site = useStore((s) => s.site);
  const seen = useStore((s) => s.sitesSeen);
  const name = useStore((s) => s.config?.site_name);
  const [sites, setSites] = useState<SiteInfo[] | null>(null);
  useEffect(() => {
    api<SiteInfo[]>("/sites").then(setSites).catch(() => setSites(null));
  }, []);
  const options = sites ? sites.map((s) => ({ id: s.id, name: s.name })) : seen.map((id) => ({ id, name: id }));
  if (options.length <= 1)
    return <span className="truncate text-sm font-medium text-muted">{name ?? site ?? "waiting for a site…"}</span>;
  return (
    <select className="input w-auto max-w-[220px] py-1" value={site ?? ""} onChange={(e) => selectSite(e.target.value)}>
      {options.map((o) => (
        <option key={o.id} value={o.id}>
          {o.name}
        </option>
      ))}
    </select>
  );
}

// Camera of a multi-camera site whose video, calibration and settings are shown (the twin shows all of them).
function CameraPicker() {
  const site = useStore((s) => s.site);
  const camera = useStore((s) => s.camera);
  const cams = useStore((s) => (s.site ? s.camerasSeen[s.site] : undefined));
  const fused = useStore((s) => s.fusedAt > 0);
  if (!site || !cams || cams.length <= 1) return null;
  return (
    <div className="flex items-center gap-1.5">
      <select className="input w-auto py-1" value={camera} onChange={(e) => selectCamera(e.target.value)} title="Camera shown in the video and the settings">
        {cams.map((c) => (
          <option key={c} value={c}>
            {c}
          </option>
        ))}
      </select>
      {fused && (
        <span className="chip" title={`Road users of all ${cams.length} cameras are fused in the twin`}>
          <Layers size={11} /> {cams.length} cams fused
        </span>
      )}
    </div>
  );
}

export function Header({ route }: { route: Route }) {
  const connected = useStore((s) => s.connected);
  const calibrated = useStore((s) => !!s.calibration);
  const healthAt = useStore((s) => s.healthAt);
  const [, tick] = useState(0);
  useEffect(() => {
    const t = setInterval(() => tick((x) => x + 1), 2000);
    return () => clearInterval(t);
  }, []);
  const stale = healthAt > 0 && performance.now() / 1000 - healthAt > 5;
  return (
    <header className="flex h-14 shrink-0 items-center gap-3 border-b border-line bg-panel px-4">
      <a href="#/live" className="flex items-center gap-2.5">
        <Logo />
        <span className="hidden whitespace-nowrap text-[15px] font-semibold tracking-tight md:inline">Junction Perception</span>
      </a>
      <Version />
      <div className="mx-1 h-6 w-px bg-line" />
      <SitePicker />
      <CameraPicker />
      <SourceBadge />
      {!connected && (
        <span className="chip border-bad/30 bg-bad/10 text-bad">
          <Radio size={11} /> broker offline
        </span>
      )}
      {connected && stale && (
        <span className="chip border-warn/40 bg-warn/10 text-warn">
          <Radio size={11} /> no data
        </span>
      )}
      {connected && !calibrated && healthAt > 0 && (
        <button className="chip border-warn/40 bg-warn/10 text-warn hover:bg-warn/20" onClick={() => navigate("settings/calibration")}>
          <TriangleAlert size={11} /> not calibrated
        </button>
      )}
      <div className="flex-1" />
      <ComputeBadge />
      <ThemeSwitch />
      {route.page === "settings" ? (
        <button className="btn" onClick={() => navigate("live")}>
          <ArrowLeft size={15} /> Live view
        </button>
      ) : (
        <button className="btn" onClick={() => navigate("settings/calibration")}>
          <Settings size={15} /> Settings
        </button>
      )}
    </header>
  );
}
