import { Activity, Camera, Cpu, Crosshair, MapPin, Palette, SlidersHorizontal } from "lucide-react";
import type { ReactNode } from "react";
import { navigate } from "../App";
import { CalibrationSection } from "../calib/CalibrationSection";
import { ComputeSection } from "./ComputeSection";
import { DisplaySection } from "./DisplaySection";
import { InferenceSection } from "./InferenceSection";
import { SitesSection } from "./SitesSection";
import { SourceSection } from "./SourceSection";
import { SystemSection } from "./SystemSection";

const SECTIONS: { id: string; label: string; icon: ReactNode; desc: string; el: () => ReactNode }[] = [
  { id: "calibration", label: "Calibration", icon: <Crosshair size={16} />, desc: "Place the camera on the map", el: () => <CalibrationSection /> },
  { id: "source", label: "Video source", icon: <Camera size={16} />, desc: "Video file or live stream", el: () => <SourceSection /> },
  { id: "inference", label: "Detection", icon: <SlidersHorizontal size={16} />, desc: "Model thresholds, live", el: () => <InferenceSection /> },
  { id: "compute", label: "Compute", icon: <Cpu size={16} />, desc: "GPU or CPU", el: () => <ComputeSection /> },
  { id: "display", label: "Display", icon: <Palette size={16} />, desc: "Theme and map", el: () => <DisplaySection /> },
  { id: "sites", label: "Sites", icon: <MapPin size={16} />, desc: "Cameras on this server", el: () => <SitesSection /> },
  { id: "system", label: "System", icon: <Activity size={16} />, desc: "Health and logs", el: () => <SystemSection /> },
];

export function SettingsPage({ section }: { section: string }) {
  const cur = SECTIONS.find((s) => s.id === section) ?? SECTIONS[0];
  return (
    <div className="flex h-full flex-col md:flex-row">
      <nav className="flex shrink-0 gap-1 overflow-x-auto border-b border-line bg-panel p-2 md:w-60 md:flex-col md:border-b-0 md:border-r md:p-3">
        <div className="card-title hidden px-3 pb-2 pt-1 md:block">Settings</div>
        {SECTIONS.map((s) => (
          <button
            key={s.id}
            onClick={() => navigate(`settings/${s.id}`)}
            className={`flex shrink-0 items-center gap-3 rounded-lg px-3 py-2 text-left transition ${
              s.id === cur.id ? "bg-accent/10 text-accent" : "text-fg hover:bg-panel2"
            }`}
          >
            {s.icon}
            <span>
              <span className="block text-sm font-medium">{s.label}</span>
              <span className="hidden text-[11px] text-muted md:block">{s.desc}</span>
            </span>
          </button>
        ))}
      </nav>
      <div className="min-h-0 flex-1 overflow-y-auto">{cur.el()}</div>
    </div>
  );
}

export function Page({ title, desc, children, wide }: { title: string; desc?: string; children: ReactNode; wide?: boolean }) {
  return (
    <div className={`mx-auto w-full ${wide ? "max-w-[1500px]" : "max-w-4xl"} p-4 md:p-6`}>
      <h1 className="text-xl font-semibold tracking-tight">{title}</h1>
      {desc && <p className="mt-1 text-sm text-muted">{desc}</p>}
      <div className="mt-5 space-y-4">{children}</div>
    </div>
  );
}

export function Row({ label, hint, children }: { label: string; hint?: string; children: ReactNode }) {
  return (
    <div className="flex flex-col gap-2 border-t border-line py-3 first:border-t-0 sm:flex-row sm:items-center sm:gap-6">
      <div className="sm:w-64">
        <div className="text-sm font-medium">{label}</div>
        {hint && <div className="text-xs text-muted">{hint}</div>}
      </div>
      <div className="min-w-0 flex-1">{children}</div>
    </div>
  );
}

export function Toggle({ on, onChange, disabled }: { on: boolean; onChange: (v: boolean) => void; disabled?: boolean }) {
  return (
    <button
      role="switch"
      aria-checked={on}
      disabled={disabled}
      onClick={() => onChange(!on)}
      className={`relative h-6 w-11 rounded-full transition disabled:opacity-50 ${on ? "bg-accent" : "bg-line"}`}
    >
      <span className={`absolute top-0.5 h-5 w-5 rounded-full bg-white shadow transition ${on ? "left-[22px]" : "left-0.5"}`} />
    </button>
  );
}

export function NoSite() {
  return <div className="card p-6 text-sm text-muted">No site is running yet. Create one under Sites.</div>;
}
