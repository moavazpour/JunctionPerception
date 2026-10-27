// Calibration inputs being edited (per site, kept in the browser until solved; the device keeps the last
// solved inputs and sends them back on calibration/inputs).
import { create } from "zustand";
import { save } from "../store";
import type { CalibInputs } from "../types";

export interface PairDraft {
  u?: number;
  v?: number;
  lat?: number;
  lon?: number;
}
export interface LineDraft {
  px: [number, number][];
  geo: [number, number][];
}
export type Tool = "point" | "line" | "vertical";

export interface Draft {
  site: string | null;
  points: PairDraft[];
  lines: LineDraft[];
  verticals: [number, number][][];
  lens: string;
  height: number;
  heightSigma: number;
  tool: Tool;
  openLine: number | null; // line being traced
  openVertical: number | null;
  set: (p: Partial<Draft>) => void;
}

export const useDraft = create<Draft>((set) => ({
  site: null,
  points: [],
  lines: [],
  verticals: [],
  lens: "phone_wide",
  height: 7,
  heightSigma: 1.5,
  tool: "point",
  openLine: null,
  openVertical: null,
  set: (p) => set(p),
}));

useDraft.subscribe((d) => {
  if (!d.site) return;
  save(`jp-calib-draft:${d.site}`, { points: d.points, lines: d.lines, verticals: d.verticals, lens: d.lens, height: d.height, heightSigma: d.heightSigma });
});

// Load the draft for a site: browser draft first, else the inputs the device last solved with.
export function loadDraft(site: string, device?: CalibInputs, defaults?: { lens?: string; height?: number; heightSigma?: number }) {
  let d: Partial<Draft> | null = null;
  try {
    const raw = localStorage.getItem(`jp-calib-draft:${site}`);
    if (raw) d = JSON.parse(raw);
  } catch {
    /* none */
  }
  if (!d && device) {
    d = {
      points: device.points.map((p) => ({ ...p })),
      lines: device.lines.map((l) => ({ px: l.px, geo: l.geo })),
      verticals: device.verticals,
      lens: device.lens,
      height: device.height_m ?? undefined,
      heightSigma: device.height_sigma,
    };
  }
  useDraft.getState().set({
    site,
    points: d?.points ?? [],
    lines: d?.lines ?? [],
    verticals: d?.verticals ?? [],
    lens: d?.lens ?? defaults?.lens ?? "phone_wide",
    height: d?.height ?? defaults?.height ?? 7,
    heightSigma: d?.heightSigma ?? defaults?.heightSigma ?? 1.5,
    openLine: null,
    openVertical: null,
  });
}

export const completePoints = (d: Draft) =>
  d.points.filter((p) => p.u !== undefined && p.lat !== undefined) as Required<PairDraft>[];
export const completeLines = (d: Draft) => d.lines.filter((l) => l.px.length >= 2 && l.geo.length === 2);
export const completeVerticals = (d: Draft) => d.verticals.filter((v) => v.length === 2);
