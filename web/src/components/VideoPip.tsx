import { Expand, Minimize2, Move, VideoOff, X } from "lucide-react";
import { useEffect, useState } from "react";
import { selectCamera } from "../mqtt";
import { useStore } from "../store";

const WIDTH = { s: 260, m: 400, l: 600 } as const;
const CORNERS = ["br", "bl", "tl", "tr"] as const;
const POS: Record<string, string> = {
  br: "bottom-4 right-4",
  bl: "bottom-4 left-4",
  tr: "top-14 right-4",
  tl: "top-14 left-4",
};

// Live view path of a camera (its runtime container): the first camera keeps the site's own name.
export const liveName = (site: string, camera: string) => (camera === "cam1" ? site : `${site}-${camera}`);

// One camera's MJPEG stream; reconnects a dropped stream (site restart, network).
function Stream({ site, camera }: { site: string; camera: string }) {
  const [failed, setFailed] = useState(false);
  const [nonce, setNonce] = useState(0);
  useEffect(() => {
    if (!failed) return;
    const t = setTimeout(() => {
      setFailed(false);
      setNonce((n) => n + 1);
    }, 4000);
    return () => clearTimeout(t);
  }, [failed]);
  if (failed)
    return (
      <div className="flex h-full w-full flex-col items-center justify-center gap-1 bg-black/80 text-xs text-white/70">
        <VideoOff size={18} /> camera view not available yet
      </div>
    );
  return (
    <img src={`/live/${liveName(site, camera)}/video.mjpg?n=${nonce}`} alt={`Live view of ${camera}`} className="h-full w-full object-contain"
      onError={() => setFailed(true)} />
  );
}

function Badge({ live, camera }: { live?: boolean; camera?: string }) {
  return (
    <span className="flex items-center gap-1">
      {live ? (
        <span className="flex items-center gap-1 rounded bg-bad px-1.5 py-0.5 text-[10px] font-bold text-white">
          <span className="pulse h-1.5 w-1.5 rounded-full bg-white" /> LIVE
        </span>
      ) : (
        <span className="rounded bg-black/60 px-1.5 py-0.5 text-[10px] font-bold text-white">REPLAY</span>
      )}
      {camera && <span className="rounded bg-black/60 px-1.5 py-0.5 text-[10px] font-bold text-white">{camera}</span>}
    </span>
  );
}

// Live camera views (MJPEG from the runtimes, boxes drawn there), docked in a corner of the 3D view: every
// camera of the site, stacked (two) or in a grid (more). Clicking a view selects that camera.
export function VideoPip() {
  const site = useStore((s) => s.site);
  const camera = useStore((s) => s.camera);
  const seen = useStore((s) => (s.site ? s.camerasSeen[s.site] : undefined));
  const cams = useStore((s) => s.cams);
  const corner = useStore((s) => s.videoCorner);
  const size = useStore((s) => s.videoSize);
  const visible = useStore((s) => s.videoVisible);
  const set = useStore((s) => s.set);
  const [full, setFull] = useState<string | null>(null);
  if (!site) return null;
  if (!visible)
    return (
      <button className={`btn absolute ${POS[corner]} z-10 shadow-card`} onClick={() => set({ videoVisible: true })}>
        Show camera
      </button>
    );
  const list = seen?.length ? seen : [camera];
  const multi = list.length > 1;
  const source = (c: string) => cams[c]?.config?.source;
  const aspect = (c: string) => {
    const s = source(c);
    return s && s.width && s.height ? `${s.width} / ${s.height}` : "16 / 9";
  };
  if (full)
    return (
      <div className="fixed inset-0 z-50 flex items-center justify-center bg-black/90 p-6" onClick={() => setFull(null)}>
        <div className="relative max-h-full max-w-full" style={{ aspectRatio: aspect(full), height: "100%" }}>
          <Stream site={site} camera={full} />
          <div className="absolute left-3 top-3"><Badge live={source(full)?.live} camera={multi ? full : undefined} /></div>
          <button className="btn absolute right-3 top-3" onClick={() => setFull(null)}>
            <Minimize2 size={14} /> Close
          </button>
        </div>
      </div>
    );
  // Several cameras share the width of one view at the chosen size, at most two per row.
  const cols = list.length > 2 ? 2 : 1;
  const left = corner.endsWith("l");
  return (
    <div className={`group absolute ${POS[corner]} z-10 grid gap-2`}
      style={{ gridTemplateColumns: `repeat(${cols}, minmax(0, 1fr))`, width: `min(${WIDTH[size] * (cols === 2 ? 1.4 : 1)}px, 45vw)`, maxHeight: "75%" }}>
      {list.map((c) => (
        <div key={c}
          className={`relative overflow-hidden rounded-xl border bg-black shadow-2xl ${multi && c === camera ? "border-accent" : "border-white/10"} ${multi ? "cursor-pointer" : ""}`}
          style={{ aspectRatio: aspect(c) }}
          onClick={() => multi && c !== camera && selectCamera(c)}
          title={multi ? (c === camera ? `${c} (selected)` : `Select ${c}`) : undefined}>
          <Stream site={site} camera={c} />
          <div className="absolute left-2 top-2"><Badge live={source(c)?.live} camera={multi ? c : undefined} /></div>
          <button className="absolute bottom-1.5 right-1.5 flex h-6 w-6 items-center justify-center rounded-md bg-black/60 text-white opacity-0 transition group-hover:opacity-100"
            title="Full screen" onClick={(e) => { e.stopPropagation(); setFull(c); }}>
            <Expand size={12} />
          </button>
        </div>
      ))}
      <div className={`absolute top-1.5 flex gap-1 opacity-0 transition group-hover:opacity-100 ${left ? "left-full ml-1.5 flex-col" : "right-full mr-1.5 flex-col"}`}>
        {(["s", "m", "l"] as const).map((k) => (
          <button key={k}
            className={`h-6 w-6 rounded-md text-[10px] font-bold ${size === k ? "bg-white text-black" : "bg-black/60 text-white"}`}
            onClick={() => set({ videoSize: k })}>
            {k.toUpperCase()}
          </button>
        ))}
        <button className="flex h-6 w-6 items-center justify-center rounded-md bg-black/60 text-white" title="Move to the next corner"
          onClick={() => set({ videoCorner: CORNERS[(CORNERS.indexOf(corner) + 1) % 4] })}>
          <Move size={12} />
        </button>
        <button className="flex h-6 w-6 items-center justify-center rounded-md bg-black/60 text-white" title="Hide" onClick={() => set({ videoVisible: false })}>
          <X size={12} />
        </button>
      </div>
    </div>
  );
}
