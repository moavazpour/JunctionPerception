import { Expand, Minimize2, Move, VideoOff, X } from "lucide-react";
import { useEffect, useState } from "react";
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

// Live camera view (MJPEG from the runtime, boxes drawn there), docked in a corner of the 3D view.
export function VideoPip() {
  const site = useStore((s) => s.site);
  const camera = useStore((s) => s.camera);
  const corner = useStore((s) => s.videoCorner);
  const size = useStore((s) => s.videoSize);
  const visible = useStore((s) => s.videoVisible);
  const source = useStore((s) => s.config?.source);
  const set = useStore((s) => s.set);
  const [full, setFull] = useState(false);
  const [failed, setFailed] = useState(false);
  const [nonce, setNonce] = useState(0);
  // Reconnect a dropped stream (site restart, network).
  useEffect(() => {
    if (!failed) return;
    const t = setTimeout(() => {
      setFailed(false);
      setNonce((n) => n + 1);
    }, 4000);
    return () => clearTimeout(t);
  }, [failed]);
  if (!site) return null;
  if (!visible)
    return (
      <button className={`btn absolute ${POS[corner]} z-10 shadow-card`} onClick={() => set({ videoVisible: true })}>
        Show camera
      </button>
    );
  const src = `/live/${liveName(site, camera)}/video.mjpg?n=${nonce}`;
  const aspect = source && source.width && source.height ? `${source.width} / ${source.height}` : "16 / 9";
  const badge = source?.live ? (
    <span className="flex items-center gap-1 rounded bg-bad px-1.5 py-0.5 text-[10px] font-bold text-white">
      <span className="pulse h-1.5 w-1.5 rounded-full bg-white" /> LIVE
    </span>
  ) : (
    <span className="rounded bg-black/60 px-1.5 py-0.5 text-[10px] font-bold text-white">REPLAY</span>
  );
  const img = failed ? (
    <div className="flex h-full w-full flex-col items-center justify-center gap-1 bg-black/80 text-xs text-white/70">
      <VideoOff size={18} /> camera view not available yet
    </div>
  ) : (
    <img src={src} alt="Live camera view" className="h-full w-full object-contain" onError={() => setFailed(true)} />
  );
  if (full)
    return (
      <div className="fixed inset-0 z-50 flex items-center justify-center bg-black/90 p-6" onClick={() => setFull(false)}>
        <div className="relative max-h-full max-w-full" style={{ aspectRatio: aspect, height: "100%" }}>
          {img}
          <div className="absolute left-3 top-3">{badge}</div>
          <button className="btn absolute right-3 top-3" onClick={() => setFull(false)}>
            <Minimize2 size={14} /> Close
          </button>
        </div>
      </div>
    );
  return (
    <div
      className={`group absolute ${POS[corner]} z-10 overflow-hidden rounded-xl border border-white/10 bg-black shadow-2xl`}
      style={{ width: `min(${WIDTH[size]}px, 45vw)`, aspectRatio: aspect, maxHeight: "60%" }}
    >
      {img}
      <div className="absolute left-2 top-2">{badge}</div>
      <div className="absolute right-1.5 top-1.5 flex gap-1 opacity-0 transition group-hover:opacity-100">
        {(["s", "m", "l"] as const).map((k) => (
          <button
            key={k}
            className={`h-6 w-6 rounded-md text-[10px] font-bold ${size === k ? "bg-white text-black" : "bg-black/60 text-white"}`}
            onClick={() => set({ videoSize: k })}
          >
            {k.toUpperCase()}
          </button>
        ))}
        <button
          className="flex h-6 w-6 items-center justify-center rounded-md bg-black/60 text-white"
          title="Move to the next corner"
          onClick={() => set({ videoCorner: CORNERS[(CORNERS.indexOf(corner) + 1) % 4] })}
        >
          <Move size={12} />
        </button>
        <button className="flex h-6 w-6 items-center justify-center rounded-md bg-black/60 text-white" title="Full screen" onClick={() => setFull(true)}>
          <Expand size={12} />
        </button>
        <button className="flex h-6 w-6 items-center justify-center rounded-md bg-black/60 text-white" title="Hide" onClick={() => set({ videoVisible: false })}>
          <X size={12} />
        </button>
      </div>
    </div>
  );
}
