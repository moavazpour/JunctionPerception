import { Eye, Focus, Layers, Map as MapIcon, Satellite, Tag, Target, Video } from "lucide-react";
import { GroundTruthPanel, useGroundTruth } from "./GroundTruthPanel";
import { useState } from "react";
import { useStore } from "../store";
import { SceneView, sceneView } from "./SceneView";
import { Sidebar } from "./Sidebar";
import { VideoPip } from "./VideoPip";

function Toolbar() {
  const mapMode = useStore((s) => s.mapMode);
  const labels = useStore((s) => s.labels);
  const occ = useStore((s) => s.occlusionLayer);
  const focus = useStore((s) => s.focusLanes);
  const hasGt = useStore((s) => !!s.gt);
  const gtVisible = useStore((s) => s.gtVisible);
  const videoVisible = useStore((s) => s.videoVisible);
  const calibrated = useStore((s) => !!s.map);
  const set = useStore((s) => s.set);
  const [view, setView] = useState<"orbit" | "top" | "camera">("orbit");
  const go = (v: typeof view) => {
    setView(v);
    sceneView(v);
  };
  return (
    <div className="absolute left-3 top-3 z-10 flex flex-wrap items-center gap-2">
      <div className="seg bg-panel/90 shadow-card backdrop-blur" role="radiogroup" aria-label="View">
        {(
          [
            ["orbit", "Chase"],
            ["top", "Top"],
            ["camera", "Camera"],
          ] as const
        ).map(([k, l]) => (
          <button key={k} data-on={view === k} disabled={!calibrated} onClick={() => go(k)}>
            {l}
          </button>
        ))}
      </div>
      <div className="seg bg-panel/90 shadow-card backdrop-blur" role="radiogroup" aria-label="Base map">
        <button data-on={mapMode === "stylized"} onClick={() => set({ mapMode: "stylized" })} title="Road map drawn from OpenStreetMap">
          <span className="flex items-center gap-1">
            <MapIcon size={12} /> Map
          </span>
        </button>
        <button data-on={mapMode === "aerial"} onClick={() => set({ mapMode: "aerial" })} title="Aerial imagery">
          <span className="flex items-center gap-1">
            <Satellite size={12} /> Aerial
          </span>
        </button>
      </div>
      <div className="seg bg-panel/90 shadow-card backdrop-blur">
        <button data-on={labels} onClick={() => set({ labels: !labels })} title="Speed labels on every road user">
          <span className="flex items-center gap-1">
            <Tag size={12} /> Labels
          </span>
        </button>
        <button data-on={focus} onClick={() => set({ focusLanes: !focus })} title="Fade the map and the road users outside the drawn lanes">
          <span className="flex items-center gap-1">
            <Focus size={12} /> Lanes
          </span>
        </button>
        <button data-on={occ} onClick={() => set({ occlusionLayer: !occ })} title="Areas the camera cannot see">
          <span className="flex items-center gap-1">
            <Eye size={12} /> Occlusion
          </span>
        </button>
        {hasGt && (
          <button data-on={gtVisible} onClick={() => set({ gtVisible: !gtVisible })} title="Ground-truth boxes and accuracy (recorded dataset)">
            <span className="flex items-center gap-1">
              <Target size={12} /> Ground truth
            </span>
          </button>
        )}
        <button data-on={videoVisible} onClick={() => set({ videoVisible: !videoVisible })} title="Camera view">
          <span className="flex items-center gap-1">
            <Video size={12} /> Camera
          </span>
        </button>
      </div>
    </div>
  );
}

function Legend() {
  return (
    <div className="pointer-events-none absolute bottom-3 left-3 z-0 hidden items-center gap-1.5 rounded-lg bg-panel/80 px-2 py-1 text-[10px] text-muted backdrop-blur md:flex">
      <Layers size={11} /> Map data © OpenStreetMap contributors · Imagery © Google
    </div>
  );
}

export function LiveView() {
  useGroundTruth();
  return (
    <div className="flex h-full flex-col lg:flex-row">
      <div className="relative min-h-[55vh] flex-1 lg:min-h-0">
        <SceneView />
        <Toolbar />
        <VideoPip />
        <GroundTruthPanel />
        <Legend />
      </div>
      <Sidebar />
    </div>
  );
}
