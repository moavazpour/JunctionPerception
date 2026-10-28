import { useEffect, useRef } from "react";
import { coveredCameras, gtFrame } from "../evaluation";
import { geoToEnu } from "../geo";
import { Scene3D } from "../scene3d";
import { resolvedTheme, useStore } from "../store";

let shared: Scene3D | null = null;
export const sceneView = (kind: "orbit" | "top" | "camera") => shared?.view(kind);

// The 3D digital twin. State flows in imperatively (store subscription), so React never re-renders it.
export function SceneView() {
  const host = useRef<HTMLDivElement>(null);
  useEffect(() => {
    const s = new Scene3D(host.current!);
    shared = s;
    const st = useStore.getState();
    s.onSelect = (id) => useStore.getState().set({ selected: id });
    s.setTheme(resolvedTheme(st.theme));
    s.setMapMode(st.mapMode);
    s.showAllLabels = st.labels;
    let viewed = false;
    const apply = (cur: typeof st, prev?: typeof st) => {
      if (cur.config && cur.config !== prev?.config) s.setAnchor(cur.config.placement.lat, cur.config.placement.lon);
      if (cur.snapshot?.anchor && cur.snapshot !== prev?.snapshot) s.setAnchor(cur.snapshot.anchor.lat, cur.snapshot.anchor.lon);
      if (cur.map && cur.map !== prev?.map) {
        s.setMap(cur.map.map, cur.map.camera_model);
        if (!viewed) {
          s.view("orbit");
          viewed = true;
        }
      }
      if (cur.objects !== prev?.objects) s.updateObjects(cur.objects, performance.now() / 1000, cur.objectsTs);
      if (cur.detectors && cur.detectors !== prev?.detectors) s.setDetectors(cur.detectors);
      if (cur.occlusion !== prev?.occlusion || cur.occlusionLayer !== prev?.occlusionLayer)
        s.setOcclusion(cur.occlusion ?? null, cur.occlusionLayer);
      if (cur.theme !== prev?.theme) s.setTheme(resolvedTheme(cur.theme));
      if (cur.mapMode !== prev?.mapMode) s.setMapMode(cur.mapMode);
      if (cur.offLane !== prev?.offLane) s.offLane = cur.offLane;
      if (cur.gtVisible !== prev?.gtVisible || cur.gt !== prev?.gt || (cur.gtVisible && cur.sourceFrame !== prev?.sourceFrame)) {
        const a = cur.map?.camera_model?.anchor;
        s.setGroundTruth(
          cur.gtVisible && cur.gt && a
            ? gtFrame(cur.gt, cur.sourceFrame, coveredCameras(cur)).map((o) => {
                const [x, y] = geoToEnu(a, o[0], o[1]);
                return { x, y, yaw: ((90 - o[2]) * Math.PI) / 180, length: o[3], width: o[4], height: o[5] };
              })
            : [],
        );
      }
      if (cur.cams !== prev?.cams || cur.camera !== prev?.camera)
        s.setOtherCameras(
          Object.entries(cur.cams)
            .filter(([id, c]) => id !== cur.camera && c.calibration)
            .map(([, c]) => c.calibration!),
        );
      if (cur.focusLanes !== prev?.focusLanes) s.setFocusLanes(cur.focusLanes);
      if (cur.userMap !== prev?.userMap) s.setSignalLayout(cur.userMap);
      if (cur.signals !== prev?.signals || cur.userMap !== prev?.userMap || cur.map !== prev?.map) s.setSignalStates(cur.signals);
      if (cur.labels !== prev?.labels) s.showAllLabels = cur.labels;
      if (cur.selected !== prev?.selected && s.selected !== cur.selected) s.selected = cur.selected;
      if (cur.events !== prev?.events && prev && cur.events[0] && cur.events[0] !== prev.events[0]) s.flashEvent(cur.events[0]);
    };
    apply(st);
    const unsub = useStore.subscribe((cur, prev) => apply(cur, prev));
    // Follow the system theme while "system" is selected.
    const mq = window.matchMedia?.("(prefers-color-scheme: dark)");
    const onSys = () => s.setTheme(resolvedTheme(useStore.getState().theme));
    mq?.addEventListener("change", onSys);
    return () => {
      unsub();
      mq?.removeEventListener("change", onSys);
      s.dispose();
      if (shared === s) shared = null;
    };
  }, []);
  return <div ref={host} className="absolute inset-0 overflow-hidden" />;
}
