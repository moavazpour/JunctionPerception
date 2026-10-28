import { useStore } from "../store";
import { Page, Row, Toggle } from "./SettingsPage";

// Per-browser display preferences.
export function DisplaySection() {
  const s = useStore();
  return (
    <Page title="Display" desc="Saved in this browser only.">
      <section className="card px-5 py-2">
        <Row label="Theme" hint="System follows the operating system's light or dark setting.">
          <div className="seg">
            {(["light", "dark", "system"] as const).map((t) => (
              <button key={t} data-on={s.theme === t} onClick={() => s.set({ theme: t })}>
                {t[0].toUpperCase() + t.slice(1)}
              </button>
            ))}
          </div>
        </Row>
        <Row label="Base map" hint="Map: roads, lanes and crossings drawn from OpenStreetMap. Aerial: Google satellite imagery (also shown while the road map cannot be loaded).">
          <div className="seg">
            <button data-on={s.mapMode === "stylized"} onClick={() => s.set({ mapMode: "stylized" })}>Map</button>
            <button data-on={s.mapMode === "aerial"} onClick={() => s.set({ mapMode: "aerial" })}>Aerial</button>
          </div>
        </Row>
        <Row label="Focus on drawn lanes" hint="Fade the map outside the lanes drawn in calibration, so the monitored lanes stand out.">
          <Toggle on={s.focusLanes} onChange={(v) => s.set({ focusLanes: v })} />
        </Row>
        <Row label="Road users off the drawn lanes" hint="While focusing on the drawn lanes: road users off them are shown normally, as faint ghosts, or hidden.">
          <div className="seg">
            {(["show", "ghost", "hide"] as const).map((k) => (
              <button key={k} data-on={s.offLane === k} onClick={() => s.set({ offLane: k })}>
                {{ show: "Show", ghost: "Ghost", hide: "Hide" }[k]}
              </button>
            ))}
          </div>
        </Row>
        <Row label="Speed labels" hint="A speed tag above every road user (the selected one always has one).">
          <Toggle on={s.labels} onChange={(v) => s.set({ labels: v })} />
        </Row>
        <Row label="Camera view" hint="The live camera image in a corner of the 3D view.">
          <Toggle on={s.videoVisible} onChange={(v) => s.set({ videoVisible: v })} />
        </Row>
        <Row label="Camera view size">
          <div className="seg">
            {(["s", "m", "l"] as const).map((k) => (
              <button key={k} data-on={s.videoSize === k} onClick={() => s.set({ videoSize: k })}>
                {{ s: "Small", m: "Medium", l: "Large" }[k]}
              </button>
            ))}
          </div>
        </Row>
        <Row label="Camera view corner">
          <div className="seg">
            {(["tl", "tr", "bl", "br"] as const).map((k) => (
              <button key={k} data-on={s.videoCorner === k} onClick={() => s.set({ videoCorner: k })}>
                {{ tl: "Top left", tr: "Top right", bl: "Bottom left", br: "Bottom right" }[k]}
              </button>
            ))}
          </div>
        </Row>
        <Row label="Occlusion layer" hint="Shade road areas the camera cannot see.">
          <Toggle on={s.occlusionLayer} onChange={(v) => s.set({ occlusionLayer: v })} />
        </Row>
      </section>
    </Page>
  );
}
