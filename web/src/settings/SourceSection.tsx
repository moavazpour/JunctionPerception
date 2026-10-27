import { Upload } from "lucide-react";
import { useEffect, useState } from "react";
import { api, json, managerHealth, uploadVideo, type SiteInfo } from "../api";
import { useStore } from "../store";
import { CameraPlacer, LENSES } from "./SitesSection";
import { NoSite, Page, Row, Toggle } from "./SettingsPage";

export function SourceSection() {
  const site = useStore((s) => s.site);
  const config = useStore((s) => s.config);
  const [info, setInfo] = useState<SiteInfo | null>(null);
  const [hasMgr, setHasMgr] = useState(false);
  const [kind, setKind] = useState<"video" | "stream">("video");
  const [file, setFile] = useState<File | null>(null);
  const [url, setUrl] = useState("");
  const [msg, setMsg] = useState<{ t: string; bad?: boolean } | null>(null);
  const [place, setPlace] = useState<{ lat: number; lon: number; bearing: number } | null>(null);
  const [height, setHeight] = useState<number | null>(null);
  useEffect(() => {
    managerHealth().then((h) => setHasMgr(!!h));
    if (site) api<SiteInfo>(`/sites/${site}`).then((i) => { setInfo(i); setKind(i.kind); }).catch(() => setInfo(null));
  }, [site]);
  if (!config) return <Page title="Video source"><NoSite /></Page>;
  const src = config.source;
  const patch = async (body: object, note: string) => {
    if (!site) return;
    try {
      setMsg({ t: note });
      setInfo(await api<SiteInfo>(`/sites/${site}`, json("PATCH", body)));
      setMsg({ t: "Saved. The site restarts with the new settings." });
    } catch (e) {
      setMsg({ t: (e as Error).message, bad: true });
    }
  };
  const changeSource = async () => {
    try {
      if (kind === "video") {
        if (!file) throw new Error("Choose a video file.");
        const video = await uploadVideo(file, (p) => setMsg({ t: `Uploading ${p} %…` }));
        await patch({ video }, "Switching to the new video…");
      } else {
        if (!/^(rtsp|rtsps|http|https):\/\//.test(url.trim())) throw new Error("The stream URL must start with rtsp:// or http(s)://");
        await patch({ stream: url.trim() }, "Switching to the stream…");
      }
    } catch (e) {
      setMsg({ t: (e as Error).message, bad: true });
    }
  };
  return (
    <Page title="Video source" desc="What the site analyses and where the camera stands.">
      <section className="card px-5 py-2">
        <Row label="Current source">
          <div className="flex flex-wrap items-center gap-2 text-sm">
            {src.live ? <span className="chip border-bad/30 bg-bad/10 text-bad">LIVE stream</span> : <span className="chip">Recorded video</span>}
            <span className="truncate text-muted">{info?.source ?? src.uri}</span>
          </div>
        </Row>
        <Row label="Resolution">{src.width} × {src.height} px · {src.fps.toFixed(0)} fps</Row>
        {!src.live && (
          <Row label="Play in a loop" hint="Without a live camera the video repeats, so the twin keeps showing traffic.">
            <Toggle on={info?.loop ?? src.loop} disabled={!hasMgr} onChange={(v) => patch({ loop: v }, "Saving…")} />
          </Row>
        )}
      </section>
      {hasMgr && info && (
        <>
          <section className="card space-y-3 p-5">
            <div className="text-sm font-semibold">Change the source</div>
            <div className="seg">
              <button data-on={kind === "video"} onClick={() => setKind("video")}>Video file</button>
              <button data-on={kind === "stream"} onClick={() => setKind("stream")}>Live stream (RTSP)</button>
            </div>
            {kind === "video" ? (
              <input type="file" accept="video/*" className="input" onChange={(e) => setFile(e.target.files?.[0] ?? null)} />
            ) : (
              <input className="input" placeholder="rtsp://user:password@camera-ip:554/stream1" value={url} onChange={(e) => setUrl(e.target.value)} />
            )}
            <div className="text-xs text-muted">A new source (another camera view) needs a new calibration; the current one is removed.</div>
            <button className="btn-primary" onClick={changeSource}>
              <Upload size={14} /> Use this source
            </button>
          </section>
          <section className="card space-y-3 p-5">
            <div className="text-sm font-semibold">Camera placement</div>
            <CameraPlacer
              value={place ?? { lat: info.position.lat, lon: info.position.lon, bearing: info.position.bearing_deg }}
              onChange={setPlace}
            />
            <div className="grid gap-4 md:grid-cols-2">
              <div>
                <label className="label">Height above the road (estimate, m)</label>
                <input type="number" className="input w-28" min={1} max={100} step={0.5} value={height ?? info.position.height_m}
                  onChange={(e) => setHeight(Number(e.target.value))} />
              </div>
              <div>
                <label className="label">Lens</label>
                <select className="input" value={info.lens} onChange={(e) => patch({ lens: e.target.value }, "Saving…")}>
                  {LENSES.map((l) => (
                    <option key={l.v} value={l.v}>{l.label}</option>
                  ))}
                </select>
              </div>
            </div>
            <button
              className="btn"
              disabled={!place && height === null}
              onClick={() => patch({ ...(place ? { lat: place.lat, lon: place.lon, bearing_deg: place.bearing } : {}), ...(height !== null ? { height_m: height } : {}) }, "Saving…")}
            >
              Save placement
            </button>
            <div className="text-xs text-muted">Moving the camera position removes the calibration.</div>
          </section>
        </>
      )}
      {msg && <div className={`text-xs ${msg.bad ? "text-bad" : "text-muted"}`}>{msg.t}</div>}
    </Page>
  );
}
