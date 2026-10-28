"""Site manager: create, configure, start, stop and delete perception instances from the web panel.

One site = one junction with one or more cameras. Each camera has its source (an uploaded video or an
RTSP/HTTP stream), placement and calibration, and runs as its own `junction-perception` container (GPU, or
CPU when there is none), publishing to the shared MQTT broker under jp/<site>/<camera>/... and serving its
live view on port 8081 (proxied by the panel at /live/<container suffix>/). The first camera (cam1) keeps
the names of single-camera sites: container jp-site-<site>, state in <data>/state/<site>; further cameras
run as jp-site-<site>-<camera> with state in <data>/state/<site>/<camera>. With two or more cameras the
manager also fuses their road users into jp/<site>/fused/objects (fusion.py).

Site YAMLs live in <data>/sites (per-camera runtime configs in <data>/sites/<site>/), uploaded videos in
<data>/videos, runtime state (calibration, drawn map, settings) in <data>/state.

Environment:
  JP_DATA_HOST    host path of the data directory (mounted here at /data)
  JP_MODELS_HOST  host path of the models directory
  JP_NETWORK      docker network of the stack (to reach the broker)
  JP_IMAGE        runtime image (default junction-perception:latest)
  JP_IMAGE_CPU    runtime image for CPU-only sites (default: JP_IMAGE, which also contains the CPU backend)
  JP_DETECTOR     detector ONNX inside /models (default jp_y26s_960.onnx)
  JP_MAX_SITES    camera containers that may run at the same time
  JP_MQTT         broker for the fusion (default tcp://mosquitto:1883)
"""
from __future__ import annotations

import copy
import json
import math
import os
import re
import shutil
import time
import urllib.parse
import urllib.request
import zlib
from pathlib import Path
from typing import Literal

import docker
import yaml
from docker.types import DeviceRequest
from fastapi import FastAPI, File, HTTPException, UploadFile
from fastapi.middleware.gzip import GZipMiddleware
from pydantic import BaseModel, Field

DATA = Path("/data")
SITES, VIDEOS, STATE, OSM = DATA / "sites", DATA / "videos", DATA / "state", DATA / "osm"
for d in (SITES, VIDEOS, STATE, OSM):
    d.mkdir(parents=True, exist_ok=True)
HOST = os.environ.get("JP_DATA_HOST", "/data")
MODELS_HOST = os.environ.get("JP_MODELS_HOST", "/models")
NETWORK = os.environ.get("JP_NETWORK", "bridge")
IMAGE = os.environ.get("JP_IMAGE", "junction-perception:latest")
IMAGE_CPU = os.environ.get("JP_IMAGE_CPU", IMAGE)
DETECTOR = os.environ.get("JP_DETECTOR", "jp_y26s_v2.onnx")
MAX_SITES = int(os.environ.get("JP_MAX_SITES", "4"))
VERSION = next((p.read_text().strip() for p in (Path(__file__).with_name("VERSION"), Path(__file__).parent.parent / "VERSION") if p.is_file()), "dev")
VIDEO_EXT = {".mp4", ".mov", ".mkv", ".avi", ".m4v"}
LENSES = {"unknown", "phone_wide", "phone_2x", "phone_3x", "cctv"}
PRIMARY = "cam1"

app = FastAPI(title="JunctionPerception site manager")
app.add_middleware(GZipMiddleware, minimum_size=4096)
dock = docker.from_env()


def gpu_available() -> bool:
    try:
        return "nvidia" in (dock.info().get("Runtimes") or {})
    except Exception:
        return False


class CameraIn(BaseModel):
    video: str | None = None        # file name returned by /api/videos
    stream: str | None = None       # rtsp:// or http(s):// URL
    lat: float = Field(ge=-90, le=90)
    lon: float = Field(ge=-180, le=180)
    height_m: float = Field(default=7.0, gt=0.5, lt=200)
    bearing_deg: float = Field(default=0.0, ge=-360, le=360)
    lens: str = "unknown"


class SiteIn(CameraIn):
    name: str = Field(min_length=1, max_length=80)
    device: Literal["auto", "gpu", "cpu"] = "auto"


class CameraPatch(BaseModel):
    video: str | None = None
    stream: str | None = None
    loop: bool | None = None
    lat: float | None = Field(default=None, ge=-90, le=90)
    lon: float | None = Field(default=None, ge=-180, le=180)
    height_m: float | None = Field(default=None, gt=0.5, lt=200)
    bearing_deg: float | None = Field(default=None, ge=-360, le=360)
    lens: str | None = None


class SitePatch(CameraPatch):
    name: str | None = Field(default=None, min_length=1, max_length=80)
    device: Literal["auto", "gpu", "cpu"] | None = None


def slug(s: str) -> str:
    s = re.sub(r"[^a-z0-9]+", "-", s.lower()).strip("-")[:40]
    return s or "site"


def check_source(video: str | None, stream: str | None):
    if video and (not (VIDEOS / video).is_file() or "/" in video):
        raise HTTPException(400, "unknown video")
    if stream and not re.match(r"^(rtsp|rtsps|http|https)://", stream):
        raise HTTPException(400, "stream URL must start with rtsp:// or http(s)://")


# ------------------------------------------------------------------ cameras of a site

def cameras_of(cfg: dict) -> list[dict]:
    """The site's cameras: its `cameras:` list, or the single `camera:` of a site made before multi-camera."""
    return cfg["cameras"] if "cameras" in cfg else [cfg["camera"]]


def camera_of(cfg: dict, cam: str) -> dict:
    for c in cameras_of(cfg):
        if c.get("id", PRIMARY) == cam:
            return c
    raise HTTPException(404, "no such camera")


def suffix(sid: str, cam: str) -> str:
    """Container / live-view name of a camera: the first camera keeps the single-camera name."""
    return sid if cam == PRIMARY else f"{sid}-{cam}"


def cam_state(sid: str, cam: str) -> Path:
    return STATE / sid if cam == PRIMARY else STATE / sid / cam


def container(name: str):
    try:
        return dock.containers.get(f"jp-site-{name}")
    except docker.errors.NotFound:
        return None


def site_containers(sid: str) -> list:
    return dock.containers.list(all=True, filters={"label": f"jp.site={sid}"})


def running_cameras() -> int:
    return len(dock.containers.list(filters={"label": "jp.site"}))


def camera_yaml(s: CameraIn, cam: str) -> dict:
    source = f"file:///data/videos/{s.video}" if s.video else s.stream
    return {"id": cam, "source": source, "realtime": True, "loop": bool(s.video), "max_fps": 15,
            "lens": s.lens if s.lens in LENSES else "unknown",
            "position": {"lat": s.lat, "lon": s.lon, "height_m": s.height_m, "height_sigma": 1.5,
                         "bearing_deg": s.bearing_deg % 360}}


def site_yaml(sid: str, s: SiteIn) -> dict:
    return {
        "site": {"id": sid, "name": s.name},
        "cameras": [camera_yaml(s, PRIMARY)],
        "calibration": {"mode": "manual", "publish_snapshot": True},
        "map": {"learn_lanes": True},
        "models": {"detector": f"/models/{DETECTOR}", "segmentation": "/models/segformer_b2_cityscapes.onnx",
                   "fp16": True, "score_threshold": 0.25, "max_range_m": 250,
                   "backend": {"auto": "auto", "gpu": "tensorrt", "cpu": "cpu"}[s.device]},
        "preview": {"enabled": True, "port": 8081, "fps": 12, "width": 960},
        "output": {"topic_prefix": "jp", "mqtt": {"uri": "tcp://mosquitto:1883"}},
        "health": {"target_fps": 15},
    }


def runtime_config(sid: str, cfg: dict, cam: dict) -> dict:
    """Single-camera runtime config of one camera of a multi-camera site."""
    cid = cam.get("id", PRIMARY)
    out = {k: copy.deepcopy(v) for k, v in cfg.items() if k not in ("cameras", "camera")}
    out["camera"] = copy.deepcopy(cam)
    state = f"/data/state/{sid}" + ("" if cid == PRIMARY else f"/{cid}")
    out.setdefault("calibration", {})["file"] = f"{state}/calibration.json"
    out["state_dir"] = state
    o = out.setdefault("output", {"topic_prefix": "jp"})
    o.setdefault("mqtt", {"uri": "tcp://mosquitto:1883"})["client_id"] = f"jp-{suffix(sid, cid)}"
    o["station_id"] = 1000 + zlib.crc32(f"{sid}/{cid}".encode()) % 8999
    return out


def device_of(cfg: dict) -> str:
    return {"tensorrt": "gpu", "cpu": "cpu"}.get(cfg.get("models", {}).get("backend", "auto"), "auto")


def start_camera(sid: str, cfg: dict, cam: dict, sync: dict | None = None):
    cid = cam.get("id", PRIMARY)
    if (c := container(suffix(sid, cid))) is not None:
        c.remove(force=True)
    if "cameras" in cfg:
        (SITES / sid).mkdir(exist_ok=True)
        rc = runtime_config(sid, cfg, cam)
        if sync:
            rc["sync"] = sync
        (SITES / sid / f"{cid}.yaml").write_text(yaml.safe_dump(rc, sort_keys=False))
        conf = f"/data/sites/{sid}/{cid}.yaml"
    else:
        conf = f"/data/sites/{sid}.yaml"
    use_gpu = device_of(cfg) != "cpu" and gpu_available()
    dock.containers.run(
        IMAGE if use_gpu else IMAGE_CPU, ["--config", conf], name=f"jp-site-{suffix(sid, cid)}", detach=True,
        restart_policy={"Name": "unless-stopped"}, network=NETWORK, labels={"jp.site": sid, "jp.camera": cid},
        device_requests=[DeviceRequest(count=-1, capabilities=[["gpu"]])] if use_gpu else [],
        volumes={f"{HOST}/sites": {"bind": "/data/sites", "mode": "ro"}, f"{HOST}/videos": {"bind": "/data/videos", "mode": "ro"},
                 f"{HOST}/state": {"bind": "/data/state", "mode": "rw"}, MODELS_HOST: {"bind": "/models", "mode": "rw"}})


def start(sid: str):
    """Starts every camera of the site. Replayed videos of a multi-camera site play in lock-step: each runtime
    waits until all cameras have loaded their models, then all start from the first frame on one shared clock
    (runtime config `sync:`; the barrier lives in the site's state directory)."""
    cfg = yaml.safe_load((SITES / f"{sid}.yaml").read_text())
    for c in site_containers(sid):
        c.remove(force=True)
    cams = cameras_of(cfg)
    sync = None
    if "cameras" in cfg and len(cams) > 1:
        d = STATE / sid / ".sync"
        shutil.rmtree(d, ignore_errors=True)
        d.mkdir(parents=True)
        sync = {"dir": f"/data/state/{sid}/.sync", "cameras": [c.get("id", PRIMARY) for c in cams],
                "token": f"{time.time():.3f}", "timeout_s": 300}
    for cam in cams:
        start_camera(sid, cfg, cam, sync)


def stop(sid: str):
    for c in site_containers(sid):
        c.remove(force=True)


def describe_camera(sid: str, cam: dict) -> dict:
    cid = cam.get("id", PRIMARY)
    c = container(suffix(sid, cid))
    src = cam["source"]
    video = src.startswith("file:///data/videos/")
    return {"id": cid, "live": suffix(sid, cid),
            "source": src.replace("file:///data/videos/", "") if video else re.sub(r"//[^/@]*@", "//***@", src),
            "kind": "video" if video else "stream", "loop": bool(cam.get("loop", False)),
            "lens": cam.get("lens", "unknown"), "position": cam["position"], "status": c.status if c else "stopped",
            "calibrated": (cam_state(sid, cid) / "calibration.json").exists()}


def describe(p: Path) -> dict:
    cfg = yaml.safe_load(p.read_text())
    sid = cfg["site"]["id"]
    cams = [describe_camera(sid, c) for c in cameras_of(cfg)]
    first = cams[0]
    status = "running" if any(c["status"] == "running" for c in cams) else first["status"]
    # The first camera's fields at the top level, as for single-camera sites.
    return {"id": sid, "name": cfg["site"]["name"], "source": first["source"], "kind": first["kind"], "loop": first["loop"],
            "lens": first["lens"], "device": device_of(cfg), "position": first["position"], "status": status,
            "calibrated": all(c["calibrated"] for c in cams), "cameras": cams, "created": p.stat().st_mtime}


@app.get("/api/sites")
def list_sites():
    return sorted((describe(p) for p in SITES.glob("*.yaml")), key=lambda s: s["created"])


@app.post("/api/videos")
async def upload(file: UploadFile = File(...)):
    ext = Path(file.filename or "").suffix.lower()
    if ext not in VIDEO_EXT:
        raise HTTPException(400, f"unsupported video type {ext or '?'} (use {', '.join(sorted(VIDEO_EXT))})")
    name = f"{int(time.time())}_{slug(Path(file.filename).stem)}{ext}"
    with open(VIDEOS / name, "wb") as f:
        shutil.copyfileobj(file.file, f, length=8 << 20)
    return {"video": name, "bytes": (VIDEOS / name).stat().st_size}


def check_capacity(extra: int):
    if running_cameras() + extra > MAX_SITES:
        raise HTTPException(409, f"at most {MAX_SITES} cameras can run at the same time; stop a site first")


@app.post("/api/sites")
def create(s: SiteIn):
    if bool(s.video) == bool(s.stream):
        raise HTTPException(400, "give either an uploaded video or a stream URL")
    check_source(s.video, s.stream)
    check_capacity(1)
    sid, k = slug(s.name), 2
    while (SITES / f"{sid}.yaml").exists():
        sid, k = f"{slug(s.name)}-{k}", k + 1
    (SITES / f"{sid}.yaml").write_text(yaml.safe_dump(site_yaml(sid, s), sort_keys=False))
    start(sid)
    return describe(SITES / f"{sid}.yaml")


def _site(sid: str) -> Path:
    p = SITES / f"{slug(sid)}.yaml"
    if not p.exists():
        raise HTTPException(404, "no such site")
    return p


def _save(p: Path, cfg: dict):
    p.write_text(yaml.safe_dump(cfg, sort_keys=False))


def _running(sid: str) -> bool:
    return any(c.status == "running" for c in site_containers(sid))


@app.get("/api/sites/{sid}")
def get_site(sid: str):
    return describe(_site(sid))


def patch_camera(sid: str, cam: dict, s: CameraPatch) -> bool:
    """Applies a camera patch; True when the camera moved or got a new source (its calibration is removed)."""
    if s.video and s.stream:
        raise HTTPException(400, "give either a video or a stream URL")
    check_source(s.video, s.stream)
    old_source, old_pos = cam["source"], dict(cam["position"])
    if s.video:
        cam["source"], cam["loop"] = f"file:///data/videos/{s.video}", True
    if s.stream:
        cam["source"], cam["loop"] = s.stream, False
    if s.loop is not None and cam["source"].startswith("file://"):
        cam["loop"] = s.loop
    for k in ("lat", "lon", "height_m"):
        if getattr(s, k) is not None:
            cam["position"][k] = getattr(s, k)
    if s.bearing_deg is not None:
        cam["position"]["bearing_deg"] = s.bearing_deg % 360
    if s.lens is not None:
        if s.lens not in LENSES:
            raise HTTPException(400, f"lens must be one of {sorted(LENSES)}")
        cam["lens"] = s.lens
    moved = cam["source"] != old_source or any(abs(cam["position"][k] - old_pos[k]) > 1e-7 for k in ("lat", "lon"))
    if moved:
        (cam_state(sid, cam.get("id", PRIMARY)) / "calibration.json").unlink(missing_ok=True)
        if cam["source"] != old_source and old_source.startswith("file:///data/videos/"):
            (VIDEOS / old_source.rsplit("/", 1)[1]).unlink(missing_ok=True)
    return moved


@app.patch("/api/sites/{sid}")
def update_site(sid: str, s: SitePatch):
    """Changes the name, compute device, or the first camera's source, placement or lens; restarts the site if
    it was running. A new source or camera position invalidates that camera's calibration (it is removed)."""
    p = _site(sid)
    cfg = yaml.safe_load(p.read_text())
    if s.name:
        cfg["site"]["name"] = s.name
    if s.device is not None:
        cfg.setdefault("models", {})["backend"] = {"auto": "auto", "gpu": "tensorrt", "cpu": "cpu"}[s.device]
    cfg.setdefault("preview", {"enabled": True, "port": 8081, "fps": 12, "width": 960})
    patch_camera(p.stem, cameras_of(cfg)[0], s)
    _save(p, cfg)
    if _running(p.stem):
        start(p.stem)
    return describe(p)


@app.post("/api/sites/{sid}/cameras")
def add_camera(sid: str, s: CameraIn):
    """Adds a camera to the site. A single-camera site is converted to the camera list in place (its first
    camera keeps its container, state and calibration). The new camera gets the site's drawn layout."""
    p = _site(sid)
    if bool(s.video) == bool(s.stream):
        raise HTTPException(400, "give either an uploaded video or a stream URL")
    check_source(s.video, s.stream)
    cfg = yaml.safe_load(p.read_text())
    if "camera" in cfg:
        first = cfg.pop("camera")
        first["id"] = PRIMARY
        cfg["cameras"] = [first]
        cfg.get("calibration", {}).pop("file", None)
        cfg.pop("state_dir", None)
        cfg.get("output", {}).get("mqtt", {}).pop("client_id", None)
        cfg.get("output", {}).pop("station_id", None)
    ids = {c["id"] for c in cfg["cameras"]}
    k = 2
    while f"cam{k}" in ids:
        k += 1
    cid = f"cam{k}"
    running = _running(p.stem)
    if running:
        check_capacity(1)
    cam = camera_yaml(s, cid)
    cfg["cameras"].append(cam)
    st = cam_state(p.stem, cid)
    st.mkdir(parents=True, exist_ok=True)
    for f in ("user_map.json", "settings.json"):
        if (STATE / p.stem / f).exists():
            shutil.copy(STATE / p.stem / f, st / f)
    _save(p, cfg)
    if running:
        start(p.stem)  # all cameras again, so the replays stay in lock-step
    return describe(p)


@app.patch("/api/sites/{sid}/cameras/{cam}")
def update_camera(sid: str, cam: str, s: CameraPatch):
    p = _site(sid)
    cfg = yaml.safe_load(p.read_text())
    c = camera_of(cfg, cam)
    patch_camera(p.stem, c, s)
    _save(p, cfg)
    if _running(p.stem):
        start(p.stem)  # all cameras again, so the replays stay in lock-step
    return describe(p)


@app.delete("/api/sites/{sid}/cameras/{cam}")
def delete_camera(sid: str, cam: str):
    p = _site(sid)
    cfg = yaml.safe_load(p.read_text())
    c = camera_of(cfg, cam)
    if cam == PRIMARY or "cameras" not in cfg:
        raise HTTPException(400, "the first camera cannot be removed (delete the site instead)")
    if (ct := container(suffix(p.stem, cam))) is not None:
        ct.remove(force=True)
    if c["source"].startswith("file:///data/videos/"):
        (VIDEOS / c["source"].rsplit("/", 1)[1]).unlink(missing_ok=True)
    shutil.rmtree(cam_state(p.stem, cam), ignore_errors=True)
    (SITES / p.stem / f"{cam}.yaml").unlink(missing_ok=True)
    cfg["cameras"] = [x for x in cfg["cameras"] if x.get("id") != cam]
    _save(p, cfg)
    return describe(p)


@app.post("/api/sites/{sid}/start")
def start_site(sid: str):
    p = _site(sid)
    cfg = yaml.safe_load(p.read_text())
    check_capacity(len(cameras_of(cfg)) - len([c for c in site_containers(p.stem) if c.status == "running"]))
    start(p.stem)
    return describe(p)


class ModelIn(BaseModel):
    model: str | None = None   # detector ONNX relative to the models directory
    imgsz: int | None = Field(default=None, ge=0, le=2560)  # 0: the model's own input size
    backend: Literal["auto", "cuda", "tensorrt", "cpu"] | None = None


@app.post("/api/sites/{sid}/model")
def switch_model(sid: str, m: ModelIn):
    """Switches the detection model, its input size or backend on every camera of the site and restarts the
    site from the first frame (cameras in lock-step, tracks and statistics cleared). A new model runs at its
    own input size unless one is given. Applied at a restart, not mid-stream: the old model's tracks never mix
    with the new model's detections and all cameras run the same model."""
    p = _site(sid)
    if m.model is not None and (not m.model.endswith(".onnx") or ".." in Path(m.model).parts or m.model.startswith("/")):
        raise HTTPException(400, "unknown model")
    cfg = yaml.safe_load(p.read_text())
    check_capacity(len(cameras_of(cfg)) - len([c for c in site_containers(p.stem) if c.status == "running"]))
    for cam in cameras_of(cfg):
        st = cam_state(p.stem, cam.get("id", PRIMARY))
        st.mkdir(parents=True, exist_ok=True)
        sf = st / "settings.json"
        try:
            s = json.loads(sf.read_text()) if sf.exists() else {}
        except ValueError:
            s = {}
        if m.model is not None and m.model != s.get("model"):
            s["model"] = m.model
            s.pop("imgsz", None)
        if m.imgsz == 0:
            s.pop("imgsz", None)
        elif m.imgsz is not None:
            s["imgsz"] = m.imgsz
        if m.backend is not None:
            s["backend"] = m.backend
        sf.write_text(json.dumps(s, indent=1))
    start(p.stem)
    return describe(p)


@app.post("/api/sites/{sid}/stop")
def stop_site(sid: str):
    p = _site(sid)
    stop(p.stem)
    return describe(p)


@app.delete("/api/sites/{sid}")
def delete_site(sid: str):
    p = _site(sid)
    stop(p.stem)
    cfg = yaml.safe_load(p.read_text())
    for c in cameras_of(cfg):
        if c["source"].startswith("file:///data/videos/"):
            (VIDEOS / c["source"].rsplit("/", 1)[1]).unlink(missing_ok=True)
    shutil.rmtree(STATE / p.stem, ignore_errors=True)
    shutil.rmtree(SITES / p.stem, ignore_errors=True)
    p.unlink()
    return {"deleted": p.stem}


GT_CACHE: dict[tuple[str, float], dict] = {}


@app.get("/api/sites/{sid}/groundtruth")
def groundtruth(sid: str):
    """Recorded ground truth of a replayed dataset site (site YAML `groundtruth:`), compact, one entry per source
    video frame: [lat, lon, heading_deg, length, width, height, class, track, speed m/s or null, [cameras seeing it]].
    `groundtruth:` = {file: <path under the data dir>, cameras: {<camera id>: <dataset camera>},
    calibration: {<camera id>: <reference calibration json under the data dir>}}."""
    cfg = yaml.safe_load(_site(sid).read_text())
    g = cfg.get("groundtruth")
    if not g:
        raise HTTPException(404, "this site has no ground truth")
    path = DATA / g["file"]
    key = (str(path), path.stat().st_mtime)
    if key not in GT_CACHE:
        d = json.loads(path.read_text())
        ours = {v: k for k, v in g["cameras"].items()}  # dataset camera -> our camera id
        timeline = next(iter(g["cameras"].values()))
        frames = []
        for f in d["cameras"][timeline]["frames"]:
            frames.append([[round(o["lat"], 7), round(o["lon"], 7), round(o["heading_deg"], 1), round(o["length"], 2),
                            round(o["width"], 2), round(o["height"], 2), o["class"], o["track"],
                            None if o.get("speed") is None else round(o["speed"], 2),
                            sorted(ours[c] for c, v in o.get("visible", {}).items() if v and c in ours)]
                           for o in f["objects"]])
        GT_CACHE.clear()
        GT_CACHE[key] = {"dataset": d.get("dataset", ""), "fps": d["cameras"][timeline].get("fps"), "frames": frames}
    out = dict(GT_CACHE[key])
    out["twin"] = bool(g.get("twin", True))
    out["cameras"] = g["cameras"]
    out["calibration"] = {cam: json.loads((DATA / p).read_text()) for cam, p in g.get("calibration", {}).items()}
    return out


class TwinIn(BaseModel):
    on: bool


@app.put("/api/sites/{sid}/groundtruth/twin")
def groundtruth_twin(sid: str, t: TwinIn):
    """Twin source of a replayed dataset site: its recorded labels (default, `groundtruth.twin` absent or true; no
    detector detects reliably yet) or the perception output (`groundtruth.twin: false`). Kept in the site YAML, so every viewer sees the same."""
    p = _site(sid)
    cfg = yaml.safe_load(p.read_text())
    if not cfg.get("groundtruth"):
        raise HTTPException(404, "this site has no ground truth")
    cfg["groundtruth"]["twin"] = t.on
    _save(p, cfg)
    return {"twin": t.on}


@app.get("/api/sites/{sid}/logs")
def logs(sid: str, lines: int = 80, camera: str = PRIMARY):
    p = _site(sid)
    c = container(suffix(p.stem, camera))
    return {"logs": c.logs(tail=min(lines, 500)).decode(errors="replace") if c else ""}


OVERPASS = ["https://overpass-api.de/api/interpreter", "https://overpass.kumi.systems/api/interpreter",
            "https://maps.mail.ru/osm/tools/overpass/api/interpreter", "https://overpass.private.coffee/api/interpreter"]


@app.get("/api/osm")
def osm(lat: float, lon: float, r: int = 350):
    """Roads, crossings and traffic lights around a point (for the drawn base map), cached on disk: the public
    Overpass servers are often busy, so every mirror is tried and a site's map is fetched only once."""
    if not (-90 <= lat <= 90 and -180 <= lon <= 180 and 50 <= r <= 1500):
        raise HTTPException(400, "bad area")
    key = OSM / f"{lat:.5f}_{lon:.5f}_{r}.json"
    if key.exists():
        return json.loads(key.read_text())
    # A re-solved calibration moves the camera anchor by a few metres: reuse a cached area that covers this one.
    for p in OSM.glob("*.json"):
        try:
            clat, clon, cr = (float(v) for v in p.stem.split("_"))
        except ValueError:
            continue
        d = math.hypot((clat - lat) * 111320.0, (clon - lon) * 111320.0 * math.cos(math.radians(lat)))
        if d + r <= cr + 50:
            return json.loads(p.read_text())
    q = (f"[out:json][timeout:25];(way[\"highway\"](around:{r},{lat},{lon});"
         f"node[\"highway\"~\"traffic_signals|crossing|stop\"](around:{r},{lat},{lon}););out body geom;")
    data = urllib.parse.urlencode({"data": q}).encode()
    last = "no server answered"
    # One pass over the mirrors: the panel retries on its own and shows the aerial imagery meanwhile.
    for url in OVERPASS:
        try:
            req = urllib.request.Request(url, data=data, headers={"User-Agent": "junction-perception/0.2"})
            with urllib.request.urlopen(req, timeout=20) as resp:
                j = json.loads(resp.read())
            out = {"ways": [e for e in j.get("elements", []) if e.get("type") == "way" and e.get("geometry")],
                   "nodes": [e for e in j.get("elements", []) if e.get("type") == "node"]}
            if out["ways"]:
                key.write_text(json.dumps(out))
                return out
        except Exception as e:  # busy mirror, timeout: try the next one
            last = f"{url}: {e}"
    raise HTTPException(503, f"OpenStreetMap is not reachable ({last})")


@app.get("/api/health")
def health():
    return {"ok": True, "version": VERSION, "image": IMAGE, "gpu": gpu_available(), "detector": DETECTOR,
            "sites": len(list(SITES.glob("*.yaml"))), "max_sites": MAX_SITES}


def multi_camera_sites() -> dict[str, list[str]]:
    """Sites with two or more cameras (for the fusion), re-read on every call (the YAMLs are tiny)."""
    out = {}
    for p in SITES.glob("*.yaml"):
        try:
            cfg = yaml.safe_load(p.read_text())
            cams = [c.get("id", PRIMARY) for c in cameras_of(cfg)]
        except Exception:
            continue
        if len(cams) >= 2:
            out[cfg["site"]["id"]] = cams
    return out


@app.on_event("startup")
def _fusion():
    from fusion import Fusion

    Fusion(os.environ.get("JP_MQTT", "tcp://mosquitto:1883"), multi_camera_sites).start()


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8000)
