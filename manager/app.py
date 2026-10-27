"""Site manager: create, configure, start, stop and delete perception instances from the web panel.

One site = one camera source (an uploaded video or an RTSP/HTTP stream) + its placement. Each site runs
as its own `junction-perception` container (GPU, or CPU when there is none), publishing to the shared MQTT
broker under jp/<site>/cam1/... and serving its live view on port 8081 (proxied by the panel at
/live/<site>/). Site YAMLs live in <data>/sites, uploaded videos in <data>/videos, runtime state
(calibration, drawn map, settings) in <data>/state.

Environment:
  JP_DATA_HOST    host path of the data directory (mounted here at /data)
  JP_MODELS_HOST  host path of the models directory
  JP_NETWORK      docker network of the stack (to reach the broker)
  JP_IMAGE        runtime image (default junction-perception:latest)
  JP_IMAGE_CPU    runtime image for CPU-only sites (default: JP_IMAGE, which also contains the CPU backend)
  JP_DETECTOR     detector ONNX inside /models (default jp_y26s_960.onnx)
"""
from __future__ import annotations

import json
import os
import re
import shutil
import time
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Literal

import docker
import yaml
from docker.types import DeviceRequest
from fastapi import FastAPI, File, HTTPException, UploadFile
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
VIDEO_EXT = {".mp4", ".mov", ".mkv", ".avi", ".m4v"}
LENSES = {"unknown", "phone_wide", "phone_2x", "phone_3x", "cctv"}

app = FastAPI(title="JunctionPerception site manager")
dock = docker.from_env()


def gpu_available() -> bool:
    try:
        return "nvidia" in (dock.info().get("Runtimes") or {})
    except Exception:
        return False


class SiteIn(BaseModel):
    name: str = Field(min_length=1, max_length=80)
    video: str | None = None        # file name returned by /api/videos
    stream: str | None = None       # rtsp:// or http(s):// URL
    lat: float = Field(ge=-90, le=90)
    lon: float = Field(ge=-180, le=180)
    height_m: float = Field(default=7.0, gt=0.5, lt=200)
    bearing_deg: float = Field(default=0.0, ge=-360, le=360)
    lens: str = "unknown"
    device: Literal["auto", "gpu", "cpu"] = "auto"


class SitePatch(BaseModel):
    name: str | None = Field(default=None, min_length=1, max_length=80)
    video: str | None = None
    stream: str | None = None
    loop: bool | None = None
    lat: float | None = Field(default=None, ge=-90, le=90)
    lon: float | None = Field(default=None, ge=-180, le=180)
    height_m: float | None = Field(default=None, gt=0.5, lt=200)
    bearing_deg: float | None = Field(default=None, ge=-360, le=360)
    lens: str | None = None
    device: Literal["auto", "gpu", "cpu"] | None = None


def slug(s: str) -> str:
    s = re.sub(r"[^a-z0-9]+", "-", s.lower()).strip("-")[:40]
    return s or "site"


def container(site: str):
    try:
        return dock.containers.get(f"jp-site-{site}")
    except docker.errors.NotFound:
        return None


def check_source(video: str | None, stream: str | None):
    if video and (not (VIDEOS / video).is_file() or "/" in video):
        raise HTTPException(400, "unknown video")
    if stream and not re.match(r"^(rtsp|rtsps|http|https)://", stream):
        raise HTTPException(400, "stream URL must start with rtsp:// or http(s)://")


def site_yaml(sid: str, s: SiteIn) -> dict:
    source = f"file:///data/videos/{s.video}" if s.video else s.stream
    return {
        "site": {"id": sid, "name": s.name},
        "camera": {"id": "cam1", "source": source, "realtime": True, "loop": bool(s.video), "max_fps": 15,
                   "lens": s.lens if s.lens in LENSES else "unknown",
                   "position": {"lat": s.lat, "lon": s.lon, "height_m": s.height_m, "height_sigma": 1.5,
                                "bearing_deg": s.bearing_deg % 360}},
        "calibration": {"mode": "manual", "file": f"/data/state/{sid}/calibration.json", "publish_snapshot": True},
        "map": {"learn_lanes": True},
        "models": {"detector": f"/models/{DETECTOR}", "segmentation": "/models/segformer_b2_cityscapes.onnx",
                   "fp16": True, "score_threshold": 0.25, "max_range_m": 250,
                   "backend": {"auto": "auto", "gpu": "tensorrt", "cpu": "cpu"}[s.device]},
        "preview": {"enabled": True, "port": 8081, "fps": 12, "width": 960},
        "output": {"topic_prefix": "jp", "mqtt": {"uri": "tcp://mosquitto:1883", "client_id": f"jp-{sid}"},
                   "station_id": 1000 + (abs(hash(sid)) % 8999)},
        "health": {"target_fps": 15},
        "state_dir": f"/data/state/{sid}",
    }


def device_of(cfg: dict) -> str:
    return {"tensorrt": "gpu", "cpu": "cpu"}.get(cfg.get("models", {}).get("backend", "auto"), "auto")


def start(sid: str):
    c = container(sid)
    if c is not None:
        c.remove(force=True)
    cfg = yaml.safe_load((SITES / f"{sid}.yaml").read_text())
    dev = device_of(cfg)
    use_gpu = dev != "cpu" and gpu_available()
    dock.containers.run(
        IMAGE if use_gpu else IMAGE_CPU, ["--config", f"/data/sites/{sid}.yaml"], name=f"jp-site-{sid}", detach=True,
        restart_policy={"Name": "unless-stopped"}, network=NETWORK, labels={"jp.site": sid},
        device_requests=[DeviceRequest(count=-1, capabilities=[["gpu"]])] if use_gpu else [],
        volumes={f"{HOST}/sites": {"bind": "/data/sites", "mode": "ro"}, f"{HOST}/videos": {"bind": "/data/videos", "mode": "ro"},
                 f"{HOST}/state": {"bind": "/data/state", "mode": "rw"}, MODELS_HOST: {"bind": "/models", "mode": "rw"}})


def describe(p: Path) -> dict:
    cfg = yaml.safe_load(p.read_text())
    sid = cfg["site"]["id"]
    c = container(sid)
    cal = STATE / sid / "calibration.json"
    src = cfg["camera"]["source"]
    video = src.startswith("file:///data/videos/")
    return {"id": sid, "name": cfg["site"]["name"],
            "source": src.replace("file:///data/videos/", "") if video else re.sub(r"//[^/@]*@", "//***@", src),
            "kind": "video" if video else "stream", "loop": bool(cfg["camera"].get("loop", False)),
            "lens": cfg["camera"].get("lens", "unknown"), "device": device_of(cfg),
            "position": cfg["camera"]["position"], "status": c.status if c else "stopped",
            "calibrated": cal.exists(), "created": p.stat().st_mtime}


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


@app.post("/api/sites")
def create(s: SiteIn):
    if bool(s.video) == bool(s.stream):
        raise HTTPException(400, "give either an uploaded video or a stream URL")
    check_source(s.video, s.stream)
    running = [p for p in SITES.glob("*.yaml") if (c := container(p.stem)) is not None and c.status == "running"]
    if len(running) >= MAX_SITES:
        raise HTTPException(409, f"at most {MAX_SITES} sites can run at the same time; stop one first")
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


@app.get("/api/sites/{sid}")
def get_site(sid: str):
    return describe(_site(sid))


@app.patch("/api/sites/{sid}")
def update_site(sid: str, s: SitePatch):
    """Changes the source, placement, lens or compute device; restarts the site if it was running.
    A new source or camera position invalidates the calibration (it is removed)."""
    p = _site(sid)
    cfg = yaml.safe_load(p.read_text())
    cam = cfg["camera"]
    if s.video and s.stream:
        raise HTTPException(400, "give either a video or a stream URL")
    check_source(s.video, s.stream)
    old_source, old_pos = cam["source"], dict(cam["position"])
    if s.name:
        cfg["site"]["name"] = s.name
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
    if s.device is not None:
        cfg.setdefault("models", {})["backend"] = {"auto": "auto", "gpu": "tensorrt", "cpu": "cpu"}[s.device]
    cfg.setdefault("preview", {"enabled": True, "port": 8081, "fps": 12, "width": 960})
    moved = cam["source"] != old_source or any(abs(cam["position"][k] - old_pos[k]) > 1e-7 for k in ("lat", "lon"))
    if moved:
        (STATE / p.stem / "calibration.json").unlink(missing_ok=True)
        if cam["source"] != old_source and old_source.startswith("file:///data/videos/"):
            (VIDEOS / old_source.rsplit("/", 1)[1]).unlink(missing_ok=True)
    p.write_text(yaml.safe_dump(cfg, sort_keys=False))
    c = container(p.stem)
    if c is not None and c.status == "running":
        start(p.stem)
    return describe(p)


@app.post("/api/sites/{sid}/start")
def start_site(sid: str):
    p = _site(sid)
    start(p.stem)
    return describe(p)


@app.post("/api/sites/{sid}/stop")
def stop_site(sid: str):
    p = _site(sid)
    if (c := container(p.stem)) is not None:
        c.remove(force=True)
    return describe(p)


@app.delete("/api/sites/{sid}")
def delete_site(sid: str):
    p = _site(sid)
    if (c := container(p.stem)) is not None:
        c.remove(force=True)
    cfg = yaml.safe_load(p.read_text())
    src = cfg["camera"]["source"]
    if src.startswith("file:///data/videos/"):
        (VIDEOS / src.rsplit("/", 1)[1]).unlink(missing_ok=True)
    shutil.rmtree(STATE / p.stem, ignore_errors=True)
    p.unlink()
    return {"deleted": p.stem}


@app.get("/api/sites/{sid}/logs")
def logs(sid: str, lines: int = 80):
    p = _site(sid)
    c = container(p.stem)
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
    q = (f"[out:json][timeout:25];(way[\"highway\"](around:{r},{lat},{lon});"
         f"node[\"highway\"~\"traffic_signals|crossing|stop\"](around:{r},{lat},{lon}););out body geom;")
    data = urllib.parse.urlencode({"data": q}).encode()
    last = "no server answered"
    for attempt in range(2):
        for url in OVERPASS:
            try:
                req = urllib.request.Request(url, data=data, headers={"User-Agent": "junction-perception/0.2"})
                with urllib.request.urlopen(req, timeout=30) as resp:
                    j = json.loads(resp.read())
                out = {"ways": [e for e in j.get("elements", []) if e.get("type") == "way" and e.get("geometry")],
                       "nodes": [e for e in j.get("elements", []) if e.get("type") == "node"]}
                if out["ways"]:
                    key.write_text(json.dumps(out))
                    return out
            except Exception as e:  # busy mirror, timeout: try the next one
                last = f"{url}: {e}"
        time.sleep(2)
    raise HTTPException(503, f"OpenStreetMap is not reachable ({last})")


@app.get("/api/health")
def health():
    return {"ok": True, "image": IMAGE, "gpu": gpu_available(), "detector": DETECTOR,
            "sites": len(list(SITES.glob("*.yaml"))), "max_sites": MAX_SITES}


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8000)
