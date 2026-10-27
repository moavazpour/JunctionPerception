"""Site manager: create, start, stop and delete perception instances from the web panel.

One site = one camera source (an uploaded video or an RTSP/HTTP stream) + its placement. Each site runs
as its own `junction-perception` container on the GPU, publishing to the shared MQTT broker under
jp/<site>/cam1/... . Site YAMLs live in <data>/sites, uploaded videos in <data>/videos, runtime state
(calibration, learned map) in <data>/state.

Environment:
  JP_DATA_HOST    host path of the data directory (mounted here at /data)
  JP_MODELS_HOST  host path of the models directory
  JP_NETWORK      docker network of the stack (to reach the broker)
  JP_IMAGE        runtime image (default junction-perception:latest)
  JP_DETECTOR     detector ONNX inside /models (default jp_y26s_960.onnx)
"""
from __future__ import annotations

import os
import re
import shutil
import time
from pathlib import Path

import docker
import yaml
from docker.types import DeviceRequest
from fastapi import FastAPI, File, HTTPException, UploadFile
from pydantic import BaseModel, Field

DATA = Path("/data")
SITES, VIDEOS, STATE = DATA / "sites", DATA / "videos", DATA / "state"
for d in (SITES, VIDEOS, STATE):
    d.mkdir(parents=True, exist_ok=True)
HOST = os.environ.get("JP_DATA_HOST", "/data")
MODELS_HOST = os.environ.get("JP_MODELS_HOST", "/models")
NETWORK = os.environ.get("JP_NETWORK", "bridge")
IMAGE = os.environ.get("JP_IMAGE", "junction-perception:latest")
DETECTOR = os.environ.get("JP_DETECTOR", "jp_y26s_960.onnx")
MAX_SITES = int(os.environ.get("JP_MAX_SITES", "4"))
VIDEO_EXT = {".mp4", ".mov", ".mkv", ".avi", ".m4v"}

app = FastAPI(title="JunctionPerception site manager")
dock = docker.from_env()


class SiteIn(BaseModel):
    name: str = Field(min_length=1, max_length=80)
    video: str | None = None        # file name returned by /api/videos
    stream: str | None = None       # rtsp:// or http(s):// URL
    lat: float = Field(ge=-90, le=90)
    lon: float = Field(ge=-180, le=180)
    height_m: float = Field(default=7.0, gt=0.5, lt=200)
    bearing_deg: float = Field(default=0.0, ge=-360, le=360)


def slug(s: str) -> str:
    s = re.sub(r"[^a-z0-9]+", "-", s.lower()).strip("-")[:40]
    return s or "site"


def container(site: str):
    try:
        return dock.containers.get(f"jp-site-{site}")
    except docker.errors.NotFound:
        return None


def site_yaml(sid: str, s: SiteIn) -> dict:
    source = f"file:///data/videos/{s.video}" if s.video else s.stream
    return {
        "site": {"id": sid, "name": s.name},
        "camera": {"id": "cam1", "source": source, "realtime": True, "loop": bool(s.video), "max_fps": 15,
                   "position": {"lat": s.lat, "lon": s.lon, "height_m": s.height_m, "height_sigma": 1.5,
                                "bearing_deg": s.bearing_deg % 360}},
        "calibration": {"mode": "manual", "file": f"/data/state/{sid}/calibration.json", "publish_snapshot": True},
        "map": {"learn_lanes": True},
        "models": {"detector": f"/models/{DETECTOR}", "segmentation": "/models/segformer_b2_cityscapes.onnx",
                   "fp16": True, "score_threshold": 0.25, "max_range_m": 250},
        "output": {"topic_prefix": "jp", "mqtt": {"uri": "tcp://mosquitto:1883", "client_id": f"jp-{sid}"},
                   "station_id": 1000 + (abs(hash(sid)) % 8999)},
        "health": {"target_fps": 15},
        "state_dir": f"/data/state/{sid}",
    }


def start(sid: str):
    c = container(sid)
    if c is not None:
        c.remove(force=True)
    dock.containers.run(
        IMAGE, ["--config", f"/data/sites/{sid}.yaml"], name=f"jp-site-{sid}", detach=True,
        restart_policy={"Name": "unless-stopped"}, network=NETWORK, labels={"jp.site": sid},
        device_requests=[DeviceRequest(count=-1, capabilities=[["gpu"]])],
        volumes={f"{HOST}/sites": {"bind": "/data/sites", "mode": "ro"}, f"{HOST}/videos": {"bind": "/data/videos", "mode": "ro"},
                 f"{HOST}/state": {"bind": "/data/state", "mode": "rw"}, MODELS_HOST: {"bind": "/models", "mode": "rw"}})


def describe(p: Path) -> dict:
    cfg = yaml.safe_load(p.read_text())
    sid = cfg["site"]["id"]
    c = container(sid)
    cal = STATE / sid / "calibration.json"
    return {"id": sid, "name": cfg["site"]["name"], "source": cfg["camera"]["source"].replace("file:///data/videos/", "video: "),
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
    if s.video and (not (VIDEOS / s.video).is_file() or "/" in s.video):
        raise HTTPException(400, "unknown video")
    if s.stream and not re.match(r"^(rtsp|rtsps|http|https)://", s.stream):
        raise HTTPException(400, "stream URL must start with rtsp:// or http(s)://")
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


@app.get("/api/health")
def health():
    return {"ok": True, "image": IMAGE, "detector": DETECTOR, "sites": len(list(SITES.glob("*.yaml")))}


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8000)
