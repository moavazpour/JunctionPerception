"""Site manager: the always-on service behind the web console.

A site is one intersection: a scene folder scenes/<id>/ with scene.yaml (world frame at the
intersection centre, map area), the zones shared by its cameras, and up to four cameras in
scenes/<id>/cameras/<camera>/ (camera.yaml with name, approach and image size, plus the camera's
calibration). The runtime settings the console edits (name, which cameras run, each camera's
source, playback and video stream) are kept in scenes/<id>/site.yaml. Older single-camera scene
folders (calibration next to scene.yaml, e.g. the benchmark scenes) are sites with one camera.

The manager runs one jp_server per active camera on a local port and proxies it under
/sites/<id>/cameras/<camera>/, restarts servers that exit (a calibration "apply" exits on
purpose), keeps their logs and reports the host.

  GET  /manager/api/health                    liveness and version
  GET  /manager/api/system                    host, GPU, backend state
  POST /manager/api/system/<action>           start-backend | stop-backend | restart-backend |
                                              restart-manager
  GET  /manager/api/sites                     all sites with their cameras and live status
  POST /manager/api/sites                     create a site (scene folder, cameras, site.yaml)
  GET  /manager/api/sites/<id>                one site
  PATCH /manager/api/sites/<id>               name, location, map area or map placement
  DELETE /manager/api/sites/<id>              stop and move the scene folder to the trash
  POST /manager/api/sites/<id>/playback       play | pause | seek all cameras of the site
  POST /manager/api/sites/<id>/<action>       start | stop | restart | prepare-map
  POST /manager/api/sites/<id>/cameras        add a camera
  PATCH /manager/api/sites/<id>/cameras/<c>   change a camera (restarts it if it runs)
  DELETE /manager/api/sites/<id>/cameras/<c>  stop and move the camera folder to the trash
  POST /manager/api/sites/<id>/cameras/<c>/<action>  start | stop | restart
  GET  /manager/api/sites/<id>/state          latest site state (world model, JSON)
  GET  /manager/api/inputs                    videos, sequences and replays to choose from
  GET  /manager/api/probe?input=              image size, frame rate and length of a video or stream
  POST /manager/api/videos                    upload a video (multipart "file") below the data root
  GET  /manager/api/logs?source=&lines=       recent log lines (manager, a site or site/camera)
  GET  /manager/api/logs/stream?source=       the same as server-sent events
  WS   /sites/<id>/ws/site                    site state stream: fused road users of the site
  *    /sites/<id>/map/...                    map layers (also while the site is stopped)
  *    /sites/<id>/cameras/<c>/...            proxied to the camera's jp_server (HTTP, WebSockets)
  *    /sites/<id>/...                        proxied to the site's first camera
  GET  /...                                   the built console (web/dist)

Usage: python3 manager/jp_manager.py [--config configs/manager.yaml]
Environment: JP_DATA_ROOT (dataset root, as for jp_server).
"""

from __future__ import annotations

import argparse
import asyncio
import collections
import datetime
import json
import logging
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import aiohttp
import yaml
from aiohttp import web

from site_feed import FeedCamera, SiteFeed
from world_model import ClassSize, FusionSettings

REPO_ROOT = Path(__file__).resolve().parent.parent
SITE_ID = re.compile(r"^[a-z0-9][a-z0-9_-]{1,47}$")
CAMERA_ID = re.compile(r"^[a-z0-9][a-z0-9_-]{0,31}$")
MAX_CAMERAS = 4
# Arm of the intersection a camera watches.
APPROACHES = (
    "north", "north-east", "east", "south-east", "south", "south-west", "west", "north-west",
    "centre",
)
MODES = ("live", "oracle", "replay")
VIDEO_EXTENSIONS = {".mp4", ".mov", ".mkv", ".avi", ".m4v"}
# Calibration files of a camera (moved along when a single-camera scene becomes an intersection).
CAMERA_FILES = ("ground_homography.yaml", "control_points.yaml")
# spdlog's default pattern: "[2026-11-01 12:00:00.123] [info] message".
SPDLOG_LINE = re.compile(r"^\[[^\]]+\] \[(\w+)\] (.*)$")
STATUS_INTERVAL_S = 1.0
STOP_TIMEOUT_S = 8.0
STABLE_RUN_S = 60.0
INPUT_SCAN_DEPTH = 4
DEFAULT_RADIUS_M = 200.0

log = logging.getLogger("manager")


# ------------------------------------------------------------------------------------- logs


@dataclass
class LogLine:
    time: float
    level: str
    source: str
    message: str

    def to_json(self) -> dict[str, Any]:
        return {"t": self.time, "level": self.level, "source": self.source, "msg": self.message}


class LogHub:
    """Recent lines per source (the manager, each site and each camera "site/camera") and live
    subscribers. A site's source also covers the lines of its cameras."""

    def __init__(self, capacity: int) -> None:
        self.capacity = capacity
        self.lines: dict[str, collections.deque[LogLine]] = {}
        self.subscribers: dict[str, set[asyncio.Queue[LogLine]]] = {}

    def add(self, source: str, level: str, message: str) -> None:
        line = LogLine(time.time(), level, source, message)
        self.lines.setdefault(source, collections.deque(maxlen=self.capacity)).append(line)
        for key in {source, source.split("/")[0], "*"}:
            for queue in self.subscribers.get(key, ()):
                if queue.qsize() < 1000:
                    queue.put_nowait(line)

    def recent(self, source: str, count: int) -> list[LogLine]:
        if source == "*":
            groups = list(self.lines.values())
        elif "/" in source:
            groups = [self.lines.get(source, collections.deque())]
        else:
            groups = [
                lines for key, lines in self.lines.items()
                if key == source or key.startswith(source + "/")
            ]
        if len(groups) == 1:
            return list(groups[0])[-count:]
        merged = [line for lines in groups for line in lines]
        return sorted(merged, key=lambda line: line.time)[-count:]

    def subscribe(self, source: str) -> asyncio.Queue[LogLine]:
        queue: asyncio.Queue[LogLine] = asyncio.Queue()
        self.subscribers.setdefault(source, set()).add(queue)
        return queue

    def unsubscribe(self, source: str, queue: asyncio.Queue[LogLine]) -> None:
        self.subscribers.get(source, set()).discard(queue)


class HubHandler(logging.Handler):
    """Sends the manager's own log records to the hub."""

    def __init__(self, hub: LogHub) -> None:
        super().__init__()
        self.hub = hub

    def emit(self, record: logging.LogRecord) -> None:
        level = {"WARNING": "warning", "ERROR": "error", "CRITICAL": "critical"}.get(
            record.levelname, "info" if record.levelno >= logging.INFO else "debug"
        )
        self.hub.add("manager", level, record.getMessage())


# ----------------------------------------------------------------------------------- config


@dataclass
class ManagerConfig:
    address: str
    port: int
    web_root: Path
    scenes_dir: Path
    map_cache: Path
    server_binary: Path
    server_config: Path
    first_port: int
    last_port: int
    max_running: int
    log_lines: int
    backoff_s: tuple[float, float]
    state_dir: Path
    upload_dir: Path
    upload_max_bytes: int
    probe_timeout_s: float
    data_root: Path
    world_model: dict[str, Any]

    @staticmethod
    def load(path: Path) -> ManagerConfig:
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
        resolve = lambda p: (REPO_ROOT / p).resolve()  # noqa: E731
        ports = raw["site_ports"]
        backoff = raw["restart_backoff_s"]
        data_root = Path(os.environ.get("JP_DATA_ROOT", raw.get("data_root", "/data")))
        return ManagerConfig(
            address=raw["listen"]["address"],
            port=int(raw["listen"]["port"]),
            web_root=resolve(raw["web_root"]),
            scenes_dir=resolve(raw["scenes_dir"]),
            map_cache=resolve(raw["map_cache"]),
            server_binary=resolve(raw["server_binary"]),
            server_config=resolve(raw["server_config"]),
            first_port=int(ports[0]),
            last_port=int(ports[1]),
            max_running=int(raw["max_running"]),
            log_lines=int(raw["log_lines"]),
            backoff_s=(float(backoff[0]), float(backoff[1])),
            state_dir=resolve(raw["state_dir"]),
            data_root=data_root,
            upload_dir=data_root / raw.get("upload_dir", "uploads"),
            upload_max_bytes=int(float(raw.get("upload_max_gb", 20)) * 1024**3),
            probe_timeout_s=float(raw.get("probe_timeout_s", 20)),
            world_model=dict(raw.get("world_model") or {}),
        )


def read_yaml(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    return yaml.safe_load(path.read_text(encoding="utf-8")) or {}



def load_class_sizes(path: Path) -> dict[str, ClassSize]:
    """Footprint of each twin class (configs/classes.yaml), for road users without a measured one."""
    classes = read_yaml(path).get("classes", {})
    return {name: ClassSize(float(c["length_m"]), float(c["width_m"])) for name, c in classes.items()}

def write_yaml(path: Path, data: dict[str, Any], header: str = "") -> None:
    text = yaml.safe_dump(data, sort_keys=False, allow_unicode=True)
    path.write_text(header + text, encoding="utf-8")


def git_tracked(path: Path) -> bool:
    """Whether a file is part of the repository (then the manager leaves its layout alone)."""
    try:
        result = subprocess.run(
            ["git", "ls-files", "--error-unmatch", str(path)],
            cwd=REPO_ROOT,
            capture_output=True,
            timeout=5,
        )
    except (OSError, subprocess.SubprocessError):
        return True
    # 1: not tracked; anything else (also "not a repository") counts as tracked, to be safe.
    return result.returncode != 1


class BadRequest(Exception):
    """Invalid request content; becomes a 400 response with the message."""


# --------------------------------------------------------------------------------- settings


def merge(base: dict[str, Any], patch: dict[str, Any]) -> dict[str, Any]:
    out = dict(base)
    for key, value in patch.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = merge(out[key], value)
        else:
            out[key] = value
    return out


def default_camera_settings(
    folder: Path, definition: dict[str, Any], base: dict[str, Any]
) -> dict[str, Any]:
    """Runtime settings of a camera without stored ones: the startup scene of server.yaml runs
    its configured source, other cameras their recorded video."""
    startup = base.get("startup", {})
    is_startup = Path(str(startup.get("scene", ""))).name == folder.name
    source = definition.get("source") or {}
    if is_startup:
        mode, source_input = startup.get("mode", "live"), startup.get("input", "")
    else:
        mode, source_input = "live", source.get("video", "")
    return {
        "enabled": True,
        "source": {"mode": mode, "input": source_input},
        "playback": dict(base.get("playback", {"loop": True, "speed": 1.0})),
        "video": dict(base.get("video", {"max_fps": 5, "width_px": 960, "jpeg_quality": 70})),
    }


def validate_camera_settings(settings: dict[str, Any], require_source: bool = True) -> None:
    source = settings["source"]
    if source.get("mode") not in MODES:
        raise BadRequest("source.mode must be live, oracle or replay")
    if not isinstance(source.get("input"), str):
        raise BadRequest("source.input must be a string")
    if require_source and not source["input"].strip():
        raise BadRequest("the camera needs a source (video, stream URL or recording)")
    if "://" in source["input"] and not re.match(r"^(rtsp|rtsps|http|https)://", source["input"]):
        raise BadRequest("stream URLs must start with rtsp://, rtsps://, http:// or https://")
    playback, video = settings["playback"], settings["video"]
    if not 0.05 <= float(playback.get("speed", 1)) <= 8:
        raise BadRequest("playback.speed must be between 0.05 and 8")
    if not 0.5 <= float(video.get("max_fps", 5)) <= 30:
        raise BadRequest("video.max_fps must be between 0.5 and 30")
    if not 320 <= int(video.get("width_px", 960)) <= 1920:
        raise BadRequest("video.width_px must be between 320 and 1920")
    if not 30 <= int(video.get("jpeg_quality", 70)) <= 95:
        raise BadRequest("video.jpeg_quality must be between 30 and 95")
    if not -60_000 <= float(settings.get("time_offset_ms", 0)) <= 60_000:
        raise BadRequest("time_offset_ms must be within ±60000")


def image_size(value: Any) -> list[int]:
    if not (isinstance(value, list) and len(value) == 2 and all(int(v) >= 64 for v in value)):
        raise BadRequest("image_size_px must be [width, height] in pixels")
    return [int(value[0]), int(value[1])]


def geo_anchor(value: Any) -> dict[str, float]:
    if not isinstance(value, dict):
        raise BadRequest("the location needs lat_deg and lon_deg")
    lat, lon = float(value["lat_deg"]), float(value["lon_deg"])
    if not (-90 <= lat <= 90 and -180 <= lon <= 180):
        raise BadRequest("latitude must be within ±90°, longitude within ±180°")
    return {"lat_deg": lat, "lon_deg": lon, "alt_m": float(value.get("alt_m") or 0)}


def approach(value: Any) -> str:
    if value not in APPROACHES:
        raise BadRequest("approach must be one of " + ", ".join(APPROACHES))
    return str(value)


# ---------------------------------------------------------------------------------- cameras


@dataclass
class Runtime:
    """Process state of one camera server; `state` is what the console shows."""

    state: str = "stopped"  # stopped | starting | running | stopping | backoff | failed
    process: asyncio.subprocess.Process | None = None
    port: int | None = None
    started_at: float | None = None
    restarts: int = 0
    last_exit: int | None = None
    error: str | None = None
    status: dict[str, Any] | None = None
    backoff_s: float = 0.0
    restart_now: bool = False
    stopping: bool = False
    supervisor: asyncio.Task[None] | None = None


class Camera:
    """One camera of a site and its jp_server."""

    def __init__(self, site: Site, camera_id: str, folder: Path, single: bool) -> None:
        self.site = site
        self.manager = site.manager
        self.id = camera_id
        self.dir = folder
        # Single-camera scene: the camera's definition is the scene itself.
        self.single = single
        self.runtime = Runtime()
        self.definition: dict[str, Any] = {}
        self.settings: dict[str, Any] = {}

    @property
    def key(self) -> str:
        """Log source and display key: "site/camera"."""
        return f"{self.site.id}/{self.id}"

    def load(self, stored: dict[str, Any] | None) -> None:
        self.definition = self.site.scene if self.single else read_yaml(self.dir / "camera.yaml")
        defaults = default_camera_settings(self.dir, self.definition, self.manager.base_config)
        self.settings = merge(defaults, stored or {})

    @property
    def name(self) -> str:
        return str(self.definition.get("name") or self.id.replace("_", " ").capitalize())

    @property
    def enabled(self) -> bool:
        return bool(self.settings.get("enabled", True))

    def save_definition(self, changes: dict[str, Any]) -> None:
        if self.single:
            scene = merge(self.site.scene, changes)
            write_yaml(self.site.dir / "scene.yaml", scene)
            self.site.scene = scene
        else:
            self.definition = merge(self.definition, changes)
            write_yaml(
                self.dir / "camera.yaml",
                self.definition,
                f"# Camera of the site {self.site.id}; source and runtime settings: ../../site.yaml.\n",
            )

    def regions(self) -> dict[str, Any]:
        """Detection areas, counting lines and traffic lights of the camera (regions.json),
        summarized."""
        try:
            regions = json.loads((self.dir / "regions.json").read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return {"areas": 0, "lines": [], "signals": []}
        return {
            "areas": len(regions.get("areas", [])),
            "lines": [
                {"id": line["id"], "name": line.get("name") or line["id"]}
                for line in regions.get("lines", [])
            ],
            "signals": [
                {
                    "id": signal["id"],
                    "name": signal.get("name") or signal["id"],
                    "placed": signal.get("position") is not None,
                }
                for signal in regions.get("signals", [])
            ],
        }

    def describe(self) -> dict[str, Any]:
        r = self.runtime
        return {
            "id": self.id,
            "name": self.name,
            "approach": self.definition.get("approach", "centre"),
            "image_size_px": self.definition.get("image_size_px"),
            "enabled": self.enabled,
            "calibrated": (self.dir / "ground_homography.yaml").is_file(),
            "control_points": (self.dir / "control_points.yaml").is_file(),
            "regions": self.regions(),
            "source": self.settings["source"],
            "playback": self.settings["playback"],
            "video": self.settings["video"],
            "time_offset_ms": float(self.settings.get("time_offset_ms", 0)),
            "runtime": {
                "state": r.state,
                "pid": r.process.pid if r.process and r.process.returncode is None else None,
                "port": r.port,
                "started_at": r.started_at,
                "uptime_s": time.time() - r.started_at if r.started_at else None,
                "restarts": r.restarts,
                "last_exit": r.last_exit,
                "error": r.error,
                "status": r.status,
            },
        }

    def server_yaml(self) -> Path:
        """Server config of this camera: the shared base with its port, playback and video."""
        config = json.loads(json.dumps(self.manager.base_config))
        config["server"]["bind_address"] = "127.0.0.1"
        config["server"]["port"] = self.runtime.port
        config["video"] = merge(config.get("video", {}), self.settings["video"])
        config["playback"] = merge(config.get("playback", {}), self.settings["playback"])
        if self.site.synced():
            # The recordings of the site play by one clock (seek, pause and loop together).
            config["playback"]["clock"] = str(self.site.clock_path.resolve())
            config["playback"]["speed"] = self.site.clock_speed()
        source = self.settings["source"]
        config["startup"] = {
            "scene": str(self.dir),
            "mode": source["mode"],
            "input": source["input"],
        }
        path = self.manager.config.state_dir / "sites" / self.site.id / self.id / "server.yaml"
        path.parent.mkdir(parents=True, exist_ok=True)
        write_yaml(path, config, f"# Generated by the manager for camera {self.key}.\n")
        return path

    # -------------------------------------------------------------------- process control

    def log(self, level: str, message: str) -> None:
        self.manager.logs.add(self.key, level, message)

    def start(self) -> None:
        """Runs the camera's server and keeps it running until stop()."""
        r = self.runtime
        if r.supervisor and not r.supervisor.done():
            return
        if not str(self.settings["source"].get("input", "")).strip():
            r.state, r.error = "failed", "no source configured: set one in the camera settings"
            self.log("error", r.error)
            return
        running = sum(1 for c in self.manager.cameras() if c.is_active())
        if running >= self.manager.config.max_running:
            r.state, r.error = "failed", (
                f"capacity reached: at most {self.manager.config.max_running} cameras run at once"
            )
            self.log("error", r.error)
            return
        r.error = None
        r.backoff_s = 0.0
        if self.site.synced() and self.site.clock is None:
            self.site.reset_clock()
        r.supervisor = asyncio.create_task(self.supervise())

    async def stop(self) -> None:
        r = self.runtime
        r.stopping = True
        await self.terminate()
        if r.supervisor:
            r.supervisor.cancel()
            try:
                await r.supervisor
            except asyncio.CancelledError:
                pass
            r.supervisor = None
        r.stopping = False
        r.state, r.status, r.started_at, r.port = "stopped", None, None, None

    async def restart(self) -> None:
        r = self.runtime
        if r.supervisor and not r.supervisor.done() and r.process:
            r.restart_now = True
            await self.terminate()
            return
        # Waiting to retry after a crash, or not running: start afresh.
        await self.stop()
        self.start()

    def is_active(self) -> bool:
        return bool(self.runtime.supervisor and not self.runtime.supervisor.done())

    async def terminate(self) -> None:
        process = self.runtime.process
        if not process or process.returncode is not None:
            return
        self.runtime.state = "stopping"
        process.send_signal(signal.SIGTERM)
        try:
            await asyncio.wait_for(process.wait(), STOP_TIMEOUT_S)
        except asyncio.TimeoutError:
            self.log("warning", "server did not stop in time; killed")
            process.kill()
            await process.wait()

    async def supervise(self) -> None:
        r = self.runtime
        cfg = self.manager.config
        try:
            while True:
                r.port = self.manager.free_port(self)
                command = [
                    str(cfg.server_binary),
                    "--config",
                    str(self.server_yaml()),
                    "--data-root",
                    str(cfg.data_root),
                ]
                self.log("info", f"starting server on port {r.port}")
                r.state, r.status, r.restart_now = "starting", None, False
                r.started_at = time.time()
                try:
                    r.process = await asyncio.create_subprocess_exec(
                        *command,
                        cwd=REPO_ROOT,
                        stdout=asyncio.subprocess.PIPE,
                        stderr=asyncio.subprocess.STDOUT,
                        start_new_session=True,
                    )
                except OSError as error:
                    r.state, r.error = "failed", f"cannot start {cfg.server_binary}: {error}"
                    self.log("error", r.error)
                    return
                assert r.process.stdout is not None
                await self.pump(r.process.stdout)
                code = await r.process.wait()
                ran = time.time() - (r.started_at or time.time())
                r.last_exit, r.status, r.started_at = code, None, None
                if r.stopping:
                    self.log("info", "server stopped")
                    return
                if r.restart_now or code == 0:
                    # Restart from the console, or the server's own /api/restart after a
                    # calibration was applied.
                    self.log("info", f"server exited ({code}); restarting")
                    r.restarts += 1
                    continue
                r.backoff_s = (
                    cfg.backoff_s[0]
                    if ran > STABLE_RUN_S or r.backoff_s == 0
                    else min(r.backoff_s * 2, cfg.backoff_s[1])
                )
                r.state, r.error = "backoff", f"server exited with code {code}"
                self.log("error", f"server exited with code {code}; retry in {r.backoff_s:.0f} s")
                await asyncio.sleep(r.backoff_s)
                r.restarts += 1
        finally:
            r.process = None

    async def pump(self, stream: asyncio.StreamReader) -> None:
        while True:
            raw = await stream.readline()
            if not raw:
                return
            text = raw.decode("utf-8", "replace").rstrip()
            if not text:
                continue
            match = SPDLOG_LINE.match(text)
            level, message = (match.group(1), match.group(2)) if match else ("info", text)
            self.log("warning" if level == "warn" else level, message)

    async def send_playback(self, command: dict[str, Any]) -> None:
        """Play, pause or seek of a camera that plays by itself (no site clock)."""
        r = self.runtime
        if r.state != "running" or r.port is None or self.manager.session is None:
            return
        try:
            timeout = aiohttp.ClientTimeout(total=2)
            async with self.manager.session.post(
                f"http://127.0.0.1:{r.port}/api/playback", json=command, timeout=timeout
            ):
                pass
        except (aiohttp.ClientError, asyncio.TimeoutError):
            self.log("warning", f"playback command {command.get('action')} not delivered")

    async def poll_status(self, session: aiohttp.ClientSession) -> None:
        r = self.runtime
        if r.state not in ("starting", "running") or r.port is None:
            return
        try:
            timeout = aiohttp.ClientTimeout(total=0.8)
            async with session.get(f"http://127.0.0.1:{r.port}/api/status", timeout=timeout) as resp:
                r.status = await resp.json()
                if r.state == "starting":
                    self.log("info", "server is up")
                r.state = "running"
        except (aiohttp.ClientError, asyncio.TimeoutError, ValueError):
            r.status = None


# ------------------------------------------------------------------------------------ sites


class Site:
    def __init__(self, manager: Manager, scene_dir: Path) -> None:
        self.manager = manager
        self.dir = scene_dir
        self.id = scene_dir.name
        self.cameras: dict[str, Camera] = {}
        self.map_job: dict[str, Any] | None = None
        self.tasks: set[asyncio.Task[Any]] = set()
        # Playback clock of the cameras (see synced()); written to clock_path for their servers.
        self.clock: dict[str, Any] | None = None
        self.reload()

    @property
    def single(self) -> bool:
        """An older single-camera scene: the calibration lies next to scene.yaml."""
        return not (self.dir / "cameras").is_dir()

    def reload(self) -> None:
        self.scene = read_yaml(self.dir / "scene.yaml")
        stored = read_yaml(self.dir / "site.yaml")
        folders = (
            [(self.id, self.dir)]
            if self.single
            else sorted(
                (p.name, p)
                for p in (self.dir / "cameras").iterdir()
                if (p / "camera.yaml").is_file() and CAMERA_ID.match(p.name)
            )
        )
        stored_cameras = stored.get("cameras") or {}
        if not stored_cameras and "source" in stored and folders:
            # site.yaml of a single-camera site: its settings belong to the first camera.
            stored_cameras = {
                folders[0][0]: {k: stored[k] for k in ("source", "playback", "video") if k in stored}
            }
        cameras: dict[str, Camera] = {}
        for camera_id, folder in folders:
            camera = self.cameras.get(camera_id) or Camera(self, camera_id, folder, self.single)
            camera.dir = folder
            camera.load(stored_cameras.get(camera_id))
            cameras[camera_id] = camera
        self.cameras = cameras
        startup = any(
            Path(str(self.manager.base_config.get("startup", {}).get("scene", ""))).name == c.dir.name
            for c in cameras.values()
        )
        self.settings = {
            "name": stored.get("name") or self.id.replace("_", " "),
            "description": stored.get("description", ""),
            "enabled": bool(stored.get("enabled", startup)),
        }

    def save_settings(self) -> None:
        data = dict(self.settings)
        data["cameras"] = {c.id: c.settings for c in self.cameras.values()}
        write_yaml(
            self.dir / "site.yaml",
            data,
            "# Site settings edited in the console (manager/jp_manager.py).\n",
        )

    def convert_to_intersection(self) -> None:
        """Turns a single-camera scene into an intersection whose first camera keeps the
        calibration, image size and settings."""
        stored = {c.id: c.settings for c in self.cameras.values()}
        folder = self.dir / "cameras" / "cam1"
        folder.mkdir(parents=True)
        for name in CAMERA_FILES:
            if (self.dir / name).is_file():
                shutil.move(str(self.dir / name), str(folder / name))
        scene = dict(self.scene)
        camera = {
            "name": "Camera 1",
            "approach": "centre",
            "image_size_px": scene.pop("image_size_px", [1920, 1080]),
        }
        if scene.get("source"):
            camera["source"] = scene.pop("source")
        scene.pop("calibration", None)
        write_yaml(folder / "camera.yaml", camera)
        write_yaml(self.dir / "scene.yaml", scene)
        old = next(iter(stored.values()), None)
        self.cameras = {}
        self.reload()
        if old is not None and "cam1" in self.cameras:
            self.cameras["cam1"].settings = old
        self.save_settings()
        log.info("site %s: now an intersection with cameras in %s", self.id, self.dir / "cameras")

    @property
    def georeferenced(self) -> bool:
        """World frame is local east-north-up at the geo anchor (custom sites)."""
        return bool(self.scene.get("geo_anchor"))

    @property
    def location(self) -> dict[str, Any] | None:
        """Geographic position of the site: its geo anchor (the intersection centre) or, for
        scenes in a sensor frame, the anchor of the map layers (aligned with map.transform)."""
        return self.scene.get("geo_anchor") or (self.scene.get("map") or {}).get("anchor")

    @property
    def enabled(self) -> bool:
        return bool(self.settings.get("enabled"))

    def state(self) -> str:
        """Summary of the cameras: stopped | starting | running | partial | stopping | backoff |
        failed."""
        states = [c.runtime.state for c in self.cameras.values() if c.runtime.state != "stopped"]
        if not states:
            return "stopped"
        if all(s == "running" for s in states):
            return "running"
        if "running" in states:
            return "partial"
        for state in ("starting", "stopping", "backoff"):
            if state in states:
                return state
        return "failed"

    def describe(self) -> dict[str, Any]:
        errors = [f"{c.name}: {c.runtime.error}" for c in self.cameras.values() if c.runtime.error]
        return {
            "id": self.id,
            "name": self.settings.get("name") or self.id,
            "description": self.settings.get("description", ""),
            "enabled": self.enabled,
            "kind": self.scene.get("kind", "custom"),
            "layout": "single" if self.single else "intersection",
            "geo_anchor": self.location,
            "georeferenced": self.georeferenced,
            "map_radius_m": (self.scene.get("map") or {}).get("radius_m"),
            "map_transform": (self.scene.get("map") or {}).get("transform"),
            "has_map": (self.manager.config.map_cache / self.id / "map.json").is_file(),
            "zones": (self.dir / "zones.geojson").is_file(),
            "max_cameras": 1 if self.single and self.scene.get("kind") == "benchmark" else MAX_CAMERAS,
            "cameras": [c.describe() for c in self.cameras.values()],
            "runtime": {
                "state": self.state(),
                "error": "; ".join(errors) or None,
                "map_job": self.map_job,
                "world": self.manager.world_summary(self.id),
            },
        }

    def camera(self, camera_id: str) -> Camera:
        camera = self.cameras.get(camera_id)
        if not camera:
            raise web.HTTPNotFound(
                text=json.dumps({"error": "no such camera"}), content_type="application/json"
            )
        return camera

    def start(self) -> None:
        if self.synced() and not self.is_active():
            self.reset_clock()
        for camera in self.cameras.values():
            if camera.enabled:
                camera.start()

    async def stop(self) -> None:
        await asyncio.gather(*(c.stop() for c in self.cameras.values()))

    async def restart(self) -> None:
        await asyncio.gather(*(c.restart() for c in self.cameras.values() if c.enabled))
        if self.synced():
            self.reset_clock()

    # ------------------------------------------------------------------- playback clock

    @property
    def clock_path(self) -> Path:
        return self.manager.config.state_dir / "sites" / self.id / "clock.json"

    def synced(self) -> bool:
        """Whether the cameras play their recordings by one clock: two or more enabled cameras,
        all with recordings (streams play live and cannot seek). A camera that restarts or falls
        behind then rejoins the others at the clock's position."""
        cameras = [c for c in self.cameras.values() if c.enabled]
        return len(cameras) > 1 and all(
            "://" not in str(c.settings["source"].get("input", "")) for c in cameras
        )

    def clock_speed(self) -> float:
        speeds = [float(c.settings["playback"].get("speed", 1.0)) for c in self.cameras.values()]
        return speeds[0] if speeds else 1.0

    def reset_clock(self) -> None:
        """Playing from the start of the recordings, now."""
        period = self.clock["period_ns"] if self.clock else 0
        self.clock = {
            "epoch_ns": time.time_ns(),
            "media_ns": 0,
            "speed": self.clock_speed(),
            "playing": True,
            "period_ns": period,
        }
        self.write_clock()

    def write_clock(self) -> None:
        assert self.clock is not None
        path = self.clock_path
        path.parent.mkdir(parents=True, exist_ok=True)
        # Replaced atomically: the camera servers never read half a file.
        temporary = path.with_suffix(".tmp")
        temporary.write_text(json.dumps(self.clock), encoding="utf-8")
        os.replace(temporary, path)

    def clock_position(self, now_ns: int) -> int:
        """Recording time the clock is at (same rule as the servers' SiteClock::position)."""
        assert self.clock is not None
        c = self.clock
        at = c["media_ns"] + (round((now_ns - c["epoch_ns"]) * c["speed"]) if c["playing"] else 0)
        return at % c["period_ns"] if c["period_ns"] > 0 else at

    def update_period(self) -> None:
        """Loop length: the longest recording of the running cameras (they report it)."""
        if self.clock is None or not self.synced():
            return
        durations = [
            int(c.runtime.status["duration_ns"])
            for c in self.cameras.values()
            if c.runtime.status and c.runtime.status.get("duration_ns")
        ]
        if durations and max(durations) != self.clock["period_ns"]:
            self.clock["period_ns"] = max(durations)
            self.write_clock()

    async def playback(self, command: dict[str, Any]) -> dict[str, Any]:
        """Play, pause or seek all cameras of the site together."""
        action = command.get("action")
        if action not in ("play", "pause", "seek"):
            raise BadRequest('expected {"action": "play" | "pause" | "seek"}')
        frame = command.get("frame")
        if action == "seek" and (not isinstance(frame, int) or frame < 0):
            raise BadRequest("seek needs a frame (0 or more)")
        if not self.synced():
            # Each camera plays by itself: the command goes to every running one.
            await asyncio.gather(*(c.send_playback(command) for c in self.cameras.values()))
            return {"synced": False}
        if self.clock is None:
            self.reset_clock()
        assert self.clock is not None
        now = time.time_ns()
        position = self.clock_position(now)
        if action == "seek":
            # The frame of the camera the console shows, at its frame rate.
            shown = self.cameras.get(str(command.get("camera", "")))
            statuses = [shown.runtime.status] if shown and shown.runtime.status else []
            statuses += [c.runtime.status for c in self.cameras.values() if c.runtime.status]
            fps = float(next((s["video_fps"] for s in statuses if s.get("video_fps")), 10.0))
            position = round(int(frame) / fps * 1e9)
        else:
            self.clock["playing"] = action == "play"
        self.clock["epoch_ns"] = now
        self.clock["media_ns"] = position
        self.write_clock()
        return {"synced": True, "clock": self.clock}

    def is_active(self) -> bool:
        return any(c.is_active() for c in self.cameras.values())

    async def add_camera(self, body: dict[str, Any]) -> Camera:
        if self.scene.get("kind") == "benchmark":
            raise BadRequest("benchmark scenes have one camera")
        if len(self.cameras) >= MAX_CAMERAS:
            raise BadRequest(f"an intersection has at most {MAX_CAMERAS} cameras")
        if self.single:
            # The first camera's files move to cameras/cam1/: its server restarts from there.
            was_active = self.is_active()
            await self.stop()
            self.convert_to_intersection()
            if was_active:
                self.start()
        camera_id = str(body.get("id") or "").strip().lower() or next(
            f"cam{n}" for n in range(1, 100) if f"cam{n}" not in self.cameras
            and not (self.dir / "cameras" / f"cam{n}").exists()
        )
        if not CAMERA_ID.match(camera_id):
            raise BadRequest("camera id: 1–32 characters, lowercase letters, digits, - and _")
        folder = self.dir / "cameras" / camera_id
        if camera_id in self.cameras or folder.exists():
            raise BadRequest(f"the site has a camera {camera_id} already")
        settings = merge(
            default_camera_settings(folder, {}, self.manager.base_config),
            {k: body[k] for k in ("enabled", "source", "playback", "video") if k in body},
        )
        validate_camera_settings(settings, require_source=False)
        definition = {
            "name": str(body.get("name") or f"Camera {len(self.cameras) + 1}"),
            "approach": approach(body.get("approach", "centre")),
            "image_size_px": image_size(body.get("image_size_px") or [1920, 1080]),
        }
        source_input = settings["source"]["input"]
        if source_input and "://" not in source_input and settings["source"]["mode"] == "live":
            definition["source"] = {"video": source_input}
        folder.mkdir(parents=True)
        created = datetime.date.today().isoformat()
        write_yaml(
            folder / "camera.yaml",
            definition,
            f"# Camera of the site {self.id}, created in the console on {created}.\n",
        )
        camera = Camera(self, camera_id, folder, single=False)
        camera.load(settings)
        self.cameras[camera_id] = camera
        self.cameras = dict(sorted(self.cameras.items()))
        self.save_settings()
        log.info("site %s: added camera %s", self.id, camera_id)
        return camera

    async def remove_camera(self, camera: Camera) -> None:
        if self.single:
            raise BadRequest("this site has a single camera; delete the site instead")
        await camera.stop()
        trash = self.manager.config.state_dir / "trash"
        trash.mkdir(parents=True, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        shutil.move(str(camera.dir), str(trash / f"{self.id}-{camera.id}-{stamp}"))
        del self.cameras[camera.id]
        self.save_settings()
        log.info("site %s: removed camera %s (moved to %s)", self.id, camera.id, trash)

    async def prepare_map(self) -> None:
        if self.map_job and self.map_job["state"] == "running":
            raise BadRequest("map data is already being prepared")
        if not self.location:
            raise BadRequest("the site needs a geographic location (latitude, longitude) first")
        self.map_job = {"state": "running", "started_at": time.time(), "message": "downloading"}
        task = asyncio.create_task(self._map_job())
        self.tasks.add(task)
        task.add_done_callback(self.tasks.discard)

    async def _map_job(self) -> None:
        job = self.map_job
        assert job is not None
        logs = self.manager.logs
        radius = (self.scene.get("map") or {}).get("radius_m", DEFAULT_RADIUS_M)
        logs.add(self.id, "info", f"preparing map layers within {radius:.0f} m (imagery, OSM, buildings)")
        process = await asyncio.create_subprocess_exec(
            sys.executable,
            str(REPO_ROOT / "tools" / "prepare_map_data.py"),
            "--scene",
            str(self.dir),
            "--out",
            str(self.manager.config.map_cache),
            cwd=REPO_ROOT,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
        )
        assert process.stdout is not None
        last = ""
        async for raw in process.stdout:
            last = raw.decode("utf-8", "replace").rstrip()
            if last:
                logs.add(self.id, "info", f"map: {last}")
        code = await process.wait()
        ok_ = code == 0
        job.update(
            state="done" if ok_ else "failed",
            finished_at=time.time(),
            message="map layers ready" if ok_ else (last or f"exit code {code}"),
        )
        logs.add(self.id, "info" if ok_ else "error", f"map: {job['message']}")


# ---------------------------------------------------------------------------------- manager


class Manager:
    def __init__(self, config: ManagerConfig) -> None:
        self.config = config
        self.logs = LogHub(config.log_lines)
        self.base_config = read_yaml(config.server_config)
        self.sites: dict[str, Site] = {}
        self.started_at = time.time()
        self.version = self._version()
        self.state_file = config.state_dir / "state.json"
        self.backend_on = True
        self.session: aiohttp.ClientSession | None = None
        self.fusion = FusionSettings.from_dict(config.world_model)
        self.class_sizes = load_class_sizes(REPO_ROOT / "configs" / "classes.yaml")
        self.feeds: dict[str, SiteFeed] = {}

    @staticmethod
    def _version() -> str:
        try:
            out = subprocess.run(
                ["git", "describe", "--always", "--dirty", "--tags"],
                cwd=REPO_ROOT,
                capture_output=True,
                text=True,
                timeout=5,
                check=True,
            )
            return out.stdout.strip()
        except (OSError, subprocess.SubprocessError):
            return "unknown"

    def load_sites(self) -> None:
        for scene_dir in sorted(self.config.scenes_dir.iterdir()):
            if not ((scene_dir / "scene.yaml").is_file() and SITE_ID.match(scene_dir.name)):
                continue
            try:
                site = Site(self, scene_dir)
                if (
                    site.single
                    and site.scene.get("kind", "custom") == "custom"
                    and not git_tracked(scene_dir / "scene.yaml")
                ):
                    # Sites made in the console before intersections existed.
                    site.convert_to_intersection()
                self.sites[scene_dir.name] = site
            except (yaml.YAMLError, OSError, KeyError) as error:
                log.error("cannot load site %s: %s", scene_dir.name, error)
        log.info(
            "%d sites with %d cameras in %s",
            len(self.sites),
            sum(len(s.cameras) for s in self.sites.values()),
            self.config.scenes_dir,
        )

    def cameras(self) -> list[Camera]:
        return [c for s in self.sites.values() for c in s.cameras.values()]

    def load_state(self) -> None:
        if self.state_file.is_file():
            self.backend_on = bool(json.loads(self.state_file.read_text()).get("backend_on", True))

    def save_state(self) -> None:
        self.state_file.parent.mkdir(parents=True, exist_ok=True)
        self.state_file.write_text(json.dumps({"backend_on": self.backend_on}))

    def free_port(self, camera: Camera) -> int:
        used = {c.runtime.port for c in self.cameras() if c is not camera and c.is_active()}
        for port in range(self.config.first_port, self.config.last_port + 1):
            if port not in used:
                return port
        raise RuntimeError("no free camera port")

    def site(self, site_id: str) -> Site:
        site = self.sites.get(site_id)
        if not site:
            raise web.HTTPNotFound(text=json.dumps({"error": "no such site"}),
                                   content_type="application/json")
        return site

    async def start_backend(self) -> None:
        self.backend_on = True
        self.save_state()
        for site in self.sites.values():
            if site.enabled:
                site.start()

    async def stop_backend(self) -> None:
        self.backend_on = False
        self.save_state()
        await asyncio.gather(*(site.stop() for site in self.sites.values()))

    async def restart_backend(self) -> None:
        await asyncio.gather(*(site.stop() for site in self.sites.values()))
        await self.start_backend()

    async def status_loop(self) -> None:
        assert self.session is not None
        while True:
            await asyncio.gather(*(c.poll_status(self.session) for c in self.cameras()))
            for site in self.sites.values():
                site.update_period()
            self.follow_sites()
            await asyncio.sleep(STATUS_INTERVAL_S)

    def world_summary(self, site_id: str) -> dict[str, Any] | None:
        """Fusion mode and road users of a site's latest state (for the console's lists)."""
        feed = self.feeds.get(site_id)
        state = feed.model_state() if feed else None
        if state is None:
            return None
        return {"mode": state["fusion"]["mode"], "road_users": len(state["road_users"]),
                "t_ns": state["t_ns"]}

    def feed(self, site_id: str) -> SiteFeed:
        """World model of a site (created on first use, kept while the manager runs)."""
        feed = self.feeds.get(site_id)
        if feed is None:
            feed = SiteFeed(
                site_id, self.fusion, self.class_sizes,
                lambda level, message, s=site_id: self.logs.add(s, level, message),
            )
            self.feeds[site_id] = feed
        return feed

    def follow_sites(self) -> None:
        """Feeds the running cameras of every site into its world model."""
        assert self.session is not None
        for site_id in [s for s in self.feeds if s not in self.sites]:
            asyncio.ensure_future(self.feeds.pop(site_id).close())
        for site in self.sites.values():
            cameras = {
                c.id: FeedCamera(c.runtime.port, round(float(c.settings.get("time_offset_ms", 0)) * 1e6))
                for c in site.cameras.values()
                if c.runtime.state == "running" and c.runtime.port is not None
            }
            if cameras or site.id in self.feeds:
                self.feed(site.id).configure(self.session, cameras, site.synced())

    def system(self) -> dict[str, Any]:
        sites = list(self.sites.values())
        cameras = self.cameras()
        return {
            "version": self.version,
            "started_at": self.started_at,
            "uptime_s": time.time() - self.started_at,
            "backend_on": self.backend_on,
            "sites_total": len(sites),
            "sites_enabled": sum(s.enabled for s in sites),
            "sites_running": sum(s.state() in ("running", "partial") for s in sites),
            "cameras_total": len(cameras),
            "cameras_running": sum(c.runtime.state == "running" for c in cameras),
            "max_running": self.config.max_running,
            "data_root": str(self.config.data_root),
            "server_binary": str(self.config.server_binary),
            "server_binary_found": self.config.server_binary.is_file(),
            "host": host_info(self.config.data_root),
            "gpus": gpu_info(),
        }

    def inputs(self) -> dict[str, list[dict[str, Any]]]:
        root = self.config.data_root
        videos: list[dict[str, Any]] = []
        sequences: list[dict[str, Any]] = []
        if root.is_dir():
            for path, depth in walk(root, INPUT_SCAN_DEPTH):
                rel = path.relative_to(root).as_posix()
                if path.is_file() and path.suffix.lower() in VIDEO_EXTENSIONS:
                    videos.append({"path": rel, "size": path.stat().st_size})
                elif path.is_dir() and depth <= 2 and (path / "_labels").is_dir():
                    sequences.append({"path": rel, "size": None})
        replays = [
            {"path": p.relative_to(REPO_ROOT).as_posix(), "size": p.stat().st_size}
            for p in sorted((REPO_ROOT / "outputs").rglob("*.jsonl"))
        ] if (REPO_ROOT / "outputs").is_dir() else []
        return {"videos": videos, "sequences": sequences, "replays": replays}


def walk(root: Path, max_depth: int, depth: int = 0):
    try:
        entries = sorted(root.iterdir())
    except OSError:
        return
    for entry in entries:
        if entry.name.startswith("."):
            continue
        yield entry, depth + 1
        if entry.is_dir() and depth + 1 < max_depth:
            yield from walk(entry, max_depth, depth + 1)


def host_info(data_root: Path) -> dict[str, Any]:
    info: dict[str, Any] = {"cpu_count": os.cpu_count()}
    try:
        info["load"] = list(os.getloadavg())
    except OSError:
        info["load"] = None
    try:
        meminfo = dict(
            (line.split(":")[0], int(line.split()[1]) * 1024)
            for line in Path("/proc/meminfo").read_text().splitlines()
            if line.split(":")[0] in ("MemTotal", "MemAvailable")
        )
        info["memory"] = {
            "total": meminfo["MemTotal"],
            "used": meminfo["MemTotal"] - meminfo["MemAvailable"],
        }
    except (OSError, KeyError, ValueError):
        info["memory"] = None
    try:
        usage = shutil.disk_usage(data_root if data_root.exists() else REPO_ROOT)
        info["disk"] = {"total": usage.total, "used": usage.used}
    except OSError:
        info["disk"] = None
    return info


def gpu_info() -> list[dict[str, Any]] | None:
    fields = "name,utilization.gpu,memory.used,memory.total,temperature.gpu,power.draw"
    try:
        out = subprocess.run(
            ["nvidia-smi", f"--query-gpu={fields}", "--format=csv,noheader,nounits"],
            capture_output=True,
            text=True,
            timeout=3,
            check=True,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    gpus = []
    for line in out.stdout.strip().splitlines():
        parts = [p.strip() for p in line.split(",")]
        number = lambda s: float(s) if re.match(r"^[\d.]+$", s) else None  # noqa: E731
        gpus.append({
            "name": parts[0],
            "utilization": number(parts[1]),
            "memory_used_mib": number(parts[2]),
            "memory_total_mib": number(parts[3]),
            "temperature_c": number(parts[4]),
            "power_w": number(parts[5]) if len(parts) > 5 else None,
        })
    return gpus


# ------------------------------------------------------------------------------------- HTTP


def ok(body: Any, status: int = 200) -> web.Response:
    return web.json_response(body, status=status)


@web.middleware
async def errors(request: web.Request, handler):
    try:
        return await handler(request)
    except BadRequest as error:
        return ok({"error": str(error)}, 400)
    except (json.JSONDecodeError, KeyError, TypeError, ValueError) as error:
        if not request.path.startswith("/manager/api/"):
            raise
        return ok({"error": f"invalid request: {error}"}, 400)


async def body_of(request: web.Request) -> dict[str, Any]:
    body = await request.json()
    if not isinstance(body, dict):
        raise BadRequest("expected a JSON object")
    return body


def site_scene(body: dict[str, Any], site_id: str) -> dict[str, Any]:
    """scene.yaml of a new intersection: the world frame at its centre and the map area."""
    if not body.get("geo_anchor"):
        raise BadRequest("the site needs the location of the intersection centre")
    radius = float(body.get("map_radius_m") or DEFAULT_RADIUS_M)
    if not 50 <= radius <= 1000:
        raise BadRequest("the map radius must be between 50 and 1000 m")
    return {
        "scene_id": site_id,
        "kind": "custom",
        "geo_anchor": geo_anchor(body["geo_anchor"]),
        "map": {"radius_m": radius},
    }


async def proxy(request: web.Request, manager: Manager, camera: Camera, path: str):
    port = camera.runtime.port
    if camera.runtime.state not in ("running", "starting") or port is None:
        return ok({"error": f"camera {camera.key} is not running"}, 503)
    url = f"http://127.0.0.1:{port}/{path}"
    assert manager.session is not None
    if request.headers.get("Upgrade", "").lower() == "websocket":
        return await proxy_websocket(request, manager.session, url)
    try:
        async with manager.session.request(
            request.method,
            url,
            params=request.query,
            data=await request.read(),
            headers={"Content-Type": request.headers.get("Content-Type", "application/json")},
            timeout=aiohttp.ClientTimeout(total=30),
        ) as upstream:
            body = await upstream.read()
            headers = {"Cache-Control": "no-store"}
            if "Content-Type" in upstream.headers:
                headers["Content-Type"] = upstream.headers["Content-Type"]
            return web.Response(body=body, status=upstream.status, headers=headers)
    except (aiohttp.ClientError, asyncio.TimeoutError) as error:
        return ok({"error": f"camera {camera.key} did not answer: {error}"}, 502)


def routes(manager: Manager) -> web.RouteTableDef:
    r = web.RouteTableDef()
    api = "/manager/api"

    @r.get(f"{api}/health")
    async def health(_: web.Request) -> web.Response:
        return ok({"ok": True, "version": manager.version})

    @r.get(f"{api}/system")
    async def system(_: web.Request) -> web.Response:
        return ok(await asyncio.to_thread(manager.system))

    @r.post(f"{api}/system/{{action}}")
    async def system_action(request: web.Request) -> web.Response:
        action = request.match_info["action"]
        log.info("system action: %s", action)
        if action == "start-backend":
            await manager.start_backend()
        elif action == "stop-backend":
            await manager.stop_backend()
        elif action == "restart-backend":
            await manager.restart_backend()
        elif action == "restart-manager":
            # The container's restart policy brings the manager back with all sites.
            asyncio.get_running_loop().call_later(0.5, lambda: os.kill(os.getpid(), signal.SIGTERM))
        else:
            raise BadRequest(f"unknown action {action}")
        return ok({"ok": True})

    @r.get(f"{api}/sites")
    async def list_sites(_: web.Request) -> web.Response:
        return ok({"sites": [site.describe() for site in manager.sites.values()]})

    @r.post(f"{api}/sites")
    async def create_site(request: web.Request) -> web.Response:
        body = await body_of(request)
        site_id = str(body.get("id", "")).strip().lower()
        if not SITE_ID.match(site_id):
            raise BadRequest("site id: 2–48 characters, lowercase letters, digits, - and _")
        scene_dir = manager.config.scenes_dir / site_id
        if site_id in manager.sites or scene_dir.exists():
            raise BadRequest(f"a site named {site_id} exists already")
        cameras = body.get("cameras") or []
        if not isinstance(cameras, list) or len(cameras) > MAX_CAMERAS:
            raise BadRequest(f"an intersection has 1 to {MAX_CAMERAS} cameras")
        scene = site_scene(body, site_id)
        for camera in cameras:
            approach(camera.get("approach", "centre"))
            image_size(camera.get("image_size_px") or [1920, 1080])
        (scene_dir / "cameras").mkdir(parents=True)
        created = datetime.date.today().isoformat()
        write_yaml(
            scene_dir / "scene.yaml",
            scene,
            f"# Intersection created in the console on {created}; cameras in cameras/.\n",
        )
        site = Site(manager, scene_dir)
        site.settings = {
            "name": str(body.get("name") or site_id),
            "description": str(body.get("description") or ""),
            "enabled": bool(body.get("enabled", False)),
        }
        manager.sites[site_id] = site
        try:
            for camera in cameras:
                await site.add_camera(camera)
        except BadRequest:
            del manager.sites[site_id]
            shutil.rmtree(scene_dir, ignore_errors=True)
            raise
        site.save_settings()
        log.info("created site %s with %d cameras", site_id, len(site.cameras))
        if body.get("prepare_map"):
            await site.prepare_map()
        if site.enabled and manager.backend_on:
            site.start()
        return ok(site.describe(), 201)

    @r.get(f"{api}/sites/{{id}}")
    async def get_site(request: web.Request) -> web.Response:
        return ok(manager.site(request.match_info["id"]).describe())

    @r.patch(f"{api}/sites/{{id}}")
    async def update_site(request: web.Request) -> web.Response:
        site = manager.site(request.match_info["id"])
        body = await body_of(request)
        for key in ("name", "description"):
            if key in body:
                site.settings[key] = str(body[key])
        scene = dict(site.scene)
        world_changed = False
        if "map_radius_m" in body:
            radius = float(body["map_radius_m"])
            if not 50 <= radius <= 1000:
                raise BadRequest("the map radius must be between 50 and 1000 m")
            scene["map"] = {**(scene.get("map") or {}), "radius_m": radius}
        if body.get("geo_anchor"):
            anchor = geo_anchor(body["geo_anchor"])
            if site.georeferenced:
                # The world frame of every camera moves with it: their servers restart.
                world_changed = anchor != scene.get("geo_anchor")
                scene["geo_anchor"] = anchor
            else:
                # Scenes in a sensor frame (benchmark): the location only places the map.
                scene["map"] = {**(scene.get("map") or {}), "anchor": anchor}
        if "map_transform" in body:
            transform = body["map_transform"]
            scene["map"] = {
                **(scene.get("map") or {}),
                "transform": {
                    "yaw_deg": float(transform["yaw_deg"]),
                    "x_m": float(transform["x_m"]),
                    "y_m": float(transform["y_m"]),
                },
            }
        if scene != site.scene:
            write_yaml(site.dir / "scene.yaml", scene)
        site.save_settings()
        site.reload()
        log.info("updated site %s", site.id)
        if world_changed and site.is_active():
            await site.restart()
        return ok(site.describe())

    @r.delete(f"{api}/sites/{{id}}")
    async def delete_site(request: web.Request) -> web.Response:
        site = manager.site(request.match_info["id"])
        await site.stop()
        trash = manager.config.state_dir / "trash"
        trash.mkdir(parents=True, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        shutil.move(str(site.dir), str(trash / f"{site.id}-{stamp}"))
        del manager.sites[site.id]
        log.info("deleted site %s (moved to %s)", site.id, trash)
        return ok({"ok": True})

    # Registered before /sites/<id>/<action>, which would take "cameras" for an action.
    @r.post(f"{api}/sites/{{id}}/cameras")
    async def add_camera(request: web.Request) -> web.Response:
        site = manager.site(request.match_info["id"])
        camera = await site.add_camera(await body_of(request))
        if site.enabled and camera.enabled and manager.backend_on:
            camera.start()
        return ok(site.describe(), 201)

    @r.patch(f"{api}/sites/{{id}}/cameras/{{camera}}")
    async def update_camera(request: web.Request) -> web.Response:
        site = manager.site(request.match_info["id"])
        camera = site.camera(request.match_info["camera"])
        body = await body_of(request)
        allowed = {k: body[k] for k in ("source", "playback", "video", "time_offset_ms") if k in body}
        settings = merge(camera.settings, allowed)
        validate_camera_settings(settings, require_source=False)
        changes: dict[str, Any] = {}
        if "name" in body:
            changes["name"] = str(body["name"]).strip() or camera.name
        if "approach" in body:
            changes["approach"] = approach(body["approach"])
        if "image_size_px" in body:
            changes["image_size_px"] = image_size(body["image_size_px"])
        source_input = settings["source"]["input"]
        if "source" in allowed and not camera.single and source_input and "://" not in source_input:
            changes["source"] = {"video": source_input}
        if changes:
            camera.save_definition(changes)
        camera.settings = settings
        site.save_settings()
        site.reload()
        log.info("updated camera %s", camera.key)
        # Name, approach and time offset do not concern the server; everything else restarts it.
        if camera.is_active() and (set(allowed) - {"time_offset_ms"} or "image_size_px" in changes):
            await camera.restart()
        return ok(site.describe())

    @r.delete(f"{api}/sites/{{id}}/cameras/{{camera}}")
    async def delete_camera(request: web.Request) -> web.Response:
        site = manager.site(request.match_info["id"])
        await site.remove_camera(site.camera(request.match_info["camera"]))
        return ok(site.describe())

    @r.post(f"{api}/sites/{{id}}/cameras/{{camera}}/{{action}}")
    async def camera_action(request: web.Request) -> web.Response:
        site = manager.site(request.match_info["id"])
        camera = site.camera(request.match_info["camera"])
        action = request.match_info["action"]
        if action == "start":
            if not manager.backend_on:
                raise BadRequest("the backend is stopped; start it under System first")
            camera.settings["enabled"] = True
            site.settings["enabled"] = True
            site.save_settings()
            camera.start()
        elif action == "stop":
            camera.settings["enabled"] = False
            site.save_settings()
            await camera.stop()
        elif action == "restart":
            await camera.restart()
        else:
            raise BadRequest(f"unknown action {action}")
        log.info("camera %s: %s", camera.key, action)
        return ok(site.describe())

    @r.post(f"{api}/sites/{{id}}/playback")
    async def site_playback(request: web.Request) -> web.Response:
        """Play, pause or seek ({"frame", "camera"}) of all cameras of the site together."""
        site = manager.site(request.match_info["id"])
        return ok(await site.playback(await body_of(request)))

    @r.post(f"{api}/sites/{{id}}/{{action}}")
    async def site_action(request: web.Request) -> web.Response:
        site = manager.site(request.match_info["id"])
        action = request.match_info["action"]
        if action == "start":
            if not manager.backend_on:
                raise BadRequest("the backend is stopped; start it under System first")
            if not site.cameras:
                raise BadRequest("the site has no camera yet: add one in its settings")
            site.settings["enabled"] = True
            if not any(c.enabled for c in site.cameras.values()):
                for camera in site.cameras.values():
                    camera.settings["enabled"] = True
            site.save_settings()
            site.start()
        elif action == "stop":
            site.settings["enabled"] = False
            site.save_settings()
            await site.stop()
        elif action == "restart":
            await site.restart()
        elif action == "prepare-map":
            await site.prepare_map()
        else:
            raise BadRequest(f"unknown action {action}")
        log.info("site %s: %s", site.id, action)
        return ok(site.describe())

    @r.get(f"{api}/probe")
    async def probe(request: web.Request) -> web.Response:
        """Image size, frame rate and length of a video below the data root or a stream URL."""
        source_input = request.query.get("input", "").strip()
        if not source_input:
            raise BadRequest("input is missing")
        if "://" not in source_input and ".." in Path(source_input).parts:
            raise BadRequest("the path must stay below the data root")
        cfg = manager.config
        process = await asyncio.create_subprocess_exec(
            str(cfg.server_binary),
            "--probe",
            "--input",
            source_input,
            "--data-root",
            str(cfg.data_root),
            cwd=REPO_ROOT,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )
        try:
            out, err = await asyncio.wait_for(process.communicate(), cfg.probe_timeout_s)
        except asyncio.TimeoutError:
            process.kill()
            await process.wait()
            raise BadRequest(f"no answer from {source_input} within {cfg.probe_timeout_s:.0f} s")
        if process.returncode != 0:
            lines = err.decode("utf-8", "replace").strip().splitlines()
            match = SPDLOG_LINE.match(lines[-1]) if lines else None
            raise BadRequest(match.group(2) if match else f"cannot open {source_input}")
        return ok(json.loads(out.decode().strip().splitlines()[-1]))

    @r.post(f"{api}/videos")
    async def upload_video(request: web.Request) -> web.Response:
        """Stores an uploaded video below the data root (multipart field "file")."""
        reader = await request.multipart()
        part = await reader.next()
        if part is None or part.name != "file" or not part.filename:
            raise BadRequest('expected a multipart field "file"')
        name = re.sub(r"[^A-Za-z0-9._-]+", "_", Path(part.filename).name).strip("._") or "video"
        if Path(name).suffix.lower() not in VIDEO_EXTENSIONS:
            raise BadRequest("supported videos: " + ", ".join(sorted(VIDEO_EXTENSIONS)))
        folder = manager.config.upload_dir
        try:
            folder.mkdir(parents=True, exist_ok=True)
        except OSError as error:
            raise BadRequest(f"the upload folder {folder} is not writable: {error}") from error
        target = folder / name
        stem, suffix, n = target.stem, target.suffix, 1
        while target.exists():
            target = folder / f"{stem}_{n}{suffix}"
            n += 1
        partial = target.with_suffix(target.suffix + ".part")
        size = 0
        try:
            with partial.open("wb") as out:
                while chunk := await part.read_chunk(1024 * 1024):
                    size += len(chunk)
                    if size > manager.config.upload_max_bytes:
                        raise BadRequest("the video is larger than the upload limit")
                    out.write(chunk)
            partial.rename(target)
        finally:
            partial.unlink(missing_ok=True)
        rel = target.relative_to(manager.config.data_root).as_posix()
        log.info("uploaded %s (%.1f MB)", rel, size / 1024**2)
        return ok({"path": rel, "size": size}, 201)

    @r.get(f"{api}/inputs")
    async def inputs(_: web.Request) -> web.Response:
        return ok(await asyncio.to_thread(manager.inputs))

    @r.get(f"{api}/logs")
    async def logs(request: web.Request) -> web.Response:
        source = request.query.get("source", "*")
        count = min(int(request.query.get("lines", "500")), manager.config.log_lines)
        return ok({"lines": [line.to_json() for line in manager.logs.recent(source, count)]})

    @r.get(f"{api}/logs/stream")
    async def log_stream(request: web.Request) -> web.StreamResponse:
        source = request.query.get("source", "*")
        response = web.StreamResponse(
            headers={"Content-Type": "text/event-stream", "Cache-Control": "no-cache"}
        )
        await response.prepare(request)
        queue = manager.logs.subscribe(source)
        try:
            while True:
                try:
                    line = await asyncio.wait_for(queue.get(), 15)
                    await response.write(f"data: {json.dumps(line.to_json())}\n\n".encode())
                except asyncio.TimeoutError:
                    await response.write(b": keep-alive\n\n")
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        finally:
            manager.logs.unsubscribe(source, queue)
        return response

    def serve_map(site: Site, path: str) -> web.StreamResponse:
        root = (manager.config.map_cache / site.id).resolve()
        file = (root / path).resolve()
        if not file.is_file() or root not in file.parents:
            raise web.HTTPNotFound()
        return web.FileResponse(file)

    @r.route("*", "/sites/{id}/map/{path:.*}")
    async def site_map(request: web.Request) -> web.StreamResponse:
        return serve_map(manager.site(request.match_info["id"]), request.match_info["path"])

    @r.route("*", "/sites/{id}/cameras/{camera}/map/{path:.*}")
    async def camera_map(request: web.Request) -> web.StreamResponse:
        return serve_map(manager.site(request.match_info["id"]), request.match_info["path"])

    @r.route("*", "/sites/{id}/cameras/{camera}/{path:.*}")
    async def camera_proxy(request: web.Request) -> web.StreamResponse:
        site = manager.site(request.match_info["id"])
        camera = site.camera(request.match_info["camera"])
        return await proxy(request, manager, camera, request.match_info["path"])

    @r.get("/sites/{id}/ws/site")
    async def site_state_stream(request: web.Request) -> web.StreamResponse:
        site = manager.site(request.match_info["id"])
        return await manager.feed(site.id).serve(request)

    @r.get(f"{api}/sites/{{id}}/state")
    async def site_state(request: web.Request) -> web.Response:
        site = manager.site(request.match_info["id"])
        latest = manager.feed(site.id).latest
        if latest is None:
            return ok({"error": f"no site state of {site.id} yet"}, 404)
        return web.Response(text=latest, content_type="application/json")

    @r.route("*", "/sites/{id}/{path:.*}")
    async def site_proxy(request: web.Request) -> web.StreamResponse:
        site = manager.site(request.match_info["id"])
        if not site.cameras:
            return ok({"error": f"site {site.id} has no camera"}, 503)
        camera = next(iter(site.cameras.values()))
        return await proxy(request, manager, camera, request.match_info["path"])

    @r.get("/{path:.*}")
    async def static(request: web.Request) -> web.StreamResponse:
        root = manager.config.web_root
        relative = request.match_info["path"] or "index.html"
        file = (root / relative).resolve()
        if file.is_dir():
            file = file / "index.html"
        if not file.is_file() or root.resolve() not in file.parents:
            if not (root / "index.html").is_file():
                return web.Response(
                    status=503, text="console not built: run npm run build in web/"
                )
            raise web.HTTPNotFound()
        headers = {}
        if "/assets/" in f"/{relative}":
            headers["Cache-Control"] = "public, max-age=31536000, immutable"
        return web.FileResponse(file, headers=headers)

    return r


async def proxy_websocket(
    request: web.Request, session: aiohttp.ClientSession, url: str
) -> web.WebSocketResponse:
    downstream = web.WebSocketResponse(max_msg_size=0, heartbeat=None)
    await downstream.prepare(request)
    try:
        async with session.ws_connect("ws" + url[4:], max_msg_size=0) as upstream:

            async def forward(source, target) -> None:
                async for message in source:
                    if message.type == aiohttp.WSMsgType.TEXT:
                        await target.send_str(message.data)
                    elif message.type == aiohttp.WSMsgType.BINARY:
                        await target.send_bytes(message.data)
                    else:
                        break

            tasks = [
                asyncio.create_task(forward(upstream, downstream)),
                asyncio.create_task(forward(downstream, upstream)),
            ]
            _, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for task in pending:
                task.cancel()
    except (aiohttp.ClientError, ConnectionResetError):
        pass
    await downstream.close()
    return downstream


async def serve(config: ManagerConfig) -> None:
    manager = Manager(config)
    handler = HubHandler(manager.logs)
    logging.getLogger().addHandler(handler)
    manager.load_state()
    manager.load_sites()
    if not config.server_binary.is_file():
        log.warning("server binary %s not found: build it first", config.server_binary)

    app = web.Application(middlewares=[errors], client_max_size=16 * 1024**2)
    app.add_routes(routes(manager))
    manager.session = aiohttp.ClientSession()
    runner = web.AppRunner(app, access_log=None)
    await runner.setup()
    await web.TCPSite(runner, config.address, config.port).start()
    log.info("console on http://%s:%d (version %s)", config.address, config.port, manager.version)

    if manager.backend_on:
        await manager.start_backend()
    else:
        log.info("backend is stopped (start it under System)")
    status = asyncio.create_task(manager.status_loop())

    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set)
    await stop.wait()
    log.info("stopping")
    status.cancel()
    await asyncio.gather(*(feed.close() for feed in manager.feeds.values()))
    await asyncio.gather(*(site.stop() for site in manager.sites.values()))
    await manager.session.close()
    await runner.cleanup()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", type=Path, default=REPO_ROOT / "configs" / "manager.yaml")
    args = parser.parse_args()
    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s] [%(levelname)s] %(message)s",
        stream=sys.stdout,
    )
    asyncio.run(serve(ManagerConfig.load(args.config)))


if __name__ == "__main__":
    main()
