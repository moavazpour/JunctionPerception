"""Site manager: the always-on service behind the web console.

A site is a scene folder (scenes/<id>/: scene.yaml, calibration, zones) plus the runtime settings
the console edits (scenes/<id>/site.yaml: name, camera source, playback, video stream). The manager
runs one jp_server per active site on a local port and proxies it under /sites/<id>/, restarts
servers that exit (a calibration "apply" exits on purpose), keeps their logs and reports the host.

  GET  /manager/api/health                    liveness and version
  GET  /manager/api/system                    host, GPU, backend state
  POST /manager/api/system/<action>           start-backend | stop-backend | restart-backend |
                                              restart-manager
  GET  /manager/api/sites                     all sites with runtime state and live status
  POST /manager/api/sites                     create a site (scene folder + site.yaml)
  GET  /manager/api/sites/<id>                one site
  PATCH /manager/api/sites/<id>               change settings (restarts a running site)
  DELETE /manager/api/sites/<id>              stop and move the scene folder to the trash
  POST /manager/api/sites/<id>/<action>       start | stop | restart | prepare-map
  GET  /manager/api/inputs                    videos, sequences and replays to choose from
  GET  /manager/api/probe?input=              image size, frame rate and length of a video or stream
  POST /manager/api/videos                    upload a video (multipart "file") below the data root
  GET  /manager/api/logs?source=&lines=       recent log lines (source: manager or a site id)
  GET  /manager/api/logs/stream?source=       the same as server-sent events
  *    /sites/<id>/map/...                    map layers (also while the site is stopped)
  *    /sites/<id>/...                        proxied to the site's jp_server (HTTP, WebSockets)
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
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import aiohttp
import yaml
from aiohttp import web

REPO_ROOT = Path(__file__).resolve().parent.parent
SITE_ID = re.compile(r"^[a-z0-9][a-z0-9_-]{1,47}$")
MODES = ("live", "oracle", "replay")
VIDEO_EXTENSIONS = {".mp4", ".mov", ".mkv", ".avi", ".m4v"}
# spdlog's default pattern: "[2026-11-01 12:00:00.123] [info] message".
SPDLOG_LINE = re.compile(r"^\[[^\]]+\] \[(\w+)\] (.*)$")
STATUS_INTERVAL_S = 1.0
STOP_TIMEOUT_S = 8.0
STABLE_RUN_S = 60.0
INPUT_SCAN_DEPTH = 4

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
    """Recent lines per source (the manager and each site) and live subscribers."""

    def __init__(self, capacity: int) -> None:
        self.capacity = capacity
        self.lines: dict[str, collections.deque[LogLine]] = {}
        self.subscribers: dict[str, set[asyncio.Queue[LogLine]]] = {}

    def add(self, source: str, level: str, message: str) -> None:
        line = LogLine(time.time(), level, source, message)
        self.lines.setdefault(source, collections.deque(maxlen=self.capacity)).append(line)
        for key in (source, "*"):
            for queue in self.subscribers.get(key, ()):
                if queue.qsize() < 1000:
                    queue.put_nowait(line)

    def recent(self, source: str, count: int) -> list[LogLine]:
        if source == "*":
            merged = [line for lines in self.lines.values() for line in lines]
            return sorted(merged, key=lambda line: line.time)[-count:]
        return list(self.lines.get(source, ()))[-count:]

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

    @staticmethod
    def load(path: Path) -> ManagerConfig:
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
        resolve = lambda p: (REPO_ROOT / p).resolve()  # noqa: E731
        ports = raw["site_ports"]
        backoff = raw["restart_backoff_s"]
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
            data_root=Path(os.environ.get("JP_DATA_ROOT", raw.get("data_root", "/data"))),
            upload_dir=Path(os.environ.get("JP_DATA_ROOT", raw.get("data_root", "/data")))
            / raw.get("upload_dir", "uploads"),
            upload_max_bytes=int(float(raw.get("upload_max_gb", 20)) * 1024**3),
            probe_timeout_s=float(raw.get("probe_timeout_s", 20)),
        )


def read_yaml(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    return yaml.safe_load(path.read_text(encoding="utf-8")) or {}


def write_yaml(path: Path, data: dict[str, Any], header: str = "") -> None:
    text = yaml.safe_dump(data, sort_keys=False, allow_unicode=True)
    path.write_text(header + text, encoding="utf-8")


class BadRequest(Exception):
    """Invalid request content; becomes a 400 response with the message."""


# ------------------------------------------------------------------------------------ sites


def default_settings(scene_id: str, scene: dict[str, Any], base: dict[str, Any]) -> dict[str, Any]:
    """Settings of a site without site.yaml: the startup scene of server.yaml runs, others don't."""
    startup = base.get("startup", {})
    is_startup = Path(str(startup.get("scene", ""))).name == scene_id
    source = scene.get("source") or {}
    if is_startup:
        mode, source_input = startup.get("mode", "live"), startup.get("input", "")
    else:
        mode, source_input = "live", source.get("video", "")
    return {
        "name": scene_id.replace("_", " "),
        "description": "",
        "enabled": is_startup,
        "source": {"mode": mode, "input": source_input},
        "playback": dict(base.get("playback", {"loop": True, "speed": 1.0})),
        "video": dict(base.get("video", {"max_fps": 5, "width_px": 960, "jpeg_quality": 70})),
    }


def merge(base: dict[str, Any], patch: dict[str, Any]) -> dict[str, Any]:
    out = dict(base)
    for key, value in patch.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = merge(out[key], value)
        else:
            out[key] = value
    return out


def validate_settings(settings: dict[str, Any]) -> None:
    source = settings["source"]
    if source.get("mode") not in MODES:
        raise BadRequest("source.mode must be live, oracle or replay")
    if not isinstance(source.get("input"), str) or not source["input"].strip():
        raise BadRequest("the site needs a camera source (video, stream URL or recording)")
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


@dataclass
class SiteRuntime:
    """Process state of one site; `state` is what the console shows."""

    state: str = "stopped"  # stopped | starting | running | stopping | backoff | failed
    process: asyncio.subprocess.Process | None = None
    port: int | None = None
    started_at: float | None = None
    restarts: int = 0
    last_exit: int | None = None
    error: str | None = None
    status: dict[str, Any] | None = None
    backoff_s: float = 0.0
    map_job: dict[str, Any] | None = None
    restart_now: bool = False
    stopping: bool = False
    supervisor: asyncio.Task[None] | None = None
    tasks: set[asyncio.Task[Any]] = field(default_factory=set)


class Site:
    def __init__(self, manager: Manager, scene_dir: Path) -> None:
        self.manager = manager
        self.dir = scene_dir
        self.id = scene_dir.name
        self.runtime = SiteRuntime()
        self.reload()

    def reload(self) -> None:
        self.scene = read_yaml(self.dir / "scene.yaml")
        stored = read_yaml(self.dir / "site.yaml")
        self.settings = merge(default_settings(self.id, self.scene, self.manager.base_config), stored)

    def save_settings(self) -> None:
        write_yaml(
            self.dir / "site.yaml",
            self.settings,
            "# Site settings edited in the console (manager/jp_manager.py).\n",
        )

    @property
    def georeferenced(self) -> bool:
        """World frame is local east-north-up at the geo anchor (custom scenes)."""
        return bool(self.scene.get("geo_anchor"))

    @property
    def location(self) -> dict[str, Any] | None:
        """Geographic position of the site: its geo anchor or, for scenes in a sensor frame, the
        anchor of the map layers (aligned in the console with map.transform)."""
        return self.scene.get("geo_anchor") or (self.scene.get("map") or {}).get("anchor")

    @property
    def enabled(self) -> bool:
        return bool(self.settings.get("enabled"))

    def describe(self) -> dict[str, Any]:
        r = self.runtime
        return {
            "id": self.id,
            "name": self.settings.get("name") or self.id,
            "description": self.settings.get("description", ""),
            "enabled": self.enabled,
            "kind": self.scene.get("kind", "custom"),
            "image_size_px": self.scene.get("image_size_px"),
            "geo_anchor": self.location,
            "georeferenced": self.georeferenced,
            "map_radius_m": (self.scene.get("map") or {}).get("radius_m"),
            "map_transform": (self.scene.get("map") or {}).get("transform"),
            "calibrated": (self.dir / "ground_homography.yaml").is_file(),
            "has_map": (self.manager.config.map_cache / self.id / "map.json").is_file(),
            "zones": (self.dir / "zones.geojson").is_file(),
            "source": self.settings["source"],
            "playback": self.settings["playback"],
            "video": self.settings["video"],
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
                "map_job": r.map_job,
            },
        }

    def server_yaml(self) -> Path:
        """Server config of this site: the shared base with its port, playback and video."""
        config = json.loads(json.dumps(self.manager.base_config))
        config["server"]["bind_address"] = "127.0.0.1"
        config["server"]["port"] = self.runtime.port
        config["video"] = merge(config.get("video", {}), self.settings["video"])
        config["playback"] = merge(config.get("playback", {}), self.settings["playback"])
        source = self.settings["source"]
        config["startup"] = {
            "scene": str(self.dir),
            "mode": source["mode"],
            "input": source["input"],
        }
        path = self.manager.config.state_dir / "sites" / self.id / "server.yaml"
        path.parent.mkdir(parents=True, exist_ok=True)
        write_yaml(path, config, f"# Generated by the manager for site {self.id}.\n")
        return path

    # -------------------------------------------------------------------- process control

    def start(self) -> None:
        """Runs the site's server and keeps it running until stop()."""
        r = self.runtime
        if r.supervisor and not r.supervisor.done():
            return
        if not str(self.settings["source"].get("input", "")).strip():
            r.state, r.error = "failed", "no camera source configured: set one in the site settings"
            self.manager.logs.add(self.id, "error", r.error)
            return
        running = sum(1 for s in self.manager.sites.values() if s.is_active())
        if running >= self.manager.config.max_running:
            r.state, r.error = "failed", (
                f"capacity reached: at most {self.manager.config.max_running} sites run at once"
            )
            self.manager.logs.add(self.id, "error", r.error)
            return
        r.error = None
        r.backoff_s = 0.0
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
            self.manager.logs.add(self.id, "warning", "server did not stop in time; killed")
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
                self.manager.logs.add(self.id, "info", f"starting server on port {r.port}")
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
                    self.manager.logs.add(self.id, "error", r.error)
                    return
                assert r.process.stdout is not None
                await self.pump(r.process.stdout)
                code = await r.process.wait()
                ran = time.time() - (r.started_at or time.time())
                r.last_exit, r.status, r.started_at = code, None, None
                if r.stopping:
                    self.manager.logs.add(self.id, "info", "server stopped")
                    return
                if r.restart_now or code == 0:
                    # Restart from the console, or the server's own /api/restart after a
                    # calibration was applied.
                    self.manager.logs.add(self.id, "info", f"server exited ({code}); restarting")
                    r.restarts += 1
                    continue
                r.backoff_s = (
                    cfg.backoff_s[0]
                    if ran > STABLE_RUN_S or r.backoff_s == 0
                    else min(r.backoff_s * 2, cfg.backoff_s[1])
                )
                r.state, r.error = "backoff", f"server exited with code {code}"
                self.manager.logs.add(
                    self.id, "error", f"server exited with code {code}; retry in {r.backoff_s:.0f} s"
                )
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
            self.manager.logs.add(self.id, "warning" if level == "warn" else level, message)

    async def poll_status(self, session: aiohttp.ClientSession) -> None:
        r = self.runtime
        if r.state not in ("starting", "running") or r.port is None:
            return
        try:
            timeout = aiohttp.ClientTimeout(total=0.8)
            async with session.get(f"http://127.0.0.1:{r.port}/api/status", timeout=timeout) as resp:
                r.status = await resp.json()
                if r.state == "starting":
                    self.manager.logs.add(self.id, "info", "server is up")
                r.state = "running"
        except (aiohttp.ClientError, asyncio.TimeoutError, ValueError):
            r.status = None

    async def prepare_map(self) -> None:
        r = self.runtime
        if r.map_job and r.map_job["state"] == "running":
            raise BadRequest("map data is already being prepared")
        if not self.location:
            raise BadRequest("the site needs a geographic anchor (latitude, longitude) first")
        r.map_job = {"state": "running", "started_at": time.time(), "message": "downloading"}
        task = asyncio.create_task(self._map_job())
        r.tasks.add(task)
        task.add_done_callback(r.tasks.discard)

    async def _map_job(self) -> None:
        r = self.runtime
        assert r.map_job is not None
        self.manager.logs.add(self.id, "info", "preparing map layers (orthophoto, OSM, buildings)")
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
                self.manager.logs.add(self.id, "info", f"map: {last}")
        code = await process.wait()
        ok = code == 0
        r.map_job.update(
            state="done" if ok else "failed",
            finished_at=time.time(),
            message="map layers ready" if ok else (last or f"exit code {code}"),
        )
        self.manager.logs.add(self.id, "info" if ok else "error", f"map: {r.map_job['message']}")


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
            if (scene_dir / "scene.yaml").is_file() and SITE_ID.match(scene_dir.name):
                try:
                    self.sites[scene_dir.name] = Site(self, scene_dir)
                except (yaml.YAMLError, OSError, KeyError) as error:
                    log.error("cannot load site %s: %s", scene_dir.name, error)
        log.info("%d sites in %s", len(self.sites), self.config.scenes_dir)

    def load_state(self) -> None:
        if self.state_file.is_file():
            self.backend_on = bool(json.loads(self.state_file.read_text()).get("backend_on", True))

    def save_state(self) -> None:
        self.state_file.parent.mkdir(parents=True, exist_ok=True)
        self.state_file.write_text(json.dumps({"backend_on": self.backend_on}))

    def free_port(self, site: Site) -> int:
        used = {s.runtime.port for s in self.sites.values() if s is not site and s.is_active()}
        for port in range(self.config.first_port, self.config.last_port + 1):
            if port not in used:
                return port
        raise RuntimeError("no free site port")

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
            await asyncio.gather(*(s.poll_status(self.session) for s in self.sites.values()))
            await asyncio.sleep(STATUS_INTERVAL_S)

    def system(self) -> dict[str, Any]:
        sites = list(self.sites.values())
        return {
            "version": self.version,
            "started_at": self.started_at,
            "uptime_s": time.time() - self.started_at,
            "backend_on": self.backend_on,
            "sites_total": len(sites),
            "sites_enabled": sum(s.enabled for s in sites),
            "sites_running": sum(s.runtime.state == "running" for s in sites),
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


def scene_yaml(body: dict[str, Any], site_id: str) -> dict[str, Any]:
    size = body.get("image_size_px")
    if not (isinstance(size, list) and len(size) == 2 and all(int(v) >= 64 for v in size)):
        raise BadRequest("image_size_px must be [width, height] in pixels")
    scene: dict[str, Any] = {
        "scene_id": site_id,
        "kind": "custom",
        "image_size_px": [int(size[0]), int(size[1])],
    }
    source_input = (body.get("source") or {}).get("input", "")
    if source_input and "://" not in source_input:
        scene["source"] = {"video": source_input}
    anchor = body.get("geo_anchor")
    if anchor:
        lat, lon = float(anchor["lat_deg"]), float(anchor["lon_deg"])
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            raise BadRequest("latitude must be within ±90°, longitude within ±180°")
        scene["geo_anchor"] = {"lat_deg": lat, "lon_deg": lon, "alt_m": float(anchor.get("alt_m", 0))}
    scene["map"] = {"radius_m": float(body.get("map_radius_m") or 250)}
    scene["calibration"] = "ground_homography.yaml"
    return scene


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
        settings = merge(
            default_settings(site_id, {}, manager.base_config),
            {
                "name": str(body.get("name") or site_id),
                "description": str(body.get("description") or ""),
                "enabled": bool(body.get("enabled", False)),
                "source": body.get("source") or {},
            },
        )
        validate_settings(settings)
        scene = scene_yaml(body, site_id)
        scene_dir.mkdir(parents=True)
        created = datetime.date.today().isoformat()
        write_yaml(scene_dir / "scene.yaml", scene, f"# Created in the console on {created}.\n")
        site = Site(manager, scene_dir)
        site.settings = settings
        site.save_settings()
        manager.sites[site_id] = site
        log.info("created site %s", site_id)
        if body.get("prepare_map") and scene.get("geo_anchor"):
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
        allowed = {k: body[k] for k in ("name", "description", "source", "playback", "video") if k in body}
        settings = merge(site.settings, allowed)
        validate_settings(settings)
        scene_keys = ("image_size_px", "geo_anchor", "map_radius_m")
        if any(k in body for k in scene_keys):
            current = {
                "image_size_px": site.scene.get("image_size_px"),
                "geo_anchor": site.location,
                "map_radius_m": (site.scene.get("map") or {}).get("radius_m"),
                "source": settings["source"],
            }
            changed = scene_yaml(merge(current, {k: body[k] for k in scene_keys if k in body}), site.id)
            scene = dict(site.scene)
            scene["image_size_px"] = changed["image_size_px"]
            scene["map"] = {**(scene.get("map") or {}), **changed["map"]}
            if "geo_anchor" in changed:
                if site.georeferenced:
                    scene["geo_anchor"] = changed["geo_anchor"]
                else:
                    # Scenes in a sensor frame (benchmark): the location only places the map.
                    scene["map"]["anchor"] = changed["geo_anchor"]
            if scene != site.scene:
                write_yaml(site.dir / "scene.yaml", scene)
        if "map_transform" in body:
            transform = body["map_transform"]
            scene = dict(site.scene)
            scene["map"] = {
                **(scene.get("map") or {}),
                "transform": {
                    "yaw_deg": float(transform["yaw_deg"]),
                    "x_m": float(transform["x_m"]),
                    "y_m": float(transform["y_m"]),
                },
            }
            write_yaml(site.dir / "scene.yaml", scene)
        site.settings = settings
        site.save_settings()
        site.reload()
        log.info("updated site %s", site.id)
        # Aligning the map changes only what the console draws; everything else restarts the site.
        if site.is_active() and (allowed or any(k in body for k in scene_keys)):
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

    @r.post(f"{api}/sites/{{id}}/{{action}}")
    async def site_action(request: web.Request) -> web.Response:
        site = manager.site(request.match_info["id"])
        action = request.match_info["action"]
        if action == "start":
            site.settings["enabled"] = True
            site.save_settings()
            if not manager.backend_on:
                raise BadRequest("the backend is stopped; start it under System first")
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
        field = await reader.next()
        if field is None or field.name != "file" or not field.filename:
            raise BadRequest('expected a multipart field "file"')
        name = re.sub(r"[^A-Za-z0-9._-]+", "_", Path(field.filename).name).strip("._") or "video"
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
                while chunk := await field.read_chunk(1024 * 1024):
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

    @r.route("*", "/sites/{id}/map/{path:.*}")
    async def site_map(request: web.Request) -> web.StreamResponse:
        site = manager.site(request.match_info["id"])
        root = (manager.config.map_cache / site.id).resolve()
        file = (root / request.match_info["path"]).resolve()
        if not file.is_file() or root not in file.parents:
            raise web.HTTPNotFound()
        return web.FileResponse(file)

    @r.route("*", "/sites/{id}/{path:.*}")
    async def site_proxy(request: web.Request) -> web.StreamResponse:
        site = manager.site(request.match_info["id"])
        port = site.runtime.port
        if site.runtime.state not in ("running", "starting") or port is None:
            return ok({"error": f"site {site.id} is not running"}, 503)
        url = f"http://127.0.0.1:{port}/{request.match_info['path']}"
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
            return ok({"error": f"site {site.id} did not answer: {error}"}, 502)

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
