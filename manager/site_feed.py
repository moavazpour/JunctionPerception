"""Live glue of the site world model: follows the camera servers of a site, runs its
SiteWorldModel and serves the resulting site state on /sites/<id>/ws/site.

It runs whether or not a browser is connected (headless), and every client receives the same
state. The model itself (world_model.py) holds all association and fusion logic.
"""

from __future__ import annotations

import asyncio
import json
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import aiohttp
from aiohttp import web

from world_model import ClassSize, FusionSettings, SiteWorldModel

RECONNECT_S = 1.0


@dataclass(frozen=True)
class FeedCamera:
    port: int
    offset_ns: int


class SiteFeed:
    """World model and clients of one site."""

    def __init__(self, site_id: str, settings: FusionSettings, class_sizes: dict[str, ClassSize],
                 log: Callable[[str, str], None]) -> None:
        self.site_id = site_id
        self.settings = settings
        self.class_sizes = class_sizes
        self.log = log
        self.cameras: dict[str, FeedCamera] = {}
        self.shared_clock = True
        self.model = SiteWorldModel(site_id, {}, settings, class_sizes)
        self.tasks: dict[str, asyncio.Task[None]] = {}
        self.clients: set[web.WebSocketResponse] = set()
        self.latest: str | None = None

    def configure(self, session: aiohttp.ClientSession, cameras: dict[str, FeedCamera],
                  shared_clock: bool) -> None:
        """Running cameras of the site (id -> port and time offset). A changed set of cameras or
        clock rebuilds the model: global IDs start over."""
        if cameras != self.cameras or shared_clock != self.shared_clock:
            for camera_id in [c for c in self.tasks if c not in cameras or cameras[c] != self.cameras.get(c)]:
                self.tasks.pop(camera_id).cancel()
            self.cameras = dict(cameras)
            self.shared_clock = shared_clock
            self.model = SiteWorldModel(
                self.site_id, {c: v.offset_ns for c, v in cameras.items()}, self.settings,
                self.class_sizes, shared_media_clock=shared_clock or len(cameras) <= 1,
            )
            self.log("info", f"world model: {len(cameras)} camera(s), "
                             f"{'one clock' if shared_clock else 'own clocks'}")
        for camera_id, camera in cameras.items():
            task = self.tasks.get(camera_id)
            if task is None or task.done():
                self.tasks[camera_id] = asyncio.create_task(self.follow(session, camera_id, camera.port))

    async def close(self) -> None:
        for task in self.tasks.values():
            task.cancel()
        self.tasks.clear()
        for client in list(self.clients):
            await client.close()

    async def follow(self, session: aiohttp.ClientSession, camera_id: str, port: int) -> None:
        """Reads one camera's observation stream; reconnects after drops."""
        while True:
            try:
                async with session.ws_connect(f"ws://127.0.0.1:{port}/ws/world",
                                              max_msg_size=0, heartbeat=None) as ws:
                    async for message in ws:
                        if message.type != aiohttp.WSMsgType.TEXT:
                            break
                        try:
                            self.on_state(camera_id, json.loads(message.data))
                        except (ValueError, KeyError, TypeError) as error:
                            self.log("warning", f"world model: bad state of {camera_id} ({error})")
            except (aiohttp.ClientError, ConnectionResetError, asyncio.TimeoutError):
                pass
            await asyncio.sleep(RECONNECT_S)

    def on_state(self, camera_id: str, state: dict[str, Any]) -> None:
        self.model.observe(camera_id, state)
        site_state = self.model.step()
        if site_state is not None:
            self.latest = json.dumps(site_state, separators=(",", ":"))
            for client in list(self.clients):
                if client.closed:
                    self.clients.discard(client)
                else:
                    asyncio.ensure_future(self.send(client, self.latest))

    async def send(self, client: web.WebSocketResponse, text: str) -> None:
        try:
            await client.send_str(text)
        except (ConnectionResetError, RuntimeError):
            self.clients.discard(client)

    async def serve(self, request: web.Request) -> web.WebSocketResponse:
        """A client of the site state: the latest state at once, then every new one."""
        ws = web.WebSocketResponse(heartbeat=30)
        await ws.prepare(request)
        self.clients.add(ws)
        if self.latest is not None:
            await self.send(ws, self.latest)
        try:
            async for _ in ws:
                pass  # clients only listen
        finally:
            self.clients.discard(ws)
        return ws
