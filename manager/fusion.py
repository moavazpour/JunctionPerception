"""Road users of a multi-camera site fused into one list (jp/<site>/fused/objects).

Every camera runs its own tracker and publishes its road users with lat/lon. RATE_HZ times a second the latest
list of each camera (if fresh) is dead-reckoned to the same instant (each object moved along its heading by its
speed for the time since its list arrived), brought into one local frame (ENU around the site's first object),
and objects seen by several cameras are associated: the same class group, within GATE_M metres, headings within
GATE_HEADING degrees when both move, at most one object per camera in a group. Pairs that formed one road user
on the previous tick are kept together more readily, so groups do not flicker. A group becomes one road user:
position averaged with 1 / sigma^2 weights, the rest from the member with the smallest position sigma.

Global ids stay stable and unique: each (camera, track id) keeps the global id it was given, a group takes the
oldest free global id among its members, and a group of new tracks gets a new one.

When the cameras behind a road user change (a second camera picks it up, one loses it), the averaged position
and the best member's heading step by the disagreement between the cameras. That step is not shown at once: it
is kept as an offset that fades out over BLEND_S, so the road user glides to its new position.
"""
from __future__ import annotations

import json
import math
import threading
import time
from typing import Callable

import paho.mqtt.client as mqtt

GATE_M = 2.5
GATE_HEADING = 45.0
KEEP_BONUS = 1.6     # gate widened (and distance shrunk) by this factor for pairs fused on the previous tick
FRESH_S = 1.0
MAX_AHEAD_S = 0.5    # dead-reckoning limit
RATE_HZ = 15.0
BLEND_S = 0.6        # time constant of the offset fading after a change of cameras
MAX_BLEND_M = 5.0    # larger steps are real (a wrong association ended) and shown at once
VEHICLES = {"car", "van", "truck", "bus", "trailer"}
TWO_WHEEL = {"bicycle", "motorcycle", "escooter"}
K = 111320.0


def group_of(cls: str) -> str:
    return "vehicle" if cls in VEHICLES else "two_wheel" if cls in TWO_WHEEL else cls


def heading_diff(a: float, b: float) -> float:
    return abs((a - b + 180.0) % 360.0 - 180.0)


class Fusion(threading.Thread):
    def __init__(self, uri: str, sites: Callable[[], dict[str, list[str]]], prefix: str = "jp"):
        super().__init__(daemon=True, name="fusion")
        self.uri, self.sites, self.prefix = uri, sites, prefix
        # (site, camera) -> (received, objects, source video frame)
        self.latest: dict[tuple[str, str], tuple[float, list[dict], int | None]] = {}
        self.gid: dict[tuple[str, str, int], tuple[int, float]] = {}       # (site, camera, id) -> (global id, last seen)
        self.next_id: dict[str, int] = {}
        self.origin: dict[str, tuple[float, float]] = {}                   # site -> lat0, lon0 of the local frame
        # (site, global id) -> state of the output: x, y, heading, offsets dx, dy, dh, members, time
        self.out: dict[tuple[str, int], dict] = {}
        self.lock = threading.Lock()
        self.multi: dict[str, list[str]] = {}

    def run(self):
        host, port = self.uri.replace("tcp://", "").split(":")
        c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="jp-fusion")
        c.on_connect = lambda cl, *_: cl.subscribe(f"{self.prefix}/+/+/objects")
        c.on_message = self.on_message
        c.connect_async(host, int(port))
        c.loop_start()
        t_sites = 0.0
        t_next = time.time()
        while True:
            now = time.time()
            if now - t_sites > 5:
                try:
                    self.multi = self.sites()
                except Exception:
                    pass
                t_sites = now
            for site, cams in self.multi.items():
                out = self.fuse(site, cams, now)
                if out is not None:
                    c.publish(f"{self.prefix}/{site}/fused/objects", json.dumps(out), qos=0)
            t_next = max(t_next + 1.0 / RATE_HZ, time.time())
            time.sleep(max(0.0, t_next - time.time()))

    def on_message(self, client, userdata, msg):
        parts = msg.topic.split("/")
        if len(parts) != 4 or parts[2] == "fused" or parts[1] not in self.multi:
            return
        try:
            m = json.loads(msg.payload)
        except ValueError:
            return
        with self.lock:
            self.latest[(parts[1], parts[2])] = (time.time(), m.get("objects") or [], m.get("source_frame"))

    def local(self, site: str, lat: float, lon: float) -> tuple[float, float]:
        lat0, lon0 = self.origin[site]
        return (lon - lon0) * K * math.cos(math.radians(lat0)), (lat - lat0) * K

    def geo(self, site: str, x: float, y: float) -> tuple[float, float]:
        lat0, lon0 = self.origin[site]
        return lat0 + y / K, lon0 + x / (K * math.cos(math.radians(lat0)))

    def fuse(self, site: str, cams: list[str], now: float) -> dict | None:
        lists: dict[str, tuple[float, list[dict]]] = {}
        frames: list[int] = []
        with self.lock:
            for cam in cams:
                v = self.latest.get((site, cam))
                if v and now - v[0] < FRESH_S:
                    lists[cam] = (v[0], v[1])
                    if v[2] is not None:
                        frames.append(v[2])
        if not lists:
            return None
        items = [(cam, o) for cam, (_, objs) in lists.items() for o in objs if "lat" in o and o.get("state") != "tentative"]
        if not items:
            return self.message(site, now, [], frames)
        self.origin.setdefault(site, (items[0][1]["lat"], items[0][1]["lon"]))
        # Every list at the same instant: move each object along its heading for the age of its list.
        pts = []
        for cam, o in items:
            x, y = self.local(site, o["lat"], o["lon"])
            ahead = min(now - lists[cam][0], MAX_AHEAD_S)
            v, h = o.get("speed", 0.0) or 0.0, math.radians(o.get("heading", 0.0) or 0.0)
            pts.append((x + v * ahead * math.sin(h), y + v * ahead * math.cos(h)))
        prev = {k: g for k, (g, _) in self.gid.items()}
        # Greedy association over the closest compatible pairs.
        pairs = []
        for i in range(len(items)):
            for j in range(i + 1, len(items)):
                (ci, a), (cj, b) = items[i], items[j]
                if ci == cj or group_of(a["class"]) != group_of(b["class"]):
                    continue
                d = math.hypot(pts[i][0] - pts[j][0], pts[i][1] - pts[j][1])
                gi, gj = prev.get((site, ci, int(a["id"]))), prev.get((site, cj, int(b["id"])))
                kept = gi is not None and gi == gj
                if d > GATE_M * (KEEP_BONUS if kept else 1.0):
                    continue
                if a.get("speed", 0) > 1.0 and b.get("speed", 0) > 1.0 and heading_diff(a["heading"], b["heading"]) > GATE_HEADING:
                    continue
                pairs.append((d / KEEP_BONUS if kept else d, i, j))
        pairs.sort()
        group = list(range(len(items)))  # union-find

        def root(k: int) -> int:
            while group[k] != k:
                group[k] = group[group[k]]
                k = group[k]
            return k

        members: dict[int, set[str]] = {i: {items[i][0]} for i in range(len(items))}
        for _, i, j in pairs:
            ri, rj = root(i), root(j)
            if ri == rj or members[ri] & members[rj]:
                continue  # one object per camera in a group
            group[rj] = ri
            members[ri] |= members.pop(rj)
        groups: dict[int, list[int]] = {}
        for k in range(len(items)):
            groups.setdefault(root(k), []).append(k)
        # Global ids: groups holding the oldest ids choose first, an id is used once.
        def known(ks: list[int]) -> list[int]:
            return sorted({prev[(site, items[k][0], int(items[k][1]["id"]))] for k in ks
                           if (site, items[k][0], int(items[k][1]["id"])) in prev})

        order = sorted(groups.values(), key=lambda ks: (known(ks) or [math.inf])[0])
        used: set[int] = set()
        fused = []
        for ks in order:
            g = next((x for x in known(ks) if x not in used), None)
            if g is None:
                g = self.next_id.get(site, 1)
                self.next_id[site] = g + 1
            used.add(g)
            fused.append(self.merge(site, g, [items[k] for k in ks], [pts[k] for k in ks], now))
        # Forget associations of tracks not seen for a while, and outputs that ended.
        self.gid = {k: v for k, v in self.gid.items() if now - v[1] < 10}
        self.out = {k: v for k, v in self.out.items() if k[0] != site or k[1] in used or now - v["t"] < 2}
        return self.message(site, now, fused, frames)

    def merge(self, site: str, g: int, members: list[tuple[str, dict]], pts: list[tuple[float, float]], now: float) -> dict:
        for cam, o in members:
            self.gid[(site, cam, int(o["id"]))] = (g, now)
        k = min(range(len(members)), key=lambda i: members[i][1].get("sigma", {}).get("pos", 1.0))
        best = members[k][1]
        w = [1.0 / max(o.get("sigma", {}).get("pos", 1.0), 0.05) ** 2 for _, o in members]
        W = sum(w)
        x = sum(wi * p[0] for wi, p in zip(w, pts)) / W
        y = sum(wi * p[1] for wi, p in zip(w, pts)) / W
        h = float(best.get("heading", 0.0) or 0.0)
        cams = sorted(cam for cam, _ in members)
        s = self.out.get((site, g))
        if s is None:
            s = {"dx": 0.0, "dy": 0.0, "dh": 0.0}
        else:
            dt = max(now - s["t"], 0.0)
            fade = math.exp(-dt / BLEND_S)
            s["dx"] *= fade
            s["dy"] *= fade
            s["dh"] *= fade
            if s["cams"] != cams:
                # Where the road user was shown, moved on to now; the new cameras' position takes over slowly.
                v = float(best.get("speed", 0.0) or 0.0)
                hr = math.radians(s["h"])
                ex = s["x"] + s["dx"] + v * dt * math.sin(hr) - x
                ey = s["y"] + s["dy"] + v * dt * math.cos(hr) - y
                if math.hypot(ex, ey) < MAX_BLEND_M:
                    s["dx"], s["dy"] = ex, ey
                else:
                    s["dx"] = s["dy"] = 0.0
                s["dh"] = (s["h"] + s["dh"] - h + 180.0) % 360.0 - 180.0
        s.update(x=x, y=y, h=h, cams=cams, t=now)
        self.out[(site, g)] = s
        lat, lon = self.geo(site, x + s["dx"], y + s["dy"])
        out = dict(best)
        out["id"] = g
        out["lat"] = round(lat, 7)
        out["lon"] = round(lon, 7)
        out["heading"] = round((h + s["dh"]) % 360.0, 2)
        if "yaw" in out:
            out["yaw"] = round(math.radians(90.0 - out["heading"]), 4)
        out["confidence"] = round(max(o.get("confidence", 0) for _, o in members), 2)
        out["cameras"] = cams
        out["source_ids"] = {cam: o["id"] for cam, o in members}
        return out

    def message(self, site: str, now: float, objects: list[dict], frames: list[int]) -> dict:
        return {"schema": "junction-perception/1.0", "type": "objects", "site": site, "camera": "fused",
                "ts": round(now, 3), "source_frame": max(frames) if frames else None, "objects": objects}
