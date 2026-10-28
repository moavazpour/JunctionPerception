"""Road users of a multi-camera site fused into one list (jp/<site>/fused/objects).

Every camera runs its own tracker and publishes its road users with lat/lon. Ten times a second the latest
list of each camera (if fresh) is brought into one local frame (ENU around the site's first object), and
objects seen by several cameras are associated: the same class group, within GATE_M metres, headings within
GATE_HEADING degrees when both move, at most one object per camera in a group. A group becomes one road
user: position averaged with 1 / sigma^2 weights, the rest from the member with the smallest position sigma.

Global ids stay stable: each (camera, track id) keeps the global id it was given; a group takes the oldest
global id among its members, and a group of new tracks gets a new one.
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
FRESH_S = 1.0
RATE_HZ = 10.0
VEHICLES = {"car", "van", "truck", "bus", "trailer"}
TWO_WHEEL = {"bicycle", "motorcycle", "escooter"}


def group_of(cls: str) -> str:
    return "vehicle" if cls in VEHICLES else "two_wheel" if cls in TWO_WHEEL else cls


def enu(lat0: float, lon0: float, lat: float, lon: float) -> tuple[float, float]:
    k = 111320.0
    return (lon - lon0) * k * math.cos(math.radians(lat0)), (lat - lat0) * k


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
            time.sleep(1.0 / RATE_HZ)

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

    def fuse(self, site: str, cams: list[str], now: float) -> dict | None:
        lists: dict[str, list[dict]] = {}
        frames: list[int] = []
        with self.lock:
            for cam in cams:
                v = self.latest.get((site, cam))
                if v and now - v[0] < FRESH_S:
                    lists[cam] = v[1]
                    if v[2] is not None:
                        frames.append(v[2])
        if not lists:
            return None
        items = [(cam, o) for cam, objs in lists.items() for o in objs if "lat" in o and o.get("state") != "tentative"]
        if not items:
            return self.message(site, now, [], frames)
        lat0, lon0 = items[0][1]["lat"], items[0][1]["lon"]
        pts = [enu(lat0, lon0, o["lat"], o["lon"]) for _, o in items]
        # Greedy association over the closest compatible pairs.
        pairs = []
        for i in range(len(items)):
            for j in range(i + 1, len(items)):
                (ci, a), (cj, b) = items[i], items[j]
                if ci == cj or group_of(a["class"]) != group_of(b["class"]):
                    continue
                d = math.hypot(pts[i][0] - pts[j][0], pts[i][1] - pts[j][1])
                if d > GATE_M:
                    continue
                if a.get("speed", 0) > 1.0 and b.get("speed", 0) > 1.0 and heading_diff(a["heading"], b["heading"]) > GATE_HEADING:
                    continue
                pairs.append((d, i, j))
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
        fused = [self.merge(site, [items[k] for k in ks], now) for ks in groups.values()]
        # Forget associations of tracks not seen for a while.
        self.gid = {k: v for k, v in self.gid.items() if now - v[1] < 10}
        return self.message(site, now, fused, frames)

    def merge(self, site: str, members: list[tuple[str, dict]], now: float) -> dict:
        keys = [(site, cam, int(o["id"])) for cam, o in members]
        known = sorted(self.gid[k][0] for k in keys if k in self.gid)
        if known:
            g = known[0]
        else:
            g = self.next_id.get(site, 1)
            self.next_id[site] = g + 1
        for k in keys:
            self.gid[k] = (g, now)
        best = min(members, key=lambda m: m[1].get("sigma", {}).get("pos", 1.0))[1]
        w = [1.0 / max(o.get("sigma", {}).get("pos", 1.0), 0.05) ** 2 for _, o in members]
        W = sum(w)
        out = dict(best)
        out["id"] = g
        out["lat"] = round(sum(wi * o["lat"] for wi, (_, o) in zip(w, members)) / W, 7)
        out["lon"] = round(sum(wi * o["lon"] for wi, (_, o) in zip(w, members)) / W, 7)
        out["confidence"] = round(max(o.get("confidence", 0) for _, o in members), 2)
        out["cameras"] = sorted(cam for cam, _ in members)
        out["source_ids"] = {cam: o["id"] for cam, o in members}
        return out

    def message(self, site: str, now: float, objects: list[dict], frames: list[int]) -> dict:
        return {"schema": "junction-perception/1.0", "type": "objects", "site": site, "camera": "fused",
                "ts": round(now, 3), "source_frame": max(frames) if frames else None, "objects": objects}
