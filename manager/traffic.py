"""Lane traffic of a site: queue and passing vehicles of each lane, measured on the site state.

A lane is a line with three or more points that the user drew in a camera image (regions.json),
projected into the site frame with that camera's homography, with the direction of travel the
user set. Measures use the world model's global road-user IDs, so a vehicle seen by several
cameras is counted once per lane. Lanes that two cameras both outline are separate lanes.

Pure: no I/O. The site feed (site_feed.py) adds the result to the site state as "traffic".
"""

from __future__ import annotations

import math
from collections import Counter
from dataclasses import dataclass, field, fields
from typing import Any

Point = tuple[float, float]


@dataclass(frozen=True)
class TrafficSettings:
    """Thresholds of the lane measures (configs/manager.yaml, traffic:)."""

    # A vehicle in a lane slower than this is queued.
    queue_speed_mps: float = 1.5
    # Travel along the lane direction, from where a vehicle entered the lane to where it left,
    # for it to count as passing. Shorter or backwards movements are not counted.
    min_travel_m: float = 3.0

    @staticmethod
    def from_dict(raw: dict[str, Any]) -> TrafficSettings:
        known = {f.name for f in fields(TrafficSettings)}
        unknown = set(raw) - known
        if unknown:
            raise ValueError(f"unknown traffic settings: {', '.join(sorted(unknown))}")
        return TrafficSettings(**{k: float(v) for k, v in raw.items()})


@dataclass(frozen=True)
class Lane:
    id: str  # "<camera>/<line id>"
    camera: str
    name: str
    polygon: tuple[Point, ...]  # site frame (m)
    direction: Point | None  # unit vector in the site frame; None while the user has not set it


def ground_homography(h: list[list[float]], size: tuple[float, float]) -> list[list[float]]:
    """The homography scaled so that ground points have a positive w (a fit normalised to
    h33 = 1 is negative on the ground when the image corner (0, 0) lies above the horizon). The
    bottom centre of the image is on the ground. Same rule as web/src/bev/geometry.ts."""
    w = h[2][0] * size[0] / 2 + h[2][1] * size[1] + h[2][2]
    return [[-v for v in row] for row in h] if w < 0 else h


def to_ground(h: list[list[float]], u: float, v: float) -> Point | None:
    """Pixel to site frame with an oriented homography; None at or behind the horizon."""
    w = h[2][0] * u + h[2][1] * v + h[2][2]
    if w <= 1e-9:
        return None
    return ((h[0][0] * u + h[0][1] * v + h[0][2]) / w, (h[1][0] * u + h[1][1] * v + h[1][2]) / w)


def lanes_from_regions(camera: str, regions: dict[str, Any], image_to_ground: list[list[float]],
                       image_size: tuple[float, float]) -> list[Lane]:
    """The lanes of one camera: its lines with three or more points that lie on the ground."""
    h = ground_homography(image_to_ground, image_size)
    lanes = []
    for line in regions.get("lines", []):
        pixels = [(float(u), float(v)) for u, v in line["points"]]
        if len(pixels) > 1 and pixels[0] == pixels[-1]:
            pixels.pop()  # drawn closed
        polygon = [to_ground(h, u, v) for u, v in pixels]
        if len(polygon) < 3 or any(p is None for p in polygon):
            continue
        direction = None
        if line.get("direction"):
            tail, head = (to_ground(h, float(u), float(v)) for u, v in line["direction"])
            if tail is not None and head is not None:
                dx, dy = head[0] - tail[0], head[1] - tail[1]
                length = math.hypot(dx, dy)
                if length > 1e-6:
                    direction = (dx / length, dy / length)
        lanes.append(Lane(f"{camera}/{line['id']}", camera, line.get("name") or line["id"],
                          tuple(p for p in polygon if p is not None), direction))
    return lanes


def inside(polygon: tuple[Point, ...], x: float, y: float) -> bool:
    """Even-odd point in polygon."""
    result = False
    j = len(polygon) - 1
    for i, (xi, yi) in enumerate(polygon):
        xj, yj = polygon[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            result = not result
        j = i
    return result


@dataclass
class _Visit:
    cls: str
    entry_s: float  # position along the lane direction where it entered (m)
    last_s: float


@dataclass
class _LaneCount:
    visits: dict[int, _Visit] = field(default_factory=dict)
    counted: set[int] = field(default_factory=set)  # passed already (by global ID)
    passed: Counter[str] = field(default_factory=Counter)


class LaneTraffic:
    """Queue and passing vehicles of the lanes of one site, from its stream of site states. The
    counts start over with each run of the world model (seek, loop, restart: IDs start over)."""

    def __init__(self, lanes: list[Lane], settings: TrafficSettings | None = None) -> None:
        self.lanes = list(lanes)
        self.settings = settings or TrafficSettings()
        self.run: int | None = None
        self.since_ns = 0
        self.counts = {lane.id: _LaneCount() for lane in self.lanes}

    def update(self, state: dict[str, Any]) -> dict[str, Any]:
        """Advances with one site state; returns its "traffic" block."""
        if state["run"] != self.run:
            self.run = state["run"]
            self.since_ns = state["t_ns"]
            self.counts = {lane.id: _LaneCount() for lane in self.lanes}
        vehicles = [u for u in state["road_users"] if not u["vru"]]
        present = {u["id"] for u in vehicles}
        lanes = []
        for lane in self.lanes:
            count = self.counts[lane.id]
            here = [u for u in vehicles if inside(lane.polygon, u["x"], u["y"])]
            for user in here:
                s = self._along(lane, user)
                visit = count.visits.get(user["id"])
                if visit is None:
                    count.visits[user["id"]] = _Visit(user["class"], s, s)
                else:
                    visit.last_s = s
            # Left the lane, or the site: a pass if it travelled far enough along the lane.
            for user_id in [i for i in count.visits if i not in {u["id"] for u in here}]:
                visit = count.visits.pop(user_id)
                if (lane.direction is not None and user_id not in count.counted
                        and visit.last_s - visit.entry_s >= self.settings.min_travel_m):
                    count.counted.add(user_id)
                    count.passed[visit.cls] += 1
            count.counted &= present
            lanes.append({
                "id": lane.id,
                "camera": lane.camera,
                "name": lane.name,
                "direction_set": lane.direction is not None,
                "vehicles": len(here),
                "queue": sum(u["speed_mps"] < self.settings.queue_speed_mps for u in here),
                "passed": sum(count.passed.values()),
                "passed_by_class": dict(sorted(count.passed.items())),
            })
        return {
            "since_ns": self.since_ns,
            "present_by_class": dict(sorted(Counter(u["class"] for u in state["road_users"]).items())),
            "lanes": lanes,
        }

    @staticmethod
    def _along(lane: Lane, user: dict[str, Any]) -> float:
        if lane.direction is None:
            return 0.0
        return user["x"] * lane.direction[0] + user["y"] * lane.direction[1]
