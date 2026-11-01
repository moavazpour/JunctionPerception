"""Traffic statistics of the counting lines of all cameras.

Every running camera's world stream is followed (CameraCounter): a road user whose ground track
crosses one of the camera's lines (regions.json, drawn on the camera image and mapped to the
ground with the camera's calibration) is recorded once per line with time, class, direction and
speed. Safety events are recorded when they start. Observation time is kept per camera run, so
flow rates refer to the time a camera actually counted.

report() derives the measures traffic engineers use for design, signal timing and safety
(FHWA Traffic Monitoring Guide, HCM): volumes per 15 minutes and hour, flow rate in vehicles and
passenger car units per hour, peak hour and peak hour factor, heavy-vehicle share, directional
split, spot speeds (time-mean and space-mean speed, 15th/50th/85th/95th percentile, pace, share
above the limit), headways, density estimate, turning movements (origin-destination between the
lines of a camera) and the vulnerable road users and safety events.
"""

from __future__ import annotations

import asyncio
import csv
import io
import json
import math
import sqlite3
import statistics
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

import aiohttp
import yaml

# Passenger car equivalents (HCM: trucks and buses about 2; two-wheelers less than a car).
PCE = {
    "car": 1.0, "van": 1.5, "truck": 2.0, "trailer": 2.5, "bus": 2.0, "emergency_vehicle": 1.5,
    "motorcycle": 0.5, "bicycle": 0.3, "pedestrian": 0.0, "other": 1.0,
}
HEAVY = {"truck", "trailer", "bus"}
VRU = {"pedestrian", "bicycle", "motorcycle"}
SPEED_BIN_KMH = 5
PACE_KMH = 10
SHORT_HEADWAY_S = 2.0
RUN_TOUCH_S = 10.0
RECONNECT_S = 2.0
# A track that touches a line twice within this time (jitter on the line) counts once.
RECROSS_S = 30.0

SCHEMA = """
CREATE TABLE IF NOT EXISTS crossings (
    t REAL NOT NULL, site TEXT NOT NULL, camera TEXT NOT NULL, line TEXT NOT NULL,
    run TEXT NOT NULL, track INTEGER NOT NULL, class TEXT NOT NULL, vru INTEGER NOT NULL,
    direction INTEGER NOT NULL, speed_mps REAL NOT NULL, video_s REAL
);
CREATE INDEX IF NOT EXISTS crossings_line ON crossings (site, camera, line, t);
CREATE TABLE IF NOT EXISTS events (
    t REAL NOT NULL, site TEXT NOT NULL, camera TEXT NOT NULL, run TEXT NOT NULL,
    event INTEGER NOT NULL, type TEXT NOT NULL, zone TEXT, min_distance_m REAL,
    time_to_closest_s REAL
);
CREATE INDEX IF NOT EXISTS events_site ON events (site, t);
CREATE TABLE IF NOT EXISTS runs (
    run TEXT PRIMARY KEY, site TEXT NOT NULL, camera TEXT NOT NULL,
    start REAL NOT NULL, end REAL NOT NULL, lines TEXT NOT NULL
);
"""


# ----------------------------------------------------------------------------------- geometry


def apply_homography(h: list[list[float]], u: float, v: float) -> tuple[float, float] | None:
    w = h[2][0] * u + h[2][1] * v + h[2][2]
    if abs(w) < 1e-12:
        return None
    return ((h[0][0] * u + h[0][1] * v + h[0][2]) / w, (h[1][0] * u + h[1][1] * v + h[1][2]) / w)


def cross(ax: float, ay: float, bx: float, by: float) -> float:
    return ax * by - ay * bx


def crossing_direction(
    p: tuple[float, float], q: tuple[float, float], line: list[tuple[float, float]]
) -> int:
    """+1 if the move p -> q crosses the polyline from its left to its right side (as drawn),
    -1 for right to left, 0 if it does not cross."""
    for a, b in zip(line, line[1:]):
        d = (b[0] - a[0], b[1] - a[1])
        m = (q[0] - p[0], q[1] - p[1])
        denominator = cross(m[0], m[1], d[0], d[1])
        if abs(denominator) < 1e-12:
            continue
        ap = (a[0] - p[0], a[1] - p[1])
        t = cross(ap[0], ap[1], d[0], d[1]) / denominator  # along the move
        s = cross(ap[0], ap[1], m[0], m[1]) / denominator  # along the segment
        if 0.0 <= t <= 1.0 and 0.0 <= s <= 1.0:
            # Ground frame east-north-up: left of the line direction has positive cross product.
            side_before = cross(d[0], d[1], p[0] - a[0], p[1] - a[1])
            return 1 if side_before > 0 else -1
    return 0


@dataclass
class GroundLine:
    id: str
    name: str
    points: list[tuple[float, float]]
    speed_limit_kmh: float


def load_lines(camera_dir: Path) -> list[GroundLine]:
    """Lines of a camera on the ground; empty without regions or calibration."""
    regions_path = camera_dir / "regions.json"
    calibration_path = camera_dir / "ground_homography.yaml"
    if not regions_path.is_file() or not calibration_path.is_file():
        return []
    regions = json.loads(regions_path.read_text(encoding="utf-8"))
    h = (yaml.safe_load(calibration_path.read_text(encoding="utf-8")) or {}).get("image_to_ground")
    if not h:
        return []
    lines = []
    for line in regions.get("lines", []):
        ground = [apply_homography(h, float(u), float(v)) for u, v in line.get("points", [])]
        if len(ground) >= 2 and all(g is not None for g in ground):
            lines.append(GroundLine(
                id=str(line["id"]),
                name=str(line.get("name") or line["id"]),
                points=[g for g in ground if g is not None],
                speed_limit_kmh=float(line.get("speed_limit_kmh") or 0.0),
            ))
    return lines


# ------------------------------------------------------------------------------------ storage


class TrafficStats:
    """SQLite store of crossings, events and observation runs."""

    def __init__(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(str(path), check_same_thread=False)
        self.db.executescript(SCHEMA)
        self.lock = threading.Lock()

    def record_crossing(self, row: tuple[Any, ...]) -> None:
        with self.lock:
            self.db.execute("INSERT INTO crossings VALUES (?,?,?,?,?,?,?,?,?,?,?)", row)
            self.db.commit()

    def record_event(self, row: tuple[Any, ...]) -> None:
        with self.lock:
            self.db.execute("INSERT INTO events VALUES (?,?,?,?,?,?,?,?,?)", row)
            self.db.commit()

    def touch_run(self, run: str, site: str, camera: str, start: float, end: float,
                  lines: list[GroundLine]) -> None:
        names = json.dumps({line.id: [line.name, line.speed_limit_kmh] for line in lines})
        with self.lock:
            self.db.execute(
                "INSERT INTO runs VALUES (?,?,?,?,?,?) ON CONFLICT(run) DO UPDATE SET "
                "end = excluded.end, lines = excluded.lines",
                (run, site, camera, start, end, names),
            )
            self.db.commit()

    def clear(self, site: str | None) -> None:
        with self.lock:
            for table in ("crossings", "events", "runs"):
                if site:
                    self.db.execute(f"DELETE FROM {table} WHERE site = ?", (site,))  # noqa: S608
                else:
                    self.db.execute(f"DELETE FROM {table}")  # noqa: S608
            self.db.commit()

    def _query(self, sql: str, args: tuple[Any, ...]) -> list[tuple[Any, ...]]:
        with self.lock:
            return self.db.execute(sql, args).fetchall()

    @staticmethod
    def _where(filters: dict[str, Any], time_column: str = "t") -> tuple[str, tuple[Any, ...]]:
        clauses, args = [], []
        for key in ("site", "camera", "line"):
            if filters.get(key):
                clauses.append(f"{key} = ?")
                args.append(filters[key])
        if filters.get("from") is not None:
            clauses.append(f"{time_column} >= ?")
            args.append(float(filters["from"]))
        if filters.get("to") is not None:
            clauses.append(f"{time_column} < ?")
            args.append(float(filters["to"]))
        return (" WHERE " + " AND ".join(clauses)) if clauses else "", tuple(args)

    def crossings(self, filters: dict[str, Any]) -> list[dict[str, Any]]:
        where, args = self._where(filters)
        rows = self._query(
            "SELECT t, site, camera, line, run, track, class, vru, direction, speed_mps, video_s "
            f"FROM crossings{where} ORDER BY t",  # noqa: S608
            args,
        )
        keys = ("t", "site", "camera", "line", "run", "track", "class", "vru", "direction",
                "speed_mps", "video_s")
        return [dict(zip(keys, row)) for row in rows]

    def events(self, filters: dict[str, Any]) -> list[dict[str, Any]]:
        where, args = self._where({k: v for k, v in filters.items() if k != "line"})
        rows = self._query(
            "SELECT t, site, camera, type, zone, min_distance_m, time_to_closest_s "
            f"FROM events{where} ORDER BY t",  # noqa: S608
            args,
        )
        keys = ("t", "site", "camera", "type", "zone", "min_distance_m", "time_to_closest_s")
        return [dict(zip(keys, row)) for row in rows]

    def runs(self, filters: dict[str, Any]) -> list[dict[str, Any]]:
        clauses, args = [], []
        for key in ("site", "camera"):
            if filters.get(key):
                clauses.append(f"{key} = ?")
                args.append(filters[key])
        if filters.get("from") is not None:
            clauses.append("end >= ?")
            args.append(float(filters["from"]))
        if filters.get("to") is not None:
            clauses.append("start < ?")
            args.append(float(filters["to"]))
        where = (" WHERE " + " AND ".join(clauses)) if clauses else ""
        rows = self._query(
            f"SELECT run, site, camera, start, end, lines FROM runs{where}",  # noqa: S608
            tuple(args),
        )
        return [
            {"run": r[0], "site": r[1], "camera": r[2], "start": r[3], "end": r[4],
             "lines": json.loads(r[5])}
            for r in rows
        ]

    def export_csv(self, filters: dict[str, Any]) -> str:
        out = io.StringIO()
        writer = csv.writer(out)
        writer.writerow(["time_utc", "site", "camera", "line", "track", "class", "direction",
                         "speed_kmh", "video_s"])
        for c in self.crossings(filters):
            writer.writerow([
                time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(c["t"])) + f".{int(c['t'] % 1 * 1000):03d}Z",
                c["site"], c["camera"], c["line"], c["track"], c["class"],
                "A" if c["direction"] > 0 else "B", round(c["speed_mps"] * 3.6, 1),
                "" if c["video_s"] is None else round(c["video_s"], 2),
            ])
        return out.getvalue()

    # -------------------------------------------------------------------------- report

    def report(self, filters: dict[str, Any], bucket_s: int = 900) -> dict[str, Any]:
        crossings = self.crossings(filters)
        runs = self.runs(filters)
        lo = float(filters["from"]) if filters.get("from") is not None else None
        hi = float(filters["to"]) if filters.get("to") is not None else None

        def observed(site: str, camera: str) -> float:
            total = 0.0
            for r in runs:
                if r["site"] == site and r["camera"] == camera:
                    start = max(r["start"], lo) if lo is not None else r["start"]
                    end = min(r["end"], hi) if hi is not None else r["end"]
                    total += max(0.0, end - start)
            return total

        names: dict[tuple[str, str, str], tuple[str, float]] = {}
        for r in runs:
            for line_id, (name, limit) in r["lines"].items():
                names[(r["site"], r["camera"], line_id)] = (name, float(limit))
        groups: dict[tuple[str, str, str], list[dict[str, Any]]] = {}
        for c in crossings:
            groups.setdefault((c["site"], c["camera"], c["line"]), []).append(c)
        for key in names:
            if filters.get("line") and key[2] != filters["line"]:
                continue
            groups.setdefault(key, [])

        lines = []
        for (site, camera, line), rows in sorted(groups.items()):
            name, limit = names.get((site, camera, line), (line, 0.0))
            lines.append({
                "site": site, "camera": camera, "line": line, "name": name,
                "speed_limit_kmh": limit or None,
                "observed_s": observed(site, camera),
                **measures(rows, observed(site, camera), limit, bucket_s),
            })
        all_vehicle_rows = [c for c in crossings if c["class"] not in VRU or c["class"] == "motorcycle"]
        return {
            "generated_at": time.time(),
            "bucket_s": bucket_s,
            "lines": lines,
            "series": series(crossings, bucket_s),
            "hourly": hourly(crossings),
            "speed_histogram": speed_histogram(all_vehicle_rows),
            "turning": turning_movements(crossings, names),
            "events": event_summary(self.events(filters)),
            "observed_s": sum(observed(r["site"], r["camera"]) for r in
                              {(r["site"], r["camera"]): r for r in runs}.values()),
            "totals": totals(crossings),
        }


# ----------------------------------------------------------------------------------- measures


def percentile(sorted_values: list[float], p: float) -> float | None:
    if not sorted_values:
        return None
    k = (len(sorted_values) - 1) * p
    f, c = math.floor(k), math.ceil(k)
    if f == c:
        return sorted_values[int(k)]
    return sorted_values[f] + (sorted_values[c] - sorted_values[f]) * (k - f)


def rounded(value: float | None, digits: int = 1) -> float | None:
    return None if value is None else round(value, digits)


def speed_measures(speeds_kmh: list[float], limit_kmh: float) -> dict[str, Any]:
    s = sorted(v for v in speeds_kmh if v > 0.5)
    if not s:
        return {"n": 0}
    pace_start, pace_count = 0.0, 0
    for start in range(0, int(s[-1]) + 1):
        count = sum(1 for v in s if start <= v < start + PACE_KMH)
        if count > pace_count:
            pace_start, pace_count = float(start), count
    return {
        "n": len(s),
        "mean": rounded(statistics.fmean(s)),
        "space_mean": rounded(len(s) / sum(1 / v for v in s)),
        "std": rounded(statistics.pstdev(s)),
        "min": rounded(s[0]),
        "max": rounded(s[-1]),
        "p15": rounded(percentile(s, 0.15)),
        "p50": rounded(percentile(s, 0.50)),
        "p85": rounded(percentile(s, 0.85)),
        "p95": rounded(percentile(s, 0.95)),
        "pace_kmh": [pace_start, pace_start + PACE_KMH],
        "pace_share": rounded(pace_count / len(s), 3),
        "over_limit_share": rounded(sum(1 for v in s if v > limit_kmh) / len(s), 3)
        if limit_kmh > 0 else None,
    }


def headway_measures(times: list[float]) -> dict[str, Any]:
    t = sorted(times)
    gaps = sorted(b - a for a, b in zip(t, t[1:]) if b > a)
    if not gaps:
        return {"n": 0}
    return {
        "n": len(gaps),
        "mean_s": rounded(statistics.fmean(gaps), 2),
        "min_s": rounded(gaps[0], 2),
        "p15_s": rounded(percentile(gaps, 0.15), 2),
        "p50_s": rounded(percentile(gaps, 0.50), 2),
        "short_share": rounded(sum(1 for g in gaps if g < SHORT_HEADWAY_S) / len(gaps), 3),
    }


def peak_hour(rows: list[dict[str, Any]]) -> dict[str, Any] | None:
    """Busiest 60 minutes in 15-minute steps and its peak hour factor V / (4 · V15max)."""
    if not rows:
        return None
    quarter = 900.0
    bins: dict[int, int] = {}
    for c in rows:
        bins[int(c["t"] // quarter)] = bins.get(int(c["t"] // quarter), 0) + 1
    best = None
    for start in bins:
        counts = [bins.get(start + i, 0) for i in range(4)]
        volume = sum(counts)
        if best is None or volume > best[1]:
            best = (start, volume, max(counts))
    assert best is not None
    start, volume, top = best
    return {
        "start": start * quarter,
        "volume": volume,
        "peak_15min": top,
        "phf": rounded(volume / (4 * top), 2) if top else None,
    }


def measures(rows: list[dict[str, Any]], observed_s: float, limit_kmh: float,
             bucket_s: int) -> dict[str, Any]:
    vehicles = [c for c in rows if c["class"] not in ("pedestrian", "bicycle")]
    by_class: dict[str, int] = {}
    for c in rows:
        by_class[c["class"]] = by_class.get(c["class"], 0) + 1
    hours = observed_s / 3600.0
    pcu = sum(PCE.get(c["class"], 1.0) for c in vehicles)
    speeds = [c["speed_mps"] * 3.6 for c in vehicles]
    speed = speed_measures(speeds, limit_kmh)
    flow = len(vehicles) / hours if hours > 0 else None
    density = flow / speed["space_mean"] if flow and speed.get("space_mean") else None
    directions = {
        "A": sum(1 for c in rows if c["direction"] > 0),
        "B": sum(1 for c in rows if c["direction"] < 0),
    }
    return {
        "count": len(rows),
        "vehicles": len(vehicles),
        "vru": sum(1 for c in rows if c["class"] in VRU),
        "by_class": by_class,
        "directions": directions,
        "flow_vph": rounded(flow),
        "flow_pcuph": rounded(pcu / hours) if hours > 0 else None,
        "heavy_share": rounded(sum(1 for c in vehicles if c["class"] in HEAVY) / len(vehicles), 3)
        if vehicles else None,
        # A peak hour needs an hour of observation.
        "peak_hour": peak_hour(vehicles) if observed_s >= 3600 else None,
        "speed_kmh": speed,
        "speed_by_direction": {
            d: speed_measures([c["speed_mps"] * 3.6 for c in vehicles
                               if (c["direction"] > 0) == (d == "A")], limit_kmh)
            for d in ("A", "B")
        },
        "headway": {
            d: headway_measures([c["t"] for c in vehicles if (c["direction"] > 0) == (d == "A")])
            for d in ("A", "B")
        },
        "density_vpkm": rounded(density),
        "series": series(rows, bucket_s),
    }


def series(rows: list[dict[str, Any]], bucket_s: int) -> list[dict[str, Any]]:
    """Counts per class, vehicle mean and 85th percentile speed per time bucket."""
    buckets: dict[int, list[dict[str, Any]]] = {}
    for c in rows:
        buckets.setdefault(int(c["t"] // bucket_s), []).append(c)
    out = []
    for index in sorted(buckets):
        group = buckets[index]
        speeds = sorted(c["speed_mps"] * 3.6 for c in group
                        if c["class"] not in ("pedestrian", "bicycle") and c["speed_mps"] > 0.15)
        by_class: dict[str, int] = {}
        for c in group:
            by_class[c["class"]] = by_class.get(c["class"], 0) + 1
        out.append({
            "t": index * bucket_s,
            "count": len(group),
            "by_class": by_class,
            "mean_kmh": rounded(statistics.fmean(speeds)) if speeds else None,
            "p85_kmh": rounded(percentile(speeds, 0.85)),
        })
    return out


def hourly(rows: list[dict[str, Any]]) -> list[int]:
    """Crossings per hour of the day (local time of the server)."""
    counts = [0] * 24
    for c in rows:
        counts[time.localtime(c["t"]).tm_hour] += 1
    return counts


def speed_histogram(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    bins: dict[int, int] = {}
    for c in rows:
        kmh = c["speed_mps"] * 3.6
        if kmh > 0.5:
            bins[int(kmh // SPEED_BIN_KMH)] = bins.get(int(kmh // SPEED_BIN_KMH), 0) + 1
    if not bins:
        return []
    return [{"from": b * SPEED_BIN_KMH, "count": bins.get(b, 0)}
            for b in range(min(bins), max(bins) + 1)]


def turning_movements(rows: list[dict[str, Any]],
                      names: dict[tuple[str, str, str], tuple[str, float]]) -> list[dict[str, Any]]:
    """Origin-destination of the tracks that crossed two or more lines of one camera: from the
    first line they crossed to the last."""
    paths: dict[tuple[str, str, str, int], list[dict[str, Any]]] = {}
    for c in rows:
        paths.setdefault((c["site"], c["camera"], c["run"], c["track"]), []).append(c)
    movements: dict[tuple[str, str, str, str], dict[str, Any]] = {}
    for (site, camera, _, _), path in paths.items():
        if len(path) < 2:
            continue
        path.sort(key=lambda c: c["t"])
        origin, destination = path[0]["line"], path[-1]["line"]
        if origin == destination:
            continue
        key = (site, camera, origin, destination)
        entry = movements.setdefault(key, {
            "site": site, "camera": camera,
            "from": origin, "from_name": names.get((site, camera, origin), (origin, 0))[0],
            "to": destination, "to_name": names.get((site, camera, destination), (destination, 0))[0],
            "count": 0, "by_class": {},
        })
        entry["count"] += 1
        cls = path[0]["class"]
        entry["by_class"][cls] = entry["by_class"].get(cls, 0) + 1
    return sorted(movements.values(), key=lambda m: -m["count"])


def event_summary(events: list[dict[str, Any]]) -> dict[str, Any]:
    by_type: dict[str, int] = {}
    for e in events:
        by_type[e["type"]] = by_type.get(e["type"], 0) + 1
    ttc = sorted(e["time_to_closest_s"] for e in events if e["time_to_closest_s"] is not None)
    gaps = sorted(e["min_distance_m"] for e in events if e["min_distance_m"] is not None)
    return {
        "count": len(events),
        "by_type": by_type,
        "min_ttc_s": rounded(ttc[0], 2) if ttc else None,
        "median_ttc_s": rounded(percentile(ttc, 0.5), 2) if ttc else None,
        "min_gap_m": rounded(gaps[0], 2) if gaps else None,
        "recent": events[-20:],
    }


def totals(rows: list[dict[str, Any]]) -> dict[str, Any]:
    by_class: dict[str, int] = {}
    for c in rows:
        by_class[c["class"]] = by_class.get(c["class"], 0) + 1
    return {"count": len(rows), "by_class": by_class}


# ----------------------------------------------------------------------------------- counting


class CameraCounter:
    """Follows one camera's world stream and records crossings and events."""

    def __init__(self, stats: TrafficStats, site: str, camera: str, camera_dir: Path,
                 port: int, log: Callable[[str, str], None]) -> None:
        self.stats = stats
        self.site = site
        self.camera = camera
        self.dir = camera_dir
        self.port = port
        self.log = log
        self.lines: list[GroundLine] = []
        self.lines_mtime = 0.0
        self.run = ""
        self.run_start = 0.0
        self.last_touch = 0.0
        self.last_frame = -1
        self.positions: dict[int, tuple[float, float]] = {}
        self.counted: dict[tuple[int, str], float] = {}
        self.events_seen: set[int] = set()

    def reload_lines(self) -> None:
        paths = [self.dir / "regions.json", self.dir / "ground_homography.yaml"]
        mtime = max((p.stat().st_mtime for p in paths if p.is_file()), default=0.0)
        if mtime != self.lines_mtime:
            self.lines_mtime = mtime
            try:
                self.lines = load_lines(self.dir)
            except (OSError, ValueError, KeyError, yaml.YAMLError) as error:
                self.lines = []
                self.log("warning", f"counting lines not readable: {error}")
            if self.lines:
                self.log("info", f"counting on {len(self.lines)} lines")

    def new_run(self) -> None:
        self.run_start = time.time()
        self.run = f"{self.site}/{self.camera}/{self.run_start:.3f}"
        self.positions.clear()
        self.counted.clear()
        self.events_seen.clear()

    def process(self, state: dict[str, Any]) -> None:
        now = time.time()
        frame = int(state.get("frame", 0))
        if frame < self.last_frame or not self.run:
            # Started, seeked or looped: track ids may repeat.
            self.new_run()
        self.last_frame = frame
        if now - self.last_touch > RUN_TOUCH_S:
            self.reload_lines()
            self.last_touch = now
            if self.lines:
                self.stats.touch_run(self.run, self.site, self.camera, self.run_start, now,
                                     self.lines)
        video_s = float(state.get("timestamp_ns", 0)) / 1e9
        present = set()
        for o in state.get("objects", []):
            track = int(o["id"])
            present.add(track)
            here = (float(o["x"]), float(o["y"]))
            before = self.positions.get(track)
            self.positions[track] = here
            if before is None or not self.lines:
                continue
            for line in self.lines:
                direction = crossing_direction(before, here, line.points)
                if direction == 0:
                    continue
                key = (track, line.id)
                if now - self.counted.get(key, -1e9) < RECROSS_S:
                    continue
                self.counted[key] = now
                self.stats.record_crossing((
                    now, self.site, self.camera, line.id, self.run, track, str(o["class"]),
                    int(bool(o.get("vru"))), direction, float(o.get("speed_mps", 0.0)), video_s,
                ))
        for track in [t for t in self.positions if t not in present]:
            del self.positions[track]
        for e in state.get("events", []):
            event_id = int(e["id"])
            if e.get("state") == "active" and event_id not in self.events_seen:
                self.events_seen.add(event_id)
                self.stats.record_event((
                    now, self.site, self.camera, self.run, event_id, str(e["type"]),
                    e.get("zone"), e.get("min_distance_m"), e.get("time_to_closest_s"),
                ))

    async def follow(self, session: aiohttp.ClientSession, running: Callable[[], bool]) -> None:
        """Reads the world stream while the camera runs; reconnects after drops."""
        self.reload_lines()
        while running():
            try:
                async with session.ws_connect(f"ws://127.0.0.1:{self.port}/ws/world",
                                              max_msg_size=0, heartbeat=None) as ws:
                    async for message in ws:
                        if message.type != aiohttp.WSMsgType.TEXT:
                            break
                        try:
                            self.process(json.loads(message.data))
                        except (ValueError, KeyError, TypeError) as error:
                            self.log("warning", f"statistics: bad world state ({error})")
            except (aiohttp.ClientError, ConnectionResetError, asyncio.TimeoutError):
                pass
            await asyncio.sleep(RECONNECT_S)
        if self.run and self.lines:
            self.stats.touch_run(self.run, self.site, self.camera, self.run_start, time.time(),
                                 self.lines)
