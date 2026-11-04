"""Replays recorded camera observations of several cameras through the site world model
(manager/world_model.py) and scores the site states against TUMTraf ground truth.

Inputs per camera: the observation JSONL of jp_run (--world-jsonl), the labels of the same run
(--reference-csv) and the scene folder (for its ground frame). The first camera's scene ground
is the site frame; the others are moved into it with the scenes' sensor_from_ground. Capture
times come from the image file names of the sequence (<seconds>_<nanoseconds>_<camera>.jpg).

Per site state, road users are matched greedily to the labels interpolated at the state's time
(gate 2 m for vehicles, 1 m for VRUs). Reported: matched road users and their position error,
duplicates (a second road user within the gate of a matched label), false positives, global ID
switches per label track, and recall against the labels some camera track matched at that time.

Usage (inside the development image):
  python3 tools/eval_fusion.py --sequence /data/a9_dataset_r02_s03 \
      --camera south1 outputs/fidelity/south1_world.jsonl outputs/fidelity/south1_gt.csv \
          scenes/tumtraf_s110_south1 \
      --camera south2 outputs/fidelity/south2_world.jsonl outputs/fidelity/south2_gt.csv \
          scenes/tumtraf_s110_south2 \
      [--skew-ms 300] [--offset-ms -300] [--out report.json]
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import statistics
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "manager"))
from world_model import NS, ClassSize, FusionSettings, SiteWorldModel  # noqa: E402

VRU_LABELS = {"PEDESTRIAN", "BICYCLE", "MOTORCYCLE"}
GATE_M = {False: 2.0, True: 1.0}
MAX_LABEL_GAP_S = 0.3


def sensor_from_ground(scene: Path) -> list[list[float]]:
    data = yaml.safe_load((scene / "ground_homography.yaml").read_text(encoding="utf-8"))
    return data["sensor_from_ground"]


def rigid_inverse(m: list[list[float]]) -> list[list[float]]:
    r = [[m[j][i] for j in range(3)] for i in range(3)]
    t = [-sum(r[i][k] * m[k][3] for k in range(3)) for i in range(3)]
    return [r[0] + [t[0]], r[1] + [t[1]], r[2] + [t[2]], [0.0, 0.0, 0.0, 1.0]]


def multiply(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


class Frame2D:
    """Planar part of a scene-ground -> site-ground transform."""

    def __init__(self, m: list[list[float]]) -> None:
        self.m = m
        self.yaw = math.atan2(m[1][0], m[0][0])

    def point(self, x: float, y: float) -> tuple[float, float]:
        m = self.m
        return m[0][0] * x + m[0][1] * y + m[0][3], m[1][0] * x + m[1][1] * y + m[1][3]

    def vector(self, x: float, y: float) -> tuple[float, float]:
        m = self.m
        return m[0][0] * x + m[0][1] * y, m[1][0] * x + m[1][1] * y

    def covariance(self, c: list[float]) -> list[float]:
        (a, b), (d, e) = (self.m[0][0], self.m[0][1]), (self.m[1][0], self.m[1][1])
        xx, xy, yy = c
        # R C R^T
        r00 = a * (a * xx + b * xy) + b * (a * xy + b * yy)
        r01 = a * (d * xx + e * xy) + b * (d * xy + e * yy)
        r11 = d * (d * xx + e * xy) + e * (d * xy + e * yy)
        return [r00, r01, r11]


def capture_times(sequence: Path, camera: str) -> list[int]:
    folder = sequence / "images" / f"s110_camera_basler_{camera}_8mm"
    names = sorted(p.name for p in folder.iterdir() if p.suffix == ".jpg")
    return [int(n.split("_")[0]) * NS + int(n.split("_")[1]) for n in names]


def load_camera(camera: str, world: Path, labels: Path, frame: Frame2D, times: list[int],
                skew_ns: int) -> tuple[list[dict], list[tuple[int, int, str, float, float]]]:
    """Observation states in the site frame on capture time (plus a clock error `skew_ns`),
    and the labels (time, track, category, x, y)."""
    states = []
    with world.open(encoding="utf-8") as f:
        for line in f:
            s = json.loads(line)
            s["timestamp_ns"] = times[s["frame"]] + skew_ns
            s["time_source"] = "capture"
            for o in s["objects"]:
                o["x"], o["y"] = frame.point(o["x"], o["y"])
                o["vx"], o["vy"] = frame.vector(o["vx"], o["vy"])
                if o.get("heading_rad") is not None:
                    o["heading_rad"] += frame.yaw
                if o.get("position_cov_m2"):
                    o["position_cov_m2"] = frame.covariance(o["position_cov_m2"])
            states.append(s)
    rows = []
    with labels.open(encoding="utf-8") as f:
        for r in csv.DictReader(f):
            x, y = frame.point(float(r["x"]), float(r["y"]))
            rows.append((times[int(r["frame"])], int(r["track_id"]), r["category"], x, y))
    return states, rows


class Labels:
    """Label tracks of all cameras on one time line, interpolated at any instant."""

    def __init__(self, rows: list[tuple[int, int, str, float, float]]) -> None:
        tracks: dict[int, dict[int, tuple[float, float]]] = {}
        self.category: dict[int, str] = {}
        for t, track, category, x, y in rows:
            tracks.setdefault(track, {})[t] = (x, y)
            self.category[track] = category
        self.tracks = {k: sorted(v.items()) for k, v in tracks.items()}
        self.times = {k: [t for t, _ in v] for k, v in self.tracks.items()}

    def at(self, t: int) -> dict[int, tuple[float, float]]:
        out = {}
        for track, samples in self.tracks.items():
            times = self.times[track]
            i = bisect.bisect_left(times, t)
            if i < len(times) and times[i] == t:
                out[track] = samples[i][1]
            elif 0 < i < len(times) and (times[i] - times[i - 1]) / NS <= MAX_LABEL_GAP_S:
                (t0, (x0, y0)), (t1, (x1, y1)) = samples[i - 1], samples[i]
                w = (t - t0) / (t1 - t0)
                out[track] = (x0 + w * (x1 - x0), y0 + w * (y1 - y0))
        return out

    def vru(self, track: int) -> bool:
        return self.category[track] in VRU_LABELS


def match(users: list[dict], truth: dict[int, tuple[float, float]], labels: Labels):
    """Greedy nearest matching within the gate; returns {user id: label track} and, for the
    unmatched users, the label they are close to (duplicates) or None (false positives)."""
    pairs = []
    for u in users:
        for track, (x, y) in truth.items():
            if labels.vru(track) != u["vru"]:
                continue
            d = math.hypot(u["x"] - x, u["y"] - y)
            if d < GATE_M[u["vru"]]:
                pairs.append((d, u["id"], track))
    matched: dict[int, int] = {}
    used: set[int] = set()
    for d, uid, track in sorted(pairs):
        if uid not in matched and track not in used:
            matched[uid] = track
            used.add(track)
    near = {}
    for d, uid, track in sorted(pairs):
        if uid not in matched and uid not in near:
            near[uid] = track
    return matched, near


def run(states: dict[str, list[dict]], offsets: dict[str, int], fuse: bool,
        class_sizes: dict[str, ClassSize]) -> list[dict]:
    model = SiteWorldModel("eval", offsets, FusionSettings(fuse_cameras=fuse), class_sizes)
    feed = sorted(((s["timestamp_ns"] + offsets[c], c, s) for c, ss in states.items() for s in ss),
                  key=lambda e: (e[0], e[1]))
    out = []
    for _, camera, s in feed:
        model.observe(camera, s)
        site = model.step()
        if site is not None:
            out.append(site)
    return out


def score(steps: list[dict], labels: Labels, observable: dict[int, set[int]] | None = None) -> dict:
    errors: list[float] = []
    users = matched_n = duplicates = false_positives = 0
    observed = recalled = 0
    last_id: dict[int, int] = {}
    switches = 0
    seen: dict[int, set[int]] = {}
    for site in steps:
        truth = labels.at(site["t_ns"])
        matched, near = match(site["road_users"], truth, labels)
        users += len(site["road_users"])
        matched_n += len(matched)
        duplicates += sum(1 for v in near.values() if v in matched.values())
        false_positives += len(site["road_users"]) - len(matched) - sum(
            1 for v in near.values() if v in matched.values())
        by_id = {u["id"]: u for u in site["road_users"]}
        for uid, track in matched.items():
            x, y = truth[track]
            errors.append(math.hypot(by_id[uid]["x"] - x, by_id[uid]["y"] - y))
            if track in last_id and last_id[track] != uid:
                switches += 1
            last_id[track] = uid
        seen[site["seq"]] = set(matched.values())
        if observable is not None:
            wanted = observable.get(site["t_ns"], set())
            observed += len(wanted)
            recalled += len(wanted & set(matched.values()))

    def stat(v: list[float]) -> dict:
        if not v:
            return {"mean": None, "median": None, "p90": None}
        s = sorted(v)
        return {"mean": round(statistics.fmean(s), 3), "median": round(statistics.median(s), 3),
                "p90": round(s[min(len(s) - 1, int(0.9 * len(s)))], 3)}

    tracks = len(last_id)
    duration_min = (steps[-1]["t_ns"] - steps[0]["t_ns"]) / NS / 60 if len(steps) > 1 else 0
    return {
        "steps": len(steps),
        "road_users": users,
        "matched": matched_n,
        "precision": round(matched_n / users, 4) if users else None,
        "duplicates": duplicates,
        "duplicate_rate": round(duplicates / matched_n, 4) if matched_n else None,
        "false_positives": false_positives,
        "position_m": stat(errors),
        "label_tracks_matched": tracks,
        "id_switches": switches,
        "id_switches_per_min": round(switches / duration_min, 2) if duration_min else None,
        "recall_of_observable": round(recalled / observed, 4) if observed else None,
    }


class SeenBy:
    """Labels each single camera matched, by capture time: where the cameras overlap."""

    def __init__(self, singles: dict[str, list[dict]], labels: Labels) -> None:
        self.cameras = {}
        for camera, steps in singles.items():
            times = [s["t_ns"] for s in steps]
            seen = [set(match(s["road_users"], labels.at(s["t_ns"]), labels)[0].values())
                    for s in steps]
            self.cameras[camera] = (times, seen)

    def by_all(self, t: int, tolerance_ns: int = NS // 10) -> set[int]:
        """Labels every camera matched within `tolerance_ns` of `t`."""
        common: set[int] | None = None
        for times, seen in self.cameras.values():
            i = min(max(bisect.bisect_left(times, t), 1), len(times) - 1)
            j = min((i - 1, i), key=lambda k: abs(times[k] - t))
            here = seen[j] if abs(times[j] - t) <= tolerance_ns else set()
            common = here if common is None else common & here
        return common or set()


def overlap(steps: list[dict], labels: Labels, seen_by: SeenBy) -> dict:
    """Labels that every camera sees at once: how many road users represent each (ideal 1,
    counted within twice the match gate) and the error of the nearest one."""
    counts: list[int] = []
    errors: list[float] = []
    for site in steps:
        truth = labels.at(site["t_ns"])
        for track in seen_by.by_all(site["t_ns"]):
            if track not in truth:
                continue
            x, y = truth[track]
            vru = labels.vru(track)
            near = [math.hypot(u["x"] - x, u["y"] - y) for u in site["road_users"]
                    if u["vru"] == vru]
            near = [d for d in near if d < 2 * GATE_M[vru]]
            counts.append(len(near))
            if near:
                errors.append(min(near))
    n = len(counts)
    errors.sort()
    return {
        "label_instants": n,
        "represented_once": round(sum(c == 1 for c in counts) / n, 4) if n else None,
        "represented_twice_or_more": round(sum(c >= 2 for c in counts) / n, 4) if n else None,
        "missed": round(sum(c == 0 for c in counts) / n, 4) if n else None,
        "nearest_error_m_median": round(statistics.median(errors), 3) if errors else None,
        "nearest_error_m_p90": round(errors[int(0.9 * (len(errors) - 1))], 3) if errors else None,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--sequence", type=Path, required=True)
    parser.add_argument("--camera", nargs=4, action="append", required=True,
                        metavar=("ID", "WORLD_JSONL", "LABELS_CSV", "SCENE"))
    parser.add_argument("--skew-ms", type=float, default=0.0,
                        help="clock error added to the last camera's timestamps")
    parser.add_argument("--offset-ms", type=float, default=0.0,
                        help="time offset configured for the last camera (corrects a skew)")
    parser.add_argument("--classes", type=Path,
                        default=Path(__file__).resolve().parent.parent / "configs" / "classes.yaml")
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()

    site_ground = sensor_from_ground(Path(args.camera[0][3]))
    states: dict[str, list[dict]] = {}
    rows: list[tuple[int, int, str, float, float]] = []
    offsets: dict[str, int] = {}
    for i, (camera, world, labels_csv, scene) in enumerate(args.camera):
        last = i == len(args.camera) - 1
        frame = Frame2D(multiply(rigid_inverse(site_ground), sensor_from_ground(Path(scene))))
        times = capture_times(args.sequence, camera)
        states[camera], camera_rows = load_camera(
            camera, Path(world), Path(labels_csv), frame, times,
            round(args.skew_ms * 1e6) if last else 0)
        rows += camera_rows
        offsets[camera] = round(args.offset_ms * 1e6) if last else 0
    labels = Labels(rows)
    classes = yaml.safe_load(args.classes.read_text(encoding="utf-8"))["classes"]
    sizes = {k: ClassSize(v["length_m"], v["width_m"]) for k, v in classes.items()}

    unfused = run(states, offsets, fuse=False, class_sizes=sizes)
    # Labels some camera track matched: what a perfect fusion could at best represent.
    observable: dict[int, set[int]] = {}
    for site in unfused:
        matched, _ = match(site["road_users"], labels.at(site["t_ns"]), labels)
        observable[site["t_ns"]] = set(matched.values())
    fused = run(states, offsets, fuse=True, class_sizes=sizes)
    observable_fused: dict[int, set[int]] = {}
    for site in fused:
        nearest = min(observable, key=lambda t: abs(t - site["t_ns"]))
        observable_fused[site["t_ns"]] = observable[nearest]
    report = {
        "cameras": [c[0] for c in args.camera],
        "skew_ms": args.skew_ms,
        "offset_ms": args.offset_ms,
        "modes": {"fused": fused[-1]["fusion"]["mode"], "baseline": unfused[-1]["fusion"]["mode"]},
        "fused": score(fused, labels, observable_fused),
        "unfused_baseline": score(unfused, labels, observable),
    }
    singles = {c: run({c: states[c]}, {c: offsets[c]}, True, sizes) for c in states}
    for camera, single in singles.items():
        report[f"single_{camera}"] = score(single, labels)
    seen_by = SeenBy(singles, labels)
    report["overlap"] = {
        "fused": overlap(fused, labels, seen_by),
        "unfused_baseline": overlap(unfused, labels, seen_by),
    }
    text = json.dumps(report, indent=2)
    print(text)
    if args.out:
        args.out.write_text(text + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
