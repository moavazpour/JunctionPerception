"""Merge converted sources into JP-Unified, run QC, split 80/10/10 by group and export YOLO pose data.

    python -m jpdata.build_unified --out ~/data/junction_perception/unified \
        --src tumtraf_i=~/data/junction_perception/jp3d/all.jsonl:~/data/junction_perception/datasets/TUMTraf \
        --src carla=~/data/junction_perception/carla/all.jsonl:~/data/junction_perception/carla \
        --src kitti=~/.../kitti.jsonl:~/.../KITTI/object
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

from jp3d import geometry as G

DIM_RANGES = {  # plausible (min, max) for l, w, h
    "car": ((2.5, 7.0), (1.3, 2.6), (1.0, 3.2)),
    "bus": ((6.0, 25.0), (2.0, 3.2), (2.3, 4.5)),
    "truck": ((3.5, 13.0), (1.6, 3.2), (1.8, 4.5)),
    "trailer": ((3.0, 18.0), (1.6, 3.2), (1.0, 4.5)),
    "pedestrian": ((0.2, 1.5), (0.2, 1.5), (0.9, 2.3)),
    "bicycle": ((0.8, 2.6), (0.2, 1.5), (0.9, 2.3)),
    "escooter": ((0.6, 1.8), (0.2, 1.2), (1.0, 2.3)),
    "motorcycle": ((1.2, 3.2), (0.4, 1.6), (0.9, 2.3)),
}


def group_of(s: dict) -> str:
    if "group" in s:
        return s["group"]
    src = s.get("source") or s.get("release", "unknown")
    name = Path(s["image"]).name
    if s.get("release") == "CARLA" or src == "carla":
        m = re.match(r"(.+_\d{3})_\d{3}\.jpg", name)  # one camera placement
        return f"carla:{m.group(1) if m else name}"
    m = re.match(r"(\d+)_", name)
    t = int(m.group(1)) if m else 0
    return f"{src}:{s.get('camera')}:{t // 30}"


def split_of(group: str, seed: int, val: float, test: float) -> str:
    h = int(hashlib.sha1(f"{seed}:{group}".encode()).hexdigest()[:8], 16) / 0xFFFFFFFF
    return "test" if h < test else "val" if h < test + val else "train"


def qc(s: dict, stats: Counter) -> dict:
    """Drop objects that violate geometric sanity checks; count reasons."""
    plane = np.array(s["plane"])
    keep = []
    for o in s["objects"]:
        if o["cls"] not in G.CLASSES:
            stats["unknown_class"] += 1
            continue
        rng = DIM_RANGES[o["cls"]]
        if not all(lo <= v <= hi for v, (lo, hi) in zip(o["dims"], rng)):
            stats[f"dims_out_of_range:{o['cls']}"] += 1
            continue
        dz = float(G.plane_height(plane, np.array([o["bottom"]]))[0])
        if abs(dz) > 0.6:
            stats["off_ground"] += 1
            continue
        keep.append(o)
    stats["objects_kept"] += len(keep)
    return {**s, "objects": keep}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", action="append", required=True, help="name=jsonl:image_root")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--val", type=float, default=0.1)
    ap.add_argument("--test", type=float, default=0.1)
    ap.add_argument("--seed", type=int, default=2026)
    a = ap.parse_args()
    out = a.out.expanduser()
    out.mkdir(parents=True, exist_ok=True)
    (out / "splits").mkdir(exist_ok=True)
    writers = {k: open(out / f"{k}.jsonl", "w") for k in ("train", "val", "test")}
    summary = defaultdict(Counter)
    groups = defaultdict(set)
    stats = Counter()
    for spec in a.src:
        name, rest = spec.split("=", 1)
        jl, root = rest.split(":", 1)
        for line in Path(jl).expanduser().read_text().splitlines():
            if not line.strip():
                continue
            s = json.loads(line)
            s["source"] = name
            s["_root"] = str(Path(root).expanduser())
            g = group_of(s)
            s["group"] = g
            sp = split_of(g, a.seed, a.val, a.test)
            s = qc(s, stats)
            writers[sp].write(json.dumps(s) + "\n")
            groups[sp].add(g)
            summary[(name, sp)]["images"] += 1
            for o in s["objects"]:
                summary[(name, sp)][o["cls"]] += 1
    for w in writers.values():
        w.close()
    for sp, gs in groups.items():
        (out / "splits" / f"{sp}_groups.txt").write_text("\n".join(sorted(gs)) + "\n")
    report = {f"{k[0]}/{k[1]}": dict(v) for k, v in sorted(summary.items())}
    report["_qc"] = dict(stats)
    (out / "summary.json").write_text(json.dumps(report, indent=1))
    for k, v in report.items():
        print(k, v)


if __name__ == "__main__":
    main()
