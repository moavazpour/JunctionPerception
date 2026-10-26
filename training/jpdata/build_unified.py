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
from jpdata.truck_merge import merge_truck_trailer

DIM_RANGES = {  # plausible (min, max) for l, w, h, covering every source's conventions
    "car": ((2.5, 7.0), (1.3, 2.7), (1.0, 3.2)),
    "bus": ((6.0, 19.5), (2.0, 3.7), (2.3, 4.6)),
    "truck": ((1.5, 25.0), (1.6, 3.6), (1.6, 4.6)),  # includes merged tractor + trailer(s)
    "trailer": ((3.0, 20.5), (1.6, 3.5), (1.0, 4.6)),
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


def h01(key: str) -> float:
    return int(hashlib.sha1(key.encode()).hexdigest()[:8], 16) / 0xFFFFFFFF


def split_of(group: str, seed: int, val: float, test: float) -> str:
    h = int(hashlib.sha1(f"{seed}:{group}".encode()).hexdigest()[:8], 16) / 0xFFFFFFFF
    return "test" if h < test else "val" if h < test + val else "train"


def qc(s: dict, stats: Counter, metric: bool = True) -> dict:
    """Drop objects that violate geometric sanity checks; count reasons."""
    plane = np.array(s["plane"])
    keep = []
    ignore = list(s.get("ignore", []))
    for o in s["objects"]:
        if o["cls"] not in G.CLASSES:
            stats["unknown_class"] += 1
            continue
        rng = DIM_RANGES[o["cls"]]
        if metric and not all(lo <= v <= hi for v, (lo, hi) in zip(o["dims"], rng)):
            stats[f"dims_out_of_range:{s['source']}:{o['cls']}"] += 1
            ignore.append(o["box2d"])  # visible but unreliable: neither positive nor negative
            continue
        dz = float(G.plane_height(plane, np.array([o["bottom"]]))[0])
        if metric and abs(dz) > 0.6:
            stats[f"off_ground:{s['source']}"] += 1
            ignore.append(o["box2d"])
            continue
        keep.append(o)
    stats["objects_kept"] += len(keep)
    return {**s, "objects": keep, "ignore": ignore}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", action="append", required=True, help="name=jsonl:image_root")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--val", type=float, default=0.1)
    ap.add_argument("--test", type=float, default=0.1)
    ap.add_argument("--seed", type=int, default=2026)
    ap.add_argument("--subsample", action="append", default=[], help="name=fraction: keep this share of a source (by image)")
    ap.add_argument("--vehicle_side", default="kitti,dair_spd_vehicle", help="sources capped together")
    ap.add_argument("--nonmetric", default="dair_spd_infra,dair_c_infra,dair_spd_vehicle",
                    help="sources whose 3D boxes are approximate (keypoints exact): kept for training, excluded from 3D metrics")
    ap.add_argument("--coverage_sources", default="",
                    help="sources whose labels stop at a sensor range: beyond it the image is ignored")
    ap.add_argument("--vehicle_cap", type=float, default=0.10, help="max share of vehicle-side images in the merged set")
    ap.add_argument("--qc", action="store_true",
                    help="size / ground-plane checks (failures become ignore); off: labels are used as converted")
    a = ap.parse_args()
    out = a.out.expanduser()
    out.mkdir(parents=True, exist_ok=True)
    (out / "splits").mkdir(exist_ok=True)
    writers = {k: open(out / f"{k}.jsonl", "w") for k in ("train", "val", "test")}
    summary = defaultdict(Counter)
    groups = defaultdict(set)
    stats = Counter()
    frac = {k: float(v) for k, v in (x.split("=") for x in a.subsample)}
    nonmetric = set(a.nonmetric.split(",")) if a.nonmetric else set()
    cov_sources = set(a.coverage_sources.split(",")) if a.coverage_sources else set()
    loaded = []
    for spec in a.src:
        name, rest = spec.split("=", 1)
        jl, root = rest.split(":", 1)
        lines = [l for l in Path(jl).expanduser().read_text().splitlines() if l.strip()]
        loaded.append((name, root, lines))
    # Cap vehicle-side cameras at a share of the final set.
    veh = set(a.vehicle_side.split(","))
    n_other = sum(len(l) * frac.get(n, 1.0) for n, _, l in loaded if n not in veh)
    n_veh = sum(len(l) * frac.get(n, 1.0) for n, _, l in loaded if n in veh)
    if n_veh > 0:
        allowed = a.vehicle_cap / (1 - a.vehicle_cap) * n_other
        if n_veh > allowed:
            for n, _, _ in loaded:
                if n in veh:
                    frac[n] = frac.get(n, 1.0) * allowed / n_veh
    print("sampling fractions:", {n: round(frac.get(n, 1.0), 3) for n, _, _ in loaded})
    for name, root, lines in loaded:
        picked = [l for l in lines
                  if h01(f"{a.seed}:pick:{name}:{hashlib.sha1(l[:300].encode()).hexdigest()}") < frac.get(name, 1.0)]
        # Balanced group split per source: groups in a seeded random order fill test, then val, up to
        # their image shares; everything else is train. Groups never straddle splits.
        sizes = Counter(group_of({**json.loads(l), "source": name}) for l in picked)
        coverage = {}
        if name in cov_sources:
            # Labels come from a range-limited sensor: per camera, the range that contains 95 % of labels
            # (conservative: beyond it a large share of visible objects is unlabelled).
            rng = defaultdict(list)
            for l in picked:
                s0 = json.loads(l)
                rng[s0["camera"]] += [float(np.hypot(o["bottom"][0], o["bottom"][2])) for o in s0["objects"]]
            coverage = {c: float(np.percentile(v, 95)) for c, v in rng.items() if len(v) > 50}
            print(f"label coverage {name}:", {c: round(r) for c, r in coverage.items()})
        order = sorted(sizes, key=lambda g: h01(f"{a.seed}:{g}"))
        total = sum(sizes.values())
        assign, acc = {}, 0
        for g in order:
            if acc < a.test * total:
                assign[g] = "test"
            elif acc < (a.test + a.val) * total:
                assign[g] = "val"
            else:
                assign[g] = "train"
            acc += sizes[g]
        for line in picked:
            s = json.loads(line)
            s["source"] = name
            s["_root"] = str(Path(root).expanduser())
            g = group_of(s)
            s["group"] = g
            sp = assign[g]
            if s.get("camera") in coverage:
                s["coverage_range"] = coverage[s["camera"]]
            s["metric3d"] = name not in nonmetric
            s = merge_truck_trailer(s)
            if a.qc:
                s = qc(s, stats, s["metric3d"])
            else:
                s.pop("ignore", None)
                s.pop("ignore_det", None)
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
