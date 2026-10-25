"""Evaluate camera-frame 3D predictions (from jp_bench or the PyTorch model) against a JP3D split.

    python -m tools.eval_predictions --gt ~/data/junction_perception/jp3d/test.jsonl --pred preds.jsonl \
        --out report.json [--by camera|release]
"""
from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path

import numpy as np

from jp3d.evaluate import evaluate, format_results


def load(path: Path) -> list[dict]:
    return [json.loads(l) for l in path.read_text().splitlines() if l.strip()]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gt", type=Path, required=True)
    ap.add_argument("--pred", type=Path, required=True)
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--by", default=None, help="extra breakdown key: camera | release")
    ap.add_argument("--score", type=float, default=0.05)
    args = ap.parse_args()
    gt = {s["image"]: s for s in load(args.gt)}
    preds = {p["image"]: p for p in load(args.pred)}
    groups = defaultdict(list)
    for img, s in gt.items():
        dets = [o for o in preds.get(img, {"objects": []})["objects"] if o["score"] >= args.score]
        frame = (s["objects"], dets, np.array(s["plane"], float))
        groups["all"].append(frame)
        if args.by:
            groups[f"{args.by}={s[args.by]}"].append(frame)
    report = {}
    for k, frames in groups.items():
        res = evaluate(frames)
        report[k] = res
        print(f"===== {k} ({len(frames)} images)")
        print(format_results(res))
    if args.out:
        args.out.write_text(json.dumps(report, indent=1, default=float))


if __name__ == "__main__":
    main()
