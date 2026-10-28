"""Benchmark protocol splits. TUMTraf withholds test labels, so the official val split is the benchmark
("bench") and model selection uses a held-out block of the training data ("sel"), split by 30 s
time blocks so neighbouring frames never straddle the two sets.

    python -m tools.make_splits --jp3d ~/JunctionPerception/data/jp3d --sel 0.1
"""
from __future__ import annotations

import argparse
import json
import random
import re
from pathlib import Path


def block_key(s: dict) -> tuple:
    m = re.match(r"(\d+)", Path(s["image"]).name)
    t = int(m.group(1)) if m else 0
    return (s.get("release"), s.get("camera"), t // 30)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--jp3d", type=Path, required=True)
    ap.add_argument("--sel", type=float, default=0.1)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=Path, required=True, help="writes train.jsonl (fit), val.jsonl (selection), bench.jsonl")
    a = ap.parse_args()
    d = a.jp3d.expanduser()
    train = [json.loads(l) for l in (d / "train.jsonl").read_text().splitlines() if l.strip()]
    blocks = sorted({block_key(s) for s in train}, key=str)
    random.Random(a.seed).shuffle(blocks)
    sel_blocks = set(blocks[: max(1, int(len(blocks) * a.sel))])
    tr = [s for s in train if block_key(s) not in sel_blocks]
    sel = [s for s in train if block_key(s) in sel_blocks]
    o = a.out.expanduser()
    o.mkdir(parents=True, exist_ok=True)
    (o / "train.jsonl").write_text("".join(json.dumps(s) + "\n" for s in tr))
    (o / "val.jsonl").write_text("".join(json.dumps(s) + "\n" for s in sel))
    (o / "bench.jsonl").write_text((d / "val.jsonl").read_text())
    print(f"train {len(tr)}  val(selection) {len(sel)}  bench {sum(1 for _ in (o / 'bench.jsonl').open())}")


if __name__ == "__main__":
    main()
