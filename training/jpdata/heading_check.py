"""Check that every converted source uses the same heading convention (heading = the vehicle's front).

For sources recorded as sequences, each moving vehicle is linked to its nearest same-class neighbour in
the next frame of the same camera; its displacement must point the way its heading points. A source
whose convention is flipped shows most moving vehicles going "backwards".

    python -m jpdata.heading_check converted/*/data.jsonl
"""
from __future__ import annotations

import json
import re
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

VEH = {"car", "bus", "truck", "trailer"}


def order_key(s):
    m = re.findall(r"\d+", Path(s["image"]).stem)
    return [int(x) for x in m] if m else [0]


def main():
    for path in sys.argv[1:]:
        samples = [json.loads(l) for l in open(path) if l.strip()]
        name = Path(path).parent.name
        seqs = defaultdict(list)
        for s in samples:
            seqs[(s.get("camera"), s.get("sequence"), s.get("release"))].append(s)
        fwd = back = 0
        for frames in seqs.values():
            frames.sort(key=order_key)
            for a, b in zip(frames, frames[1:]):
                if not a["objects"] or not b["objects"]:
                    continue
                pb = np.array([o["bottom"] for o in b["objects"]])
                cb = [o["cls"] for o in b["objects"]]
                for o in a["objects"]:
                    if o["cls"] not in VEH:
                        continue
                    p = np.array(o["bottom"])
                    d = np.linalg.norm(pb - p, axis=1)
                    d[[c != o["cls"] for c in cb]] = 1e9
                    j = int(np.argmin(d))
                    if not (0.5 < d[j] < 3.0):  # moving, and unambiguous between consecutive frames
                        continue
                    if np.sort(d)[1] < 2.0 * d[j] if len(d) > 1 else False:
                        continue
                    disp = pb[j] - p
                    h = np.array(o["heading"])
                    c = disp @ h / (np.linalg.norm(disp) * np.linalg.norm(h) + 1e-9)
                    if abs(c) < 0.7:  # turning / lateral: not informative
                        continue
                    fwd += c > 0
                    back += c < 0
        n = fwd + back
        print(f"{name:18s} moving vehicles checked {n:6d}   heading = direction of travel {100 * fwd / max(n, 1):5.1f} %"
              f"   reversed {100 * back / max(n, 1):5.1f} %")


if __name__ == "__main__":
    main()
