"""Per-label agreement between converted labels and the strong detector (detect_all output).

For every clearly visible label the best detection of the same class group is classified as
  ok        IoU >= 0.5
  loose     0.3 <= IoU < 0.5
  shifted   IoU < 0.3, but a detection of similar size lies within one box width (label lags / leads
            the object: time offset between labels and frames, or a calibration error)
  missing   no detection near the label (occluded, far, dark, or a label without an object)
Writes one line per image: {"image", "n", "ok", "loose", "shifted", "missing", "miou", "shift_px"}.

    python -m jpdata.label_audit --data converted/lumpi/data.jsonl --dets converted/lumpi/dets.jsonl \
        --out converted/lumpi/audit.jsonl
"""
from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path

import numpy as np

from jpdata.verify import VEH_D, VEH_L, VRU_D, iou


def visible(o) -> bool:
    b = o["box2d"]
    return (b[3] - b[1]) >= 24 and (b[2] - b[0]) >= 12 and o.get("truncation", 0) < 0.5


def classify(b, dets):
    """Return (kind, iou, shift in label widths)."""
    if not dets:
        return "missing", 0.0, None
    ious = [iou(b, d) for d in dets]
    k = int(np.argmax(ious))
    if ious[k] >= 0.5:
        return "ok", ious[k], None
    if ious[k] >= 0.3:
        return "loose", ious[k], None
    w, h = b[2] - b[0], b[3] - b[1]
    cx, cy = (b[0] + b[2]) / 2, (b[1] + b[3]) / 2
    best = None
    for d in dets:
        dw, dh = d[2] - d[0], d[3] - d[1]
        if not (0.4 <= dh / h <= 2.5 and 0.3 <= dw / w <= 3.0):
            continue
        dist = np.hypot((d[0] + d[2]) / 2 - cx, (d[1] + d[3]) / 2 - cy) / max(w, 1.0)
        if dist <= 1.0 and (best is None or dist < best):
            best = dist
    if best is not None:
        return "shifted", ious[k], float(best)
    return "missing", ious[k], None


def audit(s, dets, conf=0.3):
    D = [d for d in dets if d[4] >= conf]
    r = Counter()
    ious, shifts = [], []
    for o in s["objects"]:
        if not visible(o):
            continue
        grp = VEH_D if o["cls"] in VEH_L else VRU_D
        kind, v, sh = classify(o["box2d"], [d[:4] for d in D if d[5] in grp])
        o["audit"] = kind
        r["n"] += 1
        r[kind] += 1
        if kind in ("ok", "loose", "shifted"):
            ious.append(v)
        if sh is not None:
            shifts.append(sh)
    return {"image": s["image"], **{k: r[k] for k in ("n", "ok", "loose", "shifted", "missing")},
            "miou": round(float(np.mean(ious)), 3) if ious else None,
            "shift": round(float(np.mean(shifts)), 3) if shifts else None}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", type=Path, required=True)
    ap.add_argument("--dets", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    dets = {}
    for l in a.dets.read_text().splitlines():
        d = json.loads(l)
        dets[d["image"]] = d["dets"]
    tot = Counter()
    with open(a.out, "w") as f:
        for l in a.data.read_text().splitlines():
            if not l.strip():
                continue
            s = json.loads(l)
            if s["image"] not in dets:
                continue
            r = audit(s, dets[s["image"]])
            for k in ("n", "ok", "loose", "shifted", "missing"):
                tot[k] += r[k]
            f.write(json.dumps(r) + "\n")
    n = max(tot["n"], 1)
    print(f"{a.data.parent.name}: visible labels {tot['n']}  " +
          "  ".join(f"{k} {100 * tot[k] / n:.1f}%" for k in ("ok", "loose", "shifted", "missing")))


if __name__ == "__main__":
    main()
