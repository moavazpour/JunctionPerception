"""Find objects a source leaves unlabelled: confident detections of a trained model that match no
label (and no ignore region), binned by ground range. A source with label coverage holes (e.g. LiDAR
range limits) shows many confident unmatched detections beyond some distance.

    python -m yolo3d.label_coverage --weights best.pt --data unified_v3/val.jsonl --sources lumpi,dair_i,tumtraf_i
"""
from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path

import numpy as np
from ultralytics import YOLO

from jp3d import geometry as G

BINS = [0, 20, 40, 60, 80, 100, 150, 1e9]


def iou(a, b):
    x0, y0, x1, y1 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    i = max(0.0, x1 - x0) * max(0.0, y1 - y0)
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - i
    return i / u if u > 0 else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", required=True)
    ap.add_argument("--data", type=Path, required=True)
    ap.add_argument("--sources", default="")
    ap.add_argument("--n", type=int, default=150)
    ap.add_argument("--conf", type=float, default=0.5)
    ap.add_argument("--imgsz", type=int, default=1280)
    a = ap.parse_args()
    model = YOLO(a.weights)
    want = set(a.sources.split(",")) if a.sources else None
    by_src = defaultdict(list)
    for l in a.data.read_text().splitlines():
        s = json.loads(l)
        if want is None or s["source"] in want:
            by_src[s["source"]].append(s)
    for src, samples in sorted(by_src.items()):
        samples = samples[:: max(1, len(samples) // a.n)][: a.n]
        unmatched, matched, labelled = defaultdict(int), defaultdict(int), defaultdict(int)
        for s in samples:
            K, plane = np.array(s["K"]), np.array(s["plane"])
            boxes = [o["box2d"] for o in s["objects"]] + list(s.get("ignore", []))
            for o in s["objects"]:
                r = float(np.hypot(o["bottom"][0], o["bottom"][2]))
                labelled[np.digitize(r, BINS) - 1] += 1
            res = model.predict(str(Path(s["_root"]) / s["image"]), imgsz=a.imgsz, conf=a.conf, verbose=False)[0]
            if res.keypoints is None:
                continue
            kp = res.keypoints.data.cpu().numpy()
            for i, b in enumerate(res.boxes.xyxy.cpu().numpy()):
                # Range from the predicted bottom-centre keypoint on the ground plane.
                g = G.ray_plane_intersect(G.pixel_rays(K, kp[i, :1, :2]), plane)[0]
                if not np.isfinite(g).all():
                    continue
                r = float(np.hypot(g[0], g[2]))
                k = np.digitize(r, BINS) - 1
                if max((iou(b, x) for x in boxes), default=0.0) < 0.1:
                    unmatched[k] += 1
                else:
                    matched[k] += 1
        print(f"== {src}: {len(samples)} images, confident detections (conf >= {a.conf}) vs labels by range")
        print(f"{'range m':>10}{'labelled':>10}{'det matched':>13}{'det UNLABELLED':>16}{'share':>8}")
        for k in range(len(BINS) - 1):
            lo, hi = BINS[k], BINS[k + 1]
            m, u = matched[k], unmatched[k]
            print(f"{f'{lo:.0f}-{hi:.0f}' if hi < 1e9 else f'{lo:.0f}+':>10}{labelled[k]:>10}{m:>13}{u:>16}"
                  f"{(u / (m + u) if m + u else 0):>8.0%}")


if __name__ == "__main__":
    main()
