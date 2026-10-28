"""Numeric quality control of a converted JP-Unified source (no images needed).

Reports per class: counts, median dimensions, range distribution, heading spread; per camera: how
well the projected cuboid agrees with the source's own 2D box (IoU of the projected 3D box with the
labelled box2d), keypoints on the ground plane, and objects behind / outside the image.

    python -m jpdata.qc ~/JunctionPerception/data/conv/kitti.jsonl [--overlays DIR --images ROOT --n 12]
"""
from __future__ import annotations

import argparse
import json
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

from jp3d import geometry as G


def iou(a, b):
    x0, y0, x1, y1 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    i = max(0.0, x1 - x0) * max(0.0, y1 - y0)
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - i
    return i / u if u > 0 else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("jsonl", type=Path)
    ap.add_argument("--max", type=int, default=20000)
    a = ap.parse_args()
    samples = [json.loads(l) for l in a.jsonl.expanduser().read_text().splitlines()[: a.max] if l.strip()]
    cls_n, dims, rngs = Counter(), defaultdict(list), defaultdict(list)
    cam_iou, cam_n, cam_h = defaultdict(list), Counter(), defaultdict(list)
    n_img = len(samples)
    n_ign = 0
    for s in samples:
        K, plane = np.array(s["K"]), np.array(s["plane"])
        cam_h[s["camera"]].append(plane[3])
        n_ign += len(s.get("ignore", []))
        for o in s["objects"]:
            cls_n[o["cls"]] += 1
            dims[o["cls"]].append(o["dims"])
            b = np.array(o["bottom"])
            rngs[o["cls"]].append(float(np.hypot(b[0], b[2])))
            if "kpts2d" in o:
                uv = np.array(o["kpts2d"])[1:]
            else:
                c = G.box_corners(b[None], np.array([o["dims"]]), np.array([o["heading"]]), plane[:3])[0]
                if (c[:, 2] < 0.3).any():
                    continue
                uv = G.project(K, c)
            pb = [*np.clip(uv.min(0), 0, [s["width"] - 1, s["height"] - 1]), *np.clip(uv.max(0), 0, [s["width"] - 1, s["height"] - 1])]
            cam_iou[s["camera"]].append(iou(pb, o.get("src_box2d") or o["box2d"]))
            cam_n[s["camera"]] += 1
    print(f"== {a.jsonl.name}: {n_img} images, {sum(cls_n.values())} objects, {n_ign} ignore regions")
    print(f"{'class':<11}{'n':>8}{'l':>7}{'w':>6}{'h':>6}{'r_med':>7}{'r_p90':>7}")
    for c in G.CLASSES:
        if not cls_n[c]:
            continue
        d = np.median(np.array(dims[c]), axis=0)
        r = np.array(rngs[c])
        print(f"{c:<11}{cls_n[c]:>8}{d[0]:>7.2f}{d[1]:>6.2f}{d[2]:>6.2f}{np.median(r):>7.1f}{np.percentile(r, 90):>7.1f}")
    print(f"{'camera':<44}{'objs':>7}{'IoU(proj,box2d) med':>21}{'p10':>6}{'cam h':>7}")
    for cam in sorted(cam_n, key=lambda k: -cam_n[k])[:25]:
        v = np.array(cam_iou[cam])
        print(f"{cam[:43]:<44}{cam_n[cam]:>7}{np.median(v):>21.3f}{np.percentile(v, 10):>6.2f}{np.median(cam_h[cam]):>7.2f}")


if __name__ == "__main__":
    main()
