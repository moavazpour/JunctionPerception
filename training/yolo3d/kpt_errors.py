"""Per-keypoint accuracy of a YOLO pose model on a split of the exported dataset.

Predictions are matched to ground-truth boxes (same class, IoU >= 0.5); for every keypoint the pixel
error is reported absolutely and relative to the ground-truth box height, separately for the bottom
centre, the 4 footprint corners and the 4 top corners.

    python -m yolo3d.kpt_errors --weights best.pt --data merged/yolo --split val --n 300 --device cpu
"""
from __future__ import annotations

import argparse
import random
from pathlib import Path

import numpy as np
import torch
from ultralytics import YOLO

NAMES = ["bottom centre", "bottom FL", "bottom FR", "bottom RR", "bottom RL", "top FL", "top FR", "top RR", "top RL"]


def iou(a, b):
    x1, y1 = np.maximum(a[:2], b[:2])
    x2, y2 = np.minimum(a[2:], b[2:])
    i = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    return i / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - i + 1e-9)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", required=True)
    ap.add_argument("--data", type=Path, required=True, help="exported YOLO dataset root")
    ap.add_argument("--split", default="val")
    ap.add_argument("--n", type=int, default=300)
    ap.add_argument("--imgsz", type=int, default=960)
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--threads", type=int, default=4)
    a = ap.parse_args()
    torch.set_num_threads(a.threads)
    model = YOLO(a.weights)
    imgs = sorted((a.data / "images" / a.split).iterdir())
    random.Random(0).shuffle(imgs)
    abs_err = [[] for _ in range(9)]
    rel_err = [[] for _ in range(9)]
    matched = total = 0
    for p in imgs[: a.n]:
        lab = a.data / "labels" / a.split / (p.stem + ".txt")
        if not lab.exists():
            continue
        rows = [list(map(float, l.split())) for l in lab.read_text().splitlines() if l.strip()]
        if not rows:
            continue
        r = model.predict(str(p), imgsz=a.imgsz, conf=0.25, device=a.device, verbose=False)[0]
        H, W = r.orig_shape
        pb = r.boxes.xyxy.cpu().numpy() if len(r.boxes) else np.zeros((0, 4))
        pc = r.boxes.cls.cpu().numpy().astype(int) if len(r.boxes) else np.zeros(0, int)
        pk = r.keypoints.data.cpu().numpy() if r.keypoints is not None and len(r.boxes) else np.zeros((0, 9, 3))
        used = set()
        for g in rows:
            total += 1
            c, cx, cy, bw, bh = int(g[0]), g[1] * W, g[2] * H, g[3] * W, g[4] * H
            gb = np.array([cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2])
            best, bi = 0.5, -1
            for i in range(len(pb)):
                if i in used or pc[i] != c:
                    continue
                v = iou(gb, pb[i])
                if v >= best:
                    best, bi = v, i
            if bi < 0:
                continue
            used.add(bi)
            matched += 1
            kg = np.array(g[5:], float).reshape(9, 3)
            for k in range(9):
                if kg[k, 2] == 0:  # outside the image
                    continue
                e = float(np.hypot(pk[bi, k, 0] - kg[k, 0] * W, pk[bi, k, 1] - kg[k, 1] * H))
                abs_err[k].append(e)
                rel_err[k].append(e / max(bh, 1.0))
    print(f"{matched}/{total} ground-truth objects matched (IoU >= 0.5)")
    print(f"{'keypoint':14s} {'median px':>9s} {'p90 px':>8s} {'median %h':>9s} {'p90 %h':>8s}")
    for k in range(9):
        e, rel = np.array(abs_err[k]), np.array(rel_err[k]) * 100
        print(f"{NAMES[k]:14s} {np.median(e):9.1f} {np.percentile(e, 90):8.1f} {np.median(rel):9.1f} {np.percentile(rel, 90):8.1f}")
    for name, ks in (("bottom (k0-k4)", range(5)), ("top (k5-k8)", range(5, 9))):
        rel = np.concatenate([rel_err[k] for k in ks]) * 100
        print(f"{name:14s} median {np.median(rel):.1f} % of box height, p90 {np.percentile(rel, 90):.1f} %")


if __name__ == "__main__":
    main()
