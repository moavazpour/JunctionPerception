"""Render the final YOLO pose dataset exactly as the trainer reads it (image files + label .txt) for a
visual check. Images are numbered; index.csv maps numbers to files.

Drawing: 2D box and class, the projected cuboid from keypoints 1-8 (bottom FL FR RR RL, top FL FR RR
RL), front edge in red, bottom centre (k0) as a white dot. Keypoints with visibility 0 are not drawn.

    python -m yolo3d.visualize_yolo --root ~/data/junction_perception/yolo_unified_v3 \
        --out ~/data/junction_perception/qc/yolo_v3_check --per_source 40
"""
from __future__ import annotations

import argparse
import csv
import random
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

NAMES = ["car", "bus", "truck", "trailer", "pedestrian", "bicycle", "escooter", "motorcycle"]
COLORS = [(255, 160, 60), (60, 200, 255), (60, 255, 160), (200, 255, 60), (80, 80, 255), (255, 80, 255),
          (255, 255, 80), (160, 80, 255)]
EDGES = [(1, 2), (2, 3), (3, 4), (4, 1), (5, 6), (6, 7), (7, 8), (8, 5), (1, 5), (2, 6), (3, 7), (4, 8)]


def draw(img: np.ndarray, label: Path) -> tuple[np.ndarray, int]:
    H, W = img.shape[:2]
    n = 0
    lines = label.read_text().splitlines() if label.exists() else []
    t = max(1, round(W / 900))
    for line in lines:
        v = [float(x) for x in line.split()]
        if len(v) < 5 + 27:
            continue
        c = int(v[0])
        cx, cy, bw, bh = v[1] * W, v[2] * H, v[3] * W, v[4] * H
        col = COLORS[c % len(COLORS)]
        k = np.array(v[5:5 + 27]).reshape(9, 3)
        pts = [(int(x * W), int(y * H)) if vis > 0 else None for x, y, vis in k]
        cv2.rectangle(img, (int(cx - bw / 2), int(cy - bh / 2)), (int(cx + bw / 2), int(cy + bh / 2)), col, t)
        for a, b in EDGES:
            if pts[a] and pts[b]:
                cv2.line(img, pts[a], pts[b], (0, 0, 255) if (a, b) == (1, 2) else col, t + (1 if (a, b) == (1, 2) else 0),
                         cv2.LINE_AA)
        if pts[0]:
            cv2.circle(img, pts[0], t + 2, (255, 255, 255), -1)
        cv2.putText(img, NAMES[c] if c < len(NAMES) else str(c), (int(cx - bw / 2), int(cy - bh / 2) - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45 * t, col, max(1, t - 1), cv2.LINE_AA)
        n += 1
    return img, n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--per_source", type=int, default=40, help="images per source and split (train); val/test get half")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    root, out = a.root.expanduser(), a.out.expanduser()
    out.mkdir(parents=True, exist_ok=True)
    rng = random.Random(a.seed)
    rows = []
    idx = 0
    for split in ("train", "val", "test"):
        by_src = defaultdict(list)
        for p in (root / "images" / split).iterdir():
            by_src[p.name.split("__")[0]].append(p)
        for src in sorted(by_src):
            files = sorted(by_src[src])
            k = a.per_source if split == "train" else max(1, a.per_source // 2)
            for p in rng.sample(files, min(k, len(files))):
                img = cv2.imread(str(p))
                if img is None:
                    continue
                img, n = draw(img, root / "labels" / split / (p.stem + ".txt"))
                s = a.width / img.shape[1]
                img = cv2.resize(img, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
                idx += 1
                cv2.putText(img, f"#{idx}  {split}  {p.name}  ({n} objects)", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 0, 0), 4, cv2.LINE_AA)
                cv2.putText(img, f"#{idx}  {split}  {p.name}  ({n} objects)", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (255, 255, 255), 1, cv2.LINE_AA)
                name = f"{idx:04d}_{split}_{p.stem}.jpg"
                cv2.imwrite(str(out / name), img, [cv2.IMWRITE_JPEG_QUALITY, 88])
                rows.append([idx, split, src, p.name, n, str(p.resolve())])
    with open(out / "index.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["number", "split", "source", "file", "objects", "source_image"])
        w.writerows(rows)
    print(f"wrote {idx} images to {out}")


if __name__ == "__main__":
    main()
