"""Draw converted labels (or predictions) on images to verify calibration and conversion.

    python -m tools.visualize_labels --data ~/JunctionPerception/data/jp3d/train.jsonl \
        --images ~/JunctionPerception/data/datasets/TUMTraf --out /tmp/vis --n 8
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from jp3d import geometry as G

COLORS = {"car": (255, 160, 60), "bus": (60, 200, 255), "truck": (60, 255, 160), "trailer": (200, 255, 60),
          "pedestrian": (80, 80, 255), "bicycle": (255, 80, 255), "escooter": (255, 255, 80), "motorcycle": (160, 80, 255)}
EDGES = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]


def draw_objects(img, K, plane, objs, thickness=2):
    n = np.asarray(plane[:3])
    for o in objs:
        c = G.box_corners(np.array([o["bottom"]]), np.array([o["dims"]]), np.array([o["heading"]]), n)[0]
        if (c[:, 2] < 0.5).any():
            continue
        uv = G.project(K, c).astype(int)
        col = COLORS.get(o["cls"], (255, 255, 255))
        for a, b in EDGES:
            cv2.line(img, tuple(uv[a]), tuple(uv[b]), col, thickness, cv2.LINE_AA)
        cv2.line(img, tuple(uv[0]), tuple(uv[1]), (0, 0, 255), thickness + 1, cv2.LINE_AA)  # front face
        kp = G.project(K, np.array([o["bottom"]]))[0].astype(int)
        cv2.circle(img, tuple(kp), 3, col, -1)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", type=Path, required=True)
    ap.add_argument("--images", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--n", type=int, default=8)
    ap.add_argument("--camera", default=None)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    samples = [json.loads(l) for l in args.data.expanduser().read_text().splitlines()]
    if args.camera:
        samples = [s for s in samples if args.camera in s["camera"]]
    picks = np.linspace(0, len(samples) - 1, args.n).astype(int)
    for i in picks:
        s = samples[i]
        img = cv2.imread(str(Path(s.get("_root", args.images.expanduser())) / s["image"]))
        draw_objects(img, np.array(s["K"]), np.array(s["plane"]), s["objects"])
        name = f"{s.get('source', s.get('release', 'x'))}_{s['camera'].replace('s110_camera_basler_', '')}_{i}.jpg"
        cv2.imwrite(str(args.out / name), cv2.resize(img, None, fx=0.5, fy=0.5))
    print("wrote", len(picks), "images to", args.out)


if __name__ == "__main__":
    main()
