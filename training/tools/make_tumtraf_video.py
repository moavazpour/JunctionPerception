"""Turn one TUMTraf camera sequence into a video, to replay it through the runtime (calibration validation
against the dev-kit ground truth).

    python -m tools.make_tumtraf_video \
        --images ~/JunctionPerception/data/raw/TUMTraf/R2/a9_dataset_r02_s03/a9_dataset_r02_s03/images/s110_camera_basler_south2_8mm \
        --out ~/JunctionPerception/data/raw/test_videos/tumtraf_s03_south2_8mm.mp4
"""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


def stamp(p: Path) -> float:
    s, ns = p.stem.split("_")[:2]
    return int(s) + int(ns) * 1e-9


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    files = sorted(a.images.glob("*.jpg"), key=stamp)
    t = np.array([stamp(p) for p in files])
    fps = float(round(1.0 / np.median(np.diff(t)), 2))
    first = cv2.imread(str(files[0]))
    h, w = first.shape[:2]
    a.out.parent.mkdir(parents=True, exist_ok=True)
    vw = cv2.VideoWriter(str(a.out), cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
    for p in files:
        vw.write(cv2.imread(str(p)))
    vw.release()
    print(f"{len(files)} frames at {fps} fps ({w}x{h}) -> {a.out}")


if __name__ == "__main__":
    main()
