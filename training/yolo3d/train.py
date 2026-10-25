"""Train YOLO26-pose on projected 3D-box keypoints.

    python -m yolo3d.train --data ~/data/junction_perception/yolo3d/data.yaml --model yolo26m-pose.pt \
        --out ~/data/junction_perception/runs --name y26m_kp3d --epochs 60
"""
from __future__ import annotations

import argparse

from ultralytics import YOLO


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True)
    ap.add_argument("--model", default="yolo26m-pose.pt")
    ap.add_argument("--out", required=True)
    ap.add_argument("--name", default="y26_kp3d")
    ap.add_argument("--epochs", type=int, default=60)
    ap.add_argument("--imgsz", type=int, default=1280)
    ap.add_argument("--batch", type=int, default=8)
    ap.add_argument("--workers", type=int, default=10)
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--fraction", type=float, default=1.0, help="share of the training set (smoke tests)")
    a = ap.parse_args()
    model = YOLO(a.model)
    model.train(
        data=a.data, epochs=a.epochs, imgsz=a.imgsz, batch=a.batch, workers=a.workers, project=a.out, name=a.name,
        exist_ok=True, resume=a.resume, cos_lr=True, close_mosaic=10, fraction=a.fraction, patience=25,
        # Geometry-preserving augmentation only: keypoints are projections of a rigid cuboid, so shear and
        # perspective warps would teach impossible boxes. Scale/translate/flip are camera-consistent.
        degrees=0.0, shear=0.0, perspective=0.0, scale=0.5, translate=0.1, fliplr=0.5, mosaic=0.8, mixup=0.0,
        hsv_h=0.015, hsv_s=0.6, hsv_v=0.5, pose=12.0, kobj=1.0,
    )


if __name__ == "__main__":
    main()
