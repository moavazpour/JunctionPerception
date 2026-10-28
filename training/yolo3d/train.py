"""Train YOLO26-pose on projected 3D-box keypoints.

    python -m yolo3d.train --data ~/JunctionPerception/data/yolo3d/data.yaml --model yolo26m-pose.pt \
        --out ~/JunctionPerception/data/runs --name y26m_kp3d --epochs 60
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
    ap.add_argument("--lr0", type=float, default=0.01, help="initial learning rate (fine-tuning: lower)")
    ap.add_argument("--close_mosaic", type=int, default=10)
    ap.add_argument("--warmup_epochs", type=float, default=3.0)
    ap.add_argument("--optimizer", default="auto", help="auto picks MuSGD at lr 0.01 and ignores --lr0; set it explicitly to fine-tune")
    a = ap.parse_args()
    model = YOLO(a.model)
    model.train(
        data=a.data, epochs=a.epochs, imgsz=a.imgsz, batch=a.batch, workers=a.workers, project=a.out, name=a.name,
        exist_ok=True, resume=a.resume, cos_lr=True, close_mosaic=a.close_mosaic, fraction=a.fraction, patience=25,
        lr0=a.lr0, warmup_epochs=a.warmup_epochs, optimizer=a.optimizer,
        # Geometry-preserving augmentation only: keypoints are projections of a rigid cuboid, so shear and
        # perspective warps would teach impossible boxes. Scale/translate/flip are camera-consistent.
        degrees=0.0, shear=0.0, perspective=0.0, scale=0.5, translate=0.1, fliplr=0.5, mosaic=0.8, mixup=0.0,
        hsv_h=0.015, hsv_s=0.6, hsv_v=0.5, pose=12.0, kobj=1.0,
    )


if __name__ == "__main__":
    main()
