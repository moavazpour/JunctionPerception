"""Run a YOLO26-pose keypoint model over a JP3D split, lift to 3D and write jp_bench-format predictions.

    python -m yolo3d.predict --weights best.pt --data test.jsonl --images ROOT --out preds_y26.jsonl
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from ultralytics import YOLO

from jp3d import geometry as G
from yolo3d.lift import lift_keypoints


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", required=True)
    ap.add_argument("--data", type=Path, required=True)
    ap.add_argument("--images", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--imgsz", type=int, default=1280)
    ap.add_argument("--conf", type=float, default=0.05)
    a = ap.parse_args()
    model = YOLO(a.weights)
    samples = [json.loads(l) for l in a.data.read_text().splitlines() if l.strip()]
    with open(a.out, "w") as f:
        for s in samples:
            r = model.predict(str(a.images / s["image"]), imgsz=a.imgsz, conf=a.conf, verbose=False, max_det=300)[0]
            K, plane = np.array(s["K"]), np.array(s["plane"])
            objs = []
            if r.keypoints is not None and len(r.boxes):
                kp = r.keypoints.data.cpu().numpy()  # N x 9 x 3
                for i in range(len(r.boxes)):
                    cls = G.CLASSES[int(r.boxes.cls[i])]
                    o = lift_keypoints(kp[i], cls, float(r.boxes.conf[i]), K, plane)
                    if o:
                        o["box2d"] = r.boxes.xyxy[i].cpu().numpy().tolist()
                        objs.append(o)
            f.write(json.dumps({"image": s["image"], "objects": objs}) + "\n")
    print("wrote", a.out)


if __name__ == "__main__":
    main()
