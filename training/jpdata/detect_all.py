"""Run a strong pretrained COCO detector over every image of a converted source and store its boxes.
Used to verify labels image by image and to find visible road users a source left unlabelled.

    python -m jpdata.detect_all --src lumpi=conv/lumpi2.jsonl:undist/lumpi2 --model yolo26x.pt --out conv/lumpi2.dets.jsonl
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from ultralytics import YOLO

# COCO ids -> JP class (cyclists appear as person + bicycle; matching handles that)
COCO = {0: "pedestrian", 1: "bicycle", 2: "car", 3: "motorcycle", 5: "bus", 7: "truck"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True, help="name=jsonl:image_root")
    ap.add_argument("--model", default="yolo26x.pt")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--imgsz", type=int, default=1536)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--batch", type=int, default=8)
    a = ap.parse_args()
    name, rest = a.src.split("=", 1)
    jl, root = rest.split(":", 1)
    root = Path(root).expanduser()
    images = [json.loads(l)["image"] for l in Path(jl).expanduser().read_text().splitlines() if l.strip()]
    done = set()
    if a.out.exists():
        done = {json.loads(l)["image"] for l in a.out.read_text().splitlines() if l.strip()}
    todo = [i for i in images if i not in done]
    model = YOLO(a.model)
    with open(a.out, "a") as f:
        for k in range(0, len(todo), a.batch):
            chunk = todo[k:k + a.batch]
            res = model.predict([str(root / i) for i in chunk], imgsz=a.imgsz, conf=a.conf, classes=list(COCO),
                                verbose=False, half=True)
            for img, r in zip(chunk, res):
                b = r.boxes
                dets = [[*map(lambda v: round(float(v), 1), xyxy), round(float(c), 3), COCO[int(k_)]]
                        for xyxy, c, k_ in zip(b.xyxy.cpu().numpy(), b.conf.cpu().numpy(), b.cls.cpu().numpy())]
                f.write(json.dumps({"image": img, "dets": dets}) + "\n")
            if k % 800 == 0:
                print(f"{name}: {k + len(chunk)}/{len(todo)}", flush=True)
    print(f"{name}: done")


if __name__ == "__main__":
    main()
