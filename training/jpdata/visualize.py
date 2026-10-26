"""Visual check of one converted dataset (no text on the images).

Each image shows the labels as they will be trained:
  * projected 3D cuboid in the class colour, front edge red, bottom centre white dot
  * ignore regions (unreliable labels) as dark-grey outlines, unlabelled road users found by the
    detector (ignore_det) as light-grey outlines; nothing is filled or painted
Images are numbered; index.csv maps numbers to files; legend.png explains the colours.
Dropped images (label/image mismatch, black frames) are rendered into dropped/ with their labels.

    python -m jpdata.visualize --data converted/lumpi/verified.jsonl --root <image_root> \
        --report converted/lumpi/report.txt --orig converted/lumpi/data.jsonl --out converted/lumpi/vis --n 80
"""
from __future__ import annotations

import argparse
import csv
import json
import random
from pathlib import Path

import cv2
import numpy as np

from jp3d import geometry as G

COLORS = {"car": (255, 160, 60), "bus": (60, 200, 255), "truck": (60, 255, 160), "trailer": (200, 255, 60),
          "pedestrian": (80, 80, 255), "bicycle": (255, 80, 255), "escooter": (255, 255, 80), "motorcycle": (160, 80, 255)}
EDGES = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]


def cuboid_px(o, K, plane):
    if "kpts2d" in o:
        return np.array(o["kpts2d"], float)[1:], np.array(o["kpts2d"][0], float)
    n = np.array(plane[:3])
    c = G.box_corners(np.array([o["bottom"]]), np.array([o["dims"]]), np.array([o["heading"]]), n)[0]
    if (c[:, 2] < 0.3).any():
        return None, None
    return G.project(K, c), G.project(K, np.array([o["bottom"]]))[0]


BOX2D = (255, 255, 255)  # 2D box (the detector's box target): white, thin


def render(s, root: Path, width: int, labels: bool = False):
    img = cv2.imread(str(Path(s.get("_root", root)) / s["image"]))
    if img is None:
        return None
    t = max(1, round(img.shape[1] / 900))
    K, plane = np.array(s["K"]), np.array(s["plane"])
    if labels:  # 2D boxes first so the cuboids stay on top
        for o in s["objects"]:
            x1, y1, x2, y2 = map(int, o["box2d"])
            cv2.rectangle(img, (x1, y1), (x2, y2), BOX2D, max(1, t - 1), cv2.LINE_AA)
    for x1, y1, x2, y2 in s.get("ignore", []):
        cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (70, 70, 70), t)
    for x1, y1, x2, y2 in s.get("ignore_det", []):
        cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (200, 200, 200), t)
    for o in s["objects"]:
        uv, kp = cuboid_px(o, K, plane)
        if uv is None:
            continue
        col = COLORS.get(o["cls"], (255, 255, 255))
        p = uv.astype(int)
        for a, b in EDGES:
            cv2.line(img, tuple(p[a]), tuple(p[b]), (0, 0, 255) if (a, b) == (0, 1) else col,
                     t + (1 if (a, b) == (0, 1) else 0), cv2.LINE_AA)
        cv2.circle(img, tuple(kp.astype(int)), t + 2, (255, 255, 255), -1)
    s_ = width / img.shape[1]
    img = cv2.resize(img, None, fx=s_, fy=s_, interpolation=cv2.INTER_AREA)
    if labels:  # class names after resizing so the text stays readable
        for o in s["objects"]:
            x1, y1 = int(o["box2d"][0] * s_), int(o["box2d"][1] * s_)
            name = o["cls"]
            (tw, th), _ = cv2.getTextSize(name, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
            y0 = max(th + 3, y1)
            cv2.rectangle(img, (x1, y0 - th - 3), (x1 + tw + 2, y0), COLORS.get(name, (255, 255, 255)), -1)
            cv2.putText(img, name, (x1 + 1, y0 - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1, cv2.LINE_AA)
    return img


def legend(path: Path, labels: bool = False):
    rows = list(COLORS.items()) + [("front edge", (0, 0, 255))]
    rows += [("2D box", BOX2D)] if labels else [("ignore (unreliable label)", (70, 70, 70)),
                                                ("unlabelled object found by detector", (200, 200, 200))]
    im = np.full((32 * len(rows) + 16, 460, 3), 255, np.uint8)
    for i, (name, col) in enumerate(rows):
        y = 24 + 32 * i
        cv2.rectangle(im, (12, y - 14), (52, y + 6), col, -1)
        cv2.putText(im, name, (66, y + 2), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.imwrite(str(path), im)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True, help="one or more jsonl files, comma separated")
    ap.add_argument("--root", type=Path, default=Path("."), help="image root (merged samples carry their own _root)")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--n", type=int, default=80)
    ap.add_argument("--report", type=Path, default=None)
    ap.add_argument("--orig", type=Path, default=None, help="converted data before verification (for dropped images)")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--labels", action="store_true", help="also draw 2D boxes and class names")
    a = ap.parse_args()
    out = a.out
    out.mkdir(parents=True, exist_ok=True)
    legend(out / "legend.png", a.labels)
    samples = []
    for d in a.data.split(","):
        samples += [json.loads(l) for l in Path(d).read_text().splitlines() if l.strip()]
    rng = random.Random(a.seed)
    rows = []
    for i, s in enumerate(rng.sample(samples, min(a.n, len(samples))), 1):
        im = render(s, a.root, a.width, a.labels)
        if im is None:
            continue
        cv2.imwrite(str(out / f"{i:04d}.jpg"), im, [cv2.IMWRITE_JPEG_QUALITY, 90])
        rows.append([f"{i:04d}", s.get("source", ""), s["image"], len(s["objects"])])
    with open(out / "index.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["number", "source", "image", "labels"])
        w.writerows(rows)
    if a.report and a.orig and a.report.exists():
        dropped = [l.strip().split("  ")[0] for l in a.report.read_text().splitlines() if l.startswith("    ")]
        by_img = {}
        for l in a.orig.read_text().splitlines():
            s = json.loads(l)
            by_img[s["image"]] = s
        d_out = out / "dropped"
        d_out.mkdir(exist_ok=True)
        drows = []
        for j, img in enumerate(dropped[:60], 1):
            if img in by_img:
                im = render(by_img[img], a.root, a.width)
                if im is not None:
                    cv2.imwrite(str(d_out / f"{j:04d}.jpg"), im, [cv2.IMWRITE_JPEG_QUALITY, 90])
                    drows.append([f"{j:04d}", img])
        with open(d_out / "index.csv", "w", newline="") as f:
            csv.writer(f).writerows([["number", "image"]] + drows)
    print(f"rendered {len(rows)} images to {out}")


if __name__ == "__main__":
    main()
