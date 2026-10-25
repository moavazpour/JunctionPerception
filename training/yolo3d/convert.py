"""Convert JP3D splits into an Ultralytics pose dataset of projected 3D-box keypoints.

Keypoints (9): bottom centre, bottom corners front-left, front-right, rear-right, rear-left, then the
same four on top. Visibility is 2 when the point projects inside the image in front of the camera.
The box is the extent of the projected cuboid (what the runtime uses as the 2D box).

    python -m yolo3d.convert --jp3d ~/data/junction_perception/jp3d --images ~/data/junction_perception/datasets/TUMTraf \
        --extra ~/data/junction_perception/carla:~/data/junction_perception/carla --out ~/data/junction_perception/yolo3d
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

import numpy as np

from jp3d import geometry as G

FLIP_IDX = [0, 2, 1, 4, 3, 6, 5, 8, 7]


def keypoints(o: dict, K: np.ndarray, plane: np.ndarray):
    n = plane[:3]
    b = np.array(o["bottom"])
    c = G.box_corners(b[None], np.array([o["dims"]]), np.array([o["heading"]]), n)[0]
    pts = np.vstack([b[None], c])  # 9 x 3
    return pts


def convert_split(samples: list[dict], image_root: Path, out: Path, split: str, prefix: str):
    (out / "images" / split).mkdir(parents=True, exist_ok=True)
    (out / "labels" / split).mkdir(parents=True, exist_ok=True)
    n_img = n_obj = 0
    for s in samples:
        K = np.array(s["K"])
        plane = np.array(s["plane"])
        W, H = s["width"], s["height"]
        lines = []
        for o in s["objects"]:
            if "kpts2d" in o:
                # Exact image keypoints from the source projection (e.g. DAIR affine calibration).
                uv = np.array(o["kpts2d"], float)
            else:
                pts = keypoints(o, K, plane)
                if (pts[:, 2] < 0.3).any():
                    continue
                uv = G.project(K, pts)
            x1, y1 = np.clip(uv.min(0), 0, [W - 1, H - 1])
            x2, y2 = np.clip(uv.max(0), 0, [W - 1, H - 1])
            if x2 - x1 < 3 or y2 - y1 < 3:
                continue
            ci = G.CLASSES.index(o["cls"])
            vals = [ci, (x1 + x2) / 2 / W, (y1 + y2) / 2 / H, (x2 - x1) / W, (y2 - y1) / H]
            for (u, v) in uv:
                inside = 0 <= u < W and 0 <= v < H
                vals += [u / W if inside else 0.0, v / H if inside else 0.0, 2 if inside else 0]
            lines.append(" ".join(f"{v:.6f}" if isinstance(v, float) else str(v) for v in vals))
        name = prefix + (s.get("source", "") + "__" if s.get("source") else "") + s["image"].replace("/", "__")
        src = Path(s.get("_root", image_root)) / s["image"]
        dst = out / "images" / split / name
        if not dst.exists():
            if s.get("ignore"):
                # Ignore regions (DontCare, ignored classes) are painted grey: neither positive nor negative.
                import cv2

                im = cv2.imread(str(src))
                for x1, y1, x2, y2 in s["ignore"]:
                    im[int(max(y1, 0)):int(y2) + 1, int(max(x1, 0)):int(x2) + 1] = 114
                dst = dst.with_suffix(".jpg")
                cv2.imwrite(str(dst), im, [cv2.IMWRITE_JPEG_QUALITY, 95])
            else:
                os.symlink(src.resolve(), dst)
        (out / "labels" / split / (Path(name).stem + ".txt")).write_text("\n".join(lines) + ("\n" if lines else ""))
        # Camera model next to the label, for 3D evaluation of the pose model.
        n_img += 1
        n_obj += len(lines)
    return n_img, n_obj


def weather_copies(samples: list[dict], out: Path, frac: float, seed: int = 0):
    """Offline adverse-weather copies of a fraction of the training images (fog, rain, night, glare, droplets)."""
    import random

    import cv2

    from jp3d.weather import WeatherAug

    rng = random.Random(seed)
    aug = WeatherAug(p_color=0.5, p_weather=1.0, p_blur=0.3, p_jpeg=0.3)
    picked = [s for s in samples if rng.random() < frac]
    copies = []
    (out / "weather").mkdir(parents=True, exist_ok=True)
    for i, s in enumerate(picked):
        im = cv2.imread(str(Path(s["_root"]) / s["image"]))
        if im is None:
            continue
        for x1, y1, x2, y2 in s.get("ignore", []):
            im[int(max(y1, 0)):int(y2) + 1, int(max(x1, 0)):int(x2) + 1] = 114
        im = aug(im, np.random.default_rng(seed * 100003 + i))
        rel = f"weather/{s['source']}_{i:06d}.jpg"
        cv2.imwrite(str(out / rel), im, [cv2.IMWRITE_JPEG_QUALITY, 92])
        copies.append({**s, "_root": str(out), "image": rel, "ignore": [], "source": s["source"] + "_wx"})
    return copies


def main_unified(unified: Path, out: Path, weather_frac: float = 0.25):
    for split in ("train", "val", "test"):
        samples = [json.loads(l) for l in (unified / f"{split}.jsonl").read_text().splitlines() if l.strip()]
        if split == "train" and weather_frac > 0:
            samples += weather_copies(samples, out, weather_frac)
        ni, no = convert_split(samples, Path("."), out, split, "")
        print(split, ni, "images", no, "objects")
    (out / "data.yaml").write_text(
        f"path: {out}\ntrain: images/train\nval: images/val\ntest: images/test\n"
        f"kpt_shape: [9, 3]\nflip_idx: {FLIP_IDX}\n"
        "names:\n" + "".join(f"  {i}: {c}\n" for i, c in enumerate(G.CLASSES)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--unified", type=Path, default=None, help="JP-Unified dir with train/val/test.jsonl")
    ap.add_argument("--jp3d", type=Path, default=None)
    ap.add_argument("--images", type=Path, default=None)
    ap.add_argument("--extra", action="append", default=[], help="jsonl_dir:image_root for more sources")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--weather", type=float, default=0.25, help="fraction of train images duplicated with weather effects")
    args = ap.parse_args()
    out = args.out.expanduser()
    if args.unified:
        main_unified(args.unified.expanduser(), out, args.weather)
        return
    sources = [(args.jp3d.expanduser(), args.images.expanduser(), "tum__")]
    for e in args.extra:
        d, r = e.split(":")
        sources.append((Path(d).expanduser(), Path(r).expanduser(), Path(d).name + "__"))
    for split in ("train", "val"):
        for d, root, prefix in sources:
            f = d / f"{split}.jsonl"
            if not f.exists():
                continue
            samples = [json.loads(l) for l in f.read_text().splitlines() if l.strip()]
            ni, no = convert_split(samples, root, out, split, prefix)
            print(split, d, ni, "images", no, "objects")
    (out / "data.yaml").write_text(
        f"path: {out}\ntrain: images/train\nval: images/val\n"
        f"kpt_shape: [9, 3]\nflip_idx: {FLIP_IDX}\n"
        "names:\n" + "".join(f"  {i}: {c}\n" for i, c in enumerate(G.CLASSES)))
    print("wrote", out / "data.yaml")


if __name__ == "__main__":
    main()
