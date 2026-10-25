"""KITTI-format sources (KITTI 3D object, Rope3D, DAIR-V2X-I in KITTI layout) -> JP-Unified jsonl.

Layout expected under --root:  <split>/image_2/*.{png,jpg}, <split>/label_2/*.txt, <split>/calib/*.txt
and optionally <split>/denorm/*.txt (ground plane a b c d in camera coordinates, Rope3D/DAIR).

KITTI label line: type trunc occ alpha x1 y1 x2 y2 h w l x y z rotation_y
  (x y z) = bottom centre in the rectified reference camera frame, rotation_y about the camera y axis.

    python -m jpdata.kitti_like --source kitti --root ~/data/.../KITTI/object --splits training --out kitti.jsonl
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from jp3d import geometry as G

MAPS = {
    "kitti": {"Car": "car", "Van": "car", "Truck": "truck", "Pedestrian": "pedestrian", "Cyclist": "bicycle"},
    "rope3d": {"car": "car", "van": "car", "bus": "bus", "truck": "truck", "pedestrian": "pedestrian",
               "cyclist": "bicycle", "motorcyclist": "motorcycle"},
    "dair": {"Car": "car", "Van": "car", "Bus": "bus", "Truck": "truck", "Pedestrian": "pedestrian",
             "Cyclist": "bicycle", "Motorcyclist": "motorcycle"},
}
IGNORE_TYPES = {"DontCare", "Misc", "Tram", "Person_sitting", "tricyclist", "barrow", "Tricyclist", "Barrowlist"}


def read_calib(path: Path) -> np.ndarray:
    for line in path.read_text().splitlines():
        if line.startswith("P2:"):
            return np.array(line.split()[1:13], dtype=float).reshape(3, 4)
    # Rope3D / DAIR: a single 3x4 or 3x3 matrix without key.
    vals = np.array(path.read_text().split()[-12:], dtype=float)
    return vals.reshape(3, 4)


def fit_plane_from_bottoms(bottoms: list[np.ndarray], default_h: float) -> np.ndarray:
    up = np.array([0.0, -1.0, 0.0])
    if len(bottoms) >= 4:
        try:
            p = G.fit_plane(np.array(bottoms), up, iters=100, thresh=0.2)
            if p[:3] @ up > 0.95 and 0.5 < p[3] < 30:
                return p
        except Exception:
            pass
    return np.array([0.0, -1.0, 0.0, default_h])


def convert(source: str, root: Path, splits: list[str], cam_height: float):
    cmap = MAPS[source]
    for split in splits:
        sd = root / split
        for lab in sorted((sd / "label_2").glob("*.txt")):
            stem = lab.stem
            img = next((p for ext in (".png", ".jpg") if (p := sd / "image_2" / (stem + ext)).exists()), None)
            if img is None:
                continue
            P = read_calib(sd / "calib" / f"{stem}.txt")
            K = P[:, :3] / P[2, 2]
            t = np.linalg.solve(P[:, :3], P[:, 3])  # reference camera -> this camera offset
            objs, ignore, bottoms = [], [], []
            for line in lab.read_text().splitlines():
                f = line.split()
                if len(f) < 15:
                    continue
                typ = f[0]
                box = [float(v) for v in f[4:8]]
                if typ in IGNORE_TYPES or typ not in cmap:
                    ignore.append(box)
                    continue
                h, w, l = map(float, f[8:11])
                loc = np.array(f[11:14], dtype=float) + t
                ry = float(f[14])
                heading = np.array([np.cos(ry), 0.0, -np.sin(ry)])
                objs.append({"cls": cmap[typ], "src_type": typ, "bottom": loc, "dims": [l, w, h], "heading": heading,
                             "box2d": box, "truncation": float(f[1]), "occlusion": f[2]})
                bottoms.append(loc)
            dn = sd / "denorm" / f"{stem}.txt"
            if dn.exists():
                a = np.array(dn.read_text().split()[:4], dtype=float)
                n = a[:3] / np.linalg.norm(a[:3])
                d = a[3] / np.linalg.norm(a[:3])
                if n[1] > 0:  # orient normal upwards (camera y points down)
                    n, d = -n, -d
                plane = np.concatenate([n, [d]])
            else:
                plane = fit_plane_from_bottoms(bottoms, cam_height)
            n = plane[:3]
            out_objs = []
            for o in objs:
                hd = o["heading"] - (o["heading"] @ n) * n
                hd /= np.linalg.norm(hd)
                out_objs.append({**o, "bottom": o["bottom"].tolist(), "heading": hd.tolist()})
            import cv2  # lazy: only for image size

            im = cv2.imread(str(img))
            yield {"source": source, "sequence": split, "group": f"{source}:{split}:{int(stem) // 50 if stem.isdigit() else stem}",
                   "image": str(img.relative_to(root)), "camera": source, "width": im.shape[1], "height": im.shape[0],
                   "K": K.tolist(), "plane": plane.tolist(), "ignore": ignore, "objects": out_objs}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", choices=list(MAPS), required=True)
    ap.add_argument("--root", type=Path, required=True)
    ap.add_argument("--splits", default="training")
    ap.add_argument("--cam_height", type=float, default=1.65)
    ap.add_argument("--groups", type=Path, default=None, help="optional file: <stem> <group id> per line (e.g. KITTI drive)")
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    groups = {}
    if a.groups:
        for line in a.groups.read_text().splitlines():
            k, g = line.split()[:2]
            groups[k] = g
    n = 0
    with open(a.out, "w") as f:
        for s in convert(a.source, a.root.expanduser(), a.splits.split(","), a.cam_height):
            stem = Path(s["image"]).stem
            if stem in groups:
                s["group"] = f"{a.source}:{groups[stem]}"
            f.write(json.dumps(s) + "\n")
            n += 1
    print(a.source, n, "images ->", a.out)


if __name__ == "__main__":
    main()
