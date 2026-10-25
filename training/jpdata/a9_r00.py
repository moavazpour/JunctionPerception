"""TUMTraf A9 Highway R00 -> JP-Unified jsonl.

R00 labels: box3d.location (bottom centre, z = 0 on the highway plane) in the S40/S50 world frame,
dimension l w h, rotationYaw about world z. Calibration per camera: projection_matrix world -> image.
R00 sequences s03/s04 contain LiDAR only and are skipped. R01 camera labels are annotator-drawn
pseudo-cuboids (vertical edges forced vertical, parallel faces), not metric projections, so they are
not used (see docs/DATASET.md).

    python -m jpdata.a9_r00 --root ~/data/junction_perception/raw/TUMTraf/a9x --out a9_r00.jsonl
"""
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import numpy as np

from jp3d import geometry as G

CMAP = {"CAR": "car", "VAN": "car", "EMERGENCY_VEHICLE": "car", "BUS": "bus", "TRUCK": "truck", "TRAILER": "trailer",
        "PEDESTRIAN": "pedestrian", "BICYCLE": "bicycle", "MOTORCYCLE": "motorcycle"}


def cam_key(name: str) -> str:
    m = re.search(r"(s\d+_camera_basler_\w+?_\d+mm)", name)
    return m.group(1).replace("s040", "s40").replace("s050", "s50") if m else ""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    root = a.root.expanduser()
    n = 0
    with open(a.out, "w") as f:
        for seq in sorted(root.glob("a9_dataset_r00_s0*")):
            if not (seq / "_images").exists():
                continue
            calibs = {}
            for c in (seq / "_calibration").glob("*.json"):
                d = json.loads(c.read_text())
                P = np.array(d["projection_matrix"], float)
                K, T = G.decompose_projection(P)
                calibs[cam_key(c.stem)] = (K, T, d.get("image_width", 1920), d.get("image_height", 1200))
            for lab in sorted((seq / "_labels").glob("*.json")):
                d = json.loads(lab.read_text())
                img = seq / "_images" / d["image_file_name"]
                ck = cam_key(d["image_file_name"])
                if not img.exists() or ck not in calibs:
                    continue
                K, T, W, H = calibs[ck]
                R, t = T[:3, :3], T[:3, 3]
                up = R @ np.array([0.0, 0.0, 1.0])
                # World plane z = 0 in camera coordinates.
                plane = G.transform_plane(np.array([0.0, 0.0, 1.0, 0.0]), T)
                objs, ignore = [], []
                for l in d["labels"]:
                    b = l["box3d"]
                    typ = l["category"].upper()
                    loc = np.array([b["location"][k] for k in "xyz"], float)
                    yaw = float(b["orientation"]["rotationYaw"])
                    dims = [float(b["dimension"][k]) for k in ("length", "width", "height")]
                    bottom = R @ loc + t
                    head = R @ np.array([np.cos(yaw), np.sin(yaw), 0.0])
                    head = head - (head @ up) * up
                    head /= np.linalg.norm(head)
                    c = G.box_corners(bottom[None], np.array([dims]), head[None], up)[0]
                    if (c[:, 2] < 0.5).any():
                        continue
                    uv = G.project(K, c)
                    x1, y1 = np.clip(uv.min(0), 0, [W - 1, H - 1])
                    x2, y2 = np.clip(uv.max(0), 0, [W - 1, H - 1])
                    if x2 - x1 < 2 or y2 - y1 < 2:
                        continue
                    box = [float(x1), float(y1), float(x2), float(y2)]
                    if typ not in CMAP:
                        ignore.append(box)
                        continue
                    objs.append({"cls": CMAP[typ], "src_type": typ, "bottom": bottom.tolist(), "dims": dims,
                                 "heading": head.tolist(), "box2d": box})
                f.write(json.dumps({
                    "source": "a9_r00", "sequence": seq.name, "group": f"a9_r00:{seq.name}:{ck}",
                    "image": str(img.relative_to(root)), "camera": ck, "width": W, "height": H,
                    "K": K.tolist(), "plane": plane.tolist(), "ignore": ignore, "objects": objs,
                    "weather": d.get("weather_type", "")}) + "\n")
                n += 1
    print("a9_r00", n, "samples ->", a.out)


if __name__ == "__main__":
    main()
