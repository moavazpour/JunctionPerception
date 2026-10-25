"""LUMPI (Leibniz University Multi-Perspective Intersection) -> JP-Unified jsonl.

Conventions (from the official lumpi-sdk-python):
  * Label.csv: time, id, BEV rect(4), score, class_id, visibility(points), centre x y z (world, z up),
    l w h, heading (rad about world z). Classes 0 pedestrian, 1 car, 2 bicycle, 3 motorcycle, 4 bus,
    5 truck, 6 van, 7 unknown.
  * meta.json session[...]["extrinsic"] is camera -> world (the SDK inverts it); intrinsic and
    distortion are applied on the raw video. frame = floor(time * fps), videos start at t = 0.
Frames are sampled every --every seconds, undistorted, and the labels expressed in the rectified camera.
The mask video gives a foreground mask: objects whose projected box is < 10 % foreground are hidden
behind something and become ignore regions.

    python -m jpdata.lumpi --labels ~/data/junction_perception/raw/LUMPI/x/Label \
        --videos M0=~/data/.../LUMPI/v/1,M1=~/data/.../v/2,... --out lumpi.jsonl --undist ~/data/junction_perception/undist/lumpi
"""
from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

from jp3d import geometry as G

CMAP = {0: "pedestrian", 1: "car", 2: "bicycle", 3: "motorcycle", 4: "bus", 5: "truck", 6: "car"}
MEAS_DIR = {"M0": "Measurment0", "M1": "Measurement1", "M2": "Measurement2", "M3": "Measurement3",
            "M4": "Measurement4", "M5": "Measurement5", "M6": "Measurement6"}


def load_labels(path: Path):
    by_t = defaultdict(list)
    with open(path) as fh:
        r = csv.reader(fh)
        next(r)
        for row in r:
            v = [float(x) for x in row[:16]]
            by_t[round(v[0], 1)].append(v)
    return by_t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--labels", type=Path, required=True)
    ap.add_argument("--videos", required=True, help="M0=dir,M1=dir,... (dir contains cam/<id>/video.mp4)")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--undist", type=Path, required=True)
    ap.add_argument("--every", type=float, default=2.0)
    ap.add_argument("--min_fg", type=float, default=0.10)
    a = ap.parse_args()
    meta = json.loads((a.labels / "meta.json").read_text())
    out_dir = a.undist.expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    n_written = 0
    stats = defaultdict(int)
    with open(a.out, "w") as f:
        for spec in a.videos.split(","):
            mkey, vdir = spec.split("=")
            vdir = Path(vdir).expanduser()
            mdir = a.labels / MEAS_DIR[mkey]
            exp = meta["experiment"][mkey[1:]]
            labels = load_labels(mdir / "Label.csv")
            times = sorted(labels)
            # Ground plane (world): robust plane through all box bottoms of this measurement.
            bottoms = np.array([[v[9], v[10], v[11] - v[14] / 2] for t in times[:: max(1, len(times) // 400)] for v in labels[t]])
            plane_w = G.fit_plane(bottoms, np.array([0.0, 0.0, 1.0]), iters=200, thresh=0.2)
            for dev_dir in sorted((vdir / "cam").iterdir()):
                dev = dev_dir.name
                if dev not in exp:
                    continue
                sess = meta["session"][str(exp[dev])]
                if sess.get("type") != "camera":
                    continue
                K = np.array(sess["intrinsic"], float)
                D = np.array(sess["distortion"], float).reshape(-1)
                T_wc = np.array(sess["extrinsic"], float)  # camera -> world
                T_cw = np.linalg.inv(T_wc)
                fps = float(sess["fps"])
                R, t = T_cw[:3, :3], T_cw[:3, 3]
                plane = G.transform_plane(plane_w, T_cw)
                n = plane[:3]
                cap = cv2.VideoCapture(str(dev_dir / "video.mp4"))
                W, H = int(cap.get(3)), int(cap.get(4))
                nframes = int(cap.get(7))
                m1, m2 = cv2.initUndistortRectifyMap(K, D, np.eye(3), K, (W, H), cv2.CV_16SC2)
                mask_path = mdir / "cam" / dev / "mask.mp4"
                mcap = cv2.VideoCapture(str(mask_path)) if mask_path.exists() else None
                t_s = 0.0
                while t_s <= times[-1]:
                    tk = round(t_s, 1)
                    t_s += a.every
                    frame = int(np.floor(tk * fps))
                    if frame >= nframes or tk not in labels:
                        continue
                    cap.set(cv2.CAP_PROP_POS_FRAMES, frame)
                    ok, img = cap.read()
                    if not ok:
                        continue
                    fg = None
                    if mcap is not None:
                        mcap.set(cv2.CAP_PROP_POS_FRAMES, frame)
                        okm, mk = mcap.read()
                        if okm:
                            fg = cv2.remap((mk.max(axis=2) > 60).astype(np.uint8), m1, m2, cv2.INTER_NEAREST)
                    und = cv2.remap(img, m1, m2, cv2.INTER_LINEAR)
                    objs, ignore = [], []
                    for v in labels[tk]:
                        cid = int(v[7])
                        c_w = np.array(v[9:12])
                        l, w, h = v[12:15]
                        yaw = v[15]
                        bottom = R @ (c_w - np.array([0, 0, h / 2])) + t
                        head = R @ np.array([np.cos(yaw), np.sin(yaw), 0.0])
                        head = head - (head @ n) * n
                        if np.linalg.norm(head) < 1e-6:
                            continue
                        head /= np.linalg.norm(head)
                        cs = G.box_corners(bottom[None], np.array([[l, w, h]]), head[None], n)[0]
                        if (cs[:, 2] < 0.5).any():
                            continue
                        uv = G.project(K, cs)
                        x1, y1 = np.clip(uv.min(0), 0, [W - 1, H - 1])
                        x2, y2 = np.clip(uv.max(0), 0, [W - 1, H - 1])
                        if x2 - x1 < 3 or y2 - y1 < 3:
                            continue
                        box = [float(x1), float(y1), float(x2), float(y2)]
                        if cid not in CMAP:
                            ignore.append(box)
                            continue
                        if fg is not None:
                            frac = float(fg[int(y1):int(y2) + 1, int(x1):int(x2) + 1].mean())
                            if frac < a.min_fg:
                                stats["hidden"] += 1
                                ignore.append(box)
                                continue
                        objs.append({"cls": CMAP[cid], "src_type": str(cid), "bottom": bottom.tolist(),
                                     "dims": [l, w, h], "heading": head.tolist(), "box2d": box,
                                     "num_points": int(v[8])})
                        stats["kept"] += 1
                    name = f"{mkey}_c{dev}_{frame:06d}.jpg"
                    cv2.imwrite(str(out_dir / name), und, [cv2.IMWRITE_JPEG_QUALITY, 92])
                    f.write(json.dumps({
                        "source": "lumpi", "sequence": f"{mkey}_c{dev}", "group": f"lumpi:{mkey}:c{dev}:{int(tk // 60)}",
                        "image": name, "camera": f"lumpi_c{dev}", "width": W, "height": H, "K": K.tolist(),
                        "plane": plane.tolist(), "ignore": ignore, "objects": objs, "time": tk}) + "\n")
                    n_written += 1
                print(f"{mkey} cam {dev}: done ({n_written} samples so far)", flush=True)
    print("lumpi", n_written, "samples", dict(stats))


if __name__ == "__main__":
    main()
