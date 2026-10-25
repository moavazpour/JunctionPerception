"""DAIR-V2X-C and V2X-Seq-SPD (infrastructure or vehicle side) -> JP-Unified jsonl.

DAIR images are raw (distorted, plumb_bob k1 ~ -0.45). Every image is undistorted with cam_K/cam_D into
the rectified camera P, and labels are expressed in that rectified camera:

  label/camera/<id>.json  : boxes in the (virtual) LiDAR frame: 3d_location = cuboid centre,
                            3d_dimensions l w h, rotation = yaw about LiDAR z (up)
  calib/camera_intrinsic  : cam_K, cam_D, P (rectified)
  calib/virtuallidar_to_camera (infrastructure) | calib/lidar_to_camera (vehicle): R, t

    python -m jpdata.dair --root .../V2X-Seq-SPD --side infrastructure-side --images .../V2X-Seq-SPD-infrastructure-side-image \
        --out dair_spd_infra.jsonl --undist ~/data/junction_perception/undist/dair_spd_infra
"""
from __future__ import annotations

import argparse
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import cv2
import numpy as np

from jp3d import geometry as G

CMAP = {"Car": "car", "Van": "car", "Bus": "bus", "Truck": "truck", "Pedestrian": "pedestrian",
        "Cyclist": "bicycle", "Motorcyclist": "motorcycle"}
IGNORE = {"Tricyclist", "Barrowlist", "Trafficcone", "TrafficCone", "Unknown", "Unknown_movable", "Unknown_unmovable"}


def orthonormal(R: np.ndarray) -> np.ndarray:
    u, _, vt = np.linalg.svd(R)
    R = u @ vt
    return R if np.linalg.det(R) > 0 else -R


def load_frame(root: Path, side: str, fid: str):
    lab = json.loads((root / side / "label" / "camera" / f"{fid}.json").read_text())
    intr = json.loads((root / side / "calib" / "camera_intrinsic" / f"{fid}.json").read_text())
    ext_dir = "virtuallidar_to_camera" if side.startswith("infra") else "lidar_to_camera"
    ext = json.loads((root / side / "calib" / ext_dir / f"{fid}.json").read_text())
    return lab, intr, ext


def undistort_one(args):
    src, dst, K, D, Knew, w, h = args
    if Path(dst).exists():
        return True
    im = cv2.imread(src)
    if im is None:
        return False
    m1, m2 = cv2.initUndistortRectifyMap(np.array(K), np.array(D), np.eye(3), np.array(Knew), (w, h), cv2.CV_16SC2)
    out = cv2.remap(im, m1, m2, cv2.INTER_LINEAR)
    Path(dst).parent.mkdir(parents=True, exist_ok=True)
    return cv2.imwrite(dst, out, [cv2.IMWRITE_JPEG_QUALITY, 93])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", type=Path, required=True, help="dir containing infrastructure-side/ vehicle-side/")
    ap.add_argument("--side", default="infrastructure-side")
    ap.add_argument("--images", type=Path, required=True, help="dir with <frame_id>.jpg")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--undist", type=Path, required=True)
    ap.add_argument("--name", default="dair")
    ap.add_argument("--workers", type=int, default=14)
    a = ap.parse_args()
    root, side = a.root.expanduser(), a.side
    info = {}
    di = root / side / "data_info.json"
    if di.exists():
        for e in json.loads(di.read_text()):
            fid = Path(e.get("image_path", e.get("frame_id", ""))).stem
            info[fid] = e
    fids = sorted(p.stem for p in (root / side / "label" / "camera").glob("*.json"))
    jobs, records = [], []
    up_lidar = np.array([0.0, 0.0, 1.0])
    for fid in fids:
        src = a.images.expanduser() / f"{fid}.jpg"
        if not src.exists():
            continue
        lab, intr, ext = load_frame(root, side, fid)
        K = np.array(intr["cam_K"], float).reshape(3, 3)
        D = np.array(intr["cam_D"], float)
        w, h = int(intr.get("width", 1920)), int(intr.get("height", 1080))
        if "P" in intr:
            Knew = np.array(intr["P"], float).reshape(3, 4)[:, :3]
        elif np.abs(D).sum() > 0:
            Knew, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 0.0)
        else:
            Knew = K.copy()
        R = orthonormal(np.array(ext["rotation"], float))
        t = np.array(ext["translation"], float).reshape(3)
        n_cam = R @ up_lidar
        objs, ignore, bottoms = [], [], []
        for o in lab:
            typ = o["type"]
            c = np.array([o["3d_location"][k] for k in "xyz"], float)
            l, wd, hh = (float(o["3d_dimensions"][k]) for k in "lwh")
            yaw = float(o["rotation"])
            centre = R @ c + t
            bottom = centre - n_cam * hh / 2
            head = R @ np.array([np.cos(yaw), np.sin(yaw), 0.0])
            corners = G.box_corners(bottom[None], np.array([[l, wd, hh]]), head[None], n_cam)[0]
            if (corners[:, 2] < 0.5).any():
                continue
            uv = G.project(Knew, corners)
            x1, y1 = np.clip(uv.min(0), 0, [w - 1, h - 1])
            x2, y2 = np.clip(uv.max(0), 0, [w - 1, h - 1])
            if x2 - x1 < 2 or y2 - y1 < 2:
                continue
            box = [float(x1), float(y1), float(x2), float(y2)]
            if typ not in CMAP:
                ignore.append(box)
                continue
            sb = o.get("2d_box")
            src_box = None
            if sb:
                pts = np.array([[sb["xmin"], sb["ymin"]], [sb["xmax"], sb["ymin"]], [sb["xmax"], sb["ymax"]], [sb["xmin"], sb["ymax"]],
                                [(sb["xmin"] + sb["xmax"]) / 2, sb["ymin"]], [(sb["xmin"] + sb["xmax"]) / 2, sb["ymax"]],
                                [sb["xmin"], (sb["ymin"] + sb["ymax"]) / 2], [sb["xmax"], (sb["ymin"] + sb["ymax"]) / 2]], float)
                und = cv2.undistortPoints(pts.reshape(-1, 1, 2), K, D, P=Knew).reshape(-1, 2)
                src_box = [float(und[:, 0].min()), float(und[:, 1].min()), float(und[:, 0].max()), float(und[:, 1].max())]
            objs.append({"cls": CMAP[typ], "src_type": typ, "bottom": bottom, "dims": [l, wd, hh], "heading": head,
                         "box2d": box, "src_box2d": src_box, "truncation": float(o.get("truncated_state", 0)),
                         "occlusion": str(o.get("occluded_state", ""))})
            bottoms.append(bottom)
        e = info.get(fid, {})
        seq = e.get("sequence_id") or e.get("batch_id") or e.get("intersection_loc") or "na"
        cam_id = intr.get("cameraID") or intr.get("serial_number", "cam")
        records.append({"fid": fid, "K": Knew, "n": n_cam, "objs": objs, "ignore": ignore, "bottoms": bottoms,
                        "w": w, "h": h, "seq": str(seq), "cam": str(cam_id), "loc": str(e.get("intersection_loc", ""))})
        dst = str(a.undist.expanduser() / f"{fid}.jpg")
        jobs.append((str(src), dst, K.tolist(), D.tolist(), Knew.tolist(), w, h))
    # Ground plane per (camera, sequence): LiDAR up through the median bottom height (robust to slopes).
    planes = {}
    for r in records:
        planes.setdefault((r["cam"], r["seq"]), []).extend([(r["n"], b) for b in r["bottoms"]])
    plane_of = {}
    for k, nb in planes.items():
        if not nb:
            continue
        pts = np.array([b for _, b in nb])
        n = nb[0][0]
        try:
            p = G.fit_plane(pts, n, iters=150, thresh=0.25) if len(pts) >= 10 else None
        except Exception:
            p = None
        if p is None or p[:3] @ n < 0.97:
            p = np.concatenate([n, [-float(np.median(pts @ n))]])
        plane_of[k] = p
    print(f"undistorting {len(jobs)} images ...", flush=True)
    with ProcessPoolExecutor(a.workers) as ex:
        ok = list(ex.map(undistort_one, jobs, chunksize=32))
    print(f"undistorted {sum(ok)}/{len(ok)}")
    n_written = 0
    with open(a.out, "w") as f:
        for r in records:
            p = plane_of.get((r["cam"], r["seq"]))
            if p is None:
                p = np.concatenate([r["n"], [7.0]])
            n = p[:3]
            objs = []
            for o in r["objs"]:
                hd = o["heading"] - (o["heading"] @ n) * n
                hd /= np.linalg.norm(hd)
                objs.append({**o, "bottom": o["bottom"].tolist(), "heading": hd.tolist()})
            f.write(json.dumps({
                "source": a.name, "sequence": r["seq"], "group": f"{a.name}:{r['cam']}:{r['seq']}",
                "image": f"{r['fid']}.jpg", "camera": f"{a.name}_{r['cam']}", "width": r["w"], "height": r["h"],
                "K": r["K"].tolist(), "plane": p.tolist(), "ignore": r["ignore"], "objects": objs,
                "intersection": r["loc"]}) + "\n")
            n_written += 1
    print(a.name, n_written, "samples ->", a.out, "(image root:", a.undist, ")")


if __name__ == "__main__":
    main()
