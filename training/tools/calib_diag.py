"""Diagnose auto-calibration on dumped calibration-free detections (jp_bench --video) against a known
camera: evaluates the calibration cost terms at the ground truth and over a parameter sweep.

    python -m tools.calib_diag --dets tum_s03_south2_dets.jsonl --K 1315.6,1368.4,969.4,579.1 \
        --plane -0.0098,-0.9416,-0.3365,8.717
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from jp3d import geometry as G

VEH = {"car", "bus", "truck", "trailer", "motorcycle"}


def load(path, min_score=0.45, W=1920, H=1200):
    frames = [json.loads(l) for l in Path(path).read_text().splitlines()]
    dets = []
    for f in frames:
        for d in f["dets"]:
            b = d["box2d"]
            if d["score"] < min_score or d["cls"] not in VEH or b[0] < 3 or b[1] < 3 or b[2] > W - 3 or b[3] > H - 3:
                continue
            dets.append({**d, "t": f["t"]})
    return frames, dets


def box_cost(dets, K, plane):
    if not dets:
        return np.nan, np.nan
    kp = np.array([d["kp"] for d in dets])
    dz = np.array([d["dz"] for d in dets])
    dims = np.array([d["dims"] for d in dets])
    alpha = np.array([d["alpha"] for d in dets])
    box = np.array([d["box2d"] for d in dets])
    rays = G.pixel_rays(K, kp)
    bottom = G.ray_plane_intersect(rays, plane, dz)
    ok = np.isfinite(bottom).all(1)
    n = plane[:3]
    head = G.alpha_to_heading(alpha, rays, n)
    res = np.ones(len(dets))
    c = G.box_corners(bottom[ok], dims[ok], head[ok], n)
    front = (c[..., 2] > 0.3).all(1)
    uv = G.project(K, c)
    bh = box[ok, 3] - box[ok, 1]
    rt = (uv[..., 1].min(1) - box[ok, 1]) / bh
    rl = (uv[..., 0].min(1) - box[ok, 0]) / bh
    rr = (uv[..., 0].max(1) - box[ok, 2]) / bh
    r = np.sqrt(rt ** 2 + 0.25 * (rl ** 2 + rr ** 2))
    r[~front] = 1.0
    res[ok] = r
    cost = np.log1p((res / 0.1) ** 2).mean()
    return cost, np.median(res)


def pose_plane(f, pitch, roll, h):
    c, s = np.cos(pitch), np.sin(pitch)
    n = np.array([c * np.sin(roll), -c * np.cos(roll), -s])
    return np.array([[f, 0, 959.5], [0, f, 599.5], [0, 0, 1]]), np.concatenate([n, [h]])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dets", required=True)
    ap.add_argument("--K", required=True)
    ap.add_argument("--plane", required=True)
    a = ap.parse_args()
    fx, fy, cx, cy = map(float, a.K.split(","))
    Kgt = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    pgt = np.array(list(map(float, a.plane.split(","))))
    frames, dets = load(a.dets)
    print(f"{len(frames)} frames, {len(dets)} confident vehicle detections")
    c, med = box_cost(dets, Kgt, pgt)
    print(f"GT camera: cost {c:.3f}, median box residual {med:.3f}")
    gt_pitch = np.degrees(np.arcsin(-pgt[2]))
    print(f"GT pitch {gt_pitch:.2f}, height {pgt[3]:.2f}")
    best = None
    for f in np.geomspace(700, 4000, 30):
        for pitch in np.arange(4, 32, 1.0):
            for h in np.arange(3, 14, 0.5):
                K, p = pose_plane(f, np.radians(pitch), 0.0, h)
                cc, m = box_cost(dets, K, p)
                if best is None or cc < best[0]:
                    best = (cc, f, pitch, h, m)
    print("best box-only fit: cost %.3f f %.0f pitch %.1f h %.1f med %.3f" % best)
    for f in (1340, 1800, 2360):
        row = []
        for pitch in (8.6, 14, 19.7, 25):
            K, p = pose_plane(f, np.radians(pitch), 0.0, 8.7)
            row.append(f"{box_cost(dets, K, p)[0]:.3f}")
        print(f"f={f}: cost at pitch 8.6/14/19.7/25 (h 8.7) =", row)


if __name__ == "__main__":
    main()
