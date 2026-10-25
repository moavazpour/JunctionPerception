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



def image_tracks(frames, min_score=0.45):
    """IoU tracker in image space (same as the C++ AutoCalibrator)."""
    def iou(a, b):
        x0, y0, x1, y1 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
        i = max(0, x1 - x0) * max(0, y1 - y0)
        u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - i
        return i / u if u > 0 else 0
    live, done = [], []
    for f in frames:
        ds = [d for d in f["dets"] if d["score"] >= min_score and d["cls"] in VEH]
        used = set()
        for tr in live:
            best, bi = 0.3, -1
            for j, d in enumerate(ds):
                if j in used or d["cls"] != tr[-1]["cls"]:
                    continue
                v = iou(tr[-1]["box2d"], d["box2d"])
                if v > best:
                    best, bi = v, j
            if bi >= 0:
                used.add(bi)
                tr.append({**ds[bi], "t": f["t"]})
        for j, d in enumerate(ds):
            if j not in used:
                live.append([{**d, "t": f["t"]}])
        keep = []
        for tr in live:
            (keep if f["t"] - tr[-1]["t"] <= 0.5 else done).append(tr)
        live = keep
    return [t for t in done + live if len(t) >= 10]


def vp_quality(tracks, K, plane):
    """Angle of each straight track's image line to the ground-plane horizon under the given camera."""
    n = plane[:3]
    out = []
    for t in tracks:
        p = np.array([d["kp"] for d in t])
        if np.linalg.norm(p[-1] - p[0]) < 0.04 * 1920:
            continue
        g = G.ray_plane_intersect(G.pixel_rays(K, p), plane)
        ok = np.isfinite(g).all(1)
        if ok.sum() < 5:
            continue
        g = g[ok]
        dvec = g[-1] - g[0]
        out.append(np.degrees(np.arcsin(abs(dvec @ n) / max(np.linalg.norm(dvec), 1e-6))))
    return out



def extra_report(path, Kgt, pgt):
    frames, _ = load(path)
    tr = image_tracks(frames)
    print(f"{len(tr)} image tracks (>=10 frames)")
    # Directions of straight tracks on the GT ground: how many distinct road directions does traffic use?
    yaws = []
    n = pgt[:3]
    for t in tr:
        p = np.array([d["kp"] for d in t])
        g = G.ray_plane_intersect(G.pixel_rays(Kgt, p), pgt)
        g = g[np.isfinite(g).all(1)]
        if len(g) < 5 or np.linalg.norm(g[-1] - g[0]) < 5:
            continue
        dv = g[-1] - g[0]
        e1 = G.normalize(np.cross(n, [1.0, 0, 0]))
        e2 = np.cross(n, e1)
        yaws.append(np.degrees(np.arctan2(dv @ e2, dv @ e1)) % 180)
    h, edges = np.histogram(yaws, bins=18, range=(0, 180))
    print("track direction histogram (10 deg bins, mod 180):", h.tolist())


if __name__ == "__main__":
    import sys
    main()
    a = sys.argv
    fx, fy, cx, cy = map(float, [x for x in a if x.startswith("--K=")][0][4:].split(","))
    pl = np.array(list(map(float, [x for x in a if x.startswith("--plane=")][0][8:].split(","))))
    extra_report([x for x in a if x.endswith(".jsonl")][0], np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]]), pl)
