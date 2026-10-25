"""Lift projected-cuboid keypoints to a 3D box on the ground plane.

Footprint corners are intersected with the ground plane directly, so position, heading, length and
width need no size prior; height comes from the top corners. Missing corners fall back to the bottom
centre keypoint and class priors.
"""
from __future__ import annotations

import numpy as np

from jp3d import geometry as G

KP_CONF = 0.35


def _height_from_tops(K, n, bottoms, tops_uv, h_prior):
    """1D search for the box height that best reprojects the visible top corners."""
    if not tops_uv:
        return h_prior
    hs = np.linspace(0.3, 5.0, 236)
    best, best_e = h_prior, 1e18
    for h in hs:
        e = 0.0
        for b, uv in zip(bottoms, tops_uv):
            p = G.project(K, (b + n * h)[None])[0]
            e += float(((p - uv) ** 2).sum())
        if e < best_e:
            best, best_e = h, e
    return float(best)


def lift_keypoints(kpts: np.ndarray, cls: str, score: float, K: np.ndarray, plane: np.ndarray) -> dict | None:
    """kpts: (9, 3) pixel u, v and confidence. Returns a camera-frame object or None."""
    n = plane[:3]
    prior = np.array(G.CLASS_MEAN_DIMS[cls])
    vis = kpts[:, 2] >= KP_CONF
    rays = G.pixel_rays(K, kpts[:, :2])
    ground = G.ray_plane_intersect(rays, plane)
    ok = vis & np.isfinite(ground).all(1)
    fl, fr, rr, rl = 1, 2, 3, 4
    corners = {i: ground[i] for i in (fl, fr, rr, rl) if ok[i]}
    # Complete a missing corner of the footprint parallelogram.
    if len(corners) == 3:
        order = [fl, fr, rr, rl]
        miss = next(i for i in order if i not in corners)
        k = order.index(miss)
        a, b, c = corners[order[(k + 1) % 4]], corners[order[(k + 2) % 4]], corners[order[(k + 3) % 4]]
        corners[miss] = a + c - b
    if len(corners) == 4:
        front = (corners[fl] + corners[fr]) / 2
        rear = (corners[rl] + corners[rr]) / 2
        head = front - rear
        length = float(np.linalg.norm(head))
        width = float((np.linalg.norm(corners[fl] - corners[fr]) + np.linalg.norm(corners[rl] - corners[rr])) / 2)
        bottom = (front + rear) / 2
        # Implausible footprints (keypoint noise far away) fall back to priors around the centre.
        if not (0.3 * prior[0] < length < 2.5 * prior[0] and 0.3 * prior[1] < width < 2.5 * prior[1]):
            length, width = prior[0], prior[1]
    else:
        if not ok[0]:
            return None
        bottom = ground[0]
        pairs = [(fl, rl), (fr, rr)]
        hv = [corners[a] - corners[b] for a, b in pairs if a in corners and b in corners]
        if hv:
            head = np.mean(hv, axis=0)
        else:
            # No usable corners: heading unknown, point along the viewing ray.
            r = rays[0] - (rays[0] @ n) * n
            head = r
        length, width = prior[0], prior[1]
    head = head - (head @ n) * n
    if np.linalg.norm(head) < 1e-6:
        return None
    head = head / np.linalg.norm(head)
    # Height from the top corners above the corresponding bottom corners.
    c4 = G.box_corners(bottom[None], np.array([[length, width, prior[2]]]), head[None], n)[0]
    bots, tops = [], []
    for j in range(4):
        if vis[5 + j]:
            bots.append(c4[j])
            tops.append(kpts[5 + j, :2])
    h = _height_from_tops(K, n, bots, tops, prior[2])
    return {"cls": cls, "score": float(score), "bottom": bottom.tolist(), "dims": [length, width, h],
            "heading": head.tolist()}
