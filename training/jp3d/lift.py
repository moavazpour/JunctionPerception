"""Lift calibration-free network detections to 3D with intrinsics + ground plane (reference implementation
of what the C++ runtime does)."""
from __future__ import annotations

import numpy as np

from . import geometry as G


def lift(dets: np.ndarray, K: np.ndarray, plane: np.ndarray, score_thr: float = 0.1) -> list[dict]:
    """dets [N,13] from DeployWrapper (pixels in the image that K describes)."""
    dets = dets[dets[:, 0] >= score_thr]
    if len(dets) == 0:
        return []
    uv = dets[:, 2:4]
    rays = G.pixel_rays(K, uv)
    bottoms = G.ray_plane_intersect(rays, plane, height=dets[:, 8])
    n = plane[:3]
    heads = G.alpha_to_heading(dets[:, 12], rays, n)
    out = []
    for i, d in enumerate(dets):
        if not np.isfinite(bottoms[i]).all():
            continue
        out.append({
            "score": float(d[0]),
            "cls": G.CLASSES[int(d[1])],
            "bottom": bottoms[i].tolist(),
            "dims": d[9:12].tolist(),
            "heading": heads[i].tolist(),
            "kp": uv[i].tolist(),
            "box2d": [float(d[2] - d[4]), float(d[3] - d[5]), float(d[2] + d[6]), float(d[3] + d[7])],
        })
    return out
