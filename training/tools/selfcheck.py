"""Fast consistency checks of the training geometry (no GPU, no dataset)."""
from __future__ import annotations

import numpy as np

from jp3d import geometry as G
from jp3d.evaluate import evaluate


def main():
    rng = np.random.default_rng(0)
    # Projection decomposition round trip.
    K = np.array([[1300.0, 0, 950], [0, 1320, 600], [0, 0, 1]])
    a = rng.normal(size=3)
    R = np.linalg.qr(rng.normal(size=(3, 3)))[0]
    R *= np.sign(np.linalg.det(R))
    t = rng.normal(size=3) * 5
    P = K @ np.hstack([R, t[:, None]])
    K2, T2 = G.decompose_projection(P)
    assert np.allclose(K2, K, atol=1e-6) and np.allclose(T2[:3, :3], R, atol=1e-6) and np.allclose(T2[:3, 3], t, atol=1e-6)
    del a

    # Alpha <-> heading round trip on a tilted plane.
    n = G.normalize(np.array([0.05, -0.95, -0.3]))
    rays = rng.normal(size=(50, 3)) + np.array([0, 0, 5])
    heads = rng.normal(size=(50, 3))
    heads = G.normalize(heads - (heads @ n)[:, None] * n)
    alpha = G.heading_to_alpha(heads, rays, n)
    assert np.allclose(G.alpha_to_heading(alpha, rays, n), heads, atol=1e-9)

    # Ray-plane intersection lands on the plane.
    plane = np.concatenate([n, [7.0]])
    pts = G.ray_plane_intersect(rays, plane)
    ok = np.isfinite(pts).all(1)
    assert np.allclose(G.plane_height(plane, pts[ok]), 0, atol=1e-9)

    # Perfect predictions score AP 1.
    objs = [{"cls": "car", "bottom": [2.0, 7.0, 30.0 + i * 8], "dims": [4.5, 1.9, 1.5], "heading": [0, 0, 1.0],
             "box2d": [0, 0, 50, 40]} for i in range(5)]
    dets = [dict(o, score=0.9) for o in objs]
    res = evaluate([(objs, dets, plane)], classes=["car"])
    assert abs(res["mAP_dist"] - 1.0) < 1e-6, res["mAP_dist"]
    print("selfcheck ok")


if __name__ == "__main__":
    main()
