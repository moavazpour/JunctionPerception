"""Camera / ground-plane geometry shared by data conversion, training targets and evaluation.

Conventions
-----------
* Camera frame: OpenCV (x right, y down, z forward).
* Ground plane in camera frame: ``n . X + d = 0`` with unit normal ``n`` pointing *up* (away from the road).
  The signed height of a point above the ground is ``n . X + d``.
* A 3D box is described by its *bottom centre* B (centre of the footprint, on/near the ground),
  dimensions (l, w, h) and a heading unit vector lying in the ground plane.
* The observation angle ``alpha`` is the angle, measured in the ground plane around ``n``, from the
  viewing ray through B (projected on the plane) to the heading. It is what the image appearance
  determines, independently of where the camera is mounted.
"""
from __future__ import annotations

import numpy as np

CLASSES = ["car", "bus", "truck", "trailer", "pedestrian", "bicycle", "escooter", "motorcycle"]

# Prior dimensions (l, w, h) in metres, refined from TUMTraf statistics.
CLASS_MEAN_DIMS = {
    "car": (4.45, 1.90, 1.60),
    "bus": (12.0, 2.60, 3.20),
    "truck": (6.80, 2.50, 3.20),
    "trailer": (10.5, 2.60, 3.40),
    "pedestrian": (0.65, 0.65, 1.72),
    "bicycle": (1.75, 0.65, 1.70),
    "escooter": (1.15, 0.60, 1.80),
    "motorcycle": (2.10, 0.85, 1.60),
}

TUMTRAF_TO_CLASS = {
    "CAR": "car",
    "VAN": "car",
    "EMERGENCY_VEHICLE": "car",
    "BUS": "bus",
    "TRUCK": "truck",
    "TRAILER": "trailer",
    "PEDESTRIAN": "pedestrian",
    "BICYCLE": "bicycle",
    "MOTORCYCLE": "motorcycle",
}


def normalize(v: np.ndarray) -> np.ndarray:
    return v / np.linalg.norm(v, axis=-1, keepdims=True)


def plane_height(plane: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """Signed height of points (N,3) above plane (4,)."""
    return pts @ plane[:3] + plane[3]


def transform_plane(plane: np.ndarray, T: np.ndarray) -> np.ndarray:
    """Transform plane (n, d) defined in frame A into frame B given T_B_from_A (4x4)."""
    R, t = T[:3, :3], T[:3, 3]
    n = R @ plane[:3]
    d = plane[3] - n @ t
    return np.concatenate([n, [d]])


def pixel_rays(K: np.ndarray, uv: np.ndarray) -> np.ndarray:
    """Unnormalised camera-frame ray directions (N,3) for pixels (N,2)."""
    uv1 = np.concatenate([uv, np.ones((len(uv), 1))], axis=1)
    return uv1 @ np.linalg.inv(K).T


def ray_plane_intersect(rays: np.ndarray, plane: np.ndarray, height: np.ndarray | float = 0.0) -> np.ndarray:
    """Intersect rays from the camera origin with the plane offset by ``height`` along its normal.

    Returns (N,3) points; rays that do not hit the plane in front of the camera produce NaN.
    """
    n, d = plane[:3], plane[3]
    denom = rays @ n
    t = (np.asarray(height) - d) / denom
    pts = rays * t[:, None]
    pts[(t <= 0) | ~np.isfinite(t)] = np.nan
    return pts


def ray_basis_on_plane(rays: np.ndarray, n: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """In-plane forward axis (ray projected on plane) and its left-hand perpendicular n x r."""
    r = rays - (rays @ n)[:, None] * n[None, :]
    r = normalize(r)
    s = np.cross(n[None, :], r)
    return r, s


def heading_to_alpha(heading: np.ndarray, rays: np.ndarray, n: np.ndarray) -> np.ndarray:
    r, s = ray_basis_on_plane(rays, n)
    return np.arctan2((heading * s).sum(-1), (heading * r).sum(-1))


def alpha_to_heading(alpha: np.ndarray, rays: np.ndarray, n: np.ndarray) -> np.ndarray:
    r, s = ray_basis_on_plane(rays, n)
    return np.cos(alpha)[:, None] * r + np.sin(alpha)[:, None] * s


def box_corners(bottom: np.ndarray, dims: np.ndarray, heading: np.ndarray, n: np.ndarray) -> np.ndarray:
    """8 corners (N,8,3). Corners 0-3 on the ground (front-left, front-right, rear-right, rear-left), 4-7 on top."""
    x = normalize(heading - (heading @ n)[:, None] * n[None, :])
    y = np.cross(n[None, :], x)
    z = np.broadcast_to(n, x.shape)
    l, w, h = dims[:, 0:1], dims[:, 1:2], dims[:, 2:3]
    signs = np.array([[1, 1], [1, -1], [-1, -1], [-1, 1]], dtype=float)
    corners = []
    for top in (0.0, 1.0):
        for sx, sy in signs:
            corners.append(bottom + sx * l / 2 * x + sy * w / 2 * y + top * h * z)
    return np.stack(corners, axis=1)


def project(K: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """Project camera-frame points (...,3) to pixels (...,2)."""
    p = pts @ K.T
    return p[..., :2] / p[..., 2:3]


def rq3(M: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """RQ decomposition of a 3x3 matrix with positive diagonal of R."""
    P = np.flipud(np.eye(3))
    Q, R = np.linalg.qr((P @ M).T)
    R = P @ R.T @ P
    Q = P @ Q.T
    D = np.diag(np.sign(np.diag(R)))
    return R @ D, D @ Q


def decompose_projection(P: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Split a 3x4 projection matrix into K (3x3, K[2,2]=1) and T_cam_from_world (4x4)."""
    if np.linalg.det(P[:, :3]) < 0:
        P = -P
    K, R = rq3(P[:, :3])
    t = np.linalg.solve(K, P[:, 3])
    K = K / K[2, 2]
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return K, T


def fit_plane(pts: np.ndarray, up_hint: np.ndarray, iters: int = 200, thresh: float = 0.15,
              rng: np.random.Generator | None = None) -> np.ndarray:
    """Robust plane fit (RANSAC + least-squares refinement). Normal oriented along ``up_hint``."""
    rng = rng or np.random.default_rng(0)
    best, best_inl = None, -1
    for _ in range(iters):
        s = pts[rng.choice(len(pts), 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0])
        if np.linalg.norm(n) < 1e-9:
            continue
        n = n / np.linalg.norm(n)
        d = -n @ s[0]
        inl = (np.abs(pts @ n + d) < thresh).sum()
        if inl > best_inl:
            best, best_inl = (n, d), inl
    n, d = best
    inliers = pts[np.abs(pts @ n + d) < thresh]
    c = inliers.mean(0)
    _, _, vt = np.linalg.svd(inliers - c)
    n = vt[-1]
    if n @ up_hint < 0:
        n = -n
    return np.concatenate([n, [-n @ c]])
