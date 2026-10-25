"""Convert TUMTraf (Intersection R02 and V2X R04) into the JP3D camera-frame sample format.

Every output sample is one (image, camera) pair:

    {
      "image": "<path relative to dataset root>",
      "camera": "s110_camera_basler_south1_8mm",
      "frame_id": "<label file stem>",
      "width": 1920, "height": 1200,
      "K": 3x3,                      # intrinsics of the stored image
      "T_cam_from_ref": 4x4,         # reference frame = the LiDAR frame the labels are in
      "plane": [nx, ny, nz, d],      # ground plane in camera frame, normal up
      "objects": [{
          "cls": "car", "type": "VAN",
          "bottom": [x, y, z],       # camera frame, bottom centre of the box
          "dims": [l, w, h],
          "heading": [hx, hy, hz],   # camera frame unit vector
          "box2d": [x1, y1, x2, y2], # projected 3D box, clipped
          "trunc": 0.0..1.0, "num_points": int, "occlusion": str,
      }]
    }
"""
from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .. import geometry as G

# LiDAR -> image projection matrices. South-LiDAR projections for south1/south2 come from the dev-kit
# calibration files; north/east are the values published in the dev-kit visualisation utilities.
P_SOUTH_TO = {
    "s110_camera_basler_south1_8mm": [
        [7.04216073e02, -1.37317442e03, -4.32235765e02, -2.03369364e04],
        [-9.28351327e01, -1.77543929e01, -1.45629177e03, 9.80290034e02],
        [8.71736000e-01, -9.03453000e-02, -4.81574000e-01, -2.58546000e00],
    ],
    "s110_camera_basler_south2_8mm": [
        [1546.63215008, -436.92407115, -295.58362676, 1319.79271737],
        [93.20805656, 47.90351592, -1482.13403199, 687.84781276],
        [0.73326062, 0.59708904, -0.32528854, -1.30114325],
    ],
    "s110_camera_basler_east_8mm": [
        [-2666.70160799, -655.44528859, -790.96345758, -33010.77350141],
        [430.89231274, 66.06703744, -2053.70223986, 6630.65222157],
        [-0.00932524, -0.96164431, -0.27414094, 11.41820108],
    ],
    "s110_camera_basler_north_8mm": (
        np.array([[1360.68, 0, 849.369], [0, 1470.71, 632.174], [0, 0, 1]])
        @ np.array([
            [-0.564602, -0.824833, -0.0295815, -12.9358],
            [-0.458346, 0.343143, -0.819861, 7.22666],
            [0.686399, -0.449337, -0.571798, -6.75018],
        ])
    ).tolist(),
}
P_NORTH_TO = {
    "s110_camera_basler_south1_8mm": [
        [290.06440167, -1522.65885503, -417.08293461, -398.03125035],
        [-88.96283956, 6.86258619, -1451.78013497, 454.22071755],
        [0.81846385, -0.32818492, -0.47160557, -0.18257704],
    ],
    "s110_camera_basler_south2_8mm": [
        [1318.95273325, -859.15213894, -289.13390611, 11272.03223502],
        [90.01799314, -2.9727517, -1445.63809767, 585.78988153],
        [0.876766, 0.344395, -0.335669, -7.26891],
    ],
}
# Both S110 LiDARs are mounted roughly level, so +z of the LiDAR frame is a good "up" hint.
UP_HINT_REF = np.array([0.0, 0.0, 1.0])

IMG_W, IMG_H = 1920, 1200
_TS = re.compile(r"^(\d+)_(\d+)_")


def _timestamp(name: str) -> float:
    m = _TS.match(os.path.basename(name))
    return float(m.group(1)) + float(m.group(2)) * 1e-9


@dataclass
class Camera:
    name: str
    K: np.ndarray
    T_cam_from_ref: np.ndarray

    @staticmethod
    def from_projection(name: str, P) -> "Camera":
        K, T = G.decompose_projection(np.asarray(P, dtype=float))
        return Camera(name, K, T)


def parse_openlabel(path: Path) -> list[dict]:
    data = json.loads(Path(path).read_text())
    objs = []
    for frame in data["openlabel"]["frames"].values():
        for obj in frame["objects"].values():
            od = obj["object_data"]
            v = od["cuboid"]["val"]
            q = np.array(v[3:7], dtype=float)
            if np.linalg.norm(q) == 0:
                continue
            attrs = od["cuboid"].get("attributes", {})
            num = {a["name"]: a["val"] for a in attrs.get("num", [])}
            txt = {a["name"]: a["val"] for a in attrs.get("text", [])}
            objs.append({
                "type": od["type"].upper(),
                "center": np.array(v[0:3], dtype=float),
                "rot": Rotation.from_quat(q).as_matrix(),
                "dims": np.array(v[7:10], dtype=float),
                "num_points": int(float(num.get("num_points", -1) or -1)),
                "occlusion": str(txt.get("occlusion_level", "")),
            })
    return objs


def objects_to_camera(objs: list[dict], cam: Camera, plane_ref: np.ndarray) -> list[dict]:
    """Express reference-frame cuboids in the camera frame; keep those whose footprint centre is visible."""
    out = []
    R, t = cam.T_cam_from_ref[:3, :3], cam.T_cam_from_ref[:3, 3]
    n_ref = plane_ref[:3]
    n_cam = R @ n_ref
    for o in objs:
        cls = G.TUMTRAF_TO_CLASS.get(o["type"])
        if cls is None:
            continue
        l, w, h = o["dims"]
        # Bottom centre: drop the cuboid centre along the ground normal by h/2.
        bottom_ref = o["center"] - n_ref * (h / 2)
        head_ref = o["rot"][:, 0]
        bottom = R @ bottom_ref + t
        heading = R @ head_ref
        heading = heading - (heading @ n_cam) * n_cam
        if np.linalg.norm(heading) < 1e-6 or bottom[2] < 1.0:
            continue
        heading /= np.linalg.norm(heading)
        corners = G.box_corners(bottom[None], np.array([[l, w, h]]), heading[None], n_cam)[0]
        if (corners[:, 2] < 0.5).any():
            continue
        uv = G.project(cam.K, corners)
        x1, y1 = uv.min(0)
        x2, y2 = uv.max(0)
        cx1, cy1 = np.clip([x1, y1], 0, [IMG_W - 1, IMG_H - 1])
        cx2, cy2 = np.clip([x2, y2], 0, [IMG_W - 1, IMG_H - 1])
        if cx2 - cx1 < 2 or cy2 - cy1 < 2:
            continue
        area = max((x2 - x1) * (y2 - y1), 1e-6)
        trunc = 1.0 - (cx2 - cx1) * (cy2 - cy1) / area
        kp = G.project(cam.K, bottom[None])[0]
        if not (0 <= kp[0] < IMG_W and 0 <= kp[1] < IMG_H):
            continue
        out.append({
            "cls": cls,
            "type": o["type"],
            "bottom": bottom.tolist(),
            "dims": [float(l), float(w), float(h)],
            "heading": heading.tolist(),
            "box2d": [float(cx1), float(cy1), float(cx2), float(cy2)],
            "trunc": float(trunc),
            "num_points": o["num_points"],
            "occlusion": o["occlusion"],
        })
    return out


def dedup(objs: list[dict], dist: float = 1.0) -> list[dict]:
    """Merge duplicates coming from overlapping LiDAR label sets (same class, close bottoms)."""
    keep: list[dict] = []
    for o in objs:
        b = np.array(o["bottom"])
        if any(k["cls"] == o["cls"] and np.linalg.norm(np.array(k["bottom"]) - b) < dist for k in keep):
            continue
        keep.append(o)
    return keep


def nearest_image(images: list[str], stamps: np.ndarray, ts: float, tol: float = 0.06) -> str | None:
    i = int(np.argmin(np.abs(stamps - ts)))
    return images[i] if abs(stamps[i] - ts) <= tol else None
