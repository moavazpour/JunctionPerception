"""Unify the truck/trailer convention across sources.

DAIR, KITTI and LUMPI label an articulated truck as one box; TUMTraf and A9 label the tractor (TRUCK)
and each trailer (TRAILER) separately. JP-Unified follows the single-box convention: a trailer whose
front is close to a truck's rear (or to another trailer already attached) is merged into that truck's
box, and a stand-alone trailer is a truck as well.
"""
from __future__ import annotations

import numpy as np

from jp3d import geometry as G


def _footprint(o, n):
    return G.box_corners(np.array([o["bottom"]]), np.array([o["dims"]]), np.array([o["heading"]]), n)[0]


def _ends(o):
    b, h, l = np.array(o["bottom"]), np.array(o["heading"]), o["dims"][0]
    return b + h * l / 2, b - h * l / 2  # front, rear


def merge_truck_trailer(s: dict, max_gap: float = 3.5, max_angle_deg: float = 40.0) -> dict:
    objs = s["objects"]
    if not any(o["cls"] == "trailer" for o in objs) or any("kpts2d" in o for o in objs):
        for o in objs:
            if o["cls"] == "trailer":
                o["cls"] = "truck"
        return s
    n = np.array(s["plane"][:3])
    trucks = [o for o in objs if o["cls"] == "truck"]
    trailers = [o for o in objs if o["cls"] == "trailer"]
    others = [o for o in objs if o["cls"] not in ("truck", "trailer")]
    chains = {id(t): [t] for t in trucks}
    unattached = []
    # Attach trailers nearest-first so a road train (truck + 2 trailers) chains correctly.
    pending = sorted(trailers, key=lambda tr: min((np.linalg.norm(_ends(tr)[0] - _ends(t)[1]) for t in trucks), default=1e9))
    for tr in pending:
        best, best_d = None, max_gap
        for key, chain in chains.items():
            last = chain[-1]
            ang = np.degrees(np.arccos(np.clip(np.dot(last["heading"], tr["heading"]), -1, 1)))
            d = np.linalg.norm(_ends(tr)[0] - _ends(last)[1])
            if ang <= max_angle_deg and d < best_d:
                best, best_d = key, d
        if best is None:
            unattached.append(tr)
        else:
            chains[best].append(tr)
    merged = []
    for chain in chains.values():
        head = chain[0]
        if len(chain) == 1:
            merged.append(head)
            continue
        hx = np.array(head["heading"])
        hy = np.cross(n, hx)
        pts = np.vstack([_footprint(o, n)[:4] for o in chain])
        b0 = np.array(head["bottom"])
        along, lat = (pts - b0) @ hx, (pts - b0) @ hy
        length = float(along.max() - along.min())
        width = float(lat.max() - lat.min())
        height = float(max(o["dims"][2] for o in chain))
        bottom = b0 + hx * (along.max() + along.min()) / 2 + hy * (lat.max() + lat.min()) / 2
        x1 = min(o["box2d"][0] for o in chain)
        y1 = min(o["box2d"][1] for o in chain)
        x2 = max(o["box2d"][2] for o in chain)
        y2 = max(o["box2d"][3] for o in chain)
        merged.append({**head, "cls": "truck", "src_type": "TRUCK+" + "+".join("TRAILER" for _ in chain[1:]),
                       "bottom": bottom.tolist(), "dims": [length, min(width, 3.2), height],
                       "box2d": [x1, y1, x2, y2], "merged_parts": len(chain)})
    for tr in unattached:
        merged.append({**tr, "cls": "truck", "src_type": tr.get("src_type", "TRAILER") + "(standalone)"})
    s["objects"] = others + merged
    return s
