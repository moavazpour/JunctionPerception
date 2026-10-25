"""Ground-plane 3D detection metrics.

Objects are compared in a camera-centred bird's-eye view on the ground plane, so results do not depend
on any site-specific world frame. Two matching criteria are reported per class:

* centre distance (nuScenes style), thresholds 0.5/1/2/4 m -> mAP_dist
* rotated BEV IoU, thresholds 0.1 (official TUMTraf), 0.25, 0.5

plus true-positive errors at 2 m: translation (ATE, m), scale (ASE, 1-IoU), orientation (AOE, rad),
and a breakdown by range.
"""
from __future__ import annotations

from collections import defaultdict

import numpy as np
from shapely.geometry import Polygon

from . import geometry as G

DIST_THRESHOLDS = (0.5, 1.0, 2.0, 4.0)
IOU_THRESHOLDS = (0.1, 0.25, 0.5)
RANGES = ((0, 30), (30, 60), (60, 100), (100, 1e9))
EVAL_CLASSES = ["car", "bus", "truck", "trailer", "pedestrian", "bicycle", "motorcycle"]


def bev_basis(plane: np.ndarray):
    n = plane[:3]
    fwd = np.array([0.0, 0.0, 1.0]) - n[2] * n
    fwd /= np.linalg.norm(fwd)
    left = np.cross(n, fwd)
    return n, fwd, left


def to_bev(objs: list[dict], plane: np.ndarray) -> list[dict]:
    n, fwd, left = bev_basis(plane)
    out = []
    for o in objs:
        b = np.asarray(o["bottom"], float)
        b = b - (b @ n + plane[3]) * n
        h = np.asarray(o["heading"], float)
        x, y = b @ fwd, b @ left
        yaw = float(np.arctan2(h @ left, h @ fwd))
        l, w, hh = o["dims"]
        c, s = np.cos(yaw), np.sin(yaw)
        pts = [(x + c * dx - s * dy, y + s * dx + c * dy)
               for dx, dy in ((l / 2, w / 2), (l / 2, -w / 2), (-l / 2, -w / 2), (-l / 2, w / 2))]
        out.append({**o, "xy": np.array([x, y]), "yaw": yaw, "poly": Polygon(pts), "range": float(np.hypot(x, y))})
    return out


def ap_from_pr(tp: np.ndarray, scores: np.ndarray, n_gt: int) -> float:
    if n_gt == 0:
        return float("nan")
    if len(tp) == 0:
        return 0.0
    order = np.argsort(-scores)
    tp = tp[order]
    ctp = np.cumsum(tp)
    cfp = np.cumsum(1 - tp)
    rec = ctp / n_gt
    prec = ctp / np.maximum(ctp + cfp, 1e-9)
    ap = 0.0
    for r in np.linspace(0, 1, 101):
        p = prec[rec >= r]
        ap += (p.max() if len(p) else 0.0) / 101
    return ap


def match(gts, dets, crit, thr):
    """Greedy matching by score. Returns tp flags per det (1/0, -1 = ignored) and matched pairs."""
    flags = np.zeros(len(dets))
    pairs = []
    used = set()
    for di in np.argsort([-d["score"] for d in dets]):
        d = dets[di]
        best, best_v = None, None
        for gi, g in enumerate(gts):
            if gi in used:
                continue
            if crit == "dist":
                v = np.linalg.norm(d["xy"] - g["xy"])
                ok = v <= thr and (best_v is None or v < best_v)
            else:
                inter = d["poly"].intersection(g["poly"]).area
                v = inter / max(d["poly"].area + g["poly"].area - inter, 1e-9)
                ok = v >= thr and (best_v is None or v > best_v)
            if ok:
                best, best_v = gi, v
        if best is None:
            flags[di] = 0
        else:
            used.add(best)
            flags[di] = -1 if gts[best].get("ignore") else 1
            if flags[di] == 1:
                pairs.append((di, best))
    return flags, pairs


def yaw_diff(a, b):
    d = (a - b + np.pi) % (2 * np.pi) - np.pi
    return abs(d)


def evaluate(frames: list[tuple[list[dict], list[dict], np.ndarray]], classes=EVAL_CLASSES,
             min_box_h: float = 10.0) -> dict:
    """frames: list of (gt_objects, det_objects, plane). Objects use camera-frame bottom/dims/heading."""
    acc = defaultdict(lambda: {"tp": [], "sc": [], "ngt": 0})
    err = defaultdict(lambda: defaultdict(list))
    for gts, dets, plane in frames:
        gts = to_bev(gts, plane)
        dets = to_bev(dets, plane)
        for g in gts:
            b = g.get("box2d")
            g["ignore"] = b is not None and (b[3] - b[1]) < min_box_h
        for c in classes:
            gc = [g for g in gts if g["cls"] == c]
            dc = [d for d in dets if d["cls"] == c]
            n_valid = sum(1 for g in gc if not g["ignore"])
            for rng_i, (r0, r1) in enumerate(RANGES + ((0, 1e9),)):
                key_r = "all" if rng_i == len(RANGES) else f"{r0}-{int(r1) if r1 < 1e9 else 'inf'}"
                g_r = [dict(g, ignore=g["ignore"] or not (r0 <= g["range"] < r1)) for g in gc]
                d_r = [d for d in dc if r0 - 2 <= d["range"] < r1 + 2] if key_r != "all" else dc
                n_r = sum(1 for g in g_r if not g["ignore"])
                crits = (("dist", DIST_THRESHOLDS), ("iou", IOU_THRESHOLDS)) if key_r == "all" else (("dist", (2.0,)),)
                for crit, thrs in crits:
                    for thr in thrs:
                        flags, pairs = match(g_r, d_r, crit, thr)
                        a = acc[(c, crit, thr, key_r)]
                        keep = flags >= 0
                        a["tp"] += list(flags[keep])
                        a["sc"] += [d["score"] for d, k in zip(d_r, keep) if k]
                        a["ngt"] += n_r
                        if crit == "dist" and thr == 2.0 and key_r == "all":
                            for di, gi in pairs:
                                d, g = d_r[di], g_r[gi]
                                err[c]["ate"].append(np.linalg.norm(d["xy"] - g["xy"]))
                                dl, gl = np.array(d["dims"]), np.array(g["dims"])
                                inter = np.prod(np.minimum(dl, gl))
                                err[c]["ase"].append(1 - inter / (np.prod(dl) + np.prod(gl) - inter))
                                err[c]["aoe"].append(yaw_diff(d["yaw"], g["yaw"]))
                                err[c]["aoe_pi"].append(min(err[c]["aoe"][-1], np.pi - err[c]["aoe"][-1]))
                                err[c]["range_err_rel"].append(np.linalg.norm(d["xy"] - g["xy"]) / max(g["range"], 1))
            del n_valid
    res = {"per_class": {}, "range": {}}
    for c in classes:
        r = {}
        for crit, thrs in (("dist", DIST_THRESHOLDS), ("iou", IOU_THRESHOLDS)):
            for thr in thrs:
                a = acc[(c, crit, thr, "all")]
                r[f"AP_{crit}{thr}"] = ap_from_pr(np.array(a["tp"]), np.array(a["sc"]), a["ngt"])
        r["mAP_dist"] = float(np.nanmean([r[f"AP_dist{t}"] for t in DIST_THRESHOLDS]))
        r["n_gt"] = acc[(c, "dist", 2.0, "all")]["ngt"]
        for k in ("ate", "ase", "aoe", "aoe_pi", "range_err_rel"):
            v = err[c][k]
            r[k.upper()] = float(np.mean(v)) if v else float("nan")
        res["per_class"][c] = r
        for (r0, r1) in RANGES:
            key_r = f"{r0}-{int(r1) if r1 < 1e9 else 'inf'}"
            a = acc[(c, "dist", 2.0, key_r)]
            res["range"].setdefault(key_r, {})[c] = {
                "AP_dist2.0": ap_from_pr(np.array(a["tp"]), np.array(a["sc"]), a["ngt"]), "n_gt": a["ngt"]}
    valid = [c for c in classes if res["per_class"][c]["n_gt"] > 0]
    res["mAP_dist"] = float(np.nanmean([res["per_class"][c]["mAP_dist"] for c in valid]))
    for thr in IOU_THRESHOLDS:
        res[f"mAP_iou{thr}"] = float(np.nanmean([res["per_class"][c][f"AP_iou{thr}"] for c in valid]))
    for k in ("ATE", "ASE", "AOE"):
        res["m" + k] = float(np.nanmean([res["per_class"][c][k] for c in valid]))
    return res


def format_results(res: dict) -> str:
    lines = [f"mAP_dist={res['mAP_dist']:.3f}  " + "  ".join(f"mAP_iou{t}={res[f'mAP_iou{t}']:.3f}" for t in IOU_THRESHOLDS)
             + f"  mATE={res['mATE']:.2f}m  mASE={res['mASE']:.3f}  mAOE={res['mAOE']:.3f}rad"]
    hdr = f"{'class':<11}{'n_gt':>7}{'mAPd':>7}" + "".join(f"{'d'+str(t):>7}" for t in DIST_THRESHOLDS) + \
          "".join(f"{'iou'+str(t):>8}" for t in IOU_THRESHOLDS) + f"{'ATE':>7}{'ASE':>7}{'AOE':>7}{'AOEpi':>7}"
    lines.append(hdr)
    for c, r in res["per_class"].items():
        lines.append(f"{c:<11}{r['n_gt']:>7}{r['mAP_dist']:>7.3f}" + "".join(f"{r[f'AP_dist{t}']:>7.3f}" for t in DIST_THRESHOLDS)
                     + "".join(f"{r[f'AP_iou{t}']:>8.3f}" for t in IOU_THRESHOLDS)
                     + f"{r['ATE']:>7.2f}{r['ASE']:>7.3f}{r['AOE']:>7.3f}{r['AOE_PI']:>7.3f}")
    lines.append("AP_dist2.0 by range:")
    for k, v in res["range"].items():
        lines.append(f"  {k:>8}: " + "  ".join(f"{c}={x['AP_dist2.0']:.2f}({x['n_gt']})" for c, x in v.items() if x["n_gt"]))
    return "\n".join(lines)
