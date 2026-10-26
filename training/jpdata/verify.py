"""Image-by-image verification of a converted source against an independent strong detector.

For every image:
  * labels are checked against detections (class-group agnostic: vehicle labels vs car/bus/truck
    detections, VRU labels vs person/bicycle/motorcycle). Labels are amodal projected cuboids, so a
    detection inside a label box also counts.
  * an image whose clearly visible labels mostly have no detection is a label/image mismatch
    (wrong frame, wrong calibration) and is dropped.
  * confident detections that no label explains are visible but unlabelled road users: they are
    stored as `ignore_det` boxes (used only by the ignore version of the dataset).
  * optionally, black/blank frames are dropped.

    python -m jpdata.verify --src lumpi=conv/lumpi2.jsonl:undist/lumpi2 --dets conv/lumpi.dets.jsonl \
        --out conv/lumpi.verified.jsonl --report qc/verify_lumpi.txt [--check_black]
"""
from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path

import numpy as np

from jpdata.truck_merge import merge_truck_trailer
from jpdata.build_unified import qc

NONMETRIC = {"dair_spd_infra", "dair_c_infra", "dair_spd_vehicle", "dair_i"}

VEH_L, VRU_L = {"car", "bus", "truck", "trailer"}, {"pedestrian", "bicycle", "motorcycle", "escooter"}
VEH_D, VRU_D = {"car", "bus", "truck"}, {"pedestrian", "bicycle", "motorcycle"}


def inter(a, b):
    return max(0.0, min(a[2], b[2]) - max(a[0], b[0])) * max(0.0, min(a[3], b[3]) - max(a[1], b[1]))


def area(a):
    return max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])


def iou(a, b):
    i = inter(a, b)
    u = area(a) + area(b) - i
    return i / u if u > 0 else 0.0


def explains(label_box, det_box) -> bool:
    """Does a (projected, amodal) label box account for this detection?"""
    if iou(label_box, det_box) >= 0.3:
        return True
    i = inter(label_box, det_box)
    return i >= 0.7 * area(det_box) and area(det_box) >= 0.15 * area(label_box)


def main():
    from jpdata.label_audit import audit  # imported here: label_audit imports this module
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True, help="name=jsonl:image_root")
    ap.add_argument("--dets", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--report", type=Path, required=True)
    ap.add_argument("--check_black", action="store_true")
    ap.add_argument("--min_agree", type=float, default=0.5)
    ap.add_argument("--conf_match", type=float, default=0.3)
    ap.add_argument("--conf_unlabelled", type=float, default=0.5)
    ap.add_argument("--no_qc", action="store_true",
                    help="keep every label (LiDAR-labelled sources: the single ground plane is only an approximation)")
    ap.add_argument("--max_shifted", type=float, default=0.0,
                    help="drop an image when this share of its visible labels is shifted against the object (0: off)")
    a = ap.parse_args()
    name, rest = a.src.split("=", 1)
    jl, root = rest.split(":", 1)
    root = Path(root).expanduser()
    dets = {}
    for l in a.dets.read_text().splitlines():
        d = json.loads(l)
        dets[d["image"]] = d["dets"]
    stats = Counter()
    qc_stats = Counter()
    agree_all, dropped = [], []
    with open(a.out, "w") as f:
        for line in Path(jl).expanduser().read_text().splitlines():
            if not line.strip():
                continue
            s = merge_truck_trailer(json.loads(line))
            s["source"] = name
            if not a.no_qc:
                s = qc(s, qc_stats, name not in NONMETRIC)  # size / ground-plane checks: failures become ignore
            stats["images"] += 1
            if s["image"] not in dets:
                stats["no_detections_file"] += 1
                dropped.append((s["image"], "not processed by detector"))
                continue
            if a.check_black:
                import cv2

                im = cv2.imread(str(root / s["image"]), cv2.IMREAD_REDUCED_GRAYSCALE_8)
                if im is None or im.mean() < 8:
                    stats["dropped_black"] += 1
                    dropped.append((s["image"], "black/unreadable frame"))
                    continue
            D = [d for d in dets[s["image"]] if d[4] >= a.conf_match]
            checked = verified = 0
            for o in s["objects"]:
                b = o["box2d"]
                visible = (b[3] - b[1]) >= 24 and (b[2] - b[0]) >= 12 and o.get("truncation", 0) < 0.5
                grp = VEH_D if o["cls"] in VEH_L else VRU_D
                ok = any(d[5] in grp and explains(b, d[:4]) for d in D)
                o["det_ok"] = bool(ok)
                if visible:
                    checked += 1
                    verified += ok
            agree = verified / checked if checked else 1.0
            s["qc_agree"] = round(agree, 3)
            if checked >= 3:
                agree_all.append(agree)
            if checked >= 3 and agree < a.min_agree:
                stats["dropped_mismatch"] += 1
                dropped.append((s["image"], f"labels do not match image ({verified}/{checked} confirmed)"))
                continue
            if a.max_shifted > 0:
                r = audit(s, dets[s["image"]], a.conf_match)
                s["objects"] = [{k: v for k, v in o.items() if k != "audit"} for o in s["objects"]]
                if r["shifted"] >= 2 and r["shifted"] >= a.max_shifted * r["n"]:
                    stats["dropped_shifted"] += 1
                    dropped.append((s["image"], f"labels shifted against objects ({r['shifted']}/{r['n']})"))
                    continue
            # Visible road users nobody labelled.
            labelled = [o["box2d"] for o in s["objects"]]
            ign = list(s.get("ignore", []))
            unl = []
            for d in dets[s["image"]]:
                if d[4] < a.conf_unlabelled:
                    continue
                box = d[:4]
                if any(explains(lb, box) or iou(lb, box) >= 0.1 for lb in labelled):
                    continue
                if any(inter(g, box) >= 0.5 * area(box) for g in ign):
                    continue
                unl.append([round(v, 1) for v in box])
            s["ignore_det"] = unl
            stats["unlabelled_dets"] += len(unl)
            stats["kept"] += 1
            f.write(json.dumps(s) + "\n")
    lines = [f"source {name}: {stats['images']} images, kept {stats['kept']}",
             f"  dropped: mismatch {stats['dropped_mismatch']}, black {stats['dropped_black']}, "
             f"shifted {stats['dropped_shifted']}, not detected {stats['no_detections_file']}",
             f"  label/detector agreement: median {np.median(agree_all) if agree_all else float('nan'):.3f}, "
             f"p10 {np.percentile(agree_all, 10) if agree_all else float('nan'):.3f}",
             f"  unlabelled visible road users (ignore_det): {stats['unlabelled_dets']} "
             f"({stats['unlabelled_dets'] / max(stats['kept'], 1):.2f} per image)",
             f"  size/ground checks: {dict(qc_stats)}",
             "  dropped images:"] + [f"    {i}  {why}" for i, why in dropped]
    a.report.parent.mkdir(parents=True, exist_ok=True)
    a.report.write_text("\n".join(lines) + "\n")
    print("\n".join(lines[:4]))


if __name__ == "__main__":
    main()
