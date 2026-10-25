"""Convert TUMTraf R02 (Intersection) and R04 (V2X) into JP3D jsonl splits.

Usage:
    python -m tools.convert_tumtraf --root ~/data/junction_perception/datasets/TUMTraf --out ~/data/junction_perception/jp3d
"""
from __future__ import annotations

import argparse
import json
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

from jp3d import geometry as G
from jp3d.data import tumtraf as T

R2_CAMS = ["s110_camera_basler_south1_8mm", "s110_camera_basler_south2_8mm"]
R4_CAMS = ["s110_camera_basler_south1_8mm", "s110_camera_basler_south2_8mm",
           "s110_camera_basler_north_8mm", "s110_camera_basler_east_8mm"]


def list_images(d: Path) -> tuple[list[str], np.ndarray]:
    files = sorted(p.name for p in d.glob("*.jpg"))
    return files, np.array([T._timestamp(f) for f in files])


def r2_records(root: Path, devkit: Path):
    """Yield (split, image_rel, camera, frame_id, [(objs, P)])."""
    split_of = {}
    for split, name in (("train", "train"), ("val", "val"), ("test", "test_full")):
        for cam in R2_CAMS:
            f = devkit / "data_split" / name / "images" / cam / "file_names.txt"
            for line in f.read_text().split():
                split_of[line.strip()] = split
    for seq_dir in sorted((root / "R2").glob("a9_dataset_r02_s0*")):
        base = seq_dir / seq_dir.name
        north_dir = base / "labels_point_clouds" / "s110_lidar_ouster_north"
        north_files = sorted(north_dir.glob("*.json"))
        north_ts = np.array([T._timestamp(p.name) for p in north_files])
        imgs = {c: list_images(base / "images" / c) for c in R2_CAMS}
        for lab in sorted((base / "labels_point_clouds" / "s110_lidar_ouster_south").glob("*.json")):
            ts = T._timestamp(lab.name)
            south = T.parse_openlabel(lab)
            north = []
            if len(north_files):
                i = int(np.argmin(np.abs(north_ts - ts)))
                if abs(north_ts[i] - ts) < 0.06:
                    north = T.parse_openlabel(north_files[i])
            for cam in R2_CAMS:
                names, stamps = imgs[cam]
                img = T.nearest_image(names, stamps, ts)
                if img is None or img not in split_of:
                    continue
                rel = (base / "images" / cam / img).relative_to(root).as_posix()
                sets = [(south, T.P_SOUTH_TO[cam], "south")]
                if north:
                    sets.append((north, T.P_NORTH_TO[cam], "north"))
                yield split_of[img], rel, cam, lab.stem, sets


def r4_records(root: Path):
    for split in ("train", "val", "test"):
        sd = root / "R4" / split
        for lab in sorted((sd / "labels_point_clouds").glob("*/*.json")):
            data = json.loads(lab.read_text())
            frame = next(iter(data["openlabel"]["frames"].values()))
            names = frame["frame_properties"].get("image_file_names", [])
            objs = T.parse_openlabel(lab)
            for cam in R4_CAMS:
                img = next((n for n in names if cam in n), None)
                if img is None or not (sd / "images" / cam / img).exists():
                    continue
                rel = (sd / "images" / cam / img).relative_to(root).as_posix()
                yield split, rel, cam, lab.stem, [(objs, T.P_SOUTH_TO[cam], "south")]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", type=Path, required=True)
    ap.add_argument("--devkit", type=Path, default=None)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--releases", default="R2,R4")
    args = ap.parse_args()
    rels = args.releases.split(",")
    root = args.root.expanduser()
    devkit = (args.devkit or root / "tum-traffic-dataset-dev-kit-main").expanduser()
    out = args.out.expanduser()
    out.mkdir(parents=True, exist_ok=True)

    records = []
    if "R2" in rels:
        records += [("R2",) + r for r in r2_records(root, devkit)]
    if "R4" in rels:
        records += [("R4",) + r for r in r4_records(root)]
    print(f"{len(records)} image samples")

    # Pass 1: fit one ground plane per (release, camera, lidar) from label bottoms in the LiDAR frame.
    bottoms = defaultdict(list)
    for rel_name, split, rel, cam, fid, sets in records:
        for objs, P, lidar in sets:
            for o in objs:
                if o["type"] in G.TUMTRAF_TO_CLASS:
                    bottoms[(rel_name, cam, lidar)].append(o["center"] - np.array([0, 0, o["dims"][2] / 2]))
    planes_ref = {}
    for key, pts in bottoms.items():
        pts = np.array(pts)
        planes_ref[key] = G.fit_plane(pts, T.UP_HINT_REF, thresh=0.25)
        res = G.plane_height(planes_ref[key], pts)
        print(f"plane {key}: n={planes_ref[key][:3].round(3)} d={planes_ref[key][3]:.2f} "
              f"|res| median={np.median(np.abs(res)):.3f} m over {len(pts)} boxes")

    # Pass 2: camera-frame samples.
    cams = {}
    writers = {s: open(out / f"{s}.jsonl", "w") for s in ("train", "val", "test")}
    stats = Counter()
    for rel_name, split, rel, cam, fid, sets in records:
        objs_cam = []
        plane_cam = None
        T_cam_ref = None
        for objs, P, lidar in sets:
            key = (cam, lidar)
            if key not in cams:
                cams[key] = T.Camera.from_projection(cam, P)
            c = cams[key]
            pr = planes_ref[(rel_name, cam, lidar)]
            pc = G.transform_plane(pr, c.T_cam_from_ref)
            if plane_cam is None:
                plane_cam, T_cam_ref, K = pc, c.T_cam_from_ref, c.K
            objs_cam += T.objects_to_camera(objs, c, pr)
        objs_cam = T.dedup(objs_cam)
        for o in objs_cam:
            stats[(split, o["cls"])] += 1
        writers[split].write(json.dumps({
            "release": rel_name, "image": rel, "camera": cam, "frame_id": fid,
            "width": T.IMG_W, "height": T.IMG_H,
            "K": K.tolist(), "T_cam_from_ref": T_cam_ref.tolist(), "plane": plane_cam.tolist(),
            "objects": objs_cam,
        }) + "\n")
        stats[(split, "_images")] += 1
    for w in writers.values():
        w.close()
    for k in sorted(stats):
        print(k, stats[k])
    for key, c in cams.items():
        print(key, "f=", c.K[0, 0].round(1), c.K[1, 1].round(1), "c=", c.K[:2, 2].round(1),
              "cam height above ground ~", round(float(G.transform_plane(planes_ref[next(k for k in planes_ref if k[1:] == key)], c.T_cam_from_ref)[3]), 2))


if __name__ == "__main__":
    main()
