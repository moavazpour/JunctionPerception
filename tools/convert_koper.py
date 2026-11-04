"""Convert the Ko-PER Sequence1 object labels into ground truth aligned with the camera videos.

Ko-PER (Strigel et al., ITSC 2014, Uni Ulm) labels every road user at 12.5 Hz as a box on the
ground in the intersection frame C_w (east-north-up, metres). The cameras run at 25 Hz in phase
with the laserscanners. For each camera image this writes the labelled objects at the image
timestamp, linearly interpolated between the neighbouring label frames (the labels are
themselves interpolated between manually placed key frames), as twin world states.

Input, one folder per part (Sequence1a ... Sequence1d) as unpacked from the dataset zips:
  1_Rec*_TS_merged_part*.mat               camera streams (image file name and timestamp)
  3_BoxfittingLabels_REF_GP_eff_part*.mat  object labels
and the calibration of the undistorted images, calib/Calib_undist.mat of the DataSetViewer.

Output in <out>/:
  <part>_<camera>.jsonl       world states (docs/architecture.md), one per video frame, with the
                              objects whose box centre projects into this camera's image
  <part>_all.jsonl            the same with every labelled object of the intersection, at the
                              times of the KAB_SK_1 images
  <part>_<camera>_frames.csv  frame index, timestamp (us), image file name
  calibration_<camera>.json   intrinsics, camera_from_world and the ground homographies
Objects carry the world-state fields plus height_m, front_x, front_y (the labelled reference
point: centre of the front face on the ground) and label_class. x, y: box centre on the ground
in C_w; heading_rad: box orientation (rad, from the x axis); vx, vy: from the track positions
over a centred 0.24 s window.

With --scenes, also writes two sites in that folder, both with C_w as their world frame and the
dataset calibration of both cameras (no control points needed):
  koper_aschaffenburg/  cameras sk1, sk4 playing the videos through the live pipeline
  koper_ground_truth/   cameras sk1, sk4 replaying the ground truth of the same video
The sites are separate so that the ground truth is not fused with the live cameras.

Label classes (counts match Tab. VI of the dataset documentation): 0 pedestrian, 1 bike,
3 car, 5 truck.

Requires: numpy, scipy (and pyyaml with --scenes).

Example:
    python tools/convert_koper.py --root ~/Dataset/Ko-PER \
        --calib ~/Dataset/Ko-PER/viewer/DataSetViewer/calib/Calib_undist.mat \
        --out ~/Dataset/Ko-PER/ground_truth --scenes scenes
"""

import argparse
import glob
import json
import math
import os

import numpy as np
import scipy.io as sio

CAMERAS = {"KAB_SK_1": "sk1", "KAB_SK_4": "sk4"}  # dataset name: camera id of the sites
IMAGE_SIZE = (656, 494)
CLASSES = {0: "pedestrian", 1: "bicycle", 3: "car", 5: "truck"}
VRU = {"pedestrian", "bicycle"}
# A track present in only one of the two neighbouring label frames is kept up to this gap (us).
MAX_EXTRAPOLATION_US = 40_000
VELOCITY_HALF_WINDOW = 1  # label frames on each side (12.5 Hz -> 0.24 s window)
WORLD_STATE_SCHEMA_VERSION = 1
LIVE_SITE = "koper_aschaffenburg"
TRUTH_SITE = "koper_ground_truth"
# Paths the sites use: videos below the data root, replays below the repository.
VIDEO_PATH = "Ko-PER/videos/{part}_{camera}.mp4"
REPLAY_PATH = "outputs/koper_ground_truth/{part}_{camera}.jsonl"
DEFAULT_PART = "Sequence1a"

# Columns of FEAT_* in BoxfittingLabels_REF.
X, Y, Z, WIDTH, LENGTH, HEIGHT, ORIENTATION, REFERENCE_POINT, ID, CLASSIFICATION = range(10)


def load_labels(path):
    labels = sio.loadmat(path)["result"][0, 0]["BoxfittingLabels_REF"][0, 0]
    names = [str(f[0][0]) for f in labels["availableFeatures"]]
    expected = ["FEAT_X_POS", "FEAT_Y_POS", "FEAT_Z_POS", "FEAT_WIDTH", "FEAT_LENGTH",
                "FEAT_HEIGHT", "FEAT_ORIENTATION_ANGLE", "FEAT_REFERENCEPOINT", "FEAT_ID",
                "FEAT_CLASSIFICATION"]
    if names != expected:
        raise RuntimeError(f"{path}: unexpected label features {names}")
    times = labels["time"].ravel().astype(np.int64)
    frames = []
    for entry in labels["data"][0]:
        features = entry["features"][0, 0]
        frames.append({int(row[ID]): row for row in features} if features.size else {})
    return times, frames


def track_velocities(times, frames):
    """Velocity (vx, vy) of every track in every label frame, by central differences."""
    velocities = []
    for k, frame in enumerate(frames):
        lo = max(0, k - VELOCITY_HALF_WINDOW)
        hi = min(len(frames) - 1, k + VELOCITY_HALF_WINDOW)
        out = {}
        for track_id, row in frame.items():
            a = next(i for i in range(lo, k + 1) if track_id in frames[i])
            b = next(i for i in range(hi, k - 1, -1) if track_id in frames[i])
            if b == a:
                out[track_id] = (0.0, 0.0)
                continue
            dt = (times[b] - times[a]) * 1e-6
            pa, pb = frames[a][track_id], frames[b][track_id]
            out[track_id] = ((pb[X] - pa[X]) / dt, (pb[Y] - pa[Y]) / dt)
        velocities.append(out)
    return velocities


def angle_lerp(a, b, w):
    return a + w * math.atan2(math.sin(b - a), math.cos(b - a))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def object_json(row, velocity, weight=0.0, other=None, other_velocity=None):
    def mix(column):
        return row[column] if other is None else (1 - weight) * row[column] + weight * other[column]

    vx, vy = velocity
    if other is not None:
        vx = (1 - weight) * vx + weight * other_velocity[0]
        vy = (1 - weight) * vy + weight * other_velocity[1]
    heading = row[ORIENTATION] if other is None else angle_lerp(row[ORIENTATION],
                                                                other[ORIENTATION], weight)
    length = mix(LENGTH)
    front_x, front_y = mix(X), mix(Y)
    # The labelled point is the front centre; the box extends backwards along the heading.
    x = front_x - 0.5 * length * math.cos(heading)
    y = front_y - 0.5 * length * math.sin(heading)
    label_class = int(row[CLASSIFICATION])
    object_class = CLASSES.get(label_class, "other")
    return {
        "id": int(row[ID]),
        "class": object_class,
        "vru": object_class in VRU,
        "rider": object_class == "bicycle",
        "x": round(float(x), 3),
        "y": round(float(y), 3),
        "vx": round(float(vx), 3),
        "vy": round(float(vy), 3),
        "heading_rad": round(wrap(float(heading)), 4),
        "speed_mps": round(math.hypot(vx, vy), 3),
        "speed_std_mps": None,
        "confidence": 1.0,
        "zones": [],
        "turning": False,
        "validated": True,
        "length_m": round(float(length), 3),
        "width_m": round(float(mix(WIDTH)), 3),
        "height_m": round(float(mix(HEIGHT)), 3),
        "front_x": round(float(front_x), 3),
        "front_y": round(float(front_y), 3),
        "label_class": label_class,
    }


def world_state(frame, timestamp_us, objects):
    return {"schema_version": WORLD_STATE_SCHEMA_VERSION, "scene_id": TRUTH_SITE, "frame": frame,
            "timestamp_ns": int(timestamp_us) * 1000, "source": "oracle", "objects": objects,
            "events": [], "signals": []}


def load_calibration(path, camera):
    """Pinhole calibration of the undistorted images of one camera (DataSetViewer format)."""
    calib = sio.loadmat(path)[f"CALIB_UNDIST_{camera.replace('KAB_', '')}"][0, 0]
    if np.any(calib["kc"]):
        raise RuntimeError(f"{path}: {camera} has distortion coefficients")
    fx, fy = calib["fc"].ravel().astype(float)
    cx, cy = calib["cc"].ravel().astype(float)
    intrinsics = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    camera_from_world = calib["Tc_homo"].astype(float)
    projection = intrinsics @ camera_from_world[:3]
    # Ground plane z = 0: pixel = K [r1 r2 t] (x, y, 1).
    ground_to_image = projection[:, [0, 1, 3]] / projection[2, 3]
    image_to_ground = np.linalg.inv(ground_to_image)
    image_to_ground /= image_to_ground[2, 2]
    position = -camera_from_world[:3, :3].T @ camera_from_world[:3, 3]
    return {
        "camera": camera,
        "image_size_px": list(IMAGE_SIZE),
        "frame": "Ko-PER intersection frame C_w (east-north-up, m)",
        "intrinsics": intrinsics.tolist(),
        "camera_from_world": camera_from_world.tolist(),
        "ground_projection": (projection / projection[2, 3]).tolist(),
        "ground_to_image": ground_to_image.tolist(),
        "image_to_ground": image_to_ground.tolist(),
        "camera_position_m": position.tolist(),
    }


def in_image(calibration, x, y):
    camera_from_world = np.array(calibration["camera_from_world"])
    point = camera_from_world @ np.array([x, y, 0.0, 1.0])
    if point[2] <= 0:
        return False
    u, v, w = np.array(calibration["intrinsics"]) @ point[:3]
    width, height = calibration["image_size_px"]
    return bool(0 <= u / w < width and 0 <= v / w < height)


def objects_at(t, times, frames, velocities):
    k = int(np.searchsorted(times, t, side="right")) - 1  # last label frame at or before t
    before = (k, frames[k], velocities[k]) if k >= 0 else None
    after = (k + 1, frames[k + 1], velocities[k + 1]) if k + 1 < len(frames) else None
    objects = []
    ids = set(before[1] if before else {}) | set(after[1] if after else {})
    for track_id in sorted(ids):
        in_before = before is not None and track_id in before[1]
        in_after = after is not None and track_id in after[1]
        if in_before and in_after:
            t0, t1 = times[before[0]], times[after[0]]
            w = (t - t0) / (t1 - t0)
            objects.append(object_json(before[1][track_id], before[2][track_id], w,
                                       after[1][track_id], after[2][track_id]))
        else:
            k_near, frame, velocity = before if in_before else after
            if abs(t - times[k_near]) <= MAX_EXTRAPOLATION_US:
                objects.append(object_json(frame[track_id], velocity[track_id]))
    return objects


def camera_frames(cameras, camera):
    """Image timestamps (us) and file names of one camera stream."""
    stream = cameras[f"{camera}_undist"][0, 0]
    image_times = stream["time"].ravel().astype(np.int64)
    images = [str(entry["image"][0, 0][0]) for entry in stream["data"][0]]
    # A dropped camera frame appears as the previous image listed twice with the same
    # timestamp. The slot keeps the repeated image (the video repeats it too, so video frame
    # k stays at k / 25 s) and gets the time halfway to the next frame.
    repeated = [False] * len(images)
    for k in range(1, len(images) - 1):
        if images[k] == images[k - 1] and image_times[k] == image_times[k - 1]:
            repeated[k] = True
            image_times[k] = (image_times[k - 1] + image_times[k + 1]) // 2
    if images != sorted(images) or np.any(np.diff(image_times) <= 0):
        raise RuntimeError(f"{camera}: images are not in time order")
    return image_times, images, repeated


def convert_part(part_dir, out_dir, calibrations):
    part = os.path.basename(os.path.normpath(part_dir))
    times, frames = load_labels(glob.glob(os.path.join(part_dir, "3_BoxfittingLabels_*.mat"))[0])
    velocities = track_velocities(times, frames)
    cameras = sio.loadmat(glob.glob(os.path.join(part_dir, "1_Rec*.mat"))[0])["result"][0, 0]
    for n, camera in enumerate(CAMERAS):
        image_times, images, repeated = camera_frames(cameras, camera)
        stem = os.path.join(out_dir, f"{part}_{camera}")
        everything = open(os.path.join(out_dir, f"{part}_all.jsonl"), "w") if n == 0 else None
        boxes = visible = 0
        with open(stem + ".jsonl", "w") as jsonl, open(stem + "_frames.csv", "w") as csv:
            csv.write("frame,timestamp_us,image,repeated\n")
            for index, (t, image) in enumerate(zip(image_times, images)):
                objects = objects_at(t, times, frames, velocities)
                seen = [o for o in objects if in_image(calibrations[camera], o["x"], o["y"])]
                boxes += len(objects)
                visible += len(seen)
                jsonl.write(json.dumps(world_state(index, t, seen)) + "\n")
                if everything:
                    everything.write(json.dumps(world_state(index, t, objects)) + "\n")
                csv.write(f"{index},{t},{image},{int(repeated[index])}\n")
        if everything:
            everything.close()
        print(f"{part}/{camera}: {len(images)} frames ({sum(repeated)} repeated), {boxes} boxes "
              f"({visible} in the image), {len(times)} label frames")


def write_scenes(scenes_dir, calibrations, calib_path):
    import yaml

    def dump(path, comment, data):
        with open(path, "w") as f:
            f.write(comment)
            yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True, width=100)

    sites = {
        LIVE_SITE: ("Ko-PER Aschaffenburg", "Ko-PER Sequence1 cameras through the live pipeline",
                    lambda camera: {"mode": "live",
                                    "input": VIDEO_PATH.format(part=DEFAULT_PART, camera=camera)}),
        TRUTH_SITE: ("Ko-PER Aschaffenburg · ground truth",
                     "Ko-PER Sequence1 labels of each camera's view, replayed",
                     lambda camera: {"mode": "replay",
                                     "input": REPLAY_PATH.format(part=DEFAULT_PART,
                                                                 camera=camera)}),
    }
    for site, (name, description, source) in sites.items():
        site_dir = os.path.join(scenes_dir, site)
        dump(os.path.join(site_dir_ready(site_dir), "scene.yaml"),
             "# Ko-PER intersection, Aschaffenburg (Uni Ulm dataset, written by\n"
             "# tools/convert_koper.py). World frame: the dataset's intersection frame C_w\n"
             "# (east-north-up, m), the frame of its labels; not geo-referenced.\n",
             {"scene_id": site, "kind": "custom"})
        settings = {"name": name, "description": description, "enabled": False, "cameras": {}}
        for camera, camera_id in CAMERAS.items():
            calibration = calibrations[camera]
            camera_dir = site_dir_ready(os.path.join(site_dir, "cameras", camera_id))
            dump(os.path.join(camera_dir, "camera.yaml"),
                 f"# Camera {camera} of the Ko-PER intersection (undistorted images).\n",
                 {"name": camera, "approach": "centre", "image_size_px": list(IMAGE_SIZE)})
            position = calibration["camera_position_m"]
            dump(os.path.join(camera_dir, "ground_homography.yaml"),
                 "# Dataset calibration (Calib_undist.mat of the Ko-PER DataSetViewer) on the\n"
                 "# ground plane z = 0 of C_w.\n",
                 {"camera": f"{site}/{camera_id}",
                  "image_size_px": list(IMAGE_SIZE),
                  "source": os.path.basename(calib_path),
                  "image_to_ground": calibration["image_to_ground"],
                  "ground_to_image": calibration["ground_to_image"],
                  "ground_projection": calibration["ground_projection"],
                  "ground_projection_estimated": False,
                  "camera_ground_position_m": position[:2],
                  "camera_height_m": position[2]})
            settings["cameras"][camera_id] = {
                "enabled": True,
                "source": source(camera),
                "playback": {"loop": True, "speed": 1.0},
                "video": {"max_fps": 25, "width_px": IMAGE_SIZE[0], "jpeg_quality": 80},
            }
        dump(os.path.join(site_dir, "site.yaml"),
             "# Site settings edited in the console (manager/jp_manager.py).\n", settings)
        print(f"wrote {site_dir}")


def site_dir_ready(path):
    os.makedirs(path, exist_ok=True)
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--root", required=True, help="Folder with Sequence1a ... Sequence1d")
    parser.add_argument("--calib", required=True, help="Calib_undist.mat of the DataSetViewer")
    parser.add_argument("--out", required=True, help="Output folder")
    parser.add_argument("--scenes", help="Also write the two Ko-PER sites into this folder")
    args = parser.parse_args()
    root = os.path.expanduser(args.root)
    out_dir = os.path.expanduser(args.out)
    os.makedirs(out_dir, exist_ok=True)
    parts = sorted(glob.glob(os.path.join(root, "Sequence1?")))
    if not parts:
        raise SystemExit(f"no Sequence1? folders in {root}")
    calib_path = os.path.expanduser(args.calib)
    calibrations = {}
    for camera in CAMERAS:
        calibrations[camera] = load_calibration(calib_path, camera)
        with open(os.path.join(out_dir, f"calibration_{camera}.json"), "w") as f:
            json.dump(calibrations[camera], f, indent=2)
    for part_dir in parts:
        convert_part(part_dir, out_dir, calibrations)
    if args.scenes:
        write_scenes(os.path.expanduser(args.scenes), calibrations, calib_path)


if __name__ == "__main__":
    main()
