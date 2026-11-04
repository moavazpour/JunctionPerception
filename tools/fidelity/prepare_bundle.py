"""Bundle for the 3D-fidelity measurement: backend world states of both cameras and ground truth,
in one frame (south2 scene ground) on one clock (image capture times), with each backend object
matched to its ground-truth track."""
import csv, json, math, os, sys

ROOT = sys.argv[1]  # outputs/fidelity
DATA = sys.argv[2]  # /data/a9_dataset_r02_s03/images
SCENES = sys.argv[3]  # scenes
CAMERAS = ["south1", "south2"]
VRU = {"PEDESTRIAN", "BICYCLE", "MOTORCYCLE"}
GATE = {False: 2.0, True: 1.0}


def matrix(camera):
    import re
    text = open(f"{SCENES}/tumtraf_s110_{camera}/ground_homography.yaml").read()
    block = text.split("sensor_from_ground:")[1]
    rows = re.findall(r"- \[([^\]]+)\]", block)[:4]
    return [[float(v) for v in r.split(",")] for r in rows]


def inv_rigid(m):
    r = [[m[j][i] for j in range(3)] for i in range(3)]
    t = [-sum(r[i][k] * m[k][3] for k in range(3)) for i in range(3)]
    return [r[0] + [t[0]], r[1] + [t[1]], r[2] + [t[2]], [0, 0, 0, 1]]


def mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


S1, S2 = matrix("south1"), matrix("south2")
TO2 = {"south1": mul(inv_rigid(S2), S1), "south2": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]}


def point(c, x, y):
    m = TO2[c]
    return m[0][0] * x + m[0][1] * y + m[0][3], m[1][0] * x + m[1][1] * y + m[1][3]


def vec(c, x, y):
    m = TO2[c]
    return m[0][0] * x + m[0][1] * y, m[1][0] * x + m[1][1] * y


def yaw(c, a):
    m = TO2[c]
    return a + math.atan2(m[1][0], m[0][0])


times = {}
for c in CAMERAS:
    names = sorted(os.listdir(f"{DATA}/s110_camera_basler_{c}_8mm"))
    times[c] = [int(n.split("_")[0]) * 10**9 + int(n.split("_")[1]) for n in names]
origin = min(t[0] for t in times.values())

gt_tracks, gt_frames = {}, {}
for c in CAMERAS:
    gt_frames[c] = {}
    with open(f"{ROOT}/{c}_gt.csv") as f:
        for r in csv.DictReader(f):
            frame = int(r["frame"])
            t = times[c][frame] - origin
            x, y = point(c, float(r["x"]), float(r["y"]))
            g = {"id": int(r["track_id"]), "cat": r["category"], "x": x, "y": y,
                 "yaw": yaw(c, float(r["yaw_rad"])), "l": float(r["length_m"]), "w": float(r["width_m"])}
            gt_frames[c].setdefault(frame, []).append(g)
            gt_tracks.setdefault(g["id"], []).append([t, x, y, g["yaw"], g["l"], g["w"], g["cat"]])
for samples in gt_tracks.values():
    samples.sort(key=lambda s: s[0])

states, matches, stats = {}, {}, {}
for c in CAMERAS:
    out, matched, total = [], 0, 0
    with open(f"{ROOT}/{c}_world.jsonl") as f:
        for line in f:
            s = json.loads(line)
            frame = s["frame"]
            s["timestamp_ns"] = times[c][frame] - origin
            for o in s["objects"]:
                o["x"], o["y"] = point(c, o["x"], o["y"])
                o["vx"], o["vy"] = vec(c, o["vx"], o["vy"])
                if o["heading_rad"] is not None:
                    o["heading_rad"] = yaw(c, o["heading_rad"])
            # Greedy nearest matching to this frame's labels, by kind (vehicle or VRU).
            pairs = []
            for o in s["objects"]:
                for g in gt_frames[c].get(frame, []):
                    if (g["cat"] in VRU) == o["vru"]:
                        d = math.hypot(o["x"] - g["x"], o["y"] - g["y"])
                        if d < GATE[o["vru"]]:
                            pairs.append((d, o["id"], g["id"]))
            used_o, used_g = set(), set()
            for d, oid, gid in sorted(pairs):
                if oid not in used_o and gid not in used_g:
                    used_o.add(oid)
                    used_g.add(gid)
                    matches[f"{c}:{frame}:{oid}"] = gid
            total += len(s["objects"])
            matched += len(used_o)
            out.append(s)
    states[c] = out
    stats[c] = {"objects": total, "matched": matched}

cams = {}
for c in CAMERAS:
    text = open(f"{SCENES}/tumtraf_s110_{c}/ground_homography.yaml").read()
    pos = [float(v) for v in text.split("camera_ground_position_m:")[1].split("]")[0].strip(" [").split(",")]
    cams[c] = point(c, *pos)

json.dump({"origin_ns": origin, "states": states, "gt": gt_tracks, "matches": matches, "stats": stats,
           "cameras": cams, "map_transform": {"yaw_deg": -165.6, "x_m": 0.0, "y_m": 0.0}},
          open(f"{ROOT}/bundle.json", "w"))
print("bundle", {c: len(states[c]) for c in CAMERAS}, stats, "gt tracks", len(gt_tracks),
      "start offset ms", {c: (times[c][0] - origin) / 1e6 for c in CAMERAS})
