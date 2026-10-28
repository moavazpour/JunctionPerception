"""Synthetic roadside 3D detection data from CARLA with randomised camera mountings and weather.

Every placement samples a mounting the real system may meet: footbridges looking down an approach,
roadside poles, junction corners; heights 4-12 m, pitch 3-35 deg, horizontal FOV 20-90 deg. Labels are
exact 3D boxes; visibility comes from the instance-segmentation camera. Output is the JP3D jsonl format.

    ~/carla/CarlaUE4.sh -RenderOffScreen -carla-rpc-port=2000 &
    python tools/carla_gen.py --out ~/JunctionPerception/data/carla --maps Town03,Town05,Town10HD_Opt \
        --placements 60 --frames 25
"""
from __future__ import annotations

import argparse
import json
import math
import queue
import random
import time
from pathlib import Path

import numpy as np

try:
    import carla
except ImportError as e:  # pragma: no cover
    raise SystemExit("needs the CARLA Python API") from e
import cv2

W, H = 1920, 1080
CLASS_OF_BASE = {"car": "car", "van": "car", "truck": "truck", "bus": "bus", "motorcycle": "motorcycle",
                 "bicycle": "bicycle"}
MIN_VISIBLE = 0.10  # visible share of the projected box
BLOCKING_TAGS = [3, 4, 9]  # building, wall, vegetation (CARLA 0.9.14+ semantic tags)
WEATHERS = ["ClearNoon", "CloudyNoon", "WetNoon", "WetCloudyNoon", "MidRainyNoon", "HardRainNoon", "SoftRainNoon",
            "ClearSunset", "CloudySunset", "WetSunset", "WetCloudySunset", "MidRainSunset", "ClearNight",
            "CloudyNight", "WetNight", "SoftRainNight", "MidRainyNight", "HardRainNight", "DustStorm"]


def ue_to_cv(v: np.ndarray) -> np.ndarray:
    """UE camera axes (x fwd, y right, z up) -> OpenCV (x right, y down, z fwd)."""
    return np.stack([v[..., 1], -v[..., 2], v[..., 0]], axis=-1)


def world_to_cam_matrix(cam_tf: carla.Transform) -> np.ndarray:
    return np.array(cam_tf.get_inverse_matrix())


def classify(actor) -> str | None:
    if actor.type_id.startswith("walker.pedestrian"):
        return "pedestrian"
    if actor.type_id.startswith("vehicle."):
        base = actor.attributes.get("base_type", "").lower()
        if base in CLASS_OF_BASE:
            return CLASS_OF_BASE[base]
        n = int(actor.attributes.get("number_of_wheels", 4))
        return "bicycle" if n == 2 else "car"
    return None


def sample_mounting(world_map, rng: random.Random):
    """Pick a camera pose looking along a road towards a junction."""
    wps = [w for w in world_map.generate_waypoints(4.0) if not w.is_junction and w.lane_type == carla.LaneType.Driving]
    for _ in range(200):
        w = rng.choice(wps)
        ahead, dist = w, 0.0
        for _ in range(40):
            nxt = ahead.next(4.0)
            if not nxt:
                break
            ahead, dist = nxt[0], dist + 4.0
            if ahead.is_junction:
                break
        if not ahead.is_junction or dist < 15:
            continue
        kind = rng.choices(["bridge", "pole", "corner"], [0.4, 0.35, 0.25])[0]
        tf = w.transform
        fwd = tf.get_forward_vector()
        right = tf.get_right_vector()
        back = rng.uniform(0, 30) if kind != "corner" else 0
        if kind == "bridge":
            lat = rng.uniform(-6, 6)
            height = rng.uniform(5, 10)
            target_d = rng.uniform(30, 90)
        elif kind == "pole":
            lat = rng.choice([-1, 1]) * rng.uniform(w.lane_width * 1.5, w.lane_width * 3 + 4)
            height = rng.uniform(4, 9)
            target_d = rng.uniform(20, 70)
        else:
            # Junction corner, looking into the junction like a signal-mast camera.
            jc = ahead.get_junction().bounding_box.location
            lat = rng.choice([-1, 1]) * rng.uniform(8, 18)
            height = rng.uniform(6, 12)
            target_d = rng.uniform(15, 40)
            loc = carla.Location(jc.x + right.x * lat - fwd.x * 15, jc.y + right.y * lat - fwd.y * 15, jc.z)
            yaw = math.degrees(math.atan2(jc.y - loc.y, jc.x - loc.x)) + rng.uniform(-25, 25)
            pitch = -math.degrees(math.atan2(height, target_d)) + rng.uniform(-3, 3)
            return carla.Transform(carla.Location(loc.x, loc.y, loc.z + height), carla.Rotation(pitch=pitch, yaw=yaw,
                                   roll=rng.uniform(-1.5, 1.5))), kind
        base = tf.location
        loc = carla.Location(base.x - fwd.x * back + right.x * lat, base.y - fwd.y * back + right.y * lat, base.z + height)
        yaw = tf.rotation.yaw + rng.uniform(-15, 15) - math.degrees(math.atan2(lat, target_d)) * rng.uniform(0.5, 1.0)
        pitch = -math.degrees(math.atan2(height, target_d)) + rng.uniform(-2, 2)
        pitch = max(min(pitch, -2.5), -40)
        return carla.Transform(loc, carla.Rotation(pitch=pitch, yaw=yaw, roll=rng.uniform(-1.5, 1.5))), kind
    raise RuntimeError("no mounting found")


def spawn_traffic(client, world, rng: random.Random, n_veh: int, n_walk: int, tm_port: int):
    bp = world.get_blueprint_library()
    vehicles_bp = [b for b in bp.filter("vehicle.*")]
    spawn_points = world.get_map().get_spawn_points()
    rng.shuffle(spawn_points)
    tm = client.get_trafficmanager(tm_port)
    tm.set_synchronous_mode(True)
    tm.set_global_distance_to_leading_vehicle(2.0)
    actors = []
    batch = []
    for sp in spawn_points[:n_veh]:
        b = rng.choice(vehicles_bp)
        if b.has_attribute("color"):
            b.set_attribute("color", rng.choice(b.get_attribute("color").recommended_values))
        batch.append(carla.command.SpawnActor(b, sp).then(carla.command.SetAutopilot(carla.command.FutureActor, True, tm_port)))
    for r in client.apply_batch_sync(batch, True):
        if not r.error:
            actors.append(r.actor_id)
    walkers, controllers = [], []
    wbps = bp.filter("walker.pedestrian.*")
    wbatch = []
    for _ in range(n_walk):
        loc = world.get_random_location_from_navigation()
        if loc:
            wb = rng.choice(wbps)
            if wb.has_attribute("is_invincible"):
                wb.set_attribute("is_invincible", "false")
            wbatch.append(carla.command.SpawnActor(wb, carla.Transform(loc)))
    for r in client.apply_batch_sync(wbatch, True):
        if not r.error:
            walkers.append(r.actor_id)
    cbp = bp.find("controller.ai.walker")
    cbatch = [carla.command.SpawnActor(cbp, carla.Transform(), w) for w in walkers]
    for r in client.apply_batch_sync(cbatch, True):
        if not r.error:
            controllers.append(r.actor_id)
    world.tick()
    world.set_pedestrians_cross_factor(0.3)
    for cid in controllers:
        c = world.get_actor(cid)
        c.start()
        c.go_to_location(world.get_random_location_from_navigation())
        c.set_max_speed(1.0 + rng.random())
    return actors, walkers, controllers


def labels_for(world, cam_tf, K, inst: np.ndarray, rng_max=220.0):
    """3D boxes of visible actors in the OpenCV camera frame."""
    M = world_to_cam_matrix(cam_tf)
    # Instance camera: actor id encoded in G (low byte) and B (high byte).
    ids = inst[:, :, 1].astype(np.int32) + (inst[:, :, 0].astype(np.int32) << 8)
    uniq, counts = np.unique(ids, return_counts=True)
    pix = dict(zip(uniq.tolist(), counts.tolist()))
    objs, bottoms, ignore = [], [], []
    for a in list(world.get_actors().filter("vehicle.*")) + list(world.get_actors().filter("walker.pedestrian.*")):
        cls = classify(a)
        if cls is None:
            continue
        npx = pix.get(a.id & 0xFFFF, 0)
        if npx < 25:
            continue
        tf = a.get_transform()
        bb = a.bounding_box
        # Bottom centre in world coordinates.
        c_local = carla.Location(bb.location.x, bb.location.y, bb.location.z - bb.extent.z)
        bw = tf.transform(c_local)
        p = M @ np.array([bw.x, bw.y, bw.z, 1.0])
        bottom = ue_to_cv(p[:3])
        if bottom[2] < 1.0 or np.linalg.norm(bottom[[0, 2]]) > rng_max:
            continue
        fv = tf.get_forward_vector()
        hw = M[:3, :3] @ np.array([fv.x, fv.y, fv.z])
        heading = ue_to_cv(hw)
        dims = [2 * bb.extent.x, 2 * bb.extent.y, 2 * bb.extent.z]
        if cls == "pedestrian":
            dims = [max(dims[0], 0.5), max(dims[1], 0.5), dims[2]]
        # Visible 2D extent from the instance mask, projected box for the network target.
        verts = [tf.transform(carla.Location(bb.location.x + sx * bb.extent.x, bb.location.y + sy * bb.extent.y,
                                             bb.location.z + sz * bb.extent.z))
                 for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
        vc = np.array([ue_to_cv((M @ np.array([v.x, v.y, v.z, 1.0]))[:3]) for v in verts])
        if (vc[:, 2] < 0.3).any():
            continue
        uv = vc @ K.T
        uv = uv[:, :2] / uv[:, 2:3]
        x1, y1 = np.clip(uv.min(0), 0, [W - 1, H - 1])
        x2, y2 = np.clip(uv.max(0), 0, [W - 1, H - 1])
        kp = K @ bottom
        kp = kp[:2] / kp[2]
        if not (0 <= kp[0] < W and 0 <= kp[1] < H):
            continue
        area = max(1.0, (x2 - x1) * (y2 - y1))
        vis = npx / area
        if vis < MIN_VISIBLE or npx < 60:
            # Barely visible: neither a positive (would teach hallucination) nor a negative.
            ignore.append([float(x1), float(y1), float(x2), float(y2)])
            continue
        objs.append({"cls": cls, "type": a.type_id, "bottom": bottom.tolist(), "dims": dims,
                     "heading": heading.tolist(), "box2d": [float(x1), float(y1), float(x2), float(y2)],
                     "trunc": 0.0, "num_points": int(npx), "occlusion": f"{min(1.0, npx / area):.2f}"})
        bottoms.append(bottom)
    return objs, bottoms, ignore


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--maps", default="Town03,Town05,Town10HD_Opt")
    ap.add_argument("--placements", type=int, default=50, help="per map")
    ap.add_argument("--frames", type=int, default=25, help="per placement")
    ap.add_argument("--interval", type=int, default=12, help="ticks between saved frames (20 Hz)")
    ap.add_argument("--vehicles", type=int, default=90)
    ap.add_argument("--walkers", type=int, default=70)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--split_val", type=float, default=0.1)
    args = ap.parse_args()
    rng = random.Random(args.seed)
    out = args.out.expanduser()
    (out / "images").mkdir(parents=True, exist_ok=True)
    client = carla.Client("localhost", args.port)
    client.set_timeout(120.0)
    writers = {s: open(out / f"{s}.jsonl", "a") for s in ("train", "val")}
    for town in args.maps.split(","):
        world = client.load_world(town)
        # Parked cars are static map props without actor ids (no labels): remove them.
        if town.endswith("_Opt"):
            world.unload_map_layer(carla.MapLayer.ParkedVehicles)
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)
        actors, walkers, controllers = spawn_traffic(client, world, rng, args.vehicles, args.walkers, 8000)
        for _ in range(100):
            world.tick()
        for pi in range(args.placements):
            w = getattr(carla.WeatherParameters, rng.choice(WEATHERS))
            if rng.random() < 0.25:
                w.fog_density = rng.uniform(10, 60)
                w.fog_distance = rng.uniform(5, 40)
            world.set_weather(w)
            cam_tf, kind = sample_mounting(world.get_map(), rng)
            fov = rng.uniform(20, 90)
            f = W / (2 * math.tan(math.radians(fov) / 2))
            K = np.array([[f, 0, W / 2], [0, f, H / 2], [0, 0, 1]])
            bp = world.get_blueprint_library()
            sensors, qs = [], []
            for name in ("sensor.camera.rgb", "sensor.camera.instance_segmentation"):
                b = bp.find(name)
                b.set_attribute("image_size_x", str(W))
                b.set_attribute("image_size_y", str(H))
                b.set_attribute("fov", f"{fov:.3f}")
                if name.endswith("rgb"):
                    b.set_attribute("motion_blur_intensity", "0.2")
                s = world.spawn_actor(b, cam_tf)
                q = queue.Queue()
                s.listen(q.put)
                sensors.append(s)
                qs.append(q)
            split = "val" if rng.random() < args.split_val else "train"
            saved = 0
            for fi in range(args.frames * args.interval):
                frame = world.tick()
                imgs = []
                for q in qs:
                    while True:
                        d = q.get(timeout=30)
                        if d.frame == frame:
                            break
                    imgs.append(d)
                if fi % args.interval:
                    continue
                rgb = np.frombuffer(imgs[0].raw_data, dtype=np.uint8).reshape(H, W, 4)[:, :, :3]
                inst = np.frombuffer(imgs[1].raw_data, dtype=np.uint8).reshape(H, W, 4)[:, :, :3]
                if fi == 0:
                    # Reject mountings whose view is mostly blocked (camera inside a building, behind a tree).
                    sem = inst[:, :, 2]
                    blocked = float(np.isin(sem, BLOCKING_TAGS).mean())
                    if blocked > 0.45:
                        print(f"  rejected mounting ({blocked:.0%} blocked)", flush=True)
                        break
                objs, bottoms, ignore = labels_for(world, cam_tf, K, inst)
                # Ground plane in camera frame: world up through the mean bottom height of visible objects,
                # or through the road below the view centre.
                Mi = world_to_cam_matrix(cam_tf)
                up = ue_to_cv(Mi[:3, :3] @ np.array([0, 0, 1.0]))
                if bottoms:
                    d0 = -float(np.median([up @ b for b in bottoms]))
                else:
                    wp = world.get_map().get_waypoint(cam_tf.location)
                    gz = wp.transform.location.z
                    pg = ue_to_cv((Mi @ np.array([cam_tf.location.x, cam_tf.location.y, gz, 1.0]))[:3])
                    d0 = -float(up @ pg)
                name = f"{town}_{pi:03d}_{saved:03d}.jpg"
                cv2.imwrite(str(out / "images" / name), rgb, [cv2.IMWRITE_JPEG_QUALITY, 90])
                writers[split].write(json.dumps({
                    "release": "CARLA", "image": f"images/{name}", "camera": f"carla_{kind}", "frame_id": name[:-4],
                    "width": W, "height": H, "K": K.tolist(), "plane": [*up.tolist(), d0], "objects": objs, "ignore": ignore,
                    "weather": str(w), "fov": fov}) + "\n")
                saved += 1
            for s in sensors:
                s.stop()
                s.destroy()
            for wr in writers.values():
                wr.flush()
            print(f"{town} placement {pi + 1}/{args.placements} ({kind}, fov {fov:.0f}) saved {saved}", flush=True)
        client.apply_batch([carla.command.DestroyActor(x) for x in actors + walkers + controllers])
        world.tick()
    for wr in writers.values():
        wr.close()


if __name__ == "__main__":
    main()
