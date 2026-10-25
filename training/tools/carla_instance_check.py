"""Check which instance-segmentation id encoding matches CARLA actor ids (pixels must fall inside the
projected box of their actor)."""
from __future__ import annotations

import math
import queue
import random

import carla
import numpy as np

from tools.carla_gen import W, H, labels_for, sample_mounting, spawn_traffic, ue_to_cv, world_to_cam_matrix


def try_mount(world, tf, fov) -> bool:
    """True when several vehicles are within the view frustum of a mounting."""
    M = world_to_cam_matrix(tf)
    f = W / (2 * math.tan(math.radians(fov) / 2))
    n = 0
    for a in world.get_actors().filter("vehicle.*"):
        l = a.get_location()
        p = ue_to_cv((M @ np.array([l.x, l.y, l.z, 1.0]))[:3])
        if 3 < p[2] < 80 and abs(p[0] / p[2]) * f < W / 2 and abs(p[1] / p[2]) * f < H / 2:
            n += 1
    return n >= 4


def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(120)
    world = client.load_world("Town10HD_Opt")
    world.unload_map_layer(carla.MapLayer.ParkedVehicles)
    s = world.get_settings()
    s.synchronous_mode = True
    s.fixed_delta_seconds = 0.05
    world.apply_settings(s)
    rng = random.Random(1)
    spawn_traffic(client, world, rng, 80, 40, 8000)
    for _ in range(60):
        world.tick()
    fov = 50
    for _try in range(30):
        tf, kind = sample_mounting(world.get_map(), rng)
        if try_mount(world, tf, fov):
            break
    f = W / (2 * math.tan(math.radians(fov) / 2))
    K = np.array([[f, 0, W / 2], [0, f, H / 2], [0, 0, 1]])
    b = world.get_blueprint_library().find("sensor.camera.instance_segmentation")
    b.set_attribute("image_size_x", str(W))
    b.set_attribute("image_size_y", str(H))
    b.set_attribute("fov", str(fov))
    cam = world.spawn_actor(b, tf)
    q = queue.Queue()
    cam.listen(q.put)
    fr = world.tick()
    d = q.get(timeout=30)
    while d.frame != fr:
        d = q.get(timeout=30)
    raw = np.frombuffer(d.raw_data, dtype=np.uint8).reshape(H, W, 4)
    B, Gc, R = raw[..., 0].astype(int), raw[..., 1].astype(int), raw[..., 2].astype(int)
    veh = np.isin(R, [12, 13, 14, 15, 16, 18, 19])
    print("vehicle/person pixels:", int(veh.sum()))
    for name0, ids0 in {"G+B<<8": Gc + (B << 8), "B+G<<8": B + (Gc << 8)}.items():
        u = np.unique(ids0[veh])
        print(name0, "ids on vehicle pixels:", u[:15].tolist(), "actor ids:", sorted(a.id for a in world.get_actors().filter("vehicle.*"))[:15])
    encodings = {"G+B<<8": Gc + (B << 8), "B+G<<8": B + (Gc << 8)}
    M = world_to_cam_matrix(tf)
    for name, ids in encodings.items():
        inside = total = 0
        for a in list(world.get_actors().filter("vehicle.*")):
            mask = ids == (a.id & 0xFFFF)
            n = int(mask.sum())
            if n == 0:
                continue
            bb, t = a.bounding_box, a.get_transform()
            vs = [t.transform(carla.Location(bb.location.x + sx * bb.extent.x, bb.location.y + sy * bb.extent.y,
                                             bb.location.z + sz * bb.extent.z))
                  for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
            vc = np.array([ue_to_cv((M @ np.array([v.x, v.y, v.z, 1.0]))[:3]) for v in vs])
            if (vc[:, 2] < 0.3).any():
                continue
            uv = vc @ K.T
            uv = uv[:, :2] / uv[:, 2:3]
            ys, xs = np.nonzero(mask)
            ok = (xs >= uv[:, 0].min() - 5) & (xs <= uv[:, 0].max() + 5) & (ys >= uv[:, 1].min() - 5) & (ys <= uv[:, 1].max() + 5)
            inside += int(ok.sum())
            total += n
        print(name, "pixels matched to actors:", total, "inside own projected box:", inside,
              f"({inside / max(total, 1):.1%})")
    print("semantic tags present (R):", np.unique(R)[:30])
    cam.destroy()


if __name__ == "__main__":
    main()
