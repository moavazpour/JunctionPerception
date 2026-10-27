"""Repair front/back flips of the heading in a converted source from vehicle motion (in place).

Some sources (DAIR V2X-Seq SPD) label the box heading inconsistently: whole tracks reversed, or flipping by
180 deg between frames. For a fixed camera the direction of travel resolves it: per track, frames where
the vehicle moves (> min_speed) get the heading that points along the motion; standing frames take the
orientation of the nearest moving frame of the same track (vehicles do not turn around on the spot);
tracks that never move are left as labelled. A flip swaps the corner labels (FL<->RR, FR<->RL) and
negates the heading; the box itself is unchanged.

    python -m jpdata.fix_heading --data converted/dair_spd_infra/data.jsonl
"""
from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path

import numpy as np

FLIP = [0, 3, 4, 1, 2, 7, 8, 5, 6]  # keypoint order of the same cuboid seen with its heading reversed
VEH = {"car", "bus", "truck", "trailer", "motorcycle", "bicycle"}


def flip(o: dict):
    o["heading"] = [-v for v in o["heading"]]
    if "kpts2d" in o:
        k = o["kpts2d"]
        o["kpts2d"] = [k[i] for i in FLIP]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", type=Path, required=True)
    ap.add_argument("--min_speed", type=float, default=1.5, help="m/s: slower is treated as standing")
    ap.add_argument("--max_dt", type=float, default=0.5, help="s: longer gaps are not used for motion")
    a = ap.parse_args()
    samples = [json.loads(l) for l in a.data.read_text().splitlines() if l.strip()]
    # Observations per (camera, track): (time, sample index, object index).
    tracks = defaultdict(list)
    for si, s in enumerate(samples):
        t = s.get("time")
        if not t:
            continue
        for oi, o in enumerate(s["objects"]):
            if o.get("track_id") and o["cls"] in VEH:
                tracks[(s["camera"], o["track_id"])].append((t, si, oi))
    n_obs = n_flip = n_moving = n_static_tracks = 0
    for obs in tracks.values():
        obs.sort()
        pos = np.array([samples[si]["objects"][oi]["bottom"] for _, si, oi in obs])
        ts = np.array([t for t, _, _ in obs])
        # Velocity per observation from its neighbours (central difference where possible).
        want = [None] * len(obs)
        for i in range(len(obs)):
            j0, j1 = max(i - 1, 0), min(i + 1, len(obs) - 1)
            dt = ts[j1] - ts[j0]
            if j1 == j0 or dt <= 0 or dt > 2 * a.max_dt:
                continue
            v = (pos[j1] - pos[j0]) / dt
            if np.linalg.norm(v) >= a.min_speed:
                want[i] = v / np.linalg.norm(v)
        moving = [i for i in range(len(obs)) if want[i] is not None]
        n_obs += len(obs)
        if not moving:
            n_static_tracks += 1
            continue
        n_moving += len(moving)
        for i in range(len(obs)):
            _, si, oi = obs[i]
            o = samples[si]["objects"][oi]
            h = np.array(o["heading"])
            if want[i] is not None:
                ref = want[i]
            else:
                # Standing: orientation of the nearest moving observation (already corrected below).
                k = min(moving, key=lambda m: abs(ts[m] - ts[i]))
                ref = np.array(samples[obs[k][1]]["objects"][obs[k][2]]["heading"])
                if want[k] is not None and np.dot(ref, want[k]) < 0:
                    ref = -ref
            if np.dot(h, ref) < 0:
                flip(o)
                n_flip += 1
    with open(a.data, "w") as f:
        for s in samples:
            f.write(json.dumps(s) + "\n")
    print(f"{a.data.parent.name}: {len(tracks)} tracks, {n_obs} observations ({n_moving} moving); "
          f"{n_flip} headings flipped ({100 * n_flip / max(n_obs, 1):.1f} %); {n_static_tracks} tracks never move (unchanged)")


if __name__ == "__main__":
    main()
