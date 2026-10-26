"""Compare a runtime calibration file against known ground truth.

    python tools/compare_calibration.py state/calibration_tumtraf_south2.json --f 1316 --pitch 19.7 --roll -0.6 --height 8.72
"""
from __future__ import annotations

import argparse
import json
import math


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("calibration")
    ap.add_argument("--f", type=float, required=True)
    ap.add_argument("--pitch", type=float, required=True, help="deg, down positive")
    ap.add_argument("--roll", type=float, required=True, help="deg")
    ap.add_argument("--height", type=float, required=True, help="m")
    a = ap.parse_args()
    try:
        c = json.load(open(a.calibration))
    except FileNotFoundError:
        print("no calibration was produced")
        return
    f = c["K"][0][0]
    n = c["plane"]
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -n[2]))))
    roll = math.degrees(math.atan2(n[0], -n[1]))
    h = n[3]
    print(f"source {c.get('source')}, method {c.get('method', '?')}, confidence {c.get('confidence', '?')}")
    print(f"focal  {f:8.1f} px  truth {a.f:8.1f}  error {100 * (f / a.f - 1):+.1f} %")
    print(f"pitch  {pitch:8.2f} deg truth {a.pitch:8.2f}  error {pitch - a.pitch:+.2f} deg")
    print(f"roll   {roll:8.2f} deg truth {a.roll:8.2f}  error {roll - a.roll:+.2f} deg")
    print(f"height {h:8.2f} m   truth {a.height:8.2f}  error {100 * (h / a.height - 1):+.1f} %")
    # Range error of a ground point straight ahead at typical distances (flat ground, centre column).
    for d in (25, 50, 100):
        ang_t = math.radians(a.pitch) - math.atan2(a.height, d)  # angle below the optical axis, truth
        # pixel row offset in the truth camera, then back-projected through the estimate
        v = a.f * math.tan(-ang_t)
        ang_e = math.atan2(v, f)
        dep = math.radians(pitch) + ang_e
        d_e = h / math.tan(dep) if dep > 0 else float("inf")
        print(f"range at {d:3d} m -> {d_e:7.1f} m ({100 * (d_e / d - 1):+.1f} %)")


if __name__ == "__main__":
    main()
