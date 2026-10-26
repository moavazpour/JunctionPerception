"""Summary of a runtime metadata recording (FileSink jsonl): calibration, road users, lanes, events, health.

    python tools/summarize_metadata.py state/metadata_cam1.jsonl
"""
from __future__ import annotations

import json
import math
import sys
from collections import Counter, defaultdict


def main():
    path = sys.argv[1]
    kinds = Counter()
    ids = {}
    speeds = defaultdict(list)
    per_frame = []
    events = Counter()
    health = Counter()
    calib = []
    lanes = 0
    try:
        lines = open(path).read().splitlines()
    except FileNotFoundError:
        print("no metadata was produced")
        return
    for line in lines:
        try:
            r = json.loads(line)
        except json.JSONDecodeError:
            continue
        leaf = "/".join(r["topic"].split("/")[3:])
        m = r["msg"]
        kinds[leaf.split("/")[0] if leaf.startswith("detector/") else leaf] += 1
        if leaf == "objects":
            per_frame.append(len(m["objects"]))
            for o in m["objects"]:
                ids[o["id"]] = o["class"]
                if o.get("state") == "tracked" and o.get("speed") is not None:
                    speeds[o["class"]].append(o["speed"])
        elif leaf == "calibration":
            calib.append(m["calibration"])
        elif leaf == "events/safety":
            e = m["event"]
            events[(e.get("kind"), e.get("severity"))] += 1
        elif leaf == "health":
            health[m["health"].get("level")] += 1
        elif leaf == "map":
            lanes = len(m["map"].get("lanes", []))
    print(f"messages: {dict(kinds)}")
    for c in calib:
        n = c["plane"]
        print(f"calibration ({c.get('source')}, {c.get('method', '')}): f={c['K'][0][0]:.0f} px, "
              f"pitch={math.degrees(math.asin(-n[2])):.2f} deg, roll={math.degrees(math.atan2(n[0], -n[1])):.2f} deg, "
              f"height={n[3]:.2f} m, bearing={c.get('bearing_deg', 0):.1f} deg, confidence={c.get('confidence', '-')}")
    if per_frame:
        print(f"objects per message: mean {sum(per_frame) / len(per_frame):.1f}, max {max(per_frame)}")
    print(f"distinct tracks: {len(ids)} {dict(Counter(ids.values()))}")
    for c, v in sorted(speeds.items()):
        v = sorted(v)
        print(f"  {c:10s} speed median {3.6 * v[len(v) // 2]:5.1f} km/h, p90 {3.6 * v[int(0.9 * (len(v) - 1))]:5.1f} km/h")
    print(f"learned lanes: {lanes}")
    print(f"safety events: {dict(events)}")
    print(f"health levels: {dict(health)}")


if __name__ == "__main__":
    main()
