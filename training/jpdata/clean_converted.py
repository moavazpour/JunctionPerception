"""Final cleaning of one converted source, in place.

  * removes every ignore region (JP-Unified trains on the labels only)
  * optionally drops images by name prefix (e.g. a camera whose labels are out of sync)
  * optionally drops black / unreadable frames (and deletes their files if they belong to the source)

    python -m jpdata.clean_converted --data converted/lumpi/data.jsonl --root converted/lumpi/images \
        --drop_prefix M2_c8_ --check_black --delete_files
"""
from __future__ import annotations

import argparse
import json
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import cv2


def is_black(path: Path) -> bool:
    im = cv2.imread(str(path), cv2.IMREAD_REDUCED_GRAYSCALE_4)
    return im is None or (im < 12).mean() > 0.5


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", type=Path, required=True)
    ap.add_argument("--root", type=Path, required=True)
    ap.add_argument("--drop_prefix", action="append", default=[])
    ap.add_argument("--check_black", action="store_true")
    ap.add_argument("--delete_files", action="store_true", help="delete dropped images (only for frames the converter wrote)")
    a = ap.parse_args()
    samples = [json.loads(l) for l in a.data.read_text().splitlines() if l.strip()]
    drop = {s["image"] for s in samples if any(s["image"].startswith(p) for p in a.drop_prefix)}
    n_prefix = len(drop)
    n_black = 0
    if a.check_black:
        todo = [s["image"] for s in samples if s["image"] not in drop]
        with ThreadPoolExecutor(16) as ex:
            black = [i for i, b in zip(todo, ex.map(lambda i: is_black(a.root / i), todo)) if b]
        n_black = len(black)
        drop.update(black)
    n_ign = 0
    with open(a.data, "w") as f:
        for s in samples:
            if s["image"] in drop:
                continue
            n_ign += len(s.pop("ignore", []))
            s.pop("ignore_det", None)
            f.write(json.dumps(s) + "\n")
    if a.delete_files:
        for i in drop:
            (a.root / i).unlink(missing_ok=True)
    print(f"{a.data.parent.name}: {len(samples)} -> {len(samples) - len(drop)} images "
          f"(dropped by prefix {n_prefix}, black {n_black}); ignore regions removed {n_ign}")


if __name__ == "__main__":
    main()
