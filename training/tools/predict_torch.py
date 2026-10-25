"""Run the PyTorch checkpoint over a JP3D split and write predictions in the jp_bench format
(reference for checking that the TensorRT deployment preserves accuracy).

    python -m tools.predict_torch --ckpt best.pt --data test.jsonl --images ROOT --out preds_torch.jsonl
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import torch

from jp3d.dataset import JP3DDataset
from jp3d.lift import lift
from jp3d.model import DeployWrapper, GroundCenter3D


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", type=Path, required=True)
    ap.add_argument("--data", type=Path, required=True)
    ap.add_argument("--images", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=736)
    args = ap.parse_args()
    ck = torch.load(args.ckpt, map_location="cpu")
    model = GroundCenter3D(ck.get("backbone", "dla34"), pretrained=False)
    model.load_state_dict(ck["model"])
    w = DeployWrapper(model).cuda().eval()
    ds = JP3DDataset(args.data, args.images, args.width, args.height, train=False)
    with open(args.out, "w") as f, torch.no_grad():
        for i in range(len(ds)):
            x, _, meta = ds[i]
            dets = w(x[None].cuda()).float().cpu().numpy()[0]
            K, plane = meta["K"].numpy().astype(float), meta["plane"].numpy().astype(float)
            objs = lift(dets, K, plane, 0.05)
            s = ds.samples[i]
            # Back to original-image pixels for box2d (eval-mode letterbox: scale + offset).
            sc = K[0, 0] / s["K"][0][0]
            ox, oy = K[0, 2] - s["K"][0][2] * sc, K[1, 2] - s["K"][1][2] * sc
            for o in objs:
                b = o["box2d"]
                o["box2d"] = [(b[0] - ox) / sc, (b[1] - oy) / sc, (b[2] - ox) / sc, (b[3] - oy) / sc]
                o.pop("kp", None)
            f.write(json.dumps({"image": s["image"], "objects": objs}, default=float) + "\n")
    print("wrote", args.out)


if __name__ == "__main__":
    np.set_printoptions(precision=3)
    main()
