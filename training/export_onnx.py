"""Export the trained detector (with in-graph decoding) to ONNX.

    python export_onnx.py --ckpt runs/gc3d_dla34/best.pt --out ../models/gc3d.onnx --width 1280 --height 736
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import torch

from jp3d.dataset import MEAN, STD
from jp3d.geometry import CLASSES
from jp3d.model import DeployWrapper, GroundCenter3D


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=736)
    ap.add_argument("--topk", type=int, default=256)
    args = ap.parse_args()
    ck = torch.load(args.ckpt, map_location="cpu")
    model = GroundCenter3D(ck.get("backbone", "dla34"), pretrained=False)
    model.load_state_dict(ck["model"])
    wrapper = DeployWrapper(model, topk=args.topk).eval()
    x = torch.zeros(1, 3, args.height, args.width)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    torch.onnx.export(wrapper, x, str(args.out), input_names=["image"], output_names=["dets"], opset_version=17,
                      do_constant_folding=True, dynamo=False)
    meta = {
        "input": {"width": args.width, "height": args.height, "format": "RGB", "layout": "NCHW",
                  "mean": (MEAN).tolist(), "std": (STD).tolist()},
        "output": {"name": "dets", "topk": args.topk,
                   "fields": ["score", "class", "kp_u", "kp_v", "box_l", "box_t", "box_r", "box_b",
                              "dz", "l", "w", "h", "alpha"]},
        "classes": CLASSES,
        "checkpoint": str(args.ckpt),
        "epoch": ck.get("epoch"),
        "val_mAP_dist": ck.get("best"),
    }
    args.out.with_suffix(".json").write_text(json.dumps(meta, indent=1))
    try:
        import onnxruntime as ort
        sess = ort.InferenceSession(str(args.out), providers=["CPUExecutionProvider"])
        xr = torch.randn(1, 3, args.height, args.width)
        a = sess.run(None, {"image": xr.numpy()})[0]
        with torch.no_grad():
            b = wrapper(xr).numpy()
        print("onnx vs torch max abs diff (scores):", float(np.abs(a[..., 0] - b[..., 0]).max()))
    except ImportError:
        pass
    print("exported", args.out)


if __name__ == "__main__":
    main()
