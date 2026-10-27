"""Export a trained YOLO26-pose checkpoint to the ONNX layout the C++ runtime reads.

The runtime expects one input [1, 3, S, S] (RGB, 0..1, letterboxed with 114 padding) and one end-to-end
output [1, K, 6 + 9 * 3]: x1 y1 x2 y2 score class, then 9 keypoints (u, v, confidence) in input pixels.
The TensorRT engine is built from this ONNX by the runtime on first start (cached next to it).

    python -m yolo3d.export --weights runs/jp_unified_y26s_960/weights/best.pt --imgsz 960 \
        --out ~/data/junction_perception/models/jp_y26s_960.onnx [--dynamic]
"""
from __future__ import annotations

import argparse
import shutil
from pathlib import Path

import onnx
from ultralytics import YOLO


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", required=True)
    ap.add_argument("--imgsz", type=int, default=960)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--opset", type=int, default=17)
    ap.add_argument("--dynamic", action="store_true",
                    help="dynamic input height/width: the runtime then chooses the input size (settings: imgsz, road-area crop)")
    a = ap.parse_args()
    model = YOLO(a.weights)
    path = model.export(format="onnx", imgsz=a.imgsz, opset=a.opset, simplify=True, dynamic=a.dynamic, half=False,
                        device="cpu", end2end=True)
    m = onnx.load(path)
    dims = lambda t: [d.dim_value for d in t.type.tensor_type.shape.dim]  # noqa: E731
    ins, outs = [dims(t) for t in m.graph.input], [dims(t) for t in m.graph.output]
    want = [0, 3, 0, 0] if a.dynamic else [1, 3, a.imgsz, a.imgsz]  # dynamic dims read as 0
    if len(ins) != 1 or ins[0] != want or len(outs) != 1 or outs[0][2] != 6 + 27:
        raise SystemExit(f"unexpected ONNX layout: inputs {ins}, outputs {outs}")
    a.out.parent.mkdir(parents=True, exist_ok=True)
    if Path(path).resolve() != a.out.resolve():
        shutil.copy(path, a.out)
    print(f"exported {a.weights} -> {a.out}: input {ins[0]}, output {outs[0]}")


if __name__ == "__main__":
    main()
