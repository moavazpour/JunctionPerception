"""Fetch COCO-pretrained YOLO26 weights and export them to an end-to-end (NMS-free) ONNX model.

Requires the `ultralytics` and `onnx` packages. Weights and the exported model are written to
--out-dir (default: models/, not tracked by git) as <variant>.pt and <variant>_<imgsz>.onnx.

Example:
    python tools/export_yolo26.py --variant yolo26s --imgsz 640
"""

import argparse
import os
import pathlib
import shutil

import onnx
from ultralytics import YOLO

VARIANTS = ("yolo26n", "yolo26s", "yolo26m", "yolo26l", "yolo26x")


def describe(model_path: pathlib.Path) -> None:
    """Print the input and output tensors and the metadata of an ONNX model."""
    model = onnx.load(str(model_path))

    def shape(value_info) -> list:
        return [d.dim_param or d.dim_value for d in value_info.type.tensor_type.shape.dim]

    for tensor in model.graph.input:
        print(f"input  {tensor.name}: {shape(tensor)}")
    for tensor in model.graph.output:
        print(f"output {tensor.name}: {shape(tensor)}")
    for prop in model.metadata_props:
        print(f"meta   {prop.key}: {prop.value}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--variant", default="yolo26s", choices=VARIANTS)
    parser.add_argument("--imgsz", type=int, default=640, help="square network input size (px)")
    parser.add_argument("--out-dir", type=pathlib.Path, default=pathlib.Path("models"))
    args = parser.parse_args()

    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    # Ultralytics downloads named weights into the working directory.
    os.chdir(out_dir)
    model = YOLO(f"{args.variant}.pt")
    # nms=False selects the NMS-free one-to-one head (end-to-end output, no NMS afterwards).
    exported = pathlib.Path(
        model.export(format="onnx", imgsz=args.imgsz, dynamic=False, nms=False)
    )

    target = out_dir / f"{args.variant}_{args.imgsz}.onnx"
    shutil.move(exported, target)
    print(f"wrote {target}")
    describe(target)


if __name__ == "__main__":
    main()
