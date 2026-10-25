"""Export a Cityscapes SegFormer to ONNX with in-graph upsampling + argmax (output: int64 [1,H,W] labels).

    python export_segformer.py --model nvidia/segformer-b2-finetuned-cityscapes-1024-1024 \
        --out ../models/segformer_b2_cityscapes.onnx --width 1024 --height 576
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import torch
import torch.nn as nn
import torch.nn.functional as F


class SegWrapper(nn.Module):
    def __init__(self, model):
        super().__init__()
        self.model = model

    def forward(self, x):
        logits = self.model(pixel_values=x).logits
        logits = F.interpolate(logits, size=x.shape[-2:], mode="bilinear", align_corners=False)
        return torch.argmax(logits, dim=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="nvidia/segformer-b2-finetuned-cityscapes-1024-1024")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--width", type=int, default=1024)
    ap.add_argument("--height", type=int, default=576)
    args = ap.parse_args()
    from transformers import SegformerForSemanticSegmentation

    model = SegformerForSemanticSegmentation.from_pretrained(args.model).eval()
    w = SegWrapper(model).eval()
    x = torch.zeros(1, 3, args.height, args.width)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    torch.onnx.export(w, x, str(args.out), input_names=["image"], output_names=["labels"], opset_version=17,
                      do_constant_folding=True, dynamo=False)
    args.out.with_suffix(".json").write_text(json.dumps({
        "source": args.model, "input": {"width": args.width, "height": args.height, "format": "RGB",
                                        "mean": [123.675, 116.28, 103.53], "std": [58.395, 57.12, 57.375]},
        "labels": list(model.config.id2label.values())}, indent=1))
    print("exported", args.out)


if __name__ == "__main__":
    main()
