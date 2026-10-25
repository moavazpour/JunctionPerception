#!/usr/bin/env bash
# End-to-end TUMTraf benchmark of the deployed engine (TensorRT FP16 through the C++ runtime code path).
#   tools/run_benchmark.sh <run_dir> [split...]
# Expects: ~/data/junction_perception/{jp3d,datasets/TUMTraf,models} and the junction-perception:build image.
set -euo pipefail
RUN=${1:?run dir with best.pt}
shift || true
SPLITS=${*:-"val test"}
D=$HOME/data/junction_perception
PY=$HOME/miniconda3/envs/jp/bin/python
OUT=$RUN/bench
mkdir -p "$OUT"
cd "$(dirname "$0")/../training"

$PY export_onnx.py --ckpt "$RUN/best.pt" --out "$D/models/gc3d.onnx"
for s in $SPLITS; do
  docker run --rm --gpus all -v "$D:/d" jp-build /src/build/apps/jp_bench \
    --onnx /d/models/gc3d.onnx --data "/d/jp3d/$s.jsonl" --images /d/datasets/TUMTraf --out "/d/runs/$(basename "$RUN")/bench/trt_$s.jsonl"
  $PY -m tools.predict_torch --ckpt "$RUN/best.pt" --data "$D/jp3d/$s.jsonl" --images "$D/datasets/TUMTraf" --out "$OUT/torch_$s.jsonl"
  $PY -m tools.eval_predictions --gt "$D/jp3d/$s.jsonl" --pred "$OUT/trt_$s.jsonl" --out "$OUT/report_trt_$s.json" --by camera | tee "$OUT/report_trt_$s.txt"
  $PY -m tools.eval_predictions --gt "$D/jp3d/$s.jsonl" --pred "$OUT/torch_$s.jsonl" --out "$OUT/report_torch_$s.json" | tee "$OUT/report_torch_$s.txt"
done
