#!/bin/bash
# After training: put the trained detector into service and measure the deployed path.
#  1. export best.pt to the runtime's ONNX layout (CPU)
#  2. build the runtime image (C++ build + unit tests)
#  3. benchmark TensorRT FP16 through the C++ detector + lifting on the held-out test split, per source
#  4. replay TUMTraf S110 south2 with auto-calibration and compare against the dev-kit ground truth
#  5. run the test-site video end to end (calibration, tracks, ETA, events) and summarise the metadata
#
#   RUN=~/JunctionPerception/data/runs/jp_unified_y26s_960 tools/deploy_model.sh
set -eu
J=/home/mo_avazpour/JunctionPerception/data
RUN=${RUN:-$J/runs/jp_unified_y26s_960}
IMGSZ=${IMGSZ:-960}
NAME=${NAME:-jp_y26s_960}
PY=~/miniconda3/envs/yolo/bin/python
PJ=~/miniconda3/envs/jp/bin/python
REPO=$(cd "$(dirname "$0")/.." && pwd)
B=$J/runs/deploy_$NAME
mkdir -p $B $J/state
log() { echo "$(date '+%F %T') $*"; }

log "1. export"
cd $REPO/training
$PY -m yolo3d.export --weights $RUN/weights/best.pt --imgsz $IMGSZ --out $J/models/$NAME.onnx

log "2. runtime image"
cd $REPO
docker build -q -t junction-perception:latest -f docker/Dockerfile . > /dev/null
docker build -q -t jp-build:latest --target build -f docker/Dockerfile . > /dev/null
run() { docker run --rm --gpus all -v $J:$J -v $J/models:/models "$@"; }

log "3. C++ benchmark on the test split"
cd $REPO/training
for s in $(python3 -c "import json; print(' '.join(sorted({json.loads(l)['source'] for l in open('$J/merged/test.jsonl')})))"); do
  grep "\"source\": \"$s\"" $J/merged/test.jsonl > $B/test_$s.jsonl
  run --entrypoint /opt/jp/bin/jp_bench junction-perception:latest \
      --onnx /models/$NAME.onnx --data $B/test_$s.jsonl --images / --out $B/trt_$s.jsonl 2> $B/trt_$s.log
  $PJ -m tools.eval_predictions --gt $B/test_$s.jsonl --pred $B/trt_$s.jsonl --out $B/report_$s.json > $B/report_$s.txt 2>&1
  log "   $s: $(grep -m1 -i 'map\|ap' $B/report_$s.txt || true)"
done

log "4. TUMTraf auto-calibration vs ground truth"
V=$J/raw/test_videos
[ -f $V/tumtraf_s03_south2_8mm.mp4 ] || $PJ -m tools.make_tumtraf_video \
  --images $J/raw/TUMTraf/R2/a9_dataset_r02_s03/a9_dataset_r02_s03/images/s110_camera_basler_south2_8mm \
  --out $V/tumtraf_s03_south2_8mm.mp4
rm -f $J/state/metadata_tumtraf_south2.jsonl
run -v $REPO/configs/sites:/config:ro -v $J/state:/data/state -v $V:/data/videos:ro junction-perception:latest \
    --config /config/tumtraf_s110_south2.yaml --no-realtime --max-frames 3000 > $B/tumtraf_run.log 2>&1 || true
$PJ $REPO/tools/compare_calibration.py $J/state/calibration_tumtraf_south2.json \
    --f 1316 --pitch 19.7 --roll -0.6 --height 8.72 | tee $B/tumtraf_calibration.txt

log "5. test-site video"
rm -f $J/state/metadata_cam1.jsonl $J/state/calibration_cam1.json
run -v $REPO/configs/sites:/config:ro -v $J/state:/data/state -v $V:/data/videos:ro junction-perception:latest \
    --config /config/munich_gbr_lerchenauer.yaml --no-realtime --max-frames 20000 > $B/site_run.log 2>&1 || true
$PJ $REPO/tools/summarize_metadata.py $J/state/metadata_cam1.jsonl | tee $B/site_summary.txt
log "DEPLOY_DONE ($B)"
