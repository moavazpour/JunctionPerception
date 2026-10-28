#!/bin/bash
# Final YOLO26-pose training, started automatically when the CARLA ablation finishes.
#  1. Waits for ABLATION_DONE.
#  2. Keeps CARLA only if it does not hurt pose mAP50-95 on the real-only validation set.
#  3. Trains on JP-Unified v2 (includes DAIR-V2X-I) at 1280 px, validating on real data.
#  4. Evaluates the best checkpoint once on the held-out test split (per source).
set -u
J=~/JunctionPerception/data
R=$J/runs
Y=$J/yolo_unified_v2
P=~/miniconda3/envs/yolo/bin/python
MODEL=${MODEL:-yolo26s-pose.pt}
EPOCHS=${EPOCHS:-60}
IMGSZ=${IMGSZ:-1280}
BATCH=${BATCH:-16}
log() { echo "$(date '+%F %T') $*"; }

until grep -q ABLATION_DONE $R/ablation.log 2>/dev/null; do sleep 120; done
until [ -f $Y/data.yaml ]; do sleep 120; done

best() {  # best pose mAP50-95 over epochs of a run
  python3 - "$1" <<'EOF'
import csv, sys
rows = list(csv.DictReader(open(sys.argv[1])))
key = next(k for k in rows[0] if "mAP50-95(P)" in k.replace(" ", ""))
print(max(float(r[key]) for r in rows))
EOF
}
A=$(best $R/ablate_all/results.csv)
B=$(best $R/ablate_nocarla/results.csv)
log "ablation pose mAP50-95 on real val: with CARLA $A, without $B"
USE_CARLA=$(python3 -c "print('yes' if $A >= $B - 0.002 else 'no')")
log "use CARLA: $USE_CARLA"

cd $Y
find $Y/images/train -maxdepth 1 \( -type f -o -type l \) | sort > lists_train_all.txt
grep -v "/carla" lists_train_all.txt > lists_train_nocarla.txt
find $Y/images/val -maxdepth 1 \( -type f -o -type l \) | grep -v "/carla" | sort > lists_val_real.txt
TRAIN=lists_train_all.txt; [ "$USE_CARLA" = no ] && TRAIN=lists_train_nocarla.txt
{
  echo "path: $Y"
  echo "train: $Y/$TRAIN"
  echo "val: $Y/lists_val_real.txt"
  echo "test: images/test"
  sed -n '/^kpt_shape/,$p' data.yaml
} > data_final.yaml
log "training $MODEL on $(wc -l < $TRAIN) images, ${IMGSZ}px, $EPOCHS epochs"

cd ~/JunctionPerception/training
$P -m yolo3d.train --data $Y/data_final.yaml --model $MODEL --out $R --name final \
   --epochs $EPOCHS --imgsz $IMGSZ --batch $BATCH --workers 12 > $R/final.log 2>&1
log "training finished"

# One-time evaluation on the held-out test split, per source (2D keypoint metrics and 3D metrics).
$P - <<EOF > $R/final_test.txt 2>&1
from ultralytics import YOLO
m = YOLO("$R/final/weights/best.pt")
r = m.val(data="$Y/data_final.yaml", split="test", imgsz=$IMGSZ, batch=$BATCH, plots=True, project="$R", name="final_test")
print(r.results_dict)
EOF
for s in $(python3 -c "import json; print(' '.join(sorted({json.loads(l)['source'] for l in open('$J/unified_v2/test.jsonl')})))"); do
  grep "\"source\": \"$s\"" $J/unified_v2/test.jsonl > /tmp/test_$s.jsonl
  $P -m yolo3d.predict --weights $R/final/weights/best.pt --data /tmp/test_$s.jsonl --images / --out $R/final_pred_$s.jsonl --imgsz $IMGSZ
  ~/miniconda3/envs/jp/bin/python -m tools.eval_predictions --gt /tmp/test_$s.jsonl --pred $R/final_pred_$s.jsonl --out $R/final_3d_$s.json > $R/final_3d_$s.txt 2>&1
done
log "FINAL_DONE"
