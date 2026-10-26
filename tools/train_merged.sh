#!/bin/bash
# Final YOLO26-pose training on JP-Unified (merged/):
#  1. exports merged/{train,val,test}.jsonl to an Ultralytics pose dataset (images untouched, 25 % weather copies)
#  2. trains at 1280 px, validating on val
#  3. evaluates the best checkpoint once on the held-out test split (2D keypoints, then 3D per source)
set -eu
J=/home/mo_avazpour/data/junction_perception
M=$J/merged
Y=$M/yolo
R=$J/runs
P=~/miniconda3/envs/yolo/bin/python
PJ=~/miniconda3/envs/jp/bin/python
MODEL=${MODEL:-yolo26s-pose.pt}
NAME=${NAME:-jp_unified_y26s}
EPOCHS=${EPOCHS:-60}
IMGSZ=${IMGSZ:-1280}
BATCH=${BATCH:-16}
log() { echo "$(date '+%F %T') $*"; }

cd ~/JunctionPerception/training
if [ ! -f $Y/data.yaml ]; then
  rm -rf $Y
  log "exporting YOLO dataset"
  $PJ -m yolo3d.convert --unified $M --out $Y --mode clean --weather 0.25
fi
log "training $MODEL, ${IMGSZ}px, $EPOCHS epochs, batch $BATCH"
$P -m yolo3d.train --data $Y/data.yaml --model $MODEL --out $R --name $NAME \
   --epochs $EPOCHS --imgsz $IMGSZ --batch $BATCH --workers 12 > $R/$NAME.log 2>&1
log "training finished"

$P - <<EOF > $R/${NAME}_test.txt 2>&1
from ultralytics import YOLO
m = YOLO("$R/$NAME/weights/best.pt")
r = m.val(data="$Y/data.yaml", split="test", imgsz=$IMGSZ, batch=$BATCH, plots=True, project="$R", name="${NAME}_test")
print(r.results_dict)
EOF
for s in $(python3 -c "import json; print(' '.join(sorted({json.loads(l)['source'] for l in open('$M/test.jsonl')})))"); do
  grep "\"source\": \"$s\"" $M/test.jsonl > /tmp/test_$s.jsonl
  $P -m yolo3d.predict --weights $R/$NAME/weights/best.pt --data /tmp/test_$s.jsonl --images / --out $R/${NAME}_pred_$s.jsonl --imgsz $IMGSZ
  $PJ -m tools.eval_predictions --gt /tmp/test_$s.jsonl --pred $R/${NAME}_pred_$s.jsonl --out $R/${NAME}_3d_$s.json > $R/${NAME}_3d_$s.txt 2>&1
done
log "FINAL_DONE"
