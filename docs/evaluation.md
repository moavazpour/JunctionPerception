# Evaluation

## How we checked

- **Data:** TUMTraf Intersection R02, south1 camera on s01 and south2 on s03 (1 315 frames, day).
- **System as deployed:** the console's own calibration (points picked on the map, camera pose
  estimated), its detection areas, the COCO-pretrained YOLO26 detector (not trained on TUMTraf).
- **Ground truth:** the LiDAR 3D labels, moved into the site's map frame by a rigid transform fitted
  to the control points (0.85 m RMS; TUMTraf publishes no geographic position).
- **Compared:** every road user the twin shows with a heading, in every frame, against the labels
  of that frame. Road users without a heading are left out.
- **Correct:** within 2 m of a label (1 m for pedestrians and two-wheelers).

## Result

| Distance from camera | Objects | Within 2 m | Median error |
|---|---|---|---|
| 0–40 m | 3 248 | 85 % | 0.54 m |
| 40–60 m | 3 638 | 25 % (50 % more 2–5 m off) | 0.57 m |
| 60–80 m | 242 | 9 % | 1.68 m |
| all | 7 128 | 52 % (62 % where labels exist) | 0.55 m (P90 1.13 m) |

- Median heading error 3.8°.
- 17 % of the twin's road users have no label nearby. The 16 largest of these tracks, checked on the
  video, are all real road users the dataset did not label; no false detection was found.
- Beyond 40 m the error comes from the calibration: south2's height is estimated 0.9 m too high.

## Limits

Two day sequences, nearly all objects from south2; the 0.85 m label alignment is part of every
error.

## Reproduce

Run each camera with `jp_run --scene <site camera> --regions <its regions.json>`, move the labels
with `tools/tumtraf_site_frame.py`, then:

```bash
jp_eval --run <name>,day,<world.jsonl>,<sequence>,<eval scene> --labels merged --merge-trailers --out <dir>
```

`<dir>/objects.csv` lists every compared road user with its label and errors. Earlier
measurements (dataset calibration, S1–S4, fusion) are in the git history of this file.
