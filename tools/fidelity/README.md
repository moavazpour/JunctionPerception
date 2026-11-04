# 3D fidelity harness

Measures how far the 3D twin draws road users from the backend world state and from ground
truth (report: `docs/3d-fidelity.md`). It drives the twin page's own `TwinView` with recorded
world states on a simulated clock; no source file of the twin is changed.

1. Record world states and labels of one sequence with the same run, per camera:

   ```bash
   scripts/in_docker.sh build/release/src/apps/jp_run \
       --source /data/a9_dataset_r02_s03_s110_camera_basler_south2_8mm.mp4 \
       --scene scenes/tumtraf_s110_south2 --bias-correction box_fit \
       --config <pipeline config> --reference /data/a9_dataset_r02_s03 \
       --reference-csv outputs/fidelity/south2_gt.csv \
       --world-jsonl outputs/fidelity/south2_world.jsonl
   ```

   (and the same for south1).
2. Build the bundle (one frame, capture-time clock, objects matched to labels):

   ```bash
   scripts/in_docker.sh python3 tools/fidelity/prepare_bundle.py outputs/fidelity \
       /data/a9_dataset_r02_s03/images scenes
   ```

3. Serve `bundle.json`, `osm.json` (the site's OSM layer) and `harness.js` on the same origin as
   the twin page, open the twin with `?bench=1`, set `window.FIDELITY_BASE` to their folder and run
   `(await import(<harness.js>)).run()`; `results.records` holds every measured frame.
