# Deployment

## Edge stack (x86 + NVIDIA GPU)

```bash
git clone git@github.com:moavazpour/JunctionPerception.git && cd JunctionPerception
mkdir -p models data/state data/videos
# models/jp_y26s_960.onnx             <- python -m yolo3d.export --weights best.pt --imgsz 960 --out ...
# models/segformer_b2_cityscapes.onnx <- training/export_segformer.py
JP_MODELS=$PWD/models JP_DATA=$PWD/data docker compose up -d --build
```

After a training run, `tools/deploy_model.sh` does the whole hand-over on the server: ONNX export, runtime
image build (with unit tests), the TensorRT FP16 benchmark through the C++ detector and lifting on the
held-out test split (per source), the TUMTraf S110 auto-calibration check against the dev-kit ground
truth, and an end-to-end run of the test-site video. Reports land in `runs/deploy_<model>/`.

The stack starts three services:

| Service | Purpose |
|---|---|
| `mosquitto` | MQTT broker on port 1883 (WebSocket listener internal) |
| `perception-cam1` | the runtime. On first start it builds the TensorRT engines (cached next to the ONNX files) and auto-calibrates (≈45 s of traffic) |
| `web` | the panel at `http://<host>:8080`. It reaches MQTT through `/mqtt` |

Add a camera by duplicating the `perception-cam1` service with its own site YAML (unique `camera.id`).

## A new site

1. Mount the camera and note its GPS position, its approximate height and the rough direction it looks.
2. Derive the map priors:
   ```bash
   python3 tools/osm_site.py --lat <lat> --lon <lon> --bearing <deg>
   ```
   Paste the output into the site YAML: road bearing, stop-line reference and crosswalks.
3. Start the service. The `health` topic reports `auto-calibration in progress n%` until calibration finishes.
4. Check the result in the panel's **Calibration** tab. Range rings and learned lanes are drawn over a
   pixelated snapshot.
   If auto-calibration fails or looks wrong, use **Manual calibration** in the same tab: click a point on
   the road surface in the snapshot (lane-marking corners, stop-line ends, kerb corners), then the same
   point on the aerial map; repeat for at least 4 (better 6+) points spread from near to far and across
   the road, then **Solve**. The device solves focal length, pose and the exact camera position, reports
   the reprojection error and keeps the result. **Restart auto-calibration** discards it again.
5. Lanes are learned from traffic over the first minutes. Stop lines come from the configured reference
   point and are refined from observed queues. A hand-made `map.file` overrides learning.

## Jetson Orin

```bash
docker build -f docker/Dockerfile.jetson -t junction-perception:jetson .
```

Use the same compose file with `image: junction-perception:jetson`. Engines are rebuilt on the device on
first start. Prefer a GStreamer `nvv4l2decoder` source for hardware decoding (see the end of
`docker/Dockerfile.jetson`).

## Security checklist for production

- Enable MQTT authentication (`password_file`) and TLS listeners in `docker/mosquitto.conf`.
  Set `output.mqtt.username/password`.
- Put the web panel behind TLS and authentication (reverse proxy).
- Keep `calibration.publish_snapshot: false` unless an installer needs the calibration view.
- `output.file` records metadata only. Rotate it, or disable it when not needed.
