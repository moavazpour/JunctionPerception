# JunctionPerception — Project Plan

Camera-only 3D perception for road intersections with automatic calibration, self-monitoring
and standard outputs, designed for edge deployment.

## Fixed decisions

| Topic | Decision |
|---|---|
| Core language | C++20 everywhere (host and CUDA code) |
| Model training | Python / PyTorch, offline only; export ONNX → TensorRT engine |
| 3D detection | **YOLO26-pose (m or s)** predicting 9 projected 3D-box keypoints + class for every road user; 3D by lifting the footprint onto the calibrated ground plane |
| Dataset | **JP-Unified**: all sources merged into one label format (see [DATASET.md](DATASET.md)), split 80/10/10 by sequence/scene, test fully held out |
| Classes | car, bus, truck, trailer, pedestrian, bicycle (with rider), e-scooter (with rider), motorcycle |
| Deployment | Docker; target edge = NVIDIA Jetson Orin; development on RTX 3080 server |
| Privacy | No frame is ever stored or transmitted; outputs are metadata only (a pixelated snapshot in calibration mode only) |
| Outputs | MQTT JSON, ETSI CPM content, virtual detectors for signal controllers |
| Web panel | TypeScript + Three.js, Tesla-style visualization |

## What is trained, and where

| Model | Trained? | Notes |
|---|---|---|
| YOLO26-pose 3D-keypoint detector | **yes**, the only training in the project | JP-Unified, on the RTX 3080 server (Colab if larger variants do not fit) |
| SegFormer-B2 (Cityscapes) | no, pretrained | used only for static occlusion and visibility; fine-tuning on Mapillary only if needed |
| GroundCenter3D (first prototype) | already trained | kept as a benchmark baseline, not deployed |
| Calibration, tracking, scene, safety, health | no training | algorithmic |

## Status

| Phase | Content | Status |
|---|---|---|
| 0 | Repo, CMake, Docker (x86 + Jetson), CI, server setup | ✅ done |
| 1 | C++ runtime: ingest (file/RTSP/USB/GStreamer), TensorRT, lifting, CTRA tracking | ✅ done, 17 unit tests |
| 2 | Scene model: lane learning, stop line, ETA with acceleration, queues, virtual detectors | ✅ done |
| 3 | Near-miss: TTC, MTTC, PET, DRAC, hard braking | ✅ done; false positives under bad calibration, see 6 |
| 4 | Self-monitoring: shake/drift compensation, visibility/weather, calibration consistency, occlusion, integrity | ✅ done |
| 5 | Outputs (MQTT, CPM, detectors) + docker compose + web panel | ✅ done, running on the server |
| 6 | Auto-calibration | ⚠️ implemented; on real intersection footage the hard vanishing-point constraint fails (turning traffic). Redesign: 4-DoF search with box consistency as the main term and vanishing points as soft terms, validated against TUMTraf ground truth |
| 7 | **Data**: JP-Unified spec, converters for every source, CARLA generation, QC, splits | 🔄 in progress |
| 8 | **Training**: YOLO26-pose-s and -m on JP-Unified, all useful augmentations | ⏳ after the datasets are downloaded |
| 9 | Benchmark: JP-Unified test split per source, TUMTraf official val, cross-dataset (zero-shot camera) tests, the deployed TensorRT engine through the C++ path | ⏳ |
| 10 | Test-site run (Georg-Brauchle-Ring video): calibration, tracks, ETA, events, panel | ⏳ after 6 and 8 |
| 11 | Edge: INT8, Jetson image validation, long-run test | ⏳ |

## Phase 7 — data (current)

1. Define JP-Unified: classes, 9 amodal cuboid keypoints, ground plane per image, ignore regions → [DATASET.md](DATASET.md).
2. Converters, one per source → JP-Unified jsonl:
   - TUMTraf R02 (Intersection) and R04 (V2X) ✅
   - TUMTraf A9 Highway R00/R01
   - KITTI 3D
   - Rope3D
   - DAIR-V2X-I
   - LUMPI
   - CARLA ✅ generator
3. CARLA synthetic data:
   - 4 towns, 240 random mountings × 25 frames, weather presets and fog
   - more towns and night-heavy runs as needed
4. QC per source: overlays, statistics, automatic checks.
5. Merge, deduplicate, assign 80/10/10 splits by group, export the YOLO pose dataset (`data.yaml`, `kpt_shape [9,3]`, `flip_idx`).

## Phase 8 — training

| Item | Setting |
|---|---|
| Model | YOLO26-pose-s (edge) and -m (accuracy), COCO-pose pretrained |
| Input | imgsz 1280 |
| Schedule | ~100 epochs, cosine LR, close mosaic for the last 10 |
| Augmentation | camera-consistent geometric (scale/crop, mirror with flip_idx, mosaic), photometric (HSV, blur, JPEG), offline weather copies (fog, rain, night, glare, droplets), CARLA weather |
| Excluded | rotation/shear/perspective (they break cuboid geometry) |
| Selection | val split only |
| Final number | the test split, evaluated once |

## Performance targets

- Jetson Orin, 1–4 cameras per device
- 15 Hz per camera
- < 100 ms from frame to MQTT
- Position error < 0.5 m up to 50 m and < 1.5 m up to 120 m
- Speed error < 1 m/s
- ETA error < 0.5 s within the last 5 s before the stop line

## Working rules

- Code is written locally, committed and pushed often, then pulled and deployed on the server.
- Videos, datasets, model weights and any image data never enter git.
- Every phase ends with a measurable deliverable.
