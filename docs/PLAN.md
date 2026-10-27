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
| 6 | Calibration | ✅ manual calibration is the deployed path: point pairs, traced lines and verticals with height and lens priors (MAP, robust), per-input residuals, expected accuracy, traffic check that proposes small corrections; keypoint self-calibration validated on synthetic cameras, not yet reliable on real footage (focal–pitch–height ambiguity; next: vanishing-point terms) |
| 7 | **Data**: JP-Unified spec, one converter per source, cleaning, merge | ✅ done (35k train / 4.5k val / 4.9k test images, see DATASET.md) |
| 8 | **Training**: YOLO26-pose-s at 960 px on JP-Unified | ✅ v1 (box mAP50 0.824, mAP50-95 0.606, pose mAP50-95 0.432); v2 fine-tune on heading-repaired DAIR labels deployed (front/back flips 13.8 % → 9.8 %, pose mAP50-95 0.402 → 0.431, box mAP50-95 0.606 → 0.589 on test); v1 kept in models/v1 |
| 9 | Benchmark: JP-Unified test split per source, TUMTraf official val, cross-dataset (zero-shot camera) tests, the deployed TensorRT engine through the C++ path | ✅ per-source TensorRT benchmark (LUMPI ATE 0.46 m, TUMTraf-I 0.57 m) |
| 10 | Test-site run (Georg-Brauchle-Ring video): calibration, tracks, ETA, events, panel | 🔄 demo site running (looped video); manual calibration of the site pending |
| 11 | Edge: INT8, Jetson image validation, long-run test | ⏳ |
| 12 | Product demo: React panel (live twin, camera view, statistics, object cards, light/dark), settings (source, detection thresholds live, compute, sites, system), calibration wizard with drawn lanes, ONNX Runtime CPU backend | ✅ done; plus GPU inference with ONNX Runtime FP32 (TensorRT optional), model/input-size/backend switching live, road-area crop, motion stabilisation, traffic light states |

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
