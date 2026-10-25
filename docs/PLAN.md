# JunctionPerception — Project Plan

Camera-only 3D perception for road intersections with automatic calibration, self-monitoring
and standard outputs, designed for edge deployment.

## Fixed decisions

| Topic | Decision |
|---|---|
| Core language | C++20 everywhere (host and CUDA code) |
| Model training | Python / PyTorch, offline only; export ONNX → TensorRT engine |
| 3D detection | Learned monocular 3D detector for roadside cameras (ground-aware, see below) |
| Classes | car, bus, truck, trailer, pedestrian, bicycle (with rider), e-scooter (with rider), motorcycle |
| Deployment | Docker; target edge = NVIDIA Jetson Orin; development on RTX 3080 server |
| Privacy | No frame is ever stored or transmitted; outputs are metadata only. A fully blurred snapshot is allowed in calibration mode only |
| Outputs | Universal adapter layer: MQTT JSON first, ETSI CPM-style JSON, virtual detectors for signal controllers |
| Web panel | TypeScript + Three.js, Tesla-style visualization |

## Infrastructure

- GPU server `mo_avazpour@192.168.100.2`: Ubuntu 22.04, RTX 3080 Laptop 16 GB (SM 8.6), 16 cores, 31 GB RAM, Docker with NVIDIA runtime.
- Server layout: code in `~/JunctionPerception`, data in `~/data/junction_perception/{videos,datasets,models}`.
- Server pulls from GitHub through SSH agent forwarding (`ssh -A`).
- Training runs on the server. Anything that does not fit in 16 GB moves to Colab.

## Data

- Test video `IMG_5538.MOV`: 1920×1080, 30 fps, 120 s. It was filmed handheld from the Walther-Bathe-Weg footbridge
  (48.175995, 11.555590) at roughly 6–8 m, looking east along Georg-Brauchle-Ring towards Lerchenauer Str.
  - The camera shakes, so the clip doubles as a shake-detection test.
  - The road is wet, so it doubles as a weather test.
- TUMTraf:
  - R2 is TUMTraf Intersection: 4 sequences, cameras south1/south2, LiDAR labels.
  - R4 is TUMTraf-V2X: 4 infrastructure cameras plus a vehicle camera, LiDAR labels in the `s110_lidar_ouster_south` frame.
  - Calibration comes from the official dev-kit.
- TUMTraf has no e-scooter class. The e-scooter class is supported in the schema but is excluded from the benchmark.
  Later sources for it: CARLA synthetic data and our own CVAT-labeled clips.
- Candidate additional datasets: TUMTraf A9 Highway (R0/R1), LUMPI, Rope3D, DAIR-V2X-I, Mapillary Vistas.

## Architecture

```
[any camera] → Ingest → Undistort/Resize → TensorRT (3D det + ground seg) → Lifting → Tracking → Scene → Adapters
  file/RTSP/      OpenCV + GStreamer          calibration-free network         ray ∩ ground  CTRA EKF  lanes, ETA,  MQTT / CPM /
  USB/GigE                                    outputs (image space)            plane         ByteTrack near-miss    virtual detectors
                      ↑                                                                                ↓
             Self-monitor: shake & drift, calibration consistency, weather/visibility, occlusion  →  health topic
                                                                                                       ↓
                                                                           Web panel (Three.js over MQTT/WebSocket)
```

### Why 3D boxes are required

A 2D box plus a homography only gives a single ground point. That point fails in six places:

1. **Large vehicles.** The bottom of the 2D box of an oblique bus or trailer is metres away from its true centre, which corrupts speed, ETA and distance to the stop line.
2. **Heading.** Lane assignment, turn detection and prediction need it.
3. **Dimensions.** Separating truck, trailer and bus needs them, and so does checking calibration consistency.
4. **Near-miss.** TTC and PET are defined on footprints, not points.
5. **Occlusion.** Ray casting needs object height and extent.
6. **Digital twin.** Rendering needs the correct pose and size.

### 3D detector: ground-aware monocular model

The network predicts only image-space, calibration-free quantities per object:

- class heat-map peak at the **projected bottom centre** of the 3D box
- sub-pixel offset
- 2D box
- residual height above ground
- log-dimensions relative to class priors
- **ground-relative observation angle**: the heading relative to the viewing ray, measured in the ground plane

Lifting to 3D intersects the pixel ray with the calibrated ground plane. This is the same insight as BEVHeight and
MonoUNI: regress height above ground, not depth. It makes the model independent of camera mounting height and pitch,
which is what "plug in any camera" requires. The only calibration the runtime needs is the intrinsics plus the ground
plane in camera coordinates, which is exactly what auto-calibration estimates. The network uses only standard ops
(conv, upsample, max-pool NMS, TopK), so it exports cleanly to TensorRT.

### Auto-calibration

Inputs are the camera geo-position, an approximate mounting height and the site map (OpenStreetMap extract).
The procedure:

1. **Road vanishing point.** Collect ~60 s of calibration-free detections. Estimate the dominant road vanishing point
   from vehicle trajectories and lane-marking segments.
2. **Solve the camera.** Solve focal length, pitch, roll and height by minimising the disagreement between lifted 3D
   boxes and network 2D boxes. Regularise with class-size priors, lane-width priors and the prior on mounting height.
3. **Register to the map.** Rotate the ground frame so the vanishing-point direction matches the OSM road bearing,
   anchored at the camera's geo-position.

### Self-monitoring

- **Shake / drift.** Track features in static background with LK flow against a reference frame. High-pass RMS means shake; low-pass offset means drift. Small rotations are compensated automatically.
- **Calibration consistency.** Watch 2D-vs-projected-3D box agreement, lifted object size distribution and track-heading alignment with lanes.
- **Weather / visibility.** Monitor sharpness, contrast, dark-channel haze, low light, glare and persistent local blur (lens obstruction). Also watch detection-score drift, track fragmentation and marking visibility. Output a visibility score and an effective range.
- **Occlusion.** Keep a BEV grid with static occlusion (from segmentation history) and dynamic occlusion (ray casting through current 3D boxes). Report occluded zones per lane and crosswalk.
- **Integrity state.** Overall OK / DEGRADED / FAULT with reasons, plus watchdog on latency and frame rate.

### Safety (near-miss)

- TTC and MTTC on oriented footprints
- PET on a timestamped occupancy grid
- DRAC
- hard braking (a < −3 m/s²)
- severity classes

## Performance targets

- Jetson Orin, 1–4 cameras per device
- 15 Hz per camera (minimum 10 Hz)
- < 100 ms from frame to MQTT
- Position error < 0.5 m up to 50 m and < 1.5 m up to 120 m
- Speed error < 1 m/s
- ETA error < 0.5 s within the last 5 s before the stop line

## Phases

| Phase | Content | Deliverable |
|---|---|---|
| 0 | Repo skeleton, CMake, Docker, CI, data on server | Green build in Docker on the server |
| 1 | TUMTraf conversion to a camera-frame training format | Verified label projections |
| 2 | Ground-aware mono-3D detector: training, ONNX, TensorRT | Trained engine + validation metrics |
| 3 | Ground segmentation + road markings | Segmentation engine |
| 4 | Auto-calibration + geo-registration | Calibration of the test site |
| 5 | C++ runtime: ingest, inference, lifting, tracking (speed, acceleration) | Tracks on the test video |
| 6 | Scene model: lanes, stop line, ETA, queues | Full road-user messages |
| 7 | Near-miss detection | Safety events |
| 8 | Self-monitoring | Health topic and alerts |
| 9 | Output adapters + docker compose | MQTT / CPM / virtual detectors |
| 10 | Three.js web panel | Live dashboard |
| 11 | TUMTraf benchmark of the deployed engine | Benchmark report |
| 12 | Jetson image, INT8, long-run test | Edge image |

## Working rules

- Code is written locally, committed and pushed often, then pulled and deployed on the server.
- Videos, datasets, model weights and any image data never enter git.
