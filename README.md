# JunctionPerception

Camera-only 3D perception for road intersections, built for edge deployment. Plug in a camera and give
its GPS position and approximate height. The system calibrates itself from traffic, tracks every road
user in 3D and publishes standard metadata. It also continuously reports how far its own output can
be trusted.

**What it does**

- Monocular 3D detection of cars, buses, trucks, pedestrians, cyclists and motorcyclists (e-scooter
  riders once data exists), from any viewing angle. A YOLO26-pose network predicts the 9 projected
  corners of every road user's 3D box; the box is lifted onto the calibrated ground plane. It is trained
  on JP-Unified, a merge of TUMTraf, A9, LUMPI, KITTI and DAIR-V2X with one label format.
- Automatic calibration from traffic: focal length, pitch, roll and height from the predicted car boxes
  (re-projection of fitted boxes, size priors, speed consistency), OpenStreetMap-aligned
  geo-registration. The deployed path is manual calibration in the web panel: point pairs and traced
  lane lines between the camera image and the aerial map, with height and lens priors, per-point errors,
  a visual check (lanes and a 5 m grid drawn into the image) and a traffic check that proposes small
  corrections.
- 3D tracking with a CTRA Kalman filter: speed, acceleration and turn rate, each with uncertainty.
- Intersection model:
  - lanes and stop lines (learned from traffic or configured)
  - time to stop line with acceleration
  - queues and virtual loop detectors
- Near-miss detection with surrogate safety measures (TTC, MTTC, PET, DRAC, hard braking).
- Self-monitoring:
  - camera shake and drift (compensated per frame)
  - calibration consistency
  - weather and visibility (fog, night, glare, blur, lens obstruction, frozen stream) and effective detection range
  - static and dynamic occlusion of every lane
  - a fused integrity level and confidence
- Privacy by design: only metadata is published. The live camera view for the panel (MJPEG) can be
  switched off (`preview.enabled: false`); the calibration snapshot pixelates road users.
- Outputs:
  - MQTT JSON
  - ETSI CPM content
  - presence topics per virtual detector for signal controllers
  - a web panel (React + Three.js): live 3D twin on an OpenStreetMap-drawn road map, live camera view,
    per-object class, speed and acceleration, statistics, light and dark theme, settings and calibration

## Current focus and limitations

The current focus is the digital twin of one intersection: **TUMTraf S110** (Garching, recording R02 s03,
cameras south1 and south2).

- **Road users come from the dataset's labels, not from a detector.** None of our detectors detects
  reliably yet. The twin places every road user at its labelled position, frame by frame with the video
  (`groundtruth.twin`, the default for replayed dataset sites; toggle in Settings, Detection). The
  toolbar shows a "dataset labels" badge meanwhile.
- **A YOLO detector is in training** to detect road users reliably. Once it does, its output replaces the
  dataset labels as the twin's source; the rest of the twin (map, tracks, lanes, statistics) stays as it is.
- **Calibration is manual**, done in the web panel (point pairs and lane lines between the camera image and
  the aerial map). The dataset's own calibration is used only as a reference.
- **Cameras play in lock-step.** Both videos start together on one shared clock and the twin follows one
  camera's frame index, so all views and the twin show the same moment (the two recordings agree within
  ±0.3 s per frame).
- Accuracy against ground truth and the perception's safety events are not shown while the twin draws
  from the labels.

**Stack.** C++20 runtime with TensorRT 10 (GPU) or ONNX Runtime (CPU) and OpenCV, and a paho MQTT client;
it ships as Docker images for x86/dGPU, CPU-only hosts and Jetson Orin. Models are trained in PyTorch and
exported to ONNX. The web panel uses React, TypeScript, Tailwind, Three.js, Leaflet and Vite.

| Document | Content |
|---|---|
| [docs/USER_GUIDE.md](docs/USER_GUIDE.md) | using the web demo: add a site, calibrate, the digital twin |
| [docs/CODE_ARCHITECTURE.md](docs/CODE_ARCHITECTURE.md) | complete code architecture: modules, files, data flow, threads, training and deployment |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | design and algorithms |
| [docs/INTERFACES.md](docs/INTERFACES.md) | inputs, MQTT topics and message schema |
| [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md) | running the stack, new sites, Jetson |
| [docs/BENCHMARK.md](docs/BENCHMARK.md) | test-split results of the deployed model (written after training) |
| [docs/PLAN.md](docs/PLAN.md) | project plan |

## Repository

```
core/       C++20 library: geometry, inference, calibration, tracking, scene, safety, health, output
apps/       junction_perception (runtime), jp_bench (benchmark runner)
tests/      GoogleTest unit tests (geometry, tracking, safety, auto-calibration)
training/   jpdata (one converter per dataset, merge), yolo3d (YOLO export, training, ONNX export), evaluation
web/        React + Three.js panel
tools/      site setup from OpenStreetMap, training and post-training deployment scripts
configs/    site configurations
docker/     Dockerfiles (x86, Jetson, web), broker and proxy config
```

## Quick start

```bash
cmake -S . -B build -G Ninja && cmake --build build && ctest --test-dir build   # CPU-only: -DJP_WITH_TENSORRT=OFF
JP_MODELS=$PWD/models JP_DATA=$PWD/data docker compose up -d --build              # one configured camera, panel on :8080
JP_MODELS=$PWD/models JP_DATA=$PWD/demo docker compose -f docker-compose.demo.yml up -d --build   # web demo: sites from the browser
```
