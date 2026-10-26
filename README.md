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
  geo-registration, and a manual fallback in the web panel (click points in the camera view and on an
  aerial map).
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
- Privacy by design: frames never leave process memory and only metadata is published.
- Outputs:
  - MQTT JSON
  - ETSI CPM content
  - presence topics per virtual detector for signal controllers
  - a live Three.js 3D web panel

**Stack.** C++20 runtime with TensorRT 10 and OpenCV, and a paho MQTT client; it ships as Docker
images for x86/dGPU and Jetson Orin. Models are trained in PyTorch and exported to ONNX. The web panel
uses TypeScript, Three.js and Vite.

| Document | Content |
|---|---|
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
web/        Three.js panel
tools/      site setup from OpenStreetMap, training and post-training deployment scripts
configs/    site configurations
docker/     Dockerfiles (x86, Jetson, web), broker and proxy config
```

## Quick start

```bash
cmake -S . -B build -G Ninja && cmake --build build && ctest --test-dir build   # CPU-only: -DJP_WITH_TENSORRT=OFF
JP_MODELS=$PWD/models JP_DATA=$PWD/data docker compose up -d --build              # full stack, panel on :8080
```
