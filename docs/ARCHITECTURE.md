# Architecture

```
            ┌──────────────────────── junction_perception (one process per camera, C++20) ───────────────────────┐
 camera ──► │ VideoSource ─► Letterbox(undistort+resize) ─► Detector3D (TensorRT) ─► CameraModel.lift ─► Tracker │
 (any)      │      │                                              │                      ▲              (CTRA EKF) │
            │      │                                    calibration-free               │                    │    │
            │      │                                    image-space output      StabilityMonitor             ▼    │
            │      │                                              │             (per-frame rotation)  IntersectionModel
            │      │                                              ▼                                   lanes, stop line,│
            │      │                                       AutoCalibrator ──► CameraModel             ETA, queues,    │
            │      │                                                                                  detectors       │
            │      ├─► Segmenter (TensorRT, 0.5 Hz) ─► OcclusionMap ◄── tracks                            │           │
            │      └─► VisibilityMonitor                     │                                NearMissDetector        │
            │                    CalibrationMonitor ◄── detections, users                    (TTC, MTTC, PET, DRAC)   │
            │                            └──────────► assessIntegrity ──► health                       │              │
            └──────────────────────────────────────────────────────────────────────────── Sink (MQTT / file) ─────────┘
                                                                                                    │
                                                                    Mosquitto ◄─────────────────────┘ ──► web panel (Three.js)
                                                                                                         controllers, V2X, cloud
```

## Design principles

- **Calibration-free network, calibrated geometry.** The detector predicts only image-space
  quantities: the projected bottom centre, the 2D box, dimensions, height above ground and the
  ground-relative observation angle. All 3D comes from intersecting rays with the ground plane, so one
  engine serves every camera. It also means the network output can drive the calibration itself.
- **Every estimate carries its uncertainty.** Each lifted detection carries a position covariance
  obtained by propagating pixel noise through the ray–ground intersection; far objects get larger
  covariances. The tracker gates associations with these covariances and reports σ for position,
  speed and acceleration. ETA uncertainty is propagated from them.
- **The system checks itself continuously.** Independent monitors cover the camera pose, the image
  quality, the calibration consistency and the coverage. They are fused into one integrity level and a
  confidence value that consumers can act on. The calibration heals itself: small drifts are
  compensated per frame and absorbed, and a persistent inconsistency triggers a new auto-calibration.
- **Privacy by design.** Frames live only in process memory. Outputs are metadata. The optional
  calibration snapshot is downscaled and every road user in it is pixelated on the device.
- **Edge first.** C++20 runtime and TensorRT FP16 (INT8-ready). Capture runs in its own thread and
  processing always uses the newest frame. GPU work is limited to the detector at frame rate and the
  segmenter at 0.5 Hz.

## Modules (`core/`)

| Module | Responsibility |
|---|---|
| `geometry/camera` | pinhole and ground-plane model, lifting with covariance, box projection, ENU/WGS84 conversion |
| `geometry/bev` | oriented footprints (SAT overlap, distances), polylines, polygons |
| `infer` | TensorRT 10 engine (built from ONNX and cached per GPU), letterbox and undistortion, detector, segmenter |
| `calib` | `AutoCalibrator`: keypoint self-calibration (box re-projection, size priors, speed consistency) or the vanishing-point path for box-only models; map-based bearing. `calibrateFromCorrespondences`: manual calibration from map points |
| `tracking` | CTRA EKF, two-stage ByteTrack-style association with Mahalanobis gating, Hungarian assignment |
| `scene` | site map, lane learner (lanes and stop lines from traffic), intersection model (lane, s, ETA, queues, zones) |
| `safety` | near-miss detection: TTC/MTTC on oriented footprints, PET occupancy grid, DRAC, hard braking |
| `health` | stability (shake / drift / moved / lost), visibility (weather, lens, frozen, detection horizon), calibration consistency, occlusion map, integrity fusion |
| `output` | message schema, ETSI CPM content, MQTT (paho async) and file sinks |
| `pipeline` | orchestration, rates, self-healing |

## 3D detector (`training/yolo3d`)

YOLO26-pose (s at 960 px for the edge, m when accuracy matters more) predicts, for every road user, a
class, a 2D box and 9 keypoints: the projected bottom centre and the 8 corners of its amodal 3D box
(bottom FL FR RR RL, top FL FR RR RL; `flip_idx` swaps left and right for mirroring). The network stays
calibration-free; `CameraModel::liftKeypoints` intersects the footprint corners with the calibrated
ground plane (position, heading, length, width) and finds the height from the top corners, with class
priors as the fallback for missing corners.

The export is NMS-free end-to-end, so TensorRT emits a flat `[K, 6 + 27]` table (box, score, class,
keypoints with confidence). The older `GroundCenter3D` CenterNet-style model (`[K, 13]`) is still
readable by the runtime and kept as a baseline.

Training data is JP-Unified (see [DATASET.md](DATASET.md)): every source converted to one label format,
merged without ignore regions, split 80/10/10 by sequence. Augmentation keeps the cuboid geometry exact:
scale/translate, mirror with `flip_idx`, mosaic, HSV, and offline weather copies (fog, rain, night,
glare, lens droplets, blur, JPEG). Rotation, shear and perspective warps are off.

## Auto-calibration

1. Collect about 45 s of network output and build image-space tracks of the vehicles.
2. With a keypoint detector (the deployed model), solve focal length, pitch and roll directly
   (4 degrees of freedom with the height):
   - For a candidate camera at unit height, lift each car's footprint corners to the ground, take the
     height from the top corners, and fit a box (centre, heading, length, width, height) to all 9
     keypoints by Levenberg–Marquardt in pixels. A wrong camera bends footprints and verticals, which
     appears as re-projection error; working in pixels weights near and far cars by what the image
     actually resolves.
   - Lifted sizes scale with the camera height, so the height follows in closed form from the size
     ratios against the class prior, combined with the installer's height prior.
   - Cost: re-projection error + spread of the per-car scale + speed change between the near and far
     halves of each track + roll and height priors. Coarse grid, then Nelder–Mead from the best seeds.
   Without keypoints, the older path applies: road vanishing point from straight tracks (RANSAC), then
   focal length, rotation about the road axis and height from 2D box consistency and speed.
3. Rotate the ground frame so the dominant direction of travel matches the road bearing from
   OpenStreetMap. The installer's rough viewing direction resolves the ±180° ambiguity. The frame is
   anchored at the camera's GPS position.

The unit tests recover synthetic cameras (footbridge phone, gantry camera, wide-angle low mount, 1–2 px
keypoint noise, varying car sizes) to within a few percent in focal length and height and well under a
degree in pitch and roll.

### Manual fallback

When auto-calibration fails or an installer wants certainty, the web panel's Calibration tab takes
point pairs: a point on the road surface in the pixelated snapshot and the same point on an aerial map.
With 4 or more pairs the runtime estimates the focal length from the ground homography, refines focal
length and pose by minimising the reprojection error (IPPE + iterative PnP over a focal search), and
solves the camera position as well (the configured GPS position only defines the local frame). The
result, with its reprojection error, replaces the calibration and is persisted.
