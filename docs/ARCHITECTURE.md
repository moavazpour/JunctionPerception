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
| `calib` | `AutoCalibrator`: vanishing point from trajectories, then grid search and Nelder–Mead over focal length, rotation about the road axis and height; map-based bearing |
| `tracking` | CTRA EKF, two-stage ByteTrack-style association with Mahalanobis gating, Hungarian assignment |
| `scene` | site map, lane learner (lanes and stop lines from traffic), intersection model (lane, s, ETA, queues, zones) |
| `safety` | near-miss detection: TTC/MTTC on oriented footprints, PET occupancy grid, DRAC, hard braking |
| `health` | stability (shake / drift / moved / lost), visibility (weather, lens, frozen, detection horizon), calibration consistency, occlusion map, integrity fusion |
| `output` | message schema, ETSI CPM content, MQTT (paho async) and file sinks |
| `pipeline` | orchestration, rates, self-healing |

## 3D detector (`training/`)

`GroundCenter3D` is a DLA-34 backbone with an FPN neck at stride 4 and CenterNet-style heads:

| Head | Output |
|---|---|
| heat-map | 8 classes |
| offset | sub-pixel offset of the keypoint |
| 2D box | edges relative to the keypoint |
| dz | height above ground |
| dimensions | log-dimensions relative to class priors |
| orientation | 12-bin observation angle plus a residual |

The deploy wrapper adds sigmoid, max-pool NMS, top-K and decoding inside the graph, so TensorRT emits
a flat `[K, 13]` table.

Training-time camera augmentation is what lets one model serve any camera:

1. Sample a virtual camera with a new focal length and principal point, a small rotation and a mirror.
2. Warp the image with the induced homography.
3. Transform the 3D labels with the same rotation.

Labels therefore stay exact while the model sees many mountings. Photometric and weather augmentation
(fog, rain, night, glare, lens droplets, blur, JPEG) target robustness in adverse conditions.

## Auto-calibration

1. Collect about 45 s of network output and build image-space tracks of the vehicle keypoints.
2. Fit lines to the straight tracks and find the road vanishing point (RANSAC plus least squares). The
   vanishing direction must lie in the ground plane, which leaves three unknowns:
   - focal length
   - rotation about the road axis (≈ roll)
   - camera height
3. Minimise the robust cost below. Grid search first, then Nelder–Mead.
   - Re-projected lifted 3D boxes against the network 2D boxes. This fixes the scale (height/focal) and the pitch.
   - Speed change between the near and far halves of each track. Perspective errors make speeds vary with range.
   - Priors on height (installer) and roll.
4. Rotate the ground frame so the vanishing direction matches the road bearing from OpenStreetMap.
   The installer's rough viewing direction resolves the ±180° ambiguity. The frame is anchored at the
   camera's GPS position.

The unit test `AutoCalibration.RecoversCameraFromTraffic` recovers a synthetic camera to within a
few percent of focal length, under 0.6° in pitch and roll, and under 0.6 m in height.
