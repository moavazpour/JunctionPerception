# JP-Unified: the project's 3D road-user dataset

One label format for every source, designed for a YOLO26-pose model that predicts the projected
3D box of each road user as keypoints. Every source dataset is converted into this format, checked
visually and statistically, and split with no leakage between train, val and test.

## 1. Classes

| id | class | includes | excludes |
|---|---|---|---|
| 0 | `car` | passenger cars, SUVs, vans, pick-ups, emergency cars/vans | — |
| 1 | `bus` | city, coach and articulated buses | trams (ignored) |
| 2 | `truck` | rigid trucks and tractor units (cab) | the semi-trailer behind a tractor |
| 3 | `trailer` | semi-trailers and towed trailers (separate box from the tractor) | — |
| 4 | `pedestrian` | walking or standing persons | seated persons (ignored) |
| 5 | `bicycle` | bicycle **with its rider** as one object | parked bicycles without rider (ignored when labelled separately) |
| 6 | `escooter` | e-scooter **with its rider** | — |
| 7 | `motorcycle` | motorcycles and mopeds with rider | — |

**Ignored** (not trained, regions masked in the image, see §4): tram, train, tricycle, wheelchair,
seated person, "other", "misc", "DontCare".

## 2. Geometry

For every object the converter stores the exact 3D box in the **camera frame** (OpenCV: x right,
y down, z forward) together with the camera intrinsics and the **ground plane** in the camera frame.

| Field | Meaning |
|---|---|
| `bottom` | centre of the box footprint (m) |
| `dims` | length (along heading), width, height (m) |
| `heading` | unit vector of the object's forward direction, lying in the ground plane |
| `K` | intrinsics of the stored, undistorted image |
| `plane` | `n·X + d = 0`, with `n` pointing up and `d` the camera height (m) |

### 2.1 Keypoints (9 per object, amodal)

| k | point |
|---|---|
| 0 | bottom centre (footprint centre) |
| 1 | bottom front-left |
| 2 | bottom front-right |
| 3 | bottom rear-right |
| 4 | bottom rear-left |
| 5 | top front-left |
| 6 | top front-right |
| 7 | top rear-right |
| 8 | top rear-left |

"Front" is the heading direction. Left and right are as seen from the object, facing forward.
Horizontal mirroring swaps left and right: `flip_idx = [0, 2, 1, 4, 3, 6, 5, 8, 7]`.

Keypoints are **amodal**: corners hidden by the object itself or by other objects are still labelled,
because the 3D lifting needs them. Visibility:

| v | Meaning |
|---|---|
| 2 | inside the image and in front of the camera |
| 0 | outside the image (not supervised) |

The 2D box is the extent of the projected cuboid, clipped to the image. For pedestrians the heading
is the walking or facing direction from the source label; corner-order errors there are expected
and acceptable.

### 2.2 Lifting at runtime

1. The footprint corners (k1–k4) are intersected with the ground plane, which gives position,
   heading, length and width without any size prior.
2. Height is recovered from the top corners (k5–k8).
3. If corners are missing, the bottom centre (k0) and class priors are used.

The same code exists in `training/yolo3d/lift.py` and `CameraModel::liftKeypoints` (C++).

## 3. Sample record (`*.jsonl`, one line per image)

```json
{"source": "tumtraf_i", "sequence": "r02_s03", "image": "<path>", "camera": "s110_camera_basler_south2_8mm",
 "width": 1920, "height": 1200, "K": [[...]], "plane": [nx, ny, nz, d],
 "ignore": [[x1, y1, x2, y2], ...],
 "objects": [{"cls": "car", "src_type": "VAN", "bottom": [x, y, z], "dims": [l, w, h], "heading": [hx, hy, hz],
              "box2d": [x1, y1, x2, y2], "occlusion": "...", "truncation": 0.0}]}
```

## 4. Source datasets and conversion rules

The table below is the label mapping per source. Raw label → JP-Unified class, `—` means ignored.

| Source | Views | Frame of raw 3D labels | Mapping |
|---|---|---|---|
| TUMTraf Intersection (R02) | 2 × gantry cameras, 8 mm | LiDAR (south/north), cuboid centre + quaternion | CAR/VAN/EMERGENCY_VEHICLE→car, BUS→bus, TRUCK→truck, TRAILER→trailer, PEDESTRIAN→pedestrian, BICYCLE→bicycle, MOTORCYCLE→motorcycle, OTHER→— |
| TUMTraf-V2X (R04) | infrastructure cameras | registered LiDAR frame | same as above. Only south2 and north are used, because the dev-kit projections of south1 and east do not align. Test labels are withheld, so the test frames are unused |
| TUMTraf A9 Highway (R00/R01) | gantry cameras 16/50 mm along the highway | R00: image-labelled 3D boxes, R01: LiDAR | same TUMTraf class names (R00 uses lowercase) |
| KITTI 3D object | vehicle camera at 1.65 m | camera frame, KITTI convention | Car/Van→car, Truck→truck, Pedestrian→pedestrian, Cyclist→bicycle, Person_sitting/Tram/Misc/DontCare→— |
| Rope3D | roadside poles, many heights and pitches | camera frame, KITTI-like, ground plane per image (`denorm`) | car/van→car, bus→bus, truck→truck, pedestrian→pedestrian, cyclist→bicycle, motorcyclist→motorcycle, tricyclist/barrow/cones→— |
| DAIR-V2X-I | roadside poles | camera frame, KITTI-like, `denorm` ground plane | as Rope3D |
| LUMPI | intersection, multiple cameras | world frame + per-camera extrinsics | car/van→car, bus→bus, truck→truck, person→pedestrian, bicycle→bicycle, motorcycle→motorcycle |
| CARLA (synthetic, generated here) | random mountings 4–12 m, pitch 3–35°, HFOV 20–90°, 19 weather presets + fog | exact actor boxes | blueprint `base_type`: car/van→car, truck→truck, bus→bus, motorcycle→motorcycle, bicycle→bicycle, walker→pedestrian |

### Convention differences handled by the converters

| Difference | Handling |
|---|---|
| Box centre vs bottom centre | KITTI-style `location` is already the bottom centre; TUMTraf/LUMPI/CARLA store the cuboid centre → drop by h/2 along the ground normal |
| Dimension order | KITTI `h w l` → `l w h`; TUMTraf `l w h`; Rope3D like KITTI |
| Heading | KITTI `rotation_y` about camera −y → heading `(cos ry, 0, −sin ry)` in camera frame; TUMTraf quaternion x-axis; CARLA forward vector. All re-projected into the ground plane |
| Ground plane | KITTI: fitted per image to object bottoms, falling back to camera height 1.65 m. Rope3D/DAIR: `denorm`. TUMTraf: plane fitted to all label bottoms per camera (median residual ≈ 0.15 m). CARLA: world up through object bottoms |
| Distortion | every image is checked: TUMTraf R02/R04 and KITTI are already rectified (verified with overlays). **DAIR-V2X / V2X-Seq images are raw with strong barrel distortion (k1 ≈ −0.45)**: they are undistorted with `cam_K`/`cam_D` into the rectified `P`. LUMPI and A9 R00/R01 are undistorted with their calibration when distortion coefficients are present. Undistorted copies live in `undist/<source>/` |
| Vehicle-side data | KITTI and the vehicle-side camera of V2X-Seq-SPD see the road from 1.5 m. They are kept for viewpoint coverage but capped at **10 % of the training images** |
| Image-only 3D labels | A9 R01 camera labels give the 8 projected cuboid corners without a 3D box. They are used directly as keypoints. The bottom centre is the intersection of the footprint diagonals (projectively exact), and left/right are ordered by the signed area of the footprint. These samples train the keypoint model but are excluded from 3D-metric evaluation |
| Duplicate objects | multi-LiDAR label sets (TUMTraf north/south) are merged by class and 1 m footprint distance |
| Truck + trailer | kept as two objects (TUMTraf convention). In sources labelling one articulated box, the object goes to `truck` |
| Ignore regions | DontCare/ignored-class boxes are painted grey (114) in the training copy of the image so they are neither positives nor false negatives |
| Unlabelled visible objects | CARLA parked-car props are removed from the map; tiny objects (< 8 px tall) are dropped from targets |

## 5. Splits (80 / 10 / 10)

- The split is made **per source** by **groups** that never cross splits: TUMTraf sequence plus 30 s
  time block, KITTI drive, Rope3D/DAIR camera plus scene, LUMPI measurement plus camera, CARLA camera
  placement.
- 80 % **train**, 10 % **val** (model selection, early stopping), 10 % **test**. Test never enters
  training or model selection and is evaluated once, at the end.
- Split assignment is deterministic: hash of the group id with a fixed seed. Group lists are stored
  in `splits/*.txt` for reproducibility.
- Class balance: rare classes (bicycle, motorcycle, bus, trailer) are oversampled in train only.

## 6. Augmentation (training)

| Kind | Augmentations |
|---|---|
| Geometric, camera-consistent | random scale/crop (equivalent to a new focal length / principal point), horizontal mirror with `flip_idx`, mosaic (first epochs) |
| Excluded | rotation, shear and perspective warps: they would teach impossible cuboids |
| Photometric | HSV, brightness and contrast, blur, JPEG |
| Weather (offline copies) | fog, rain streaks, night, glare, lens droplets (`jp3d/weather.py`) |
| Synthetic | CARLA weather presets and fog |

## 7. Quality control

Every converted source is checked before training:

- projected cuboid overlays rendered for random samples of every camera (`tools/visualize_labels.py`)
- statistics per class: dimensions, distances, heading histogram, object counts
- automatic checks:
  - keypoints inside the 2D box
  - footprint on the ground plane (|dz| < 0.3 m)
  - dimensions within class ranges

## 8. Current build (JP-Unified, final)

Each source is converted by its own converter into `converted/<source>/data.jsonl` (images untouched),
cleaned, then merged into `merged/{train,val,test}.jsonl`. No ignore regions, no detector-based
filtering and no size or ground-plane checks are applied: labels are used as the datasets provide them.

| Source | Images | Cleaning |
|---|---|---|
| TUMTraf Intersection (R02) | 4 257 | south-LiDAR labels only (the dev-kit north-LiDAR projections are offset) |
| TUMTraf-V2X (R04) | 1 800 | south2 + north cameras only (south1/east projections do not align); test labels withheld upstream |
| A9 Highway R00 | 299 | s01/s02 frames whose calibration reproduces the dataset's own projections; s00/R01 have no 3D boxes, s03/s04 no images |
| LUMPI | 6 867 | M1, M2, M4–M6 (M0 automatic labels only, M3 video missing); 836 black frames (camera c5) and camera c8 of M2 (labels drift in time against the video) removed; undistorted |
| KITTI | 7 481 (1 738 used) | vehicle-side cap |
| DAIR V2X-C infra | 8 577 | correct image package only; affine calibration applied exactly, keypoints exact, 3D approximate (non-metric) |
| DAIR V2X-Seq SPD infra | 11 275 | images without labels removed; as above |
| DAIR V2X-Seq SPD vehicle | 12 252 (2 767 used) | vehicle-side cap; non-metric |
| DAIR-V2X-I | 7 058 | as above |

Tractor + trailer pairs are merged into one truck box everywhere (DAIR, KITTI and LUMPI label them
that way); stand-alone trailers are trucks. Vehicle-side sources are capped at 10 % of the merged set.
CARLA is not used (it lowered pose mAP on real validation data in an ablation).

Splits (by group, per source): train 35 265 (+8 801 adverse-weather copies in the YOLO export),
val 4 457, test 4 916 images. Train objects: car 429k, pedestrian 90k, motorcycle 49k, bicycle 41k,
truck 24k, bus 11k; 29 % of the images are from German sites. Per-source counts are in
`merged/summary.json`.
