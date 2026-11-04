# Experiment: bird's-eye view (BEV) next to the 3D twin

Branch `feature/bev-experiment`, November 2026. The question: can a clean, metric 2D bird's-eye
view replace the 3D visualization of the twin, and is a BEV-centred architecture the better basis
for intersection perception and traffic analysis? The experiment adds a BEV mode next to the
unchanged 3D twin, measures both on the same world states and separates what the visualization
can change (rendering, readability, honesty about uncertainty) from what it cannot (perception
accuracy).

**Short answer.** The BEV is sufficient for every monitoring and traffic-analysis use case of the
project, cheaper to render (about half the frame cost, and it draws only when data arrives) and,
above all, honest: it shows what the pipeline measured, while the 3D twin repairs headings and
motion in the browser. It does not make localization better. The limits of the system are in
perception, tracking and multi-camera fusion, not in the view. Recommendation: make the BEV the
primary view for operation and analysis and keep the 3D twin as a presentation view; invest next
in a backend world model (time-aligned multi-camera fusion) and in lane geometry.

## 1. The existing system

| Part | Where | Reusable for BEV | Notes |
|---|---|---|---|
| Detection, ByteTrack, localization, BEV Kalman filter | `src/detection`, `src/tracking`, `src/localization` | yes, unchanged | Already a ground-plane (BEV) pipeline: homography, box fit or keypoint footprint, constant-velocity Kalman filter with anisotropic measurement noise. |
| Heading | `track_localizer.cpp` | yes | `null` until measured; from motion (gated on speed σ, smoothed, turn-rate limited) or box corners (keypoint models); held while slow. |
| World state | `core/world_state.hpp`, `/ws/world` | yes | Metric site frame, per camera, timestamps per frame. Position covariance and the heading's origin were not exposed (added here). |
| Zones, events | `src/vru` | yes | Crosswalk occupancy, VRU–vehicle conflicts (constant-velocity footprint gaps). |
| Multi-camera fusion | `web/src/twin/fusion.ts` | **no** | Runs in the browser on interpolated positions at message arrival time; no timestamps, no uncertainty. |
| Rendering, smoothing | `web/src/twin/twinView.ts`, `interpolation.ts`, `flowField.ts` | **no** (3D-only) | Interpolation one frame behind, missing heading drawn as 0°, entry heading from a flow field learned in the browser, turn-rate limit of the drawn heading, dead reckoning of lost tracks while they fade. |
| Map | `tools/prepare_map_data.py`, `map/mapData.ts` | yes | OSM centre lines with width and `oneway`, crossings, LoD2 buildings, orthophoto. No lanes, stop lines or turn restrictions. |
| Traffic statistics | `manager/traffic_stats.py` | yes | Line crossings per camera (volume, speeds, headways, turning movements between lines of one camera). |

What is tightly coupled to the 3D view is everything that makes it *look* right: the entry heading
from the flow field, the turn-rate limit, the fade-out with dead reckoning and the interpolation.
These hide localization errors by design; the BEV uses none of them.

## 2. Implementation

All new frontend code is in `web/src/bev/`, isolated from the 3D twin:

| File | Content |
|---|---|
| `geometry.ts` | Footprints, covariance ellipses, polygon axis, point in polygon, homography. |
| `bevModel.ts` | `BevWorld`: latest raw world state per camera, raw trajectories, heading status, motion state, standing time, possible ID switches, cross-camera duplicate candidates with their time offset. |
| `laneAnalytics.ts` | Lane, intersection and crosswalk status for the operator: vehicles, standing vehicles, queue length, waiting time, arrivals per minute, mean speed, intersection blocking. |
| `bevView.ts` | Canvas 2D renderer in metres: north up, uniform scale, pan, zoom, selection; draws only when data arrives or the view changes. |
| `bevPanel.ts`, `bevMode.ts` | Status card and the "3D twin / BEV" switch; both views receive the same `WorldState` objects. |
| `perf.ts` | Frame statistics of both views (`window.jpPerf`). |

Backend changes are additive and do not change any algorithm:

- `heading_source` per object (`motion`, `keypoints`, `held`, `label`, or `null`), set where the
  localizer already decides where the heading comes from;
- `position_cov_m2` per object, the position covariance the Kalman filter already had;
- `image_to_ground` in `/api/scene`, so the BEV can draw each camera's ground coverage and the
  counting lines on the ground.

**What the BEV draws, and how it marks certainty**

- Vehicles: oriented footprint with the measured size (solid outline) or the class size (dashed).
  The front triangle is filled for a measured heading, hollow for a held one, dotted where the
  heading is the direction of the lane (only for lanes with a known direction). Without heading a
  vehicle is an orientation-free dashed disc — never a box at 0°.
- VRUs: discs; cyclists and motorcyclists oriented when a heading exists.
- 2σ position ellipse from `position_cov_m2`; velocity vector over 1 s (dashed while the speed is
  uncertain); raw trajectories; objects outside the validated region faded.
- Per camera tracks by default; likely duplicates of two cameras linked with their time offset.
  "Duplicates hidden" drops the less confident track of each pair that is at most 0.25 s apart;
  lane counts always use this deduplicated set. Pairs further apart in time are flagged, not merged.
- Camera ground coverage (image border through the homography, cut at 160 m), so "no road user"
  can be told apart from "not observed".
- Lanes coloured by state, chevrons for a known direction (dashed when from OSM), the queue as a
  bar from the lane end, signals at their placed position with colour and time in state.

**Heading rules**: a missing heading stays missing; a lane direction is used only for a vehicle
without any heading and only where the lane's direction is known from observed traffic (at least
5 tracks, 80 % agreement) or from a one-way OSM road along the lane; the last measured heading is
kept by the backend (`held`) and shown as such; nothing is smoothed in the browser.

**Lane status (for the signal operator)**, every value marked M (measured), E (estimate) or X
(experimental): vehicles and standing vehicles (< 5 km/h, speed σ ≤ 1 m/s; uncertain speeds are
counted separately), queue length (standing vehicles chained from the downstream lane end with
gaps ≤ 10 m; the lane end stands in for the stop line, which is not mapped), the longest standing
time in the queue (restarts with a new track ID), arrivals per minute, mean speed; intersection
blocking (a vehicle standing ≥ 5 s inside the intersection zone); VRUs on each crosswalk; signal
state and its duration; active conflict events (X).

## 3. Measurements

Test setup: a copy of the repository on the GPU server (port 8090) with the TUMTraf S110 s03
site (two cameras, south1 and south2), YOLO26l at 1280 (the production detector), RTX 3080 Laptop
GPU shared with the production copy; browser: the Claude desktop browser (Chromium), canvases
960 × 439 CSS px at device pixel ratio 1.25.

### 3.1 Rendering

Frame cost of one synchronous draw including the GPU (`gl.finish()` for 3D, a pixel read for the
2D canvas), 150 frames per case after warm-up; synthetic road users added through a third camera
feed into both views. Milliseconds, mean / median / P95:

| Road users | 3D twin | BEV | 3D draw calls / triangles |
|---|---|---|---|
| 15 (live) | 5.7 / 3.5 / 12.7 | 5.1 / 4.3 / 9.3 | 137 / 21 k |
| 65 (3D: 50, its fusion merged 15) | 9.0 / 8.5 / 12.6 | 3.9 / 3.6 / 5.3 | 350 / 29 k |
| 200 | 18.2 / 17.0 / 27.7 | 8.1 / 7.8 / 9.7 | 1 245 / 60 k |
| 500 | 31.4 / 30.1 / 40.1 | 15.7 / 15.5 / 17.6 | 3 072 / 124 k |

- At the load of one intersection (tens of road users) both views stay within a 60 Hz frame
  budget; the BEV's cost is then dominated by redrawing the OSM map (could be cached).
- The 3D cost grows with about six draw calls per road user; at 500 road users the 3D twin is
  limited to about 30 fps, the BEV to about 60.
- The 3D twin renders every display frame (60 Hz) whatever the data rate; the BEV only when a
  world state arrives (4–10 Hz here) or the view moves. Derived from the costs above (estimate,
  not measured): about 34 % of a CPU core for 3D at 60 Hz × 5.7 ms against about 2–5 % for the BEV
  at 4–10 Hz × 5 ms.
- Not measured: the real frame rate in the browser (`requestAnimationFrame`) — the browser pane
  was hidden during the run and paused animation frames; the per-frame cost above is the limiting
  factor in any case. GPU utilisation of the views alone could not be separated from the
  detector's on the shared GPU.
- Memory: the page's JavaScript heap was 46–78 MB with both views loaded (not separable per view).
  The 3D twin holds a 4096 × 4096 shadow map (64 MB of GPU memory) and one geometry per mesh part
  (137 for 15 road users); the BEV canvas is 2.5 MB.

### 3.2 Timing and multi-camera consistency (live, 20 s, 40 samples)

| Quantity | Mean | Median | P90 | Max |
|---|---|---|---|---|
| Frame-time offset between the two cameras' latest states (ms) | 367 | 319 | 633 | 742 |
| Age of a camera's latest state at the browser (ms) | — | 151 | 261 | — |

Measured while the GPU was shared (3.3–4 fps per camera); the production copy alone ran at
4–10 fps, so its offsets are smaller but not zero. The 3D twin fuses the two cameras' tracks at
the time messages arrive, so a vehicle at 10 m/s can be compared with itself 3–6 m apart — beyond
the 3 m fusion gate. The BEV reports the offset and does not merge pairs further than 0.25 s apart.

### 3.3 Localization and tracking against ground truth

The visualization does not change these numbers. TUMTraf Intersection R02, S1–S4, both south
cameras, matched to the south-LiDAR labels (`jp_eval`, run of 2026-11-02, box-fit contact model,
gates 2 m vehicles / 1 m VRUs). Errors: mean / median / P90; heading and speed: median / P90.

| | YOLO26l 1280 (production) | Keypoint model jp_pose5 |
|---|---|---|
| Precision / recall | 0.69 / 0.52 | 0.85 / 0.52 |
| Position error (m) | 0.81 / 0.67 / 1.70 | 0.89 / 0.81 / 1.70 |
| Heading error, matched moving (deg) | 2.6 / 11.7 | 2.7 / 11.3 |
| Speed error (m/s) | 0.08 / 1.09 | 0.08 / 1.20 |
| MOTA / IDF1 | 0.28 / 0.56 | 0.42 / 0.60 |
| ID switches | 274 | 57 |
| Standing vehicles with a heading | 62 % (6 654 of 10 653) | 99 % (11 157 of 11 242) |
| Axis error of standing vehicles (deg) | 4.7 / 31.2 | 4.1 / 14.9 |
| Night (S4): recall, position error (m) | 0.18, 1.36 / 1.43 / 1.83 | 0.14, 1.61 / 1.74 / 1.95 |

Cross-camera duplicates and global ID consistency are not evaluated by `jp_eval` (it evaluates one
camera at a time); in the live snapshot the BEV found no duplicate pair, the overlap of the two
cameras' detection areas being small.

### 3.4 Live observations in the BEV (production model, about 3 minutes)

- Standing vehicles mostly have no or only a held heading (snapshots: 0 measured / 9 held /
  4 unknown; 3 / 6 / 6) — the 3D twin draws all of them oriented, from its flow field.
- No measured footprint: the box detector has no corners, every vehicle is drawn at class size.
- south2: 16 of 270 started tracks began where another had just been lost (possible ID switches);
  270 tracks for about a dozen visible road users within minutes shows strong fragmentation.
- At low processing rates the filter's speed σ stays above 1 m/s, so standing and moving cannot be
  told apart (motion "uncertain"): queue and waiting time depend on a processing rate of several
  frames per second.

## 4. Problems found

| # | Area | Problem | Evidence |
|---|---|---|---|
| 1 | Visualization | The 3D twin hides perception errors: missing heading becomes 0° in the interpolator, new vehicles take a flow-field heading, the drawn heading is turn-rate limited, lost tracks keep moving while fading. | `interpolation.ts` (`?? 0`), `twinView.ts` |
| 2 | Fusion | Multi-camera fusion exists only in the browser, on arrival time, without uncertainty or timestamps; the manager's statistics count per camera, so road users in overlaps can be counted twice. | `fusion.ts`, 3.2 |
| 3 | Time | Cameras' states are 0.3–0.7 s apart; video timestamps are relative to each file's start, so cross-camera alignment assumes simultaneous recording starts; no common capture clock for streams. | 3.2 |
| 4 | Tracking | Fragmentation and ID switches with the production detector (274 vs 57 with the keypoint model); standing time and waiting times restart with every new ID. | 3.3, 3.4 |
| 5 | Heading | 38 % of standing vehicles have no heading with the box detector; night heading of the keypoint model flips front and back (P90 103°). | 3.3, benchmark |
| 6 | Localization | Median 0.67 m, P90 1.7 m; worse beyond 40 m and at night (recall 0.14–0.18). Trucks with trailers mostly unmatched. | 3.3, `docs/evaluation.md` |
| 7 | Map | No lane geometry, stop lines, turn relations or signal-to-lane assignment; the production site has crosswalk and intersection zones only, so the lane status stays empty there. | `zones.geojson` |
| 8 | Operations | After the host resumed from sleep, CUDA failed host-wide; all servers silently fell back to the CPU (0.5 fps) with only a log line. | logs of 2026-11-04 |
| 9 | Interface | Position covariance and the origin of the heading were not available to any client (fixed on this branch). | — |

## 5. Traffic analytics: what the data supports

| Feature | Status | Basis |
|---|---|---|
| Trajectories | implemented (raw, per camera) | measured |
| Speed | shown with its σ | measured (filter) |
| Acceleration | not implemented | speed error P90 ≈ 1.1 m/s at 4–10 Hz: its derivative is noise |
| Lane occupancy, standing vehicles | implemented per lane zone | measured |
| Queue length, waiting time | implemented | estimate (lane end as stop line, ID switches restart waiting) |
| Arrivals per lane | implemented (per minute, deduplicated) | estimate |
| Volume, direction, speeds at lines | existing (statistics page) | measured |
| Stop-line crossing | via counting lines drawn at the stop line | measured, needs the lines |
| Intersection occupancy and blocking | implemented | measured / estimate |
| Turning movements | existing between lines of one camera; across cameras needs fusion | — |
| Conflicts | existing events, marked experimental | constant-velocity footprint gaps |
| TTC, PET | **not reported** | need position errors well below 0.5 m, time-aligned cameras and ≥ 10 Hz; P90 1.7 m and 0.3–0.7 s offsets do not meet that |

## 6. Assessment

1. **Is a BEV sufficient for the main use cases?** Yes. Monitoring, queue and lane status,
   counting, speeds, conflicts and the debugging of localization are all ground-plane problems,
   and the pipeline is already a BEV pipeline. The BEV shows them with correct scale and without
   invented detail.
2. **What needs 3D?** Communication and public presentation; visibility and occlusion analysis
   (sight lines, camera placement with buildings); vehicle heights (clearance, trucks hiding VRUs);
   checking camera calibration against buildings. None of them is part of daily operation.
3. **Main limits of localization and tracking:** perspective error growing with range, a P90 of
   1.7 m, weak night performance, box-only detector without footprint and with many ID switches,
   heading only from motion for standing vehicles, a processing rate that drops under load.
4. **Missing for multi-camera fusion:** a common clock (capture timestamps from the cameras, NTP or
   PTP), a backend world model that predicts every camera's tracks to one time step and associates
   them with their covariances, global IDs with hand-over across cameras, evaluation of duplicates
   and ID consistency against multi-camera ground truth.
5. **Towards a production platform:** separate perception (per camera) from a world-model service
   (fusion, global IDs, map matching) and from analytics (lane state, counts, signal performance);
   health monitoring of every stage (GPU fallback, frame rate, latency, calibration drift);
   recorded world states for replay and regression tests.
6. **For signal-performance analysis and optimisation:** lane geometry with stop lines and turn
   relations; signal phases linked to lanes or signal groups (ideally from the controller, not the
   image); vehicle arrival and departure at stop lines per cycle; queue at the start of green,
   residual queue, delay per vehicle and saturation flow; arrivals on green and split failures;
   30 days of history with per-interval aggregates.
7. **Does a BEV-first architecture simplify deployment?** Yes: no WebGL scene, shadow maps or 3D
   models; a smaller, faster frontend that runs on modest hardware and in a control room; the same
   metric data as the analytics, so what is drawn is what is counted. The 3D twin can stay as an
   optional module.
8. **Priorities** — see below.

**Dashboard, world model, digital twin.** A *visualization dashboard* draws what it receives (the
current 3D twin, partly also the BEV). A *perception-based world model* holds one time-aligned,
fused, uncertainty-aware state of all road users with global IDs, lanes and signals — this does not
exist yet; fusion happens in the browser. An *operational digital twin* adds a validated world
model, history, prediction and a closed loop to the infrastructure (signal timing, alerts),
with known accuracy. The project is between the first and the second.

## 7. Roadmap

| Priority | Step | Complexity | Expected benefit |
|---|---|---|---|
| 1 | Lane zones with direction and stop lines (calibration editor), signals assigned to lanes | low | Lane status, queues and stop-line events become available at all sites |
| 2 | Health monitoring: GPU fallback, frame rate, latency and camera offset as alerts in the console | low | Silent degradation like on 2026-11-04 becomes visible |
| 3 | Keypoint detector in production (or train the box model with corners) | low–medium | ID switches 274 → 57, standing heading 62 % → 99 %, measured footprints |
| 4 | Backend world model: time-aligned multi-camera fusion with covariances, global IDs | medium–high | One consistent state for drawing and counting; no double counts; hand-over |
| 5 | Capture timestamps from the cameras and a common clock | medium | Fusion and conflict timing become meaningful |
| 6 | BEV as the default view, 3D twin optional; cache the static map layer | low | Lower client cost, honest display |
| 7 | Signal-performance measures per cycle (arrivals on green, queue at green, delay) | medium | First basis for signal-timing optimisation |
| 8 | Multi-camera ground truth and evaluation (duplicates, global ID switches, time alignment) | medium | Fusion quality becomes measurable |
| 9 | Re-identification across fragments, night model | high | Longer tracks, usable night data |
| 10 | TTC / PET after 4, 5 and position errors < 0.5 m (P90) are reached | medium | Safety metrics that can be trusted |

## 8. Using and testing the BEV

- On the twin page, "3D twin | BEV" in the toolbar; the choice is remembered. Pan by dragging,
  zoom with the wheel, double click to fit, click a road user for its details.
- `?bench=1` redraws the BEV every frame and exposes both views as `window.jpBench`;
  `window.jpPerf` holds frame statistics of both.
- Tests: `web/tests/bev.test.ts` (geometry, heading rules, ID switches, duplicates, queues, lane
  direction) and the C++ tests of the new world-state fields.
