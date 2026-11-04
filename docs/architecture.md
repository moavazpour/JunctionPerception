# Architecture

What exists on branch `refactor/bev-digital-twin` (November 2026). Rules for changing it:
[AGENTS.md](../AGENTS.md). Decisions: [docs/adr/](adr/).

## 1. System

```mermaid
flowchart LR
    subgraph camera["per camera: jp_server (C++)"]
        V[video file / image sequence / stream] --> D[detection<br/>YOLO26, ONNX Runtime]
        D --> T[tracking<br/>ByteTrack, riders]
        T --> L[localization<br/>homography, box fit or corners,<br/>BEV Kalman, heading + source]
        L --> O[camera observation<br/>schema 2]
        V --> S[traffic-light colour]
        S --> O
    end
    O -- "/ws/world" --> F
    subgraph manager["manager (Python, one process)"]
        F[site_feed.py<br/>follows the running cameras] --> W[world_model.py<br/>time alignment, association,<br/>fusion, global IDs, lifecycle]
        W --> TR[traffic.py<br/>lane queue, passing vehicles]
        TR --> SS[site state v1]
        C[site clock, config,<br/>calibration files, processes]
    end
    SS -- "/sites/&lt;id&gt;/ws/site" --> B[browser: BEV view<br/>draws only]
    SS -- "/manager/api/sites/&lt;id&gt;/state" --> H[headless clients, tools]
    O -- "jp_run --world-jsonl" --> E[jp_eval, tools/eval_fusion.py]
    C -. clock.json .-> camera
```

Processes: one manager (`manager/jp_manager.py`, aiohttp) per host; one `jp_server` per running
camera, started, supervised and proxied by the manager. The site world model runs inside the
manager, one instance per site, whether or not a browser is connected; every browser receives the
same site state.

## 2. Ownership

| Concern | Owner | Code |
|---|---|---|
| frame decoding, timestamps, time source | C++ | `src/io`, `src/sources/world_sources.cpp` |
| detection, rider association | C++ | `src/detection` |
| tracking, camera-local track IDs | C++ | `src/tracking` |
| ground position, velocity, heading + source, size, covariances | C++ | `src/localization` (`TrackLocalizer`, `BevKalman`) |
| validation flag (near the calibration's control points) | C++ | `localization::ValidatedRegion`, `sources::mark_validated` |
| traffic-light colour | C++ | `src/sources/signal_reader.cpp` |
| calibration fit, homography, camera pose | C++ | `src/calibration`, served by `jp_server` |
| time alignment of cameras, association, fusion, global IDs, coasting | Python | `manager/world_model.py` |
| site/camera configuration, calibration files, playback clock, processes, logs | Python | `manager/jp_manager.py` |
| lanes from the cameras' regions, lane queue, passing vehicles per lane | Python | `manager/traffic.py` |
| live feeding of the world model, site-state stream | Python | `manager/site_feed.py` |
| drawing, video, calibration and configuration UI | TypeScript | `web/src` |

Not owned by anyone (removed): 3D twin, VRU/conflict events, warnings, risk, waiting times.
See ADR 0003; lane queue and passing counts came back in ADR 0004.

## 3. Contracts

### 3.1 Camera observation (schema 2)

One JSON object per frame on `jp_server`'s `/ws/world` and per line in `jp_run --world-jsonl`.
Type: `core::WorldState`; serializer `src/twin/world_state_json.cpp`; fixture
[`tests/fixtures/camera_observation_v2.json`](../tests/fixtures/camera_observation_v2.json),
reproduced exactly by the serializer and read by the world model. Version 1 recordings (with
events, zones, turning flags) are still read; those fields are ignored.

| Field | Meaning |
|---|---|
| `schema_version` | `2` |
| `scene_id`, `camera_id` | site and camera folder (`camera_id` empty for single-camera scenes) |
| `frame` | frame index of the source |
| `timestamp_ns`, `time_source` | `media`: since the start of the recording; `capture`: absolute capture time (Unix ns, image sequences with timestamps); `arrival`: absolute receive time (live streams, includes network and decoder latency) |
| `source` | `live`, `oracle` (benchmark labels) or `replay` (JSONL) |
| `objects[]` | camera tracks: `id` (camera-local), `class`, `vru`, `rider`, `x`, `y` (m, site frame), `vx`, `vy`, `speed_mps`, `speed_std_mps`, `heading_rad` + `heading_source` (`motion`, `keypoints`, `held`, `label`; both `null` while unknown), `position_cov_m2` `[xx, xy, yy]`, `confidence`, `length_m`, `width_m` (measured footprint or `null`), `validated` |
| `signals[]` | traffic lights of the camera: `id`, `name`, `state`, `confidence`, `since_ns` |

### 3.2 Site state (`site_state` v1)

The only physical road-user state clients see. Producer `manager/world_model.py`
(`SiteWorldModel.site_state`) plus `traffic` from `manager/traffic.py` (added by the site feed), consumer `web/src/net/siteState.ts` (`isSiteState`), fixture
[`tests/fixtures/site_state_v1.json`](../tests/fixtures/site_state_v1.json) tested on both sides.

| Field | Meaning |
|---|---|
| `schema`, `schema_version` | `"site_state"`, `1` |
| `site_id`, `seq`, `run` | `run` changes on a seek, loop or restart: IDs start over |
| `t_ns`, `clock` | the instant the state describes, on the site clock (`media`, `unix`, or `mixed` when unfused) |
| `coordinate_frame` | `"site"`: the site's world frame (east-north-up at its geo anchor, or a sensor frame) |
| `fusion` | `mode`: `single`, `fused` or `unfused`; `reason` when unfused |
| `cameras[]` | per camera: `state` (`ok`, `stale`, `offline`), `time_source`, `offset_ms`, `last_t_ns`, `lag_ms`, `frame`, `objects` |
| `road_users[]` | `id` (global, never reused within a run), `class`, `vru`, `rider`, `confidence`, `status` (`tracked`, `coasting`), `x`, `y`, `position_cov_m2`, `vx`, `vy`, `speed_mps`, `speed_std_mps`, `heading_rad` + `heading_source` (unknown stays `null`), `length_m`, `width_m`, `size_source` (`measured`, `class`), `validated`, `first_seen_ns`, `last_observed_ns`, `observed_by[]` (`camera`, `track_id`, `t_ns`, `dt_ms`) |
| `signals[]` | the cameras' traffic lights, with `camera` |
| `traffic` | `since_ns` (start of the run: counts start over with it), `present_by_class` (road users now), `lanes[]`: `id` (`<camera>/<line id>`), `camera`, `name`, `direction_set`, `vehicles` (in the lane now), `queue` (of them slower than `queue_speed_mps`), `passed` and `passed_by_class` (drove through in the lane direction) |

## 4. Time

- **Recordings** (`media`) play by one site clock when a site has two or more cameras with
  recordings (`.cache/manager/sites/<site>/clock.json`, written by the manager, followed by every
  `jp_server`): all cameras show the same recording position, and seek, pause and loop act on
  the whole site. Media time is comparable across cameras only under that clock, plus each
  camera's `time_offset_ms` (camera setting) for recordings that did not start together.
- **Image sequences** with timestamps (`capture`) and **live streams** (`arrival`) are absolute
  and comparable directly; `arrival` includes the latency of each stream, which a configured
  `time_offset_ms` can compensate.
- **Alignment** (`SiteWorldModel.step`): the site state is produced for the newest instant every
  fresh camera has reported (`t_out` = the oldest latest time among cameras no more than
  `stale_after_s` behind the newest). Each camera's state nearest before `t_out` is predicted to
  `t_out` with its velocity (at most `max_predict_s`), its covariance grown by the velocity
  uncertainty. No positions of different instants are averaged.
- **Degraded mode**: cameras on different clock kinds, recordings without a common clock, or
  `world_model.fuse_cameras: false` give `fusion.mode = "unfused"` with the reason: each camera's
  road users are shown at their own latest time, never associated across cameras.

## 5. Association and fusion

Per step, deterministic (cameras in id order, tracks in id order):

1. Every global road user is predicted to `t_out` (constant velocity if its speed is known to
   `coast_max_speed_sigma_mps`, else in place), covariance grown.
2. A camera track bound to a road user stays bound while its squared Mahalanobis distance (summed
   covariances plus `camera_bias_m` for cross-camera calibration disagreement) stays below
   `keep_chi2`; after `unbind_after` violations in a row it is unbound.
3. Unbound tracks are matched to road users of the same kind (vehicle/VRU) without a track of
   that camera: gate `gate_chi2` and Euclidean ceilings (`max_distance_*_m`), velocity difference
   within `max_velocity_difference_mps` plus 2σ. A track whose second-best candidate is within
   `ambiguity_margin_chi2` of the best is not merged; it starts its own road user. Coasting road
   users are candidates too: camera ID switches and short occlusions keep the global ID.
4. Road users of different cameras that are mutually nearest within the gate, unambiguously, for
   `merge_after` steps are merged (the older ID stays).
5. Fusion of the bound tracks: positions and velocities weighted by their inverse covariances
   (cameras measure independently); class, confidence and size from the most confident member;
   the heading from the member with the best source (`label` > `keypoints` > `motion` > `held`),
   never averaged, never invented.
6. A road user without tracks coasts (`status: coasting`, heading source `held`) for
   `coast_vehicle_s` / `coast_vru_s`, then is dropped. IDs are never reused within a run.

Parameters: `FusionSettings` in `manager/world_model.py`, overridable in `configs/manager.yaml`
(`world_model:`).

## 5a. Lane traffic

A lane is a line with three or more points that the user drew in a camera image
(`regions.json`), with the direction of travel the user set there (two clicks, tail and head).
The manager projects it into the site frame with that camera's homography (`ground_homography`:
the scale whose `w` is positive at the image's bottom centre, as in the frontend) and
`manager/traffic.py` measures it on every site state, vehicles only (`vru: false`):

- **in the lane**: the road user's position lies inside the lane polygon;
- **queue**: vehicles in the lane slower than `queue_speed_mps`;
- **passed**: a vehicle that left the lane (or the site) having travelled at least
  `min_travel_m` along the lane direction since it entered; once per global ID and lane, so a
  vehicle seen by several cameras counts once. Lanes without a direction count nothing.

Counts start over with each run (`run` changes on seek, loop or restart) and are not stored.
Lanes two cameras both outline are separate lanes and both count. Thresholds: `TrafficSettings`,
`configs/manager.yaml` (`traffic:`).

## 6. Frontend

`web/src/pages/twin.ts` subscribes to `/sites/<id>/ws/site` for every placed site and draws it
with `web/src/bev/bevView.ts` (2D canvas, north up, metric): map layers, camera coverage (from
each camera's homography), zones, detection lines, traffic lights and road users exactly as
reported — footprint along a known heading (dashed outline for class sizes), a disc when the
heading is unknown, faded when coasting, 2σ ellipse, velocity arrow (dashed when not clearly
above its noise), trail of the reported positions. Camera video comes from each camera's
`/ws/video`. Configuration in the view: map alignment of sensor-frame sites (drag/turn the site
over its map, saved as `map_transform`) and placement of traffic lights (saved in the camera's
`regions.json`). `web/src/bev/siteFrame.ts` holds the site→view transforms; they are tested to
keep positions metric and unchanged. Other pages: overview, site settings, new site,
calibration, system, logs.

## 7. Coordinate frames

| Frame | Unit | Description |
|---|---|---|
| `image` | px | camera image (custom videos assumed free of lens distortion) |
| `scene_ground` / site frame | m | metric ground plane of a site, z = 0 on the ground; for geo-referenced sites east-north-up at `geo_anchor`, for benchmark scenes the ground plane fitted to the labels in the south LiDAR frame |
| `wgs84` | deg | control points, zones, geo anchor |
| view | m | the BEV: east-north-up at the first site with a location; sites at their geographic offset, sensor-frame sites turned by their `map_transform` |

Every camera of a site must be calibrated into the same site frame; the world model assumes it.

## 8. Perception details

- **Position uncertainty.** The image noise of the ground contact is propagated through the
  homography into an anisotropic, range-dependent covariance (along the viewing ray one pixel
  covers range² / (focal length × camera height)), plus a floor for the footprint model and the
  calibration. Measurements the motion model cannot explain are gated; persistent ones re-anchor
  the track with unknown velocity. Process noise per class.
- **Keypoint detector.** `models/jp_pose5_736x1280.onnx` (class source `jp_pose5`) gives the four
  bottom corners of each 3D box: their centre is the position, the sides the size, the corner
  order the heading with its front, also for standing road users (`heading_source: keypoints`).
- **Heading.** From significant motion (`motion`), corners (`keypoints`) or held while too slow to
  measure (`held`); `null` until measured.
- **Detection areas and lines** (`regions.json`, Calibration → "ROI and lines"): detections
  outside the areas, or farther than `band_m` from a banded line, are dropped before tracking.
- **Traffic lights**: the lit lamp of each signal area is read every frame and confirmed over
  `signals.confirm_frames`, held over flicker for `signals.hold_frames`.
- **Validation**: for scenes calibrated from control points, positions outside the control-point
  outline plus `validation.margin_m`, or where one pixel along the ray exceeds
  `validation.max_ground_m_per_px`, are flagged `validated: false` (drawn faded).

## 9. Interfaces

Camera server (`jp_server`, proxied as `/sites/<id>/cameras/<camera>/...`): `GET /api/scene`
(camera, image size, homography, zones, classes), `GET /api/status`, `POST /api/playback`,
`WS /ws/world` (observations), `WS /ws/video` (8-byte frame index + JPEG), `GET /api/frame`,
`GET|POST /api/calibration…`, `POST /api/zones`, `GET|POST /api/regions`, `POST /api/restart`.

Manager (`/manager/api/...`, full list in the docstring of `jp_manager.py`): sites and cameras
(CRUD, start/stop, playback of the whole site), `GET /manager/api/sites/<id>/state` (latest site
state), `WS /sites/<id>/ws/site` (site state stream), system, inputs, probe, uploads, logs. A
site's description carries `runtime.world` (fusion mode, road users, time) for the console
lists. Camera setting `time_offset_ms` shifts a camera's timestamps on the site clock.

## 10. Extending

- New perception quantity: compute it in C++, add it to `core::WorldObject` and the serializer,
  update the observation fixture; if clients need it, carry it through `world_model.py`
  (`road_user()`), the site-state fixture and `siteState.ts`, then draw it.
- New fusion behaviour: `world_model.py` + a synthetic test in `manager/tests/`; measure with
  `tools/eval_fusion.py` on TUMTraf before and after.
- New UI: `web/src/pages` / `web/src/ui`; it may read only the site state, the scene description
  and manager APIs.
- A new process or service, a new owner of a calculation or a schema version needs an ADR.

## 11. Testing

| Suite | What it guards |
|---|---|
| `ctest` (C++, 104 tests) | perception, calibration, localization, tracking, observation schema (fixture byte-for-byte), zones, evaluation, server config |
| `manager/tests/test_world_model.py` (21 tests) | single/multi camera, delays and offsets, stale cameras, association and ambiguity, ID stability through ID switches and occlusion, coasting, unknown headings, unfused modes, seek/runs, determinism, both fixtures |
| `web/tests` (vitest, 26 tests) | site-state contract, BEV positions = backend positions, map alignment math, architecture rules |
| `tools/eval_fusion.py` | site model vs TUMTraf labels (see docs/evaluation.md) |

Run everything: `scripts/in_docker.sh scripts/build_and_test.sh all release`.
