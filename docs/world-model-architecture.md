# Site-level world model: architecture and migration plan

Proposal for review, November 2026. Nothing described here is implemented. It moves multi-camera
fusion and traffic analytics out of the browser into the backend and makes one site-level world
state the single source for the BEV, the 3D twin, the statistics and any other client.

## 1. Where things stand

### 1.1 Data flow today

```text
camera video ──► jp_server (C++, one per camera)
                   detection → ByteTrack → localization (BEV Kalman, heading) → zones, VRU events
                   └─► /ws/world  (world state v1, one camera, media time)
                         ├─► manager (Python): CameraCounter → SQLite crossings/events (per camera)
                         └─► browser (via manager proxy), per camera
                               ├─ 3D twin: interpolation, cross-camera fusion, heading invention
                               └─ BEV: duplicates, ID-switch heuristic, lane direction, queues,
                                       waiting time, arrivals, occupancy, blocking
```

### 1.2 What each part does and where its limits are

| Component | Where | Relevant facts | Consequence |
|---|---|---|---|
| Camera pipeline | `src/sources`, `src/localization` | Per camera: tracks, filtered position, velocity, `speed_std`, heading with `heading_source`, `position_cov_m2` (since `a343505`) | Good base: each camera already estimates state and uncertainty |
| Timestamps | `src/io/frame_source.cpp:49` | `timestamp_ns` = `CAP_PROP_POS_MSEC`: media time since the start of the file; streams: decoder time, no wall-clock capture time | Cameras cannot be aligned in time from the data alone |
| Site clock | `include/jp/server/site_clock.hpp` | Recordings play by one clock (wall time ↔ media time); cameras skip frames when behind | Alignment of *playback*, not of *capture*; live streams play as they come |
| Events | `src/vru/event_detector.cpp` | Per camera, with the site's zones | A VRU seen only by camera A and a vehicle seen only by camera B never form a conflict |
| Statistics | `manager/traffic_stats.py` `CameraCounter` | Line crossings per camera track; time = manager reception (`time.time()`), run per loop | The same vehicle crossing a line in an overlap counts once per camera; times are not capture times |
| Fusion | `web/src/twin/fusion.ts` | Browser, on interpolated positions at arrival, confidence-weighted mean, gate 3 m / 1.5 m, capture-time gate 200 ms | No covariances, one browser's view, not available to statistics |
| Heading in 3D | `web/src/twin/twinView.ts`, `flowField.ts`, `interpolation.ts` | Learned flow field per browser, OSM road axis, right-hand lane side, turn-rate limit, `?? 0` | Headings never measured are shown as if measured |
| Lane analytics | `web/src/bev/laneAnalytics.ts` | Queue, wait, arrivals, occupancy, blocking; lane direction learned per browser (localStorage) | Starts empty per page load, differs between browsers, not stored |
| Zones | `scenes/<site>/zones.geojson` | Polygons with `id`, `type` (`lane`, `crosswalk`, `intersection`); no direction, stop line, signal or movement | Lane semantics are guessed at run time |

### 1.3 Baseline of the frontend logic (measured)

From `docs/3d-fidelity.md` (TUMTraf R02 s03, both south cameras, production detector YOLO26l,
`jp_run` world states replayed into the production `TwinView`, ground truth from the labels of the
same run). Distances in metres, mean / median / P90.

| Quantity | Value |
|---|---|
| Camera-level localization (backend = BEV) vs ground truth | 0.68 / 0.55 / 1.50 |
| Frontend fusion, in-step cameras, 200 ms gate: drawn vs ground truth (overlap) | 1.34 / 1.28 / 1.49 |
| Same, best single camera track (oracle choice) | 1.34 / 1.28 / 1.43 |
| Frontend fusion, one camera 300 ms late, 200 ms gate: duplicates in overlap | 91 % |
| Same: best single camera track vs averaged (∞ gate) | 0.75 vs 1.28 median |
| Identity changes per minute (all 117 labelled road users), ∞ / 200 / 100 ms | 1.6 / 2.0 / 5.9 |
| Interpolation lag, moving road users | 0.27 / 0.11 / 0.76 |
| Camera-level ID switches (benchmark S1–S4, YOLO26l) | 274 (keypoint model: 57) |
| Standing vehicles with a heading (YOLO26l / keypoints) | 62 % / 99 % |
| Axis error of standing vehicles, P90 (YOLO26l / keypoints) | 31° / 15° |
| Live: phantom heading from motion along the viewing ray | observed (cam1, 1.5 m jump → 137° held on a 260° one-way road) |

Not measurable yet: the frontend lane analytics against ground truth, because no lane geometry
exists for the benchmark scenes (phase 0 adds it), and the double counting of the per-camera
statistics, because the benchmark scenes have no counting lines (phase 0 adds them).

Caveat: only 3–4 distinct labelled road users are seen by both TUMTraf south cameras at the same
time in s03. Overlap statistics need S1–S4 and, for controlled time offsets, synthetic data
(CARLA, see §8).

## 2. Goals and non-goals

Goals (the eight requirements):

1. Timestamp-aligned observations from all cameras, by original capture time.
2. Uncertainty-aware cross-camera association and fusion.
3. Stable global road-user IDs through occlusion, camera ID switches and short losses.
4. One heading estimate with explicit provenance and uncertainty; no artificial correction.
5. Persistent lane definitions: direction, stop line, movements, signal association.
6. Backend queue length, waiting time, arrivals, occupancy, turning movements.
7. Persistent statistics independent of browser sessions.
8. One authoritative site-level world state for BEV, 3D, statistics and other clients.

Non-goals of this plan: better detection (separate track: keypoint model, night model), signal
control, multi-host deployment, re-identification by appearance.

## 3. Target architecture

```text
                       ┌───────────────────────── one site ─────────────────────────┐
camera A ─► jp_server ─┤ /ws/observations (v2: capture time, track state, cov, src) │
camera B ─► jp_server ─┤                                                            │
                       │   jp_site (C++, one process per site)                      │
                       │   ├─ TimeAligner     capture clocks, offsets, lag window   │
                       │   ├─ SiteTracker     association, fusion, global IDs      │
                       │   ├─ HeadingEstimator provenance, σ, no invention          │
                       │   ├─ SiteEvents      conflicts, crosswalks (site-wide)     │
                       │   └─ TrafficAnalytics lanes, queues, stop line, movements  │
                       │        │ /ws/site (world state v2)   │ /ws/site-events     │
                       └────────┼─────────────────────────────┼─────────────────────┘
                                ▼                             ▼
                manager (Python): proxy, persistence (SQLite), aggregation, config, health
                                ▼
                browser: BEV, 3D twin, statistics — drawing and interaction only
```

### 3.1 Placement: C++ or Python

| Component | Place | Reason |
|---|---|---|
| Observation export (capture time, covariances) | C++ `jp_server` | Data exists there; additive fields |
| Time alignment, association, fusion, global IDs, heading | **C++ `jp_site`** | Reuses `BevKalman`, `core/assignment`, class catalog, geodesy, zones, JSON; the same code runs offline in `jp_eval`; deterministic and fast (well under 1 ms per tick for 4 cameras × 50 tracks) |
| Site events (conflicts, crosswalks) | C++ `jp_site` | Reuses `vru::EventDetector`, now on fused road users: cross-camera conflicts become possible |
| Lane analytics (queue, wait, arrivals, occupancy, movements) | **C++ `jp_site`** | Needs the fused state at full rate and capture time; one implementation for live and offline evaluation |
| Persistence, aggregation, reports, CSV | Python manager | I/O bound; existing SQLite store and report code; single writer |
| Configuration (lanes, signal groups), versioning | Python manager + files | Same model as zones and site settings today |
| Orchestration, health, restarts, proxy | Python manager | Exists |
| Rendering, interaction | Browser | Only that |

A Python world model was considered: simpler to iterate, numpy-based Kalman filters are fast
enough at this scale. It loses on three counts: the estimation code would exist twice (C++ camera
filters and Python fusion), offline evaluation with `jp_eval` would need a second path, and the
Python manager would gain CPU-bound work in its asyncio loop. Kept in Python: everything that is
storage, aggregation and administration.

Deployment: `jp_site` is a binary of the existing build, started by the manager like `jp_server`,
one per site with two or more cameras (single-camera sites can use it too, for uniform clients).

### 3.2 Module boundaries (C++)

| Module | Content | New / reused |
|---|---|---|
| `jp::core` | `Observation`, `SiteWorldState` types, schema v2 JSON | new types, existing JSON helpers |
| `jp::sync` | `CameraClock` (capture-time mapping, offset, σ), `TimeAligner` (buffers, lag window) | new |
| `jp::fusion` | `SiteTracker` (global tracks, gating, assignment, binding memory, lifecycle), `TrackFusion` (covariance intersection; later measurement-level) | new, uses `BevKalman`, `assignment` |
| `jp::heading` | `HeadingEstimator` (sources, circular fusion, significance tests, provenance) | new, logic moved from `TrackLocalizer` |
| `jp::traffic` | `LaneModel` (geometry, stop line, movements), `LaneState`, `QueueEstimator`, `MovementTracker`, event emitter | new, queue logic ported from `laneAnalytics.ts` |
| `jp::vru` | `EventDetector` on site state | reused |
| `apps/jp_site` | process: subscribes to camera servers, runs the model, serves `/ws/site`, `/ws/site-events`, `/api/site/*`, writes site JSONL | new |
| `apps/jp_eval` | site-level evaluation by capture time | extended |

## 4. Design

### 4.1 Time

**Capture time.** Every observation carries `capture_ns`: absolute Unix time of the exposure.

| Source | How `capture_ns` is obtained | Quality flag |
|---|---|---|
| Image sequences with timestamps (TUMTraf) | File name timestamp | `exact` |
| Recorded videos | `recording_start_ns` (from metadata or set per camera) + media time | `configured` |
| Live RTSP/RTP | RTP timestamp mapped through RTCP sender reports (NTP) of the camera | `ntp` |
| Live without RTCP | arrival time − measured pipeline latency | `estimated` (σ 30–100 ms) |

Cameras must be NTP-synchronised in production; the per-camera offset is monitored (§4.1.3).

**Alignment.** `jp_site` keeps a short buffer per camera and emits the site state for
`t_out = now − L` at a fixed rate (10 Hz). `L` covers capture-to-arrival latency plus one frame
interval of the slowest camera (default 250 ms, configurable; measured latency 130–250 ms). Each
camera track is predicted from its own capture time to `t_out` with its filter's velocity and
covariance (at most one frame interval, ≤ 120 ms); a camera whose latest observation is older than
`max_gap` (default 400 ms) is *stale*: its tracks are not fused, they coast in the global tracker.
No observation is averaged across different instants: everything is first brought to `t_out`, with
the prediction's uncertainty added.

**Offset monitoring.** For road users seen by two cameras, the residual of their fused positions
along their velocity gives the time offset between the cameras (offset ≈ residual / speed).
`jp_site` estimates it continuously (robust median over moving road users), publishes it per camera
and raises an alarm above 50 ms. It does not silently correct with it; a correction is applied only
when configured (`offset_mode: monitor | correct`).

### 4.2 Association and fusion

**Inputs.** Camera *tracks* (not raw detections) in phase 1: position, velocity, their covariances,
class, confidence, heading with source and σ, size, camera track ID, capture time.

**Global tracker.** One constant-velocity Kalman filter per global road user in the site frame.

Per tick:
1. Predict every global track to `t_out`.
2. Predict every fresh camera track to `t_out` (adds `F P Fᵀ + Q` for the gap).
3. Candidate pairs (camera track ↔ global track): class compatible (vehicle / VRU, then class
   confidence), squared Mahalanobis distance of positions with the summed covariances below the
   χ²(2) gate (99 %: 9.21), velocity difference within its gate.
4. **Binding memory first:** a camera track bound to a global track stays bound while its distance
   stays below a looser gate (χ² 18.4); it is unbound only after `n_unbind` (3) consecutive
   violations. This keeps IDs stable when a camera's estimate wobbles.
5. Remaining pairs: optimal assignment (Hungarian, `core/assignment`), at most one track per camera
   per global track.
6. Fuse the bound camera tracks of a global track by **covariance intersection** (consistent for
   correlated track estimates; the weights favour the camera whose ellipse is small in that
   direction: two cameras looking from different sides cancel each other's along-ray error).
7. Unbound camera tracks start tentative global tracks.

**Phase 6 option, measurement-level fusion.** Cameras export detections (ground point and its
anisotropic covariance from the homography Jacobian) and the global filter fuses measurements
directly. More accurate, but the camera trackers stop being the authority on tracks; decided by
phase 1–3 results.

**Global ID lifecycle.**

| State | Enters when | Leaves when |
|---|---|---|
| tentative | new unbound camera track | 3 bound updates in 0.5 s → confirmed; else deleted |
| confirmed | as above | no bound camera track → coasting |
| coasting | no camera sees it | a camera track binds again → confirmed; `T_coast` (1.0 s vehicles, 0.6 s VRUs) or position σ > 2 m → lost |
| lost | as above | kept 3 s for re-attachment, then deleted (ID never reused) |

**Re-attachment (camera ID switch, occlusion).** A new camera track binds to a coasting or lost
global track before a new one is created, if it is within the gate of the global prediction, of the
same kind and moving consistently. The ID-switch heuristic of the BEV becomes a measured event:
`rebind` with the old and new camera track IDs.

**IDs.** `gid` = 64-bit, `site_epoch << 32 | counter`; never reused; the site epoch changes on a
site restart, a seek or a loop of recordings (published as `run_id`).

**Coasting is marked, never hidden.** Coasting road users are published with `status: coasting`,
their predicted position and growing covariance; clients draw them as not observed. Analytics count
only confirmed road users for entries and crossings (§4.5).

### 4.3 Heading

One estimate per global road user:

```text
heading: {
  value_rad | null,            // null while unknown — never 0 by default
  sigma_rad | null,
  source: "keypoints" | "motion" | "fused" | "held" | null,
  front_known: bool,           // false: axis known, front/back not (e.g. keypoints at night)
  since_ns                     // when this estimate was last measured
}
heading_prior: { value_rad, source: "lane" | "osm", lane_id } | null   // separate, never mixed in
```

Rules:
- **Motion** counts only if the fused velocity is significant along its own direction:
  `speed > k · σ_v(direction)` with `k = 3`, using the fused velocity covariance (it contains the
  large along-ray uncertainty of a single camera), *and* a net displacement ≥ 1.5 m over ≥ 0.8 s.
  This rejects the phantom motion found live (1.5 m jump along the viewing ray in 0.6 s).
- **Keypoints** from each camera with their σ; fused with motion on the circle (weights 1/σ²),
  front/back resolved by motion when it is significant.
- **Held**: the last measured estimate, with σ growing slowly while the road user stands
  (e.g. 1°/s) and the time it was measured; becomes unknown after a configured time.
- **Lane or OSM direction** is published only as `heading_prior`, so a client may draw it as such.
- No turn-rate limit or smoothing for appearance; filtering belongs to the estimator and is
  documented with its σ.

### 4.4 Lanes, stop lines, movements, signal groups

New versioned site file `scenes/<site>/lanes.geojson` (zones stay for crosswalks and intersection):

```json
{
  "type": "FeatureCollection",
  "version": 3,
  "coordinate_frame": "site",
  "features": [
    {
      "type": "Feature",
      "geometry": { "type": "LineString", "coordinates": [[-62.0, -8.1], [-14.2, -3.0]] },
      "properties": {
        "kind": "lane", "id": "W1", "approach": "west", "index": 1,
        "width_m": 3.25,
        "stop_line_m": 47.6,
        "movements": [ { "to": "north", "type": "left" }, { "to": "east", "type": "through" } ],
        "signal_group": "SG2"
      }
    },
    { "type": "Feature", "geometry": { "type": "Polygon", "coordinates": [] },
      "properties": { "kind": "exit", "id": "north", "arm": "north" } }
  ],
  "signal_groups": [ { "id": "SG2", "signals": ["south2:signal_1"] } ]
}
```

- A lane is a **centre line drawn in the direction of travel** with a width: the direction is
  defined, never learned or guessed; the stop line is the distance along it (default: its end).
- Exits per arm (polygons) define movements: entry lane → stop-line crossing → exit.
- Signal groups link the camera-read signal areas to lanes.
- Every save increments `version`; analytics and statistics rows carry the lane version.
- Editor: the calibration's map view (draw the centre line from upstream to the stop line, set
  width, movements and signal group); migration tool: a `lane` polygon of `zones.geojson` becomes a
  centre line along its long axis, with the direction to be confirmed by the user.

### 4.5 Traffic analytics (in `jp_site`, on confirmed global road users)

| Quantity | Definition | Basis |
|---|---|---|
| Lane membership | footprint centre in the lane strip, with 0.5 m hysteresis; if the heading is measured and differs more than 60° from the lane, membership is `uncertain` | measured |
| Occupancy | vehicles in the lane; share of lane length covered by footprints | measured / estimate |
| Standing | speed < 1.4 m/s with σ ≤ 1 m/s for ≥ 1 s | measured |
| Queue | standing vehicles chained from the stop line upstream (gap ≤ 10 m), length to the rear of the last, with σ from the positions | estimate |
| Waiting time | per global ID from the start of standing in the lane until it moves again or crosses the stop line; survives camera ID switches because the global ID does | estimate |
| Arrivals | confirmed global IDs entering the lane's upstream part, once per ID and lane | measured |
| Stop-line crossings (departures) | trajectory crosses the stop line in the direction of travel, once per ID; with speed and the signal state at that instant | measured |
| Turning movements | entry lane → stop-line crossing → exit arm, once per ID | measured |
| Blocking | a vehicle standing ≥ 5 s inside the intersection zone | measured |

**Double counting** is excluded by construction: every count is keyed by `(gid, object)` and
recorded once (`object` = lane, line, movement); a road user's re-entry within `recross_s` is not
counted again; coasting road users never start a count; per-camera counting stops once a site has
`jp_site` (§6).

### 4.6 Site world state v2 (`/ws/site`, JSONL)

```json
{
  "schema_version": 2,
  "site_id": "tumtraf_s110",
  "run_id": "2026-11-05T08:00:00Z#4",
  "t_ns": 1793770000123000000,
  "lag_ms": 250,
  "cameras": [
    { "id": "south2", "last_capture_ns": 1793770000101000000, "offset_ms": 12.0,
      "offset_sigma_ms": 6.0, "time_source": "ntp", "state": "ok", "fps": 8.6 }
  ],
  "road_users": [
    {
      "gid": 4294967321, "status": "confirmed", "class": "car", "class_conf": 0.86,
      "vru": false, "rider": false,
      "x": -9.71, "y": -47.92, "cov_m2": [0.09, 0.01, 0.11],
      "vx": 2.1, "vy": 1.1, "vcov_m2s2": [0.04, 0.0, 0.05],
      "speed_mps": 2.37, "speed_sigma_mps": 0.21,
      "heading": { "value_rad": 1.08, "sigma_rad": 0.05, "source": "motion",
                   "front_known": true, "since_ns": 1793770000023000000 },
      "heading_prior": null,
      "size": { "length_m": 4.4, "width_m": 1.8, "measured": false },
      "observed_by": [ { "camera": "south2", "track": 28, "capture_ns": 1793770000101000000,
                         "residual_m": 0.31 } ],
      "age_s": 12.4, "lane": "W1", "zones": [], "validated": true
    }
  ],
  "lanes": [
    { "id": "W1", "version": 3, "vehicles": 4, "standing": 3,
      "queue": { "length_m": 18.2, "sigma_m": 1.1, "vehicles": 3 },
      "max_wait_s": 41.0, "occupancy": 0.31, "arrivals_per_min": 6.0, "signal": "red" }
  ],
  "events": [],
  "signals": []
}
```

Discrete events on `/ws/site-events` and in JSONL: `lane_enter`, `stand_start`, `stand_end`,
`stop_line_cross`, `movement`, `rebind`, `gid_lost`, `conflict`, `crosswalk`, each with `t_ns`,
`gid`, and the lane version.

### 4.7 Camera stream v1 → observation fields (additive)

Added to each object of `/ws/world` (old clients ignore them; the frontend schema already accepts
optional fields): `capture_ns` (state level), `time_source`, `vcov_m2s2`, `heading_sigma_rad`,
`front_known`, `contact_px` (image point the position comes from, for along-ray checks),
`measured_this_frame`. No change of meaning of existing fields.

### 4.8 APIs

| Endpoint | Owner | Change |
|---|---|---|
| `WS /sites/<id>/ws/site` | jp_site via manager | new: site world state v2 |
| `WS /sites/<id>/ws/site-events` | jp_site via manager | new |
| `GET /sites/<id>/api/site/status` | jp_site | new: cameras, offsets, lag, tick time, counts |
| `GET/PUT /manager/api/sites/<id>/lanes` | manager | new: versioned lane file |
| `GET /manager/api/stats/...` | manager | extended: lanes, queues, waits, movements by `gid`; `source=site|camera` |
| `GET /manager/api/sites` | manager | `world_model: off | shadow | on` per site |
| `WS /sites/<id>/cameras/<c>/ws/world` | jp_server | unchanged, plus §4.7 fields |

### 4.9 Storage (manager, SQLite in WAL mode)

| Table | Rows | Estimate (one site, 12 lanes, busy day) |
|---|---|---|
| `road_users` (gid, run, class, first/last seen, cameras, lanes, movement) | one per road user | ~30 k/day, ~5 MB |
| `traffic_events` (t, gid, kind, lane, line, movement, speed, signal, lane_version) | one per event | ~150 k/day, ~20 MB |
| `lane_samples` (t, lane, vehicles, standing, queue_m, queue_sigma, max_wait, occupancy) | 1 Hz per lane, on change | ≤ 1 M/day, ≤ 60 MB |
| `aggregates_15min` (lane, movement, counts, mean wait, max queue, …) | 96 per lane per day | small |
| `runs`, `lane_versions`, `camera_health` | few | small |

Retention: raw samples and events 30 days (configurable), aggregates unlimited. Writes batched
(every second, one transaction). If a site outgrows SQLite (many sites, years of raw data),
PostgreSQL with a time-series extension behind the same report API.

### 4.10 Failure handling

| Failure | Behaviour |
|---|---|
| Camera stream stops | camera `stale` after `max_gap`; its tracks stop being fused; global tracks coast and are lost by the lifecycle; status and alarm |
| Camera time offset unknown or > 50 ms | camera flagged; with `offset_mode: monitor` its tracks are associated but not fused (published as separate observations of the same `gid` with a warning) |
| jp_site crash | manager restarts it (as jp_server); new `run_id`; clients fall back to camera streams with a visible notice; statistics mark the gap |
| Manager restart | jp_site keeps running; events buffered in jp_site (ring buffer, 5 min) and replayed with sequence numbers, duplicates ignored by the store (unique `(run_id, seq)`) |
| SQLite write error | batch kept in memory and retried; alarm |
| Seek / loop of recordings | new `run_id`; global state cleared; statistics per run |
| GPU fallback to CPU (2026-11-04) | camera status `degraded` from jp_server's provider; alarm in the console |

## 5. Pitfalls and how the design avoids them

| Pitfall | Countermeasure |
|---|---|
| Double counting | counts keyed by `(gid, object)`, once; coasting road users never start a count; per-camera counting disabled when the site model is on; test: two-camera replay counts = labels ±3 % |
| Unstable global IDs | binding memory with hysteresis (enter χ² 9.21, keep χ² 18.4, unbind after 3 violations); re-attachment before creation; IDs never reused; measured: global ID switches and IDF1 per run |
| Timestamp mismatch | capture time per observation, alignment to `t_out` with prediction and its covariance; stale cameras not fused; offset monitored; no fusion across `max_skew` without prediction |
| Artificial heading | heading only from measurements with provenance and σ; priors separate; motion significance along its own direction plus displacement; no turn-rate smoothing for display; test: rate of headings >45° off ground truth for standing road users |
| Averaging unlike estimates | covariance intersection instead of confidence-weighted mean; per-camera residuals published |
| Lane direction guessed | direction defined by the drawn centre line |
| Analytics differ between clients | computed once in jp_site; clients only draw |

## 6. Migration plan

Every phase runs behind the per-site flag `world_model: off | shadow | on`. In `shadow`, jp_site
runs and is evaluated, clients and statistics still use the old path.

| Phase | Content | Acceptance criteria (measured) | Rollback |
|---|---|---|---|
| **0. Baseline and ground truth** (1–2 wk) | §4.7 observation fields incl. `capture_ns`; `jp_eval --site` evaluates states by capture time; lanes, exits and counting lines drawn for the TUMTraf S110 frame; GT analytics: the same lane code run on labelled tracks; CARLA multi-camera GT checked; baseline report of the frontend fusion and BEV analytics (TS modules run headless on the same JSONL) | `capture_ns` within ±5 ms of image times on TUMTraf; baseline report for S1–S4 with all metrics of §7; schema tests pass | fields are additive; nothing to roll back |
| **1. jp_site in shadow** (3–4 wk) | TimeAligner, SiteTracker (CI fusion, binding, lifecycle), `/ws/site`, site JSONL; offline mode reading camera JSONL | Overlap: median position error ≤ best single camera, P90 ≤ 0.8 × frontend fused P90; duplicates in overlap ≤ 2 % in-step and with a 300 ms offset (offset corrected or flagged); global ID switches/min ≤ frontend ∞ baseline (1.6); IDF1 ≥ best camera; single-camera regions unchanged (±0.02 m); added latency p95 ≤ 300 ms; CPU ≤ 0.5 core per site | flag `off`; process not started |
| **2. Heading** (2 wk) | HeadingEstimator with provenance and σ; heading moved out of the per-camera display logic | Standing vehicles with heading > 45° off ground truth: −50 % vs camera level; phantom-motion headings (ground truth standing, heading from motion): −80 %; moving heading P90 not worse than 11.7°; no prior published as measured (unit test) | flag; camera headings still in v1 |
| **3. Lanes and analytics** (3–4 wk) | `lanes.geojson`, editor, migration of lane zones; TrafficAnalytics; events; manager tables and aggregates (shadow) | Against GT analytics on S110: queue length error median ≤ 3 m, P90 ≤ 8 m; arrivals per 5 min ±5 %; stop-line crossings ±3 %; turning movement matrix ≥ 90 % correct; zero double counts in overlap tests; statistics identical after manager and jp_site restarts (replay test) | flag; tables unused; lane file ignored |
| **4. Clients on the site state** (2 wk) | BEV and 3D read `/ws/site`; frontend fusion, flow field, interpolation-heading and lane analytics removed behind the flag; coasting drawn as such | Fidelity harness: drawn = site state, 0.000 m; the same numbers in two browsers; render cost unchanged ±10 %; fallback to camera streams works when jp_site stops | flag `shadow`: clients use camera streams again |
| **5. Statistics from the site model** (2 wk) | crossings, movements, queues and waits from `traffic_events`; per-camera counting off for sites with `on`; old rows kept read-only and labelled | Two-camera replay: counts = labels ±3 %; before/after over-count in overlaps reported; reports unchanged for single-camera sites | per-camera counting re-enabled by flag; new tables kept |
| **6. Live operation and hardening** (3 wk) | RTP/RTCP capture times, offset monitoring and alarms, health page, retention, optional measurement-level fusion | Offsets of live cameras within ±20 ms (NTP) and monitored; 72 h soak without leak or ID-counter overflow; restart drills pass | per-camera time source switchable to `estimated` |

Effort: about 16–20 weeks for one developer, phases 0–3 being the core (9–12 weeks).

## 7. Validation against ground truth

**Data.**
- TUMTraf R02 S1–S4, cameras south1 and south2, labels from the LiDAR (one track ID per road user
  across both cameras: the reference for global IDs). Small overlap: report overlap metrics with
  their sample size.
- CARLA (`carla_denninger_24mm`, four approach cameras, exact calibration): if its ground truth
  export is available, the controlled case for time offsets (0, 100, 300 ms injected), occlusions
  and hand-overs.
- Live recordings: no ground truth; consistency checks only (double counts, ID switches, offsets).

**Method.** `jp_run` per camera (world JSONL with capture times) → `jp_site --offline` → site
JSONL → `jp_eval --site`: labels interpolated to each `t_ns`, matched with the existing gates (2 m
vehicles, 1 m VRUs).

**Metrics** (mean / median / P90 where meaningful), each compared with the phase 0 baseline:

| Group | Metrics |
|---|---|
| Position | error overall, in the overlap, per camera-only region; along-ray vs across-ray |
| Identity | global ID switches, IDF1, MOTA, fragmentation, duplicates (labels matched by > 1 road user), re-attachments |
| Time | estimated camera offsets vs known (CARLA) or image times (TUMTraf); error vs prediction gap |
| Heading | error by source; standing vehicles: share with heading, axis error, share > 45° off; phantom-motion rate |
| Analytics | queue length error (vs GT analytics), waiting time error, arrivals, stop-line crossings, movement matrix accuracy, double counts |
| System | latency (capture → `/ws/site`), CPU and memory per site, restart behaviour |

## 8. Backward compatibility

- Camera stream v1 keeps every field and meaning; new fields are optional (frontend schema already
  treats unknown optional fields correctly).
- Old JSONL recordings replay per camera as today; `jp_site --offline` accepts them with
  `time_source: configured` (start time per camera given).
- `zones.geojson` stays for crosswalks and the intersection; lane polygons are migrated with the
  user's confirmation; until then, lanes without direction produce no queue (as now).
- Statistics: old per-camera rows stay queryable and are labelled `source=camera`; new rows
  `source=site`; reports show which source a number comes from.
- Clients detect `world_model` per site and fall back to camera streams.

## 9. Open decisions for review

1. Track-to-track fusion with covariance intersection first (proposed) or measurement-level fusion
   from the start.
2. Output lag `L` (250 ms proposed): lower latency or fewer stale cameras.
3. Lane editing: centre lines (proposed) or keeping polygons with an added direction and stop line.
4. Retention of raw samples (30 days proposed).
5. Live time source: require NTP-synchronised cameras with RTCP, or accept estimated offsets.
