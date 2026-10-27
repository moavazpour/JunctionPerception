# Interfaces

JunctionPerception is one module per camera. It takes a video stream plus a small YAML site
configuration and emits metadata. Images leave the device only in two optional forms: a pixelated
calibration snapshot (MQTT) and the live camera view for the panel (HTTP MJPEG, `preview.enabled`).

## Input

| Field | Meaning |
|---|---|
| `camera.source` | `file://…`, `rtsp://…` (ONVIF/RTSP IP cameras), `usb:N`, `v4l2:///dev/videoN`, or `gst:<pipeline>` (GigE via aravis, Jetson CSI, NVDEC, …) |
| `camera.position` | WGS84 position of the camera, approximate mounting height (± sigma) and rough viewing bearing |
| `camera.intrinsics` | optional focal length / distortion; estimated automatically when absent |
| `calibration.road_bearing_deg` | bearing of the observed road. `tools/osm_site.py` derives it from OpenStreetMap |
| `map.stop_line` | a point on the stop line of the junction ahead (OSM traffic signal node or surveyed) |
| `map.file` | optional hand-made lane map; otherwise lanes are learned from traffic |

## Output: MQTT

Topics are `jp/<site>/<camera>/<leaf>`.

Every JSON message carries:

```json
{"schema": "junction-perception/1.0", "type": "...", "site": "...", "camera": "...", "ts": 1729850000.123}
```

All timestamps are stream time (UNIX seconds).

| Leaf | Rate | Retained | Content |
|---|---|---|---|
| `objects` | 10 Hz | no | road users (below) |
| `cpm` | 10 Hz | no | ETSI TS 103 324 Collective Perception Message content (JSON, standard units) |
| `detectors` | 2 Hz + on change | yes | virtual detector zones and lane states |
| `detector/<zone>` | on change | yes | `"0"` / `"1"` presence: the simplest controller input (loop-detector emulation) |
| `events/safety` | on event | no | near-miss and hard-braking events with surrogate safety measures |
| `events/stopline` | on event | no | stop-line crossings per lane |
| `health` | 1 Hz | yes | integrity level, confidence and reasons, plus stability, visibility, calibration and coverage |
| `occlusion` | 1 Hz | no | BEV visibility grid (RLE) and occluded lane segments with cause |
| `calibration` | on change | yes | camera model and calibration quality (`method`: `keypoints` or `vanishing-point`; `source`: `auto`, `manual`, `file`, `drift-compensated`) |
| `map` | on change | yes | lanes, stop lines, zones, crosswalks (local ENU metres) + camera |
| `snapshot` | calibration mode | yes | optional pixelated JPEG for the calibration view (also before any calibration exists), with the frame size and the configured camera position |
| `calibration/manual/result` | on command | no | outcome of an operator command (below) |
| `calibration/inputs` | on solve | yes | the last manual-calibration inputs (points, lines, verticals, height, lens), so the panel can reload them |
| `calibration/refine` | 0.2 Hz while collecting | yes | traffic check of a manual calibration: `status` (`collecting`, `solving`, `proposal`, `consistent`, `rejected`), `progress`, `tracks`, `spread_deg`, optional `proposal` (before/after path spread and point fit, pitch/roll/yaw/focal correction, camera) |
| `config` | on change | yes | live settings, compute (`backend` `tensorrt`/`cpu`, `device`, `precision`, `model`, `input`), source (`live`, `loop`, size, fps), camera `placement`, live-view port |
| `map/user` | on change | yes | installer-drawn layout (WGS84): `lanes` (`id`, `points` in travel direction, `width`), `stop_lines`, `crosswalks`, `roi` |

### Operator commands (subscribed)

The web panel publishes JSON commands to `<prefix>/<site>/<camera>/calibration/manual`:

| `action` | Fields | Effect |
|---|---|---|
| `solve` | `points: [{u, v, lat, lon}, …]` (≥ 4; frame pixels), optional `lines: [{px: [[u, v], …], geo: [[lat, lon], [lat, lon]]}]`, `verticals: [[[u, v], [u, v]]]`, `height_m`, `height_sigma`, `lens` (`unknown`, `phone_wide`, `phone_2x`, `phone_3x`, `cctv`), `apply` (default true), `force` | manual calibration (maximum a-posteriori: points and lines weighted by click and map accuracy at their distance, robust loss, priors on focal length, height and position). Applied when the fit is acceptable (or `force` and physically plausible) |
| `auto` | – | discard the calibration (manual mode: wait for a new one) |
| `snapshot` | – | publish a fresh calibration snapshot |
| `refine_accept` / `refine_reject` | – | apply or dismiss the traffic-check proposal |

`solve` is answered on `calibration/manual/result` with `ok`, `plausible`, `applied`, `quality` (`good`
≤ 0.5 m, `fair` ≤ 1.25 m, `poor`), `ground_rms_m`/`ground_max_m`, `rms_px`/`max_px`, `focal_px`,
`fov_deg`, `pitch_deg`, `roll_deg`, `height_m`, `bearing_deg`, `warnings`, per-input `residuals`
(`kind`, `index`, `px`, `m`), `accuracy` (expected position sigma by range) and the solved `camera`.

`<prefix>/<site>/<camera>/map/user/set` takes `{"map": {lanes, stop_lines, crosswalks, roi}}` (as
`map/user`); drawn lanes replace the learned ones. `<prefix>/<site>/<camera>/config/set` takes
`{"settings": {…}}` with any of `confidence`, `keypoint_confidence`, `max_range_m`, `max_fps`, `classes`,
`roi_filter`, `preview_overlay`, `preview_fps`; applied within a frame and persisted in
`state_dir/settings.json`. Enable MQTT authentication before exposing these channels.

### Live camera view (HTTP)

Each runtime serves `GET /video.mjpg` (MJPEG, `preview.fps`, `preview.width`, optional boxes) and
`GET /frame.jpg` on `preview.port` (default 8081); frames are only encoded while someone watches. The
panel proxies them at `/live/<site>/`. Unlike the calibration snapshot, this view is not pixelated:
disable it (`preview.enabled: false`) where video may not leave the device.

### Road user (`objects[]`)

| Field | Unit | Meaning |
|---|---|---|
| `id` | – | track id, stable while the object is tracked |
| `class` | – | `car` `bus` `truck` `trailer` `pedestrian` `bicycle` `escooter` `motorcycle` |
| `state` | – | `tracked` (measured this frame) or `predicted` (coasting, e.g. behind an occluder) |
| `x`, `y` | m | position of the footprint centre in the local ENU frame (origin below the camera) |
| `lat`, `lon` | deg | WGS84 position |
| `heading` | deg | compass heading of travel |
| `speed`, `speed_kmh` | m/s, km/h | |
| `accel` | m/s² | longitudinal acceleration (CTRA filter) |
| `yaw_rate` | rad/s | |
| `length`, `width`, `height` | m | |
| `sigma.pos/speed/accel` | m, m/s, m/s² | 1-σ uncertainty from the filter |
| `lane`, `approach`, `lane_s`, `lateral` | –, –, m, m | lane assignment and position along/across it |
| `dist_to_stop` | m | front bumper to stop line (positive before the line) |
| `decel_required` | m/s² | constant deceleration needed to stop at the line (v²/2d); above about 3.5 a comfortable stop is no longer possible |
| `eta` | s | time to reach the stop line, using speed and acceleration |
| `eta_cv` | s | the same at constant speed |
| `eta_sigma` | s | 1-σ uncertainty of `eta` |
| `stopping` | bool | current deceleration stops the vehicle before the line |
| `queued` | bool | part of the queue at the stop line |
| `occluded` | bool | currently predicted behind a known occluder |
| `confidence` | 0..1 | |

### Near-miss event (`events/safety.event`)

| Field | Meaning |
|---|---|
| `kind` | `rear_end`, `crossing`, `head_on`, `vru_vehicle` or `hard_braking` |
| `severity` | `critical` or `warning` |
| `metrics.ttc` | minimum time-to-collision of the oriented footprints at constant velocity (s) |
| `metrics.mttc` | the same with acceleration and turn rate (CTRA) (s) |
| `metrics.pet` | post-encroachment time on a 0.5 m occupancy grid, crossing paths only (s) |
| `metrics.drac` | deceleration rate to avoid the crash (m/s²) |
| `metrics.min_accel` | for hard braking (m/s²) |
| `x`, `y`, `lat`, `lon` | conflict point |

The default thresholds are below. They are configurable in `NearMissConfig`.

| Metric | Critical | Warning |
|---|---|---|
| TTC / MTTC | < 1.5 s | < 3 s |
| PET | < 1 s | < 2 s |
| DRAC | > 3.35 m/s² | – |
| Hard braking | < −5 m/s² | < −3 m/s² |

### Health (`health.health`)

`level` is `ok`, `degraded` or `fault`. `confidence` is a 0..1 product of the component scores, and
`reasons` holds human-readable causes. The components are:

- `stability`: `stable`, `shaking`, `drifted` (compensated), `moved` (recalibration required) or `lost`, plus `jitter_px` and `drift_deg`.
- `visibility`: score, conditions (`night`, `fog`, `glare`, `low_contrast`, `blur`, `lens_obstruction`, `frozen`), effective detection range vs. baseline, and track fragmentation.
- `calibration`: `ok`, `suspect`, `invalid` or `unknown`. It is based on the 2D/3D size consistency, the range-dependent bias, the track-vs-lane heading error and the speed plausibility.
- `coverage`: per lane and crosswalk, the share that is visible, hidden behind vehicles, statically occluded, or unobservable.

## Signal controller integration

The `detector/<zone>` topics emulate loop detectors:

- one per lane at the stop bar (`<lane>_stopbar`, 20 m)
- one advance detector (`<lane>_advance`, 50–60 m upstream)

Any controller gateway can map them to detector inputs:

- digital I/O
- NTCIP 1202 detector objects
- OCIT-O detector channels

The richer `objects` stream carries ETA and queue information for adaptive or priority control. The
`cpm` messages carry the same perception to V2X roadside units.
