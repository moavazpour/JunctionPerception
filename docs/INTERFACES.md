# Interfaces

JunctionPerception is one module per camera. It takes a video stream plus a small YAML site
configuration and emits metadata only. Images never leave the device, with one exception: a pixelated
calibration snapshot, which is optional.

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

### Operator commands (`calibration/manual`, subscribed)

The web panel publishes JSON commands to `<prefix>/<site>/<camera>/calibration/manual`:

| `action` | Fields | Effect |
|---|---|---|
| `solve` | `points: [{u, v, lat, lon}, …]` (≥ 4; frame pixels) | manual calibration from ground correspondences: focal length (unless configured), pose and camera position are solved and the result replaces the current calibration (persisted like an auto-calibration) |
| `auto` | – | discard the calibration and restart auto-calibration |
| `snapshot` | – | publish a fresh calibration snapshot |

Every command is answered on `calibration/manual/result` with `ok`, `message` and, for `solve`, the
reprojection error (`rms_px`, `max_px`). Enable MQTT authentication before exposing this channel.

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
