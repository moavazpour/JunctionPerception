# Why the 3D twin looks less accurate than the BEV

Measurement of November 2026. No part of the 3D twin or the BEV was changed: the existing
`TwinView` of the production build (`cb0e0bf`) was driven with recorded world states on a
simulated clock and its drawn positions were read back. Scripts: `tools/fidelity/`.

## Result

Both views place every road user at the **same ground position** when the 3D twin's own additions
are switched off (difference 0.000 m). The perceived difference comes from what the 3D view adds
on top, in this order of impact:

| # | Effect | Share of drawn road users | Discrepancy (m), mean / median / P90 | Kind |
|---|---|---|---|---|
| 1 | Perspective and model height (tilted view) | 100 % | vehicle body 1.60 / 1.38 / 3.04, roof 3.25 / 2.80 / 6.25 | visual only, ground error 0 |
| 2 | Asynchronous multi-camera fusion | 7–8 % (camera overlap) | 0.96 / 0.99 / 1.48 from the camera's own track | ground position |
| 3 | Extrapolation of lost tracks | 6 % | 2.37 / 1.16 / 6.23 from the last measured position | ground position |
| 4 | Interpolation and rendering delay | 100 %, matters while moving | moving: 0.27 / 0.11 / 0.76 | ground position |
| 5 | Wrong or fallback heading | vehicles | footprint corners 0.41 / 0.16 / 0.45 | footprint, not centre |

The largest effect is not an error of the ground position at all: in the tilted 3D view, the body
of a vehicle appears 1.4 m (median) and its roof 2.8 m away from the point where it stands, which
is 2.5 and 5 times the backend's own localization error (0.55 m median). Fusion and lost-track
extrapolation do move the drawn ground position, but only for 7–8 % and 6 % of the road users.
The heading logic of the 3D view does not make it less accurate on this data.

## Setup

- **Data:** TUMTraf Intersection R02, sequence s03 (day), cameras south1 and south2, 1033 frames each
  (about 120 s at 8.6 fps), run again with the production pipeline (YOLO26l at 1280, box-fit
  contact model) and with the labels written by the same run (`jp_run --reference-csv`), so world
  states and ground truth share registration and frame indices.
- **One frame, one clock:** south1 converted into the south2 ground frame (the two differ by a few
  centimetres); time is the capture time from the image file names (south1 starts 26.5 ms earlier).
- **Ground truth matching:** per frame, greedy nearest within 2 m for vehicles and 1 m for VRUs, as
  in `jp_eval`; 37 404 samples of the single-camera run are matched. Unmatched objects (unlabelled or
  false detections) enter the discrepancy measures but not the ground-truth ones.
- **Display:** world states arrive 200 ms after capture (the production servers report 130–250 ms),
  the twin is rendered at 60 Hz of simulated time, every rendered frame is measured. The traffic
  direction the twin learns in the browser was taught by one pass over the sequence first (as after
  some time of live use), the map is the OSM layer of the S110 site placed by its transform.
- **Scenarios:** one camera (south2); two cameras in step; two cameras with south1 300 ms late
  (the median offset measured live); one camera with an empty flow field (a new browser).
- **Perspective:** ten snapshots of the default view of the site (the view the twin flies to),
  canvas 776 × 439 CSS px, in the tilted (40°) and the top-down mode.

## 0. Baseline: identical timestamps, no fusion, no interpolation

One camera, frames where the interpolation has reached the latest state (t = 1):

| | n | mean / median / P90 (m) |
|---|---|---|
| 3D drawn position − backend position | 33 466 | 0.000 / 0.000 / 0.000 |
| Backend (= BEV) position − ground truth at capture | 37 404 | 0.68 / 0.55 / 1.50 |

The BEV draws the backend position by construction. With equal inputs both views therefore have
the same ground position and the same error against ground truth; everything below is added by
the 3D view.

## 1. Perspective projection and model height

The mesh is placed exactly at the ground position: projecting it to the screen and back onto the
ground plane returns it with 0.000 m error. What moves is what the eye reads as the road user —
the body or the roof of a 1.5–3.4 m tall model, seen from an angle. Screen displacement from the
ground point, and where that visual point lies on the ground (the apparent position):

| View | Road users | n | Body centre: px | Body centre: apparent offset (m) | Roof: px | Roof: apparent offset (m) |
|---|---|---|---|---|---|---|
| tilted 3D (40°) | vehicles | 647 | 3.6 / 3.2 / 6.3 | 1.60 / 1.38 / 3.04 | 7.2 / 6.5 / 12.6 | 3.25 / 2.80 / 6.25 |
| tilted 3D (40°) | VRUs | 128 | 3.3 / 3.3 / 3.4 | 1.45 / 1.44 / 1.51 | 6.6 / 6.5 / 6.7 | 2.89 / 2.87 / 3.05 |
| top-down "2D" | vehicles | 313 | 1.3 / 1.3 / 1.9 | 0.24 / 0.24 / 0.34 | 2.7 / 2.6 / 3.7 | 0.49 / 0.48 / 0.68 |
| top-down "2D" | VRUs | 98 | 1.7 / 1.6 / 2.1 | 0.30 / 0.29 / 0.38 | 3.3 / 3.1 / 4.2 | 0.60 / 0.57 / 0.76 |

At this zoom one pixel is 0.22 m (tilted) or 0.18 m (top-down) of ground; pixel values grow with
the zoom, metre values do not. The offset grows with the height of the model (trucks and buses
reach the P90) and away from the image centre; the top-down mode keeps a residual parallax
because it is a perspective camera (36° field of view), not an orthographic one. The BEV has
none: it draws the footprint on the ground.

## 2. Heading

Vehicles matched to ground truth, one camera. Heading error against the labelled yaw in degrees;
"axis" ignores front and back; corner shift is how far the drawn footprint corners move because
of the heading alone (same centre and size).

| Vehicles | n | 3D axis error | BEV axis error | front/back flips 3D | flips BEV | corner shift 3D (m) |
|---|---|---|---|---|---|---|
| with a backend heading | 28 551 | 6.85 / 3.79 / 9.66 | 5.36 / 3.44 / 9.29 | 0.1 % | 2.9 % | 0.41 / 0.16 / 0.45 |
| without a backend heading | 3 053 | 2.40 / 1.47 / 2.16 | no heading drawn | 0.8 % | — | 0.10 / 0.06 / 0.09 |
| with, empty flow field | 28 551 | 5.93 / 3.79 / 11.06 | 5.36 / 3.44 / 9.29 | 3.4 % | 2.9 % | 0.31 / 0.16 / 0.51 |
| without, empty flow field | 3 053 | 3.19 / 1.52 / 2.29 | — | 1.3 % | — | 0.14 / 0.06 / 0.10 |

- The drawn heading follows the backend's heading with a median difference of 0° (P90 0.13°); its
  turn-rate limit lags in turns and, as a side effect, suppresses most of the backend's front/back
  flips (2.9 % → 0.1 %).
- Vehicles without a backend heading (about 10 %) take the direction of the OSM road and its lane
  side; on this sequence that direction is right within 1.5° (median). The fallback to 0° never
  occurred (0 of 3 053), because the S110 map has roads. On a site without map and before the flow
  field has learned, it would.
- The heading does not move the centre of a footprint; its corners move 0.16 m (median) — below
  every other effect.

## 3. Interpolation and rendering delay

One camera, every rendered frame (n = 106 585). The 3D view interpolates between the last two
states over the time the latest one took to arrive, so it runs about one state behind.

| | mean / median / P90 (m) |
|---|---|
| 3D drawn − latest backend, all | 0.11 / 0.004 / 0.31 |
| moving road users (> 1 m/s, n = 39 222) | 0.27 / 0.11 / 0.76 |
| standing road users (n = 67 363) | 0.01 / 0.002 / 0.03 |

Against ground truth, both views also carry the 200 ms latency of the data; at the time of
display:

| Ground truth at | Road users | n | 3D | BEV |
|---|---|---|---|---|
| display time | all | 36 623 | 1.02 / 0.60 / 2.56 | 0.95 / 0.59 / 2.26 |
| display time | moving | 10 949 | 2.00 / 1.82 / 3.91 | 1.76 / 1.57 / 3.44 |
| capture time of the shown state | all | 37 404 | 0.73 / 0.57 / 1.64 | 0.68 / 0.55 / 1.50 |

Interpolation adds about 0.25 m (median) for moving road users, P90 0.5 m. Most of the error of a
moving road user at display time is latency, which the BEV shares.

## 4. Asynchronous multi-camera fusion

Road users seen by both cameras: the 3D view averages the cameras' interpolated positions,
weighted by confidence, at the time messages arrive. Discrepancy against the drawn camera's own
interpolated track, and errors against ground truth at display time:

| Scenario | Share of drawn | Camera offset (ms) | Fused − own track (m) | GT error fused | GT error single camera (n) |
|---|---|---|---|---|---|
| cameras in step | 7.3 % | 45 / 29 / 109 | 0.96 / 0.99 / 1.48 | 1.76 / 1.30 / 3.54 | 1.93 / 1.83 / 3.18 (1 019) |
| south1 300 ms late | 8.4 % | 297 / 300 / 402 | 0.96 / 0.93 / 1.55 | 2.04 / 1.32 / 4.26 | 2.16 / 1.84 / 3.74 (1 107) |

Fusion moves the drawn position by about 1 m from what each camera reports (and what the BEV
shows per camera). Averaging two independent errors lowers the median error against ground truth,
but the tail grows (P90 +0.36 m in step, +0.52 m with a 300 ms offset): pairs that are merged
across a time offset or wrongly are averaged into a position neither camera saw. The overlap
region is far from both cameras, so errors there are larger than the site average.

## 5. Extrapolation of lost tracks

A track that ends keeps moving with its last velocity while it fades out (0.9 s):

| | n | mean / median / P90 |
|---|---|---|
| share of drawn road users | 6 799 | 6.0 % |
| time since lost (ms) | | 395 / 367 / 767 |
| drawn − last backend position (m) | | 2.37 / 1.16 / 6.23 |
| last speed (m/s) | | 5.8 / 4.3 / 12.6 |

The BEV draws nothing after a track ends. A comparison of these positions with ground truth is
not reported: for tracks that are being lost, the association to a label was not reliable
(implausible distances of more than 30 m), so the number would not mean anything.

## Correction plan

| Priority | Change | Effort | Effect expected (from the measurements) |
|---|---|---|---|
| 1 | Make the ground position the visible anchor: a flat footprint on the ground under every model (always drawn, also when the model is hidden), models translucent or lower in analysis mode, hover and selection on the footprint. | low | removes the 1.4 m (body) / 2.8 m (roof) apparent offset as the thing the eye reads |
| 2 | An orthographic camera for the top-down mode. | low | 0.24–0.48 m residual parallax → 0 |
| 3 | No averaging across cameras in the browser: draw each camera's track, or the most confident one without averaging; merge only within 100 ms and a gate that grows with speed × offset; long term, fusion with covariances in a backend world model. | low (browser) / high (backend) | about 1 m on 7–8 % of road users, the P90 tail of fusion |
| 4 | Lost tracks: freeze at the last measured position while fading, or drop them; at most 0.3 s of extrapolation and only with a speed σ below 1 m/s. | low | 1.2 m median, 6.2 m P90 on 6 % |
| 5 | A "faithful" setting that draws the latest state (interpolation by capture timestamps instead of arrival, no frame of lag). | low–medium | 0.11 m median, 0.76 m P90 while moving |
| 6 | Mark the heading's origin in 3D as in the BEV, and replace the `?? 0` fallback in the interpolator with "unknown". | low | no change on this data; prevents 0° headings on sites without map |
| 7 | Keep `tools/fidelity` as a regression check, for example drawn − backend P90 < 0.1 m in the faithful setting. | low | keeps the 3D view honest |

## Limits of this measurement

- One sequence (s03, daylight, straight approaches); night and turning traffic not covered.
- Constant 200 ms latency without jitter; the real arrival times vary.
- The flow field was learned on the same sequence it is tested on (optimistic for the 3D heading).
- Perspective numbers come from the default view of the site at one zoom; a closer, flatter view
  increases the apparent offset.
- The ground-truth matching is greedy with fixed gates and ignores unmatched objects; fusion
  ground-truth figures rest on about 1 000 samples.
