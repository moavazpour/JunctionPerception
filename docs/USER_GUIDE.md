# User guide: the web demo

The demo runs the full system on a server (GPU, or CPU only). Everything is done in the browser: add a
camera (a video file or a live stream), place and calibrate it once by matching the camera image with
the aerial map, and watch every road user in the 3D digital twin with its class, speed and acceleration.

## 1. Open the panel

Open `http://<server>:8080` (ask the operator for the address and, if set, the user name and password).

The header shows the site (a drop-down when there are several), **LIVE** for a camera stream or
**REPLAY** for a recorded video (played in a loop so the twin keeps showing traffic), whether the
networks run on the **GPU** (TensorRT) or the **CPU** (ONNX Runtime), the frame rate and latency, the
light/dark/system theme switch and **Settings**.

## 2. The live view

- **3D twin** (left): the road map is drawn from OpenStreetMap in the in-car style (roads, lane markings,
  crossings, traffic lights; **Map**). **Aerial** shows satellite imagery instead. **Chase / Top /
  Camera** change the viewpoint; drag to rotate, scroll to zoom. Lanes drawn during calibration appear
  in their own colours.
- **Camera view** (corner): the live video with the tracked 3D boxes, class colour and speed. Hover it
  for size (S/M/L), corner, full screen and hide.
- **Right column**: how many road users there are by class, the average and maximum speed, queues per
  lane, and every road user with speed (km/h), acceleration (m/s²), lane and distance/time to the stop
  line. Click one (in the list or in the 3D view) for its card: class, size, speed and acceleration
  with uncertainty, heading, lane, distance to the stop line, arrival time and the deceleration it would
  need to stop there (flagged above 3.5 m/s²).

## 3. Settings

| Section | What it does |
|---|---|
| **Calibration** | places the camera in the world (below) |
| **Video source** | shows the source; upload another video or switch to an RTSP stream; loop on/off; camera position, height and lens |
| **Detection** | model thresholds (confidence, keypoint confidence), maximum range, processing rate, reported classes, road-area filter, boxes in the camera view. Applied to the running site within a frame and kept across restarts |
| **Compute** | GPU or CPU, device, model, latency; run the site on Auto / GPU / CPU |
| **Display** | theme, base map, labels, camera view (this browser only) |
| **Sites** | all cameras on the server: open, start, stop, delete, **New site** |
| **System** | self-monitoring (stability, visibility, calibration check) and the runtime log |

### New site

**Settings → Sites → New site**: a name; a **video file** (MP4, MOV, MKV, AVI from a fixed camera) or a
**live stream** (`rtsp://user:password@192.168.1.20/stream1`); a rough **camera height**; the **lens**
(phone main camera, 2×, 3×, traffic camera, unknown); **Compute** (Auto uses the GPU when there is
one); then search the address and click **where the camera is** and **a point it looks at**. **Create
and start** opens the calibration. The first GPU start builds the TensorRT engine (about a minute).

## 4. Calibrate the camera (once per site)

**Settings → Calibration** has four steps. The camera may be anywhere, at any height and zoom: focal
length, tilt, roll, height and the exact camera position are all solved from your input.

1. **Camera**: lens and a rough height with how sure you are (e.g. 7 ± 1.5 m). These only rule out
   impossible solutions (an unconstrained solver can explain four points with an absurd telephoto lens).
2. **Road layout** (on the aerial map): draw each **lane** by clicking along its middle in the driving
   direction, through the stop line (double-click to finish, set the width); the **stop lines** (both
   ends); optionally **crosswalks** and the **road area** (objects outside it, e.g. parked cars, are
   ignored). **Save layout**. Lanes that cross a stop line become inbound lanes with queues, distance to
   the stop line and arrival times; without drawn lanes the system learns lanes from traffic.
3. **Match image and map** (camera image left, map right; wheel zooms, drag pans, the magnifier helps
   to click precisely, markers can be dragged to correct them):
   - **Point pair**: a sharp spot on the road surface in the image, then the same spot on the map.
     6–8 pairs, spread near and far, left and right (corners of dashed markings, stop-line ends, kerb
     corners). Never points on vehicles, poles or roofs.
   - **Line** (recommended, 2–3): click two or more points along a long straight marking in the image
     (lane line, kerb), then two points of the same marking on the map, anywhere along it. Lines pin
     down the tilt and zoom better than distant points.
   - **Vertical** (optional): the bottom and top of a pole or building edge; fixes the roll.
   - **Solve** shows the quality (good / fair / poor), how well your points fit on the ground (metres)
     and in the image (pixels), field of view, height, tilt and direction, and hints such as *point 3 is
     off by 2.4 m* or *all points are at a similar distance*. Errors are judged against what clicking can
     achieve at that distance: far away one image pixel covers many centimetres, so a far point may be
     off by a metre and still be green. Amber is larger than expected, red means the two clicks are
     probably not the same spot. **Apply calibration** activates it (a poor fit asks for confirmation).
   - The lens setting is a lower bound on the zoom: phones zoom digitally on top of the lens, so a
     narrower view than the setting is accepted, a wider one is not.
4. **Check**: the drawn lanes, stop lines and a 5 m grid are drawn into the camera image, and the
   camera image is projected onto the map. When they sit on the road, the calibration is good. Where
   they do not, **drag the white dots** (vertices of the drawn lanes, stop lines, crosswalks and road
   area) in the image to where that point really is: each drag pairs the map position of that vertex
   with the pixel, the camera is solved again within a second, and the image on the map moves with it.
   **Reset** drops the dragged pairs; **Apply calibration** activates the result. The aerial image itself
   can be a few decimetres off, so a small offset is normal. The **expected position accuracy** lists the
   error from the calibration at 10–200 m.

**Traffic check.** After applying, the system watches the traffic: vehicles on a straight road drive
parallel to each other and to the drawn lanes. If a slightly wrong tilt, roll or zoom makes their paths
fan out on the map, it proposes a small correction with before/after numbers. Nothing changes until
you click **Apply correction**.

The inputs are stored on the device; you can come back later, change a point and solve again. **Clear**
discards the calibration.

## 5. The digital twin

After calibration every road user is placed at its measured position with its heading, speed and
acceleration (from the tracker, in metres on the ground). With drawn lanes and stop lines the twin
reports per lane the users, queue length, mean speed and stop-line crossings, and per road user the
distance to the stop line, the arrival time (with its acceleration) and whether it can still stop
comfortably. Near-miss events (TTC, PET, DRAC, hard braking) are issued only with a trustworthy
calibration.

## 6. Good videos

- Fixed camera, high up (5–15 m), looking along or across the road; avoid zooming or panning.
- 1080p or more, at least 15 frames per second.
- The road surface and its markings should be visible for the calibration.
- Very distant vehicles that touch each other in the image can be merged into one detection; a camera
  closer to the area of interest gives better results.

## 7. For the operator

```bash
# models: detector ONNX + segmentation ONNX in $JP_MODELS; runtime image built once
docker build -t junction-perception:latest -f docker/Dockerfile .
JP_DATA=/abs/path/demo JP_MODELS=/abs/path/models docker compose -f docker-compose.demo.yml up -d --build
tools/set_password.sh /abs/path/demo <user>            # optional login; then restart the web service
```

`JP_PORT` changes the port (default 8080), `JP_MAX_SITES` the number of simultaneous sites (default 4).
Uploaded videos, site settings and calibrations live under `JP_DATA` (`videos/`, `sites/`, `state/`).
Before exposing the panel to the internet, set a password and put it behind HTTPS (e.g. a reverse proxy
or tunnel with TLS): the panel can create sites and run commands on the server.

**CPU-only servers.** The runtime image also contains the ONNX Runtime CPU backend; without a GPU the
manager starts sites on the CPU automatically (or build `docker/Dockerfile.cpu`, which needs no CUDA:
`docker build -t junction-perception:cpu -f docker/Dockerfile.cpu .` and set `JP_IMAGE_CPU`). Expect a
few frames per second with the 960 px detector.
