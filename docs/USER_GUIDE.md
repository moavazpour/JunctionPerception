# User guide: the web demo

The demo runs the full system on a GPU server. Everything is done in the browser: add a camera (a video
file or a live stream), tell the system where the camera is, calibrate it once by clicking a few points,
and watch every road user in the 3D digital twin on top of the aerial image of the site.

## 1. Open the panel

Open `http://<server>:8080` (ask the operator for the address and, if set, the user name and password).
The header shows the site you are looking at, the clock, the **Sites** button and the connection state
(`live` when data is flowing).

## 2. Add a site

Click **Sites** → **New site**:

1. **Name**: any name, e.g. the street and direction.
2. **Source**:
   - **Video file**: a recording from a fixed camera (a phone on a tripod or held still works; MP4, MOV,
     MKV, AVI). The video is looped.
   - **Live stream**: the camera's address, e.g. `rtsp://user:password@192.168.1.20/stream1`
     (RTSP or HTTP; the server must be able to reach the camera).
3. **Camera height**: how high the camera is above the road, in metres (a footbridge is typically 6–8 m;
   a rough value is fine, the calibration refines it).
4. **Position**: search the address, then on the aerial map click **where the camera is**, then click
   **a point it looks at**. The orange line shows the viewing direction.
5. **Create and start**. The first start of the system builds its GPU engine (about a minute); after that
   a site starts in seconds. Up to four sites can run at the same time.

The panel switches to the new site. **Sites** lists all sites with **View**, **Start/Stop** and
**Delete** (deleting removes the uploaded video and the calibration too).

## 3. Calibrate the camera (once per site)

Until the camera is calibrated the panel shows *waiting for manual calibration*. Open the
**Calibration** tab and click **Full screen**: the camera view is on the left (road users are pixelated
for privacy), the aerial map on the right.

1. Click a point **on the road surface** in the camera view, then click **the same point** on the map.
   Good points are corners of lane markings, ends of the stop line, arrow tips, kerb corners, manhole
   covers: anything you can recognise in both.
2. Repeat for **at least 4 points, better 6–8**, spread out: near and far, left and right of the road.
   Points far away matter most for accuracy at distance. Do not use points on poles, roofs or vehicles.
3. A wrong pair can be removed with ✕. Click **Solve**.

The result line reports the focal length, tilt, height, the camera position and the **reprojection
error**. A good calibration has an error of a few pixels; if it is large, one pair is usually wrong:
remove it and solve again. The calibration is kept across restarts. **Clear calibration** discards it.

**New snapshot** fetches a fresh camera view (e.g. when the current one is full of vehicles hiding the
markings).

## 4. The digital twin

After calibration the 3D view shows the aerial image of the site with every road user as a 3D model at
its measured position, its heading and speed, the learned lanes, virtual detectors and occlusion.

- **Chase / Top / Camera** switch the viewpoint; drag to rotate, scroll to zoom. Click a road user for
  its details.
- **Road users**: class, speed (km/h), acceleration, lane, distance to the stop line, time to arrival.
- **Lanes**: learned lanes with counts, queues and virtual detectors (they appear after a few minutes of
  traffic).
- **Safety**: near-miss events (TTC, PET, DRAC, hard braking). They are only issued with a trustworthy
  calibration.
- **Coverage**: how much of each lane the camera can observe right now.
- The status card at the top shows the system confidence and why it is reduced (calibration, camera
  shake, weather and visibility, occlusion).

## 5. Good videos

- Fixed camera, high up (5–15 m), looking along or across the road; avoid zooming or panning.
- 1080p or more, at least 15 frames per second.
- The road surface and its markings should be visible for the calibration.
- Very distant vehicles that touch each other in the image can be merged into one detection; a camera
  closer to the area of interest gives better results.

## 6. For the operator

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
