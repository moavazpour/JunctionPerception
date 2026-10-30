# Architecture

## Data flow

```mermaid
flowchart LR
    subgraph Sources
        V[Video file / image sequence] --> D[Detector<br/>YOLO26, ONNX Runtime]
        D --> R[Rider association]
        R --> T[ByteTrack]
        T --> L[Localization<br/>homography, box fit,<br/>BEV Kalman, heading]
        O[Oracle<br/>benchmark labels]
        P[Replay<br/>JSONL]
    end
    L --> E[Zones and VRU events]
    O --> E
    P --> E
    E --> W[World state]
    W --> WS[WebSocket /ws/world]
    W --> J[JSONL]
    V -. camera images .-> VS[WebSocket /ws/video]
    WS --> UI[Web twin]
    VS --> UI
```

All three sources (`live`, `oracle`, `replay`) produce the same world objects, which pass through
the same zone and event code before they are published.

## Modules

| Module | Responsibility |
|---|---|
| `core` | Shared types (world state), configuration, class catalog, assignment, statistics |
| `io` | Frame sources: video files (presentation timestamps kept), image sequences |
| `benchmark` | TUMTraf Intersection R02 reader, label registration and merging |
| `calibration` | Ground plane, image-to-ground homography, camera model, benchmark calibration |
| `detection` | `Detector` interface, YOLO26 end-to-end ONNX Runtime backend, rider association |
| `tracking` | ByteTrack in image space |
| `localization` | Ground contact, contact-to-centre correction (box fit), BEV Kalman filter, heading |
| `vru` | Zones (GeoJSON), crosswalk occupancy, VRU-vehicle conflicts, turning flag |
| `sources` | World sources: live pipeline, oracle (benchmark labels), replay (JSONL) |
| `twin` | World-state JSON and JSONL, debug overlay and top-down videos |
| `server` | HTTP API, WebSockets, real-time playback, static frontend |
| `apps` | `jp_run`, `jp_server`, `jp_calibrate`, `jp_eval` command-line programs |

## Coordinate frames

| Frame | Unit | Description |
|---|---|---|
| `image` | px | Camera image. Custom videos are assumed free of lens distortion. |
| `scene_ground` | m | Metric plane of a scene, z = 0 on the ground. For benchmark scenes it is the ground plane fitted to the labelled boxes in the camera's view, defined in the south LiDAR frame (`sensor_from_ground` in `ground_homography.yaml`). |
| `world` | m | Local ENU tangent plane at the scene's geo anchor. Until a scene has a geo anchor, `world` equals `scene_ground`. |
| `wgs84` | deg | Latitude / longitude, for control points, zones and the geo anchor. |
| `benchmark` | m | TUMTraf label frame (south LiDAR). |

Zones (`scenes/<id>/zones.geojson`) declare their frame in the foreign member
`coordinate_frame`; `scene_ground` is supported, WGS84 zones require the scene's geo anchor.

## World state (schema version 1)

One JSON object per frame, sent on `/ws/world` and written as one line per frame in JSONL files.
Example: [`tests/fixtures/world_state_v1.json`](../tests/fixtures/world_state_v1.json); the C++
serializer and the frontend types are both tested against it.

| Field | Type | Description |
|---|---|---|
| `schema_version` | int | `1` |
| `scene_id` | string | Scene folder name |
| `frame` | int | Frame index of the source |
| `timestamp_ns` | int | Capture time (ns); relative to the first frame for video files |
| `source` | string | `live`, `oracle` or `replay` |
| `objects` | array | Road users, see below |
| `events` | array | Active events, and events in the frame they end |

Object fields: `id` (track ID), `class` (twin class), `vru`, `rider` (two-wheeler with rider),
`x`, `y` (footprint centre, m, world frame), `vx`, `vy` (m/s), `heading_rad` (counter-clockwise
from +x, `null` until the object has moved), `speed_mps`, `confidence`, `zones` (IDs of the zones
containing the object), `turning` (vehicle heading changed by more than the configured angle over
the configured window).

Event fields: `id`, `type` (`VRU_ON_CROSSWALK`, `VRU_VEHICLE_CONFLICT`), `state` (`active` or
`ended`), `object_ids` (conflicts: vehicle, then VRU), `min_distance_m` and `time_to_closest_s`
(conflicts: closest approach predicted at constant velocity), `zone` (crosswalk, or the VRU's
zone for conflicts; `null` otherwise).

## Server interface

| Endpoint | Description |
|---|---|
| `GET /api/scene` | Scene description: camera, image size, camera ground position, zones, class catalog |
| `GET /api/status` | Playback state, frame, frame rate, processing latency, connected clients |
| `POST /api/playback` | `{"action": "play"}`, `{"action": "pause"}`, `{"action": "seek", "frame": n}` |
| `WS /ws/world` | World state per frame (text) |
| `WS /ws/video` | Camera images (binary): 8-byte little-endian frame index followed by a JPEG, rate-limited |
| `GET /` | Built frontend (`web/dist`) |

Recordings are played in real time on their own timestamps (`configs/server.yaml`).
