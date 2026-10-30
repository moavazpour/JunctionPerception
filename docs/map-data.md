# Map data

Custom scenes are placed on a map. `tools/prepare_map_data.py` downloads the map layers for a
scene once and converts them into the scene's world frame; the server serves them from
`map_cache/<scene_id>/` (not part of the repository).

```bash
python tools/prepare_map_data.py --scene scenes/munich_5538
```

The script reads `geo_anchor` and `map.radius_m` from `scene.yaml` and writes:

| File             | Content                                                                  |
| ---------------- | ------------------------------------------------------------------------ |
| `ortho.jpg/json` | Orthophoto at 0.2 m/px and the world positions of its four corners       |
| `osm.json`       | Roads, footways and crossings as polylines in metres, with lane widths   |
| `buildings.json` | Building footprints and heights                                          |
| `map.json`       | Manifest: radius, layer files and attribution                            |

## World frame

The world frame of a custom scene is a local east-north-up frame with its origin at the scene's
geo anchor (x east, y north, metres). Geographic positions are converted with PROJ (WGS84
ellipsoid, geocentric to topocentric), in Python and in C++ alike. ETRS89 coordinates of the
Bavarian datasets are treated as WGS84; the two frames differ by well under a metre in Europe,
below the accuracy of the camera calibration.

## Sources and licenses

| Layer      | Source                                                                       | License                                                             |
| ---------- | ---------------------------------------------------------------------------- | ------------------------------------------------------------------- |
| Orthophoto | Bayerische Vermessungsverwaltung, DOP20 web map service (`by_dop20c`)        | [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/)           |
| Buildings  | Bayerische Vermessungsverwaltung, LoD2 building models (CityGML, 2 km tiles) | [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/)           |
| Roads      | OpenStreetMap, official editing API (`/api/0.6/map`)                         | [ODbL](https://opendatacommons.org/licenses/odbl/) © OpenStreetMap contributors |

The twin and the calibration page show the attribution of every layer in use. Downloaded data
stays in `map_cache/` and is never committed.

## Calibration and zones

`/calibrate.html` places the camera on this map. Corresponding points are picked in a camera
frame and on the orthophoto; the server fits an image-to-ground homography with RANSAC and
reports the error of every point in metres. Every fourth point is held out as a check point,
so the reported check error measures the calibration on points the fit has not seen. The camera
position and height follow from the homography; they can be set by hand when the points do not
constrain them. Saving writes `control_points.yaml` and `ground_homography.yaml` to the scene.

Zones are drawn on the same map; crossings from OpenStreetMap can be added as starting
suggestions. They are stored in `zones.geojson` as WGS84 coordinates, so they stay valid when
the calibration changes.
