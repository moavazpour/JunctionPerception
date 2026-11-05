"""Benchmark calibration of a TUMTraf camera in the world frame of a geo-referenced site.

jp_eval reads the labels through the benchmark scene's `sensor_from_ground` (south LiDAR ->
ground plane fitted to the labels). To evaluate a run made with the site's own calibration
(control points on the map), the labels have to be in the site's east-north-up frame instead.
TUMTraf publishes no geographic position of its frames, so the rigid transform ground -> site
(rotation about the vertical and a shift, no scale) is fitted to the site camera's control points:
each point's pixel goes through the dataset's homography onto the label ground, and its latitude
and longitude through the site's geo anchor into the site frame. Neither the labels nor any
detection enter the fit; its residuals combine the errors of the points, of the map and of the
dataset's camera calibration.

Writes <out>/ground_homography.yaml: the benchmark calibration with every ground-frame quantity
moved into the site frame, plus a `site_alignment` block with the fit.

Example:
    python3 tools/tumtraf_site_frame.py --benchmark scenes/tumtraf_s110_south2 \
        --site scenes/tumtraf_s110_south2_s03 --camera south2 --out outputs/eval_scenes/south2
"""

import argparse
import math
import pathlib

import numpy as np
import yaml
from pyproj import Transformer


def enu_transformer(anchor: dict) -> Transformer:
    """Geographic (lon, lat degrees) -> east-north-up at the anchor, as jp::core::GeoAnchor."""
    return Transformer.from_pipeline(
        "+proj=pipeline +step +proj=unitconvert +xy_in=deg +xy_out=rad "
        "+step +proj=cart +ellps=WGS84 "
        f"+step +proj=topocentric +ellps=WGS84 +lat_0={anchor['lat_deg']:.12f} "
        f"+lon_0={anchor['lon_deg']:.12f} +h_0={anchor.get('alt_m', 0.0):.4f}"
    )


def fit_rigid(source: np.ndarray, target: np.ndarray) -> tuple[float, np.ndarray]:
    """Yaw and shift with target ~ R(yaw) @ source + shift (least squares, no scale)."""
    cs, ct = source.mean(axis=0), target.mean(axis=0)
    s, t = source - cs, target - ct
    yaw = math.atan2(
        float(np.sum(s[:, 0] * t[:, 1] - s[:, 1] * t[:, 0])),
        float(np.sum(s[:, 0] * t[:, 0] + s[:, 1] * t[:, 1])),
    )
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    return yaw, ct - rotation @ cs


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--benchmark", type=pathlib.Path, required=True, help="benchmark scene")
    parser.add_argument("--site", type=pathlib.Path, required=True, help="site scene folder")
    parser.add_argument("--camera", required=True, help="site camera with the control points")
    parser.add_argument("--out", type=pathlib.Path, required=True)
    args = parser.parse_args()

    calibration = yaml.safe_load((args.benchmark / "ground_homography.yaml").read_text())
    anchor = yaml.safe_load((args.site / "scene.yaml").read_text())["geo_anchor"]
    points = yaml.safe_load(
        (args.site / "cameras" / args.camera / "control_points.yaml").read_text()
    )["points"]

    image_to_ground = np.array(calibration["image_to_ground"], dtype=float)
    pixels = np.array([[p["u"], p["v"], 1.0] for p in points])
    projected = (image_to_ground @ pixels.T).T
    ground = projected[:, :2] / projected[:, 2:3]
    to_enu = enu_transformer(anchor)
    heights = [anchor.get("alt_m", 0.0)] * len(points)
    east, north, _ = to_enu.transform(
        [p["lon"] for p in points], [p["lat"] for p in points], heights
    )
    site = np.column_stack([east, north])

    yaw, shift = fit_rigid(ground, site)
    c, s = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[c, -s], [s, c]])
    residuals = np.linalg.norm((rotation @ ground.T).T + shift - site, axis=1)

    # site <- ground, as a 4x4 (3D, vertical unchanged) and a 3x3 (homogeneous plane) transform.
    site_from_ground = np.array(
        [[c, -s, 0, shift[0]], [s, c, 0, shift[1]], [0, 0, 1, 0], [0, 0, 0, 1]]
    )
    plane = np.array([[c, -s, shift[0]], [s, c, shift[1]], [0, 0, 1]])
    ground_from_site = np.linalg.inv(site_from_ground)

    def moved(key: str, transform: np.ndarray) -> list:
        return (np.array(calibration[key], dtype=float) @ transform).tolist()

    out = dict(calibration)
    out["image_to_ground"] = (plane @ image_to_ground).tolist()
    out["ground_to_image"] = moved("ground_to_image", np.linalg.inv(plane))
    out["ground_projection"] = moved("ground_projection", ground_from_site)
    out["sensor_from_ground"] = moved("sensor_from_ground", ground_from_site)
    camera = np.array(calibration["camera_ground_position_m"], dtype=float)
    out["camera_ground_position_m"] = (rotation @ camera + shift).tolist()
    out["site_alignment"] = {
        "site": str(args.site),
        "camera": args.camera,
        "yaw_deg": math.degrees(yaw),
        "x_m": float(shift[0]),
        "y_m": float(shift[1]),
        "points": len(points),
        "rms_m": float(math.sqrt(np.mean(residuals**2))),
        "max_m": float(residuals.max()),
    }
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "ground_homography.yaml").write_text(yaml.safe_dump(out, sort_keys=False))
    for p, r in zip(points, residuals):
        print(f"{p['id']:>4} {p['role']:>5}  residual {r:.2f} m")
    print(
        f"yaw {math.degrees(yaw):.2f} deg, shift ({shift[0]:.2f}, {shift[1]:.2f}) m, "
        f"rms {out['site_alignment']['rms_m']:.2f} m over {len(points)} points"
    )


if __name__ == "__main__":
    main()
