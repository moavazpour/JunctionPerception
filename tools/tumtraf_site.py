"""Prepare TUMTraf Intersection (S110, R02 s03) as a calibrated two-camera site with ground truth.

Runs on the GPU server (numpy, opencv, scipy). Steps, each one a sub-command (`all` runs them in order):

    georef   georeference s110_base: bird's-eye view of both cameras (temporal median, so moving vehicles
             vanish) fitted with a 2D rigid transform against Google satellite tiles
    videos   constant-fps videos of both cameras plus a frame -> timestamp json per camera
    calib    CameraModel JSON per camera (K, dist, plane, bearing, anchor, ...)
    gt       groundtruth.json: per video frame, every labelled object in WGS84 with heading, size, speed
    check    project the ground truth through the CameraModel JSON and through the dataset's own
             LiDAR -> camera projection, and compare (pixels), with overlay images

    python3 tools/tumtraf_site.py all --out ~/JunctionPerception/data/demo/tumtraf_s110

Frames:
    lidar    s110_lidar_ouster_south, the frame the labels are in
    base     s110_base as defined by the dev-kit (`transformation_matrix_s110_lidar_ouster_south_to_s110_base`)
    enu      local east/north/up around the WGS84 point the base origin maps to
Cameras use the camera-LiDAR calibration the labels were made with (`projection_from_s110_lidar_ouster_south`).
The images are already rectified (the decomposed intrinsics are the rectified ones listed in the label files'
stream properties, close to the dev-kit's `optimal_intrinsic_camera_matrix`; straight edges are straight), so the
calibration has no distortion coefficients.
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import re
import urllib.parse
import urllib.request
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

HOME = Path.home()
SRC = HOME / "JunctionPerception/data/raw/TUMTraf/R2/a9_dataset_r02_s03/a9_dataset_r02_s03"
DEVKIT = HOME / "JunctionPerception/data/raw/TUMTraf/devkit/tum-traffic-dataset-dev-kit-main/calib"
OUT = HOME / "JunctionPerception/data/demo/tumtraf_s110"
VIDEOS = HOME / "JunctionPerception/data/demo/videos"
CAMS = {"south1": "s110_camera_basler_south1_8mm", "south2": "s110_camera_basler_south2_8mm"}
W, H = 1920, 1200

# Rough position of the S110 intersection (Schleissheimer Strasse, east end of Garching-Hochbrueck, found on the
# satellite imagery by its layout: four-way junction with a curved slip road in the north-east corner); the
# georeference search starts here.
APPROX_LATLON = (48.2494, 11.6308)

# Same mapping as training/jp3d/geometry.py (TUMTRAF_TO_CLASS), as runtime class names.
CLASS = {
    "CAR": "car", "VAN": "car", "TRUCK": "truck", "TRAILER": "truck", "BUS": "bus",
    "MOTORCYCLE": "motorcycle", "BICYCLE": "bicycle", "PEDESTRIAN": "pedestrian", "EMERGENCY_VEHICLE": "car",
    "OTHER": "other",
}

_TS = re.compile(r"(\d+)_(\d+)_")


def stamp(name: str) -> float:
    m = _TS.match(os.path.basename(name))
    return int(m.group(1)) + int(m.group(2)) * 1e-9


# ---------------------------------------------------------------------------------------------- geo

A_WGS, E2_WGS = 6378137.0, 6.69437999014e-3


def _radii(lat: float):
    s = math.sin(math.radians(lat))
    M = A_WGS * (1 - E2_WGS) / (1 - E2_WGS * s * s) ** 1.5
    N = A_WGS / math.sqrt(1 - E2_WGS * s * s)
    return M, N


def enu_to_geo(lat0, lon0, e, n):
    """Same local tangent-plane conversion as jp::enuToGeo (core/src/geometry/camera.cpp)."""
    M, N = _radii(lat0)
    return lat0 + np.degrees(np.asarray(n) / M), lon0 + np.degrees(np.asarray(e) / (N * math.cos(math.radians(lat0))))


def geo_to_enu(lat0, lon0, lat, lon):
    M, N = _radii(lat0)
    return (np.radians(np.asarray(lon) - lon0) * N * math.cos(math.radians(lat0)), np.radians(np.asarray(lat) - lat0) * M)


# ------------------------------------------------------------------------------------------ dataset

def load_cameras():
    """K and T_cam_from_base per camera, from the camera-LiDAR projection and the LiDAR -> base transform."""
    T_base_lidar = np.array(json.loads((DEVKIT / "s110_camera_basler_south2_8mm.json").read_text())
                            ["transformation_matrix_s110_lidar_ouster_south_to_s110_base"])
    cams = {}
    for key, name in CAMS.items():
        c = json.loads((DEVKIT / f"{name}.json").read_text())
        P = np.array(c["projection_from_s110_lidar_ouster_south"], float)
        K, R = cv2.decomposeProjectionMatrix(P)[:2]
        K = K / K[2, 2]
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = np.linalg.solve(K, P[:, 3])
        cams[key] = {"name": name, "K": K, "P_lidar": P, "T_cam_lidar": T, "T_cam_base": T @ np.linalg.inv(T_base_lidar)}
    return cams, T_base_lidar


def load_labels(T_base_lidar):
    """Label frames (south LiDAR) with objects in base: [{ts, objects: [{id, type, center, rot, dims}]}]."""
    frames = []
    for f in sorted(glob.glob(str(SRC / "labels_point_clouds/s110_lidar_ouster_south/*.json")), key=stamp):
        d = json.loads(Path(f).read_text())["openlabel"]
        for fr in d["frames"].values():
            objs = []
            for oid, o in fr["objects"].items():
                od = o["object_data"]
                v = od["cuboid"]["val"]
                if np.linalg.norm(v[3:7]) == 0:
                    continue
                R = T_base_lidar[:3, :3] @ Rotation.from_quat(v[3:7]).as_matrix()
                c = T_base_lidar[:3, :3] @ np.array(v[:3]) + T_base_lidar[:3, 3]
                objs.append({"id": oid, "type": od["type"].upper(), "center": c, "rot": R,
                             "dims": np.array(v[7:10], float), "lidar_center": np.array(v[:3], float),
                             "lidar_rot": Rotation.from_quat(v[3:7]).as_matrix()})
            frames.append({"ts": float(fr["frame_properties"]["timestamp"]), "file": os.path.basename(f), "objects": objs})
    return frames


def images(cam_name):
    files = sorted(glob.glob(str(SRC / "images" / cam_name / "*.jpg")), key=stamp)
    return files, np.array([stamp(f) for f in files])


def fit_plane(pts, iters=5):
    """Robust (iteratively reweighted) plane z = a x + b y + c; returns unit normal (up) and offset: n.p + c = 0."""
    w = np.ones(len(pts))
    A = np.c_[pts[:, :2], np.ones(len(pts))]
    for _ in range(iters):
        sol = np.linalg.lstsq(A * w[:, None], pts[:, 2] * w, rcond=None)[0]
        r = pts[:, 2] - A @ sol
        s = 1.4826 * np.median(np.abs(r)) + 1e-3
        w = 1.0 / np.maximum(1.0, np.abs(r) / (2 * s))
    a, b, c = sol
    n = np.array([-a, -b, 1.0])
    k = np.linalg.norm(n)
    return n / k, -c / k, r


def bottoms(frames, step=5):
    return np.array([o["center"] - o["rot"][:, 2] * o["dims"][2] / 2 for f in frames[::step] for o in f["objects"]])


def project(K, T, pts):
    pc = (T[:3, :3] @ pts.T).T + T[:3, 3]
    uv = (K @ pc.T).T
    return uv[:, :2] / uv[:, 2:3], pc[:, 2]


def visible(cam, p_base):
    uv, z = project(cam["K"], cam["T_cam_base"], p_base[None])
    return bool(z[0] > 1 and 0 <= uv[0, 0] < W and 0 <= uv[0, 1] < H)


# ------------------------------------------------------------------------------------------ georef

RES = 0.1  # m per BEV pixel


def bev(cam, plane, img, x0, y0, nx, ny):
    """Bird's-eye view (base frame, x right, y up in the image) of one camera image on the ground plane."""
    n, c = plane
    xs = x0 + (np.arange(nx) + 0.5) * RES
    ys = y0 + (ny - np.arange(ny) - 0.5) * RES
    X, Y = np.meshgrid(xs, ys)
    Z = -(c + n[0] * X + n[1] * Y) / n[2]
    uv, z = project(cam["K"], cam["T_cam_base"], np.c_[X.ravel(), Y.ravel(), Z.ravel()])
    mx = uv[:, 0].reshape(ny, nx).astype(np.float32)
    my = uv[:, 1].reshape(ny, nx).astype(np.float32)
    bad = (z.reshape(ny, nx) < 1) | (mx < 0) | (my < 0) | (mx > W - 1) | (my > H - 1)
    # Ground range from the camera: far pixels are smeared and unreliable.
    C = -cam["T_cam_base"][:3, :3].T @ cam["T_cam_base"][:3, 3]
    rng = np.hypot(X - C[0], Y - C[1])
    out = cv2.remap(img, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return out, ~bad, rng


def median_frame(cam_name, k=25):
    files, _ = images(cam_name)
    sel = [files[i] for i in np.linspace(0, len(files) - 1, k).astype(int)]
    return np.median(np.stack([cv2.imread(f) for f in sel]), axis=0).astype(np.uint8)


def tile_xy(lat, lon, z):
    n = 2 ** z
    x = (lon + 180) / 360 * n
    y = (1 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2 * n
    return x, y


def fetch_tile(x, y, z, cache: Path):
    p = cache / f"{z}_{x}_{y}.jpg"
    if not p.exists():
        req = urllib.request.Request(f"https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}",
                                     headers={"User-Agent": "Mozilla/5.0 (JunctionPerception georeference)"})
        p.write_bytes(urllib.request.urlopen(req, timeout=20).read())
    return cv2.imread(str(p))


def satellite_enu(lat0, lon0, half, z, cache: Path):
    """Satellite image resampled onto a local ENU grid (RES m/px, north up) of +-half metres around (lat0, lon0)."""
    cache.mkdir(parents=True, exist_ok=True)
    m = int(round(2 * half / RES))
    e = -half + (np.arange(m) + 0.5) * RES
    nn = half - (np.arange(m) + 0.5) * RES
    E, N = np.meshgrid(e, nn)
    lat, lon = enu_to_geo(lat0, lon0, E, N)
    n = 2 ** z
    fx = (lon + 180) / 360 * n
    fy = (1 - np.arcsinh(np.tan(np.radians(lat))) / np.pi) / 2 * n
    tx0, ty0 = int(fx.min()), int(fy.min())
    tx1, ty1 = int(fx.max()), int(fy.max())
    mosaic = np.zeros(((ty1 - ty0 + 1) * 256, (tx1 - tx0 + 1) * 256, 3), np.uint8)
    for ty in range(ty0, ty1 + 1):
        for tx in range(tx0, tx1 + 1):
            mosaic[(ty - ty0) * 256:(ty - ty0 + 1) * 256, (tx - tx0) * 256:(tx - tx0 + 1) * 256] = fetch_tile(tx, ty, z, cache)
    mx = ((fx - tx0) * 256 - 0.5).astype(np.float32)
    my = ((fy - ty0) * 256 - 0.5).astype(np.float32)
    return cv2.remap(mosaic, mx, my, cv2.INTER_LINEAR)


def markings(img):
    """Road-marking response: bright thin structures (white top-hat), normalised."""
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32)
    th = cv2.morphologyEx(g, cv2.MORPH_TOPHAT, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15)))
    th = cv2.GaussianBlur(th, (0, 0), 1.5)
    return np.clip(th / (np.percentile(th, 99.5) + 1e-3), 0, 1).astype(np.float32)


def warp_base_to_enu(img, theta, te, tn, x0, y0, half, interp=cv2.INTER_LINEAR):
    """Resample a base-frame BEV (origin x0, y0 at its bottom-left) into the satellite ENU grid.

    enu = R(theta) base + (te, tn).
    """
    m = int(round(2 * half / RES))
    ny = img.shape[0]
    c, s = math.cos(theta), math.sin(theta)
    # ENU pixel (i, j) -> enu metres -> base metres -> BEV pixel.
    # e = -half + (j + .5) RES, n = half - (i + .5) RES; bx = c (e - te) + s (n - tn), by = -s (e - te) + c (n - tn)
    # BEV col = (bx - x0)/RES - .5, row = ny - (by - y0)/RES - .5
    # bx = c*(-half + (j+.5)RES - te) + s*(half - (i+.5)RES - tn)
    off_bx = c * (-half + 0.5 * RES - te) + s * (half - 0.5 * RES - tn)
    off_by = -s * (-half + 0.5 * RES - te) + c * (half - 0.5 * RES - tn)
    col = np.array([c, -s, (off_bx - x0) / RES - 0.5])  # col = c j - s i + ...
    row = np.array([s, c, ny - (off_by - y0) / RES - 0.5])  # row = ny - (-s j - c i + off_by - y0)/RES - .5
    M = np.array([col, row])  # maps (j, i, 1) -> (col, row)
    return cv2.warpAffine(img, M, (m, m), flags=interp | cv2.WARP_INVERSE_MAP, borderMode=cv2.BORDER_CONSTANT)


OSM_ROADS = "motorway|trunk|primary|secondary|tertiary|unclassified|residential|motorway_link|trunk_link|primary_link|secondary_link|tertiary_link"


def osm_roads(lat0, lon0, half, cache: Path):
    """Drivable OSM ways around (lat0, lon0) as ENU polylines (cached)."""
    if not cache.exists():
        la0, lo0 = enu_to_geo(lat0, lon0, -half, -half)
        la1, lo1 = enu_to_geo(lat0, lon0, half, half)
        q = f'[out:json];way["highway"~"^({OSM_ROADS})$"]({la0},{lo0},{la1},{lo1});out geom;'
        req = urllib.request.Request("https://overpass-api.de/api/interpreter",
                                     data=urllib.parse.urlencode({"data": q}).encode(),
                                     headers={"User-Agent": "JunctionPerception georeference"})
        cache.write_bytes(urllib.request.urlopen(req, timeout=60).read())
    lines = []
    for w in json.loads(cache.read_text())["elements"]:
        g = w.get("geometry") or []
        if len(g) >= 2:
            e, n = geo_to_enu(lat0, lon0, np.array([p["lat"] for p in g]), np.array([p["lon"] for p in g]))
            lines.append(np.c_[e, n])
    return lines


def ecc_refine(ms, mb, ok, pose, x0, y0, half):
    """Refine a base -> ENU pose (theta, te, tn) by ECC between satellite and BEV marking maps."""
    th, te, tn = pose
    w = warp_base_to_enu(mb, th, te, tn, x0, y0, half)
    mk = (warp_base_to_enu(ok.astype(np.uint8) * 255, th, te, tn, x0, y0, half) > 128).astype(np.uint8)
    a_s = cv2.GaussianBlur(ms, (0, 0), 2)
    a_b = cv2.GaussianBlur(w, (0, 0), 2)
    ncc0 = float(np.corrcoef(a_b[mk > 0], a_s[mk > 0])[0, 1])
    warp = np.eye(2, 3, dtype=np.float32)
    try:
        _, warp = cv2.findTransformECC(a_s, a_b, warp, cv2.MOTION_EUCLIDEAN,
                                       (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 500, 1e-7), mk, 5)
    except cv2.error as e:
        print("ECC failed:", e)
    # warp maps a satellite pixel to the BEV-in-ENU pixel showing the same ground: p_b = Rw p_s + tw.
    Rw, tw = warp[:, :2].astype(float), warp[:, 2].astype(float)

    def corrected(p_enu):
        pb = np.array([(p_enu[0] + half) / RES - 0.5, (half - p_enu[1]) / RES - 0.5])
        ps = np.linalg.solve(Rw, pb - tw)
        return np.array([(ps[0] + 0.5) * RES - half, half - (ps[1] + 0.5) * RES])

    o = corrected(base_to_enu(th, [0.0, 0.0])[0] + [te, tn])
    x = corrected(base_to_enu(th, [10.0, 0.0])[0] + [te, tn]) - o
    th2, te2, tn2 = math.atan2(x[1], x[0]), float(o[0]), float(o[1])
    w2 = cv2.GaussianBlur(warp_base_to_enu(mb, th2, te2, tn2, x0, y0, half), (0, 0), 2)
    mk2 = warp_base_to_enu(ok.astype(np.float32), th2, te2, tn2, x0, y0, half) > 0.5
    ncc1 = float(np.corrcoef(w2[mk2], a_s[mk2])[0, 1])
    return (th2, te2, tn2), ncc0, ncc1


def pose_diff(p, q, pts):
    """Max / mean distance (m) between where two base -> ENU poses put the given base points."""
    a = base_to_enu(p[0], pts) + p[1:]
    b = base_to_enu(q[0], pts) + q[1:]
    d = np.linalg.norm(a - b, axis=1)
    return float(d.max()), float(d.mean())


def road_maps(roads, half, RC, sig=4.0):
    """Distance (m) to the nearest road centreline and a Gaussian road score, on a +-half ENU grid at RC m/px."""
    m = int(2 * half / RC)
    img = np.full((m, m), 255, np.uint8)
    for l in roads:
        cv2.polylines(img, [np.int32(np.round(np.c_[(l[:, 0] + half) / RC, (half - l[:, 1]) / RC]))], False, 0, 1)
    dist = cv2.distanceTransform(img, cv2.DIST_L2, 5) * RC
    return dist, np.exp(-dist ** 2 / (2 * sig ** 2)).astype(np.float32)


def osm_match(score_map, veh, thetas, half, RC):
    """Best translation per rotation of the vehicle-position occupancy over the OSM road score map."""
    out = []
    for th in thetas:
        q = base_to_enu(th, veh)
        emin, nmax = q[:, 0].min(), q[:, 1].max()
        cc = np.int32((q[:, 0] - emin) / RC)
        rr = np.int32((nmax - q[:, 1]) / RC)
        tpl = np.zeros((rr.max() + 1, cc.max() + 1), np.float32)
        tpl[rr, cc] = 1  # occupancy, so queues of stopped vehicles do not dominate
        res = cv2.matchTemplate(score_map, tpl, cv2.TM_CCORR)
        _, v, _, loc = cv2.minMaxLoc(res)
        out.append((v / tpl.sum(), th, -half + loc[0] * RC - emin, half - loc[1] * RC - nmax))
    return out


def cmd_georef(a):
    cams, T_bl = load_cameras()
    frames = load_labels(T_bl)
    n, c, _ = fit_plane(bottoms(frames))
    # BEV extent in base: the area both cameras see.
    x0, y0, x1, y1 = -70.0, -20.0, 50.0, 90.0
    nx, ny = int((x1 - x0) / RES), int((y1 - y0) / RES)
    acc = np.zeros((ny, nx, 3), np.float32)
    best = np.full((ny, nx), np.inf, np.float32)
    per_cam = {}
    for key, cam in cams.items():
        img = median_frame(cam["name"])
        b, ok, rng = bev(cam, (n, c), img, x0, y0, nx, ny)
        ok &= rng < 60
        per_cam[key] = (b, ok)
        upd = ok & (rng < best)
        acc[upd] = b[upd]
        best[upd] = rng[upd]
        cv2.imwrite(str(a.out / f"georef_bev_{key}.jpg"), b)
    mosaic = acc.astype(np.uint8)
    valid = np.isfinite(best)
    cv2.imwrite(str(a.out / "georef_bev.jpg"), mosaic)

    # 1. Coarse: labelled vehicle positions (every other frame) against OSM road centrelines within +-200 m of
    # the approximate position, all rotations.
    lat0, lon0 = APPROX_LATLON
    RC = 0.5
    cache = a.out / "osm_roads.json"
    roads = osm_roads(lat0, lon0, 200.0, cache)
    _, score_map = road_maps(roads, 200.0, RC)
    veh = np.array([o["center"][:2] for f in frames[::2] for o in f["objects"]
                    if CLASS.get(o["type"]) in ("car", "truck", "bus")])
    cands = sorted(osm_match(score_map, veh, np.radians(np.arange(0, 360, a.coarse_step)), 200.0, RC), key=lambda r: -r[0])
    distinct = []
    for cnd in cands:
        if all(abs((math.degrees(cnd[1] - d[1]) + 180) % 360 - 180) > 10 or math.hypot(cnd[2] - d[2], cnd[3] - d[3]) > 20
               for d in distinct):
            distinct.append(cnd)
    print("OSM coarse candidates (score, theta, te, tn):",
          [(round(float(d[0]), 3), round(math.degrees(d[1]), 1), round(float(d[2]), 1), round(float(d[3]), 1)) for d in distinct[:5]])
    fine = osm_match(score_map, veh, distinct[0][1] + np.radians(np.arange(-2, 2.01, 0.1)), 200.0, RC)
    s_osm, th_osm, te_osm, tn_osm = max(fine, key=lambda r: r[0])
    # Re-centre everything on the coarse base origin.
    lat0, lon0 = (float(v) for v in enu_to_geo(lat0, lon0, te_osm, tn_osm))
    pose_osm = (float(th_osm), 0.0, 0.0)
    half = 120.0
    roads = osm_roads(lat0, lon0, half, cache)
    dist, _ = road_maps(roads, half, RC)
    m = dist.shape[0]
    sat = satellite_enu(lat0, lon0, half, a.zoom, a.out / "tiles")
    cv2.imwrite(str(a.out / "georef_satellite.jpg"), sat)
    ms = markings(sat)

    # 2. Fine: satellite road markings against the BEV markings (ECC), from the OSM pose.
    def mk_map(b, ok):
        return markings(b) * cv2.erode(ok.astype(np.uint8), np.ones((9, 9), np.uint8))

    pose, ncc0, ncc1 = ecc_refine(ms, mk_map(mosaic, valid), valid, pose_osm, x0, y0, half)
    fits = {"osm": {"theta_deg": math.degrees(pose_osm[0]) % 360, "te": 0.0, "tn": 0.0, "score": float(s_osm),
                    "second_best": {"score": float(distinct[1][0]), "theta_deg": math.degrees(distinct[1][1]) % 360,
                                    "offset_m": math.hypot(distinct[1][2] - te_osm, distinct[1][3] - tn_osm)}},
            "satellite_both": {"theta_deg": math.degrees(pose[0]) % 360, "te": pose[1], "tn": pose[2],
                               "ncc_before": ncc0, "ncc_after": ncc1}}
    poses = {"both": pose}
    for key, (b, ok) in per_cam.items():
        p, n0, n1 = ecc_refine(ms, mk_map(b, ok), ok, pose_osm, x0, y0, half)
        poses[key] = p
        fits[f"satellite_{key}"] = {"theta_deg": math.degrees(p[0]) % 360, "te": p[1], "tn": p[2], "ncc_before": n0, "ncc_after": n1}
    # Agreement, evaluated where the vehicles are.
    sample = veh[np.random.default_rng(0).choice(len(veh), min(2000, len(veh)), replace=False)]
    agree = {k: dict(zip(("max_m", "mean_m"), pose_diff(poses[k], pose, sample))) for k in cams}
    agree["osm_vs_satellite"] = dict(zip(("max_m", "mean_m"), pose_diff(pose_osm, pose, sample)))
    # Lane-level sanity: distance of the vehicle positions to the nearest OSM centreline after the final pose.
    q = base_to_enu(pose[0], veh) + pose[1:]
    ij = np.int32(np.c_[(half - q[:, 1]) / RC, (q[:, 0] + half) / RC])
    inside = (ij >= 0).all(1) & (ij < m).all(1)
    dd = dist[ij[inside, 0], ij[inside, 1]]
    for k, v in fits.items():
        print(k, json.dumps(v))
    print("agreement", json.dumps(agree))
    lat_o, lon_o = enu_to_geo(lat0, lon0, pose[1], pose[2])
    out = {
        "frame": "s110_base (dev-kit transformation_matrix_s110_lidar_ouster_south_to_s110_base)",
        "origin": {"lat": float(lat_o), "lon": float(lon_o)},
        "x_axis_from_east_deg": math.degrees(pose[0]) % 360,
        "convention": "[e, n] = R(x_axis_from_east) [x, y] (s110_base), metres from origin; local tangent plane as jp::enuToGeo",
        "ground_plane_base": {"normal": n.tolist(), "offset": float(c)},
        "method": "vehicle label positions vs OSM centrelines (coarse, all rotations), then ECC of BEV vs Google "
                  "satellite road markings (median of 25 frames per camera, ground plane from label box bottoms)",
        "satellite": {"source": "Google satellite tiles", "zoom": a.zoom, "res_m_per_px": RES,
                      "search_center": {"lat": lat0, "lon": lon0}},
        "fits": fits,
        "agreement_at_vehicle_positions": agree,
        "vehicle_to_osm_centreline_m": {"median": float(np.median(dd)), "p90": float(np.percentile(dd, 90))},
    }
    (a.out / "georeference.json").write_text(json.dumps(out, indent=1))
    # Overlays: the BEV blended onto the satellite image; BEV markings (red), OSM centrelines (cyan) and
    # labelled vehicle positions (yellow) on the satellite image.
    th, te, tn = pose
    w = warp_base_to_enu(mosaic, th, te, tn, x0, y0, half)
    mk = warp_base_to_enu(valid.astype(np.uint8) * 255, th, te, tn, x0, y0, half) > 128
    blend = sat.copy()
    blend[mk] = (0.5 * sat[mk] + 0.5 * w[mk]).astype(np.uint8)
    edges = warp_base_to_enu(markings(mosaic), th, te, tn, x0, y0, half)
    chk = sat.copy()
    chk[(edges > 0.35) & mk] = (0, 0, 255)
    for l in roads:
        cv2.polylines(chk, [np.int32(np.c_[(l[:, 0] + half) / RES, (half - l[:, 1]) / RES])], False, (255, 255, 0), 1)
    for p in q[::5]:
        cv2.circle(chk, (int((p[0] + half) / RES), int((half - p[1]) / RES)), 2, (0, 255, 255), -1)
    # Checkerboard of 8 m squares (satellite / camera BEV) over the area the cameras see, full resolution: lane
    # lines and kerbs must continue across the square borders.
    ys, xs = np.nonzero(mk)
    sq = int(8 / RES)
    ii, jj = np.mgrid[0:sat.shape[0], 0:sat.shape[1]]
    board = sat.copy()
    use = mk & (((ii // sq) + (jj // sq)) % 2 == 0)
    board[use] = w[use]
    cv2.imwrite(str(a.out / "georef_overlay_checker.jpg"), board[ys.min():ys.max() + 1, xs.min():xs.max() + 1])
    cv2.imwrite(str(a.out / "georef_overlay_blend.jpg"), blend)
    cv2.imwrite(str(a.out / "georef_overlay_markings.jpg"), chk)
    print(json.dumps({k: out[k] for k in ("origin", "x_axis_from_east_deg", "vehicle_to_osm_centreline_m")}))




def load_georef(out: Path):
    g = json.loads((out / "georeference.json").read_text())
    return g["origin"]["lat"], g["origin"]["lon"], math.radians(g["x_axis_from_east_deg"])


def base_to_enu(theta, p):
    c, s = math.cos(theta), math.sin(theta)
    p = np.atleast_2d(p)
    return np.c_[c * p[:, 0] - s * p[:, 1], s * p[:, 0] + c * p[:, 1]]


# ------------------------------------------------------------------------------------------- videos

def cmd_videos(a):
    VIDEOS.mkdir(parents=True, exist_ok=True)
    for key, name in CAMS.items():
        files, t = images(name)
        dt = np.diff(t)
        # Capture is irregular (gaps up to ~0.4 s), so writing one frame per image at a constant rate lets video
        # time drift from capture time by more than a second. Resample instead: video frame i shows the image
        # captured closest to t0 + i / fps, at the sequence's mean rate (few repeated or skipped images).
        fps = round((len(t) - 1) / (t[-1] - t[0]), 2)
        nv = int(math.floor((t[-1] - t[0]) * fps)) + 1
        vt = t[0] + np.arange(nv) / fps
        j = np.clip(np.searchsorted(t, vt), 1, len(t) - 1)
        j = np.where(np.abs(t[j - 1] - vt) <= np.abs(t[j] - vt), j - 1, j)
        off = t[j] - vt
        path = a.out / f"tumtraf_s03_{key}_8mm.mp4"
        vw = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (W, H))
        for k in j:
            vw.write(cv2.imread(files[k]))
        vw.release()
        cap = cv2.VideoCapture(str(path))
        nf, vfps = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)), cap.get(cv2.CAP_PROP_FPS)
        cap.release()
        info = {
            "camera": name, "video": path.name, "fps": fps, "images": len(files), "frames": nv,
            "video_frames": nf, "video_fps": vfps, "t0": float(t[0]), "duration_s": float(t[-1] - t[0]),
            "capture_dt_s": {"median": float(np.median(dt)), "min": float(dt.min()), "max": float(dt.max())},
            "repeated_frames": int((np.diff(j) == 0).sum()), "skipped_images": int(len(files) - len(np.unique(j))),
            "capture_minus_video_time_s": {"max_abs": float(np.abs(off).max()), "rms": float(np.sqrt(np.mean(off ** 2)))},
            "timestamps": [{"frame": i, "video_t": float(i / fps), "t": float(t[k]), "image": os.path.basename(files[k])}
                           for i, k in enumerate(j)],
        }
        (a.out / f"timestamps_{key}.json").write_text(json.dumps(info, indent=1))
        os.system(f"cp '{path}' '{VIDEOS / path.name}'")
        print(key, {k: v for k, v in info.items() if k != "timestamps"})


# -------------------------------------------------------------------------------------- calibration

def cmd_calib(a):
    cams, T_bl = load_cameras()
    frames = load_labels(T_bl)
    lat0, lon0, theta = load_georef(a.out)
    pts = bottoms(frames, step=2)
    for key, cam in cams.items():
        # Ground plane from the box bottoms this camera sees (closer than 90 m).
        C = -cam["T_cam_base"][:3, :3].T @ cam["T_cam_base"][:3, 3]
        sel = np.array([visible(cam, p) and np.hypot(*(p - C)[:2]) < 90 for p in pts])
        nb, cb, r = fit_plane(pts[sel])
        R, t = cam["T_cam_base"][:3, :3], cam["T_cam_base"][:3, 3]
        n_cam = R @ nb
        d = cb - n_cam @ t  # camera height above the plane
        ez = np.array([0.0, 0.0, 1.0])
        f_cam = ez - (ez @ n_cam) * n_cam
        f_base = R.T @ f_cam
        f_enu = base_to_enu(theta, f_base[:2])[0]
        bearing = math.degrees(math.atan2(f_enu[0], f_enu[1])) % 360
        foot = C - nb * d
        e, nn = base_to_enu(theta, foot[:2])[0]
        lat, lon = enu_to_geo(lat0, lon0, e, nn)
        K = cam["K"]
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, -n_cam[2]))))
        roll = math.degrees(math.atan2(n_cam[0], -n_cam[1]))
        out = {
            "width": W, "height": H,
            "K": K.tolist(), "dist": [],
            "plane": [*n_cam.tolist(), float(d)],
            "bearing_deg": bearing, "pitch_deg": pitch, "roll_deg": roll, "height_m": float(d),
            "anchor": {"lat": float(lat), "lon": float(lon), "alt": 0.0},
            "source": "tumtraf",
            "confidence": 1.0,
            "tumtraf": {
                "camera": cam["name"],
                "from": "dev-kit projection_from_s110_lidar_ouster_south + lidar->s110_base + georeference.json",
                "plane_fit": {"boxes": int(sel.sum()), "residual_rms_m": float(np.sqrt(np.mean(r ** 2))),
                              "residual_mad_m": float(1.4826 * np.median(np.abs(r))),
                              "tilt_deg": math.degrees(math.acos(nb[2]))},
                "camera_base": C.tolist(), "anchor_base": foot.tolist(),
                "plane_base": {"normal": nb.tolist(), "offset": float(cb)},
            },
        }
        (a.out / f"calib_{key}.json").write_text(json.dumps(out, indent=1))
        print(key, {k: out[k] for k in ("bearing_deg", "pitch_deg", "roll_deg", "height_m", "anchor")}, out["tumtraf"]["plane_fit"])


# ----------------------------------------------------------------------------------- ground truth

def track_speeds(frames, half_window=0.5):
    """Speed (m/s) and velocity (base) per (frame index, id): linear fit of the track over +-half_window s.

    A few label ids are reused for another object (a jump of tens of metres between frames); the fit never
    spans such a jump (> 40 m/s).
    """
    tracks: dict[str, list[tuple[float, int, np.ndarray]]] = {}
    for i, f in enumerate(frames):
        for o in f["objects"]:
            tracks.setdefault(o["id"], []).append((f["ts"], i, o["center"][:2]))
    out = {}
    for oid, tr in tracks.items():
        ts = np.array([x[0] for x in tr])
        ps = np.array([x[2] for x in tr])
        jump = np.linalg.norm(np.diff(ps, axis=0), axis=1) > 40 * np.maximum(np.diff(ts), 0.05)
        seg = np.r_[0, np.cumsum(jump)]
        for k, (t, i, _) in enumerate(tr):
            m = (np.abs(ts - t) <= half_window) & (seg == seg[k])
            if m.sum() < 3:
                out[(i, oid)] = None
                continue
            A = np.c_[ts[m] - t, np.ones(m.sum())]
            v = np.linalg.lstsq(A, ps[m], rcond=None)[0][0]
            out[(i, oid)] = v
    return out, len(tracks)


def gt_objects(frame, idx, speeds, lat0, lon0, theta, cams):
    objs = []
    for o in frame["objects"]:
        bottom = o["center"] - o["rot"][:, 2] * o["dims"][2] / 2
        e, n = base_to_enu(theta, o["center"][:2])[0]
        lat, lon = enu_to_geo(lat0, lon0, e, n)
        hx = o["rot"][:2, 0]
        yaw_enu = math.atan2(*base_to_enu(theta, hx)[0][::-1])
        v = speeds.get((idx, o["id"]))
        ve = base_to_enu(theta, v)[0] if v is not None else None
        objs.append({
            "id": o["id"], "track": o["id"][:8], "type": o["type"], "class": CLASS.get(o["type"], "other"),
            "lat": float(lat), "lon": float(lon), "e": float(e), "n": float(n),
            "z_base": float(bottom[2]),
            "yaw_deg": math.degrees(yaw_enu),  # counter-clockwise from east (twin ground frame yaw)
            "heading_deg": (90.0 - math.degrees(yaw_enu)) % 360,  # compass, clockwise from north (twin "heading")
            "length": float(o["dims"][0]), "width": float(o["dims"][1]), "height": float(o["dims"][2]),
            "speed": None if v is None else float(np.linalg.norm(v)),
            "vel_en": None if ve is None else [float(ve[0]), float(ve[1])],
            "visible": {k: visible(c, bottom) for k, c in cams.items()},
        })
    return objs


def cmd_gt(a):
    cams, T_bl = load_cameras()
    frames = load_labels(T_bl)
    lat0, lon0, theta = load_georef(a.out)
    speeds, ntracks = track_speeds(frames)
    lts = np.array([f["ts"] for f in frames])
    label_objs = [gt_objects(f, i, speeds, lat0, lon0, theta, cams) for i, f in enumerate(frames)]
    out = {
        "dataset": "TUMTraf Intersection R02 s03 (S110), labels from s110_lidar_ouster_south",
        "georeference": json.loads((a.out / "georeference.json").read_text())["origin"],
        "conventions": {
            "lat/lon": "WGS84 of the box centre (footprint centre)",
            "e/n": "metres east/north of georeference.origin",
            "yaw_deg": "box x-axis, counter-clockwise from east (ENU)",
            "heading_deg": "compass, clockwise from north, as the twin's road-user 'heading'",
            "speed": "m/s, linear fit of the track over +-0.5 s (null for tracks shorter than 3 frames there)",
            "visible": "bottom centre projects inside that camera's image",
            "label_dt_s": "label timestamp minus video frame capture timestamp",
        },
        "tracks": ntracks,
        "cameras": {},
    }
    for key, name in CAMS.items():
        info = json.loads((a.out / f"timestamps_{key}.json").read_text())
        frs = []
        for x in info["timestamps"]:
            j = int(np.argmin(np.abs(lts - x["t"])))
            frs.append({"frame": x["frame"], "t": x["t"], "video_t": x["video_t"], "image": x["image"], "label": frames[j]["file"],
                        "label_dt_s": float(lts[j] - x["t"]), "objects": label_objs[j]})
        dts = np.array([f["label_dt_s"] for f in frs])
        out["cameras"][key] = {"camera": name, "video": info["video"], "fps": info["fps"],
                               "label_dt_abs_max_s": float(np.abs(dts).max()), "frames": frs}
        print(key, len(frs), "frames, |label - image| max", round(float(np.abs(dts).max()), 3), "s")
    (a.out / "groundtruth.json").write_text(json.dumps(out))
    print("tracks", ntracks)


# ------------------------------------------------------------------------------------------- check

class CameraModel:
    """Python mirror of jp::CameraModel (core/src/geometry/camera.cpp), enough to project ground-frame boxes."""

    def __init__(self, j):
        self.K = np.array(j["K"])
        p = np.array(j["plane"], float)
        n = p[:3] / np.linalg.norm(p[:3])
        self.h = p[3] / np.linalg.norm(p[:3])
        ez = np.array([0, 0, 1.0])
        f = ez - (ez @ n) * n
        f /= np.linalg.norm(f)
        l = np.cross(n, f)
        C = np.c_[f, l, n]
        b = math.radians(j["bearing_deg"])
        Wm = np.c_[[math.sin(b), math.cos(b), 0], [-math.cos(b), math.sin(b), 0], [0, 0, 1]]
        self.R_gc = Wm @ C.T
        self.anchor = (j["anchor"]["lat"], j["anchor"]["lon"])
        self.n = n

    def pixel_to_ground(self, uv):
        r = np.linalg.solve(self.K, [uv[0], uv[1], 1.0])
        t = -self.h / (r @ self.n)
        return self.R_gc @ (r * t) + [0, 0, self.h] if t > 0 else None

    def ground_to_pixel(self, p):
        pc = (self.R_gc.T @ (np.atleast_2d(p) - [0, 0, self.h]).T).T
        uv = (self.K @ pc.T).T
        return uv[:, :2] / uv[:, 2:3], pc[:, 2]

    def box(self, bottom, yaw, dims):
        x = np.array([math.cos(yaw), math.sin(yaw), 0])
        y = np.array([-math.sin(yaw), math.cos(yaw), 0])
        s = [(1, 1), (1, -1), (-1, -1), (-1, 1)]
        pts = [bottom + a * dims[0] / 2 * x + b * dims[1] / 2 * y + top * dims[2] * np.array([0, 0, 1])
               for top in (0, 1) for a, b in s]
        return self.ground_to_pixel(np.array(pts))


def lidar_corners(o):
    l, w, h = o["dims"]
    s = [(1, 1), (1, -1), (-1, -1), (-1, 1)]
    loc = np.array([[a * l / 2, b * w / 2, (top - 0.5) * h] for top in (0, 1) for a, b in s])
    return (o["lidar_rot"] @ loc.T).T + o["lidar_center"]


EDGES = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]


def cmd_check(a):
    cams, T_bl = load_cameras()
    frames = load_labels(T_bl)
    gt = json.loads((a.out / "groundtruth.json").read_text())
    report = {"variants": {
        "flat": "box on the CameraModel ground plane (what the runtime assumes); includes how far the labelled box "
                "bottom is off the fitted plane",
        "exact": "box bottom raised to the label's own height above the fitted plane; isolates the conversion "
                 "(georeference round trip, plane, bearing, anchor)",
        "flat_ground_lift_m": "true box-bottom pixel lifted onto the flat CameraModel ground, distance to the ground "
                              "truth position: the localisation error floor a perfect detector would get"}}
    by_label = {f["file"]: f for f in frames}
    for key, cam in cams.items():
        cj = json.loads((a.out / f"calib_{key}.json").read_text())
        model = CameraModel(cj)
        nb, cb = np.array(cj["tumtraf"]["plane_base"]["normal"]), cj["tumtraf"]["plane_base"]["offset"]
        rows = {"flat": [], "exact": []}
        dzs, lift = [], []
        gframes = gt["cameras"][key]["frames"]
        for fi, gf in enumerate(gframes):
            src = {o["id"]: o for o in by_label[gf["label"]]["objects"]}
            draw = fi in (100, 500, 900)
            img = cv2.imread(str(SRC / "images" / cam["name"] / gf["image"])) if draw else None
            for o in gf["objects"]:
                if not o["visible"][key]:
                    continue
                s = src[o["id"]]
                q = (cam["P_lidar"] @ np.c_[lidar_corners(s), np.ones(8)].T).T
                if (q[:, 2] <= 0).any():
                    continue
                uv_d = q[:, :2] / q[:, 2:3]
                e, n = geo_to_enu(*model.anchor, o["lat"], o["lon"])
                dz = float(nb @ (s["center"] - s["rot"][:, 2] * s["dims"][2] / 2) + cb)
                dzs.append(dz)
                # Localisation floor: the true bottom-centre pixel lifted onto the flat ground plane, vs ground truth.
                qb = cam["P_lidar"] @ np.append(s["lidar_center"] - s["lidar_rot"][:, 2] * s["dims"][2] / 2, 1)
                g = model.pixel_to_ground(qb[:2] / qb[2])
                if g is not None:
                    lift.append((math.hypot(e, n), float(np.hypot(g[0] - e, g[1] - n))))
                dims = [o["length"], o["width"], o["height"]]
                for var, z in (("flat", 0.0), ("exact", dz)):
                    uv_m, zm = model.box(np.array([e, n, z]), math.radians(o["yaw_deg"]), dims)
                    if (zm <= 0).any():
                        continue
                    d = np.linalg.norm(uv_m - uv_d, axis=1)
                    rows[var].append((math.hypot(e, n), float(d.mean()), float(d[:4].mean())))
                    if draw and var == "flat":
                        for i, j in EDGES:
                            cv2.line(img, tuple(np.int32(uv_d[i])), tuple(np.int32(uv_d[j])), (0, 255, 0), 2)
                            cv2.line(img, tuple(np.int32(uv_m[i])), tuple(np.int32(uv_m[j])), (0, 0, 255), 1)
            if draw:
                cv2.putText(img, "green: dataset LiDAR->camera projection   red: CameraModel JSON + ground truth lat/lon (flat ground)",
                            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
                cv2.imwrite(str(a.out / f"check_{key}_{fi:04d}.jpg"), img)
        rep = {"label_bottom_above_plane_m": {"median": float(np.median(dzs)), "mad": float(1.4826 * np.median(np.abs(dzs - np.median(dzs))))}}
        for var, r in rows.items():
            r = np.array(r)
            by_range = {}
            for r0, r1 in ((0, 30), (30, 60), (60, 200)):
                m = (r[:, 0] >= r0) & (r[:, 0] < r1)
                if m.any():
                    by_range[f"{r0}-{r1}m"] = {"n": int(m.sum()), "median_px": float(np.median(r[m, 1])),
                                               "p95_px": float(np.percentile(r[m, 1], 95))}
            rep[var] = {"boxes": len(r), "corner_px": {"median": float(np.median(r[:, 1])), "p95": float(np.percentile(r[:, 1], 95))},
                        "bottom_corner_px": {"median": float(np.median(r[:, 2])), "p95": float(np.percentile(r[:, 2], 95))},
                        "corner_px_by_range": by_range}
        lift = np.array(lift)
        rep["flat_ground_lift_m"] = {}
        for r0, r1 in ((0, 30), (30, 60), (60, 200), (0, 200)):
            m = (lift[:, 0] >= r0) & (lift[:, 0] < r1)
            if m.any():
                rep["flat_ground_lift_m"][f"{r0}-{r1}m"] = {"n": int(m.sum()), "median": float(np.median(lift[m, 1])),
                                                            "p95": float(np.percentile(lift[m, 1], 95))}
        report[key] = rep
        print(key, json.dumps(rep))
    (a.out / "check_projection.json").write_text(json.dumps(report, indent=1))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("step", choices=["georef", "videos", "calib", "gt", "check", "all"])
    ap.add_argument("--out", type=Path, default=OUT)
    ap.add_argument("--zoom", type=int, default=20)
    ap.add_argument("--coarse-step", type=float, default=1.0, help="deg, rotation search step")
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    steps = ["georef", "videos", "calib", "gt", "check"] if a.step == "all" else [a.step]
    for s in steps:
        globals()[f"cmd_{s}"](a)


if __name__ == "__main__":
    main()
