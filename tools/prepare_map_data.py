"""Fetch open map data for a scene and convert it into the scene's world frame.

Sources (see docs/map-data.md for licenses and attribution):
  * Orthophoto DOP20 of the Bavarian survey administration (WMS, CC BY 4.0)
  * LoD2 3D building models of the Bavarian survey administration (CityGML tiles, CC BY 4.0)
  * OpenStreetMap roads, paths and crossings (OpenStreetMap API, ODbL)

Reads geo_anchor and map.radius_m from scenes/<id>/scene.yaml and writes to map_cache/<id>/:
  map.json        manifest with layers and attribution
  ortho.jpg/.json orthophoto and the world positions of its corners
  osm.json        roads, paths and crossings as world polylines / points
  buildings.json  building footprints (world polygons) with heights
  raw/            downloaded source files (cache)

World frame: local east-north-up at the anchor (x east, y north, metres), the same PROJ
pipeline as the C++ code. ETRS89 (source CRS of the Bavarian data) is taken as WGS84.

Requires: requests, pyproj, lxml, pyyaml.

Example:
    python tools/prepare_map_data.py --scene scenes/munich_5538
"""

import argparse
import json
import math
import pathlib

import requests
import yaml
from lxml import etree
from pyproj import Transformer

WMS_URL = "https://geoservices.bayern.de/od/wms/dop/v1/dop20"
WMS_LAYER = "by_dop20c"
LOD2_URL = "https://download1.bayernwolke.de/a/lod2/citygml/{e}_{n}.gml"
# OpenStreetMap API: all data of a small bounding box in one request.
OSM_MAP_URL = "https://api.openstreetmap.org/api/0.6/map"
UTM_CRS = "EPSG:25832"
# Public services ask clients to identify themselves.
HEADERS = {"User-Agent": "JunctionPerception map tool (github.com/moavazpour/JunctionPerception)"}
ORTHO_METRES_PER_PIXEL = 0.2

ATTRIBUTION = {
    "bavaria": {
        "text": "Bayerische Vermessungsverwaltung – www.geodaten.bayern.de",
        "license": "CC BY 4.0",
        "license_url": "https://creativecommons.org/licenses/by/4.0/",
        "changes": "reprojected to the scene frame, cropped, buildings reduced to footprints",
    },
    "osm": {
        "text": "© OpenStreetMap contributors",
        "license": "ODbL 1.0",
        "license_url": "https://www.openstreetmap.org/copyright",
        "changes": "roads, paths and crossings converted to the scene frame",
    },
}

# Paved width when OSM has neither width nor lanes (metres).
DEFAULT_ROAD_WIDTH = {
    "motorway": 11.0, "trunk": 10.0, "primary": 9.0, "secondary": 8.0, "tertiary": 7.0,
    "unclassified": 6.0, "residential": 6.0, "service": 4.0, "living_street": 5.0,
    "motorway_link": 5.0, "trunk_link": 5.0, "primary_link": 5.0, "secondary_link": 5.0,
    "tertiary_link": 5.0,
}
PATH_WIDTH = {"footway": 2.5, "cycleway": 2.0, "path": 2.0, "pedestrian": 4.0, "steps": 2.0}
LANE_WIDTH = 3.25

NS = {
    "core": "http://www.opengis.net/citygml/1.0",
    "bldg": "http://www.opengis.net/citygml/building/1.0",
    "gml": "http://www.opengis.net/gml",
}


class WorldFrame:
    """Local ENU at the scene anchor; geographic and UTM positions to world metres."""

    def __init__(self, lat: float, lon: float, alt: float):
        self.lat, self.lon, self.alt = lat, lon, alt
        self._enu = Transformer.from_pipeline(
            "+proj=pipeline +step +proj=unitconvert +xy_in=deg +xy_out=rad "
            "+step +proj=cart +ellps=WGS84 "
            f"+step +proj=topocentric +ellps=WGS84 +lat_0={lat:.12f} +lon_0={lon:.12f} +h_0={alt:.4f}"
        )
        self._utm_to_geo = Transformer.from_crs(UTM_CRS, "EPSG:4326", always_xy=True)
        self._geo_to_utm = Transformer.from_crs("EPSG:4326", UTM_CRS, always_xy=True)

    def from_geo(self, lat: float, lon: float) -> list:
        e, n, _ = self._enu.transform(lon, lat, self.alt)
        return [round(e, 3), round(n, 3)]

    def from_utm(self, x: float, y: float) -> list:
        lon, lat = self._utm_to_geo.transform(x, y)
        return self.from_geo(lat, lon)

    def utm_box(self, radius: float) -> tuple:
        """UTM bounding box of the square of half-size `radius` around the anchor."""
        cx, cy = self._geo_to_utm.transform(self.lon, self.lat)
        return cx - radius, cy - radius, cx + radius, cy + radius

    def geo_box(self, radius: float) -> tuple:
        """south, west, north, east of the square of half-size `radius` around the anchor."""
        dlat = radius / 111_320.0
        dlon = radius / (111_320.0 * math.cos(math.radians(self.lat)))
        return self.lat - dlat, self.lon - dlon, self.lat + dlat, self.lon + dlon


def download(url: str, target: pathlib.Path, **kwargs) -> pathlib.Path:
    """Downloads once into the raw cache."""
    if not target.exists():
        response = requests.get(url, timeout=120, headers=HEADERS, **kwargs)
        response.raise_for_status()
        target.write_bytes(response.content)
    return target


def fetch_ortho(frame: WorldFrame, radius: float, out: pathlib.Path, raw: pathlib.Path) -> dict:
    minx, miny, maxx, maxy = frame.utm_box(radius)
    size = int(round((maxx - minx) / ORTHO_METRES_PER_PIXEL))
    params = {
        "SERVICE": "WMS", "VERSION": "1.3.0", "REQUEST": "GetMap", "LAYERS": WMS_LAYER,
        "STYLES": "", "CRS": UTM_CRS, "BBOX": f"{minx},{miny},{maxx},{maxy}",
        "WIDTH": size, "HEIGHT": size, "FORMAT": "image/jpeg",
    }
    source = download(WMS_URL, raw / f"dop20_{int(minx)}_{int(miny)}_{size}.jpg", params=params)
    (out / "ortho.jpg").write_bytes(source.read_bytes())
    corners = [frame.from_utm(minx, maxy), frame.from_utm(maxx, maxy),
               frame.from_utm(maxx, miny), frame.from_utm(minx, miny)]
    meta = {"image": "ortho.jpg", "corners": corners, "corner_order": "top-left, top-right, "
            "bottom-right, bottom-left", "metres_per_pixel": ORTHO_METRES_PER_PIXEL,
            "attribution": "bavaria"}
    (out / "ortho.json").write_text(json.dumps(meta, indent=2))
    return {"file": "ortho.json", "pixels": size}


def road_width(tags: dict) -> float:
    for key in ("width", "est_width"):
        try:
            return float(tags[key].split()[0])
        except (KeyError, ValueError):
            pass
    highway = tags.get("highway", "")
    if highway in PATH_WIDTH:
        return PATH_WIDTH[highway]
    if "lanes" in tags:
        try:
            return LANE_WIDTH * float(tags["lanes"].split(";")[0])
        except ValueError:
            pass
    return DEFAULT_ROAD_WIDTH.get(highway, 5.0)


def osm_elements(path: pathlib.Path) -> tuple:
    """Nodes (id -> (lat, lon, tags)) and ways (id, tags, node ids) of an OSM XML file."""
    root = etree.parse(str(path)).getroot()

    def tags_of(element) -> dict:
        return {t.get("k"): t.get("v") for t in element.iterfind("tag")}

    nodes = {n.get("id"): (float(n.get("lat")), float(n.get("lon")), tags_of(n))
             for n in root.iterfind("node")}
    ways = [(int(w.get("id")), tags_of(w), [nd.get("ref") for nd in w.iterfind("nd")])
            for w in root.iterfind("way")]
    return nodes, ways


def fetch_osm(frame: WorldFrame, radius: float, out: pathlib.Path, raw: pathlib.Path) -> dict:
    south, west, north, east = frame.geo_box(radius)
    bbox = f"{west:.7f},{south:.7f},{east:.7f},{north:.7f}"
    cache = download(OSM_MAP_URL, raw / f"osm_{bbox.replace(',', '_')}.osm",
                     params={"bbox": bbox})
    nodes, ways = osm_elements(cache)

    roads, paths, crossings = [], [], []
    for node_id, (lat, lon, tags) in nodes.items():
        if tags.get("highway") == "crossing" or "crossing" in tags:
            crossings.append({"id": int(node_id), "kind": "node",
                              "position": frame.from_geo(lat, lon),
                              "crossing": tags.get("crossing", "")})
    for way_id, tags, refs in ways:
        if "highway" not in tags:
            continue
        points = [frame.from_geo(nodes[r][0], nodes[r][1]) for r in refs if r in nodes]
        if len(points) < 2:
            continue
        highway = tags.get("highway", "")
        entry = {"id": way_id, "highway": highway, "width_m": road_width(tags),
                 "points": points}
        if tags.get("footway") == "crossing" or tags.get("cycleway") == "crossing":
            crossings.append({**entry, "kind": "way", "crossing": tags.get("crossing", "")})
        elif highway in PATH_WIDTH:
            paths.append(entry)
        elif highway in DEFAULT_ROAD_WIDTH:
            entry["lanes"] = tags.get("lanes")
            entry["oneway"] = tags.get("oneway") == "yes"
            roads.append(entry)
    data = {"roads": roads, "paths": paths, "crossings": crossings, "attribution": "osm"}
    (out / "osm.json").write_text(json.dumps(data))
    return {"file": "osm.json", "roads": len(roads), "paths": len(paths),
            "crossings": len(crossings)}


def lod2_tiles(frame: WorldFrame, radius: float) -> list:
    """Names (km of the lower-left corner) of the 2 km x 2 km LoD2 tiles covering the scene."""
    minx, miny, maxx, maxy = frame.utm_box(radius)
    first_e, last_e = 2 * int(minx // 2000), 2 * int(maxx // 2000)
    first_n, last_n = 2 * int(miny // 2000), 2 * int(maxy // 2000)
    return [(e, n) for e in range(first_e, last_e + 1, 2) for n in range(first_n, last_n + 1, 2)]


def parse_buildings(path: pathlib.Path, frame: WorldFrame, radius: float) -> list:
    buildings = []
    tree = etree.parse(str(path))
    for building in tree.iterfind(".//bldg:Building", NS):
        ground = building.find(".//bldg:GroundSurface//gml:posList", NS)
        if ground is None:
            continue
        values = [float(v) for v in ground.text.split()]
        xyz = list(zip(values[0::3], values[1::3], values[2::3]))
        footprint = [frame.from_utm(x, y) for x, y, _ in xyz]
        if min(math.hypot(px, py) for px, py in footprint) > radius:
            continue
        height = building.findtext("bldg:measuredHeight", namespaces=NS)
        if height is None:
            roof_z = [float(v) for pos in building.iterfind(".//bldg:RoofSurface//gml:posList", NS)
                      for v in pos.text.split()[2::3]]
            height = (max(roof_z) - min(z for _, _, z in xyz)) if roof_z else 6.0
        gml_id = building.get("{http://www.opengis.net/gml}id", "")
        buildings.append({"id": gml_id, "height_m": round(float(height), 2),
                          "footprint": footprint})
    return buildings


def fetch_buildings(frame: WorldFrame, radius: float, out: pathlib.Path,
                    raw: pathlib.Path) -> dict:
    buildings = []
    for e, n in lod2_tiles(frame, radius):
        tile = download(LOD2_URL.format(e=e, n=n), raw / f"lod2_{e}_{n}.gml")
        buildings.extend(parse_buildings(tile, frame, radius))
    data = {"buildings": buildings, "attribution": "bavaria"}
    (out / "buildings.json").write_text(json.dumps(data))
    return {"file": "buildings.json", "buildings": len(buildings)}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--scene", type=pathlib.Path, required=True, help="scene folder")
    parser.add_argument("--out", type=pathlib.Path, default=pathlib.Path("map_cache"))
    args = parser.parse_args()

    scene = yaml.safe_load((args.scene / "scene.yaml").read_text())
    anchor = scene.get("geo_anchor")
    if anchor is None:
        raise SystemExit(f"{args.scene}/scene.yaml has no geo_anchor")
    radius = float(scene.get("map", {}).get("radius_m", 200.0))
    frame = WorldFrame(anchor["lat_deg"], anchor["lon_deg"], anchor["alt_m"])

    out = args.out / args.scene.name
    raw = out / "raw"
    raw.mkdir(parents=True, exist_ok=True)
    layers = {
        "ortho": fetch_ortho(frame, radius, out, raw),
        "osm": fetch_osm(frame, radius, out, raw),
        "buildings": fetch_buildings(frame, radius, out, raw),
    }
    manifest = {"scene_id": args.scene.name, "geo_anchor": anchor, "radius_m": radius,
                "layers": layers, "attribution": ATTRIBUTION}
    (out / "map.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False))
    print(json.dumps(layers, indent=2))


if __name__ == "__main__":
    main()
