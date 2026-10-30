/** Map layers of a scene as written by tools/prepare_map_data.py (world frame, metres). */

export type Point = [number, number];

export interface Attribution {
  text: string;
  license: string;
  license_url: string;
  changes: string;
}

export interface MapManifest {
  scene_id: string;
  radius_m: number;
  layers: { ortho?: { file: string }; osm?: { file: string }; buildings?: { file: string } };
  attribution: Record<string, Attribution>;
}

export interface OrthoLayer {
  image: string;
  /** World positions of the image corners: top-left, top-right, bottom-right, bottom-left. */
  corners: [Point, Point, Point, Point];
}

export interface Polyline {
  id: number;
  highway: string;
  width_m: number;
  points: Point[];
}

export interface Crossing {
  id: number;
  kind: 'node' | 'way';
  crossing: string;
  position?: Point;
  points?: Point[];
  width_m?: number;
}

export interface OsmLayer {
  roads: Polyline[];
  paths: Polyline[];
  crossings: Crossing[];
}

export interface Building {
  id: string;
  height_m: number;
  footprint: Point[];
}

export interface MapData {
  base: string;
  manifest: MapManifest;
  ortho: OrthoLayer | null;
  osm: OsmLayer | null;
  buildings: Building[];
}

async function getJson<T>(url: string): Promise<T> {
  const response = await fetch(url);
  if (!response.ok) {
    throw new Error(`GET ${url} failed: ${response.status}`);
  }
  return (await response.json()) as T;
}

/** Loads the manifest at `manifestUrl` and the layers it lists. */
export async function loadMapData(manifestUrl: string): Promise<MapData> {
  const base = manifestUrl.slice(0, manifestUrl.lastIndexOf('/') + 1);
  const manifest = await getJson<MapManifest>(manifestUrl);
  const { ortho, osm, buildings } = manifest.layers;
  return {
    base,
    manifest,
    ortho: ortho ? await getJson<OrthoLayer>(base + ortho.file) : null,
    osm: osm ? await getJson<OsmLayer>(base + osm.file) : null,
    buildings: buildings
      ? (await getJson<{ buildings: Building[] }>(base + buildings.file)).buildings
      : [],
  };
}

/**
 * Polygon of a polyline widened to `width` (one quad per segment, no joins): the outline used
 * for crossing suggestions.
 */
export function widenSegment(a: Point, b: Point, width: number): Point[] {
  const dx = b[0] - a[0];
  const dy = b[1] - a[1];
  const length = Math.hypot(dx, dy);
  if (length === 0) {
    return [];
  }
  const nx = (-dy / length) * (width / 2);
  const ny = (dx / length) * (width / 2);
  return [
    [a[0] + nx, a[1] + ny],
    [b[0] + nx, b[1] + ny],
    [b[0] - nx, b[1] - ny],
    [a[0] - nx, a[1] - ny],
  ];
}

const CROSSING_WIDTH_M = 4;
const CROSSING_NODE_LENGTH_M = 8;

/**
 * Crosswalk zone suggestions from OSM: each crossing way becomes a strip of 4 m along the way;
 * crossing nodes on a road become a 4 m x 8 m strip across that road.
 */
export function crossingSuggestions(osm: OsmLayer): Point[][] {
  const polygons: Point[][] = [];
  for (const crossing of osm.crossings) {
    if (crossing.kind === 'way' && crossing.points && crossing.points.length >= 2) {
      const first = crossing.points[0] as Point;
      const last = crossing.points[crossing.points.length - 1] as Point;
      polygons.push(widenSegment(first, last, CROSSING_WIDTH_M));
    } else if (crossing.kind === 'node' && crossing.position) {
      const road = nearestRoadDirection(osm.roads, crossing.position);
      if (road) {
        const [px, py] = crossing.position;
        const half = CROSSING_NODE_LENGTH_M / 2;
        // Across the road: perpendicular to the road direction.
        const a: Point = [px - road[1] * half, py + road[0] * half];
        const b: Point = [px + road[1] * half, py - road[0] * half];
        polygons.push(widenSegment(a, b, CROSSING_WIDTH_M));
      }
    }
  }
  return polygons.filter((p) => p.length === 4);
}

/** Unit direction of the road segment closest to a point. */
function nearestRoadDirection(roads: Polyline[], point: Point): Point | null {
  let best: Point | null = null;
  let bestDistance = Infinity;
  for (const road of roads) {
    for (let i = 1; i < road.points.length; i++) {
      const a = road.points[i - 1] as Point;
      const b = road.points[i] as Point;
      const dx = b[0] - a[0];
      const dy = b[1] - a[1];
      const lengthSq = dx * dx + dy * dy;
      if (lengthSq === 0) {
        continue;
      }
      const t = Math.min(
        Math.max(((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / lengthSq, 0),
        1,
      );
      const distance = Math.hypot(a[0] + t * dx - point[0], a[1] + t * dy - point[1]);
      if (distance < bestDistance) {
        bestDistance = distance;
        const length = Math.sqrt(lengthSq);
        best = [dx / length, dy / length];
      }
    }
  }
  return best;
}
