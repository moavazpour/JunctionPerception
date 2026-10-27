// Stylized road map of the site (the default view): roads, lane markings, sidewalks, crosswalks, stop
// lines and traffic lights built from OpenStreetMap data and drawn in the dark in-car visualisation style.
// Coordinates: local ENU (origin = ground point below the camera, x east, y north); scene = (x, h, -y).
import * as THREE from "three";

const R = 6378137.0;
const OVERPASS = ["https://overpass-api.de/api/interpreter", "https://overpass.kumi.systems/api/interpreter"];

export interface MapStyle {
  background: number;
  ground: number;
  road: number;
  sidewalk: number;
  curb: number;
  marking: number;
  stopLine: number;
}

// In-car visualisation palettes: dark (night) and light (day).
export const PALETTES: Record<"dark" | "light", MapStyle> = {
  dark: { background: 0x24272c, ground: 0x2c2f35, road: 0x141619, sidewalk: 0x3b3f46, curb: 0x8e959e, marking: 0xe9ecef, stopLine: 0xffffff },
  light: { background: 0xe9ebee, ground: 0xf2f3f5, road: 0xb9bec6, sidewalk: 0xdde0e5, curb: 0x8f969f, marking: 0xffffff, stopLine: 0xffffff },
};

type V2 = [number, number];
interface Way {
  id: number;
  nodes: number[];
  geometry: { lat: number; lon: number }[];
  tags: Record<string, string>;
}
interface OsmNode {
  id: number;
  lat: number;
  lon: number;
  tags?: Record<string, string>;
}

const CAR_ROADS = new Set([
  "motorway", "trunk", "primary", "secondary", "tertiary", "unclassified", "residential", "living_street", "service",
  "motorway_link", "trunk_link", "primary_link", "secondary_link", "tertiary_link", "road", "busway",
]);
const FOOT = new Set(["footway", "path", "pedestrian", "cycleway", "steps"]);
const LANE_W = 3.25;

function defaultLanes(t: Record<string, string>): number {
  const n = Number(t.lanes);
  if (n > 0) return n;
  const h = t.highway;
  const oneway = t.oneway === "yes" || h.endsWith("_link") || h === "motorway";
  if (h === "service" || h === "living_street") return 1;
  if (h === "motorway" || h === "trunk") return oneway ? 2 : 4;
  return oneway ? 1 : 2;
}

async function fetchOsm(lat: number, lon: number, radius: number): Promise<{ ways: Way[]; nodes: OsmNode[] } | null> {
  const key = `jp-osm:${lat.toFixed(5)},${lon.toFixed(5)},${radius}`;
  try {
    const c = localStorage.getItem(key);
    if (c) return JSON.parse(c);
  } catch {
    /* storage unavailable */
  }
  // Through the site manager first: it caches the area on disk and tries several Overpass mirrors.
  try {
    const r = await fetch(`/api/osm?lat=${lat.toFixed(5)}&lon=${lon.toFixed(5)}&r=${radius}`);
    if (r.ok) {
      const out = await r.json();
      try {
        localStorage.setItem(key, JSON.stringify(out));
      } catch {
        /* too large or unavailable: fine */
      }
      return out;
    }
  } catch {
    /* no manager (single-camera stack): ask Overpass directly */
  }
  const q = `[out:json][timeout:25];(way["highway"](around:${radius},${lat},${lon});` +
    `node["highway"~"traffic_signals|crossing|stop"](around:${radius},${lat},${lon}););out body geom;`;
  for (const url of OVERPASS) {
    try {
      const r = await fetch(url, { method: "POST", body: "data=" + encodeURIComponent(q) });
      if (!r.ok) continue;
      const j = await r.json();
      const out = {
        ways: j.elements.filter((e: any) => e.type === "way" && e.geometry),
        nodes: j.elements.filter((e: any) => e.type === "node"),
      };
      try {
        localStorage.setItem(key, JSON.stringify(out));
      } catch {
        /* too large or unavailable: fine */
      }
      return out;
    } catch {
      /* try the next server */
    }
  }
  return null;
}

// ---------------------------------------------------------------- geometry helpers

function offsetLine(pts: V2[], d: number): V2[] {
  return pts.map((p, i) => {
    const a = pts[Math.max(i - 1, 0)], b = pts[Math.min(i + 1, pts.length - 1)];
    let tx = b[0] - a[0], ty = b[1] - a[1];
    const l = Math.hypot(tx, ty) || 1;
    tx /= l;
    ty /= l;
    return [p[0] - ty * d, p[1] + tx * d];
  });
}

class Batch {
  pos: number[] = [];
  quad(a: V2, b: V2, c: V2, d: V2, h: number) {
    for (const p of [a, b, c, a, c, d]) this.pos.push(p[0], h, -p[1]);
  }
  // Ribbon of width w along pts.
  ribbon(pts: V2[], w: number, h: number) {
    const l = offsetLine(pts, w / 2), r = offsetLine(pts, -w / 2);
    for (let i = 0; i + 1 < pts.length; i++) this.quad(l[i], r[i], r[i + 1], l[i + 1], h);
  }
  // Painted line (solid or dashed) of width w along pts, skipping parts close to junctions.
  stripe(pts: V2[], w: number, h: number, dash: number, gap: number, keepOut: V2[], keepR: number) {
    let phase = 0;
    for (let i = 0; i + 1 < pts.length; i++) {
      const a = pts[i], b = pts[i + 1];
      const L = Math.hypot(b[0] - a[0], b[1] - a[1]);
      if (L < 1e-3) continue;
      const tx = (b[0] - a[0]) / L, ty = (b[1] - a[1]) / L, nx = -ty * w / 2, ny = tx * w / 2;
      const step = dash > 0 ? 0.5 : L;
      for (let s = 0; s < L; s += step) {
        const e = Math.min(s + step, L);
        const on = dash <= 0 || (phase + s) % (dash + gap) < dash;
        const p: V2 = [a[0] + tx * s, a[1] + ty * s], q: V2 = [a[0] + tx * e, a[1] + ty * e];
        const m: V2 = [(p[0] + q[0]) / 2, (p[1] + q[1]) / 2];
        if (on && !keepOut.some((k) => Math.hypot(k[0] - m[0], k[1] - m[1]) < keepR))
          this.quad([p[0] + nx, p[1] + ny], [p[0] - nx, p[1] - ny], [q[0] - nx, q[1] - ny], [q[0] + nx, q[1] + ny], h);
      }
      phase += L;
    }
  }
  mesh(color: number, order: number): THREE.Mesh {
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.Float32BufferAttribute(this.pos, 3));
    g.computeVertexNormals();
    const m = new THREE.Mesh(g, new THREE.MeshBasicMaterial({ color, side: THREE.DoubleSide, depthWrite: false }));
    m.renderOrder = order;
    return m;
  }
}

function trafficLight(): THREE.Group {
  // Floating head, no pole (as in the in-car view).
  const g = new THREE.Group();
  const box = new THREE.Mesh(new THREE.BoxGeometry(0.45, 1.2, 0.35), new THREE.MeshStandardMaterial({ color: 0x2a2d33 }));
  box.position.y = 4.5;
  g.add(box);
  [0xff3b30, 0xffb020, 0x34c759].forEach((c, i) => {
    const lamp = new THREE.Mesh(new THREE.SphereGeometry(0.12, 12, 8),
      new THREE.MeshStandardMaterial({ color: c, emissive: c, emissiveIntensity: 0.35 }));
    lamp.position.set(0, 4.85 - i * 0.35, 0.18);
    g.add(lamp);
  });
  return g;
}

// ---------------------------------------------------------------- build

export async function buildStylizedMap(lat: number, lon: number, STYLE: MapStyle, radius = 350): Promise<THREE.Group | null> {
  const osm = await fetchOsm(lat, lon, radius);
  if (!osm) return null;
  const kx = (R * Math.PI * Math.cos((lat * Math.PI) / 180)) / 180, ky = (R * Math.PI) / 180;
  const enu = (p: { lat: number; lon: number }): V2 => [(p.lon - lon) * kx, (p.lat - lat) * ky];

  // Junction nodes: shared by two or more car roads (markings stop short of them).
  const use = new Map<number, number>();
  const roads = osm.ways.filter((w) => CAR_ROADS.has(w.tags.highway) && w.tags.area !== "yes");
  for (const w of roads) for (const n of new Set(w.nodes)) use.set(n, (use.get(n) ?? 0) + 1);
  const junctions: V2[] = [];
  const nodeXY = new Map<number, V2>();
  for (const w of osm.ways) w.nodes.forEach((n, i) => nodeXY.set(n, enu(w.geometry[i])));
  for (const [n, k] of use) if (k >= 2 && nodeXY.has(n)) junctions.push(nodeXY.get(n)!);

  const asphalt = new Batch(), side = new Batch(), curb = new Batch(), paint = new Batch();
  for (const w of osm.ways) {
    const pts = w.geometry.map(enu);
    if (pts.length < 2) continue;
    const h = w.tags.highway;
    if (FOOT.has(h)) {
      if (w.tags.footway === "crossing" || w.tags.cycleway === "crossing") {
        // Zebra: stripes along the crossing.
        paint.stripe(pts, 3.0, 0.03, 0.5, 0.5, [], 0);
      } else {
        side.ribbon(pts, h === "pedestrian" ? 4 : 2.2, 0.005);
      }
      continue;
    }
    if (!CAR_ROADS.has(h)) continue;
    const lanes = defaultLanes(w.tags);
    const width = Math.max(Number(w.tags.width) || lanes * LANE_W, 3);
    asphalt.ribbon(pts, width, 0.01);
    const keep = Math.max(width * 0.6, 6);
    for (const s of [width / 2, -width / 2]) curb.stripe(offsetLine(pts, s), 0.18, 0.02, 0, 0, [], 0);
    if (lanes < 2 || h === "service") continue;
    const oneway = w.tags.oneway === "yes" || h.endsWith("_link") || h === "motorway";
    const fwd = oneway ? lanes : Number(w.tags["lanes:forward"]) || Math.ceil(lanes / 2);
    // Lane separators: dashed between same-direction lanes, solid between the directions.
    for (let k = 1; k < lanes; k++) {
      const d = width / 2 - k * (width / lanes);
      const centre = !oneway && k === fwd;
      paint.stripe(offsetLine(pts, d), centre ? 0.2 : 0.15, 0.03, centre ? 0 : 3, centre ? 0 : 6, junctions, keep);
    }
  }

  const g = new THREE.Group();
  g.add(asphalt.mesh(STYLE.road, 1), side.mesh(STYLE.sidewalk, 0), curb.mesh(STYLE.curb, 2), paint.mesh(STYLE.marking, 3));
  // Traffic lights (OSM nodes); also mark stop lines across the road where a light stands on it.
  const stop = new Batch();
  for (const n of osm.nodes) {
    const p = enu(n);
    if (n.tags?.highway === "traffic_signals") {
      const tl = trafficLight();
      tl.position.set(p[0], 0, -p[1]);
      g.add(tl);
      // Stop line across the nearest road at the signal.
      const w = roads.find((r) => r.nodes.includes(n.id));
      if (w) {
        const i = w.nodes.indexOf(n.id);
        const a = enu(w.geometry[Math.max(i - 1, 0)]), b = enu(w.geometry[Math.min(i + 1, w.geometry.length - 1)]);
        const t = [b[0] - a[0], b[1] - a[1]], l = Math.hypot(t[0], t[1]) || 1;
        const width = Math.max(Number(w.tags.width) || defaultLanes(w.tags) * LANE_W, 3);
        const nx = -t[1] / l, ny = t[0] / l;
        stop.stripe([[p[0] + nx * width / 2, p[1] + ny * width / 2], [p[0] - nx * width / 2, p[1] - ny * width / 2]], 0.5, 0.035, 0, 0, [], 0);
      }
    }
  }
  if (stop.pos.length) g.add(stop.mesh(STYLE.stopLine, 4));
  return g;
}
