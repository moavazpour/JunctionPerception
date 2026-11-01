import * as THREE from 'three';
import { mergeGeometries } from 'three/examples/jsm/utils/BufferGeometryUtils.js';

import { TWIN } from '../twin/palette';
import type { Building, MapData, Point, Polyline } from './mapData';

/** Colours of the stylized map: dark asphalt, soft markings (see src/twin/palette.ts). */
export const MAP_PALETTE = {
  road: TWIN.road,
  path: TWIN.path,
  marking: TWIN.marking,
  building: TWIN.building,
  buildingEdge: TWIN.buildingEdge,
} as const;

const BUILDING_EDGE_OPACITY = 0.55;
// Edges between faces that meet at less than this angle are not drawn (roof triangulation).
const BUILDING_EDGE_ANGLE_DEG = 25;

const ROAD_Z = 0.006;
const PATH_Z = 0.004;
const MARKING_Z = 0.01;
const ORTHO_Z = 0.003;
const STRIPE_LENGTH_M = 0.5;
const STRIPE_GAP_M = 0.5;

/** Ribbon of a polyline: one quad per segment plus discs at the joints for round corners. */
export function ribbonGeometry(points: Point[], width: number, z: number): THREE.BufferGeometry {
  const parts: THREE.BufferGeometry[] = [];
  for (let i = 1; i < points.length; i++) {
    const [ax, ay] = points[i - 1] as Point;
    const [bx, by] = points[i] as Point;
    const length = Math.hypot(bx - ax, by - ay);
    if (length === 0) {
      continue;
    }
    const segment = new THREE.PlaneGeometry(length, width);
    segment.rotateZ(Math.atan2(by - ay, bx - ax));
    segment.translate((ax + bx) / 2, (ay + by) / 2, z);
    parts.push(segment);
  }
  for (const [x, y] of points.slice(1, -1)) {
    const joint = new THREE.CircleGeometry(width / 2, 12);
    joint.translate(x, y, z);
    parts.push(joint);
  }
  return parts.length > 0 ? mergeGeometries(parts) : new THREE.BufferGeometry();
}

/** Zebra stripes along a crossing from `a` to `b`, stripes parallel to the road. */
function zebraGeometry(a: Point, b: Point, width: number): THREE.BufferGeometry[] {
  const dx = b[0] - a[0];
  const dy = b[1] - a[1];
  const length = Math.hypot(dx, dy);
  const stripes: THREE.BufferGeometry[] = [];
  for (let s = STRIPE_GAP_M; s + STRIPE_LENGTH_M <= length; s += STRIPE_LENGTH_M + STRIPE_GAP_M) {
    const t = (s + STRIPE_LENGTH_M / 2) / length;
    const stripe = new THREE.PlaneGeometry(STRIPE_LENGTH_M, width);
    stripe.rotateZ(Math.atan2(dy, dx));
    stripe.translate(a[0] + dx * t, a[1] + dy * t, MARKING_Z);
    stripes.push(stripe);
  }
  return stripes;
}

function flatMesh(geometry: THREE.BufferGeometry, color: number): THREE.Mesh {
  const mesh = new THREE.Mesh(
    geometry,
    new THREE.MeshStandardMaterial({ color, roughness: 1, metalness: 0 }),
  );
  mesh.receiveShadow = true;
  return mesh;
}

function mergedRibbons(lines: Polyline[], z: number): THREE.BufferGeometry | null {
  const geometries = lines.map((line) => ribbonGeometry(line.points, line.width_m, z));
  const valid = geometries.filter((g) => g.getAttribute('position') !== undefined);
  return valid.length > 0 ? mergeGeometries(valid) : null;
}

function buildingMesh(buildings: Building[]): THREE.Group | null {
  const geometries = buildings
    .filter((b) => b.footprint.length >= 3)
    .map((b) => {
      const shape = new THREE.Shape(b.footprint.map(([x, y]) => new THREE.Vector2(x, y)));
      return new THREE.ExtrudeGeometry(shape, { depth: b.height_m, bevelEnabled: false });
    });
  if (geometries.length === 0) {
    return null;
  }
  const merged = mergeGeometries(geometries.map((g) => g.toNonIndexed()));
  const mesh = new THREE.Mesh(
    merged,
    new THREE.MeshStandardMaterial({
      color: MAP_PALETTE.building,
      roughness: 0.9,
      metalness: 0.05,
    }),
  );
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  // Outlines give the blocks an architectural look on the dark ground.
  const edges = new THREE.LineSegments(
    new THREE.EdgesGeometry(merged, BUILDING_EDGE_ANGLE_DEG),
    new THREE.LineBasicMaterial({
      color: MAP_PALETTE.buildingEdge,
      transparent: true,
      opacity: BUILDING_EDGE_OPACITY,
    }),
  );
  return new THREE.Group().add(mesh, edges);
}

/** Stylized map from OSM roads, paths and crossings: dark flat surfaces, light zebras. */
export function buildStylizedLayer(map: MapData): THREE.Group {
  const group = new THREE.Group();
  if (map.osm) {
    const roads = mergedRibbons(map.osm.roads, ROAD_Z);
    if (roads) {
      group.add(flatMesh(roads, MAP_PALETTE.road));
    }
    const paths = mergedRibbons(map.osm.paths, PATH_Z);
    if (paths) {
      group.add(flatMesh(paths, MAP_PALETTE.path));
    }
    const stripes = map.osm.crossings.flatMap((c) =>
      c.kind === 'way' && c.points && c.points.length >= 2
        ? zebraGeometry(c.points[0] as Point, c.points[c.points.length - 1] as Point, 3)
        : [],
    );
    if (stripes.length > 0) {
      const zebras = flatMesh(mergeGeometries(stripes), MAP_PALETTE.marking);
      Object.assign(zebras.material, { transparent: true, opacity: 0.75 });
      group.add(zebras);
    }
  }
  return group;
}

/** LoD2 buildings as dark extruded blocks with outlined edges. */
export function buildBuildingLayer(map: MapData): THREE.Group {
  const group = new THREE.Group();
  const mesh = buildingMesh(map.buildings);
  if (mesh) {
    group.add(mesh);
  }
  return group;
}

/** Orthophoto on a quad through its four world corners. */
export function buildOrthoLayer(map: MapData): THREE.Mesh | null {
  if (!map.ortho) {
    return null;
  }
  const [tl, tr, br, bl] = map.ortho.corners;
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute(
    'position',
    new THREE.Float32BufferAttribute(
      [...tl, ORTHO_Z, ...tr, ORTHO_Z, ...br, ORTHO_Z, ...bl, ORTHO_Z],
      3,
    ),
  );
  geometry.setAttribute('uv', new THREE.Float32BufferAttribute([0, 1, 1, 1, 1, 0, 0, 0], 2));
  geometry.setIndex([0, 3, 1, 1, 3, 2]);
  geometry.computeVertexNormals();
  const texture = new THREE.TextureLoader().load(map.base + map.ortho.image);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.anisotropy = 8;
  // Slightly dimmed and without tone mapping, so the photo keeps its colours in the dark twin.
  const mesh = new THREE.Mesh(
    geometry,
    new THREE.MeshBasicMaterial({ map: texture, color: 0xd2d8e2, toneMapped: false }),
  );
  mesh.receiveShadow = false;
  return mesh;
}

/** Attribution lines of the layers in use, e.g. "© OpenStreetMap contributors (ODbL 1.0)". */
export function attributionLines(map: MapData): { text: string; url: string }[] {
  return Object.values(map.manifest.attribution).map((a) => ({
    text: `${a.text} (${a.license})`,
    url: a.license_url,
  }));
}
