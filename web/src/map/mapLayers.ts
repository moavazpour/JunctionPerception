import * as THREE from 'three';
import { mergeGeometries } from 'three/examples/jsm/utils/BufferGeometryUtils.js';

import type { Building, MapData, Point, Polyline } from './mapData';

/** Colours of the stylized map: light greys, white markings. */
export const MAP_PALETTE = {
  road: 0xd3d6db,
  path: 0xe2e4e8,
  marking: 0xffffff,
  building: 0xe6e8eb,
  roof: 0xf3f4f6,
} as const;

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

function buildingMesh(buildings: Building[]): THREE.Mesh | null {
  const geometries = buildings
    .filter((b) => b.footprint.length >= 3)
    .map((b) => {
      const shape = new THREE.Shape(b.footprint.map(([x, y]) => new THREE.Vector2(x, y)));
      return new THREE.ExtrudeGeometry(shape, { depth: b.height_m, bevelEnabled: false });
    });
  if (geometries.length === 0) {
    return null;
  }
  const mesh = new THREE.Mesh(
    mergeGeometries(geometries.map((g) => g.toNonIndexed())),
    new THREE.MeshStandardMaterial({ color: MAP_PALETTE.building, roughness: 0.95 }),
  );
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  return mesh;
}

/** Stylized map from OSM roads, paths and crossings: flat light-grey surfaces, white zebras. */
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
      group.add(flatMesh(mergeGeometries(stripes), MAP_PALETTE.marking));
    }
  }
  return group;
}

/** LoD2 buildings as light extruded blocks. */
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
  const mesh = new THREE.Mesh(geometry, new THREE.MeshBasicMaterial({ map: texture }));
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
