import * as THREE from 'three';

import type { ClassInfo } from '../net/client';

/** Colours of the twin: desaturated greys, one accent for highlights, red for conflicts. */
export const PALETTE = {
  background: 0xe9eaec,
  ground: 0xf2f3f5,
  grid: 0xe1e3e7,
  zoneFill: 0xffffff,
  zoneEdge: 0xc8ccd2,
  body: 0xa3a8ae,
  cabin: 0x7d838b,
  accent: 0x2f7cf6,
  conflict: 0xe5484d,
} as const;

/** Shared materials so that highlighting is a material swap per part. */
export interface ObjectMaterials {
  body: THREE.MeshStandardMaterial;
  cabin: THREE.MeshStandardMaterial;
}

/** Opacity of objects outside the validated region: tracked and shown, but not evaluated. */
export const UNVALIDATED_OPACITY = 0.35;

export function makeMaterials(highlighted: boolean, validated = true): ObjectMaterials {
  const faded = validated
    ? {}
    : { transparent: true, opacity: UNVALIDATED_OPACITY, depthWrite: false };
  const body = new THREE.MeshStandardMaterial({
    color: highlighted ? PALETTE.accent : PALETTE.body,
    roughness: 0.85,
    metalness: 0.0,
    ...faded,
  });
  const cabin = new THREE.MeshStandardMaterial({
    color: highlighted ? new THREE.Color(PALETTE.accent).multiplyScalar(0.7) : PALETTE.cabin,
    roughness: 0.6,
    metalness: 0.0,
    ...faded,
  });
  return { body, cabin };
}

/** Box with its base at z = bottom, centred at (x, 0) along the object's length axis (+x). */
function box(
  length: number,
  width: number,
  height: number,
  x: number,
  bottom: number,
  material: THREE.Material,
): THREE.Mesh {
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(length, width, height), material);
  mesh.position.set(x, 0, bottom + height / 2);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  return mesh;
}

function person(height: number, radius: number, x: number, bottom: number, m: ObjectMaterials) {
  const group = new THREE.Group();
  const torso = new THREE.Mesh(
    new THREE.CylinderGeometry(radius * 0.8, radius, height * 0.78, 8),
    m.body,
  );
  torso.rotation.x = Math.PI / 2;
  torso.position.set(x, 0, bottom + height * 0.39);
  const head = new THREE.Mesh(new THREE.SphereGeometry(radius * 0.75, 10, 8), m.cabin);
  head.position.set(x, 0, bottom + height * 0.78 + radius * 0.6);
  for (const part of [torso, head]) {
    part.castShadow = true;
    group.add(part);
  }
  return group;
}

/**
 * Low-poly mesh of a twin class at the class's fixed size, facing +x, standing on z = 0.
 * Built from primitives, so the twin needs no external model files.
 */
export function buildClassMesh(info: ClassInfo, m: ObjectMaterials): THREE.Group {
  const { length_m: l, width_m: w, height_m: h } = info;
  const group = new THREE.Group();
  const add = (...parts: THREE.Object3D[]) => parts.forEach((p) => group.add(p));
  switch (info.name) {
    case 'pedestrian':
      add(person(h, w / 2, 0, 0, m));
      break;
    case 'bicycle':
    case 'motorcycle': {
      const frameHeight = h * (info.name === 'motorcycle' ? 0.45 : 0.4);
      add(box(l, w * 0.35, frameHeight, 0, 0.05 * h, m.cabin));
      add(person(h - frameHeight * 0.6, w * 0.32, -0.1 * l, frameHeight * 0.6, m));
      break;
    }
    case 'truck':
      add(box(0.24 * l, w, h, 0.38 * l, 0, m.cabin));
      add(box(0.74 * l, w, 0.95 * h, -0.13 * l, 0, m.body));
      break;
    case 'trailer':
      add(box(l, w, 0.8 * h, 0, 0.2 * h, m.body));
      break;
    case 'bus':
      add(box(l, w, 0.55 * h, 0, 0.1 * h, m.body));
      add(box(0.98 * l, 0.98 * w, 0.25 * h, 0, 0.65 * h, m.cabin));
      add(box(l, w, 0.1 * h, 0, 0.9 * h, m.body));
      break;
    case 'van':
      add(box(l, w, 0.55 * h, 0, 0.1 * h, m.body));
      add(box(0.8 * l, 0.94 * w, 0.35 * h, -0.08 * l, 0.65 * h, m.cabin));
      break;
    default: {
      // car, emergency_vehicle, other
      add(box(l, w, 0.45 * h, 0, 0.12 * h, m.body));
      add(box(0.55 * l, 0.9 * w, 0.43 * h, -0.05 * l, 0.57 * h, m.cabin));
      if (info.name === 'emergency_vehicle') {
        add(box(0.12 * l, 0.6 * w, 0.08 * h, -0.05 * l, h, m.cabin));
      }
    }
  }
  return group;
}

/** Soft red ring drawn under objects involved in a conflict. */
export function buildConflictHalo(info: ClassInfo): THREE.Mesh {
  const radius = Math.max(info.length_m, info.width_m) * 0.75 + 0.6;
  const halo = new THREE.Mesh(
    new THREE.RingGeometry(radius * 0.7, radius, 40),
    new THREE.MeshBasicMaterial({
      color: PALETTE.conflict,
      transparent: true,
      opacity: 0.45,
      depthWrite: false,
    }),
  );
  halo.position.z = 0.02;
  return halo;
}

/** Releases the GPU resources of an object and its children. */
export function disposeObject(object: THREE.Object3D): void {
  object.traverse((child) => {
    if (child instanceof THREE.Mesh || child instanceof THREE.LineSegments) {
      child.geometry.dispose();
      const materials: THREE.Material[] = Array.isArray(child.material)
        ? child.material
        : [child.material];
      materials.forEach((material) => material.dispose());
    }
  });
}
