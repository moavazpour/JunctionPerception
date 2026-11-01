import * as THREE from 'three';

import type { ClassInfo } from '../net/client';
import { classColor, twinPalette } from './palette';

/** Opacity of objects outside the validated region: tracked and shown, but not evaluated. */
export const UNVALIDATED_OPACITY = 0.3;

/** How an object is drawn: in an active event, in a conflict, inside the validated region. */
export interface ObjectLook {
  highlighted: boolean;
  conflict: boolean;
  validated: boolean;
}

/** Shared materials so that highlighting is a material swap per part. */
export interface ObjectMaterials {
  body: THREE.MeshStandardMaterial;
  cabin: THREE.MeshStandardMaterial;
  dark: THREE.MeshStandardMaterial;
}

export function makeMaterials(className: string, look: ObjectLook): ObjectMaterials {
  const faded = look.validated
    ? {}
    : { transparent: true, opacity: UNVALIDATED_OPACITY, depthWrite: false };
  const color = new THREE.Color(classColor(className));
  const emissive = look.conflict
    ? new THREE.Color(twinPalette().conflict)
    : look.highlighted
      ? color.clone()
      : new THREE.Color(0x000000);
  const emissiveIntensity = look.conflict ? 0.55 : look.highlighted ? 0.35 : 0;
  const body = new THREE.MeshStandardMaterial({
    color,
    roughness: 0.55,
    metalness: 0.15,
    emissive,
    emissiveIntensity,
    ...faded,
  });
  const cabin = new THREE.MeshStandardMaterial({
    color: color.clone().multiplyScalar(0.42),
    roughness: 0.25,
    metalness: 0.4,
    emissive,
    emissiveIntensity: emissiveIntensity * 0.6,
    ...faded,
  });
  const dark = new THREE.MeshStandardMaterial({ color: 0x0b0f17, roughness: 0.9, ...faded });
  return { body, cabin, dark };
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

/** Wheel pair across the width at position x along the length. */
function axle(x: number, width: number, radius: number, m: ObjectMaterials): THREE.Mesh {
  const wheel = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, width, 12), m.dark);
  wheel.position.set(x, 0, radius);
  wheel.castShadow = true;
  return wheel;
}

function person(height: number, radius: number, x: number, bottom: number, m: ObjectMaterials) {
  const group = new THREE.Group();
  const torso = new THREE.Mesh(
    new THREE.CapsuleGeometry(radius * 0.85, height * 0.55, 4, 10),
    m.body,
  );
  torso.rotation.x = Math.PI / 2;
  torso.position.set(x, 0, bottom + height * 0.42);
  const head = new THREE.Mesh(new THREE.SphereGeometry(radius * 0.72, 14, 10), m.body);
  head.position.set(x, 0, bottom + height * 0.8 + radius * 0.5);
  for (const part of [torso, head]) {
    part.castShadow = true;
    group.add(part);
  }
  return group;
}

/** Car-like vehicle: lower body, glass cabin, wheels. */
function vehicle(l: number, w: number, h: number, cabinLength: number, m: ObjectMaterials) {
  const wheel = Math.min(0.36, h * 0.22);
  return [
    box(l, w, 0.42 * h, 0, wheel * 0.9, m.body),
    box(cabinLength * l, 0.88 * w, 0.36 * h, -0.06 * l, wheel * 0.9 + 0.42 * h, m.cabin),
    box(cabinLength * l * 0.82, 0.86 * w, 0.06 * h, -0.07 * l, wheel * 0.9 + 0.78 * h, m.body),
    axle(0.32 * l, w * 1.02, wheel, m),
    axle(-0.32 * l, w * 1.02, wheel, m),
  ];
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
      const motor = info.name === 'motorcycle';
      const wheel = motor ? 0.32 : 0.34;
      const frameHeight = h * (motor ? 0.45 : 0.4);
      add(axle(0.36 * l, 0.08, wheel, m), axle(-0.36 * l, 0.08, wheel, m));
      add(box(l * 0.8, w * (motor ? 0.4 : 0.18), frameHeight * 0.45, 0, wheel * 0.9, m.cabin));
      add(person(h - frameHeight * 0.6, w * 0.3, -0.08 * l, frameHeight * 0.6, m));
      break;
    }
    case 'truck': {
      const wheel = 0.5;
      add(box(0.22 * l, w, 0.86 * h, 0.39 * l, wheel * 0.8, m.cabin));
      add(box(0.74 * l, w, 0.82 * h, -0.12 * l, wheel * 0.8, m.body));
      add(axle(0.38 * l, w, wheel, m), axle(-0.1 * l, w, wheel, m), axle(-0.36 * l, w, wheel, m));
      break;
    }
    case 'trailer': {
      add(box(l, w, 0.78 * h, 0, 0.22 * h, m.body));
      add(axle(-0.3 * l, w, 0.5, m), axle(-0.4 * l, w, 0.5, m));
      break;
    }
    case 'bus': {
      add(box(l, w, 0.5 * h, 0, 0.12 * h, m.body));
      add(box(0.98 * l, 0.99 * w, 0.26 * h, 0, 0.62 * h, m.cabin));
      add(box(l, w, 0.1 * h, 0, 0.88 * h, m.body));
      add(axle(0.33 * l, w, 0.5, m), axle(-0.3 * l, w, 0.5, m));
      break;
    }
    case 'van':
      add(...vehicle(l, w, h * 1.1, 0.78, m));
      break;
    default: {
      // car, emergency_vehicle, other
      add(...vehicle(l, w, h, 0.52, m));
      if (info.name === 'emergency_vehicle') {
        const light = new THREE.MeshBasicMaterial({ color: 0x5ab0ff });
        add(box(0.12 * l, 0.6 * w, 0.08 * h, -0.05 * l, h, light));
      }
    }
  }
  return group;
}

/** Soft disc under every object, so that objects sit on the ground from any angle. */
export function buildContactShadow(info: ClassInfo): THREE.Mesh {
  const shadow = new THREE.Mesh(
    new THREE.CircleGeometry(0.5, 24),
    new THREE.MeshBasicMaterial({
      color: twinPalette().shadow,
      transparent: true,
      opacity: twinPalette().shadowOpacity,
      depthWrite: false,
    }),
  );
  shadow.scale.set(info.length_m * 1.15 + 0.4, info.width_m * 1.3 + 0.4, 1);
  shadow.position.z = 0.02;
  return shadow;
}

/** Ring in the class colour under road users at risk (VRUs), visible from far away. */
export function buildVruRing(info: ClassInfo): THREE.Mesh {
  const radius = Math.max(info.length_m, info.width_m) * 0.6 + 0.7;
  const ring = new THREE.Mesh(
    new THREE.RingGeometry(radius * 0.82, radius, 40),
    new THREE.MeshBasicMaterial({
      color: classColor(info.name),
      transparent: true,
      opacity: 0.75,
      depthWrite: false,
    }),
  );
  ring.position.z = 0.03;
  return ring;
}

/** Pulsing red ring drawn under objects involved in a conflict. */
export function buildConflictHalo(info: ClassInfo): THREE.Mesh {
  const radius = Math.max(info.length_m, info.width_m) * 0.75 + 0.9;
  const halo = new THREE.Mesh(
    new THREE.RingGeometry(radius * 0.55, radius, 48),
    new THREE.MeshBasicMaterial({
      color: twinPalette().conflict,
      transparent: true,
      opacity: 0.45,
      depthWrite: false,
    }),
  );
  halo.position.z = 0.04;
  return halo;
}

/** Ring under the road user the pointer is on. */
export function buildSelectionRing(info: ClassInfo): THREE.Mesh {
  const radius = Math.max(info.length_m, info.width_m) * 0.7 + 1.1;
  const ring = new THREE.Mesh(
    new THREE.RingGeometry(radius * 0.9, radius, 48),
    new THREE.MeshBasicMaterial({
      color: twinPalette().cameraView,
      transparent: true,
      opacity: 0.95,
      depthWrite: false,
    }),
  );
  ring.position.z = 0.05;
  return ring;
}

/**
 * Fades an object in or out: `fade` (0..1) multiplies the opacity each material was built with.
 * Fully visible objects draw opaque again, so they sort and shadow normally.
 */
export function setFade(object: THREE.Object3D, fade: number): void {
  object.traverse((child) => {
    if (!(child instanceof THREE.Mesh)) {
      return;
    }
    const materials: THREE.Material[] = Array.isArray(child.material)
      ? child.material
      : [child.material];
    for (const material of materials) {
      const data = material.userData as { baseOpacity?: number; baseTransparent?: boolean };
      if (data.baseOpacity === undefined) {
        data.baseOpacity = material.opacity;
        data.baseTransparent = material.transparent;
      }
      const transparent = fade < 1 || data.baseTransparent === true;
      if (material.transparent !== transparent) {
        material.transparent = transparent;
        material.needsUpdate = true;
      }
      material.opacity = data.baseOpacity * fade;
      material.depthWrite = !transparent;
    }
    if (child.userData['baseCastShadow'] === undefined) {
      child.userData['baseCastShadow'] = child.castShadow;
    }
    child.castShadow = child.userData['baseCastShadow'] === true && fade > 0.5;
  });
}

/** Releases the GPU resources of an object and its children. */
export function disposeObject(object: THREE.Object3D): void {
  object.traverse((child) => {
    if (
      child instanceof THREE.Mesh ||
      child instanceof THREE.LineSegments ||
      child instanceof THREE.Line
    ) {
      child.geometry.dispose();
      const materials: THREE.Material[] = Array.isArray(child.material)
        ? child.material
        : [child.material];
      materials.forEach((material) => material.dispose());
    }
  });
}
