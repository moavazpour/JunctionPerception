import * as THREE from 'three';

import type { WorldObject } from '../net/schema';
import { classColor } from './palette';

// Just above the zones (0.08 m) and below the rings under road users.
const FOOTPRINT_Z = 0.1;
const FILL_OPACITY = 0.28;
const LOST_COLOR = 0x8a94a6;
// Smallest drawn radius of a road user without heading (VRUs), metres.
const MIN_RADIUS_M = 0.35;

/** How a footprint is drawn: from what the backend reports about its size and heading. */
export interface FootprintLook {
  /** Oriented rectangle (heading known) or orientation-free disc (no heading). */
  shape: 'box' | 'disc';
  /** Size measured from box corners (solid outline) or the class size (dashed). */
  measured: boolean;
  /** No longer observed: kept at its last measured position, grey and dashed. */
  lost: boolean;
}

/** Look and size of the footprint of a backend object; classes give the size when not measured. */
export function footprintOf(
  object: WorldObject,
  classSize: { length_m: number; width_m: number },
  lost = false,
): { look: FootprintLook; length: number; width: number; heading: number } {
  const measured = Boolean(object.length_m && object.width_m);
  const length = measured ? (object.length_m as number) : classSize.length_m;
  const width = measured ? (object.width_m as number) : classSize.width_m;
  const heading = object.heading_rad;
  return {
    look: { shape: heading === null ? 'disc' : 'box', measured, lost },
    length,
    width,
    heading: heading ?? 0,
  };
}

function lookKey(look: FootprintLook): string {
  return `${look.shape}${Number(look.measured)}${Number(look.lost)}`;
}

/**
 * Flat footprint of one camera's observation of a road user, on the ground at the backend's
 * position and heading — no interpolation, no merging, no heading of the twin's own. It shows
 * where the perception data puts the road user, under the (smoothed) 3D model.
 */
export class Footprint {
  readonly group = new THREE.Group();
  private readonly fillMaterial = new THREE.MeshBasicMaterial({
    transparent: true,
    opacity: FILL_OPACITY,
    depthWrite: false,
  });
  private readonly solid = new THREE.LineBasicMaterial({ transparent: true, opacity: 0.95 });
  private readonly dashed = new THREE.LineDashedMaterial({
    transparent: true,
    opacity: 0.95,
    dashSize: 0.12,
    gapSize: 0.08,
  });
  private fill: THREE.Mesh | null = null;
  private outline: THREE.Line | null = null;
  private key = '';

  constructor() {
    this.group.position.z = FOOTPRINT_Z;
    this.group.renderOrder = 2;
  }

  /** Places the footprint; `x`, `y` in the site frame. */
  update(
    x: number,
    y: number,
    heading: number,
    length: number,
    width: number,
    look: FootprintLook,
    className: string,
  ): void {
    const key = lookKey(look);
    if (key !== this.key) {
      this.build(look);
      this.key = key;
    }
    const color = look.lost ? LOST_COLOR : classColor(className);
    this.fillMaterial.color.setHex(color);
    this.solid.color.setHex(color);
    this.dashed.color.setHex(color);
    this.fillMaterial.opacity = look.lost ? FILL_OPACITY * 0.5 : FILL_OPACITY;
    this.group.position.set(x, y, FOOTPRINT_Z);
    if (look.shape === 'box') {
      this.group.rotation.z = heading;
      this.group.scale.set(length, width, 1);
    } else {
      // Without heading the footprint is a disc as large as the object could reach.
      const radius = Math.max(Math.max(length, width) / 2, MIN_RADIUS_M);
      this.group.rotation.z = 0;
      this.group.scale.set(radius * 2, radius * 2, 1);
    }
  }

  dispose(): void {
    this.disposeGeometry();
    this.fillMaterial.dispose();
    this.solid.dispose();
    this.dashed.dispose();
  }

  private disposeGeometry(): void {
    for (const child of this.group.children) {
      if (child instanceof THREE.Mesh || child instanceof THREE.Line) {
        child.geometry.dispose();
      }
    }
  }

  private build(look: FootprintLook): void {
    this.disposeGeometry();
    this.group.clear();
    // Unit shapes, scaled to the footprint: a 1 x 1 square or a disc of diameter 1.
    const points =
      look.shape === 'box'
        ? [
            new THREE.Vector3(0.5, 0.5, 0),
            new THREE.Vector3(0.5, -0.5, 0),
            new THREE.Vector3(-0.5, -0.5, 0),
            new THREE.Vector3(-0.5, 0.5, 0),
            new THREE.Vector3(0.5, 0.5, 0),
          ]
        : Array.from({ length: 41 }, (_, i) => {
            const a = (i / 40) * 2 * Math.PI;
            return new THREE.Vector3(0.5 * Math.cos(a), 0.5 * Math.sin(a), 0);
          });
    this.fill = new THREE.Mesh(
      look.shape === 'box' ? new THREE.PlaneGeometry(1, 1) : new THREE.CircleGeometry(0.5, 40),
      this.fillMaterial,
    );
    const dashed = !look.measured || look.lost;
    this.outline = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(points),
      dashed ? this.dashed : this.solid,
    );
    if (dashed) {
      this.outline.computeLineDistances();
    }
    this.group.add(this.fill, this.outline);
    if (look.shape === 'box') {
      // Front edge: a short tick at the front, as the heading is known.
      const tick = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints([
          new THREE.Vector3(0.5, 0, 0),
          new THREE.Vector3(0.25, 0, 0),
        ]),
        this.solid,
      );
      this.group.add(tick);
    }
  }
}
