import * as THREE from 'three';

import { classColor } from './palette';

const MAX_POINTS = 48;
const MIN_STEP_M = 0.35;
const TRAIL_Z = 0.05;

interface Trail {
  points: THREE.Vector3[];
  line: THREE.Line;
  geometry: THREE.BufferGeometry;
}

/** Fading paths behind moving road users, in the colour of their class. */
export class TrailLayer {
  readonly group = new THREE.Group();
  private readonly trails = new Map<string | number, Trail>();

  /** Adds the current positions; objects that left the scene lose their trail. */
  update(positions: Map<string | number, { x: number; y: number; className: string }>): void {
    for (const [id, p] of positions) {
      let trail = this.trails.get(id);
      if (!trail) {
        trail = this.create(p.className);
        this.trails.set(id, trail);
      }
      const last = trail.points[trail.points.length - 1];
      if (last && Math.hypot(last.x - p.x, last.y - p.y) < MIN_STEP_M) {
        continue;
      }
      trail.points.push(new THREE.Vector3(p.x, p.y, TRAIL_Z));
      if (trail.points.length > MAX_POINTS) {
        trail.points.shift();
      }
      this.write(trail);
    }
    for (const [id, trail] of this.trails) {
      if (!positions.has(id)) {
        this.group.remove(trail.line);
        trail.geometry.dispose();
        (trail.line.material as THREE.Material).dispose();
        this.trails.delete(id);
      }
    }
  }

  clear(): void {
    this.update(new Map());
  }

  private create(className: string): Trail {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute(
      'position',
      new THREE.BufferAttribute(new Float32Array(MAX_POINTS * 3), 3),
    );
    const color = new THREE.Color(classColor(className));
    const colors = new Float32Array(MAX_POINTS * 4);
    geometry.setAttribute('color', new THREE.BufferAttribute(colors, 4));
    geometry.userData['color'] = color;
    geometry.setDrawRange(0, 0);
    const line = new THREE.Line(
      geometry,
      new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, depthWrite: false }),
    );
    line.frustumCulled = false;
    this.group.add(line);
    return { points: [], line, geometry };
  }

  private write(trail: Trail): void {
    const position = trail.geometry.getAttribute('position') as THREE.BufferAttribute;
    const color = trail.geometry.getAttribute('color') as THREE.BufferAttribute;
    const base = trail.geometry.userData['color'] as THREE.Color;
    const n = trail.points.length;
    trail.points.forEach((p, i) => {
      position.setXYZ(i, p.x, p.y, p.z);
      // Older points fade out.
      color.setXYZW(i, base.r, base.g, base.b, ((i + 1) / n) ** 1.6 * 0.85);
    });
    position.needsUpdate = true;
    color.needsUpdate = true;
    trail.geometry.setDrawRange(0, n);
  }
}
