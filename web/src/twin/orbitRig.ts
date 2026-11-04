import * as THREE from 'three';

const FOV_DEG = 36;
const MIN_DISTANCE_M = 12;
const MAX_DISTANCE_M = 1600;
const MIN_ELEVATION = THREE.MathUtils.degToRad(14);
const MAX_ELEVATION = THREE.MathUtils.degToRad(90);
const TILTED_ELEVATION = THREE.MathUtils.degToRad(40);
// Zoom per pixel of wheel travel (a mouse wheel notch is about 100 px).
const ZOOM_PER_PIXEL = 0.0015;
// Fraction of the remaining way to the goal view covered per second.
const EASE_RATE = 9;

/**
 * View of the orbit camera: the ground point looked at, the distance to it, the rotation about
 * the vertical axis (0: north up on screen) and the elevation angle (90°: straight down).
 */
export interface OrbitView {
  targetX: number;
  targetY: number;
  distance: number;
  azimuth: number;
  elevation: number;
}

export type ViewMode = '2d' | '3d';

/** Camera position and up vector of a view; z is up, the camera looks at the target. */
export function orbitPose(view: OrbitView): {
  position: [number, number, number];
  up: [number, number, number];
} {
  // Direction that points up on screen, projected onto the ground.
  const ux = -Math.sin(view.azimuth);
  const uy = Math.cos(view.azimuth);
  const ce = Math.cos(view.elevation);
  const se = Math.sin(view.elevation);
  return {
    position: [
      view.targetX - ux * ce * view.distance,
      view.targetY - uy * ce * view.distance,
      se * view.distance,
    ],
    // Perpendicular to the viewing direction; stays defined when looking straight down.
    up: [ux * se, uy * se, ce],
  };
}

/** Distance at which the camera shows `heightM` metres of ground vertically (looking down). */
export function distanceForHeight(heightM: number): number {
  return heightM / 2 / Math.tan(THREE.MathUtils.degToRad(FOV_DEG / 2));
}

// Elevation from which the top-down mode draws with the orthographic camera (just short of 90°,
// so the switch happens once the ease into the top-down view has arrived).
const ORTHO_FROM_ELEVATION = MAX_ELEVATION - THREE.MathUtils.degToRad(0.1);
// Depth range of the orthographic camera above the ground: covers buildings and the ground.
const ORTHO_DEPTH_ABOVE_GROUND_M = 300;

/**
 * Frustum of the orthographic top-down camera: the same ground area the perspective camera shows
 * at the target from `distance` metres, so zoom and pan carry over when the camera switches.
 */
export function orthoFrustum(
  distance: number,
  aspect: number,
): { halfWidth: number; halfHeight: number; near: number; far: number } {
  const halfHeight = distance * Math.tan(THREE.MathUtils.degToRad(FOV_DEG / 2));
  return {
    halfHeight,
    halfWidth: halfHeight * aspect,
    near: Math.max(0.5, distance - ORTHO_DEPTH_ABOVE_GROUND_M),
    far: distance + 50,
  };
}

/**
 * Camera orbiting a ground point. Left drag pans (the ground point under the cursor stays under
 * it), the wheel zooms towards the cursor, right drag (or shift + drag) rotates and, in 3D, tilts.
 * Changes ease towards their goal. The 3D view is a perspective; the top-down (2D) view is drawn
 * orthographically once it looks straight down, so tall objects show no parallax there.
 */
export class OrbitRig {
  private readonly perspective = new THREE.PerspectiveCamera(FOV_DEG, 1, 0.5, 6000);
  private readonly orthographic = new THREE.OrthographicCamera(-1, 1, 1, -1, 0.5, 6000);
  private readonly current: OrbitView;
  private readonly goal: OrbitView;
  private readonly home: OrbitView;
  private mode: ViewMode = '3d';
  private drag: { x: number; y: number; rotate: boolean; ground: THREE.Vector3 | null } | null =
    null;
  private readonly raycaster = new THREE.Raycaster();
  private readonly plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
  /** True while the user drags; hover feedback is suspended meanwhile. */
  dragging = false;
  /** False while another tool (map alignment) owns pointer drags; the wheel still zooms. */
  interactive = true;

  constructor(
    private readonly canvas: HTMLCanvasElement,
    initial: Omit<OrbitView, 'elevation'>,
  ) {
    this.home = { ...initial, elevation: TILTED_ELEVATION };
    this.current = { ...this.home };
    this.goal = { ...this.home };
    canvas.addEventListener('pointerdown', (e) => this.onPointerDown(e));
    canvas.addEventListener('pointermove', (e) => this.onPointerMove(e));
    canvas.addEventListener('pointerup', () => this.endDrag());
    canvas.addEventListener('pointercancel', () => this.endDrag());
    canvas.addEventListener('wheel', (e) => this.onWheel(e), { passive: false });
    canvas.addEventListener('contextmenu', (e) => e.preventDefault());
    this.apply();
  }

  get viewMode(): ViewMode {
    return this.mode;
  }

  /** The camera to draw with: orthographic in the settled top-down view, else perspective. */
  get camera(): THREE.PerspectiveCamera | THREE.OrthographicCamera {
    return this.orthographicActive ? this.orthographic : this.perspective;
  }

  get orthographicActive(): boolean {
    return this.mode === '2d' && this.current.elevation >= ORTHO_FROM_ELEVATION;
  }

  get distance(): number {
    return this.current.distance;
  }

  get target(): { x: number; y: number } {
    return { x: this.current.targetX, y: this.current.targetY };
  }

  /** Top-down map (2D) or tilted perspective (3D); eases into the new elevation. */
  setMode(mode: ViewMode): void {
    this.mode = mode;
    this.goal.elevation = mode === '2d' ? MAX_ELEVATION : TILTED_ELEVATION;
  }

  /** Back to the initial view, keeping the mode. */
  recenter(): void {
    Object.assign(this.goal, this.home, {
      elevation: this.mode === '2d' ? MAX_ELEVATION : TILTED_ELEVATION,
    });
  }

  /** Eases to another view (keeping the elevation), which recenter() then returns to. */
  flyTo(view: Omit<OrbitView, 'elevation'>): void {
    Object.assign(this.home, view);
    Object.assign(this.goal, view);
  }

  /** Moves the view by (dx, dy) at once, e.g. because the scene under it moved. */
  shift(dx: number, dy: number): void {
    for (const view of [this.current, this.goal, this.home]) {
      view.targetX += dx;
      view.targetY += dy;
    }
    this.apply();
  }

  /** Eases the view towards its goal; call once per frame. */
  tick(dt: number): void {
    const k = 1 - Math.exp(-EASE_RATE * dt);
    let moving = false;
    for (const key of ['targetX', 'targetY', 'distance', 'azimuth', 'elevation'] as const) {
      const delta = this.goal[key] - this.current[key];
      if (Math.abs(delta) > 1e-5) {
        this.current[key] += delta * k;
        moving = true;
      }
    }
    if (moving) {
      this.apply();
    }
  }

  /** Updates the projection after the canvas size changed. */
  resize(): void {
    const { clientWidth: width, clientHeight: height } = this.canvas;
    if (width > 0 && height > 0) {
      this.perspective.aspect = width / height;
      this.apply();
    }
  }

  /** Ground point (z = 0) under a canvas pixel, or null above the horizon. */
  groundAt(x: number, y: number): THREE.Vector3 | null {
    const ndc = new THREE.Vector2(
      (x / this.canvas.clientWidth) * 2 - 1,
      -(y / this.canvas.clientHeight) * 2 + 1,
    );
    this.raycaster.setFromCamera(ndc, this.camera);
    return this.raycaster.ray.intersectPlane(this.plane, new THREE.Vector3());
  }

  /** Normalized device coordinates of a pointer event, for picking. */
  pointer(e: PointerEvent): THREE.Vector2 {
    return new THREE.Vector2(
      (e.offsetX / this.canvas.clientWidth) * 2 - 1,
      -(e.offsetY / this.canvas.clientHeight) * 2 + 1,
    );
  }

  private apply(): void {
    const { position, up } = orbitPose(this.current);
    for (const camera of [this.perspective, this.orthographic]) {
      camera.position.set(...position);
      camera.up.set(...up);
      camera.lookAt(this.current.targetX, this.current.targetY, 0);
    }
    this.perspective.near = Math.max(0.5, this.current.distance / 200);
    this.perspective.far = this.current.distance * 30;
    const frustum = orthoFrustum(this.current.distance, this.perspective.aspect);
    Object.assign(this.orthographic, {
      left: -frustum.halfWidth,
      right: frustum.halfWidth,
      top: frustum.halfHeight,
      bottom: -frustum.halfHeight,
      near: frustum.near,
      far: frustum.far,
    });
    for (const camera of [this.perspective, this.orthographic]) {
      camera.updateProjectionMatrix();
      camera.updateMatrixWorld();
    }
  }

  private onPointerDown(e: PointerEvent): void {
    if (!this.interactive) {
      return;
    }
    const rotate = e.button === 2 || e.shiftKey;
    this.drag = {
      x: e.offsetX,
      y: e.offsetY,
      rotate,
      ground: rotate ? null : this.groundAt(e.offsetX, e.offsetY),
    };
    this.canvas.setPointerCapture(e.pointerId);
    this.canvas.classList.add(rotate ? 'rotating' : 'panning');
  }

  private endDrag(): void {
    this.drag = null;
    this.dragging = false;
    this.canvas.classList.remove('rotating', 'panning');
  }

  private onPointerMove(e: PointerEvent): void {
    if (!this.drag) {
      return;
    }
    this.dragging = true;
    const dx = e.offsetX - this.drag.x;
    const dy = e.offsetY - this.drag.y;
    if (this.drag.rotate) {
      this.goal.azimuth += (dx / this.canvas.clientWidth) * Math.PI;
      this.current.azimuth = this.goal.azimuth;
      if (this.mode === '3d') {
        this.goal.elevation = THREE.MathUtils.clamp(
          this.goal.elevation + (dy / this.canvas.clientHeight) * Math.PI * 0.6,
          MIN_ELEVATION,
          MAX_ELEVATION,
        );
        this.current.elevation = this.goal.elevation;
      }
      this.apply();
    } else if (this.drag.ground) {
      const now = this.groundAt(e.offsetX, e.offsetY);
      if (now) {
        const shiftX = this.drag.ground.x - now.x;
        const shiftY = this.drag.ground.y - now.y;
        for (const view of [this.current, this.goal]) {
          view.targetX += shiftX;
          view.targetY += shiftY;
        }
        this.apply();
      }
    }
    this.drag.x = e.offsetX;
    this.drag.y = e.offsetY;
  }

  private onWheel(e: WheelEvent): void {
    e.preventDefault();
    const pixels = e.deltaMode === WheelEvent.DOM_DELTA_LINE ? e.deltaY * 33 : e.deltaY;
    const factor = Math.exp(THREE.MathUtils.clamp(pixels, -300, 300) * ZOOM_PER_PIXEL);
    const distance = THREE.MathUtils.clamp(
      this.goal.distance * factor,
      MIN_DISTANCE_M,
      MAX_DISTANCE_M,
    );
    const applied = distance / this.goal.distance;
    // Zoom towards the cursor: the target moves along the line to the ground point under it.
    const cursor = this.groundAt(e.offsetX, e.offsetY);
    if (cursor) {
      this.goal.targetX = cursor.x + (this.goal.targetX - cursor.x) * applied;
      this.goal.targetY = cursor.y + (this.goal.targetY - cursor.y) * applied;
    }
    this.goal.distance = distance;
  }
}
