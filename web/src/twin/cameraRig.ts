import * as THREE from 'three';

import { bevFrustum } from './bevFrustum';

const CAMERA_ALTITUDE_M = 500;
const MIN_VIEW_M = 10;
const MAX_VIEW_M = 600;
const ZOOM_STEP = 1.1;

/** View of the top-down camera: centre on the ground, visible height and rotation about z. */
export interface BevViewState {
  centerX: number;
  centerY: number;
  viewHeightM: number;
  rotationRad: number;
}

/** Ground point under a canvas pixel for a top-down view (x right, y up on screen). */
export function screenToGround(
  view: BevViewState,
  pixel: { x: number; y: number },
  size: { width: number; height: number },
): { x: number; y: number } {
  const metresPerPixel = view.viewHeightM / size.height;
  const dx = (pixel.x - size.width / 2) * metresPerPixel;
  const dy = (size.height / 2 - pixel.y) * metresPerPixel;
  const c = Math.cos(view.rotationRad);
  const s = Math.sin(view.rotationRad);
  return { x: view.centerX + dx * c - dy * s, y: view.centerY + dx * s + dy * c };
}

/**
 * Orthographic top-down camera with pan (left drag), zoom towards the cursor (wheel) and rotation
 * about the vertical axis (right drag or shift + drag). The scene is z-up.
 */
export class BevCameraRig {
  readonly camera = new THREE.OrthographicCamera();
  private view: BevViewState;
  private drag: { x: number; y: number; rotate: boolean } | null = null;

  constructor(
    private readonly canvas: HTMLCanvasElement,
    initial: BevViewState,
  ) {
    this.view = { ...initial };
    this.camera.near = 1;
    this.camera.far = CAMERA_ALTITUDE_M * 2;
    canvas.addEventListener('pointerdown', (e) => this.onPointerDown(e));
    canvas.addEventListener('pointermove', (e) => this.onPointerMove(e));
    canvas.addEventListener('pointerup', () => (this.drag = null));
    canvas.addEventListener('pointerleave', () => (this.drag = null));
    canvas.addEventListener('wheel', (e) => this.onWheel(e), { passive: false });
    canvas.addEventListener('contextmenu', (e) => e.preventDefault());
    this.update();
  }

  /** Updates the projection after the canvas size changed. */
  update(): void {
    const width = this.canvas.clientWidth;
    const height = this.canvas.clientHeight;
    if (width === 0 || height === 0) {
      return;
    }
    Object.assign(this.camera, bevFrustum(this.view.viewHeightM, width / height));
    const { centerX, centerY, rotationRad } = this.view;
    this.camera.up.set(-Math.sin(rotationRad), Math.cos(rotationRad), 0);
    this.camera.position.set(centerX, centerY, CAMERA_ALTITUDE_M);
    this.camera.lookAt(centerX, centerY, 0);
    this.camera.updateProjectionMatrix();
  }

  private size(): { width: number; height: number } {
    return { width: this.canvas.clientWidth, height: this.canvas.clientHeight };
  }

  private onPointerDown(e: PointerEvent): void {
    this.drag = { x: e.offsetX, y: e.offsetY, rotate: e.button === 2 || e.shiftKey };
    this.canvas.setPointerCapture(e.pointerId);
  }

  private onPointerMove(e: PointerEvent): void {
    if (!this.drag) {
      return;
    }
    if (this.drag.rotate) {
      this.view.rotationRad += ((e.offsetX - this.drag.x) / this.size().width) * Math.PI;
    } else {
      const from = screenToGround(this.view, this.drag, this.size());
      const to = screenToGround(this.view, { x: e.offsetX, y: e.offsetY }, this.size());
      this.view.centerX -= to.x - from.x;
      this.view.centerY -= to.y - from.y;
    }
    this.drag = { ...this.drag, x: e.offsetX, y: e.offsetY };
    this.update();
  }

  private onWheel(e: WheelEvent): void {
    e.preventDefault();
    const cursor = { x: e.offsetX, y: e.offsetY };
    const before = screenToGround(this.view, cursor, this.size());
    const factor = e.deltaY > 0 ? ZOOM_STEP : 1 / ZOOM_STEP;
    this.view.viewHeightM = Math.min(
      Math.max(this.view.viewHeightM * factor, MIN_VIEW_M),
      MAX_VIEW_M,
    );
    const after = screenToGround(this.view, cursor, this.size());
    this.view.centerX += before.x - after.x;
    this.view.centerY += before.y - after.y;
    this.update();
  }
}
