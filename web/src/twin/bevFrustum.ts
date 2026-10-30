/** Extents of an orthographic camera frustum in world metres, relative to the camera centre. */
export interface FrustumExtents {
  left: number;
  right: number;
  top: number;
  bottom: number;
}

/**
 * Frustum of the top-down (BEV) orthographic camera: shows `viewHeightM` metres vertically and
 * follows the viewport aspect ratio (width / height) horizontally.
 */
export function bevFrustum(viewHeightM: number, aspect: number): FrustumExtents {
  if (!(viewHeightM > 0) || !(aspect > 0)) {
    throw new RangeError('view height and aspect ratio must be positive');
  }
  const halfHeight = viewHeightM / 2;
  const halfWidth = halfHeight * aspect;
  return { left: -halfWidth, right: halfWidth, top: halfHeight, bottom: -halfHeight };
}
