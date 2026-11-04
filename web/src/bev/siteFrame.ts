/**
 * Placement of sites in the view. Pure rendering transforms: they move what the backend
 * reports into screen space and never change a road user's state.
 *
 * Frames: each site reports positions in its own site frame (metres). The view's frame ("twin")
 * is east-north-up at the first site with a location; a site lies at its geographic offset. A
 * site in a sensor frame (benchmark) is turned so that its map, placed by `MapTransform`
 * (site = R(yaw) · map + t), is north-up.
 */
export type Vec = [number, number];

/** Placement of map layers given in their own east-north-up frame: site = R(yaw) · map + (x, y). */
export interface MapTransform {
  yaw_deg: number;
  x_m: number;
  y_m: number;
}

/** A site of the view. */
export interface SiteSpec {
  id: string;
  name: string;
  /** Where the origin of the site's world frame lies in the view (east, north in metres). */
  offset: { x: number; y: number };
  /** Map placement of a sensor-frame site; null for sites in east-north-up. */
  transform: MapTransform | null;
}

/** A traffic light of a camera, placed in its site's frame (see Regions in calibrationApi). */
export interface SignalDef {
  id: string;
  name: string;
  position: [number, number] | null;
  /** Direction the lamps face, from +x counter-clockwise. */
  heading_deg: number;
  height_m: number;
}

const EARTH_RADIUS_M = 6378137;
const RAD = Math.PI / 180;

/** East and north metres of a geographic position from an origin (local tangent plane). */
export function geoOffset(
  origin: { lat_deg: number; lon_deg: number },
  point: { lat_deg: number; lon_deg: number },
): { x: number; y: number } {
  return {
    x: (point.lon_deg - origin.lon_deg) * RAD * EARTH_RADIUS_M * Math.cos(origin.lat_deg * RAD),
    y: (point.lat_deg - origin.lat_deg) * RAD * EARTH_RADIUS_M,
  };
}

/** Site frame -> view frame. */
export function siteToView(spec: SiteSpec, transform: MapTransform, x: number, y: number): Vec {
  const { offset } = spec;
  if (spec.transform === null) {
    return [offset.x + x, offset.y + y];
  }
  // view = offset + R(-yaw) · (site - t), the inverse of the map placement.
  const yaw = -transform.yaw_deg * RAD;
  const dx = x - transform.x_m;
  const dy = y - transform.y_m;
  return [
    offset.x + Math.cos(yaw) * dx - Math.sin(yaw) * dy,
    offset.y + Math.sin(yaw) * dx + Math.cos(yaw) * dy,
  ];
}

/** View frame -> site frame (inverse of siteToView). */
export function viewToSite(spec: SiteSpec, transform: MapTransform, x: number, y: number): Vec {
  const { offset } = spec;
  if (spec.transform === null) {
    return [x - offset.x, y - offset.y];
  }
  const yaw = transform.yaw_deg * RAD;
  const dx = x - offset.x;
  const dy = y - offset.y;
  return [
    Math.cos(yaw) * dx - Math.sin(yaw) * dy + transform.x_m,
    Math.sin(yaw) * dx + Math.cos(yaw) * dy + transform.y_m,
  ];
}

/**
 * Map placement after dragging the site by `(dx, dy)` view metres over the north-up map: every
 * site point moves by that much in the view.
 */
export function shiftPlacement(t: MapTransform, dx: number, dy: number): MapTransform {
  const yaw = t.yaw_deg * RAD;
  return {
    yaw_deg: t.yaw_deg,
    x_m: t.x_m - (Math.cos(yaw) * dx - Math.sin(yaw) * dy),
    y_m: t.y_m - (Math.sin(yaw) * dx + Math.cos(yaw) * dy),
  };
}

/**
 * Map placement after turning the site by `angle` (rad, counter-clockwise) about the view point
 * `center` over the north-up map.
 */
export function rotatePlacement(
  spec: SiteSpec,
  t: MapTransform,
  center: Vec,
  angle: number,
): MapTransform {
  // view'(p) = c + R(a) (view(p) - c) with view(p) = o + R(-yaw) (p - t)
  //   => yaw' = yaw - a,  t' = t - R(yaw) (R(-a) u - u),  u = c - o.
  const ux = center[0] - spec.offset.x;
  const uy = center[1] - spec.offset.y;
  const rx = Math.cos(-angle) * ux - Math.sin(-angle) * uy - ux;
  const ry = Math.sin(-angle) * ux + Math.cos(-angle) * uy - uy;
  const yaw = t.yaw_deg * RAD;
  return {
    yaw_deg: t.yaw_deg - angle / RAD,
    x_m: t.x_m - (Math.cos(yaw) * rx - Math.sin(yaw) * ry),
    y_m: t.y_m - (Math.sin(yaw) * rx + Math.cos(yaw) * ry),
  };
}
