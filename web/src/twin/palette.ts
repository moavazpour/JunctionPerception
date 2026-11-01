/**
 * Colours of the 3D twin, matched to the console theme (src/styles/theme.css): a dark blue-grey
 * ground, subdued vehicles, vivid road users at risk (VRUs) and red for conflicts.
 */
export const TWIN = {
  background: 0x080d17,
  ground: 0x0a101b,
  gridMinor: 0x111a2b,
  gridMajor: 0x1a2740,
  road: 0x24344f,
  path: 0x1b2841,
  marking: 0xc4d2e8,
  building: 0x1f2b42,
  buildingEdge: 0x4a6490,
  crosswalk: 0x2fd4f0,
  lane: 0x3d9bff,
  intersection: 0xa68bfa,
  camera: 0xe7eef8,
  cameraView: 0x3d9bff,
  conflict: 0xff4d5e,
  shadow: 0x000000,
  sky: 0x9bb7e0,
} as const;

/** Body colour of each twin class; cabins and glass are a darker shade of it. */
export const CLASS_COLORS: Record<string, number> = {
  car: 0x8a9dbb,
  van: 0x7f95b6,
  truck: 0x7086a8,
  trailer: 0x63789b,
  bus: 0x7891b5,
  emergency_vehicle: 0xff8a3d,
  pedestrian: 0xffb547,
  bicycle: 0x34d399,
  motorcycle: 0xa68bfa,
  other: 0x94a3b8,
};

export function classColor(name: string): number {
  return CLASS_COLORS[name] ?? CLASS_COLORS['other'] ?? 0x94a3b8;
}

/** CSS colour of a class for the legend and the side panel. */
export function classCss(name: string): string {
  return `#${classColor(name).toString(16).padStart(6, '0')}`;
}

/** Display names of the classes. */
export const CLASS_LABELS: Record<string, string> = {
  car: 'Car',
  van: 'Van',
  truck: 'Truck',
  trailer: 'Trailer',
  bus: 'Bus',
  emergency_vehicle: 'Emergency vehicle',
  pedestrian: 'Pedestrian',
  bicycle: 'Cyclist',
  motorcycle: 'Motorcyclist',
  other: 'Other',
};
