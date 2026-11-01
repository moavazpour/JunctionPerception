/**
 * Colours of the 3D twin, matched to the console themes (src/styles/theme.css): a dark blue-grey
 * night palette and a light day palette. Road users at risk (VRUs) stay vivid in both, vehicles
 * are subdued, conflicts are red.
 */
export type Theme = 'dark' | 'light';

export interface TwinPalette {
  background: number;
  ground: number;
  gridMinor: number;
  gridMajor: number;
  road: number;
  path: number;
  marking: number;
  building: number;
  buildingEdge: number;
  crosswalk: number;
  lane: number;
  intersection: number;
  camera: number;
  cameraView: number;
  conflict: number;
  shadow: number;
  sky: number;
  skyGround: number;
  hemisphere: number;
  sun: number;
  shadowOpacity: number;
  /** Multiplies the aerial and road map images (dims them at night). */
  imageTint: number;
}

export const PALETTES: Record<Theme, TwinPalette> = {
  dark: {
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
    sky: 0xa9c2e6,
    skyGround: 0x1a2436,
    hemisphere: 1.35,
    sun: 1.9,
    shadowOpacity: 0.35,
    imageTint: 0xd2d8e2,
  },
  light: {
    background: 0xdde4ec,
    ground: 0xedf1f5,
    gridMinor: 0xe1e7ee,
    gridMajor: 0xd0d9e4,
    road: 0xb7bfcb,
    path: 0xd8dde5,
    marking: 0xffffff,
    building: 0xf6f8fb,
    buildingEdge: 0x9eabbd,
    crosswalk: 0x0ea5c6,
    lane: 0x2683f0,
    intersection: 0x7c5ce6,
    camera: 0x2b3a52,
    cameraView: 0x2683f0,
    conflict: 0xe5364a,
    shadow: 0x1b2433,
    sky: 0xffffff,
    skyGround: 0xc9d1dc,
    hemisphere: 1.6,
    sun: 2.2,
    shadowOpacity: 0.22,
    imageTint: 0xffffff,
  },
};

/** Body colour of each twin class; cabins and glass are a darker shade of it. */
const CLASS_COLORS: Record<Theme, Record<string, number>> = {
  dark: {
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
  },
  light: {
    car: 0x6e809d,
    van: 0x647996,
    truck: 0x5a6e8e,
    trailer: 0x4f6382,
    bus: 0x60779b,
    emergency_vehicle: 0xf06a14,
    pedestrian: 0xf59e0b,
    bicycle: 0x10a36e,
    motorcycle: 0x7c5ce6,
    other: 0x6b7a90,
  },
};

let current: Theme = 'dark';

/** Theme the twin draws with; set by TwinView.setTheme. */
export function setTwinTheme(theme: Theme): void {
  current = theme;
}

export function twinPalette(): TwinPalette {
  return PALETTES[current];
}

export function classColor(name: string, theme: Theme = current): number {
  const colors = CLASS_COLORS[theme];
  return colors[name] ?? colors['other'] ?? 0x94a3b8;
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
