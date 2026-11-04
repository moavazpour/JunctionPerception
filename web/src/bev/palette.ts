/**
 * Colours of the bird's-eye view, matched to the console themes (src/styles/theme.css): a dark
 * blue-grey night palette and a light day palette. Vulnerable road users stay vivid in both,
 * vehicles are subdued.
 */
export type Theme = 'dark' | 'light';

/** Map layer colours (CSS). */
export interface MapPalette {
  road: string;
  path: string;
  marking: string;
  building: string;
}

const MAP: Record<Theme, MapPalette> = {
  dark: { road: '#24344f', path: '#1b2841', marking: '#c4d2e8', building: '#1f2b42' },
  light: { road: '#b7bfcb', path: '#d8dde5', marking: '#ffffff', building: '#f6f8fb' },
};

const CLASS_COLORS: Record<Theme, Record<string, string>> = {
  dark: {
    car: '#8a9dbb',
    van: '#7f95b6',
    truck: '#7086a8',
    trailer: '#63789b',
    bus: '#7891b5',
    emergency_vehicle: '#ff8a3d',
    pedestrian: '#ffb547',
    bicycle: '#34d399',
    motorcycle: '#a68bfa',
    other: '#94a3b8',
  },
  light: {
    car: '#6e809d',
    van: '#647996',
    truck: '#5a6e8e',
    trailer: '#4f6382',
    bus: '#60779b',
    emergency_vehicle: '#f06a14',
    pedestrian: '#f59e0b',
    bicycle: '#10a36e',
    motorcycle: '#7c5ce6',
    other: '#6b7a90',
  },
};

let current: Theme = 'dark';

/** Theme the view draws with. */
export function setPaletteTheme(theme: Theme): void {
  current = theme;
}

export function mapPalette(): MapPalette {
  return MAP[current];
}

/** CSS colour of a class for the view, the legend and the side panel. */
export function classCss(name: string): string {
  const colors = CLASS_COLORS[current];
  return colors[name] ?? colors['other'] ?? '#94a3b8';
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
