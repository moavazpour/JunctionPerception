import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';

import { describe, expect, it } from 'vitest';

import { drawnSize, groundOutline, headingMeasured } from '../src/bev/bevView';
import {
  covarianceEllipse,
  footprintCorners,
  groundHomography,
  homography,
} from '../src/bev/geometry';
import {
  geoOffset,
  type MapTransform,
  rotatePlacement,
  shiftPlacement,
  siteToView,
  type SiteSpec,
  viewToSite,
} from '../src/bev/siteFrame';
import type { RoadUser, SiteState } from '../src/net/siteState';

const fixture = JSON.parse(
  readFileSync(
    fileURLToPath(new URL('../../tests/fixtures/site_state_v1.json', import.meta.url)),
    'utf8',
  ),
) as SiteState;

/** The first road user of the fixture (a car with a measured heading and its class size). */
function car(): RoadUser {
  const user = fixture.road_users[0];
  if (!user) {
    throw new Error('fixture without road users');
  }
  return structuredClone(user);
}

const enu: SiteSpec = { id: 'a', name: 'A', offset: { x: 120, y: -40 }, transform: null };
const sensor: SiteSpec = {
  id: 'b',
  name: 'B',
  offset: { x: 0, y: 0 },
  transform: { yaw_deg: -165.6, x_m: 3, y_m: -2 },
};
const placement = sensor.transform as MapTransform;

describe('BEV positions are the backend positions', () => {
  it('draws a road user of an east-north-up site at its world position plus the site offset', () => {
    const user = car();
    const [x, y] = siteToView(enu, { yaw_deg: 0, x_m: 0, y_m: 0 }, user.x, user.y);
    expect(x).toBeCloseTo(120 + user.x, 9);
    expect(y).toBeCloseTo(-40 + user.y, 9);
  });

  it('round-trips site and view coordinates of a sensor-frame site', () => {
    for (const [x, y] of [
      [0, 0],
      [-9.71, -47.92],
      [55, 12],
    ] as const) {
      const [vx, vy] = siteToView(sensor, placement, x, y);
      const [sx, sy] = viewToSite(sensor, placement, vx, vy);
      expect(sx).toBeCloseTo(x, 9);
      expect(sy).toBeCloseTo(y, 9);
    }
  });

  it('keeps distances: the view is metric', () => {
    const a = siteToView(sensor, placement, 0, 0);
    const b = siteToView(sensor, placement, 30, 40);
    expect(Math.hypot(b[0] - a[0], b[1] - a[1])).toBeCloseTo(50, 9);
  });
});

describe('map alignment', () => {
  it('shifting the placement moves every site point by the drag', () => {
    const moved = shiftPlacement(placement, 4, -3);
    for (const [x, y] of [
      [0, 0],
      [20, -7],
    ] as const) {
      const before = siteToView(sensor, placement, x, y);
      const after = siteToView(sensor, moved, x, y);
      expect(after[0] - before[0]).toBeCloseTo(4, 9);
      expect(after[1] - before[1]).toBeCloseTo(-3, 9);
    }
  });

  it('turning the placement rotates the site about the pointer', () => {
    const center: [number, number] = [10, 5];
    const angle = 0.3;
    const turned = rotatePlacement(sensor, placement, center, angle);
    const before = siteToView(sensor, placement, 25, -12);
    const after = siteToView(sensor, turned, 25, -12);
    const rx =
      center[0] +
      Math.cos(angle) * (before[0] - center[0]) -
      Math.sin(angle) * (before[1] - center[1]);
    const ry =
      center[1] +
      Math.sin(angle) * (before[0] - center[0]) +
      Math.cos(angle) * (before[1] - center[1]);
    expect(after[0]).toBeCloseTo(rx, 9);
    expect(after[1]).toBeCloseTo(ry, 9);
  });
});

describe('drawing helpers', () => {
  it('places sites by their geographic offset', () => {
    const d = geoOffset({ lat_deg: 48, lon_deg: 11 }, { lat_deg: 48.001, lon_deg: 11 });
    expect(d.x).toBeCloseTo(0, 6);
    expect(d.y).toBeCloseTo(111.3, 0);
  });

  it('draws the reported size, never a guessed class size for unknown ones', () => {
    const user = car();
    expect(drawnSize(user)).toEqual([4.5, 1.8]);
    user.length_m = null;
    user.width_m = null;
    expect(drawnSize(user)).toEqual([1, 1]);
  });

  it('marks held headings as not measured now', () => {
    const user = car();
    expect(headingMeasured(user)).toBe(true);
    user.heading_source = 'held';
    expect(headingMeasured(user)).toBe(false);
  });

  it('builds footprints along the heading and 2-sigma ellipses', () => {
    const corners = footprintCorners(0, 0, Math.PI / 2, 4, 2);
    expect(Math.max(...corners.map(([, y]) => y))).toBeCloseTo(2, 9);
    const e = covarianceEllipse([4, 0, 1], 2);
    expect(e?.major).toBeCloseTo(4, 9);
    expect(e?.minor).toBeCloseTo(2, 9);
  });
});

describe('ground projection', () => {
  // TUMTraf south2 as fitted by the console: normalised to h33 = 1, its image corner (0, 0) lies
  // above the horizon, so every ground pixel has a negative w.
  const south2 = [
    [0.05541632877628101, -0.27562359212014115, 140.54657655162288],
    [-0.12095883376345197, 0.04796235951105448, 196.17497813342987],
    [-0.0001018849973311642, -0.012981938854761793, 1],
  ];
  const size: [number, number] = [1920, 1200];

  it('projects ground pixels of a homography with a negative w on the ground', () => {
    expect(homography(south2, 960, 900)).toBeNull();
    const ground = groundHomography(south2, size);
    const p = homography(ground, 960, 900);
    expect(p).not.toBeNull();
    // The same point as the unscaled matrix gives: orientation does not move it.
    expect(p?.[0]).toBeCloseTo(-54.314980731274375 / -10.781554566723532, 9);
    expect(p?.[1]).toBeCloseTo(123.22062128046501 / -10.781554566723532, 9);
    // Above the horizon stays invisible.
    expect(homography(ground, 0, 0)).toBeNull();
  });

  it('keeps a homography that is already positive on the ground', () => {
    const flipped = south2.map((row) => row.map((v) => -v));
    expect(groundHomography(flipped, size)).toBe(flipped);
  });
});

describe('detection areas on the ground', () => {
  // A camera 10 m above the origin looking along +y: v = 600 is the horizon.
  const h = [
    [0.01, 0, -9.6],
    [0, 0, 10],
    [0, 0.001, -0.6],
  ];

  it('projects an area below the horizon and keeps it within range', () => {
    const area = groundOutline(
      h,
      [
        [800, 800],
        [1100, 800],
        [1100, 1000],
        [800, 1000],
      ],
      [0, 0],
    );
    expect(area.length).toBeGreaterThan(8);
    for (const [, y] of area) {
      expect(y).toBeGreaterThanOrEqual(25 - 1e-9);
      expect(y).toBeLessThanOrEqual(50 + 1e-9);
    }
  });

  it('leaves out the part above the horizon', () => {
    const area = groundOutline(
      h,
      [
        [800, 400],
        [1100, 400],
        [1100, 1000],
        [800, 1000],
      ],
      [0, 0],
      100,
    );
    for (const [x, y] of area) {
      expect(Math.hypot(x, y)).toBeLessThanOrEqual(100 + 1e-9);
      expect(y).toBeGreaterThan(0);
    }
  });
});
