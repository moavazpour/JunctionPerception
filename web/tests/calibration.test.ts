import { describe, expect, it } from 'vitest';

import { View2D } from '../src/calibration/panZoom';
import { crossingSuggestions, type OsmLayer, widenSegment } from '../src/map/mapData';
import { applyHomography, invert3 } from '../src/net/calibrationApi';

describe('View2D', () => {
  it('round-trips content and screen coordinates with y up', () => {
    const view = new View2D(true);
    view.fit(-10, -10, 10, 10, 200, 100);
    const [sx, sy] = view.toScreen(3, 4);
    expect(view.toContent(sx, sy)).toEqual([3, 4]);
    // y up: larger y is higher on screen.
    expect(view.toScreen(0, 5)[1]).toBeLessThan(view.toScreen(0, 0)[1]);
  });

  it('fits a box into the canvas centred', () => {
    const view = new View2D(false);
    view.fit(0, 0, 100, 50, 200, 200);
    expect(view.scale).toBe(2);
    expect(view.toScreen(50, 25)).toEqual([100, 100]);
  });

  it('keeps the point under the cursor fixed when zooming', () => {
    const view = new View2D(true);
    view.fit(-10, -10, 10, 10, 100, 100);
    const before = view.toContent(30, 70);
    view.zoomAt(30, 70, 1.5);
    const after = view.toContent(30, 70);
    expect(after[0]).toBeCloseTo(before[0]);
    expect(after[1]).toBeCloseTo(before[1]);
  });
});

describe('homography helpers', () => {
  const h = [
    [0.5, 0.1, 3],
    [-0.2, 0.8, -1],
    [0.001, 0.002, 1],
  ];

  it('inverts a homography', () => {
    const inverse = invert3(h);
    expect(inverse).not.toBeNull();
    const mapped = applyHomography(h, 120, 80);
    expect(mapped).not.toBeNull();
    const back = applyHomography(inverse as number[][], ...(mapped as [number, number]));
    expect(back?.[0]).toBeCloseTo(120, 6);
    expect(back?.[1]).toBeCloseTo(80, 6);
  });

  it('reports singular matrices and points at infinity', () => {
    expect(
      invert3([
        [1, 2, 3],
        [2, 4, 6],
        [0, 0, 1],
      ]),
    ).toBeNull();
    expect(applyHomography(h, -1000, 0)).toBeNull();
  });
});

describe('map geometry', () => {
  it('widens a segment symmetrically', () => {
    expect(widenSegment([0, 0], [10, 0], 4)).toEqual([
      [0, 2],
      [10, 2],
      [10, -2],
      [0, -2],
    ]);
    expect(widenSegment([1, 1], [1, 1], 4)).toEqual([]);
  });

  it('suggests crosswalks across the road at crossing nodes and along crossing ways', () => {
    const osm: OsmLayer = {
      roads: [
        {
          id: 1,
          highway: 'primary',
          width_m: 10,
          points: [
            [-50, 0],
            [50, 0],
          ],
        },
      ],
      paths: [],
      crossings: [
        { id: 2, kind: 'node', crossing: 'zebra', position: [5, 0] },
        {
          id: 3,
          kind: 'way',
          crossing: 'traffic_signals',
          points: [
            [20, -6],
            [20, 6],
          ],
        },
      ],
    };
    const [node, way] = crossingSuggestions(osm);
    // Node on an east-west road: the strip runs north-south, 4 m wide, 8 m long.
    const xs = (node ?? []).map(([x]) => x);
    const ys = (node ?? []).map(([, y]) => y);
    expect(Math.max(...xs) - Math.min(...xs)).toBeCloseTo(4);
    expect(Math.max(...ys) - Math.min(...ys)).toBeCloseTo(8);
    expect(way).toHaveLength(4);
  });
});
