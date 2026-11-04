import { describe, expect, it } from 'vitest';

import {
  BevWorld,
  type BevTrack,
  duplicateCandidates,
  headingOf,
  motionOf,
} from '../src/bev/bevModel';
import { cameraCoverage, deduplicate } from '../src/bev/bevView';
import {
  covarianceEllipse,
  footprintCorners,
  pointInPolygon,
  polygonAxis,
  type Vec,
} from '../src/bev/geometry';
import { LaneAnalytics, measureQueue } from '../src/bev/laneAnalytics';
import { FrameStats, quantile } from '../src/bev/perf';
import type { ZoneCollection } from '../src/net/client';
import type { WorldObject, WorldState } from '../src/net/schema';

function object(id: number, x: number, y: number, extra: Partial<WorldObject> = {}): WorldObject {
  return {
    id,
    class: 'car',
    vru: false,
    rider: false,
    x,
    y,
    vx: 0,
    vy: 0,
    heading_rad: null,
    speed_mps: 0,
    speed_std_mps: 0.1,
    confidence: 0.9,
    zones: [],
    turning: false,
    validated: true,
    ...extra,
  };
}

function state(frame: number, objects: WorldObject[], timestampS = frame * 0.1): WorldState {
  return {
    schema_version: 1,
    scene_id: 's',
    frame,
    timestamp_ns: Math.round(timestampS * 1e9),
    source: 'live',
    objects,
    events: [],
  };
}

function track(id: number, x: number, y: number, extra: Partial<BevTrack> = {}): BevTrack {
  const o = object(id, x, y);
  return {
    key: `cam:${id}`,
    site: 's',
    camera: 'cam',
    object: o,
    frame: 0,
    timestampNs: 0,
    ageS: 10,
    heading: null,
    headingStatus: 'unknown',
    headingNote: '',
    length: 4.5,
    width: 1.8,
    sizeMeasured: false,
    motion: 'stopped',
    stoppedS: 5,
    trail: [],
    possibleIdSwitch: false,
    ...extra,
  };
}

// A lane 60 m long along +x, 3.5 m wide, from x = 0 to x = 60.
const LANE: Vec[] = [
  [0, 0],
  [60, 0],
  [60, 3.5],
  [0, 3.5],
];

function zones(): ZoneCollection {
  return {
    type: 'FeatureCollection',
    features: [
      {
        properties: { id: 'lane_1', type: 'lane' },
        geometry: { type: 'Polygon', coordinates: [LANE.map(([x, y]) => [x, y])] },
      },
    ],
  };
}

describe('BEV geometry', () => {
  it('builds footprints along the heading', () => {
    const corners = footprintCorners(0, 0, Math.PI / 2, 4, 2);
    const ys = corners.map((c) => c[1]);
    const xs = corners.map((c) => c[0]);
    expect(Math.max(...ys)).toBeCloseTo(2);
    expect(Math.max(...xs)).toBeCloseTo(1);
  });

  it('turns a covariance into its 2-sigma ellipse', () => {
    const e = covarianceEllipse([4, 0, 1], 2);
    expect(e?.major).toBeCloseTo(4);
    expect(e?.minor).toBeCloseTo(2);
    expect(e?.angle).toBeCloseTo(0);
    const rotated = covarianceEllipse([1, 0, 4], 1);
    expect(Math.abs(rotated?.angle ?? 0)).toBeCloseTo(Math.PI / 2);
    expect(covarianceEllipse([Number.NaN, 0, 1])).toBeNull();
  });

  it('finds the long axis of a lane and points inside it', () => {
    const axis = polygonAxis(LANE);
    expect(axis).not.toBeNull();
    expect(Math.abs(axis?.ux ?? 0)).toBeCloseTo(1);
    expect((axis?.max ?? 0) - (axis?.min ?? 0)).toBeCloseTo(60);
    expect(axis?.width).toBeCloseTo(3.5);
    expect(pointInPolygon(30, 1, LANE)).toBe(true);
    expect(pointInPolygon(30, 5, LANE)).toBe(false);
  });

  it('projects the camera image border onto the ground, cut at a range', () => {
    // 0.1 m per pixel, no perspective: the coverage is the image rectangle on the ground.
    const h = [
      [0.1, 0, 0],
      [0, 0.1, 0],
      [0, 0, 1],
    ];
    const coverage = cameraCoverage(h, [100, 50], [0, 0], 8);
    expect(coverage.length).toBeGreaterThan(4);
    expect(Math.max(...coverage.map(([x, y]) => Math.hypot(x, y)))).toBeLessThanOrEqual(8 + 1e-9);
  });
});

describe('BEV headings', () => {
  it('never turns a missing heading into 0°', () => {
    const h = headingOf(object(1, 0, 0));
    expect(h.heading).toBeNull();
    expect(h.status).toBe('unknown');
  });

  it('tells measured, held and unreported headings apart', () => {
    expect(headingOf(object(1, 0, 0, { heading_rad: 1, heading_source: 'keypoints' })).status).toBe(
      'measured',
    );
    expect(headingOf(object(1, 0, 0, { heading_rad: 1, heading_source: 'held' })).status).toBe(
      'held',
    );
    // An older server: trusted only where the track clearly moves that way.
    const moving = object(1, 0, 0, { heading_rad: 0, vx: 8, speed_mps: 8 });
    expect(headingOf(moving).status).toBe('measured');
    const standing = object(1, 0, 0, { heading_rad: 0 });
    expect(headingOf(standing).status).toBe('held');
  });

  it('uses a lane direction for vehicles without heading only where it is known', () => {
    const world = new BevWorld();
    world.laneHeading = (_site, x) =>
      x > 0 ? { heading: 2, lane: 'lane_1', source: 'osm' } : null;
    world.push('s', 'cam', state(1, [object(1, 5, 1), object(2, -5, 1)]));
    const [inLane, outside] = world.tracks('s');
    expect(inLane?.headingStatus).toBe('lane');
    expect(inLane?.heading).toBe(2);
    expect(outside?.headingStatus).toBe('unknown');
    expect(outside?.heading).toBeNull();
  });

  it('says nothing about stopping while the speed is uncertain', () => {
    expect(motionOf(object(1, 0, 0, { speed_mps: 0.2, speed_std_mps: 3 }))).toBe('uncertain');
    expect(motionOf(object(1, 0, 0, { speed_mps: 0.2 }))).toBe('stopped');
    expect(motionOf(object(1, 0, 0, { speed_mps: 5 }))).toBe('moving');
  });
});

describe('BEV world', () => {
  it('keeps raw trajectories and drops them on a seek', () => {
    const world = new BevWorld();
    for (let f = 1; f <= 5; f++) {
      world.push('s', 'cam', state(f, [object(1, f, 0)]));
    }
    expect(world.tracks('s')[0]?.trail.map((p) => p.x)).toEqual([1, 2, 3, 4, 5]);
    world.push('s', 'cam', state(2, [object(1, 9, 0)]));
    expect(world.tracks('s')[0]?.trail.map((p) => p.x)).toEqual([9]);
  });

  it('flags a new track where another just got lost as a possible ID switch', () => {
    const world = new BevWorld();
    world.push('s', 'cam', state(1, [object(1, 10, 0)]));
    world.push('s', 'cam', state(2, [object(7, 10.5, 0)]));
    world.push('s', 'cam', state(3, [object(7, 11, 0), object(9, 40, 0)]));
    const tracks = world.tracks('s');
    expect(tracks.find((t) => t.object.id === 7)?.possibleIdSwitch).toBe(true);
    expect(tracks.find((t) => t.object.id === 9)?.possibleIdSwitch).toBe(false);
    expect(world.quality('s')[0]?.idSwitches).toBe(1);
  });

  it('measures how long a vehicle stands in camera time', () => {
    const world = new BevWorld();
    world.push('s', 'cam', state(1, [object(1, 0, 0, { speed_mps: 5, vx: 5 })], 0));
    world.push('s', 'cam', state(2, [object(1, 1, 0)], 1));
    world.push('s', 'cam', state(3, [object(1, 1, 0)], 4));
    expect(world.tracks('s')[0]?.stoppedS).toBeCloseTo(3);
  });

  it('pairs likely duplicates of different cameras with their time offset', () => {
    const a = track(1, 0, 0, { camera: 'a', key: 'a:1', timestampNs: 1e9 });
    const b = track(1, 1, 0, { camera: 'b', key: 'b:1', timestampNs: 1.04e9 });
    const far = track(2, 20, 0, { camera: 'b', key: 'b:2' });
    const same = track(3, -0.5, 0, { camera: 'a', key: 'a:3' });
    const pairs = duplicateCandidates([a, b, far, same]);
    expect(pairs).toHaveLength(1);
    expect(pairs[0]?.dtS).toBeCloseTo(-0.04);
    expect(deduplicate([a, b, far, same], pairs)).toHaveLength(3);
    // Too far apart in time: flagged, but not merged.
    const late = { ...b, timestampNs: 2e9 };
    expect(deduplicate([a, late], duplicateCandidates([a, late]))).toHaveLength(2);
  });
});

describe('lane status', () => {
  const axis = polygonAxis(LANE);

  it('chains standing vehicles from the lane end into a queue', () => {
    if (!axis) {
      throw new Error('no axis');
    }
    // Travel towards +x: the lane end (stop line stand-in) is at x = 60.
    const queue = measureQueue(axis, 0, [
      track(1, 57, 1.7),
      track(2, 51, 1.7),
      track(3, 45, 1.7),
      // 20 m gap: not part of the queue.
      track(4, 20, 1.7),
    ]);
    expect(queue.members.map((t) => t.object.id)).toEqual([1, 2, 3]);
    expect(queue.lengthM).toBeCloseTo(60 - (45 - 2.25));
    // The same vehicles seen against the direction of travel form no queue at the far end.
    expect(measureQueue(axis, Math.PI, [track(1, 57, 1.7)]).members).toHaveLength(0);
  });

  it('reports no queue while the lane direction is unknown', () => {
    const lanes = new LaneAnalytics(zones());
    const status = lanes.update([track(1, 57, 1.7)], 1e9);
    expect(status.lanes[0]?.direction).toBeNull();
    expect(status.lanes[0]?.queueM).toBeNull();
    expect(status.lanes[0]?.stopped).toBe(1);
    expect(status.lanes[0]?.state).toBe('unknown');
  });

  it('learns the direction from tracks that moved along the lane', () => {
    const lanes = new LaneAnalytics(zones());
    const movers = [1, 2, 3, 4, 5].map((id) =>
      track(id, 10 * id, 1.7, {
        key: `cam:${id}`,
        motion: 'moving',
        stoppedS: null,
        object: object(id, 10 * id, 1.7, { vx: -8, speed_mps: 8, speed_std_mps: 0.2 }),
      }),
    );
    lanes.update(movers, 1e9);
    const direction = lanes.direction('lane_1');
    expect(direction?.source).toBe('observed');
    expect(Math.cos(direction?.angle ?? 0)).toBeCloseTo(-1);
    // Now the stop end is x = 0.
    const status = lanes.update([track(9, 3, 1.7)], 2e9);
    expect(status.lanes[0]?.queueVehicles).toBe(1);
    expect(status.lanes[0]?.queueM).toBeCloseTo(3 + 2.25);
    expect(status.lanes[0]?.maxWaitS).toBe(5);
  });

  it('takes a one-way OpenStreetMap road along the lane, not one across it', () => {
    const along = new LaneAnalytics(zones());
    along.setRoadLookup(() => ({ angle: 0.1, oneway: true }));
    expect(along.direction('lane_1')?.source).toBe('osm');
    const across = new LaneAnalytics(zones());
    across.setRoadLookup(() => ({ angle: Math.PI / 2, oneway: true }));
    expect(across.direction('lane_1')).toBeNull();
    const twoWay = new LaneAnalytics(zones());
    twoWay.setRoadLookup(() => ({ angle: 0, oneway: false }));
    expect(twoWay.direction('lane_1')).toBeNull();
  });
});

describe('frame statistics', () => {
  it('reports frame rate and render cost', () => {
    const stats = new FrameStats();
    for (let i = 0; i < 11; i++) {
      stats.record(i * 20, 2 + (i % 2));
    }
    const s = stats.summary();
    expect(s.fps).toBeCloseTo(50);
    expect(s.costMeanMs).toBeCloseTo(2 + 5 / 11);
    expect(quantile([1, 2, 3, 4], 0.5)).toBeCloseTo(2.5);
  });
});
