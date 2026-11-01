import { describe, expect, it } from 'vitest';

import type { WorldObject } from '../src/net/schema';
import { type CameraTrack, CameraFusion } from '../src/twin/fusion';
import { geoOffset } from '../src/twin/twinView';
import { parseLatLon } from '../src/ui/latLon';

function object(id: number, vru = false, confidence = 0.8, vx = 5): WorldObject {
  return {
    id,
    class: vru ? 'pedestrian' : 'car',
    vru,
    rider: false,
    x: 0,
    y: 0,
    vx,
    vy: 0,
    heading_rad: 0,
    speed_mps: Math.abs(vx),
    speed_std_mps: 0.2,
    confidence,
    zones: [],
    turning: false,
    validated: true,
  };
}

function track(camera: string, o: WorldObject, x: number, y: number): CameraTrack {
  return { camera, object: o, x, y, heading: 0 };
}

describe('camera fusion', () => {
  it('merges close tracks of two cameras into one road user', () => {
    const fusion = new CameraFusion();
    const fused = fusion.fuse([
      track('cam1', object(1, false, 0.9), 10, 0),
      track('cam2', object(7, false, 0.6), 11, 0.5),
    ]);
    expect(fused).toHaveLength(1);
    expect(fused[0]?.cameras.sort()).toEqual(['cam1', 'cam2']);
    // Confidence-weighted position, the more confident track's identity.
    expect(fused[0]?.x).toBeCloseTo(10.4, 5);
    expect(fused[0]?.object.id).toBe(1);
    expect(fusion.keyOf('cam2', 7)).toBe(fused[0]?.key);
  });

  it('keeps tracks of the same camera, far apart or of another kind separate', () => {
    const fusion = new CameraFusion();
    const fused = fusion.fuse([
      track('cam1', object(1), 0, 0),
      track('cam1', object(2), 1, 0),
      track('cam2', object(3), 30, 0),
      track('cam2', object(4, true, 0.8, 1), 0.5, 0),
    ]);
    expect(fused).toHaveLength(4);
  });

  it('does not merge tracks that move differently', () => {
    const fusion = new CameraFusion();
    const fused = fusion.fuse([
      track('cam1', object(1, false, 0.8, 8), 0, 0),
      track('cam2', object(2, false, 0.8, -8), 0.5, 0),
    ]);
    expect(fused).toHaveLength(2);
  });

  it('keeps the key of a road user while cameras hand it over', () => {
    const fusion = new CameraFusion();
    const first = fusion.fuse([track('cam1', object(1), 0, 0)])[0]?.key;
    const both = fusion.fuse([track('cam1', object(1), 1, 0), track('cam2', object(5), 1.5, 0)]);
    expect(both[0]?.key).toBe(first);
    // A little further apart than a new match allows: the established match holds.
    const apart = fusion.fuse([track('cam1', object(1), 2, 0), track('cam2', object(5), 5.5, 0)]);
    expect(apart).toHaveLength(1);
    const handed = fusion.fuse([track('cam2', object(5), 6, 0)]);
    expect(handed[0]?.key).toBe(first);
  });
});

describe('site placement', () => {
  it('places sites east and north of the origin in metres', () => {
    const origin = { lat_deg: 48.15, lon_deg: 11.55 };
    const east = geoOffset(origin, { lat_deg: 48.15, lon_deg: 11.56 });
    const north = geoOffset(origin, { lat_deg: 48.16, lon_deg: 11.55 });
    expect(east.x).toBeCloseTo(742, -1);
    expect(east.y).toBeCloseTo(0, 6);
    expect(north.y).toBeCloseTo(1113, -1);
  });

  it('reads coordinates pasted from a map', () => {
    expect(parseLatLon('48.137154, 11.576124')).toEqual([48.137154, 11.576124]);
    expect(parseLatLon(' 48.1 11.5 ')).toEqual([48.1, 11.5]);
    expect(parseLatLon('95, 11')).toBeNull();
    expect(parseLatLon('Munich')).toBeNull();
  });
});
