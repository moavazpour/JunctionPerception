import { describe, expect, it } from 'vitest';

import type { WorldObject } from '../src/net/schema';
import { footprintOf } from '../src/twin/footprint';
import { type CameraTrack, CameraFusion, DEFAULT_MAX_SKEW_MS } from '../src/twin/fusion';
import { distanceForHeight, orthoFrustum } from '../src/twin/orbitRig';

function object(id: number, extra: Partial<WorldObject> = {}): WorldObject {
  return {
    id,
    class: 'car',
    vru: false,
    rider: false,
    x: 0,
    y: 0,
    vx: 5,
    vy: 0,
    heading_rad: 0,
    speed_mps: 5,
    speed_std_mps: 0.2,
    confidence: 0.8,
    zones: [],
    turning: false,
    validated: true,
    ...extra,
  };
}

function track(camera: string, id: number, x: number, timeMs?: number): CameraTrack {
  return {
    camera,
    object: object(id),
    x,
    y: 0,
    heading: 0,
    timestampNs: timeMs === undefined ? undefined : timeMs * 1e6,
  };
}

describe('timestamp-gated camera fusion', () => {
  it('merges tracks captured within the threshold and keeps their members', () => {
    const fusion = new CameraFusion();
    expect(fusion.maxSkewMs).toBe(DEFAULT_MAX_SKEW_MS);
    const fused = fusion.fuse([track('a', 1, 10, 1000), track('b', 2, 10.5, 1080)]);
    expect(fused).toHaveLength(1);
    expect(fused[0]?.members.map((m) => m.camera).sort()).toEqual(['a', 'b']);
    expect(fusion.rejected).toHaveLength(0);
    expect(fusion.counts).toEqual({ merged: 1, rejected: 0 });
  });

  it('keeps tracks captured too far apart separate and reports the pair', () => {
    const fusion = new CameraFusion(100);
    const fused = fusion.fuse([track('a', 1, 10, 1000), track('b', 2, 10.5, 1150)]);
    expect(fused).toHaveLength(2);
    expect(fusion.rejected).toHaveLength(1);
    expect(fusion.rejected[0]?.skewMs).toBeCloseTo(150);
    expect(fusion.counts).toEqual({ merged: 0, rejected: 1 });
    // Infinity restores merging regardless of time.
    fusion.maxSkewMs = Infinity;
    expect(fusion.fuse([track('a', 1, 10, 1000), track('b', 2, 10.5, 1150)])).toHaveLength(1);
  });

  it('never lets a group of three cameras span more than the threshold', () => {
    const fusion = new CameraFusion(100);
    // a–b and b–c are each 80 ms apart, a–c 160 ms: c must not join a's group through b.
    const fused = fusion.fuse([
      track('a', 1, 10, 1000),
      track('b', 2, 10.4, 1080),
      track('c', 3, 10.8, 1160),
    ]);
    const sizes = fused.map((f) => f.members.length).sort();
    expect(sizes).toEqual([1, 2]);
    expect(fusion.rejected).toHaveLength(1);
  });

  it('does not check tracks without capture times', () => {
    const fusion = new CameraFusion(1);
    expect(fusion.fuse([track('a', 1, 10), track('b', 2, 10.5)])).toHaveLength(1);
  });
});

describe('orthographic top-down camera', () => {
  it('shows the same ground area as the perspective camera at the target', () => {
    const height = 140;
    const frustum = orthoFrustum(distanceForHeight(height), 2);
    expect(frustum.halfHeight * 2).toBeCloseTo(height);
    expect(frustum.halfWidth).toBeCloseTo(height);
    // The ground (z = 0, at the camera distance) lies inside the depth range.
    const distance = distanceForHeight(height);
    expect(frustum.near).toBeLessThan(distance);
    expect(frustum.far).toBeGreaterThan(distance);
  });
});

describe('ground footprint', () => {
  const car = { length_m: 4.5, width_m: 1.8 };

  it('draws the backend heading and the measured size', () => {
    const f = footprintOf(object(1, { heading_rad: 1.2, length_m: 4.1, width_m: 1.7 }), car);
    expect(f.look).toEqual({ shape: 'box', measured: true, lost: false });
    expect(f.heading).toBe(1.2);
    expect([f.length, f.width]).toEqual([4.1, 1.7]);
  });

  it('draws a disc, never a box at 0°, without a heading, at the class size', () => {
    const f = footprintOf(object(1, { heading_rad: null }), car, true);
    expect(f.look).toEqual({ shape: 'disc', measured: false, lost: true });
    expect([f.length, f.width]).toEqual([4.5, 1.8]);
  });
});
