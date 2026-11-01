import { describe, expect, it } from 'vitest';

import { parseVideoMessage } from '../src/net/client';
import type { WorldEvent, WorldObject, WorldState } from '../src/net/schema';
import { distanceForHeight, orbitPose } from '../src/twin/orbitRig';
import { angleDelta, lerpAngle, WorldInterpolator } from '../src/twin/interpolation';
import { eventParticipants } from '../src/twin/twinView';
import { describeEvent, EventLog, formatTimestamp } from '../src/ui/events';

function object(id: number, x: number, y: number, heading: number | null): WorldObject {
  return {
    id,
    class: 'car',
    vru: false,
    rider: false,
    x,
    y,
    vx: 0,
    vy: 0,
    heading_rad: heading,
    speed_mps: 0,
    speed_std_mps: 0.1,
    confidence: 0.9,
    zones: [],
    turning: false,
    validated: true,
  };
}

function state(frame: number, objects: WorldObject[], events: WorldEvent[] = []): WorldState {
  return {
    schema_version: 1,
    scene_id: 's',
    frame,
    timestamp_ns: frame * 1e8,
    source: 'live',
    objects,
    events,
  };
}

const conflict: WorldEvent = {
  id: 3,
  type: 'VRU_VEHICLE_CONFLICT',
  state: 'active',
  object_ids: [17, 42],
  min_distance_m: 1.44,
  time_to_closest_s: 1.2,
  closing_speed_mps: 2.0,
  vehicle_state: 'moving',
  zone: 'crosswalk_n',
};

describe('angles', () => {
  it('interpolates along the shorter way round', () => {
    expect(angleDelta(3, -3)).toBeCloseTo(2 * Math.PI - 6);
    expect(lerpAngle(Math.PI - 0.1, -Math.PI + 0.1, 0.5)).toBeCloseTo(Math.PI);
  });
});

describe('WorldInterpolator', () => {
  it('moves objects smoothly between the two latest states', () => {
    const interpolator = new WorldInterpolator();
    interpolator.push(state(1, [object(1, 0, 0, 0)]), 0);
    interpolator.push(state(2, [object(1, 10, 0, 0)]), 100);
    expect(interpolator.poses(100).get(1)?.x).toBeCloseTo(0);
    expect(interpolator.poses(150).get(1)?.x).toBeCloseTo(5);
    expect(interpolator.poses(500).get(1)?.x).toBeCloseTo(10);
  });

  it('holds the last heading while the source reports none', () => {
    const interpolator = new WorldInterpolator();
    interpolator.push(state(1, [object(1, 0, 0, 1.0)]), 0);
    interpolator.push(state(2, [object(1, 0, 0, null)]), 100);
    expect(interpolator.poses(200).get(1)?.heading).toBeCloseTo(1.0);
  });

  it('jumps without interpolation after a seek backwards', () => {
    const interpolator = new WorldInterpolator();
    interpolator.push(state(50, [object(1, 0, 0, 0)]), 0);
    interpolator.push(state(3, [object(1, 40, 0, 0)]), 100);
    expect(interpolator.poses(100).get(1)?.x).toBeCloseTo(40);
  });
});

describe('events', () => {
  it('describes conflicts with distance, time and zone', () => {
    expect(describeEvent(conflict)).toBe('#17, #42 · 1.4 m gap in 1.2 s · crosswalk_n');
    expect(describeEvent({ ...conflict, vehicle_state: 'stopped' })).toBe(
      '#17, #42 · 1.4 m gap in 1.2 s · vehicle stopped · crosswalk_n',
    );
  });

  it('keeps active events and a history of ended ones', () => {
    const log = new EventLog();
    log.update(state(1, [], [conflict]));
    expect(log.entries()).toEqual([{ event: conflict, active: true }]);
    log.update(state(2, [], [{ ...conflict, state: 'ended' }]));
    expect(log.entries()).toEqual([{ event: { ...conflict, state: 'ended' }, active: false }]);
  });

  it('formats relative and absolute timestamps', () => {
    expect(formatTimestamp(12_345_000_000)).toBe('12.3 s');
    expect(formatTimestamp(1_651_673_063_600_000_000)).toBe('14:04:23.6 UTC');
  });

  it('finds the objects of active conflicts', () => {
    const { conflict: ids, any } = eventParticipants(state(1, [], [conflict]));
    expect([...ids]).toEqual([17, 42]);
    expect(any.has(42)).toBe(true);
  });
});

describe('video messages', () => {
  it('splits the frame index and the JPEG', () => {
    const buffer = new ArrayBuffer(8 + 3);
    new DataView(buffer).setBigUint64(0, 1234n, true);
    new Uint8Array(buffer, 8).set([0xff, 0xd8, 0xff]);
    const frame = parseVideoMessage(buffer);
    expect(frame.frame).toBe(1234);
    expect(frame.jpeg.size).toBe(3);
    expect(() => parseVideoMessage(new ArrayBuffer(4))).toThrow(RangeError);
  });
});

describe('orbitPose', () => {
  it('looks straight down at 90° elevation with screen-up along the azimuth', () => {
    const view = {
      targetX: 10,
      targetY: 20,
      distance: 50,
      azimuth: Math.PI / 2,
      elevation: Math.PI / 2,
    };
    const { position, up } = orbitPose(view);
    expect(position[0]).toBeCloseTo(10);
    expect(position[1]).toBeCloseTo(20);
    expect(position[2]).toBeCloseTo(50);
    // A quarter turn: world -x points up on screen.
    expect(up[0]).toBeCloseTo(-1);
    expect(up[1]).toBeCloseTo(0);
    expect(up[2]).toBeCloseTo(0);
  });

  it('keeps the camera at the orbit distance, behind the target, with an orthogonal up vector', () => {
    const view = { targetX: 0, targetY: 0, distance: 100, azimuth: 0, elevation: Math.PI / 6 };
    const { position, up } = orbitPose(view);
    expect(Math.hypot(...position)).toBeCloseTo(100);
    // North up on screen: the camera stands south of the target.
    expect(position[1]).toBeLessThan(0);
    const dot = up[0] * -position[0] + up[1] * -position[1] + up[2] * -position[2];
    expect(dot).toBeCloseTo(0);
  });
});

describe('distanceForHeight', () => {
  it('grows linearly with the visible ground height', () => {
    expect(distanceForHeight(200)).toBeCloseTo(2 * distanceForHeight(100));
  });
});
