import { readFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';

import { isWorldState } from '../src/net/schema';

// Shared with the C++ serializer tests: both sides must accept exactly this document.
const fixture: unknown = JSON.parse(
  readFileSync(new URL('../../tests/fixtures/world_state_v1.json', import.meta.url), 'utf8'),
);

describe('world-state schema', () => {
  it('accepts the shared example', () => {
    expect(isWorldState(fixture)).toBe(true);
  });

  it('rejects missing, extra or mistyped fields', () => {
    const state = structuredClone(fixture) as Record<string, unknown>;
    expect(isWorldState({ ...state, schema_version: 2 })).toBe(false);
    expect(isWorldState({ ...state, extra: 1 })).toBe(false);
    const objects = structuredClone(state['objects']) as Record<string, unknown>[];
    delete objects[0]?.['turning'];
    expect(isWorldState({ ...state, objects })).toBe(false);
    const events = structuredClone(state['events']) as Record<string, unknown>[];
    if (events[0]) {
      events[0]['type'] = 'UNKNOWN';
    }
    expect(isWorldState({ ...state, events })).toBe(false);
  });
});

describe('world-state schema additions', () => {
  it('accepts objects without the measured footprint of older recordings', () => {
    const state = structuredClone(fixture) as Record<string, unknown>;
    const objects = structuredClone(state['objects']) as Record<string, unknown>[];
    for (const o of objects) {
      delete o['length_m'];
      delete o['width_m'];
    }
    expect(isWorldState({ ...state, objects })).toBe(true);
  });
});
