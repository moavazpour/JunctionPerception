import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';

import { describe, expect, it } from 'vitest';

import { isSiteState, type SiteState } from '../src/net/siteState';

// The contract shared with the manager's world model (manager/tests/test_world_model.py).
const fixture = JSON.parse(
  readFileSync(
    fileURLToPath(new URL('../../tests/fixtures/site_state_v1.json', import.meta.url)),
    'utf8',
  ),
) as SiteState;

const copy = (): SiteState => structuredClone(fixture);

describe('site state contract', () => {
  it('accepts the fixture the backend is tested against', () => {
    expect(isSiteState(fixture)).toBe(true);
  });

  it('rejects other versions, extra and missing fields', () => {
    expect(isSiteState({ ...copy(), schema_version: 2 })).toBe(false);
    expect(isSiteState({ ...copy(), extra: 1 })).toBe(false);
    const missing = copy() as Partial<SiteState>;
    delete missing.fusion;
    expect(isSiteState(missing)).toBe(false);
    const user = copy();
    delete (user.road_users[0] as Partial<SiteState['road_users'][0]>).heading_source;
    expect(isSiteState(user)).toBe(false);
  });

  it('keeps an unknown heading unknown: null, not a number', () => {
    const state = copy();
    const pedestrian = state.road_users[1];
    expect(pedestrian?.heading_rad).toBeNull();
    expect(pedestrian?.heading_source).toBeNull();
    // A heading source without a heading, or an invented source, is not valid data.
    const wrong = copy();
    (wrong.road_users[1] as { heading_source: string }).heading_source = 'osm';
    expect(isSiteState(wrong)).toBe(false);
  });

  it('requires a covariance for every road user', () => {
    const state = copy();
    (state.road_users[0] as { position_cov_m2: unknown }).position_cov_m2 = null;
    expect(isSiteState(state)).toBe(false);
  });
});
