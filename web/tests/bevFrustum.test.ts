import { describe, expect, it } from 'vitest';

import { bevFrustum } from '../src/twin/bevFrustum';

describe('bevFrustum', () => {
  it('keeps the view height and scales the width by the aspect ratio', () => {
    expect(bevFrustum(100, 2)).toEqual({ left: -100, right: 100, top: 50, bottom: -50 });
  });

  it('rejects non-positive or non-finite inputs', () => {
    expect(() => bevFrustum(0, 1)).toThrow(RangeError);
    expect(() => bevFrustum(10, -1)).toThrow(RangeError);
    expect(() => bevFrustum(Number.NaN, 1)).toThrow(RangeError);
  });
});
