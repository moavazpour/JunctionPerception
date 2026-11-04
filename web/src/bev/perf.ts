/**
 * Frame statistics of a view, for comparing the 3D twin and the bird's-eye view on the same data:
 * frame interval (what the user sees) and render cost (main-thread time of one frame: scene
 * update and draw calls; GPU time is not included). Exposed as `window.jpPerf`.
 */
export class FrameStats {
  private intervals: number[] = [];
  private costs: number[] = [];
  private last: number | null = null;

  constructor(private readonly capacity = 600) {}

  /** Records a frame that started at `start` (performance.now()) and took `costMs`. */
  record(start: number, costMs: number): void {
    if (this.last !== null) {
      push(this.intervals, start - this.last, this.capacity);
    }
    this.last = start;
    push(this.costs, costMs, this.capacity);
  }

  /** Marks a pause (the view was hidden), so its gap does not count as a frame interval. */
  pause(): void {
    this.last = null;
  }

  reset(): void {
    this.intervals = [];
    this.costs = [];
    this.last = null;
  }

  summary(): {
    frames: number;
    fps: number | null;
    intervalP95Ms: number | null;
    costMeanMs: number | null;
    costP50Ms: number | null;
    costP95Ms: number | null;
  } {
    const mean = (v: number[]): number | null =>
      v.length > 0 ? v.reduce((s, x) => s + x, 0) / v.length : null;
    const interval = mean(this.intervals);
    return {
      frames: this.costs.length,
      fps: interval ? 1000 / interval : null,
      intervalP95Ms: quantile(this.intervals, 0.95),
      costMeanMs: mean(this.costs),
      costP50Ms: quantile(this.costs, 0.5),
      costP95Ms: quantile(this.costs, 0.95),
    };
  }
}

function push(values: number[], value: number, capacity: number): void {
  values.push(value);
  if (values.length > capacity) {
    values.shift();
  }
}

export function quantile(values: number[], q: number): number | null {
  if (values.length === 0) {
    return null;
  }
  const sorted = [...values].sort((a, b) => a - b);
  const position = (sorted.length - 1) * q;
  const lo = Math.floor(position);
  const hi = Math.ceil(position);
  return (
    (sorted[lo] as number) + ((sorted[hi] as number) - (sorted[lo] as number)) * (position - lo)
  );
}

export const perf = { twin: new FrameStats(), bev: new FrameStats() };

declare global {
  interface Window {
    jpPerf?: typeof perf;
  }
}

if (typeof window !== 'undefined') {
  window.jpPerf = perf;
}
