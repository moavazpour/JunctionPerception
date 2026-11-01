/**
 * Small SVG charts for the statistics page: stacked bars and lines over time, plain bars. One
 * value axis per chart, recessive grid, rounded bar ends on the baseline, a 2 px surface gap
 * between stacked segments, and a hover tooltip on every mark (crosshair on line charts).
 * Colours are CSS variables of the .viz scope (light and dark steps of one palette).
 */
import { fill, h } from './dom';

const SVG = 'http://www.w3.org/2000/svg';
const HEIGHT = 220;
const PAD = { top: 24, right: 14, bottom: 26, left: 44 };
const BAR_RADIUS = 4;
const GAP = 2;

export interface Series {
  key: string;
  label: string;
  /** CSS colour, e.g. "var(--series-1)". */
  color: string;
}

export interface BarDatum {
  label: string;
  /** Tooltip heading; defaults to the label. */
  title?: string;
  values: Record<string, number>;
}

function svg<K extends keyof SVGElementTagNameMap>(
  tag: K,
  attrs: Record<string, string | number>,
  text?: string,
): SVGElementTagNameMap[K] {
  const element = document.createElementNS(SVG, tag);
  for (const [key, value] of Object.entries(attrs)) {
    element.setAttribute(key, String(value));
  }
  if (text !== undefined) {
    element.textContent = text;
  }
  return element;
}

/** Round numbers for a value axis from 0 to at least `max`. */
export function niceTicks(max: number, count = 4): number[] {
  if (!(max > 0)) {
    return [0, 1];
  }
  const raw = max / count;
  const magnitude = 10 ** Math.floor(Math.log10(raw));
  const step = ([1, 2, 2.5, 5, 10].find((m) => m * magnitude >= raw) ?? 10) * magnitude;
  const ticks: number[] = [];
  for (let v = 0; v < max + step * 0.999; v += step) {
    ticks.push(Math.round(v * 1e6) / 1e6);
  }
  return ticks;
}

function format(value: number): string {
  return Math.abs(value) >= 1000
    ? `${(value / 1000).toFixed(1)}k`
    : `${Math.round(value * 10) / 10}`;
}

/** Path of a bar whose top corners are rounded (its data end); the base stays square. */
function barPath(x: number, y: number, width: number, height: number, round: boolean): string {
  const r = round ? Math.min(BAR_RADIUS, width / 2, height) : 0;
  return `M${x},${y + height}V${y + r}Q${x},${y} ${x + r},${y}H${x + width - r}Q${x + width},${y} ${x + width},${y + r}V${y + height}Z`;
}

class Frame {
  readonly root: HTMLElement;
  readonly plot: SVGSVGElement;
  readonly tooltip = h('div', { class: 'viz-tooltip', hidden: true });
  width = 0;

  constructor(
    readonly ariaLabel: string,
    legend: Series[] | null,
  ) {
    this.plot = svg('svg', { class: 'viz-plot', role: 'img', 'aria-label': ariaLabel });
    this.root = h(
      'div',
      { class: 'viz' },
      legend && legend.length > 1
        ? h(
            'div',
            { class: 'viz-legend' },
            legend.map((s) => h('span', {}, h('i', { style: `background:${s.color}` }), s.label)),
          )
        : null,
      h('div', { class: 'viz-area' }, this.plot, this.tooltip),
    );
  }

  /** Lays out the chart once its width is known, and again when it changes. */
  layout(draw: (width: number) => void): void {
    const run = (): void => {
      const width = Math.max(this.root.clientWidth, 280);
      if (width !== this.width) {
        this.width = width;
        this.plot.setAttribute('viewBox', `0 0 ${width} ${HEIGHT}`);
        this.plot.replaceChildren();
        draw(width);
      }
    };
    new ResizeObserver(run).observe(this.root);
    requestAnimationFrame(run);
  }

  axis(width: number, ticks: number[], unit: string): (v: number) => number {
    const top = ticks[ticks.length - 1] ?? 1;
    const y = (v: number): number =>
      PAD.top + (HEIGHT - PAD.top - PAD.bottom) * (1 - v / (top || 1));
    for (const t of ticks) {
      this.plot.append(
        svg('line', { x1: PAD.left, x2: width - PAD.right, y1: y(t), y2: y(t), class: 'viz-grid' }),
        svg(
          'text',
          { x: PAD.left - 8, y: y(t) + 4, class: 'viz-tick', 'text-anchor': 'end' },
          format(t),
        ),
      );
    }
    if (unit) {
      this.plot.append(svg('text', { x: 4, y: 10, class: 'viz-unit' }, unit));
    }
    return y;
  }

  showTooltip(x: number, y: number, title: string, rows: [string, string, string | null][]): void {
    fill(
      this.tooltip,
      h('strong', {}, title),
      rows.map(([label, value, color]) =>
        h(
          'div',
          { class: 'viz-tip-row' },
          color ? h('i', { style: `background:${color}` }) : null,
          h('span', {}, label),
          h('b', {}, value),
        ),
      ),
    );
    this.tooltip.hidden = false;
    const left = Math.min(x + 12, this.width - this.tooltip.offsetWidth - 4);
    this.tooltip.style.transform = `translate(${Math.max(left, 0)}px, ${Math.max(y - 10, 0)}px)`;
  }

  hideTooltip(): void {
    this.tooltip.hidden = true;
  }
}

/** Bars per category, stacked by series (a single series gives plain bars). */
export function barChart(
  data: BarDatum[],
  series: Series[],
  options: { ariaLabel: string; unit?: string; labelEvery?: number },
): HTMLElement {
  const frame = new Frame(options.ariaLabel, series);
  if (data.length === 0) {
    frame.root.append(h('p', { class: 'viz-empty' }, 'No data in this period'));
    return frame.root;
  }
  frame.layout((width) => {
    const totals = data.map((d) => series.reduce((sum, s) => sum + (d.values[s.key] ?? 0), 0));
    const y = frame.axis(width, niceTicks(Math.max(...totals)), options.unit ?? '');
    const inner = width - PAD.left - PAD.right;
    const slot = inner / data.length;
    const barWidth = Math.max(Math.min(slot * 0.7, 36), 2);
    const every = options.labelEvery ?? Math.ceil(data.length / Math.max(inner / 70, 1));
    data.forEach((d, i) => {
      const x = PAD.left + i * slot + (slot - barWidth) / 2;
      let base = 0;
      const present = series.filter((s) => (d.values[s.key] ?? 0) > 0);
      present.forEach((s, k) => {
        const v = d.values[s.key] ?? 0;
        const top = y(base + v);
        const bottom = y(base);
        // A 2 px surface gap separates stacked segments.
        const height = Math.max(bottom - top - (k > 0 ? GAP : 0), 0.5);
        frame.plot.append(
          svg('path', {
            d: barPath(x, top, barWidth, height, k === present.length - 1),
            fill: s.color,
            class: 'viz-bar',
          }),
        );
        base += v;
      });
      if (i % every === 0) {
        frame.plot.append(
          svg(
            'text',
            { x: x + barWidth / 2, y: HEIGHT - 8, class: 'viz-tick', 'text-anchor': 'middle' },
            d.label,
          ),
        );
      }
      // Hit target: the whole column, taller and wider than the bar.
      const hit = svg('rect', {
        x: PAD.left + i * slot,
        y: PAD.top,
        width: slot,
        height: HEIGHT - PAD.top - PAD.bottom,
        class: 'viz-hit',
      });
      hit.addEventListener('pointerenter', () => {
        const rows: [string, string, string | null][] = series
          .filter((s) => (d.values[s.key] ?? 0) > 0)
          .map((s) => [s.label, format(d.values[s.key] ?? 0), s.color]);
        if (series.length > 1) {
          rows.push(['Total', format(totals[i] ?? 0), null]);
        }
        frame.showTooltip(x + barWidth, y(totals[i] ?? 0), d.title ?? d.label, rows);
      });
      hit.addEventListener('pointerleave', () => frame.hideTooltip());
      frame.plot.append(hit);
    });
  });
  return frame.root;
}

export interface LinePoint {
  label: string;
  title?: string;
  values: Record<string, number | null>;
}

/** Lines over a shared category axis, with a crosshair tooltip and direct end labels. */
export function lineChart(
  data: LinePoint[],
  series: Series[],
  options: { ariaLabel: string; unit?: string },
): HTMLElement {
  const frame = new Frame(options.ariaLabel, series);
  const values = data.flatMap((d) =>
    series.map((s) => d.values[s.key]).filter((v): v is number => v !== null && v !== undefined),
  );
  if (values.length === 0) {
    frame.root.append(h('p', { class: 'viz-empty' }, 'No data in this period'));
    return frame.root;
  }
  frame.layout((width) => {
    const y = frame.axis(width, niceTicks(Math.max(...values)), options.unit ?? '');
    const inner = width - PAD.left - PAD.right - 40; // room for the end labels
    const x = (i: number): number =>
      PAD.left + (data.length === 1 ? inner / 2 : (inner * i) / (data.length - 1));
    const every = Math.ceil(data.length / Math.max(inner / 70, 1));
    data.forEach((d, i) => {
      if (i % every === 0) {
        frame.plot.append(
          svg(
            'text',
            { x: x(i), y: HEIGHT - 8, class: 'viz-tick', 'text-anchor': 'middle' },
            d.label,
          ),
        );
      }
    });
    const labels: { y: number; x: number; text: string }[] = [];
    for (const s of series) {
      let path = '';
      let last: { i: number; v: number } | null = null;
      const value = (i: number): number | null => data[i]?.values[s.key] ?? null;
      data.forEach((_, i) => {
        const v = value(i);
        if (v === null) {
          return;
        }
        const joined = value(i - 1) !== null;
        path += `${joined ? 'L' : 'M'}${x(i)},${y(v)}`;
        if (!joined && value(i + 1) === null) {
          // An isolated value has no segment: it gets a marker.
          frame.plot.append(
            svg('circle', { cx: x(i), cy: y(v), r: 4, fill: s.color, class: 'viz-dot' }),
          );
        }
        last = { i, v };
      });
      frame.plot.append(svg('path', { d: path, stroke: s.color, class: 'viz-line' }));
      const end = last as { i: number; v: number } | null;
      if (end && series.length <= 4) {
        labels.push({ x: x(end.i) + 8, y: y(end.v) + 4, text: s.label });
      }
    }
    // Direct labels at the line ends, pushed apart where they would overlap.
    labels.sort((a, b) => a.y - b.y);
    labels.forEach((label, i) => {
      const above = labels[i - 1];
      if (above && label.y - above.y < 13) {
        label.y = above.y + 13;
      }
      frame.plot.append(svg('text', { x: label.x, y: label.y, class: 'viz-direct' }, label.text));
    });
    const cross = svg('line', { y1: PAD.top, y2: HEIGHT - PAD.bottom, class: 'viz-cross' });
    cross.setAttribute('visibility', 'hidden');
    const dots = series.map((s) => {
      const dot = svg('circle', { r: 4, fill: s.color, class: 'viz-dot' });
      dot.setAttribute('visibility', 'hidden');
      return dot;
    });
    frame.plot.append(cross, ...dots);
    const hit = svg('rect', {
      x: PAD.left,
      y: PAD.top,
      width: width - PAD.left - PAD.right,
      height: HEIGHT - PAD.top - PAD.bottom,
      class: 'viz-hit',
    });
    hit.addEventListener('pointermove', (e) => {
      const box = frame.plot.getBoundingClientRect();
      const px = ((e.clientX - box.left) / box.width) * width;
      let i = 0;
      for (let k = 1; k < data.length; k++) {
        if (Math.abs(x(k) - px) < Math.abs(x(i) - px)) {
          i = k;
        }
      }
      const d = data[i];
      if (!d) {
        return;
      }
      cross.setAttribute('x1', String(x(i)));
      cross.setAttribute('x2', String(x(i)));
      cross.setAttribute('visibility', 'visible');
      const rows: [string, string, string | null][] = [];
      series.forEach((s, k) => {
        const v = d.values[s.key];
        const dot = dots[k];
        if (v === null || v === undefined || !dot) {
          dot?.setAttribute('visibility', 'hidden');
          return;
        }
        dot.setAttribute('cx', String(x(i)));
        dot.setAttribute('cy', String(y(v)));
        dot.setAttribute('visibility', 'visible');
        rows.push([s.label, `${format(v)}${options.unit ? ` ${options.unit}` : ''}`, s.color]);
      });
      frame.showTooltip(x(i), PAD.top, d.title ?? d.label, rows);
    });
    hit.addEventListener('pointerleave', () => {
      cross.setAttribute('visibility', 'hidden');
      dots.forEach((dot) => dot.setAttribute('visibility', 'hidden'));
      frame.hideTooltip();
    });
    frame.plot.append(hit);
  });
  return frame.root;
}
