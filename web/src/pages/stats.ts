import '../styles/pages.css';

import {
  clearStats,
  fetchStats,
  type LineStats,
  type SeriesPoint,
  type Site,
  type SpeedStats,
  type StatsFilters,
  type StatsReport,
  statsCsvUrl,
} from '../net/manager';
import { barChart, lineChart, type Series } from '../ui/charts';
import { type Child, fill, h, param, segmented } from '../ui/dom';
import { confirmDialog, toast, withBusy } from '../ui/feedback';
import { formatDuration } from '../ui/format';
import { icon, type IconName } from '../ui/icons';
import { links, mountShell } from '../ui/shell';

const REFRESH_MS = 30_000;

type Period = 'hour' | 'today' | 'day' | 'week' | 'month' | 'all';
const PERIODS: { value: Period; label: string }[] = [
  { value: 'hour', label: '1 h' },
  { value: 'today', label: 'Today' },
  { value: 'day', label: '24 h' },
  { value: 'week', label: '7 days' },
  { value: 'month', label: '30 days' },
  { value: 'all', label: 'All' },
];
const BUCKETS: { value: number; label: string }[] = [
  { value: 300, label: '5 min' },
  { value: 900, label: '15 min' },
  { value: 3600, label: '1 hour' },
  { value: 86400, label: '1 day' },
];

// Road-user groups of the charts, in the fixed order of the categorical palette.
const GROUPS: { key: string; label: string; classes: string[] }[] = [
  { key: 'car', label: 'Car', classes: ['car'] },
  { key: 'van', label: 'Van', classes: ['van'] },
  { key: 'truck', label: 'Truck', classes: ['truck', 'trailer'] },
  { key: 'bus', label: 'Bus', classes: ['bus'] },
  { key: 'two', label: 'Two-wheeler', classes: ['bicycle', 'motorcycle'] },
  { key: 'pedestrian', label: 'Pedestrian', classes: ['pedestrian'] },
  { key: 'other', label: 'Other', classes: ['other', 'emergency_vehicle'] },
];
const GROUP_SERIES: Series[] = GROUPS.map((g, i) => ({
  key: g.key,
  label: g.label,
  color: `var(--series-${i + 1})`,
}));

function grouped(byClass: Record<string, number>): Record<string, number> {
  const out: Record<string, number> = {};
  for (const g of GROUPS) {
    out[g.key] = g.classes.reduce((sum, c) => sum + (byClass[c] ?? 0), 0);
  }
  return out;
}

function range(period: Period): { from?: number; to?: number } {
  const now = Date.now() / 1000;
  switch (period) {
    case 'hour':
      return { from: now - 3600 };
    case 'today': {
      const midnight = new Date();
      midnight.setHours(0, 0, 0, 0);
      return { from: midnight.getTime() / 1000 };
    }
    case 'day':
      return { from: now - 86400 };
    case 'week':
      return { from: now - 7 * 86400 };
    case 'month':
      return { from: now - 30 * 86400 };
    default:
      return {};
  }
}

function defaultBucket(period: Period): number {
  return period === 'hour'
    ? 300
    : period === 'week' || period === 'all'
      ? 3600
      : period === 'month'
        ? 86400
        : 900;
}

const pct = (v: number | null | undefined): string =>
  v === null || v === undefined ? '–' : `${(v * 100).toFixed(1)} %`;
const num = (v: number | null | undefined, digits = 0, unit = ''): string =>
  v === null || v === undefined ? '–' : `${v.toFixed(digits)}${unit ? ` ${unit}` : ''}`;

function timeLabel(t: number, bucket: number, multiDay: boolean): string {
  const d = new Date(t * 1000);
  const date = `${String(d.getDate()).padStart(2, '0')}.${String(d.getMonth() + 1).padStart(2, '0')}`;
  const clock = `${String(d.getHours()).padStart(2, '0')}:${String(d.getMinutes()).padStart(2, '0')}`;
  if (bucket >= 86400) {
    return date;
  }
  return multiDay ? `${date} ${clock}` : clock;
}

function kpi(name: IconName, label: string, value: string, sub: string, tone = ''): HTMLElement {
  return h(
    'div',
    { class: `kpi ${tone}` },
    h('span', { class: 'label' }, icon(name), label),
    h('span', { class: 'value' }, value),
    h('span', { class: 'sub' }, sub),
  );
}

function card(title: string, sub: string, ...body: Child[]): HTMLElement {
  return h(
    'section',
    { class: 'card stats-card' },
    h('header', { class: 'card-header' }, h('div', {}, h('h2', {}, title), h('p', {}, sub))),
    h('div', { class: 'card-body' }, body),
  );
}

/** Line, site and camera names of a report row. */
function lineName(sites: Site[], l: { site: string; camera: string; name: string }): string {
  const site = sites.find((s) => s.id === l.site);
  const camera = site?.cameras.find((c) => c.id === l.camera);
  const parts = [site?.name ?? l.site];
  if (site && site.cameras.length > 1) {
    parts.push(camera?.name ?? l.camera);
  }
  return `${parts.join(' · ')} — ${l.name}`;
}

function speedCell(s: SpeedStats): string {
  return s.n > 0 ? `${num(s.mean)} / ${num(s.p85)}` : '–';
}

function lineTable(lines: LineStats[], sites: Site[]): HTMLElement {
  const head = [
    'Line',
    'Road users',
    'Direction A / B',
    'Flow veh/h',
    'Flow pcu/h',
    'Heavy',
    'Peak hour · PHF',
    'Speed mean / 85th',
    'Space-mean',
    '15th – 95th',
    'Min – max',
    'Over limit',
    'Headway A (mean · min)',
    'Headway B (mean · min)',
    'Density veh/km',
    'Observed',
  ];
  return h(
    'div',
    { class: 'table-wrap' },
    h(
      'table',
      { class: 'table stats-table' },
      h(
        'thead',
        {},
        h(
          'tr',
          {},
          head.map((t) => h('th', {}, t)),
        ),
      ),
      h(
        'tbody',
        {},
        lines.map((l) => {
          const s = l.speed_kmh;
          const peak = l.peak_hour;
          const headway = (x: LineStats['headway']['A']) =>
            x.n > 0 ? `${num(x.mean_s, 1)} s · ${num(x.min_s, 1)} s` : '–';
          return h(
            'tr',
            {},
            h('td', { class: 'strong' }, lineName(sites, l)),
            h(
              'td',
              { class: 'num' },
              `${l.count}`,
              l.vru > 0 ? h('span', { class: 'muted' }, ` (${l.vru} VRU)`) : null,
            ),
            h('td', { class: 'num' }, `${l.directions.A} / ${l.directions.B}`),
            h('td', { class: 'num' }, num(l.flow_vph)),
            h('td', { class: 'num' }, num(l.flow_pcuph)),
            h('td', { class: 'num' }, pct(l.heavy_share)),
            h(
              'td',
              { class: 'num' },
              peak
                ? `${timeLabel(peak.start, 900, true)} · ${peak.volume} · ${num(peak.phf, 2)}`
                : '–',
            ),
            h('td', { class: 'num' }, speedCell(s)),
            h('td', { class: 'num' }, num(s.space_mean)),
            h('td', { class: 'num' }, s.n > 0 ? `${num(s.p15)} – ${num(s.p95)}` : '–'),
            h('td', { class: 'num' }, s.n > 0 ? `${num(s.min)} – ${num(s.max)}` : '–'),
            h(
              'td',
              { class: 'num' },
              l.speed_limit_kmh ? `${pct(s.over_limit_share)} of ${l.speed_limit_kmh}` : '–',
            ),
            h('td', { class: 'num' }, headway(l.headway.A)),
            h('td', { class: 'num' }, headway(l.headway.B)),
            h('td', { class: 'num' }, num(l.density_vpkm, 1)),
            h('td', { class: 'num' }, formatDuration(l.observed_s)),
          );
        }),
      ),
    ),
  );
}

function volumeChart(series: SeriesPoint[], bucket: number, multiDay: boolean): HTMLElement {
  return barChart(
    series.map((p) => ({
      label: timeLabel(p.t, bucket, multiDay),
      title: `${timeLabel(p.t, bucket, true)} · ${p.count} road users`,
      values: grouped(p.by_class),
    })),
    GROUP_SERIES.filter((s) => series.some((p) => (grouped(p.by_class)[s.key] ?? 0) > 0)),
    { ariaLabel: 'Road users per interval by type', unit: 'per interval' },
  );
}

function speedChart(series: SeriesPoint[], bucket: number, multiDay: boolean): HTMLElement {
  return lineChart(
    series.map((p) => ({
      label: timeLabel(p.t, bucket, multiDay),
      title: timeLabel(p.t, bucket, true),
      values: { mean: p.mean_kmh, p85: p.p85_kmh },
    })),
    [
      { key: 'mean', label: 'Mean', color: 'var(--series-1)' },
      { key: 'p85', label: '85th percentile', color: 'var(--series-2)' },
    ],
    { ariaLabel: 'Vehicle speeds per interval', unit: 'km/h' },
  );
}

function render(report: StatsReport, sites: Site[], bucket: number): HTMLElement[] {
  const multiDay =
    report.series.length > 0 &&
    (report.series[report.series.length - 1]?.t ?? 0) - (report.series[0]?.t ?? 0) > 86400;
  const lines = report.lines;
  const vehicles = lines.reduce((sum, l) => sum + l.vehicles, 0);
  const vru = lines.reduce((sum, l) => sum + l.vru, 0);
  const flows = lines.map((l) => l.flow_vph).filter((v): v is number => v !== null);
  const pcus = lines.map((l) => l.flow_pcuph).filter((v): v is number => v !== null);
  const counted = lines.filter((l) => l.speed_kmh.n > 0);
  const weighted = (pick: (s: SpeedStats) => number | undefined): number | null => {
    const n = counted.reduce((sum, l) => sum + l.speed_kmh.n, 0);
    return n > 0
      ? counted.reduce((sum, l) => sum + (pick(l.speed_kmh) ?? 0) * l.speed_kmh.n, 0) / n
      : null;
  };
  const heavy = lines.reduce((sum, l) => sum + (l.heavy_share ?? 0) * l.vehicles, 0);
  const busiest = [...lines].sort(
    (a, b) => (b.peak_hour?.volume ?? 0) - (a.peak_hour?.volume ?? 0),
  )[0];
  const events = report.events;

  const kpis = h(
    'section',
    { class: 'kpis' },
    kpi(
      'activity',
      'Vehicles counted',
      `${vehicles}`,
      `${report.totals.count} road users on ${lines.length} lines`,
    ),
    kpi(
      'route',
      'Flow rate',
      flows.length ? `${Math.round(flows.reduce((a, b) => a + b, 0))} veh/h` : '–',
      pcus.length
        ? `${Math.round(pcus.reduce((a, b) => a + b, 0))} pcu/h over all lines`
        : 'over the observed time',
    ),
    kpi(
      'clock',
      'Speed mean · 85th',
      counted.length ? `${num(weighted((s) => s.mean))} · ${num(weighted((s) => s.p85))}` : '–',
      'km/h, vehicles',
    ),
    kpi(
      'building',
      'Heavy vehicles',
      vehicles ? pct(heavy / vehicles) : '–',
      'trucks, trailers and buses',
    ),
    kpi(
      'pulse',
      'Peak hour factor',
      busiest?.peak_hour ? num(busiest.peak_hour.phf, 2) : '–',
      busiest?.peak_hour
        ? `${busiest.name}: ${busiest.peak_hour.volume} veh from ${timeLabel(busiest.peak_hour.start, 900, true)}`
        : 'needs an hour of counts',
    ),
    kpi('walk', 'Vulnerable road users', `${vru}`, 'pedestrians, cyclists, motorcyclists'),
    kpi(
      'conflict',
      'Safety events',
      `${events.count}`,
      events.min_ttc_s !== null
        ? `shortest time to conflict ${events.min_ttc_s} s`
        : 'VRU conflicts and crosswalks',
      events.by_type['VRU_VEHICLE_CONFLICT'] ? 'warn' : '',
    ),
  );

  const histogram = barChart(
    report.speed_histogram.map((b) => ({
      label: `${b.from}`,
      title: `${b.from}–${b.from + 5} km/h`,
      values: { n: b.count },
    })),
    [{ key: 'n', label: 'Vehicles', color: 'var(--series-1)' }],
    { ariaLabel: 'Speed distribution', unit: 'vehicles' },
  );
  const hourly = barChart(
    report.hourly.map((count, hour) => ({
      label: String(hour).padStart(2, '0'),
      title: `${String(hour).padStart(2, '0')}:00–${String(hour + 1).padStart(2, '0')}:00`,
      values: { n: count },
    })),
    [{ key: 'n', label: 'Road users', color: 'var(--series-1)' }],
    { ariaLabel: 'Road users by hour of the day', unit: 'road users', labelEvery: 3 },
  );

  const turning =
    report.turning.length > 0
      ? h(
          'div',
          { class: 'table-wrap' },
          h(
            'table',
            { class: 'table' },
            h(
              'thead',
              {},
              h(
                'tr',
                {},
                ['Camera', 'From line', 'To line', 'Road users', 'Share by type'].map((t) =>
                  h('th', {}, t),
                ),
              ),
            ),
            h(
              'tbody',
              {},
              report.turning.map((m) =>
                h(
                  'tr',
                  {},
                  h(
                    'td',
                    {},
                    lineName(sites, { site: m.site, camera: m.camera, name: '' }).replace(
                      / — $/,
                      '',
                    ),
                  ),
                  h('td', { class: 'strong' }, m.from_name),
                  h('td', { class: 'strong' }, m.to_name),
                  h('td', { class: 'num' }, `${m.count}`),
                  h(
                    'td',
                    { class: 'muted' },
                    Object.entries(grouped(m.by_class))
                      .filter(([, n]) => n > 0)
                      .map(([k, n]) => `${GROUPS.find((g) => g.key === k)?.label ?? k} ${n}`)
                      .join(' · '),
                  ),
                ),
              ),
            ),
          ),
        )
      : h(
          'p',
          { class: 'help' },
          'Turning movements appear when one camera has two or more lines, e.g. one on each arm: every road user that crosses two of them is counted from the first to the last line.',
        );

  return [
    kpis,
    h(
      'div',
      { class: 'grid-2 stats-grid' },
      card(
        'Volume',
        `Road users per ${BUCKETS.find((b) => b.value === bucket)?.label ?? 'interval'}, by type`,
        volumeChart(report.series, bucket, multiDay),
      ),
      card(
        'Speeds',
        'Vehicle spot speeds at the lines, mean and 85th percentile',
        speedChart(report.series, bucket, multiDay),
      ),
      card('Speed distribution', 'Vehicles per 5 km/h', histogram),
      card('Daily profile', 'Road users by hour of the day', hourly),
    ),
    card(
      'Counting lines',
      'Traffic engineering measures per line; direction A crosses the line as shown by its arrow in the calibration',
      lines.length > 0
        ? lineTable(lines, sites)
        : h('p', { class: 'help' }, 'No line has counts in this period.'),
    ),
    card('Turning movements', 'Origin–destination between the lines of a camera', turning),
    card(
      'Definitions',
      'How the measures are computed',
      h(
        'dl',
        { class: 'kv definitions' },
        [
          [
            'Flow rate',
            'Vehicles (or passenger car units: car 1, van 1.5, truck and bus 2, trailer 2.5, motorcycle 0.5, bicycle 0.3) per hour of observed time, i.e. while the camera ran.',
          ],
          [
            'Peak hour factor',
            'Volume of the busiest hour divided by four times its busiest 15 minutes (HCM). 1.0: even demand; lower: peaky demand; used to size signal timing for the peak 15 minutes.',
          ],
          [
            'Heavy vehicles',
            'Share of trucks, trailers and buses among the vehicles; reduces saturation flow at signals.',
          ],
          [
            'Spot speeds',
            'Speed of each vehicle as it crosses the line. Mean is the time-mean speed; space-mean is the harmonic mean, used with flow for density. The 85th percentile is the usual basis for speed limits; pace is the 10 km/h band holding most vehicles.',
          ],
          [
            'Headway',
            'Time between consecutive vehicles crossing a line in one direction; short headways (under 2 s) indicate platoons or saturated flow.',
          ],
          ['Density', 'Flow rate divided by space-mean speed (vehicles per km).'],
          [
            'Turning movements',
            'Road users crossing two lines of one camera, from the first to the last line: the input for signal phasing and intersection design.',
          ],
          [
            'Safety events',
            'VRUs on crosswalks and predicted VRU–vehicle conflicts, with the time to the closest approach.',
          ],
        ].flatMap(([term, text]) => [h('dt', {}, term), h('dd', {}, text)]),
      ),
    ),
  ];
}

async function start(): Promise<void> {
  const shell = await mountShell({ page: 'stats', title: 'Traffic statistics' });
  if (!shell.managed) {
    location.replace('/twin.html');
    return;
  }
  let period: Period = 'day';
  let bucket = defaultBucket(period);
  let site = param('site') ?? '';
  let line = '';
  const body = h('div', { class: 'stack' });
  const siteSelect = h('select', { class: 'input', 'aria-label': 'Site' });
  const lineSelect = h('select', { class: 'input', 'aria-label': 'Counting line' });
  const bucketSelect = h('select', { class: 'input', 'aria-label': 'Interval' });
  const csv = h('a', { class: 'btn small', download: 'crossings.csv' }, icon('download'), 'CSV');
  const updated = h('span', { class: 'muted stats-updated' });

  const filters = (): StatsFilters => {
    const [camera, lineId] = line ? line.split('|') : [undefined, undefined];
    return { site: site || undefined, camera, line: lineId, ...range(period) };
  };

  const fillSelects = (): void => {
    const sites = shell.sites();
    fill(
      siteSelect,
      h('option', { value: '' }, 'All sites'),
      sites.map((s) => h('option', { value: s.id, selected: s.id === site }, s.name)),
    );
    const chosen = sites.find((s) => s.id === site);
    const lines = (chosen?.cameras ?? []).flatMap((c) =>
      c.regions.lines.map((l) => ({
        value: `${c.id}|${l.id}`,
        label: chosen && chosen.cameras.length > 1 ? `${c.name} · ${l.name}` : l.name,
      })),
    );
    if (!lines.some((l) => l.value === line)) {
      line = '';
    }
    fill(
      lineSelect,
      h('option', { value: '' }, chosen ? 'All lines' : 'All lines of all sites'),
      lines.map((l) => h('option', { value: l.value, selected: l.value === line }, l.label)),
    );
    lineSelect.disabled = !chosen;
    fill(
      bucketSelect,
      BUCKETS.map((b) =>
        h('option', { value: String(b.value), selected: b.value === bucket }, b.label),
      ),
    );
  };

  let loading = false;
  const load = async (): Promise<void> => {
    if (loading) {
      return;
    }
    loading = true;
    try {
      const report = await fetchStats(filters(), bucket);
      csv.setAttribute('href', statsCsvUrl(filters()));
      const sites = shell.sites();
      const anyLines = sites.some((s) => s.cameras.some((c) => c.regions.lines.length > 0));
      if (report.lines.length === 0 && !anyLines) {
        fill(
          body,
          h(
            'div',
            { class: 'card empty' },
            icon('route'),
            h('h3', {}, 'No counting lines yet'),
            h(
              'p',
              {},
              'Draw lines across the lanes in a camera’s Calibration, tab "Areas and lines", and save. While the camera runs, every road user crossing a line is recorded here with time, type, direction and speed.',
            ),
            sites[0]
              ? h(
                  'a',
                  { class: 'btn primary', href: links.calibration(sites[0].id) },
                  icon('calibration'),
                  'Open calibration',
                )
              : null,
          ),
        );
      } else {
        fill(body, render(report, sites, bucket));
      }
      updated.textContent = `updated ${new Date(report.generated_at * 1000).toLocaleTimeString()}`;
    } catch (error) {
      toast((error as Error).message, 'error');
    } finally {
      loading = false;
    }
  };

  const periodControl = segmented<Period>(PERIODS, period, (value) => {
    period = value;
    bucket = defaultBucket(value);
    fillSelects();
    void load();
  });
  siteSelect.addEventListener('change', () => {
    site = siteSelect.value;
    line = '';
    history.replaceState(
      null,
      '',
      site ? `/stats.html?site=${encodeURIComponent(site)}` : '/stats.html',
    );
    fillSelects();
    void load();
  });
  lineSelect.addEventListener('change', () => {
    line = lineSelect.value;
    void load();
  });
  bucketSelect.addEventListener('change', () => {
    bucket = Number(bucketSelect.value);
    void load();
  });
  const refresh = h(
    'button',
    { type: 'button', class: 'btn small ghost icon-only', title: 'Refresh' },
    icon('restart'),
  );
  refresh.addEventListener('click', () => void load());
  const reset = h(
    'button',
    { type: 'button', class: 'btn small ghost', title: 'Delete recorded statistics' },
    icon('trash'),
    'Reset',
  );
  reset.addEventListener('click', async () => {
    const name = shell.sites().find((s) => s.id === site)?.name;
    const ok = await confirmDialog({
      title: name ? `Delete the statistics of ${name}?` : 'Delete all statistics?',
      body: 'All recorded crossings, events and observation times are removed. This cannot be undone.',
      confirm: 'Delete statistics',
      danger: true,
    });
    if (ok) {
      await withBusy(reset, () => clearStats(site || undefined), 'Statistics deleted');
      await load();
    }
  });

  shell.setActions(
    h('a', { class: 'btn small', href: links.twin() }, icon('globe'), 'Digital twin'),
  );
  shell.content.append(
    h(
      'div',
      { class: 'page' },
      h(
        'div',
        { class: 'page-head' },
        h(
          'div',
          {},
          h('h1', {}, 'Traffic statistics'),
          h(
            'p',
            {},
            'Counts, flow, speeds, headways and turning movements at the counting lines of every camera, with date and time.',
          ),
        ),
      ),
      h(
        'div',
        { class: 'filters' },
        siteSelect,
        lineSelect,
        periodControl.root,
        bucketSelect,
        h('span', { class: 'spacer' }),
        updated,
        refresh,
        csv,
        reset,
      ),
      body,
    ),
  );
  fillSelects();
  shell.onSites(() => {
    if (document.activeElement !== siteSelect && document.activeElement !== lineSelect) {
      fillSelects();
    }
  });
  await load();
  setInterval(() => void load(), REFRESH_MS);
}

void start();
