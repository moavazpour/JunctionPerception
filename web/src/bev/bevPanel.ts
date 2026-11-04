import type { WorldState } from '../net/schema';
import { CLASS_LABELS } from '../twin/palette';
import { fill, h } from '../ui/dom';
import { icon } from '../ui/icons';
import type { BevTrack, DuplicatePair, FeedQuality } from './bevModel';
import { covarianceEllipse } from './geometry';
import type { LaneStatus, SiteStatus } from './laneAnalytics';
import { MAX_MERGE_SKEW_S } from './bevView';

/** What kind of number a value is: counted from tracks, derived with assumptions, or a trial. */
type Basis = 'measured' | 'estimate' | 'experimental';

const BASIS_TAG: Record<Basis, string> = { measured: 'M', estimate: 'E', experimental: 'X' };
const BASIS_TITLE: Record<Basis, string> = {
  measured: 'Measured: counted from the tracks of this frame',
  estimate: 'Estimate: derived with assumptions (see the note)',
  experimental: 'Experimental indicator: not validated, do not act on it alone',
};

function basis(kind: Basis): HTMLElement {
  return h('i', { class: `basis ${kind}`, title: BASIS_TITLE[kind] }, BASIS_TAG[kind]);
}

function num(value: number | null, digits = 0, unit = ''): string {
  return value === null || !Number.isFinite(value) ? '–' : `${value.toFixed(digits)}${unit}`;
}

const COMPACT_KEY = 'jp-bev-card-compact';
const FOLDED_KEY = 'jp-bev-card-folded';

function readFlag(key: string, fallback: boolean): boolean {
  try {
    const value = localStorage.getItem(key);
    return value === null ? fallback : value === '1';
  } catch {
    return fallback;
  }
}

function writeFlag(key: string, on: boolean): void {
  try {
    localStorage.setItem(key, on ? '1' : '0');
  } catch {
    // Not remembered.
  }
}

export interface PanelInput {
  siteName: string;
  status: SiteStatus | null;
  states: { camera: string; name: string; state: WorldState; ageMs: number }[];
  quality: FeedQuality[];
  tracks: BevTrack[];
  duplicates: DuplicatePair[];
  selected: BevTrack | null;
}

/**
 * Side card of the bird's-eye view with what the operator of the signals needs at a glance: the
 * state of every lane (vehicles, standing vehicles, queue, waiting time, arrivals), the
 * intersection and the crosswalks, the signals, active conflicts and the quality of the data the
 * numbers rest on.
 */
export class BevPanel {
  onLane: (id: string | null) => void = () => undefined;

  /** Compact: lanes, intersection, crosswalks and signals only (what the operator watches). */
  private compact = readFlag(COMPACT_KEY, true);
  /** Folded: the header line only, the map free. */
  private folded = readFlag(FOLDED_KEY, false);
  private last: PanelInput | null = null;
  // The header stays: the body is rebuilt a few times a second, which would swallow clicks.
  private readonly title = h('strong', {});
  private readonly time = h('span', { class: 'muted num' });
  private readonly toggle = h('button', { type: 'button', class: 'btn ghost small' });
  private readonly fold = h(
    'button',
    { type: 'button', class: 'btn ghost small icon-only fold' },
    icon('chevron'),
  );
  private readonly legend = h(
    'span',
    { class: 'bev-legend-basis' },
    basis('measured'),
    'measured ',
    basis('estimate'),
    'estimate ',
    basis('experimental'),
    'experimental',
  );
  private readonly body = h('div', {});

  constructor(private readonly root: HTMLElement) {
    // Rows are rebuilt while hovered: clear the lane highlight when the pointer leaves the card.
    root.addEventListener('mouseleave', () => this.onLane(null));
    this.toggle.addEventListener('click', () => {
      this.compact = !this.compact;
      writeFlag(COMPACT_KEY, this.compact);
      this.refresh();
    });
    this.fold.addEventListener('click', () => {
      this.folded = !this.folded;
      writeFlag(FOLDED_KEY, this.folded);
      this.refresh();
    });
    fill(
      root,
      h(
        'header',
        { class: 'bev-head' },
        this.title,
        this.time,
        this.fold,
        this.toggle,
        this.legend,
      ),
      this.body,
    );
    this.applyFold();
  }

  /** A note instead of the status (no camera yet). */
  waiting(text: string): void {
    fill(this.body, h('p', { class: 'muted' }, text));
  }

  render(input: PanelInput): void {
    this.last = input;
    const { status } = input;
    const time = input.states[0]?.state.timestamp_ns;
    this.title.textContent = input.siteName;
    this.time.textContent = time !== undefined ? `t ${(time / 1e9).toFixed(1)} s` : '';
    this.applyFold();
    if (this.folded) {
      return;
    }
    fill(
      this.body,
      this.lanes(status?.lanes ?? []),
      this.areas(status),
      this.signals(input),
      this.compact ? null : this.conflicts(input),
      this.selection(input.selected),
      this.compact ? null : this.quality(input),
      this.compact ? null : this.key(),
    );
  }

  private refresh(): void {
    if (this.last) {
      this.render(this.last);
    } else {
      this.applyFold();
    }
  }

  private applyFold(): void {
    this.root.classList.toggle('folded', this.folded);
    this.body.hidden = this.folded;
    this.toggle.hidden = this.folded;
    this.legend.hidden = this.folded || this.compact;
    this.fold.title = this.folded ? 'Show the lane status' : 'Fold the card (header only)';
    this.toggle.textContent = this.compact ? 'More' : 'Less';
    this.toggle.title = this.compact
      ? 'Show directions, arrivals, speeds, conflicts, data quality and the key'
      : 'Only what the operator watches';
  }

  private section(title: string, ...children: (Node | null)[]): HTMLElement {
    return h('section', { class: 'bev-section' }, h('h4', {}, title), ...children);
  }

  private lanes(lanes: LaneStatus[]): HTMLElement {
    if (lanes.length === 0) {
      return this.section(
        'Lanes',
        h(
          'p',
          { class: 'muted' },
          'No lanes drawn. Calibration › Zones: one zone per lane up to its stop line, type “lane”.',
        ),
      );
    }
    const full = !this.compact;
    const tag = (kind: Basis) => (full ? basis(kind) : null);
    const head = h(
      'tr',
      {},
      h('th', {}, 'Lane'),
      full ? h('th', { title: 'Direction of travel and what it rests on' }, 'Dir') : null,
      h('th', { title: 'Vehicles in the lane' }, 'Veh ', tag('measured')),
      h('th', { title: 'Standing vehicles (< 5 km/h, speed known)' }, 'Stop ', tag('measured')),
      h('th', { title: 'Queue from the lane end upstream (m)' }, 'Queue ', tag('estimate')),
      h(
        'th',
        { title: 'Longest standing time in the queue (s); restarts with a new track ID' },
        'Wait ',
        tag('estimate'),
      ),
      full
        ? h('th', { title: 'Vehicles entering the lane per minute' }, 'Arr/min ', tag('estimate'))
        : null,
      full
        ? h(
            'th',
            { title: 'Mean speed of vehicles with a known speed (km/h)' },
            'km/h ',
            tag('measured'),
          )
        : null,
    );
    const rows = lanes.map((lane) => {
      const row = h(
        'tr',
        { class: `lane-${lane.state}` },
        h('td', {}, h('span', { class: `lane-dot ${lane.state}` }), lane.id),
        full
          ? h(
              'td',
              {
                title: lane.direction
                  ? lane.direction.source === 'observed'
                    ? `Observed: ${lane.direction.agree} of ${lane.direction.total} tracks`
                    : 'One-way road in OpenStreetMap'
                  : 'Unknown: too few tracks moved along this lane, and no one-way road. No queue without it.',
              },
              lane.direction ? (lane.direction.source === 'observed' ? 'obs' : 'OSM') : '?',
            )
          : null,
        h(
          'td',
          { class: 'num' },
          String(lane.vehicles),
          lane.uncertain > 0
            ? h(
                'span',
                { class: 'muted', title: 'Speed too uncertain to say whether they stand' },
                ` +${lane.uncertain}?`,
              )
            : null,
        ),
        h('td', { class: 'num' }, String(lane.stopped)),
        h(
          'td',
          {
            class: 'num',
            title: lane.direction ? '' : 'Direction not known yet: no queue without it',
          },
          lane.queueM === null
            ? '–'
            : `${lane.queueM.toFixed(0)} m${lane.queueVehicles > 0 ? ` (${lane.queueVehicles})` : ''}`,
        ),
        h('td', { class: 'num' }, num(lane.maxWaitS, 0, ' s')),
        full
          ? h(
              'td',
              { class: 'num', title: `over ${lane.arrivalWindowS.toFixed(0)} s of camera time` },
              num(lane.arrivalsPerMin, 1),
            )
          : null,
        full ? h('td', { class: 'num' }, num(lane.meanSpeedKmh)) : null,
      );
      row.addEventListener('mouseenter', () => this.onLane(lane.id));
      row.addEventListener('mouseleave', () => this.onLane(null));
      return row;
    });
    return this.section(
      'Lanes',
      h('table', { class: 'bev-table' }, h('thead', {}, head), h('tbody', {}, rows)),
      full
        ? h(
            'p',
            { class: 'note' },
            'Queue: standing vehicles chained from the lane end (gaps ≤ 10 m). The lane end stands in for the stop line, which is not mapped.',
          )
        : null,
    );
  }

  private areas(status: SiteStatus | null): HTMLElement | null {
    const areas = status?.areas ?? [];
    if (areas.length === 0) {
      return null;
    }
    // Compact: only zones with someone in them; the rest in one line.
    const shown = this.compact ? areas.filter((a) => a.vehicles + a.vrus > 0 || a.blocked) : areas;
    const clear = areas.length - shown.length;
    return this.section(
      'Intersection and crosswalks',
      shown.length > 0
        ? h(
            'ul',
            { class: 'bev-list' },
            shown.map((a) =>
              h(
                'li',
                { class: a.blocked ? 'danger' : '' },
                h('span', { class: `area-dot ${a.type}` }),
                h('span', { class: 'name' }, a.id),
                h(
                  'span',
                  { class: 'num' },
                  `${a.vehicles} veh`,
                  a.stoppedVehicles > 0 ? ` (${a.stoppedVehicles} standing)` : '',
                ),
                h('span', { class: 'num' }, `${a.vrus} VRU`),
                this.compact ? null : basis('measured'),
                a.blocked
                  ? h(
                      'span',
                      {
                        class: 'pill danger plain',
                        title: 'A vehicle stands in the intersection ≥ 5 s',
                      },
                      `blocked ${num(a.longestStopS)} s`,
                    )
                  : null,
              ),
            ),
          )
        : null,
      clear > 0
        ? h(
            'p',
            { class: 'muted' },
            shown.length > 0 ? `${clear} more clear` : `All ${clear} zones clear`,
          )
        : null,
    );
  }

  private signals(input: PanelInput): HTMLElement | null {
    const rows = input.states.flatMap(({ name, state, ageMs }) =>
      (state.signals ?? []).map((s) => ({ name, signal: s, state, ageMs })),
    );
    if (rows.length === 0) {
      return null;
    }
    return this.section(
      'Signals',
      h(
        'ul',
        { class: 'bev-list' },
        rows.map(({ name, signal, state }) =>
          h(
            'li',
            {},
            h('span', { class: `signal-dot ${signal.state}` }),
            h('span', { class: 'name' }, signal.name),
            input.states.length > 1 ? h('span', { class: 'muted' }, name) : null,
            h('span', { class: 'num' }, signal.state.replace('_', '-')),
            h(
              'span',
              { class: 'num' },
              `${Math.max(0, (state.timestamp_ns - signal.since_ns) / 1e9).toFixed(0)} s`,
            ),
            h(
              'span',
              { class: 'muted num', title: 'Confidence of the colour reading' },
              `${(signal.confidence * 100).toFixed(0)} %`,
            ),
            basis('measured'),
          ),
        ),
      ),
      h('p', { class: 'note' }, 'Read from the camera image. Signals are not linked to lanes yet.'),
    );
  }

  private conflicts(input: PanelInput): HTMLElement | null {
    const events = input.states.flatMap(({ name, state }) =>
      state.events
        .filter((e) => e.state === 'active' && e.type === 'VRU_VEHICLE_CONFLICT')
        .map((e) => ({ name, e })),
    );
    if (events.length === 0) {
      return null;
    }
    return this.section(
      'Conflicts',
      h(
        'ul',
        { class: 'bev-list' },
        events.map(({ name, e }) =>
          h(
            'li',
            { class: 'danger' },
            h('span', { class: 'name' }, `#${e.object_ids.join(' ↔ #')}`),
            input.states.length > 1 ? h('span', { class: 'muted' }, name) : null,
            h('span', { class: 'num' }, `gap ${num(e.min_distance_m, 1, ' m')}`),
            h('span', { class: 'num' }, `in ${num(e.time_to_closest_s, 1, ' s')}`),
            h('span', { class: 'muted' }, e.vehicle_state ?? ''),
            basis('experimental'),
          ),
        ),
      ),
      h(
        'p',
        { class: 'note' },
        'Constant-velocity prediction of footprint gaps; not TTC or PET. Position errors of 0.5–1.7 m (P50–P90) make it a pointer, not a measurement.',
      ),
    );
  }

  private selection(t: BevTrack | null): HTMLElement | null {
    if (!t) {
      return null;
    }
    const o = t.object;
    const ellipse = o.position_cov_m2 ? covarianceEllipse(o.position_cov_m2, 1) : null;
    const rows: [string, string][] = [
      ['Track', `${t.camera} #${o.id}${t.possibleIdSwitch ? ' (possible ID switch)' : ''}`],
      ['Class', `${CLASS_LABELS[o.class] ?? o.class}${o.rider ? ' (rider)' : ''}`],
      ['Position', `${o.x.toFixed(2)}, ${o.y.toFixed(2)} m`],
      [
        'Position σ',
        ellipse ? `${ellipse.major.toFixed(2)} × ${ellipse.minor.toFixed(2)} m` : 'not reported',
      ],
      [
        'Speed',
        `${(o.speed_mps * 3.6).toFixed(1)}${o.speed_std_mps !== null ? ` ± ${(o.speed_std_mps * 3.6).toFixed(1)}` : ''} km/h (${t.motion})`,
      ],
      [
        'Heading',
        t.heading === null
          ? 'unknown'
          : `${((((t.heading * 180) / Math.PI) % 360) + 360) % 360 | 0}° ${t.headingStatus}: ${t.headingNote}`,
      ],
      [
        'Size',
        `${t.length.toFixed(1)} × ${t.width.toFixed(1)} m ${t.sizeMeasured ? 'measured' : 'class default'}`,
      ],
      ['Confidence', `${(o.confidence * 100).toFixed(0)} %`],
      [
        'Age',
        `${t.ageS.toFixed(1)} s${t.stoppedS !== null ? `, standing ${t.stoppedS.toFixed(0)} s` : ''}`,
      ],
      ['Frame', `${t.frame} · t ${(t.timestampNs / 1e9).toFixed(3)} s`],
      ['Zones', o.zones.join(', ') || '–'],
      ['Validated', o.validated ? 'yes' : 'no (outside the validated region)'],
    ];
    return this.section(
      'Selected road user',
      h(
        'dl',
        { class: 'bev-dl' },
        rows.flatMap(([k, v]) => [h('dt', {}, k), h('dd', {}, v)]),
      ),
    );
  }

  /** What the symbols on the map mean. */
  private key(): HTMLElement {
    const svg = (body: string): HTMLElement => {
      const span = h('span', {});
      span.innerHTML = `<svg width="22" height="12" viewBox="0 0 22 12" fill="none" stroke="currentColor" stroke-width="1.3">${body}</svg>`;
      return span.firstElementChild as HTMLElement;
    };
    const item = (symbol: string, label: string): HTMLElement => h('span', {}, svg(symbol), label);
    return this.section(
      'Key',
      h(
        'div',
        { class: 'bev-key' },
        item(
          '<rect x="2" y="2" width="15" height="8"/><path d="M17 6 13 3v6z" fill="currentColor"/>',
          'measured size, measured heading',
        ),
        item(
          '<rect x="2" y="2" width="15" height="8" stroke-dasharray="3 2"/>',
          'class size (not measured)',
        ),
        item('<path d="M17 6 13 3v6z"/>', 'heading held while slow'),
        item('<path d="M17 6 13 3v6z" stroke-dasharray="1.5 1.5"/>', 'heading from lane direction'),
        item('<circle cx="11" cy="6" r="5" stroke-dasharray="3 2"/>', 'no heading'),
        item('<ellipse cx="11" cy="6" rx="9" ry="4"/>', 'position 2σ'),
        item('<path d="M2 6h18" stroke-dasharray="3 2" stroke="#a68bfa"/>', 'likely duplicate'),
        item(
          '<circle cx="11" cy="6" r="5" stroke-dasharray="1.5 1.5" stroke="#f6b443"/>',
          'possible ID switch',
        ),
        item(
          '<path d="M2 10 11 2l9 8" stroke-dasharray="3 2" stroke="#3d9bff"/>',
          'camera ground coverage',
        ),
      ),
    );
  }

  private quality(input: PanelInput): HTMLElement {
    const times = input.states.map((s) => s.state.timestamp_ns);
    const skewS = times.length > 1 ? (Math.max(...times) - Math.min(...times)) / 1e9 : null;
    const unaligned = input.duplicates.filter((d) => Math.abs(d.dtS) > MAX_MERGE_SKEW_S).length;
    const heading = { measured: 0, held: 0, lane: 0, unknown: 0 };
    for (const t of input.tracks) {
      heading[t.headingStatus]++;
    }
    return this.section(
      'Data quality',
      h(
        'table',
        { class: 'bev-table compact' },
        h(
          'thead',
          {},
          h(
            'tr',
            {},
            h('th', {}, 'Camera'),
            h('th', { title: 'Age of the latest world state at the browser' }, 'Age'),
            h('th', {}, 'Obj'),
            h('th', { title: 'Objects without heading' }, 'No hdg'),
            h(
              'th',
              { title: 'Objects drawn at the class size (no measured footprint)' },
              'Class size',
            ),
            h(
              'th',
              { title: 'New tracks that started where another had just been lost, of all started' },
              'ID sw?',
            ),
          ),
        ),
        h(
          'tbody',
          {},
          input.quality.map((q) => {
            const state = input.states.find((s) => s.camera === q.camera);
            return h(
              'tr',
              {},
              h('td', {}, state?.name ?? q.camera),
              h('td', { class: 'num' }, state ? `${state.ageMs.toFixed(0)} ms` : '–'),
              h('td', { class: 'num' }, String(q.objects)),
              h('td', { class: 'num' }, String(q.noHeading)),
              h('td', { class: 'num' }, String(q.classSize)),
              h('td', { class: 'num' }, `${q.idSwitches}/${q.tracksStarted}`),
            );
          }),
        ),
      ),
      h(
        'p',
        { class: 'note' },
        `Heading: ${heading.measured} measured, ${heading.held} held, ${heading.lane} from lane, ${heading.unknown} unknown. `,
        input.states.length > 1
          ? `Cameras ${num(skewS === null ? null : skewS * 1000)} ms apart in frame time; ${input.duplicates.length} likely duplicates${unaligned > 0 ? `, ${unaligned} too far apart in time to merge` : ''}. Timestamps of different recordings are only comparable if they started together.`
          : 'One camera: no cross-camera duplicates.',
      ),
    );
  }
}
