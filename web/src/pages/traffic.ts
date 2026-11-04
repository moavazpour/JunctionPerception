import '../styles/pages.css';

import { subscribeSite } from '../net/client';
import type { SiteState } from '../net/siteState';
import { fill, h, param } from '../ui/dom';
import { formatDuration } from '../ui/format';
import { icon, type IconName } from '../ui/icons';
import { links, mountShell } from '../ui/shell';

// Site states arrive with every camera frame; the page is redrawn at most this often.
const RENDER_MS = 500;

function kpi(name: IconName, label: string, value: string, sub: string): HTMLElement {
  return h(
    'div',
    { class: 'kpi' },
    h('span', { class: 'label' }, icon(name), label),
    h('span', { class: 'value' }, value),
    h('span', { class: 'sub' }, sub),
  );
}

function byClass(counts: Record<string, number>): string {
  const parts = Object.entries(counts).map(([cls, n]) => `${n} ${cls}`);
  return parts.length > 0 ? parts.join(' · ') : '–';
}

function card(name: IconName, title: string, sub: string, body: HTMLElement): HTMLElement {
  return h(
    'section',
    { class: 'card', style: 'margin-top: 18px' },
    h('header', { class: 'card-header' }, icon(name), h('div', {}, h('h2', {}, title), h('p', {}, sub))),
    body,
  );
}

function table(head: string[], rows: HTMLElement[], empty: string): HTMLElement {
  return h(
    'div',
    { style: 'overflow-x: auto' },
    h(
      'table',
      { class: 'table' },
      h('thead', {}, h('tr', {}, head.map((t) => h('th', {}, t)))),
      h(
        'tbody',
        {},
        rows.length > 0 ? rows : h('tr', {}, h('td', { colspan: String(head.length) }, empty)),
      ),
    ),
  );
}

/** The site's traffic now: as the manager measured it (manager/traffic.py), drawn only. */
function render(state: SiteState): HTMLElement[] {
  const { traffic } = state;
  const running = formatDuration(Math.max(0, (state.t_ns - traffic.since_ns) / 1e9));
  const unset = traffic.lanes.filter((l) => !l.direction_set).length;
  const lanes = traffic.lanes.map((lane) =>
    h(
      'tr',
      {},
      h('td', {}, h('strong', {}, lane.name)),
      h('td', {}, lane.camera),
      h(
        'td',
        {},
        lane.direction_set
          ? h('span', { class: 'pill ok' }, 'set')
          : h(
              'span',
              { class: 'pill warn', title: 'Set it in the calibration (Regions): not counted' },
              'not set',
            ),
      ),
      h('td', { class: 'num' }, String(lane.vehicles)),
      h('td', { class: 'num' }, lane.queue > 0 ? h('strong', {}, String(lane.queue)) : '0'),
      h('td', { class: 'num' }, lane.direction_set ? String(lane.passed) : '–'),
      h('td', {}, lane.direction_set ? byClass(lane.passed_by_class) : '–'),
    ),
  );
  const signals = state.signals.map((s) =>
    h(
      'tr',
      {},
      h('td', {}, s.name),
      h('td', {}, s.camera),
      h('td', {}, h('span', { class: `pill ${s.state === 'green' ? 'ok' : s.state === 'red' ? 'danger' : 'warn'}` }, s.state)),
      h('td', { class: 'num' }, formatDuration(Math.max(0, (state.t_ns - s.since_ns) / 1e9))),
    ),
  );
  return [
    h(
      'section',
      { class: 'kpis' },
      kpi('activity', 'Road users now', String(state.road_users.length), byClass(traffic.present_by_class)),
      kpi(
        'video',
        'Cameras',
        `${state.cameras.filter((c) => c.state === 'ok').length}/${state.cameras.length}`,
        `fusion ${state.fusion.mode}${state.fusion.reason ? ` (${state.fusion.reason})` : ''}`,
      ),
      kpi(
        'logs',
        'Counting for',
        running,
        'since the start of this run (seek, loop or restart starts over)',
      ),
    ),
    card(
      'activity',
      'Lanes',
      unset > 0
        ? `${unset} lane(s) without a direction of travel are not counted`
        : 'Queue: vehicles standing or creeping in the lane. Passed: drove through it in its direction.',
      table(
        ['Lane', 'Camera', 'Direction', 'In lane', 'Queue', 'Passed', 'Passed by class'],
        lanes,
        'No lanes: draw them as lines with three or more points in the calibration (Regions).',
      ),
    ),
    card(
      'activity',
      'Traffic lights',
      'Colour each camera reads now',
      table(['Signal', 'Camera', 'State', 'For'], signals, 'No traffic lights read.'),
    ),
  ];
}

async function start(): Promise<void> {
  const siteId = param('site');
  const shell = await mountShell({ page: 'traffic', siteId, title: 'Traffic' });
  const site = shell.sites().find((s) => s.id === siteId) ?? shell.sites()[0];
  const body = h('div', {}, h('p', { class: 'empty small' }, 'Waiting for the site state…'));
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
          h('h1', {}, `Traffic${site ? ` · ${site.name}` : ''}`),
          h('p', {}, 'Queue and vehicles passing each lane, and the intersection now. Live.'),
        ),
      ),
      body,
    ),
  );
  if (!site) {
    fill(body, h('p', { class: 'empty small' }, 'No site.'));
    return;
  }
  shell.setActions(
    h('a', { class: 'btn small', href: links.twin(site.id) }, icon('twin'), 'Live twin'),
  );
  let latest: SiteState | null = null;
  subscribeSite(site.id, (state) => {
    latest = state;
  });
  setInterval(() => {
    if (latest) {
      fill(body, ...render(latest));
      latest = null;
    }
  }, RENDER_MS);
}

void start();
