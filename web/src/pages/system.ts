import '../styles/pages.css';

import {
  type Site,
  siteAction,
  siteState,
  systemAction,
  type SystemAction,
  type SystemInfo,
  systemInfo,
} from '../net/manager';
import { fill, h } from '../ui/dom';
import { confirmDialog, toast, withBusy } from '../ui/feedback';
import { formatBytes, formatDuration, formatNumber } from '../ui/format';
import { icon, type IconName } from '../ui/icons';
import { links, mountShell } from '../ui/shell';

const REFRESH_MS = 2000;

function meter(fraction: number | null): HTMLElement {
  const value = fraction === null ? 0 : Math.min(Math.max(fraction, 0), 1);
  const tone = value > 0.9 ? ' danger' : value > 0.75 ? ' warn' : '';
  return h(
    'div',
    { class: `meter${tone}` },
    h('span', { style: `width:${(value * 100).toFixed(1)}%` }),
  );
}

function resource(
  name: IconName,
  label: string,
  value: string,
  fraction: number | null,
): HTMLElement {
  return h(
    'div',
    { class: 'resource' },
    h('div', { class: 'row' }, icon(name), h('span', {}, label), h('strong', {}, value)),
    meter(fraction),
  );
}

const ACTIONS: Record<
  SystemAction,
  { title: string; body: string; confirm: string; done: string; danger?: boolean }
> = {
  'start-backend': {
    title: 'Start the backend?',
    body: 'Starts the perception servers of all active sites.',
    confirm: 'Start',
    done: 'Backend starting',
  },
  'stop-backend': {
    title: 'Stop the backend?',
    body: 'All perception servers stop: no site processes video until the backend is started again. The console stays available.',
    confirm: 'Stop backend',
    done: 'Backend stopped',
    danger: true,
  },
  'restart-backend': {
    title: 'Restart the backend?',
    body: 'All perception servers stop and start again, e.g. after a new build or a model change. Viewers reconnect automatically.',
    confirm: 'Restart backend',
    done: 'Backend restarting',
  },
  'restart-manager': {
    title: 'Restart the console service?',
    body: 'The site manager restarts together with all sites (its container restarts). The console is unavailable for a few seconds.',
    confirm: 'Restart service',
    done: 'Console service restarting',
    danger: true,
  },
};

async function runAction(
  action: SystemAction,
  button: HTMLButtonElement,
  after: () => void,
): Promise<void> {
  const text = ACTIONS[action];
  if (
    !(await confirmDialog({
      title: text.title,
      body: text.body,
      confirm: text.confirm,
      danger: text.danger,
    }))
  ) {
    return;
  }
  await withBusy(button, () => systemAction(action), text.done);
  after();
}

function controlCard(system: SystemInfo, refresh: () => void): HTMLElement {
  const button = (action: SystemAction, label: string, name: IconName, cls: string) => {
    const b = h('button', { type: 'button', class: `btn ${cls}` }, icon(name), label);
    b.addEventListener('click', () => void runAction(action, b, refresh));
    return b;
  };
  const on = system.backend_on;
  return h(
    'section',
    { class: 'card' },
    h(
      'header',
      { class: 'card-header' },
      icon('power'),
      h('div', {}, h('h2', {}, 'Backend control'), h('p', {}, 'Perception servers of all sites')),
    ),
    h(
      'div',
      { class: 'card-body control-panel' },
      h(
        'div',
        { class: `control-state${on ? '' : ' off'}` },
        h('span', { class: 'big-dot' }, icon(on ? 'activity' : 'power')),
        h(
          'div',
          {},
          h('strong', {}, on ? 'Backend running' : 'Backend stopped'),
          h(
            'span',
            {},
            `${system.sites_running} of ${system.sites_enabled} active sites running · at most ${system.max_running} at once`,
          ),
        ),
      ),
      system.server_binary_found
        ? null
        : h(
            'div',
            { class: 'banner danger' },
            icon('alert'),
            h(
              'span',
              {},
              `Server binary not found: ${system.server_binary}. Build the C++ code first.`,
            ),
          ),
      h(
        'div',
        { class: 'row' },
        on
          ? button('stop-backend', 'Stop', 'stop', 'danger')
          : button('start-backend', 'Start', 'play', 'primary'),
        button('restart-backend', 'Restart backend', 'restart', ''),
        h('span', { class: 'spacer' }),
        button('restart-manager', 'Restart service', 'server', 'ghost'),
      ),
      h(
        'dl',
        { class: 'kv' },
        h('dt', {}, 'Version'),
        h('dd', { class: 'mono' }, system.version),
        h('dt', {}, 'Console up'),
        h('dd', {}, formatDuration(system.uptime_s)),
        h('dt', {}, 'Data root'),
        h('dd', { class: 'mono' }, system.data_root),
      ),
    ),
  );
}

function resourcesCard(system: SystemInfo): HTMLElement {
  const host = system.host;
  const load = host.load?.[0] ?? null;
  const cpus = host.cpu_count ?? 1;
  const items: HTMLElement[] = [
    resource(
      'cpu',
      `CPU load (${cpus} cores)`,
      load === null ? '–' : load.toFixed(2),
      load === null ? null : load / cpus,
    ),
    resource(
      'memory',
      'Memory',
      host.memory ? `${formatBytes(host.memory.used)} / ${formatBytes(host.memory.total)}` : '–',
      host.memory ? host.memory.used / host.memory.total : null,
    ),
    resource(
      'disk',
      'Disk (data root)',
      host.disk ? `${formatBytes(host.disk.used)} / ${formatBytes(host.disk.total)}` : '–',
      host.disk ? host.disk.used / host.disk.total : null,
    ),
  ];
  for (const gpu of system.gpus ?? []) {
    items.push(
      resource(
        'gpu',
        `${gpu.name} · load`,
        formatNumber(gpu.utilization, 0, '%'),
        gpu.utilization === null ? null : gpu.utilization / 100,
      ),
      resource(
        'memory',
        `GPU memory · ${formatNumber(gpu.temperature_c, 0, '°C')} · ${formatNumber(gpu.power_w, 0, 'W')}`,
        `${formatNumber((gpu.memory_used_mib ?? 0) / 1024, 1)} / ${formatNumber((gpu.memory_total_mib ?? 0) / 1024, 1)} GB`,
        gpu.memory_used_mib !== null && gpu.memory_total_mib
          ? gpu.memory_used_mib / gpu.memory_total_mib
          : null,
      ),
    );
  }
  return h(
    'section',
    { class: 'card' },
    h(
      'header',
      { class: 'card-header' },
      icon('activity'),
      h(
        'div',
        {},
        h('h2', {}, 'Resources'),
        h('p', {}, system.gpus ? 'Host and GPU' : 'Host · no NVIDIA GPU visible'),
      ),
    ),
    h('div', { class: 'card-body', style: 'padding-top: 8px; padding-bottom: 8px' }, items),
  );
}

function processRow(site: Site, refresh: () => void): HTMLTableRowElement {
  const r = site.runtime;
  const state = siteState(site);
  const act = (label: string, name: IconName, action: 'start' | 'stop' | 'restart') => {
    const b = h(
      'button',
      { type: 'button', class: 'btn ghost small icon-only', title: label },
      icon(name),
    );
    b.addEventListener(
      'click',
      () =>
        void withBusy(b, () => siteAction(site.id, action), `${label}: ${site.name}`).then(refresh),
    );
    return b;
  };
  return h(
    'tr',
    {},
    h(
      'td',
      { class: 'strong' },
      h('a', { href: links.settings(site.id), style: 'color: inherit' }, site.name),
      h('div', { class: 'muted mono' }, site.id),
    ),
    h('td', {}, h('span', { class: `pill ${state.tone}` }, state.label)),
    h('td', { class: 'mono' }, r.pid ? `${r.pid}` : '–'),
    h('td', { class: 'mono' }, r.port ? `${r.port}` : '–'),
    h('td', { class: 'num' }, formatDuration(r.uptime_s)),
    h(
      'td',
      { class: 'num' },
      `${r.restarts}${r.last_exit !== null ? ` (exit ${r.last_exit})` : ''}`,
    ),
    h('td', { class: 'num' }, r.state === 'running' ? formatNumber(r.status?.fps, 1) : '–'),
    h(
      'td',
      { class: 'num' },
      r.state === 'running' ? formatNumber(r.status?.latency_ms, 0, 'ms') : '–',
    ),
    h(
      'td',
      { style: 'text-align: right; white-space: nowrap' },
      site.enabled ? act('Stop', 'stop', 'stop') : act('Start', 'play', 'start'),
      act('Restart', 'restart', 'restart'),
      h(
        'a',
        { class: 'btn ghost small icon-only', href: links.logs(site.id), title: 'Logs' },
        icon('logs'),
      ),
    ),
  );
}

async function start(): Promise<void> {
  const shell = await mountShell({ page: 'system', title: 'System' });
  if (!shell.managed) {
    location.replace('/twin.html');
    return;
  }
  shell.setActions(h('a', { class: 'btn small', href: links.logs() }, icon('logs'), 'All logs'));
  const top = h('div', { class: 'grid-2' });
  const processes = h('tbody', {});
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
          h('h1', {}, 'System'),
          h(
            'p',
            {},
            'Control the perception backend, watch the host and the server process of every site.',
          ),
        ),
      ),
      top,
      h(
        'section',
        { class: 'card', style: 'margin-top: 18px' },
        h(
          'header',
          { class: 'card-header' },
          icon('server'),
          h(
            'div',
            {},
            h('h2', {}, 'Site servers'),
            h('p', {}, 'One jp_server process per running site'),
          ),
        ),
        h(
          'div',
          { style: 'overflow-x: auto' },
          h(
            'table',
            { class: 'table' },
            h(
              'thead',
              {},
              h(
                'tr',
                {},
                ['Site', 'State', 'PID', 'Port', 'Uptime', 'Restarts', 'FPS', 'Latency', ''].map(
                  (t) => h('th', {}, t),
                ),
              ),
            ),
            processes,
          ),
        ),
      ),
    ),
  );

  let busy = false;
  const refresh = async (): Promise<void> => {
    if (busy) {
      return;
    }
    busy = true;
    try {
      const system = await systemInfo();
      // Keep focus and dialogs stable: only redraw while no confirmation is open.
      if (!document.querySelector('.backdrop')) {
        fill(
          top,
          controlCard(system, () => void refresh()),
          resourcesCard(system),
        );
      }
    } catch (error) {
      toast((error as Error).message, 'error');
    } finally {
      busy = false;
    }
  };
  const renderProcesses = (sites: Site[]): void => {
    fill(
      processes,
      sites.length > 0
        ? sites.map((s) => processRow(s, () => void shell.refresh()))
        : h('tr', {}, h('td', { colspan: 9, class: 'muted' }, 'No sites configured.')),
    );
  };
  shell.onSites(renderProcesses);
  renderProcesses(shell.sites());
  await refresh();
  setInterval(() => void refresh(), REFRESH_MS);
}

void start();
