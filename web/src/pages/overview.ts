import '../styles/pages.css';

import { setSite, subscribeVideo, type Unsubscribe } from '../net/client';
import {
  type Site,
  siteAction,
  siteState,
  SOURCE_LABELS,
  sourceKind,
  systemInfo,
} from '../net/manager';
import { fill, h, toggle } from '../ui/dom';
import { confirmDialog, toast, withBusy } from '../ui/feedback';
import { formatNumber } from '../ui/format';
import { icon, type IconName } from '../ui/icons';
import { links, mountShell } from '../ui/shell';

/** Live camera thumbnails of the running sites (their /ws/video streams). */
class Thumbnails {
  private readonly streams = new Map<string, { close: Unsubscribe; url: string | null }>();
  private readonly images = new Map<string, HTMLImageElement>();

  image(siteId: string): HTMLImageElement {
    let image = this.images.get(siteId);
    if (!image) {
      image = h('img', { alt: '' });
      image.hidden = true;
      this.images.set(siteId, image);
    }
    return image;
  }

  /** Keeps streams open for running sites only. */
  sync(sites: Site[]): void {
    const running = new Set(sites.filter((s) => s.runtime.state === 'running').map((s) => s.id));
    for (const [id, stream] of this.streams) {
      if (!running.has(id)) {
        stream.close();
        this.streams.delete(id);
        this.image(id).hidden = true;
      }
    }
    for (const id of running) {
      if (this.streams.has(id)) {
        continue;
      }
      setSite(id);
      const entry: { close: Unsubscribe; url: string | null } = {
        close: () => undefined,
        url: null,
      };
      entry.close = subscribeVideo((frame) => {
        const image = this.image(id);
        if (entry.url) {
          URL.revokeObjectURL(entry.url);
        }
        entry.url = URL.createObjectURL(frame.jpeg);
        image.src = entry.url;
        image.hidden = false;
      });
      this.streams.set(id, entry);
    }
  }
}

function kpi(
  name: IconName,
  label: string,
  value: string,
  sub: string,
  tone = '',
  unit = '',
): HTMLElement {
  return h(
    'div',
    { class: `kpi ${tone}` },
    h('span', { class: 'label' }, icon(name), label),
    h('span', { class: 'value' }, value, unit ? h('small', {}, unit) : null),
    h('span', { class: 'sub' }, sub),
  );
}

function siteCard(site: Site, thumbs: Thumbnails, refresh: () => Promise<void>): HTMLElement {
  const state = siteState(site);
  const status = site.runtime.status;
  const running = site.runtime.state === 'running';
  const kind = sourceKind(site.source);
  const metric = (label: string, value: string) =>
    h('div', {}, h('span', {}, label), h('strong', {}, value));

  const active = toggle(site.enabled ? 'Active' : 'Inactive', site.enabled, (checked, input) => {
    input.disabled = true;
    void withBusy(
      null,
      () => siteAction(site.id, checked ? 'start' : 'stop'),
      checked ? `${site.name} is starting` : `${site.name} stopped`,
    ).then(async (done) => {
      if (!done) {
        input.checked = !checked;
      }
      input.disabled = false;
      await refresh();
    });
  });

  const restart = h(
    'button',
    { type: 'button', class: 'btn ghost small icon-only', title: 'Restart site' },
    icon('restart'),
  );
  restart.disabled = !site.enabled;
  restart.addEventListener('click', async () => {
    const ok = await confirmDialog({
      title: `Restart ${site.name}?`,
      body: 'The perception server of this site restarts; viewers reconnect automatically.',
      confirm: 'Restart',
    });
    if (ok) {
      await withBusy(restart, () => siteAction(site.id, 'restart'), 'Restarting…');
      await refresh();
    }
  });

  const tags = [
    h('span', { class: 'tag' }, icon(kind === 'stream' ? 'stream' : 'video'), SOURCE_LABELS[kind]),
    site.calibrated
      ? h('span', { class: 'tag ok' }, icon('check'), 'Calibrated')
      : h('span', { class: 'tag warn' }, icon('calibration'), 'Not calibrated'),
    site.has_map ? h('span', { class: 'tag' }, icon('map'), 'Map') : null,
    site.kind === 'benchmark' ? h('span', { class: 'tag' }, icon('database'), 'Benchmark') : null,
  ];

  return h(
    'article',
    { class: 'card site-card' },
    h(
      'a',
      { class: 'site-thumb', href: links.twin(site.id) },
      thumbs.image(site.id),
      h(
        'div',
        { class: 'placeholder' },
        icon(running ? 'video' : 'power'),
        running ? 'Waiting for the camera image…' : site.enabled ? state.label : 'Site inactive',
      ),
      h(
        'div',
        { class: 'state' },
        h('span', { class: `pill ${state.tone}${running ? ' live' : ''}` }, state.label),
      ),
    ),
    h(
      'div',
      { class: 'site-info' },
      h(
        'div',
        { class: 'site-title' },
        h(
          'div',
          {},
          h('h3', {}, site.name),
          h('p', {}, site.description || `${site.id} · ${site.source.input || 'no source set'}`),
        ),
      ),
      h(
        'div',
        { class: 'site-metrics' },
        metric('Road users', running ? formatNumber(status?.objects) : '–'),
        metric('Events', running ? formatNumber(status?.active_events ?? 0) : '–'),
        metric('Rate', running ? formatNumber(status?.fps, 1, 'fps') : '–'),
        metric('Latency', running ? formatNumber(status?.latency_ms, 0, 'ms') : '–'),
      ),
      site.runtime.error && site.runtime.state !== 'running'
        ? h('p', { class: 'site-error' }, site.runtime.error)
        : null,
      h('div', { class: 'site-tags' }, tags),
      h(
        'div',
        { class: 'site-actions' },
        h(
          'a',
          { class: 'btn primary small', href: links.twin(site.id) },
          icon('twin'),
          'Open twin',
        ),
        h(
          'a',
          { class: 'btn small', href: links.calibration(site.id), title: 'Calibration and zones' },
          icon('calibration'),
        ),
        h(
          'a',
          { class: 'btn small', href: links.settings(site.id), title: 'Settings' },
          icon('settings'),
        ),
        restart,
        h('span', { class: 'spacer' }),
        active,
      ),
    ),
  );
}

async function start(): Promise<void> {
  const shell = await mountShell({ page: 'overview', title: 'Overview' });
  if (!shell.managed) {
    location.replace('/twin.html');
    return;
  }
  shell.setActions(h('a', { class: 'btn primary', href: links.newSite }, icon('plus'), 'New site'));
  const kpis = h('section', { class: 'kpis' });
  const grid = h('section', { class: 'sites-grid' });
  const banner = h('div', {});
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
          h('h1', {}, 'Overview'),
          h('p', {}, 'Every monitored intersection, its perception pipeline and live road users.'),
        ),
      ),
      banner,
      kpis,
      h('h2', { class: 'section-title' }, 'Sites'),
      grid,
    ),
  );

  const thumbs = new Thumbnails();
  let gpuText = '–';
  let gpuSub = 'no GPU found';
  let backendOn = true;
  const render = (sites: Site[]): void => {
    thumbs.sync(sites);
    const running = sites.filter((s) => s.runtime.state === 'running');
    const objects = running.reduce((sum, s) => sum + (s.runtime.status?.objects ?? 0), 0);
    const events = running.reduce((sum, s) => sum + (s.runtime.status?.active_events ?? 0), 0);
    const latencies = running
      .map((s) => s.runtime.status?.latency_ms)
      .filter((v): v is number => v !== undefined);
    const latency =
      latencies.length > 0 ? latencies.reduce((a, b) => a + b, 0) / latencies.length : null;
    const failing = sites.filter((s) => ['failed', 'backoff'].includes(s.runtime.state));
    fill(
      kpis,
      kpi(
        'server',
        'Active sites',
        `${running.length}`,
        `${sites.filter((s) => s.enabled).length} enabled · ${sites.length} configured`,
        failing.length > 0 ? 'danger' : running.length > 0 ? 'ok' : '',
        `/ ${sites.length}`,
      ),
      kpi('activity', 'Road users tracked', `${objects}`, 'across all running sites'),
      kpi(
        'conflict',
        'Active safety events',
        `${events}`,
        'VRUs on crosswalks and conflicts',
        events > 0 ? 'warn' : '',
      ),
      kpi('clock', 'Pipeline latency', formatNumber(latency), 'mean per frame', '', 'ms'),
      kpi('gpu', 'GPU load', gpuText, gpuSub),
    );
    fill(
      banner,
      !backendOn
        ? h(
            'div',
            { class: 'banner danger' },
            icon('power'),
            h('span', {}, 'The perception backend is stopped. No site is processing video.'),
            h('a', { class: 'btn small', href: links.system }, 'Open system control'),
          )
        : failing.length > 0
          ? h(
              'div',
              { class: 'banner' },
              icon('alert'),
              h('span', {}, `${failing.map((s) => s.name).join(', ')}: the server keeps failing.`),
              h('a', { class: 'btn small', href: links.logs(failing[0]?.id) }, 'Show logs'),
            )
          : null,
    );
    if (sites.length === 0) {
      fill(
        grid,
        h(
          'div',
          { class: 'card empty', style: 'grid-column: 1 / -1' },
          icon('pin'),
          h('h3', {}, 'No sites yet'),
          h(
            'p',
            {},
            'A site is one intersection with its camera. Create one, calibrate the camera against the map and the digital twin starts.',
          ),
          h(
            'a',
            { class: 'btn primary', href: links.newSite },
            icon('plus'),
            'Create the first site',
          ),
        ),
      );
      return;
    }
    fill(
      grid,
      sites.map((site) => siteCard(site, thumbs, shell.refresh)),
      h(
        'a',
        { class: 'new-card', href: links.newSite },
        icon('plus'),
        h('strong', {}, 'Add a site'),
        h('span', {}, 'Camera video or stream of a new intersection'),
      ),
    );
  };

  const refreshSystem = async (): Promise<void> => {
    try {
      const system = await systemInfo();
      backendOn = system.backend_on;
      const gpu = system.gpus?.[0];
      if (gpu) {
        gpuText = formatNumber(gpu.utilization, 0, '%');
        gpuSub = `${gpu.name} · ${formatNumber((gpu.memory_used_mib ?? 0) / 1024, 1)} / ${formatNumber((gpu.memory_total_mib ?? 0) / 1024, 1)} GB`;
      }
    } catch (error) {
      toast((error as Error).message, 'error');
    }
  };
  await refreshSystem();
  setInterval(() => void refreshSystem(), 5000);
  shell.onSites(render);
  render(shell.sites());
}

void start();
