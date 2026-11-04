import '../styles/pages.css';

import { cameraBase, subscribeVideo, type Unsubscribe } from '../net/client';
import {
  APPROACH_LABELS,
  type Camera,
  cameraState,
  type Site,
  siteAction,
  siteFigures,
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

/** Live camera thumbnails of the running cameras (their /ws/video streams). */
class Thumbnails {
  private readonly streams = new Map<string, { close: Unsubscribe; url: string | null }>();
  private readonly images = new Map<string, HTMLImageElement>();

  image(key: string): HTMLImageElement {
    let image = this.images.get(key);
    if (!image) {
      image = h('img', { alt: '' });
      image.hidden = true;
      this.images.set(key, image);
    }
    return image;
  }

  /** Keeps streams open for running cameras only. */
  sync(sites: Site[]): void {
    const running = new Map<string, string>();
    for (const site of sites) {
      for (const camera of site.cameras) {
        if (camera.runtime.state === 'running') {
          running.set(`${site.id}/${camera.id}`, cameraBase(site.id, camera.id));
        }
      }
    }
    for (const [key, stream] of this.streams) {
      if (!running.has(key)) {
        stream.close();
        this.streams.delete(key);
        this.image(key).hidden = true;
      }
    }
    for (const [key, base] of running) {
      if (this.streams.has(key)) {
        continue;
      }
      const entry: { close: Unsubscribe; url: string | null } = {
        close: () => undefined,
        url: null,
      };
      entry.close = subscribeVideo(
        (frame) => {
          const image = this.image(key);
          if (entry.url) {
            URL.revokeObjectURL(entry.url);
          }
          entry.url = URL.createObjectURL(frame.jpeg);
          image.src = entry.url;
          image.hidden = false;
        },
        () => undefined,
        base,
      );
      this.streams.set(key, entry);
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

/** Camera images of a site: one image, or a 2 × 2 mosaic for intersections with several. */
function thumbnail(site: Site, thumbs: Thumbnails): HTMLElement {
  const state = siteState(site);
  const tiles = site.cameras.slice(0, 4).map((camera) => {
    const running = camera.runtime.state === 'running';
    return h(
      'div',
      { class: 'tile' },
      thumbs.image(`${site.id}/${camera.id}`),
      h(
        'div',
        { class: 'placeholder' },
        icon(running ? 'video' : 'power'),
        running ? 'Waiting for the image…' : cameraState(site, camera).label,
      ),
      site.cameras.length > 1 ? h('span', { class: 'tile-label' }, camera.name) : null,
    );
  });
  const live = site.runtime.state === 'running' || site.runtime.state === 'partial';
  return h(
    'a',
    {
      class: `site-thumb${site.cameras.length > 1 ? ' mosaic' : ''}`,
      href: links.twin(site.id),
    },
    tiles.length > 0
      ? tiles
      : h('div', { class: 'tile' }, h('div', { class: 'placeholder' }, icon('video'), 'No camera')),
    h(
      'div',
      { class: 'state' },
      h('span', { class: `pill ${state.tone}${live ? ' live' : ''}` }, state.label),
    ),
  );
}

function cameraChip(site: Site, camera: Camera): HTMLElement {
  const state = cameraState(site, camera);
  const kind = sourceKind(camera.source);
  const details = [camera.name, APPROACH_LABELS[camera.approach], SOURCE_LABELS[kind], state.label];
  if (!camera.calibrated) {
    details.push('not calibrated');
  }
  return h(
    'a',
    {
      class: 'camera-chip',
      href: links.calibration(site.id, camera.id),
      title: details.join(' · '),
    },
    h('span', { class: `dot ${state.tone}` }),
    icon(kind === 'stream' ? 'stream' : 'video'),
    h('span', { class: 'name' }, camera.name),
    camera.calibrated ? null : h('i', { class: 'todo-dot' }),
  );
}

function siteCard(site: Site, thumbs: Thumbnails, refresh: () => Promise<void>): HTMLElement {
  const figures = siteFigures(site);
  const live = figures.running > 0;
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
    { type: 'button', class: 'btn ghost small icon-only', title: 'Restart all cameras' },
    icon('restart'),
  );
  restart.disabled = !site.enabled;
  restart.addEventListener('click', async () => {
    const ok = await confirmDialog({
      title: `Restart ${site.name}?`,
      body: 'The perception servers of all its cameras restart; viewers reconnect automatically.',
      confirm: 'Restart',
    });
    if (ok) {
      await withBusy(restart, () => siteAction(site.id, 'restart'), 'Restarting…');
      await refresh();
    }
  });

  const count = site.cameras.length;
  const calibrated = site.cameras.filter((c) => c.calibrated).length;
  const tags = [
    h(
      'span',
      { class: 'tag' },
      icon('junction'),
      site.layout === 'single' ? 'Single camera' : `${count} camera${count === 1 ? '' : 's'}`,
    ),
    count === 0
      ? null
      : calibrated === count
        ? h('span', { class: 'tag ok' }, icon('check'), 'Calibrated')
        : h(
            'span',
            { class: 'tag warn' },
            icon('calibration'),
            `${calibrated} of ${count} calibrated`,
          ),
    site.has_map ? h('span', { class: 'tag' }, icon('map'), 'Map') : null,
    site.kind === 'benchmark' ? h('span', { class: 'tag' }, icon('database'), 'Benchmark') : null,
  ];
  const where = site.geo_anchor
    ? `${site.id} · ${site.geo_anchor.lat_deg.toFixed(5)}, ${site.geo_anchor.lon_deg.toFixed(5)}`
    : site.id;

  return h(
    'article',
    { class: 'card site-card' },
    thumbnail(site, thumbs),
    h(
      'div',
      { class: 'site-info' },
      h(
        'div',
        { class: 'site-title' },
        h('div', {}, h('h3', {}, site.name), h('p', {}, site.description || where)),
      ),
      count > 0
        ? h(
            'div',
            { class: 'camera-chips' },
            site.cameras.map((c) => cameraChip(site, c)),
          )
        : null,
      h(
        'div',
        { class: 'site-metrics' },
        metric('Road users', live ? formatNumber(figures.objects) : '–'),
        metric('Rate', live ? formatNumber(figures.fps, 1, 'fps') : '–'),
        metric('Latency', live ? formatNumber(figures.latency_ms, 0, 'ms') : '–'),
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
          { class: 'btn small', href: links.settings(site.id), title: 'Settings and cameras' },
          icon('settings'),
        ),
        restart,
        h('span', { class: 'spacer' }),
        active,
      ),
    ),
  );
}

function failingLink(failing: Site[]): string {
  const site = failing[0];
  const camera = site?.cameras.find((c) => ['failed', 'backoff'].includes(c.runtime.state));
  return links.logs(site && camera ? `${site.id}/${camera.id}` : site?.id);
}

async function start(): Promise<void> {
  const shell = await mountShell({ page: 'overview', title: 'Overview' });
  if (!shell.managed) {
    location.replace('/twin.html');
    return;
  }
  shell.setActions(
    h('a', { class: 'btn', href: links.twin() }, icon('globe'), 'Digital twin'),
    h('a', { class: 'btn primary', href: links.newSite }, icon('plus'), 'New site'),
  );
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
          h(
            'p',
            {},
            'Every monitored intersection, its cameras, perception pipelines and live road users.',
          ),
        ),
      ),
      banner,
      kpis,
      h('h2', { class: 'section-title' }, 'Intersections'),
      grid,
    ),
  );

  const thumbs = new Thumbnails();
  let gpuText = '–';
  let gpuSub = 'no GPU found';
  let backendOn = true;
  const render = (sites: Site[]): void => {
    thumbs.sync(sites);
    const figures = sites.map(siteFigures);
    const cameras = sites.flatMap((s) => s.cameras);
    const runningCameras = figures.reduce((sum, f) => sum + f.running, 0);
    const runningSites = figures.filter((f) => f.running > 0).length;
    const objects = figures.reduce((sum, f) => sum + (f.objects ?? 0), 0);
    const latencies = figures.map((f) => f.latency_ms).filter((v): v is number => v !== null);
    const latency =
      latencies.length > 0 ? latencies.reduce((a, b) => a + b, 0) / latencies.length : null;
    const failing = sites.filter((s) =>
      s.cameras.some((c) => ['failed', 'backoff'].includes(c.runtime.state)),
    );
    fill(
      kpis,
      kpi(
        'junction',
        'Active intersections',
        `${runningSites}`,
        `${sites.filter((s) => s.enabled).length} enabled · ${sites.length} configured`,
        failing.length > 0 ? 'danger' : runningSites > 0 ? 'ok' : '',
        `/ ${sites.length}`,
      ),
      kpi(
        'video',
        'Cameras live',
        `${runningCameras}`,
        `${cameras.filter((c) => c.calibrated).length} of ${cameras.length} calibrated`,
        runningCameras > 0 ? 'ok' : '',
        `/ ${cameras.length}`,
      ),
      kpi('activity', 'Road users', `${objects}`, 'in the site world models'),
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
            h('span', {}, 'The perception backend is stopped. No camera is processing video.'),
            h('a', { class: 'btn small', href: links.system }, 'Open system control'),
          )
        : failing.length > 0
          ? h(
              'div',
              { class: 'banner' },
              icon('alert'),
              h(
                'span',
                {},
                `${failing.map((s) => s.name).join(', ')}: a camera server keeps failing.`,
              ),
              h('a', { class: 'btn small', href: failingLink(failing) }, 'Show logs'),
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
            'A site is one intersection with up to four cameras. Create it at the intersection centre, calibrate each camera against the map and the digital twin starts.',
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
        h('strong', {}, 'Add an intersection'),
        h('span', {}, 'Its location, map and one to four cameras'),
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
