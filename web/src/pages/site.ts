import '../styles/pages.css';

import {
  deleteSite,
  getSite,
  type Inputs,
  listInputs,
  type Site,
  siteAction,
  type SitePatch,
  siteState,
  updateSite,
} from '../net/manager';
import { type Child, fill, h, param, toggle } from '../ui/dom';
import { confirmDialog, toast, withBusy } from '../ui/feedback';
import { formatDuration } from '../ui/format';
import { icon, type IconName } from '../ui/icons';
import { links, mountShell } from '../ui/shell';
import { imageSizeField, type SourceEditor, sourceEditor } from '../ui/sourceEditor';

const SPEEDS = [0.25, 0.5, 1, 2, 4];
const WIDTHS = [640, 960, 1280, 1920];

function section(id: string, name: IconName, title: string, sub: string, ...body: Child[]) {
  return h(
    'section',
    { class: 'card', id },
    h(
      'header',
      { class: 'card-header' },
      icon(name),
      h('div', {}, h('h2', {}, title), h('p', {}, sub)),
    ),
    h('div', { class: 'card-body stack' }, body),
  );
}

function numberInput(value: number | null | undefined, attrs: Record<string, string | number>) {
  const input = h('input', { class: 'input num', type: 'number', ...attrs });
  input.value = value === null || value === undefined ? '' : String(value);
  return input;
}

function select<T extends string | number>(values: T[], current: T, label: (v: T) => string) {
  const element = h('select', { class: 'input' });
  for (const v of values) {
    element.append(h('option', { value: String(v), selected: v === current }, label(v)));
  }
  return element;
}

function statusLine(ok: boolean, name: IconName, title: string, text: string, ...actions: Child[]) {
  return h(
    'div',
    { class: `status-line ${ok ? 'ok' : 'warn'}` },
    icon(name),
    h('div', {}, h('strong', {}, title), h('span', {}, text)),
    actions,
  );
}

class SettingsForm {
  private initial = '';
  private readonly saveBar = h('div', { class: 'save-bar', hidden: true });
  private readonly name: HTMLInputElement;
  private readonly description: HTMLTextAreaElement;
  private readonly source: SourceEditor;
  private readonly imageSize: ReturnType<typeof imageSizeField>;
  private readonly lat: HTMLInputElement;
  private readonly lon: HTMLInputElement;
  private readonly alt: HTMLInputElement;
  private readonly radius: HTMLInputElement;
  private loop: boolean;
  private readonly speed: HTMLSelectElement;
  private readonly fps: HTMLInputElement;
  private readonly width: HTMLSelectElement;
  private readonly quality: HTMLInputElement;
  private readonly qualityLabel = h('strong', { class: 'num' });
  private readonly runtime = h('div', { class: 'stack' });
  private readonly mapStatus = h('div', {});

  constructor(
    private site: Site,
    inputs: Inputs | null,
    private readonly root: HTMLElement,
  ) {
    const changed = () => this.updateDirty();
    this.name = h('input', { class: 'input', maxlength: 80 });
    this.name.value = site.name;
    this.description = h('textarea', { class: 'input', rows: 2, maxlength: 400 });
    this.description.value = site.description;
    this.source = sourceEditor(site.source, inputs, changed);
    this.imageSize = imageSizeField(site.image_size_px ?? [1920, 1080], changed);
    this.lat = numberInput(site.geo_anchor?.lat_deg, {
      step: 0.000001,
      min: -90,
      max: 90,
      placeholder: '48.137154',
    });
    this.lon = numberInput(site.geo_anchor?.lon_deg, {
      step: 0.000001,
      min: -180,
      max: 180,
      placeholder: '11.576124',
    });
    this.alt = numberInput(site.geo_anchor?.alt_m ?? 0, { step: 0.1 });
    this.radius = numberInput(site.map_radius_m ?? 250, { step: 10, min: 50, max: 1000 });
    this.loop = site.playback.loop;
    this.speed = select(SPEEDS, site.playback.speed, (v) => `${v}×`);
    this.fps = numberInput(site.video.max_fps, { step: 0.5, min: 0.5, max: 30 });
    this.width = select(WIDTHS, site.video.width_px, (v) => `${v} px`);
    this.quality = h('input', { type: 'range', min: 30, max: 95, step: 5 });
    this.quality.value = String(site.video.jpeg_quality);
    this.qualityLabel.textContent = this.quality.value;
    this.quality.addEventListener(
      'input',
      () => (this.qualityLabel.textContent = this.quality.value),
    );
    root.addEventListener('input', changed);
    root.addEventListener('change', changed);
    this.render();
    this.initial = JSON.stringify(this.patch());
  }

  update(site: Site): void {
    this.site = site;
    this.renderRuntime();
    this.renderMap();
  }

  private patch(): SitePatch {
    const lat = Number.parseFloat(this.lat.value);
    const lon = Number.parseFloat(this.lon.value);
    const patch: SitePatch = {
      name: this.name.value.trim(),
      description: this.description.value.trim(),
      source: this.source.value(),
      image_size_px: this.imageSize.value(),
      geo_anchor:
        Number.isFinite(lat) && Number.isFinite(lon)
          ? { lat_deg: lat, lon_deg: lon, alt_m: Number.parseFloat(this.alt.value) || 0 }
          : this.site.geo_anchor,
      map_radius_m: Number.parseFloat(this.radius.value) || 250,
      playback: { loop: this.loop, speed: Number(this.speed.value) },
      video: {
        max_fps: Number.parseFloat(this.fps.value) || 5,
        width_px: Number(this.width.value),
        jpeg_quality: Number(this.quality.value),
      },
    };
    return patch;
  }

  private updateDirty(): void {
    const dirty = JSON.stringify(this.patch()) !== this.initial;
    this.saveBar.hidden = !dirty;
  }

  private async save(button: HTMLButtonElement): Promise<void> {
    const problem = this.source.problem();
    if (problem) {
      toast(problem, 'error');
      return;
    }
    if (!this.name.value.trim()) {
      toast('The site needs a name.', 'error');
      return;
    }
    const running = this.site.runtime.state === 'running' || this.site.runtime.state === 'starting';
    await withBusy(
      button,
      async () => {
        this.site = await updateSite(this.site.id, this.patch());
        this.initial = JSON.stringify(this.patch());
        this.updateDirty();
        this.update(this.site);
      },
      running ? 'Settings saved; the site restarts with them.' : 'Settings saved.',
    );
  }

  private render(): void {
    const site = this.site;
    const save = h(
      'button',
      { type: 'button', class: 'btn primary' },
      icon('check'),
      'Save changes',
    );
    save.addEventListener('click', () => void this.save(save));
    fill(
      this.saveBar,
      icon('info'),
      h(
        'span',
        {},
        site.enabled ? 'Unsaved changes · the site restarts when you save.' : 'Unsaved changes',
      ),
      h(
        'button',
        { type: 'button', class: 'btn ghost', onclick: () => location.reload() },
        'Discard',
      ),
      save,
    );
    const remove = h(
      'button',
      { type: 'button', class: 'btn danger' },
      icon('trash'),
      'Delete site',
    );
    remove.addEventListener('click', async () => {
      const ok = await confirmDialog({
        title: `Delete ${site.name}?`,
        body: 'The site stops and its folder (scene, calibration, zones) moves to the trash of the console (.cache/manager/trash on the server).',
        confirm: 'Delete site',
        danger: true,
        typeToConfirm: site.id,
      });
      if (ok && (await withBusy(remove, () => deleteSite(site.id), `${site.name} deleted`))) {
        location.href = '/';
      }
    });

    fill(
      this.root,
      h(
        'div',
        { class: 'settings-layout' },
        h(
          'nav',
          { class: 'toc' },
          [
            ['runtime', 'Status'],
            ['general', 'General'],
            ['source', 'Camera source'],
            ['location', 'Location and map'],
            ['calibration', 'Calibration'],
            ['streaming', 'Playback and video'],
            ['danger', 'Danger zone'],
          ].map(([id, label]) => h('a', { href: `#${id}` }, label)),
        ),
        h(
          'div',
          { class: 'stack' },
          section('runtime', 'activity', 'Status', 'Perception server of this site', this.runtime),
          section(
            'general',
            'info',
            'General',
            'How the site appears in the console',
            h(
              'div',
              { class: 'form-grid' },
              h('label', { class: 'field' }, h('span', {}, 'Name'), this.name),
              h(
                'label',
                { class: 'field' },
                h('span', {}, 'Site ID'),
                h('input', { class: 'input mono', value: site.id, disabled: true }),
                h('small', {}, `Folder scenes/${site.id} on the server`),
              ),
              h('label', { class: 'field wide' }, h('span', {}, 'Description'), this.description),
            ),
          ),
          section(
            'source',
            'video',
            'Camera source',
            'Where the video of the intersection comes from',
            this.source.root,
            this.imageSize.root,
          ),
          section(
            'location',
            'pin',
            'Location and map',
            'Geographic anchor of the world frame and the map layers around it',
            h(
              'div',
              { class: 'form-grid' },
              h('label', { class: 'field' }, h('span', {}, 'Latitude (°)'), this.lat),
              h('label', { class: 'field' }, h('span', {}, 'Longitude (°)'), this.lon),
              h('label', { class: 'field' }, h('span', {}, 'Altitude (m)'), this.alt),
              h('label', { class: 'field' }, h('span', {}, 'Map radius (m)'), this.radius),
            ),
            h(
              'p',
              { class: 'help' },
              'The anchor is the origin of the twin (usually the centre of the intersection or the camera). Map layers: orthophoto and LoD2 buildings (Bavaria), roads and crossings from OpenStreetMap.',
            ),
            this.mapStatus,
          ),
          section(
            'calibration',
            'calibration',
            'Calibration',
            'Image-to-ground homography and zones',
            site.calibrated
              ? statusLine(
                  true,
                  'check',
                  'Camera calibrated',
                  'Road users are placed on the map in metres.',
                  h(
                    'a',
                    { class: 'btn small', href: links.calibration(site.id) },
                    'Open calibration',
                  ),
                )
              : statusLine(
                  false,
                  'calibration',
                  'Not calibrated yet',
                  'The twin shows detections only. Click corresponding points in the camera image and on the map.',
                  h(
                    'a',
                    { class: 'btn small primary', href: links.calibration(site.id) },
                    'Calibrate now',
                  ),
                ),
          ),
          section(
            'streaming',
            'play',
            'Playback and video',
            'Timing of recordings and the camera image sent to the console',
            h(
              'div',
              { class: 'form-grid' },
              h('label', { class: 'field' }, h('span', {}, 'Playback speed'), this.speed),
              h(
                'div',
                { class: 'field' },
                h('span', {}, 'At the end of a recording'),
                toggle('Start again (loop)', this.loop, (on) => {
                  this.loop = on;
                  this.updateDirty();
                }),
              ),
              h('label', { class: 'field' }, h('span', {}, 'Camera image rate (fps)'), this.fps),
              h('label', { class: 'field' }, h('span', {}, 'Camera image width'), this.width),
              h(
                'label',
                { class: 'field' },
                h('span', {}, 'JPEG quality'),
                h('div', { class: 'row' }, this.quality, this.qualityLabel),
              ),
            ),
          ),
          h(
            'section',
            { class: 'card danger-zone', id: 'danger' },
            h(
              'header',
              { class: 'card-header' },
              icon('alert'),
              h('div', {}, h('h2', {}, 'Danger zone'), h('p', {}, 'Irreversible from the console')),
            ),
            h(
              'div',
              { class: 'card-body row' },
              h(
                'div',
                { class: 'help', style: 'flex: 1' },
                'Stops the site and removes it from the console. Its files stay in the trash folder on the server and can be restored by hand.',
              ),
              remove,
            ),
          ),
          this.saveBar,
        ),
      ),
    );
    this.renderRuntime();
    this.renderMap();
  }

  private renderRuntime(): void {
    const site = this.site;
    const r = site.runtime;
    const state = siteState(site);
    const action = (
      label: string,
      name: IconName,
      act: 'start' | 'stop' | 'restart',
      cls = 'btn small',
    ) => {
      const button = h('button', { type: 'button', class: cls }, icon(name), label);
      button.addEventListener('click', () => {
        void withBusy(
          button,
          async () => this.update(await siteAction(site.id, act)),
          `${label}: ${site.name}`,
        );
      });
      return button;
    };
    fill(
      this.runtime,
      h(
        'div',
        { class: 'row' },
        h(
          'span',
          { class: `pill ${state.tone}${r.state === 'running' ? ' live' : ''}` },
          state.label,
        ),
        h(
          'span',
          { class: 'muted' },
          r.state === 'running'
            ? `up ${formatDuration(r.uptime_s)} · ${r.status?.fps?.toFixed(1) ?? '–'} fps · ${r.status?.objects ?? 0} road users`
            : site.enabled
              ? 'active, not running'
              : 'inactive: not started with the backend',
        ),
        h('span', { class: 'spacer' }),
        site.enabled
          ? action('Stop', 'stop', 'stop')
          : action('Activate', 'power', 'start', 'btn small primary'),
        site.enabled ? action('Restart', 'restart', 'restart') : null,
        h('a', { class: 'btn small ghost', href: links.logs(site.id) }, icon('logs'), 'Logs'),
      ),
      r.error ? h('div', { class: 'banner danger' }, icon('alert'), h('span', {}, r.error)) : null,
      h(
        'dl',
        { class: 'kv' },
        h('dt', {}, 'Process'),
        h('dd', { class: 'mono' }, r.pid ? `pid ${r.pid} · port ${r.port}` : '–'),
        h('dt', {}, 'Restarts'),
        h(
          'dd',
          {},
          `${r.restarts}${r.last_exit !== null ? ` · last exit code ${r.last_exit}` : ''}`,
        ),
        h('dt', {}, 'Source'),
        h('dd', { class: 'mono' }, `${site.source.mode} · ${site.source.input || '–'}`),
      ),
    );
  }

  private renderMap(): void {
    const site = this.site;
    const job = site.runtime.map_job;
    const prepare = h(
      'button',
      { type: 'button', class: 'btn small' },
      icon('download'),
      site.has_map ? 'Update map layers' : 'Prepare map layers',
    );
    prepare.disabled = !site.geo_anchor || job?.state === 'running';
    prepare.addEventListener('click', () => {
      void withBusy(
        prepare,
        async () => this.update(await siteAction(site.id, 'prepare-map')),
        'Downloading map data…',
      );
    });
    let line: HTMLElement;
    if (job?.state === 'running') {
      line = statusLine(
        false,
        'download',
        'Preparing map layers…',
        job.message,
        h('span', { class: 'spinner' }),
      );
    } else if (job?.state === 'failed') {
      line = statusLine(false, 'alert', 'Map preparation failed', job.message, prepare);
    } else if (site.has_map) {
      line = statusLine(
        true,
        'map',
        'Map layers ready',
        'Orthophoto, OpenStreetMap and buildings are available.',
        prepare,
      );
    } else {
      line = statusLine(
        false,
        'map',
        'No map layers yet',
        site.geo_anchor
          ? 'Download them for the area around the anchor.'
          : 'Set and save the location first.',
        prepare,
      );
    }
    fill(this.mapStatus, line);
  }
}

async function start(): Promise<void> {
  const siteId = param('site');
  const shell = await mountShell({ page: 'settings', siteId, title: 'Settings' });
  if (!siteId || !shell.managed) {
    location.replace('/');
    return;
  }
  const root = h('div', { class: 'page' });
  shell.content.append(root);
  let site: Site;
  try {
    site = await getSite(siteId);
  } catch (error) {
    fill(
      root,
      h(
        'div',
        { class: 'card empty' },
        icon('alert'),
        h('h3', {}, 'Site not found'),
        h('p', {}, (error as Error).message),
      ),
    );
    return;
  }
  shell.setActions(
    h('a', { class: 'btn small', href: links.twin(siteId) }, icon('twin'), 'Live twin'),
    h(
      'a',
      { class: 'btn small', href: links.calibration(siteId) },
      icon('calibration'),
      'Calibration',
    ),
  );
  const inputs = await listInputs().catch(() => null);
  const head = h(
    'div',
    { class: 'page-head' },
    h(
      'div',
      {},
      h('h1', {}, site.name),
      h('p', {}, 'Site settings: camera source, location, playback and the perception server.'),
    ),
  );
  const body = h('div', {});
  root.append(head, body);
  const form = new SettingsForm(site, inputs, body);
  shell.onSites((sites) => {
    const fresh = sites.find((s) => s.id === siteId);
    if (fresh) {
      form.update(fresh);
    }
  });
}

void start();
