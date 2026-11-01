import '../styles/pages.css';

import {
  addCamera,
  APPROACH_LABELS,
  type Camera,
  cameraAction,
  cameraState,
  deleteCamera,
  deleteSite,
  getSite,
  type Inputs,
  listInputs,
  type Site,
  siteAction,
  siteFigures,
  type SitePatch,
  siteState,
  SOURCE_LABELS,
  sourceKind,
  updateCamera,
  updateSite,
} from '../net/manager';
import { CameraFields, intersectionDiagram } from '../ui/cameraEditor';
import { type Child, fill, h, param } from '../ui/dom';
import { confirmDialog, toast, withBusy } from '../ui/feedback';
import { formatDuration } from '../ui/format';
import { icon, type IconName } from '../ui/icons';
import { latLonInputs } from '../ui/latLon';
import { links, mountShell } from '../ui/shell';

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

function statusLine(ok: boolean, name: IconName, title: string, text: string, ...actions: Child[]) {
  return h(
    'div',
    { class: `status-line ${ok ? 'ok' : 'warn'}` },
    icon(name),
    h('div', {}, h('strong', {}, title), h('span', {}, text)),
    actions,
  );
}

/** One camera of the site: live state and actions, and its settings in a collapsible form. */
class CameraPanel {
  readonly root = h('article', { class: 'camera-panel' });
  private readonly header = h('header', { class: 'camera-panel-head' });
  private readonly body = h('div', { class: 'camera-panel-body', hidden: true });
  private readonly saveBar = h('div', { class: 'row camera-save', hidden: true });
  private fields: CameraFields | null = null;
  private initial = '';

  constructor(
    private site: Site,
    private camera: Camera,
    private readonly index: number,
    private readonly inputs: Inputs | null,
    private readonly onSite: (site: Site) => void,
  ) {
    this.root.append(this.header, this.body);
    this.renderHeader();
  }

  get dirty(): boolean {
    return !this.saveBar.hidden;
  }

  update(site: Site, camera: Camera): void {
    this.site = site;
    this.camera = camera;
    this.renderHeader();
  }

  private open(): void {
    if (!this.fields) {
      this.renderBody();
    }
    this.body.hidden = !this.body.hidden;
    this.root.classList.toggle('open', !this.body.hidden);
  }

  private action(
    label: string,
    name: IconName,
    act: 'start' | 'stop' | 'restart',
    cls = 'btn small',
  ) {
    const text = cls.includes('icon-only') ? null : label;
    const button = h('button', { type: 'button', class: cls, title: label }, icon(name), text);
    button.addEventListener('click', (e) => {
      e.stopPropagation();
      void withBusy(
        button,
        async () => this.onSite(await cameraAction(this.site.id, this.camera.id, act)),
        `${label}: ${this.camera.name}`,
      );
    });
    return button;
  }

  private renderHeader(): void {
    const { site, camera } = this;
    const r = camera.runtime;
    const state = cameraState(site, camera);
    const kind = sourceKind(camera.source);
    const on = camera.enabled && site.enabled;
    const running = r.state === 'running';
    const detail = running
      ? `up ${formatDuration(r.uptime_s)} · ${r.status?.fps?.toFixed(1) ?? '–'} fps · ${r.status?.objects ?? 0} road users`
      : `${SOURCE_LABELS[kind]} · ${camera.source.input || 'no source set'}`;
    fill(
      this.header,
      h('span', { class: 'camera-index' }, String(this.index + 1)),
      h(
        'button',
        { type: 'button', class: 'camera-title', onclick: () => this.open() },
        h(
          'div',
          {},
          h('strong', {}, camera.name),
          h('span', {}, `${APPROACH_LABELS[camera.approach]} · ${detail}`),
        ),
      ),
      h('span', { class: `pill ${state.tone}${running ? ' live' : ''}` }, state.label),
      camera.calibrated
        ? h('span', { class: 'tag ok' }, icon('check'), 'Calibrated')
        : h('span', { class: 'tag warn' }, icon('calibration'), 'Not calibrated'),
      h('span', { class: 'spacer' }),
      h(
        'a',
        {
          class: `btn small${camera.calibrated ? '' : ' primary'}`,
          href: links.calibration(site.id, camera.id),
        },
        icon('calibration'),
        camera.calibrated ? 'Calibration' : 'Calibrate',
      ),
      on || r.state !== 'stopped'
        ? this.action('Stop', 'stop', 'stop')
        : this.action('Start', 'power', 'start', 'btn small primary'),
      on ? this.action('Restart', 'restart', 'restart', 'btn small ghost icon-only') : null,
      h(
        'a',
        {
          class: 'btn small ghost icon-only',
          title: 'Logs',
          href: links.logs(`${site.id}/${camera.id}`),
        },
        icon('logs'),
      ),
      h(
        'button',
        {
          type: 'button',
          class: 'btn small ghost icon-only chevron',
          title: 'Settings',
          onclick: () => this.open(),
        },
        icon('settings'),
      ),
    );
    if (r.error) {
      this.header.append(
        h('div', { class: 'banner danger camera-error' }, icon('alert'), h('span', {}, r.error)),
      );
    }
  }

  private renderBody(): void {
    const { site, camera } = this;
    const changed = (): void => {
      this.saveBar.hidden = JSON.stringify(this.fields?.value()) === this.initial;
    };
    const remove = h(
      'button',
      { type: 'button', class: 'btn small danger' },
      icon('trash'),
      'Remove camera',
    );
    remove.disabled = site.layout === 'single';
    remove.title =
      site.layout === 'single' ? 'A single-camera site: delete the site instead' : 'Remove camera';
    remove.addEventListener('click', async () => {
      const ok = await confirmDialog({
        title: `Remove ${camera.name}?`,
        body: 'The camera stops and its folder (calibration) moves to the trash of the console on the server.',
        confirm: 'Remove camera',
        danger: true,
      });
      if (ok) {
        await withBusy(
          remove,
          async () => this.onSite(await deleteCamera(site.id, camera.id)),
          `${camera.name} removed`,
        );
      }
    });
    this.fields = new CameraFields(
      {
        name: camera.name,
        approach: camera.approach,
        image_size_px: camera.image_size_px ?? [1920, 1080],
        source: camera.source,
        playback: camera.playback,
        video: camera.video,
      },
      this.inputs,
      changed,
      { streaming: true },
    );
    this.initial = JSON.stringify(this.fields.value());
    const save = h(
      'button',
      { type: 'button', class: 'btn primary small' },
      icon('check'),
      'Save camera',
    );
    save.addEventListener('click', () => void this.save(save));
    fill(
      this.saveBar,
      icon('info'),
      h(
        'span',
        {},
        camera.runtime.state === 'running'
          ? 'Unsaved changes · the camera restarts when you save.'
          : 'Unsaved changes',
      ),
      h('span', { class: 'spacer' }),
      h(
        'button',
        {
          type: 'button',
          class: 'btn ghost small',
          onclick: () => {
            this.fields = null;
            this.saveBar.hidden = true;
            this.renderBody();
          },
        },
        'Discard',
      ),
      save,
    );
    fill(
      this.body,
      this.fields.root,
      h(
        'dl',
        { class: 'kv' },
        h('dt', {}, 'Folder'),
        h(
          'dd',
          { class: 'mono' },
          site.layout === 'single' ? `scenes/${site.id}` : `scenes/${site.id}/cameras/${camera.id}`,
        ),
        h('dt', {}, 'Process'),
        h(
          'dd',
          { class: 'mono' },
          camera.runtime.pid ? `pid ${camera.runtime.pid} · port ${camera.runtime.port}` : '–',
        ),
        h('dt', {}, 'Restarts'),
        h(
          'dd',
          {},
          `${camera.runtime.restarts}${camera.runtime.last_exit !== null ? ` · last exit code ${camera.runtime.last_exit}` : ''}`,
        ),
      ),
      this.saveBar,
      h('div', { class: 'row' }, h('span', { class: 'spacer' }), remove),
    );
  }

  private async save(button: HTMLButtonElement): Promise<void> {
    const fields = this.fields;
    if (!fields) {
      return;
    }
    const problem = fields.problem();
    if (problem) {
      toast(problem, 'error');
      return;
    }
    const running = ['running', 'starting'].includes(this.camera.runtime.state);
    await withBusy(
      button,
      async () => {
        const site = await updateCamera(this.site.id, this.camera.id, fields.value());
        this.initial = JSON.stringify(fields.value());
        this.saveBar.hidden = true;
        this.onSite(site);
      },
      running ? 'Camera saved; it restarts with the new settings.' : 'Camera saved.',
    );
  }
}

/** Form for a new camera of the site. */
function newCameraForm(
  site: Site,
  inputs: Inputs | null,
  onDone: (site: Site | null) => void,
): HTMLElement {
  const next = site.cameras.length + 1;
  const fields = new CameraFields(
    {
      name: `Camera ${next}`,
      approach: 'centre',
      image_size_px: [1920, 1080],
      source: { mode: 'live', input: '' },
    },
    inputs,
    () => undefined,
  );
  const add = h('button', { type: 'button', class: 'btn primary' }, icon('plus'), 'Add camera');
  add.addEventListener('click', () => {
    const problem = fields.problem();
    if (problem) {
      toast(problem, 'error');
      return;
    }
    void withBusy(
      add,
      async () => onDone(await addCamera(site.id, fields.value())),
      `${fields.value().name} added`,
    );
  });
  return h(
    'article',
    { class: 'camera-panel open new' },
    h(
      'header',
      { class: 'camera-panel-head' },
      h('span', { class: 'camera-index' }, String(next)),
      h(
        'div',
        { class: 'camera-title' },
        h(
          'div',
          {},
          h('strong', {}, 'New camera'),
          h('span', {}, 'Source, image size and the arm it watches'),
        ),
      ),
    ),
    h(
      'div',
      { class: 'camera-panel-body' },
      fields.root,
      h(
        'div',
        { class: 'row' },
        h('span', { class: 'spacer' }),
        h('button', { type: 'button', class: 'btn ghost', onclick: () => onDone(null) }, 'Cancel'),
        add,
      ),
    ),
  );
}

class SitePage {
  private initial = '';
  private readonly saveBar = h('div', { class: 'save-bar', hidden: true });
  private readonly name = h('input', { class: 'input', maxlength: 80 });
  private readonly description = h('textarea', { class: 'input', rows: 2, maxlength: 400 });
  private readonly lat: HTMLInputElement;
  private readonly lon: HTMLInputElement;
  private readonly alt: HTMLInputElement;
  private readonly radius: HTMLInputElement;
  private readonly status = h('div', { class: 'stack' });
  private readonly mapStatus = h('div', {});
  private readonly cameraList = h('div', { class: 'camera-list' });
  private readonly plan = h('div', { class: 'junction-plan-box' });
  private readonly cameraTools = h('div', { class: 'row' });
  private panels = new Map<string, CameraPanel>();
  private adding: HTMLElement | null = null;

  constructor(
    private site: Site,
    private readonly inputs: Inputs | null,
    private readonly root: HTMLElement,
  ) {
    const changed = () => this.updateDirty();
    this.name.value = site.name;
    this.description.value = site.description;
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
    latLonInputs(this.lat, this.lon);
    this.alt = numberInput(site.geo_anchor?.alt_m ?? 0, { step: 0.1 });
    this.radius = numberInput(site.map_radius_m ?? 200, { step: 10, min: 50, max: 1000 });
    for (const input of [this.name, this.description, this.lat, this.lon, this.alt, this.radius]) {
      input.addEventListener('input', changed);
    }
    this.render();
    this.initial = JSON.stringify(this.patch());
  }

  update(site: Site): void {
    this.site = site;
    this.renderStatus();
    this.renderMap();
    this.renderCameras();
  }

  private patch(): SitePatch {
    const lat = Number.parseFloat(this.lat.value);
    const lon = Number.parseFloat(this.lon.value);
    return {
      name: this.name.value.trim(),
      description: this.description.value.trim(),
      geo_anchor:
        Number.isFinite(lat) && Number.isFinite(lon)
          ? { lat_deg: lat, lon_deg: lon, alt_m: Number.parseFloat(this.alt.value) || 0 }
          : this.site.geo_anchor,
      map_radius_m: Number.parseFloat(this.radius.value) || 200,
    };
  }

  private updateDirty(): void {
    this.saveBar.hidden = JSON.stringify(this.patch()) === this.initial;
  }

  private async save(button: HTMLButtonElement): Promise<void> {
    if (!this.name.value.trim()) {
      toast('The site needs a name.', 'error');
      return;
    }
    const before = this.site.geo_anchor;
    const patch = this.patch();
    const moved =
      this.site.georeferenced &&
      before !== null &&
      patch.geo_anchor !== null &&
      patch.geo_anchor !== undefined &&
      (before.lat_deg !== patch.geo_anchor.lat_deg || before.lon_deg !== patch.geo_anchor.lon_deg);
    if (moved) {
      const ok = await confirmDialog({
        title: 'Move the intersection centre?',
        body: 'The centre is the origin of the world frame of all cameras. Their calibrations keep the control points, but each camera must be saved again in its calibration (open it and press Save) and the map layers prepared again.',
        confirm: 'Move the centre',
      });
      if (!ok) {
        return;
      }
    }
    await withBusy(
      button,
      async () => {
        this.update(await updateSite(this.site.id, patch));
        this.initial = JSON.stringify(this.patch());
        this.updateDirty();
      },
      'Site saved.',
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
      h('span', {}, 'Unsaved changes to the site'),
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
        body: 'All its cameras stop and the site folder (scene, cameras, calibrations, zones) moves to the trash of the console (.cache/manager/trash on the server).',
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
            ['cameras', 'Cameras'],
            ['general', 'General'],
            ['location', 'Location and map'],
            ['danger', 'Danger zone'],
          ].map(([id, label]) => h('a', { href: `#${id}` }, label)),
        ),
        h(
          'div',
          { class: 'stack' },
          section(
            'runtime',
            'activity',
            'Status',
            'The perception servers of all cameras',
            this.status,
          ),
          section(
            'cameras',
            'video',
            'Cameras',
            'Up to four cameras, each with its own source and calibration',
            h(
              'div',
              { class: 'cameras-layout' },
              h(
                'div',
                { class: 'stack', style: 'min-width: 0' },
                this.cameraList,
                this.cameraTools,
              ),
              this.plan,
            ),
          ),
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
            'location',
            'pin',
            'Location and map',
            site.georeferenced
              ? 'Centre of the intersection: the origin shared by all its cameras'
              : 'Position of the scene, to place its map',
            h(
              'div',
              { class: 'form-grid' },
              h('label', { class: 'field' }, h('span', {}, 'Latitude (°)'), this.lat),
              h('label', { class: 'field' }, h('span', {}, 'Longitude (°)'), this.lon),
              h('label', { class: 'field' }, h('span', {}, 'Altitude (m)'), this.alt),
              h(
                'label',
                { class: 'field' },
                h('span', {}, 'Map radius (m)'),
                this.radius,
                h('small', {}, 'Area around the centre the map layers cover'),
              ),
            ),
            h(
              'p',
              { class: 'help' },
              site.georeferenced
                ? 'Paste "latitude, longitude" of the intersection centre (e.g. from Google Maps) into either field. Map layers: aerial imagery (Google with an API key, else the Bavarian orthophoto), LoD2 buildings, roads and crossings from OpenStreetMap.'
                : 'This scene uses a sensor frame (benchmark), so the location only places the map: enter the position of the camera mast, prepare the map, then fine-align it in the live twin with "Align map".',
            ),
            this.mapStatus,
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
                'Stops all cameras and removes the site from the console. Its files stay in the trash folder on the server and can be restored by hand.',
              ),
              remove,
            ),
          ),
          this.saveBar,
        ),
      ),
    );
    this.renderStatus();
    this.renderMap();
    this.renderCameras();
  }

  private renderStatus(): void {
    const site = this.site;
    const state = siteState(site);
    const figures = siteFigures(site);
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
    const live = figures.running > 0;
    fill(
      this.status,
      h(
        'div',
        { class: 'row' },
        h('span', { class: `pill ${state.tone}${live ? ' live' : ''}` }, state.label),
        h(
          'span',
          { class: 'muted' },
          live
            ? `${figures.running} of ${site.cameras.length} cameras · ${figures.fps?.toFixed(1) ?? '–'} fps · ${figures.objects} road users`
            : site.enabled
              ? 'active, not running'
              : 'inactive: not started with the backend',
        ),
        h('span', { class: 'spacer' }),
        site.enabled
          ? action('Stop all', 'stop', 'stop')
          : action('Activate', 'power', 'start', 'btn small primary'),
        site.enabled ? action('Restart all', 'restart', 'restart') : null,
        h('a', { class: 'btn small ghost', href: links.logs(site.id) }, icon('logs'), 'Logs'),
      ),
      site.runtime.error
        ? h('div', { class: 'banner danger' }, icon('alert'), h('span', {}, site.runtime.error))
        : null,
    );
  }

  private renderCameras(): void {
    const site = this.site;
    const ids = site.cameras.map((c) => c.id);
    const same = ids.length === this.panels.size && ids.every((id) => this.panels.has(id));
    if (same) {
      site.cameras.forEach((c) => this.panels.get(c.id)?.update(site, c));
    } else {
      // Cameras were added or removed: rebuild the list (open forms of kept cameras are lost
      // only when the set changes).
      this.panels = new Map(
        site.cameras.map((c, i) => [
          c.id,
          new CameraPanel(site, c, i, this.inputs, (fresh) => this.update(fresh)),
        ]),
      );
      fill(
        this.cameraList,
        site.cameras.length === 0
          ? h(
              'div',
              { class: 'empty small' },
              icon('video'),
              h('h3', {}, 'No camera yet'),
              h('p', {}, 'Add the first camera of the intersection.'),
            )
          : [...this.panels.values()].map((p) => p.root),
      );
      if (this.adding) {
        this.cameraList.append(this.adding);
      }
    }
    fill(
      this.plan,
      intersectionDiagram(site.cameras),
      h('small', {}, 'Cameras on the arms (north up)'),
    );
    const full = site.cameras.length >= site.max_cameras;
    const add = h('button', { type: 'button', class: 'btn' }, icon('plus'), 'Add camera');
    add.disabled = full || this.adding !== null;
    add.addEventListener('click', () => {
      this.adding = newCameraForm(site, this.inputs, (fresh) => {
        this.adding?.remove();
        this.adding = null;
        if (fresh) {
          this.update(fresh);
        } else {
          this.renderCameras();
        }
      });
      this.cameraList.append(this.adding);
      add.disabled = true;
      this.adding.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
    });
    fill(
      this.cameraTools,
      add,
      h(
        'span',
        { class: 'muted' },
        full
          ? site.max_cameras === 1
            ? 'Benchmark scenes have one camera.'
            : `An intersection has at most ${site.max_cameras} cameras.`
          : site.layout === 'single' && site.cameras.length > 0
            ? 'Adding a camera turns this single-camera scene into an intersection; its camera keeps its calibration.'
            : `${site.cameras.length} of ${site.max_cameras} cameras`,
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
        `Imagery, OpenStreetMap and buildings within ${site.map_radius_m ?? 200} m.`,
        prepare,
      );
    } else {
      line = statusLine(
        false,
        'map',
        'No map layers yet',
        site.geo_anchor
          ? 'Download them for the area around the centre.'
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
      h(
        'p',
        {},
        'Intersection settings: its cameras, location and map, and the perception servers.',
      ),
    ),
  );
  const body = h('div', {});
  root.append(head, body);
  const page = new SitePage(site, inputs, body);
  shell.onSites((sites) => {
    const fresh = sites.find((s) => s.id === siteId);
    if (fresh) {
      page.update(fresh);
    }
  });
  if (location.hash) {
    document.querySelector(location.hash)?.scrollIntoView();
  }
}

void start();
