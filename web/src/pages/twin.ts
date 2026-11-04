import '../styles/twin.css';

import { BevMode } from '../bev/bevMode';
import { loadMapData, type MapData } from '../map/mapData';
import { attributionLines } from '../map/mapLayers';
import { fetchRegions, type Regions, saveRegions } from '../net/calibrationApi';
import {
  cameraBase,
  fetchScene,
  fetchStatus,
  type ServerStatus,
  setSite,
  subscribeVideo,
  subscribeWorld,
  type Unsubscribe,
} from '../net/client';
import {
  type Camera,
  type GeoAnchor,
  listSites,
  type Site,
  siteAction,
  sitePlayback,
  updateSite,
} from '../net/manager';
import type { WorldState } from '../net/schema';
import type { ViewMode } from '../twin/orbitRig';
import { CLASS_LABELS } from '../twin/palette';
import {
  type BaseLayer,
  type DrawnObject,
  geoOffset,
  type MapTransform,
  type SiteSpec,
  TwinView,
} from '../twin/twinView';
import { element, fill, h, param, segmented, toggle } from '../ui/dom';
import type { TaggedEvent } from '../ui/events';
import { toast, withBusy } from '../ui/feedback';
import { icon } from '../ui/icons';
import { makeResizable } from '../ui/resizer';
import { links, mountShell, type Shell } from '../ui/shell';
import { type PanelCamera, SidePanel } from '../ui/sidePanel';
import { currentTheme, onThemeChange } from '../ui/theme';

const STATUS_INTERVAL_MS = 1000;
const PANEL_INTERVAL_MS = 250;
// Status requests that may fail in a row before the shown camera counts as unreachable.
const MISSED_STATUS_LIMIT = 3;
const ALL = '';
const GRID_KEY = 'jp-twin-camera-grid';
const MAX_GRID_CAMERAS = 4;

function readFlag(key: string): boolean {
  try {
    return localStorage.getItem(key) === '1';
  } catch {
    return false;
  }
}

function writeFlag(key: string, on: boolean): void {
  try {
    localStorage.setItem(key, on ? '1' : '0');
  } catch {
    // Not remembered.
  }
}

function layout(): HTMLElement {
  return h(
    'div',
    { class: 'twin-layout' },
    h(
      'section',
      { class: 'twin-stage' },
      h('canvas', { id: 'twin' }),
      h(
        'div',
        { class: 'hud hud-tl' },
        h('div', { id: 'site-picker', class: 'glass site-picker' }),
        h('div', { id: 'live-chip', class: 'glass chip' }),
      ),
      h('div', { id: 'view-controls', class: 'hud hud-tr glass toolbar' }),
      h('div', { id: 'align-bar', class: 'align-bar glass', hidden: true }),
      h('div', { id: 'signal-bar', class: 'signal-bar glass', hidden: true }),
      h('div', { id: 'legend', class: 'hud hud-bl glass legend' }),
      h('div', { id: 'tooltip', class: 'tooltip glass', hidden: true }),
      h('p', { id: 'attribution', class: 'attribution' }),
      h(
        'footer',
        { class: 'playback glass' },
        h('button', { id: 'play', type: 'button', class: 'btn primary icon-only', title: 'Pause' }),
        h('input', { id: 'seek', type: 'range', min: 0, max: 0, value: 0, 'aria-label': 'Seek' }),
        h('span', { id: 'frame', class: 'frame num' }),
      ),
      h(
        'div',
        { id: 'offline', class: 'stage-notice', hidden: true },
        h('span', { class: 'spinner' }),
        'Connection to the camera lost, reconnecting…',
      ),
      h('div', { id: 'stage-empty', class: 'stage-empty glass', hidden: true }),
    ),
    h('div', {
      id: 'panel-resizer',
      class: 'panel-resizer',
      title: 'Drag to make the camera image larger or smaller, double click for the default',
    }),
    h(
      'aside',
      { class: 'twin-panel' },
      h(
        'section',
        { class: 'panel-card camera' },
        h(
          'header',
          {},
          icon('video'),
          h('h3', { id: 'camera-title' }, 'Camera'),
          h('nav', { id: 'camera-tabs', class: 'camera-tabs', hidden: true }),
          h('span', { id: 'video-frame', class: 'frame-tag num' }),
        ),
        h(
          'div',
          { class: 'camera-frame' },
          h('img', { id: 'video', alt: 'Camera image with tracked road users', hidden: true }),
          h('div', { id: 'video-grid', class: 'video-grid', hidden: true }),
          h(
            'button',
            {
              id: 'video-expand',
              type: 'button',
              class: 'camera-expand',
              title: 'Enlarge (Esc to close)',
              'aria-label': 'Enlarge the camera image',
            },
            icon('expand'),
          ),
          h(
            'div',
            { id: 'video-empty', class: 'camera-empty' },
            h('span', { class: 'spinner' }),
            'Waiting for the camera…',
          ),
        ),
      ),
      h('section', { id: 'stats', class: 'stats' }),
      h(
        'section',
        { id: 'signals-card', class: 'panel-card signals-card', hidden: true },
        h('header', {}, icon('pulse'), h('h3', {}, 'Traffic lights')),
        h('ul', { id: 'signals', class: 'signals' }),
      ),
      h(
        'section',
        { class: 'panel-card events-card' },
        h(
          'header',
          {},
          icon('alert'),
          h('h3', {}, 'Safety events'),
          h('span', { id: 'event-count', class: 'pill danger plain', hidden: true }),
        ),
        h('ul', { id: 'events', class: 'events' }),
      ),
    ),
  );
}

/** Live data of the twin page: which sites and cameras are drawn, focused and streamed. */
class TwinPage {
  private readonly panel = new SidePanel();
  private readonly world = new Map<string, Unsubscribe>();
  private readonly loading = new Set<string>();
  private readonly placed = new Set<string>();
  private sites: Site[] = [];
  private origin: GeoAnchor | null = null;
  private focus: string;
  private camera: string | null;
  private video: { key: string; close: Unsubscribe[] } | null = null;
  // All cameras side by side in the panel (remembered in the browser).
  private grid = readFlag(GRID_KEY);
  private status: ServerStatus | null = null;
  private missed = 0;
  private framed = false;
  private hasAerial = false;
  private readonly attributions = new Map<string, { text: string; url: string }[]>();
  // Regions (with the traffic lights) of each followed camera, by "site/camera".
  private readonly regions = new Map<string, { base: string; regions: Regions }>();

  constructor(
    private readonly view: TwinView,
    private readonly bev: BevMode,
    private readonly shell: Shell,
    focus: string | null,
    camera: string | null,
  ) {
    this.focus = focus ?? ALL;
    this.camera = camera;
    this.panel.onCamera = (id) => this.selectCamera(id);
    this.panel.onGrid = (on) => {
      this.grid = on;
      writeFlag(GRID_KEY, on);
      this.renderCameras();
    };
    // The cameras of a site play together: play, pause and seek go to the site.
    this.panel.onPlayback = async (command) => {
      const [siteId, cameraId] = (this.camera ?? '').split('/');
      if (siteId && cameraId) {
        await sitePlayback(siteId, command, cameraId).catch((error: unknown) =>
          toast((error as Error).message, 'error'),
        );
      }
    };
    view.onHover = (object, x, y) => this.tooltip(object, x, y);
    bev.view.onHover = (object, x, y) => this.tooltip(object, x, y);
    view.onSignalPlaced = (site, camera, id, position, heading) =>
      this.placeSignal(site, camera, id, position, heading);
    setInterval(() => this.renderPanel(), PANEL_INTERVAL_MS);
    setInterval(() => void this.refreshStatus(), STATUS_INTERVAL_MS);
  }

  /** Takes a new site list: places new sites, follows cameras that start or stop. */
  sync(sites: Site[]): void {
    this.sites = sites;
    if (!this.origin) {
      const focused = sites.find((s) => s.id === this.focus);
      this.origin = focused?.geo_anchor ?? sites.find((s) => s.geo_anchor)?.geo_anchor ?? null;
    }
    for (const site of sites) {
      this.place(site);
      if (this.placed.has(site.id) && !site.georeferenced && site.map_transform) {
        this.bev.view.setMapTransform(site.id, site.map_transform);
      }
      for (const camera of site.cameras) {
        this.follow(site, camera);
      }
    }
    this.renderPicker();
    this.renderCameras();
    this.renderControls();
    this.renderEmpty();
  }

  private spec(site: Site): SiteSpec | null {
    const offset = site.geo_anchor && this.origin ? geoOffset(this.origin, site.geo_anchor) : null;
    // Sites without a location can only be shown alone, at the origin.
    if (!offset && site.id !== this.focus) {
      return null;
    }
    return {
      id: site.id,
      name: site.name,
      offset: offset ?? { x: 0, y: 0 },
      transform: site.georeferenced ? null : (site.map_transform ?? { yaw_deg: 0, x_m: 0, y_m: 0 }),
    };
  }

  private place(site: Site): void {
    if (this.placed.has(site.id)) {
      return;
    }
    const spec = this.spec(site);
    if (!spec) {
      return;
    }
    this.placed.add(site.id);
    this.bev.view.addSite(spec);
    this.view.addSite(spec);
    if (spec.transform) {
      this.view.setMapTransform(site.id, spec.transform);
      this.bev.view.setMapTransform(site.id, spec.transform);
    }
    if (site.has_map) {
      void loadMapData(`/sites/${encodeURIComponent(site.id)}/map/map.json`)
        .then((map: MapData) => {
          this.view.setMap(site.id, map);
          this.bev.view.setMap(site.id, map);
          if (map.ortho) {
            this.hasAerial = true;
          }
          this.bev.renderOptions(this.hasAerial);
          this.attributions.set(site.id, attributionLines(map));
          this.renderAttribution();
        })
        .catch((error: unknown) =>
          toast(`Map of ${site.name}: ${(error as Error).message}`, 'error'),
        );
    }
    if (site.id === this.focus && site.cameras.every((c) => c.runtime.state !== 'running')) {
      // Nothing to wait for: look at the site's origin.
      this.frame();
    }
  }

  /** World stream of every running camera of a placed site. */
  private follow(site: Site, camera: Camera): void {
    const key = `${site.id}/${camera.id}`;
    const running = camera.runtime.state === 'running';
    if (!running) {
      this.world.get(key)?.();
      this.world.delete(key);
      return;
    }
    if (!this.placed.has(site.id) || this.world.has(key) || this.loading.has(key)) {
      return;
    }
    this.loading.add(key);
    const base = cameraBase(site.id, camera.id);
    void fetchScene(base)
      .then((description) => {
        this.view.addCamera(site.id, camera.id, description);
        this.bev.view.addCamera(site.id, camera.id, description);
        // Traffic lights of the camera: where they stand, and their colour from its stream.
        void fetchRegions(base)
          .then((regions) => {
            this.regions.set(key, { base, regions });
            this.view.setSignals(site.id, camera.id, regions.signals);
            this.bev.view.setSignals(site.id, camera.id, regions.signals);
            this.bev.view.setLines(site.id, camera.id, regions.lines);
            this.renderSignalBar(site.id);
          })
          .catch(() => undefined);
        this.world.set(
          key,
          subscribeWorld(
            (state) => {
              this.view.push(site.id, camera.id, state);
              this.bev.view.push(site.id, camera.id, state);
            },
            () => undefined,
            base,
          ),
        );
        if (site.id === this.focus || this.focus === ALL) {
          this.frame();
        }
      })
      .catch(() => undefined)
      .finally(() => this.loading.delete(key));
  }

  /** Initial view, once the focused site's cameras are known. */
  private frame(): void {
    if (this.framed && this.focus !== ALL) {
      return;
    }
    if (this.focus === ALL) {
      this.view.overview();
      this.bev.view.overview();
    } else {
      this.view.focus(this.focus);
      this.bev.view.focus(this.focus);
    }
    this.bev.view.setHome(this.focus === ALL ? null : this.focus);
    this.framed = true;
  }

  /** Site the BEV card reports on: the focused one, else none (the BEV takes the central one). */
  panelSite(): string | null {
    return this.focus === ALL ? null : this.focus;
  }

  siteName(siteId: string): string {
    return this.sites.find((s) => s.id === siteId)?.name ?? siteId;
  }

  cameraName(siteId: string, cameraId: string): string {
    return (
      this.sites.find((s) => s.id === siteId)?.cameras.find((c) => c.id === cameraId)?.name ??
      cameraId
    );
  }

  private focusedSite(): Site | undefined {
    return this.sites.find((s) => s.id === this.focus);
  }

  /** Cameras the panel offers: those of the focused site, or of all sites. */
  private panelCameras(): (PanelCamera & { site: Site })[] {
    const sites = this.focus === ALL ? this.sites : this.sites.filter((s) => s.id === this.focus);
    return sites.flatMap((site) =>
      site.cameras.map((c) => ({
        id: `${site.id}/${c.id}`,
        name: this.focus === ALL ? `${site.name} · ${c.name}` : c.name,
        base: cameraBase(site.id, c.id),
        running: c.runtime.state === 'running',
        site,
      })),
    );
  }

  private selectCamera(key: string): void {
    this.camera = key;
    this.status = null;
    this.missed = 0;
    this.renderCameras();
    void this.refreshStatus();
  }

  private renderCameras(): void {
    const cameras = this.panelCameras();
    const wanted =
      this.camera && this.focus !== ALL && !this.camera.includes('/')
        ? `${this.focus}/${this.camera}`
        : this.camera;
    const selected =
      cameras.find((c) => c.id === wanted) ?? cameras.find((c) => c.running) ?? cameras[0] ?? null;
    this.camera = selected?.id ?? null;
    // The grid shows every running camera of the view (at most four); otherwise only the selected
    // one is streamed: video streams cost encoding on the server.
    const running = cameras.filter((c) => c.running).slice(0, MAX_GRID_CAMERAS);
    const grid = this.grid && running.length > 1;
    this.panel.setCameras(cameras, this.camera, grid);
    const streamed = grid ? running : selected?.running ? [selected] : [];
    const key = `${grid}:${streamed.map((c) => c.id).join(',')}`;
    if (this.video?.key !== key) {
      for (const close of this.video?.close ?? []) {
        close();
      }
      this.panel.resetVideo(grid ? streamed : []);
      this.video = {
        key,
        close: streamed.map((c) =>
          subscribeVideo(
            (frame) =>
              grid ? this.panel.showGridVideo(c.id, c.name, frame) : this.panel.showVideo(frame),
            () => undefined,
            c.base,
          ),
        ),
      };
    }
  }

  private async refreshStatus(): Promise<void> {
    const camera = this.panelCameras().find((c) => c.id === this.camera);
    if (!camera?.running) {
      this.status = null;
    } else {
      try {
        this.status = await fetchStatus(camera.base);
        this.missed = 0;
      } catch {
        if (++this.missed >= MISSED_STATUS_LIMIT) {
          this.status = null;
        }
      }
    }
    const cameras = this.panelCameras();
    this.panel.showStatus(this.status, cameras.filter((c) => c.running).length, cameras.length);
  }

  private renderPanel(): void {
    const siteId = this.focus === ALL ? undefined : this.focus;
    const names = new Map(
      this.sites.flatMap((s) =>
        s.cameras.map((c): [string, string] => [
          `${s.id}/${c.id}`,
          siteId ? c.name : `${s.name} · ${c.name}`,
        ]),
      ),
    );
    const states = this.view.states(siteId);
    const several = states.length > 1;
    const events: TaggedEvent[] = states.flatMap(({ site, camera, state }) =>
      state.events.map((event) => ({
        key: `${site}/${camera}:${event.id}`,
        label: several ? (names.get(`${site}/${camera}`) ?? camera) : '',
        event,
      })),
    );
    const objects =
      this.bev.current() === 'bev' ? this.bev.view.objects(siteId) : this.view.objects(siteId);
    this.panel.showWorld(objects, events);
    this.bev.render();
    this.panel.showSignals(
      states.flatMap(({ site, camera, state }) =>
        (state.signals ?? []).map((s) => ({
          name: several ? `${s.name} · ${names.get(`${site}/${camera}`) ?? camera}` : s.name,
          state: s.state,
          seconds: Math.max(0, (state.timestamp_ns - s.since_ns) / 1e9),
        })),
      ),
    );
    const shown = states.find(({ site, camera }) => `${site}/${camera}` === this.camera);
    if (shown) {
      this.showFrame(shown.state);
    }
  }

  private showFrame(state: WorldState): void {
    this.panel.showFrame(state.frame, state.timestamp_ns);
  }

  private renderPicker(): void {
    const picker = document.querySelector<HTMLElement>('#site-picker');
    if (!picker) {
      return;
    }
    const options = [
      [ALL, 'All intersections'],
      ...this.sites
        .filter((s) => this.placed.has(s.id) || s.id === this.focus)
        .map((s) => [s.id, s.name]),
    ];
    const key = options.map((o) => o.join('=')).join('|') + `>${this.focus}`;
    if (picker.dataset['key'] === key) {
      return;
    }
    picker.dataset['key'] = key;
    const select = h('select', { class: 'input', 'aria-label': 'Intersection' });
    for (const [value, label] of options) {
      select.append(h('option', { value, selected: value === this.focus }, label));
    }
    select.addEventListener('change', () => this.setFocus(select.value));
    const hidden = this.sites.filter((s) => !this.placed.has(s.id) && s.id !== this.focus);
    fill(
      picker,
      icon('globe'),
      select,
      hidden.length > 0
        ? h(
            'span',
            { class: 'muted', title: hidden.map((s) => s.name).join(', ') },
            `${hidden.length} without location`,
          )
        : null,
    );
  }

  private setFocus(siteId: string): void {
    this.focus = siteId;
    this.camera = null;
    this.framed = false;
    history.replaceState(null, '', siteId ? links.twin(siteId) : links.twin());
    const site = this.focusedSite();
    this.shell.setTitle(site ? site.name : 'Digital twin');
    this.setActions();
    if (site && !this.placed.has(site.id)) {
      // A site without location is shown alone: reload the page around it.
      location.reload();
      return;
    }
    this.frame();
    this.renderPicker();
    this.renderCameras();
    this.renderControls();
    this.renderEmpty();
    void this.refreshStatus();
  }

  setActions(): void {
    const site = this.focusedSite();
    this.shell.setActions(
      ...(site
        ? [
            h(
              'a',
              { class: 'btn small', href: links.calibration(site.id) },
              icon('calibration'),
              'Calibration',
            ),
            h(
              'a',
              { class: 'btn small', href: links.settings(site.id) },
              icon('settings'),
              'Settings',
            ),
          ]
        : [h('a', { class: 'btn small', href: links.newSite }, icon('plus'), 'New site')]),
    );
  }

  /** Notice on the stage while the focused site has no running camera. */
  private renderEmpty(): void {
    const notice = document.querySelector<HTMLElement>('#stage-empty');
    if (!notice) {
      return;
    }
    const sites = this.focus === ALL ? this.sites : this.sites.filter((s) => s.id === this.focus);
    const running = sites.some((s) => s.cameras.some((c) => c.runtime.state === 'running'));
    const starting = sites.some((s) =>
      s.cameras.some((c) => ['starting', 'backoff'].includes(c.runtime.state)),
    );
    notice.hidden = running;
    if (running) {
      return;
    }
    const site = this.focusedSite();
    const key = `${this.focus}:${starting}:${site?.enabled}`;
    if (notice.dataset['key'] === key) {
      return;
    }
    notice.dataset['key'] = key;
    const start = h(
      'button',
      { type: 'button', class: 'btn small primary' },
      icon('power'),
      'Activate',
    );
    start.addEventListener('click', () => {
      if (site) {
        void withBusy(start, () => siteAction(site.id, 'start'), `${site.name} is starting`);
      }
    });
    fill(
      notice,
      starting ? h('span', { class: 'spinner' }) : icon('power'),
      h(
        'span',
        {},
        starting
          ? 'Cameras are starting: the detector loads and the sources open…'
          : site
            ? `No camera of ${site.name} is running. The map is shown; road users appear once a camera runs.`
            : 'No camera is running.',
      ),
      site && !starting && !site.enabled ? start : null,
      site ? h('a', { class: 'btn small ghost', href: links.settings(site.id) }, 'Settings') : null,
    );
  }

  private renderAttribution(): void {
    const element = document.querySelector<HTMLElement>('#attribution');
    const seen = new Map<string, { text: string; url: string }>();
    for (const lines of this.attributions.values()) {
      for (const line of lines) {
        seen.set(line.text, line);
      }
    }
    element?.replaceChildren(
      ...[...seen.values()].map(({ text, url }) =>
        h('a', { href: url, target: '_blank', rel: 'noopener' }, text),
      ),
    );
  }

  private controlsKey = '';

  private renderControls(): void {
    const site = this.focusedSite();
    const anyOrtho = this.sites.some((s) => s.has_map);
    // Sensor-frame scenes (benchmark) place their map by hand.
    const canAlign = Boolean(
      site && !site.georeferenced && site.has_map && this.placed.has(site.id),
    );
    const signals = site
      ? site.cameras.reduce((sum, c) => sum + (c.regions?.signals.length ?? 0), 0)
      : 0;
    const key = `${anyOrtho}:${canAlign}:${site?.id}:${signals}`;
    if (key === this.controlsKey) {
      return;
    }
    this.controlsKey = key;
    setupControls(
      this.view,
      this.bev,
      anyOrtho,
      canAlign && site ? site : null,
      site && signals > 0 ? this.signalButton(site) : null,
    );
  }

  /** "Traffic lights": place each signal of the site in the twin. */
  private signalButton(site: Site): HTMLElement {
    const button = h(
      'button',
      { type: 'button', class: 'btn small' },
      icon('pulse'),
      'Traffic lights',
    );
    button.addEventListener('click', () => {
      const bar = document.querySelector<HTMLElement>('#signal-bar');
      if (bar) {
        bar.hidden = !bar.hidden;
        this.renderSignalBar(site.id);
      }
    });
    return button;
  }

  private renderSignalBar(siteId: string): void {
    const bar = document.querySelector<HTMLElement>('#signal-bar');
    const site = this.sites.find((s) => s.id === siteId);
    if (!bar || bar.hidden || !site) {
      return;
    }
    const rows = site.cameras.flatMap((camera) => {
      const entry = this.regions.get(`${site.id}/${camera.id}`);
      return (entry?.regions.signals ?? []).map((signal) => ({ camera, entry, signal }));
    });
    const missing = site.cameras.filter(
      (c) => c.regions.signals.length > 0 && !this.regions.has(`${site.id}/${c.id}`),
    );
    fill(
      bar,
      h(
        'div',
        { class: 'signal-bar-head' },
        icon('pulse'),
        h('strong', {}, 'Traffic lights'),
        h(
          'span',
          { class: 'muted' },
          'Place: click where the signal stands, then towards the traffic it faces.',
        ),
        h('span', { class: 'spacer' }),
        h(
          'button',
          {
            type: 'button',
            class: 'btn small ghost',
            onclick: () => {
              this.view.setSignalPlacement(null);
              bar.hidden = true;
            },
          },
          'Close',
        ),
      ),
      rows.map(({ camera, entry, signal }) => {
        const height = h('input', {
          class: 'input num',
          type: 'number',
          min: 1,
          max: 20,
          step: 0.1,
          title: 'Height of the signal head above the road (m)',
        });
        height.value = String(signal.height_m);
        height.addEventListener('change', () => {
          const value = Number.parseFloat(height.value);
          if (value > 0 && value <= 20 && entry) {
            signal.height_m = value;
            void this.saveSignals(site.id, camera.id);
          }
        });
        const place = h(
          'button',
          { type: 'button', class: 'btn small primary' },
          icon('pin'),
          signal.position ? 'Move' : 'Place',
        );
        place.addEventListener('click', () => {
          this.view.setSignalPlacement(site.id, camera.id, signal.id);
          toast(`Click where ${signal.name} stands, then towards the traffic it faces.`, 'info');
        });
        return h(
          'div',
          { class: 'signal-row' },
          h('span', { class: `signal-dot ${this.signalState(site.id, camera.id, signal.id)}` }),
          h('span', { class: 'name' }, signal.name),
          site.cameras.length > 1 ? h('span', { class: 'muted' }, camera.name) : null,
          h(
            'span',
            { class: `tag ${signal.position ? 'ok' : 'warn'}` },
            signal.position ? 'placed' : 'not placed',
          ),
          h('label', { class: 'height' }, 'height m', height),
          place,
        );
      }),
      missing.length > 0
        ? h(
            'p',
            { class: 'muted' },
            `Start ${missing.map((c) => c.name).join(', ')} to place its traffic lights.`,
          )
        : null,
    );
  }

  private signalState(siteId: string, cameraId: string, signalId: string): string {
    const state = this.view
      .states(siteId)
      .find((s) => s.camera === cameraId)
      ?.state.signals?.find((s) => s.id === signalId);
    return state?.state ?? 'unknown';
  }

  private async saveSignals(siteId: string, cameraId: string): Promise<void> {
    const entry = this.regions.get(`${siteId}/${cameraId}`);
    if (!entry) {
      return;
    }
    try {
      entry.regions = await saveRegions(entry.regions, entry.base);
      this.view.setSignals(siteId, cameraId, entry.regions.signals);
      this.bev.view.setSignals(siteId, cameraId, entry.regions.signals);
      this.renderSignalBar(siteId);
    } catch (error) {
      toast((error as Error).message, 'error');
    }
  }

  /** A traffic light was placed in the view: store its position and direction. */
  private placeSignal(
    siteId: string,
    cameraId: string,
    signalId: string,
    position: [number, number],
    headingDeg: number,
  ): void {
    const signal = this.regions
      .get(`${siteId}/${cameraId}`)
      ?.regions.signals.find((s) => s.id === signalId);
    if (!signal) {
      return;
    }
    signal.position = position;
    signal.heading_deg = headingDeg;
    void this.saveSignals(siteId, cameraId).then(() => toast(`${signal.name} placed`, 'ok'));
  }

  private tooltip(drawn: DrawnObject | null, x: number, y: number): void {
    const tooltip = document.querySelector<HTMLElement>('#tooltip');
    if (!tooltip) {
      return;
    }
    if (!drawn) {
      tooltip.hidden = true;
      return;
    }
    const { object } = drawn;
    const site = this.sites.find((s) => s.id === drawn.site);
    const cameraNames = drawn.cameras.map(
      (id) => site?.cameras.find((c) => c.id === id)?.name ?? id,
    );
    const speed = object.speed_mps * 3.6;
    const std =
      object.speed_std_mps !== null ? ` ± ${(object.speed_std_mps * 3.6).toFixed(1)}` : '';
    fill(
      tooltip,
      h(
        'strong',
        {},
        `${CLASS_LABELS[object.class] ?? object.class}${object.rider ? ' (rider)' : ''}`,
        h('span', { class: 'muted' }, ` #${object.id}`),
      ),
      h('span', {}, `${speed.toFixed(1)}${std} km/h`),
      h('span', {}, `confidence ${(object.confidence * 100).toFixed(0)} %`),
      object.zones.length > 0 ? h('span', {}, `in ${object.zones.join(', ')}`) : null,
      site && (this.focus === ALL || site.cameras.length > 1)
        ? h(
            'span',
            { class: 'muted' },
            `${this.focus === ALL ? `${site.name} · ` : ''}seen by ${cameraNames.join(' + ')}`,
          )
        : null,
      object.validated ? null : h('span', { class: 'muted' }, 'outside the validated region'),
      (drawn.unmerged ?? []).map(({ camera, skewMs }) =>
        h(
          'span',
          { class: 'muted' },
          `not merged with ${site?.cameras.find((c) => c.id === camera)?.name ?? camera}: ${skewMs.toFixed(0)} ms apart`,
        ),
      ),
    );
    tooltip.style.transform = `translate(${x + 16}px, ${y + 12}px)`;
    tooltip.hidden = false;
  }
}

function setupControls(
  view: TwinView,
  bev: BevMode,
  hasMap: boolean,
  alignSite: Site | null,
  signals: HTMLElement | null = null,
): void {
  const controls = document.querySelector<HTMLElement>('#view-controls');
  if (!controls) {
    return;
  }
  const mode = segmented<ViewMode>(
    [
      { value: '2d', label: '2D' },
      { value: '3d', label: '3D' },
    ],
    '3d',
    (value) => view.setViewMode(value),
  );
  const layer = segmented<BaseLayer>(
    [
      { value: 'map', label: 'Map', icon: icon('map') },
      { value: 'ortho', label: 'Aerial', icon: icon('image') },
      { value: 'roadmap', label: 'Road map', icon: icon('route') },
    ],
    'map',
    (value) => view.showLayer(value),
  );
  // The toolbar keeps what is used often; display options go into the Layers menu, one group for
  // the 3D twin and one for the BEV (each shown in its view only).
  const twinLayers = h(
    'div',
    { class: 'layers-group twin-only' },
    hasMap ? layer.root : null,
    hasMap ? toggle('Buildings', true, (on) => view.showBuildings(on)) : null,
    toggle('Trails', true, (on) => view.setTrails(on)),
    analysisToggle(view),
  );
  const bevLayers = bev.optionControls();
  bevLayers.classList.add('bev-only');
  const twinOnly = [mode.root, alignSite ? alignButton(view, alignSite) : null, signals].filter(
    (e): e is HTMLElement => e !== null,
  );
  for (const e of twinOnly) {
    e.classList.add('twin-only');
  }
  fill(
    controls,
    bev.switchControl(),
    twinOnly,
    layersMenu(twinLayers, bevLayers),
    h(
      'button',
      {
        type: 'button',
        class: 'btn ghost small icon-only twin-only',
        title: 'Reset view',
        onclick: () => view.recenter(),
      },
      icon('recenter'),
    ),
    bev.fitControl(),
  );
}

// Closes the open Layers menu's outside-click listener when the toolbar is rebuilt.
let layersMenuListener: AbortController | null = null;

/** "Layers": a menu under the toolbar with the display options of the current view. */
function layersMenu(...groups: HTMLElement[]): HTMLElement {
  const menu = h('div', { class: 'layers-menu glass', hidden: true }, groups);
  const button = h(
    'button',
    { type: 'button', class: 'btn small ghost', title: 'Map layers and display options' },
    icon('layers'),
    'Layers',
  );
  const root = h('div', { class: 'layers' }, button, menu);
  button.addEventListener('click', () => {
    menu.hidden = !menu.hidden;
    button.classList.toggle('active', !menu.hidden);
  });
  layersMenuListener?.abort();
  layersMenuListener = new AbortController();
  document.addEventListener(
    'pointerdown',
    (e) => {
      if (!menu.hidden && !root.contains(e.target as Node)) {
        menu.hidden = true;
        button.classList.remove('active');
      }
    },
    { signal: layersMenuListener.signal },
  );
  return root;
}

/**
 * "Analysis": models translucent, so the ground footprints — where the perception data puts each
 * road user — can be compared with the BEV.
 */
function analysisToggle(view: TwinView): HTMLElement {
  const label = toggle('Analysis', false, (on) => view.setAnalysis(on));
  label.title =
    'Translucent models: the footprints on the ground show the backend positions (dashed: class size, disc: no heading, grey: lost)';
  return label;
}

/** "Align map": drag the map onto the road users, then save its placement for the site. */
function alignButton(view: TwinView, site: Site): HTMLElement {
  const button = h('button', { type: 'button', class: 'btn small' }, icon('align'), 'Align map');
  const bar = document.querySelector<HTMLElement>('#align-bar');
  if (!bar) {
    return button;
  }
  const readout = h('span', { class: 'num readout' });
  const show = (t: MapTransform): void => {
    readout.textContent = `${t.yaw_deg.toFixed(1)}° · ${t.x_m.toFixed(1)} m, ${t.y_m.toFixed(1)} m`;
  };
  let before = view.mapTransform(site.id);
  const finish = (): void => {
    view.setAlignMode(null);
    bar.hidden = true;
    button.disabled = false;
  };
  const save = h('button', { type: 'button', class: 'btn small primary' }, icon('check'), 'Save');
  save.addEventListener('click', () => {
    void withBusy(
      save,
      () => updateSite(site.id, { map_transform: view.mapTransform(site.id) }),
      'Map placement saved',
    ).then((saved) => {
      if (saved) {
        finish();
      }
    });
  });
  const cancel = h('button', { type: 'button', class: 'btn small ghost' }, 'Cancel');
  cancel.addEventListener('click', () => {
    view.setMapTransform(site.id, before);
    finish();
  });
  fill(
    bar,
    icon('align'),
    h('span', {}, 'Drag to move the map, right-drag or shift-drag to rotate it, wheel to zoom.'),
    readout,
    cancel,
    save,
  );
  view.onMapTransform = show;
  button.addEventListener('click', () => {
    before = view.mapTransform(site.id);
    show(before);
    view.setAlignMode(site.id);
    bar.hidden = false;
    button.disabled = true;
    toast('Tip: switch to Aerial and match the roads to the moving road users.', 'info');
  });
  return button;
}

/** A single jp_server without the manager: its one scene and camera. */
async function startSingle(view: TwinView, bev: BevMode): Promise<void> {
  const scene = await fetchScene();
  const id = scene.scene_id;
  const spec = { id, name: id, offset: { x: 0, y: 0 }, transform: null };
  view.addSite(spec);
  view.addCamera(id, 'main', scene);
  bev.view.addSite(spec);
  bev.view.addCamera(id, 'main', scene);
  bev.view.setHome(id);
  if (scene.map) {
    const map = await loadMapData(scene.map);
    view.setMap(id, map);
    bev.view.setMap(id, map);
    bev.renderOptions(map.ortho !== null);
    document
      .querySelector<HTMLElement>('#attribution')
      ?.replaceChildren(
        ...attributionLines(map).map(({ text, url }) =>
          h('a', { href: url, target: '_blank', rel: 'noopener' }, text),
        ),
      );
  }
  view.focus(id);
  bev.view.focus(id);
  bev.view.onHover = () => undefined;
  setupControls(view, bev, scene.map !== null, null);
  const panel = new SidePanel();
  panel.setCameras([{ id: 'main', name: 'Camera', base: '', running: true }], 'main');
  subscribeWorld(
    (state) => {
      view.push(id, 'main', state);
      bev.view.push(id, 'main', state);
      panel.showFrame(state.frame, state.timestamp_ns);
    },
    () => undefined,
  );
  subscribeVideo((frame) => panel.showVideo(frame));
  setInterval(() => {
    panel.showWorld(
      bev.current() === 'bev' ? bev.view.objects(id) : view.objects(id),
      view
        .states(id)
        .flatMap(({ state }) =>
          state.events.map((event) => ({ key: String(event.id), label: '', event })),
        ),
    );
    bev.render();
  }, PANEL_INTERVAL_MS);
  let missed = 0;
  const refresh = async (): Promise<void> => {
    try {
      panel.showStatus(await fetchStatus(), 1, 1);
      missed = 0;
    } catch {
      if (++missed >= MISSED_STATUS_LIMIT) {
        panel.showStatus(null, 0, 1);
      }
    }
  };
  await refresh();
  setInterval(() => void refresh(), STATUS_INTERVAL_MS);
}

/**
 * Options of the 3D view; `?fusionSkewMs=` (largest capture-time difference of merged camera
 * tracks, ms; "inf" merges regardless) and `?lostHoldS=` (how long a lost road user stays, s) are
 * experimental settings.
 */
function viewOptions(): ConstructorParameters<typeof TwinView>[2] {
  const number = (name: string): number | undefined => {
    const text = param(name);
    if (text === null) {
      return undefined;
    }
    const value = text === 'inf' ? Infinity : Number.parseFloat(text);
    return Number.isNaN(value) || value < 0 ? undefined : value;
  };
  return {
    theme: currentTheme(),
    fusionMaxSkewMs: number('fusionSkewMs'),
    lostHoldS: number('lostHoldS'),
  };
}

async function start(): Promise<void> {
  const siteId = param('site');
  const shell = await mountShell({
    page: 'twin',
    siteId,
    title: siteId ? 'Live twin' : 'Digital twin',
    fullBleed: true,
  });
  setSite(null);
  const twinLayout = layout();
  shell.content.append(twinLayout);
  // The side panel, and with it the camera image, is resized at its left edge.
  makeResizable(element<HTMLElement>('#panel-resizer'), {
    key: 'jp-twin-panel-width',
    initial: 380,
    min: 300,
    max: () => twinLayout.clientWidth * 0.75,
    direction: -1,
    apply: (width) => twinLayout.style.setProperty('--panel-width', `${width}px`),
  });
  const canvas = document.querySelector<HTMLCanvasElement>('#twin');
  if (!canvas) {
    throw new Error('canvas #twin not found');
  }
  if (!shell.managed) {
    const scene = await fetchScene();
    const view = new TwinView(canvas, scene.classes, viewOptions());
    const bev = new BevMode(
      element<HTMLElement>('.twin-stage'),
      view,
      { panelSite: () => scene.scene_id, siteName: (s) => s, cameraName: () => 'Camera' },
      scene.classes,
    );
    onThemeChange((theme) => {
      view.setTheme(theme);
      bev.view.setTheme();
    });
    await startSingle(view, bev);
    return;
  }
  const view = new TwinView(canvas, [], viewOptions());
  // Hooks of the BEV read the page, which needs the BEV: created first, the page set below.
  let page: TwinPage | null = null;
  const bev = new BevMode(element<HTMLElement>('.twin-stage'), view, {
    panelSite: () => page?.panelSite() ?? null,
    siteName: (s) => page?.siteName(s) ?? s,
    cameraName: (s, c) => page?.cameraName(s, c) ?? c,
  });
  onThemeChange((theme) => {
    view.setTheme(theme);
    bev.view.setTheme();
  });
  const twinPage = new TwinPage(view, bev, shell, siteId, param('camera'));
  page = twinPage;
  const sites = await listSites().catch(() => shell.sites());
  const focused = sites.find((s) => s.id === siteId);
  if (focused) {
    shell.setTitle(focused.name);
  }
  twinPage.sync(sites);
  twinPage.setActions();
  shell.onSites((fresh) => twinPage.sync(fresh));
}

void start();
