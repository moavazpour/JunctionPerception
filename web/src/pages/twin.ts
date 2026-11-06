import '../styles/twin.css';

import { BevView, type Picked } from '../bev/bevView';
import { CLASS_LABELS, setPaletteTheme } from '../bev/palette';
import { geoOffset, type MapTransform, type SiteSpec } from '../bev/siteFrame';
import { attributionLines, loadMapData, type MapData } from '../map/mapData';
import { fetchRegions, type Regions, saveRegions } from '../net/calibrationApi';
import {
  cameraBase,
  fetchScene,
  fetchStatus,
  type ServerStatus,
  subscribeSite,
  subscribeVideo,
  type Unsubscribe,
} from '../net/client';
import {
  type GeoAnchor,
  listSites,
  type Site,
  siteAction,
  sitePlayback,
  updateSite,
} from '../net/manager';
import type { SiteState } from '../net/siteState';
import { element, fill, h, param, toggle } from '../ui/dom';
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
      h('canvas', { id: 'twin', class: 'bev-canvas' }),
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
        { class: 'panel-card fusion-card' },
        h('header', {}, icon('layers'), h('h3', {}, 'World model')),
        h('ul', { id: 'fusion', class: 'fusion' }),
      ),
      h(
        'section',
        { id: 'signals-card', class: 'panel-card signals-card', hidden: true },
        h('header', {}, icon('pulse'), h('h3', {}, 'Traffic lights')),
        h('ul', { id: 'signals', class: 'signals' }),
      ),
    ),
  );
}

/** The twin page: which sites are drawn, focused and streamed, and their configuration tools. */
class TwinPage {
  private readonly panel = new SidePanel();
  // Site state stream of every placed site, and the cameras whose scenes are loaded.
  private readonly streams = new Map<string, Unsubscribe>();
  private readonly cameras = new Set<string>();
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
  private readonly attributions = new Map<string, { text: string; url: string }[]>();
  // Regions (with the traffic lights) of each loaded camera, by "site/camera".
  private readonly regions = new Map<string, { base: string; regions: Regions }>();
  private controlsKey = '';
  private hasAerial = false;

  constructor(
    private readonly view: BevView,
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
    view.onHover = (picked, x, y) => this.tooltip(picked, x, y);
    view.onSignalPlaced = (site, cameraId, id, position, heading) =>
      this.placeSignal(site, cameraId, id, position, heading);
    setInterval(() => this.renderPanel(), PANEL_INTERVAL_MS);
    setInterval(() => void this.refreshStatus(), STATUS_INTERVAL_MS);
  }

  /** Takes a new site list: places new sites, loads cameras that start. */
  sync(sites: Site[]): void {
    this.sites = sites;
    if (!this.origin) {
      const focused = sites.find((s) => s.id === this.focus);
      this.origin = focused?.geo_anchor ?? sites.find((s) => s.geo_anchor)?.geo_anchor ?? null;
    }
    for (const site of sites) {
      this.place(site);
      if (site.map_transform && !site.georeferenced && this.view.alignedSite() !== site.id) {
        this.view.setMapTransform(site.id, site.map_transform);
      }
      for (const camera of site.cameras) {
        if (camera.runtime.state === 'running') {
          this.loadCamera(site, camera.id);
        }
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

  /** Adds a site to the view: its map and its site state stream. */
  private place(site: Site): void {
    if (this.placed.has(site.id)) {
      return;
    }
    const spec = this.spec(site);
    if (!spec) {
      return;
    }
    this.placed.add(site.id);
    this.view.addSite(spec);
    if (site.has_map) {
      void loadMapData(`/sites/${encodeURIComponent(site.id)}/map/map.json`)
        .then((map: MapData) => {
          this.view.setMap(site.id, map);
          this.hasAerial ||= map.ortho !== null;
          this.controlsKey = '';
          this.renderControls();
          this.attributions.set(site.id, attributionLines(map));
          this.renderAttribution();
        })
        .catch((error: unknown) =>
          toast(`Map of ${site.name}: ${(error as Error).message}`, 'error'),
        );
    }
    this.streams.set(
      site.id,
      subscribeSite(site.id, (state) => this.view.setState(site.id, state)),
    );
    if (site.id === this.focus && site.cameras.every((c) => c.runtime.state !== 'running')) {
      // Nothing to wait for: look at the site's origin.
      this.frame();
    }
  }

  /** Scene (position, coverage, zones) and regions (lines, traffic lights) of a camera. */
  private loadCamera(site: Site, cameraId: string): void {
    const key = `${site.id}/${cameraId}`;
    if (!this.placed.has(site.id) || this.cameras.has(key) || this.loading.has(key)) {
      return;
    }
    this.loading.add(key);
    const base = cameraBase(site.id, cameraId);
    void fetchScene(base)
      .then((description) => {
        this.cameras.add(key);
        // Its place among the site's cameras keeps its colour when cameras load out of order.
        const index = site.cameras.findIndex((c) => c.id === cameraId);
        const name = site.cameras[index]?.name ?? cameraId;
        this.view.addCamera(site.id, cameraId, description, name, Math.max(index, 0));
        void fetchRegions(base)
          .then((regions) => {
            this.regions.set(key, { base, regions });
            this.view.setSignals(site.id, cameraId, regions.signals);
            this.view.setLines(site.id, cameraId, regions.lines);
            this.view.setAreas(
              site.id,
              cameraId,
              regions.areas.map((area) => area.points),
            );
            this.renderSignalBar(site.id);
          })
          .catch(() => undefined);
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
    } else {
      this.view.focus(this.focus);
    }
    this.view.setHome(this.focus === ALL ? null : this.focus);
    this.framed = true;
  }

  private focusedSite(): Site | undefined {
    return this.sites.find((s) => s.id === this.focus);
  }

  private shownSites(): Site[] {
    return this.focus === ALL ? this.sites : this.sites.filter((s) => s.id === this.focus);
  }

  /** Cameras the panel offers: those of the focused site, or of all sites. */
  private panelCameras(): (PanelCamera & { site: Site })[] {
    return this.shownSites().flatMap((site) =>
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

  /** Counters, fusion status, traffic lights and frame of the shown sites' states. */
  private renderPanel(): void {
    const states = this.shownSites()
      .map((s) => this.view.state(s.id))
      .filter((s): s is SiteState => s !== null);
    this.panel.showSites(states);
    const several = this.shownSites().some((s) => s.cameras.length > 1) || states.length > 1;
    this.panel.showSignals(
      states.flatMap((state) =>
        state.signals.map((s) => ({
          name: several ? `${s.name} · ${this.cameraName(state.site_id, s.camera)}` : s.name,
          state: s.state,
          seconds: Math.max(0, (state.t_ns - s.since_ns) / 1e9),
        })),
      ),
    );
    const [siteId, cameraId] = (this.camera ?? '').split('/');
    const camera = states.find((s) => s.site_id === siteId)?.cameras.find((c) => c.id === cameraId);
    if (camera?.frame !== null && camera?.frame !== undefined && camera.last_t_ns !== null) {
      this.panel.showFrame(camera.frame, camera.last_t_ns - camera.offset_ms * 1e6);
    }
  }

  private cameraName(siteId: string, cameraId: string): string {
    return (
      this.sites.find((s) => s.id === siteId)?.cameras.find((c) => c.id === cameraId)?.name ??
      cameraId
    );
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
    const sites = this.shownSites();
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
    const target = document.querySelector<HTMLElement>('#attribution');
    const seen = new Map<string, { text: string; url: string }>();
    for (const lines of this.attributions.values()) {
      for (const line of lines) {
        seen.set(line.text, line);
      }
    }
    target?.replaceChildren(
      ...[...seen.values()].map(({ text, url }) =>
        h('a', { href: url, target: '_blank', rel: 'noopener' }, text),
      ),
    );
  }

  private renderControls(): void {
    const site = this.focusedSite();
    // Sensor-frame scenes (benchmark) place their map by hand.
    const canAlign = Boolean(
      site && !site.georeferenced && site.has_map && this.placed.has(site.id),
    );
    const signals = site
      ? site.cameras.reduce((sum, c) => sum + (c.regions?.signals.length ?? 0), 0)
      : 0;
    const key = `${this.hasAerial}:${canAlign}:${site?.id}:${signals}`;
    if (key === this.controlsKey) {
      return;
    }
    this.controlsKey = key;
    const controls = document.querySelector<HTMLElement>('#view-controls');
    if (!controls) {
      return;
    }
    fill(
      controls,
      canAlign && site ? alignButton(this.view, site) : null,
      site && signals > 0 ? this.signalButton(site) : null,
      layersMenu(this.view, this.hasAerial),
      h(
        'button',
        {
          type: 'button',
          class: 'btn ghost small icon-only',
          title: 'Fit the view (double click)',
          onclick: () => this.view.recenter(),
        },
        icon('recenter'),
      ),
    );
  }

  /** "Traffic lights": place each signal of the site in the view. */
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
        const color = this.view.signalState(site.id, camera.id, signal.id).color;
        return h(
          'div',
          { class: 'signal-row' },
          h('span', { class: `signal-dot ${color}` }),
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

  private async saveSignals(siteId: string, cameraId: string): Promise<void> {
    const entry = this.regions.get(`${siteId}/${cameraId}`);
    if (!entry) {
      return;
    }
    try {
      entry.regions = await saveRegions(entry.regions, entry.base);
      this.view.setSignals(siteId, cameraId, entry.regions.signals);
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

  private tooltip(picked: Picked | null, x: number, y: number): void {
    const tooltip = document.querySelector<HTMLElement>('#tooltip');
    if (!tooltip) {
      return;
    }
    if (!picked) {
      tooltip.hidden = true;
      return;
    }
    const { user } = picked;
    const site = this.sites.find((s) => s.id === picked.site);
    const speed = user.speed_mps * 3.6;
    const std = user.speed_std_mps !== null ? ` ± ${(user.speed_std_mps * 3.6).toFixed(1)}` : '';
    const heading =
      user.heading_rad === null
        ? 'heading unknown'
        : `heading ${((user.heading_rad * 180) / Math.PI).toFixed(0)}° (${user.heading_source})`;
    const cameras = user.observed_by.map(
      (o) => `${site?.cameras.find((c) => c.id === o.camera)?.name ?? o.camera} #${o.track_id}`,
    );
    fill(
      tooltip,
      h(
        'strong',
        {},
        `${CLASS_LABELS[user.class] ?? user.class}${user.rider ? ' (rider)' : ''}`,
        h('span', { class: 'muted' }, ` #${user.id}`),
      ),
      h('span', {}, `${speed.toFixed(1)}${std} km/h · ${heading}`),
      h('span', {}, `confidence ${(user.confidence * 100).toFixed(0)} %`),
      h(
        'span',
        { class: 'muted' },
        user.status === 'coasting' ? 'not seen now (predicted)' : `seen by ${cameras.join(' + ')}`,
      ),
      user.size_source === 'class' ? h('span', { class: 'muted' }, 'size of its class') : null,
      user.validated ? null : h('span', { class: 'muted' }, 'outside the validated region'),
    );
    tooltip.style.transform = `translate(${x + 16}px, ${y + 12}px)`;
    tooltip.hidden = false;
  }
}

// Closes the open Layers menu's outside-click listener when the toolbar is rebuilt.
let layersMenuListener: AbortController | null = null;

/** "Layers": a menu under the toolbar with the display options of the view. */
function layersMenu(view: BevView, hasAerial: boolean): HTMLElement {
  const o = view.options;
  const menu = h(
    'div',
    { class: 'layers-menu glass', hidden: true },
    h(
      'div',
      { class: 'layers-group' },
      hasAerial ? toggle('Aerial image', o.aerial, (on) => view.setOptions({ aerial: on })) : null,
      toggle('Trails', o.trails, (on) => view.setOptions({ trails: on })),
      toggle('Uncertainty (2σ)', o.uncertainty, (on) => view.setOptions({ uncertainty: on })),
      toggle('Labels', o.labels, (on) => view.setOptions({ labels: on })),
    ),
  );
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

/** "Align map": drag the site onto its map, then save the map placement. */
function alignButton(view: BevView, site: Site): HTMLElement {
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
    h('span', {}, 'Drag to move the site over the map, right- or shift-drag to turn it.'),
    readout,
    cancel,
    save,
  );
  view.onMapTransform = (_, t) => show(t);
  button.addEventListener('click', () => {
    before = view.mapTransform(site.id);
    show(before);
    view.setAlignMode(site.id);
    bar.hidden = false;
    button.disabled = true;
    toast('Tip: switch on the aerial image and match the roads to the road users.', 'info');
  });
  return button;
}

async function start(): Promise<void> {
  const siteId = param('site');
  const shell = await mountShell({
    page: 'twin',
    siteId,
    title: siteId ? 'Live twin' : 'Digital twin',
    fullBleed: true,
  });
  const twinLayout = layout();
  shell.content.append(twinLayout);
  if (!shell.managed) {
    fill(
      element<HTMLElement>('.twin-stage'),
      h('div', { class: 'stage-empty glass' }, 'The digital twin needs the site manager.'),
    );
    return;
  }
  // The side panel, and with it the camera image, is resized at its left edge.
  makeResizable(element<HTMLElement>('#panel-resizer'), {
    key: 'jp-twin-panel-width',
    initial: 380,
    min: 300,
    max: () => twinLayout.clientWidth * 0.75,
    direction: -1,
    apply: (width) => twinLayout.style.setProperty('--panel-width', `${width}px`),
  });
  setPaletteTheme(currentTheme());
  const view = new BevView(element<HTMLCanvasElement>('#twin'));
  onThemeChange((theme) => {
    setPaletteTheme(theme);
    view.setTheme();
  });
  const page = new TwinPage(view, shell, siteId, param('camera'));
  const sites = await listSites().catch(() => shell.sites());
  const focused = sites.find((s) => s.id === siteId);
  if (focused) {
    shell.setTitle(focused.name);
  }
  page.sync(sites);
  page.setActions();
  shell.onSites((fresh) => page.sync(fresh));
}

void start();
