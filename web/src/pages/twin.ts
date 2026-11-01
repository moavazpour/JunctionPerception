import '../styles/twin.css';

import { loadMapData, type MapData } from '../map/mapData';
import { attributionLines } from '../map/mapLayers';
import {
  fetchScene,
  fetchStatus,
  setSite,
  siteUrl,
  subscribeVideo,
  subscribeWorld,
} from '../net/client';
import { getSite, type Site, updateSite } from '../net/manager';
import type { ViewMode } from '../twin/orbitRig';
import { CLASS_LABELS } from '../twin/palette';
import { type BaseLayer, type MapTransform, TwinView } from '../twin/twinView';
import { fill, h, param, segmented, toggle } from '../ui/dom';
import { toast, withBusy } from '../ui/feedback';
import { icon } from '../ui/icons';
import { links, mountShell } from '../ui/shell';
import { SidePanel } from '../ui/sidePanel';
import { waitForSite } from '../ui/siteGate';
import { currentTheme, onThemeChange } from '../ui/theme';

const STATUS_INTERVAL_MS = 1000;
// Status requests that may fail in a row before the site counts as unreachable.
const MISSED_STATUS_LIMIT = 3;

function layout(): HTMLElement {
  return h(
    'div',
    { class: 'twin-layout' },
    h(
      'section',
      { class: 'twin-stage' },
      h('canvas', { id: 'twin' }),
      h('div', { class: 'hud hud-tl' }, h('div', { id: 'live-chip', class: 'glass chip' })),
      h('div', { id: 'view-controls', class: 'hud hud-tr glass toolbar' }),
      h('div', { id: 'align-bar', class: 'align-bar glass', hidden: true }),
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
        'Connection to the site lost, reconnecting…',
      ),
    ),
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
          h('h3', {}, 'Camera'),
          h('span', { id: 'video-frame', class: 'frame-tag num' }),
        ),
        h(
          'div',
          { class: 'camera-frame' },
          h('img', { id: 'video', alt: 'Camera image with tracked road users', hidden: true }),
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

function setupControls(view: TwinView, map: MapData | null, site: Site | null): void {
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
  const layers: { value: BaseLayer; label: string; icon: Node; disabled?: boolean }[] = [
    { value: 'map', label: 'Map', icon: icon('map') },
    { value: 'ortho', label: 'Aerial', icon: icon('image'), disabled: !map?.ortho },
  ];
  if (map?.roadmap) {
    layers.push({ value: 'roadmap', label: 'Road map', icon: icon('route') });
  }
  const layer = segmented<BaseLayer>(layers, 'map', (value) => view.showLayer(value));
  // Sensor-frame scenes (benchmark) place their map by hand.
  const canAlign = map !== null && site !== null && !site.georeferenced;
  fill(
    controls,
    mode.root,
    map ? layer.root : null,
    map ? toggle('Buildings', true, (on) => view.showBuildings(on)) : null,
    toggle('Trails', true, (on) => view.setTrails(on)),
    canAlign && site ? alignButton(view, site) : null,
    h(
      'button',
      {
        type: 'button',
        class: 'btn ghost small icon-only',
        title: 'Reset view',
        onclick: () => view.recenter(),
      },
      icon('recenter'),
    ),
  );
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
  let before = view.mapTransform;
  const finish = (): void => {
    view.setAlignMode(false);
    bar.hidden = true;
    button.disabled = false;
  };
  const save = h('button', { type: 'button', class: 'btn small primary' }, icon('check'), 'Save');
  save.addEventListener('click', () => {
    void withBusy(
      save,
      () => updateSite(site.id, { map_transform: view.mapTransform }),
      'Map placement saved',
    ).then((saved) => {
      if (saved) {
        finish();
      }
    });
  });
  const cancel = h('button', { type: 'button', class: 'btn small ghost' }, 'Cancel');
  cancel.addEventListener('click', () => {
    view.setMapTransform(before);
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
    before = view.mapTransform;
    show(before);
    view.setAlignMode(true);
    bar.hidden = false;
    button.disabled = true;
    toast('Tip: switch to Aerial and match the roads to the moving road users.', 'info');
  });
  return button;
}

function setupTooltip(view: TwinView, panel: SidePanel): void {
  const tooltip = document.querySelector<HTMLElement>('#tooltip');
  if (!tooltip) {
    return;
  }
  view.onHover = (id, x, y) => {
    const object = id === null ? undefined : panel.object(id);
    if (!object) {
      tooltip.hidden = true;
      return;
    }
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
      object.validated ? null : h('span', { class: 'muted' }, 'outside the validated region'),
    );
    tooltip.style.transform = `translate(${x + 16}px, ${y + 12}px)`;
    tooltip.hidden = false;
  };
}

async function start(): Promise<void> {
  const siteId = param('site');
  const shell = await mountShell({ page: 'twin', siteId, title: 'Live twin', fullBleed: true });
  if (shell.managed && !siteId) {
    location.replace('/');
    return;
  }
  setSite(shell.managed ? siteId : null);
  let site: Site | null = null;
  if (shell.managed && siteId) {
    shell.setActions(
      h(
        'a',
        { class: 'btn small', href: links.calibration(siteId) },
        icon('calibration'),
        'Calibration',
      ),
      h('a', { class: 'btn small', href: links.settings(siteId) }, icon('settings'), 'Settings'),
    );
    await waitForSite(shell.content, siteId, 'see the live digital twin');
    site = await getSite(siteId).catch(() => null);
  }

  shell.content.append(layout());
  const canvas = document.querySelector<HTMLCanvasElement>('#twin');
  if (!canvas) {
    throw new Error('canvas #twin not found');
  }
  const scene = await fetchScene();
  shell.setTitle('Live twin');
  const view = new TwinView(canvas, scene, {
    siteKey: siteId ?? scene.scene_id,
    theme: currentTheme(),
  });
  onThemeChange((theme) => view.setTheme(theme));
  const panel = new SidePanel();
  setupTooltip(view, panel);
  let map: MapData | null = null;
  if (scene.map) {
    map = await loadMapData(siteUrl(scene.map));
    view.setMap(map);
    if (site?.map_transform) {
      view.setMapTransform(site.map_transform);
    }
    const attribution = document.querySelector<HTMLElement>('#attribution');
    attribution?.replaceChildren(
      ...attributionLines(map).map(({ text, url }) =>
        h('a', { href: url, target: '_blank', rel: 'noopener' }, text),
      ),
    );
  }
  setupControls(view, map, site);

  // The world socket reconnects quietly while playback is paused (no messages arrive then);
  // whether the site is reachable comes from the status requests.
  subscribeWorld(
    (state) => {
      view.push(state);
      panel.showState(state);
    },
    () => undefined,
  );
  subscribeVideo((frame) => panel.showVideo(frame));

  let missed = 0;
  const refreshStatus = async (): Promise<void> => {
    try {
      const status = await fetchStatus();
      missed = 0;
      panel.showStatus(status, true);
    } catch {
      if (++missed >= MISSED_STATUS_LIMIT) {
        panel.showStatus(
          { scene_id: scene.scene_id, source: '', frame_count: 0, seekable: false },
          false,
        );
      }
    }
  };
  await refreshStatus();
  setInterval(() => void refreshStatus(), STATUS_INTERVAL_MS);
}

void start();
