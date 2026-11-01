import '../styles/twin.css';

import { loadMapData } from '../map/mapData';
import { attributionLines } from '../map/mapLayers';
import {
  fetchScene,
  fetchStatus,
  setSite,
  siteUrl,
  subscribeVideo,
  subscribeWorld,
} from '../net/client';

import { CLASS_LABELS } from '../twin/palette';
import { type BaseLayer, TwinView } from '../twin/twinView';
import type { ViewMode } from '../twin/orbitRig';
import { fill, h, param, segmented, toggle } from '../ui/dom';
import { waitForSite } from '../ui/siteGate';
import { icon } from '../ui/icons';
import { links, mountShell } from '../ui/shell';
import { SidePanel } from '../ui/sidePanel';

const STATUS_INTERVAL_MS = 1000;

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

function setupControls(view: TwinView, hasOrtho: boolean, hasMap: boolean): void {
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
      { value: 'ortho', label: 'Aerial', icon: icon('image'), disabled: !hasOrtho },
    ],
    'map',
    (value) => view.showLayer(value),
  );
  fill(
    controls,
    mode.root,
    hasMap ? layer.root : null,
    hasMap ? toggle('Buildings', true, (on) => view.showBuildings(on)) : null,
    toggle('Trails', true, (on) => view.setTrails(on)),
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
  }

  shell.content.append(layout());
  const canvas = document.querySelector<HTMLCanvasElement>('#twin');
  if (!canvas) {
    throw new Error('canvas #twin not found');
  }
  const scene = await fetchScene();
  shell.setTitle('Live twin');
  const view = new TwinView(canvas, scene);
  const panel = new SidePanel();
  setupTooltip(view, panel);
  let hasOrtho = false;
  if (scene.map) {
    const map = await loadMapData(siteUrl(scene.map));
    view.setMap(map);
    hasOrtho = map.ortho !== null;
    const attribution = document.querySelector<HTMLElement>('#attribution');
    attribution?.replaceChildren(
      ...attributionLines(map).map(({ text, url }) =>
        h('a', { href: url, target: '_blank', rel: 'noopener' }, text),
      ),
    );
  }
  setupControls(view, hasOrtho, scene.map !== null);

  let connected = false;
  subscribeWorld(
    (state) => {
      view.push(state);
      panel.showState(state);
    },
    (isConnected) => (connected = isConnected),
  );
  subscribeVideo((frame) => panel.showVideo(frame));

  const refreshStatus = async (): Promise<void> => {
    try {
      panel.showStatus(await fetchStatus(), connected);
    } catch {
      panel.showStatus(
        { scene_id: scene.scene_id, source: '', frame_count: 0, seekable: false },
        false,
      );
    }
  };
  await refreshStatus();
  setInterval(() => void refreshStatus(), STATUS_INTERVAL_MS);
}

void start();
