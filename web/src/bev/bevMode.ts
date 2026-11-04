import '../styles/bev.css';

import type { ClassInfo } from '../net/client';
import type { TwinView } from '../twin/twinView';
import { h, segmented, toggle } from '../ui/dom';
import { icon } from '../ui/icons';
import { BevPanel } from './bevPanel';
import { BevView } from './bevView';

export type StageMode = 'twin' | 'bev';

const MODE_KEY = 'jp-twin-stage-mode';

export interface BevModeHooks {
  /** Site whose lanes the card shows (null: none). */
  panelSite: () => string | null;
  siteName: (site: string) => string;
  cameraName: (site: string, camera: string) => string;
}

function storedMode(): StageMode {
  try {
    return localStorage.getItem(MODE_KEY) === 'bev' ? 'bev' : 'twin';
  } catch {
    return 'twin';
  }
}

/**
 * The experimental bird's-eye view next to the 3D twin on the twin page: its canvas, options bar
 * and status card on the stage, and the switch between the two views. Both views take the same
 * world states; the hidden one draws nothing. `?bench=1` redraws the BEV every frame, so frame
 * rates of the two views can be compared (window.jpPerf).
 */
export class BevMode {
  readonly view: BevView;
  readonly panel: BevPanel;
  private mode: StageMode = 'twin';
  private switcher: { root: HTMLElement; set: (mode: StageMode) => void } | null = null;
  private readonly canvas = h('canvas', { id: 'bev', class: 'bev-only' });
  private readonly card = h('aside', { id: 'bev-card', class: 'glass bev-card bev-only' });
  private readonly options = h('div', { id: 'bev-options', class: 'glass bev-options bev-only' });
  private readonly listeners: ((mode: StageMode) => void)[] = [];

  constructor(
    private readonly stage: HTMLElement,
    private readonly twin: TwinView,
    private readonly hooks: BevModeHooks,
    classes: ClassInfo[] = [],
  ) {
    const twinCanvas = stage.querySelector('#twin');
    twinCanvas?.after(this.canvas, this.card, this.options);
    this.view = new BevView(this.canvas, classes);
    this.panel = new BevPanel(this.card);
    this.panel.onLane = (id) => this.view.highlightLane(id ? hooks.panelSite() : null, id);
    this.view.onSelect = () => this.render();
    const bench = new URLSearchParams(location.search).get('bench') === '1';
    this.view.continuous = bench;
    if (bench) {
      // Measurement hook (docs/bev-experiment.md): both views for timed, synchronous draws.
      (window as unknown as Record<string, unknown>)['jpBench'] = { twin, bev: this.view };
    }
    this.renderOptions(false);
    this.setMode(storedMode());
  }

  current(): StageMode {
    return this.mode;
  }

  onChange(listener: (mode: StageMode) => void): void {
    this.listeners.push(listener);
  }

  /** Switch between the 3D twin and the BEV, for the toolbar. */
  switchControl(): HTMLElement {
    if (!this.switcher) {
      const control = segmented<StageMode>(
        [
          { value: 'twin', label: '3D twin' },
          { value: 'bev', label: 'BEV', icon: icon('map') },
        ],
        this.mode,
        (value) => this.setMode(value),
      );
      control.root.title = 'Bird’s-eye view (experimental): metric, unsmoothed, with lane status';
      this.onChange((mode) => control.set(mode));
      this.switcher = control;
    }
    return this.switcher.root;
  }

  setMode(mode: StageMode): void {
    this.mode = mode;
    this.stage.classList.toggle('bev-mode', mode === 'bev');
    this.twin.setActive(mode === 'twin');
    this.view.setActive(mode === 'bev');
    try {
      localStorage.setItem(MODE_KEY, mode);
    } catch {
      // The choice is not remembered.
    }
    if (mode === 'bev') {
      this.render();
    }
    for (const listener of this.listeners) {
      listener(mode);
    }
  }

  /** Options of the BEV; the aerial photo only where a site has one. */
  renderOptions(hasAerial: boolean): void {
    const o = this.view.options;
    const tracks = segmented<'cameras' | 'merged'>(
      [
        { value: 'cameras', label: 'Per camera' },
        { value: 'merged', label: 'Duplicates hidden' },
      ],
      o.tracks,
      (value) => this.view.setOptions({ tracks: value }),
    );
    tracks.root.title =
      'Per camera: every camera’s tracks, likely duplicates linked. Duplicates hidden: the less confident track of each pair close in time is not drawn (lane counts always use this).';
    this.options.replaceChildren(
      tracks.root,
      hasAerial ? toggle('Aerial', o.aerial, (on) => this.view.setOptions({ aerial: on })) : '',
      toggle('Trajectories', o.trails, (on) => this.view.setOptions({ trails: on })),
      toggle('Uncertainty 2σ', o.uncertainty, (on) => this.view.setOptions({ uncertainty: on })),
      toggle('Labels', o.labels, (on) => this.view.setOptions({ labels: on })),
      h(
        'button',
        {
          type: 'button',
          class: 'btn ghost small icon-only',
          title: 'Fit the view (or double click)',
          onclick: () => this.view.recenter(),
        },
        icon('recenter'),
      ),
    );
  }

  /** Redraws the status card (called a few times a second while the BEV is shown). */
  render(): void {
    if (this.mode !== 'bev') {
      return;
    }
    const site = this.hooks.panelSite() ?? this.view.centralSite();
    if (!site) {
      this.panel.waiting('Waiting for a camera…');
      return;
    }
    const snapshot = this.view.snapshot(site);
    this.panel.render({
      siteName: this.hooks.siteName(site),
      status: this.view.status(site),
      states: this.view.cameraStates(site).map((s) => ({
        ...s,
        name: this.hooks.cameraName(site, s.camera),
      })),
      quality: this.view.world.quality(site),
      tracks: snapshot.tracks,
      duplicates: snapshot.duplicates,
      selected: this.view.selectedTrack(),
    });
  }
}
