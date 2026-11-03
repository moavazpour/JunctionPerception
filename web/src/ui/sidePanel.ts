import {
  type PlaybackCommand,
  sendPlayback,
  type ServerStatus,
  type VideoFrame,
} from '../net/client';
import type { DrawnObject } from '../twin/twinView';
import { CLASS_LABELS, classCss } from '../twin/palette';
import { element, fill, h } from './dom';
import {
  describeEvent,
  EVENT_LABELS,
  formatTimestamp,
  type TaggedEvent,
  TaggedEventLog,
} from './events';
import { icon } from './icons';

/** A camera the panel can show, with the address of its server. */
export interface PanelCamera {
  id: string;
  name: string;
  base: string;
  running: boolean;
}

/**
 * Panels of the twin page: camera image (with tabs for the cameras of the site), counters,
 * class legend, event list, playback bar of the shown camera and the live status chip.
 */
export class SidePanel {
  private readonly video = element<HTMLImageElement>('#video');
  private readonly videoFrame = element<HTMLElement>('#video-frame');
  private readonly videoEmpty = element<HTMLElement>('#video-empty');
  private readonly cameraTabs = element<HTMLElement>('#camera-tabs');
  private readonly cameraTitle = element<HTMLElement>('#camera-title');
  private readonly events = element<HTMLUListElement>('#events');
  private readonly eventCount = element<HTMLElement>('#event-count');
  private readonly stats = element<HTMLElement>('#stats');
  private readonly legend = element<HTMLElement>('#legend');
  private readonly liveChip = element<HTMLElement>('#live-chip');
  private readonly playButton = element<HTMLButtonElement>('#play');
  private readonly slider = element<HTMLInputElement>('#seek');
  private readonly frameLabel = element<HTMLElement>('#frame');
  private readonly offline = element<HTMLElement>('#offline');
  private readonly signalsCard = element<HTMLElement>('#signals-card');
  private readonly signals = element<HTMLUListElement>('#signals');
  private readonly log = new TaggedEventLog();
  private videoUrl: string | null = null;
  private playing = true;
  private sliding = false;
  private base = '';
  private tabsKey = '';

  /** Called when another camera is chosen in the tabs. */
  onCamera: (id: string) => void = () => undefined;
  /** Play, pause and seek (frame of the shown camera): by default for that camera only. */
  onPlayback: (command: PlaybackCommand) => Promise<unknown> = (command) =>
    sendPlayback(command, this.base);

  constructor() {
    // Enlarged, the camera image fills the window over the page (button, double click, Esc).
    const frame = this.video.parentElement;
    const expand = element<HTMLButtonElement>('#video-expand');
    const setExpanded = (on: boolean) => {
      frame?.classList.toggle('expanded', on);
      expand.replaceChildren(icon(on ? 'shrink' : 'expand'));
      expand.title = on ? 'Back to the panel (Esc)' : 'Enlarge (Esc to close)';
    };
    const toggle = () => setExpanded(!frame?.classList.contains('expanded'));
    expand.addEventListener('click', toggle);
    this.video.addEventListener('dblclick', toggle);
    document.addEventListener('keydown', (e) => {
      if (e.key === 'Escape') {
        setExpanded(false);
      }
    });
    this.playButton.addEventListener('click', () => {
      void this.onPlayback({ action: this.playing ? 'pause' : 'play' });
      this.setPlaying(!this.playing);
    });
    this.slider.addEventListener('input', () => (this.sliding = true));
    this.slider.addEventListener('change', () => {
      this.sliding = false;
      this.log.clear();
      void this.onPlayback({ action: 'seek', frame: Number(this.slider.value) });
    });
    this.renderEvents();
  }

  /** Cameras of the focused site and the one whose image and playback are shown. */
  setCameras(cameras: PanelCamera[], selected: string | null): void {
    const camera = cameras.find((c) => c.id === selected);
    this.base = camera?.base ?? '';
    this.cameraTitle.textContent = camera?.name ?? 'Camera';
    const key = cameras.map((c) => `${c.id}:${c.running}`).join('|') + `>${selected}`;
    if (key === this.tabsKey) {
      return;
    }
    this.tabsKey = key;
    this.cameraTabs.hidden = cameras.length < 2;
    fill(
      this.cameraTabs,
      cameras.map((c, i) =>
        h(
          'button',
          {
            type: 'button',
            class: `${c.id === selected ? 'active' : ''}${c.running ? '' : ' off'}`,
            title: c.running ? c.name : `${c.name} (not running)`,
            onclick: () => this.onCamera(c.id),
          },
          String(i + 1),
        ),
      ),
    );
  }

  /** Clears the image while another camera's stream starts. */
  resetVideo(): void {
    this.video.hidden = true;
    this.videoEmpty.hidden = false;
    this.videoFrame.textContent = '';
  }

  showVideo(frame: VideoFrame): void {
    if (this.videoUrl) {
      URL.revokeObjectURL(this.videoUrl);
    }
    this.videoUrl = URL.createObjectURL(frame.jpeg);
    this.video.src = this.videoUrl;
    this.video.hidden = false;
    this.videoEmpty.hidden = true;
    this.videoFrame.textContent = `frame ${frame.frame}`;
  }

  /** Road users and events as drawn (for the focused site, or all sites). */
  showWorld(objects: DrawnObject[], events: TaggedEvent[]): void {
    this.log.update(events);
    this.renderEvents();
    this.renderStats(objects, events);
  }

  /** Traffic lights read by the cameras: colour and how long it has shown. */
  showSignals(signals: { name: string; state: string; seconds: number }[]): void {
    this.signalsCard.hidden = signals.length === 0;
    if (signals.length === 0) {
      return;
    }
    const labels: Record<string, string> = {
      red: 'Red',
      yellow: 'Yellow',
      red_yellow: 'Red + yellow',
      green: 'Green',
      off: 'Off',
      unknown: 'Unknown',
    };
    fill(
      this.signals,
      signals.map((s) =>
        h(
          'li',
          {},
          h('span', { class: `signal-dot ${s.state}` }),
          h('span', { class: 'name' }, s.name),
          h('span', { class: 'state' }, labels[s.state] ?? s.state),
          h('span', { class: 'num muted' }, `${Math.round(s.seconds)} s`),
        ),
      ),
    );
  }

  /** Playback position of the shown camera. */
  showFrame(frame: number, timestampNs: number): void {
    if (!this.sliding) {
      this.slider.value = String(frame);
    }
    this.frameLabel.textContent = `${formatTimestamp(timestampNs)} · frame ${frame}`;
  }

  /** Status of the shown camera; `live` counts the cameras that run (of `total`). */
  showStatus(status: ServerStatus | null, live: number, total: number): void {
    const connected = status !== null;
    this.slider.max = String(Math.max((status?.frame_count ?? 1) - 1, 0));
    this.slider.disabled = !status?.seekable;
    this.playButton.disabled = !connected;
    if (status?.playing !== undefined) {
      this.setPlaying(status.playing);
    }
    this.offline.hidden = connected || total === 0;
    const source = status?.source || 'live';
    const label = { live: 'Live', oracle: 'Ground truth', replay: 'Replay' }[source] ?? source;
    const paused = status?.playing === false;
    fill(
      this.liveChip,
      h(
        'span',
        {
          class: `pill ${connected ? (paused ? 'warn' : 'ok live') : live > 0 ? 'warn' : 'danger'}`,
        },
        connected ? (paused ? 'Paused' : label) : live > 0 ? 'Camera off' : 'Offline',
      ),
      total > 1 ? h('span', { class: 'chip-metric num' }, `${live}/${total} cameras`) : null,
      connected && status.fps !== undefined
        ? h('span', { class: 'chip-metric num' }, `${status.fps.toFixed(1)} fps`)
        : null,
      connected && status.latency_ms !== undefined
        ? h('span', { class: 'chip-metric num' }, `${status.latency_ms.toFixed(0)} ms`)
        : null,
    );
  }

  private setPlaying(playing: boolean): void {
    this.playing = playing;
    fill(this.playButton, icon(playing ? 'pause' : 'play'));
    this.playButton.title = playing ? 'Pause' : 'Play';
  }

  private renderStats(objects: DrawnObject[], events: TaggedEvent[]): void {
    const vrus = objects.filter((o) => o.object.vru).length;
    const conflicts = events.filter(
      (e) => e.event.state === 'active' && e.event.type === 'VRU_VEHICLE_CONFLICT',
    ).length;
    const stat = (label: string, value: number, tone = '') =>
      h(
        'div',
        { class: `stat ${tone}` },
        h('strong', { class: 'num' }, String(value)),
        h('span', {}, label),
      );
    fill(
      this.stats,
      stat('Road users', objects.length),
      stat('Vulnerable', vrus, vrus > 0 ? 'vru' : ''),
      stat('Vehicles', objects.length - vrus),
      stat('Conflicts', conflicts, conflicts > 0 ? 'danger' : ''),
    );
    const counts = new Map<string, number>();
    for (const { object } of objects) {
      counts.set(object.class, (counts.get(object.class) ?? 0) + 1);
    }
    const order = Object.keys(CLASS_LABELS);
    const classes = [...counts.entries()].sort(([a], [b]) => order.indexOf(a) - order.indexOf(b));
    fill(
      this.legend,
      classes.length === 0
        ? h('span', { class: 'legend-empty' }, 'No road users')
        : classes.map(([name, count]) =>
            h(
              'span',
              { class: 'legend-item' },
              h('i', { style: `background:${classCss(name)}` }),
              CLASS_LABELS[name] ?? name,
              h('b', { class: 'num' }, String(count)),
            ),
          ),
    );
  }

  private renderEvents(): void {
    const entries = this.log.entries();
    const active = entries.filter((e) => e.isActive).length;
    this.eventCount.textContent = active > 0 ? `${active} active` : '';
    this.eventCount.hidden = active === 0;
    if (entries.length === 0) {
      fill(
        this.events,
        h(
          'li',
          { class: 'events-empty' },
          icon('check'),
          h('strong', {}, 'No safety events'),
          h('span', {}, 'VRUs on crosswalks and conflicts appear here.'),
        ),
      );
      return;
    }
    fill(
      this.events,
      entries.map(({ event, label, isActive }) => {
        const conflict = event.type === 'VRU_VEHICLE_CONFLICT';
        return h(
          'li',
          { class: `event ${conflict ? 'conflict' : 'crosswalk'}${isActive ? ' active' : ''}` },
          h('span', { class: 'event-icon' }, icon(conflict ? 'conflict' : 'walk')),
          h(
            'div',
            { class: 'event-body' },
            h('span', { class: 'event-title' }, EVENT_LABELS[event.type]),
            h(
              'span',
              { class: 'event-detail' },
              label ? `${label} · ${describeEvent(event)}` : describeEvent(event),
            ),
          ),
          h(
            'span',
            { class: `pill ${isActive ? (conflict ? 'danger live' : 'info live') : 'plain'}` },
            isActive ? 'Active' : 'Ended',
          ),
        );
      }),
    );
  }
}
