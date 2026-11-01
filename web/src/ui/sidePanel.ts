import { sendPlayback, type ServerStatus, type VideoFrame } from '../net/client';
import type { WorldObject, WorldState } from '../net/schema';
import { CLASS_LABELS, classCss } from '../twin/palette';
import { element, fill, h } from './dom';
import { describeEvent, EVENT_LABELS, EventLog, formatTimestamp } from './events';
import { icon } from './icons';

/**
 * Panels of the twin page: camera image, counters, class legend, event list, playback bar and
 * the live status chip.
 */
export class SidePanel {
  private readonly video = element<HTMLImageElement>('#video');
  private readonly videoFrame = element<HTMLElement>('#video-frame');
  private readonly videoEmpty = element<HTMLElement>('#video-empty');
  private readonly events = element<HTMLUListElement>('#events');
  private readonly eventCount = element<HTMLElement>('#event-count');
  private readonly stats = element<HTMLElement>('#stats');
  private readonly legend = element<HTMLElement>('#legend');
  private readonly liveChip = element<HTMLElement>('#live-chip');
  private readonly playButton = element<HTMLButtonElement>('#play');
  private readonly slider = element<HTMLInputElement>('#seek');
  private readonly frameLabel = element<HTMLElement>('#frame');
  private readonly offline = element<HTMLElement>('#offline');
  private readonly log = new EventLog();
  private videoUrl: string | null = null;
  private playing = true;
  private sliding = false;
  private lastFrame = -1;
  private latest: WorldState | null = null;

  constructor() {
    this.playButton.addEventListener('click', () => {
      void sendPlayback({ action: this.playing ? 'pause' : 'play' });
      this.setPlaying(!this.playing);
    });
    this.slider.addEventListener('input', () => (this.sliding = true));
    this.slider.addEventListener('change', () => {
      this.sliding = false;
      this.log.clear();
      void sendPlayback({ action: 'seek', frame: Number(this.slider.value) });
    });
    this.renderEvents();
  }

  object(id: number): WorldObject | undefined {
    return this.latest?.objects.find((o) => o.id === id);
  }

  showState(state: WorldState): void {
    if (state.frame < this.lastFrame) {
      this.log.clear();
    }
    this.latest = state;
    this.lastFrame = state.frame;
    this.log.update(state);
    this.renderEvents();
    this.renderStats(state);
    if (!this.sliding) {
      this.slider.value = String(state.frame);
    }
    this.frameLabel.textContent = `${formatTimestamp(state.timestamp_ns)} · frame ${state.frame}`;
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

  showStatus(status: ServerStatus, connected: boolean): void {
    this.slider.max = String(Math.max(status.frame_count - 1, 0));
    this.slider.disabled = !status.seekable;
    if (status.playing !== undefined) {
      this.setPlaying(status.playing);
    }
    this.offline.hidden = connected;
    const source = status.source || 'live';
    const label = { live: 'Live', oracle: 'Ground truth', replay: 'Replay' }[source] ?? source;
    fill(
      this.liveChip,
      h(
        'span',
        { class: `pill ${connected ? (status.playing === false ? 'warn' : 'ok live') : 'danger'}` },
        connected ? (status.playing === false ? 'Paused' : label) : 'Offline',
      ),
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

  private renderStats(state: WorldState): void {
    const vrus = state.objects.filter((o) => o.vru).length;
    const active = state.events.filter((e) => e.state === 'active');
    const conflicts = active.filter((e) => e.type === 'VRU_VEHICLE_CONFLICT').length;
    const stat = (label: string, value: number, tone = '') =>
      h(
        'div',
        { class: `stat ${tone}` },
        h('strong', { class: 'num' }, String(value)),
        h('span', {}, label),
      );
    fill(
      this.stats,
      stat('Road users', state.objects.length),
      stat('Vulnerable', vrus, vrus > 0 ? 'vru' : ''),
      stat('Vehicles', state.objects.length - vrus),
      stat('Conflicts', conflicts, conflicts > 0 ? 'danger' : ''),
    );
    const counts = new Map<string, number>();
    for (const o of state.objects) {
      counts.set(o.class, (counts.get(o.class) ?? 0) + 1);
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
    const active = entries.filter((e) => e.active).length;
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
      entries.map(({ event, active: isActive }) => {
        const conflict = event.type === 'VRU_VEHICLE_CONFLICT';
        return h(
          'li',
          { class: `event ${conflict ? 'conflict' : 'crosswalk'}${isActive ? ' active' : ''}` },
          h('span', { class: 'event-icon' }, icon(conflict ? 'conflict' : 'walk')),
          h(
            'div',
            { class: 'event-body' },
            h('span', { class: 'event-title' }, EVENT_LABELS[event.type]),
            h('span', { class: 'event-detail' }, describeEvent(event)),
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
