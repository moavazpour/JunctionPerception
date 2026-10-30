import { sendPlayback, type ServerStatus, type VideoFrame } from '../net/client';
import type { WorldState } from '../net/schema';
import { describeEvent, EVENT_LABELS, EventLog, formatTimestamp } from './events';

function element<T extends HTMLElement>(selector: string): T {
  const found = document.querySelector<T>(selector);
  if (!found) {
    throw new Error(`missing element ${selector}`);
  }
  return found;
}

/** Side panel: camera image, event list, status line and playback controls. */
export class SidePanel {
  private readonly video = element<HTMLImageElement>('#video');
  private readonly videoFrame = element<HTMLElement>('#video-frame');
  private readonly events = element<HTMLUListElement>('#events');
  private readonly status = element<HTMLElement>('#status');
  private readonly playButton = element<HTMLButtonElement>('#play');
  private readonly slider = element<HTMLInputElement>('#seek');
  private readonly frameLabel = element<HTMLElement>('#frame');
  private readonly log = new EventLog();
  private videoUrl: string | null = null;
  private playing = true;
  private sliding = false;
  private lastFrame = -1;

  constructor(sceneId: string) {
    element<HTMLElement>('#scene-id').textContent = sceneId;
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
  }

  showState(state: WorldState): void {
    if (state.frame < this.lastFrame) {
      this.log.clear();
    }
    this.lastFrame = state.frame;
    this.log.update(state);
    this.renderEvents();
    if (!this.sliding) {
      this.slider.value = String(state.frame);
    }
    this.frameLabel.textContent = `frame ${state.frame} · ${formatTimestamp(state.timestamp_ns)} · ${state.objects.length} objects`;
  }

  showVideo(frame: VideoFrame): void {
    if (this.videoUrl) {
      URL.revokeObjectURL(this.videoUrl);
    }
    this.videoUrl = URL.createObjectURL(frame.jpeg);
    this.video.src = this.videoUrl;
    this.videoFrame.textContent = `camera · frame ${frame.frame}`;
  }

  showStatus(status: ServerStatus, connected: boolean): void {
    this.slider.max = String(Math.max(status.frame_count - 1, 0));
    this.slider.disabled = !status.seekable;
    if (status.playing !== undefined) {
      this.setPlaying(status.playing);
    }
    const fps = status.fps !== undefined ? `${status.fps.toFixed(1)} fps` : '– fps';
    const latency =
      status.latency_ms !== undefined ? `${status.latency_ms.toFixed(0)} ms latency` : '';
    this.status.textContent = connected
      ? [status.source, fps, latency].filter(Boolean).join(' · ')
      : 'disconnected';
    this.status.classList.toggle('offline', !connected);
  }

  private setPlaying(playing: boolean): void {
    this.playing = playing;
    this.playButton.textContent = playing ? 'Pause' : 'Play';
  }

  private renderEvents(): void {
    const entries = this.log.entries();
    this.events.replaceChildren(
      ...entries.map(({ event, active }) => {
        const item = document.createElement('li');
        item.className = `event ${event.type === 'VRU_VEHICLE_CONFLICT' ? 'conflict' : 'crosswalk'}${
          active ? ' active' : ''
        }`;
        const title = document.createElement('span');
        title.className = 'event-title';
        title.textContent = EVENT_LABELS[event.type];
        const detail = document.createElement('span');
        detail.className = 'event-detail';
        detail.textContent = describeEvent(event);
        item.append(title, detail);
        return item;
      }),
    );
    if (entries.length === 0) {
      const empty = document.createElement('li');
      empty.className = 'event empty';
      empty.textContent = 'No events';
      this.events.append(empty);
    }
  }
}
