import './style.css';

import { fetchScene, fetchStatus, subscribeVideo, subscribeWorld } from './net/client';
import { TwinView } from './twin/twinView';
import { SidePanel } from './ui/sidePanel';

const STATUS_INTERVAL_MS = 1000;

async function start(): Promise<void> {
  const canvas = document.querySelector<HTMLCanvasElement>('#twin');
  if (!canvas) {
    throw new Error('canvas #twin not found');
  }
  const scene = await fetchScene();
  const view = new TwinView(canvas, scene);
  const panel = new SidePanel(scene.scene_id);

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
