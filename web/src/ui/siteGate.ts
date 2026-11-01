import {
  type Camera,
  cameraAction,
  cameraState,
  findCamera,
  getSite,
  type Site,
} from '../net/manager';
import { fill, h } from './dom';
import { withBusy } from './feedback';
import { icon } from './icons';
import { links } from './shell';

const WAIT_INTERVAL_MS = 2000;

/**
 * Shown in `content` while a camera of a managed site is not running; resolves with the site and
 * the camera once it runs (the given camera, else the site's first). Pages that talk to a
 * camera's server (calibration) wait here.
 */
export async function waitForCamera(
  content: HTMLElement,
  siteId: string,
  cameraId: string | null,
  purpose: string,
): Promise<{ site: Site; camera: Camera }> {
  const notice = h('div', { class: 'site-waiting' });
  content.append(notice);
  const stop = (title: string, text: string): Promise<never> => {
    fill(
      notice,
      h(
        'div',
        { class: 'empty' },
        icon('alert'),
        h('h3', {}, title),
        h('p', {}, text),
        h(
          'div',
          { class: 'row' },
          h('a', { class: 'btn', href: links.settings(siteId) }, icon('settings'), 'Site settings'),
          h('a', { class: 'btn ghost', href: '/' }, 'Overview'),
        ),
      ),
    );
    return new Promise<never>(() => undefined);
  };
  for (;;) {
    let site: Site;
    try {
      site = await getSite(siteId);
    } catch (error) {
      return stop('Site not found', (error as Error).message);
    }
    const camera = findCamera(site, cameraId);
    if (!camera) {
      return stop(`${site.name} has no camera yet`, 'Add a camera in the site settings first.');
    }
    if (camera.runtime.state === 'running') {
      notice.remove();
      return { site, camera };
    }
    const state = cameraState(site, camera);
    const startButton = h(
      'button',
      { type: 'button', class: 'btn primary' },
      icon('power'),
      'Start camera',
    );
    startButton.addEventListener(
      'click',
      () =>
        void withBusy(
          startButton,
          () => cameraAction(siteId, camera.id, 'start'),
          'Starting the camera…',
        ),
    );
    const busy =
      ['starting', 'stopping', 'backoff'].includes(camera.runtime.state) ||
      (site.enabled && camera.enabled && camera.runtime.state === 'stopped');
    const label = `${site.name} · ${camera.name}`;
    fill(
      notice,
      h(
        'div',
        { class: 'empty' },
        busy ? h('span', { class: 'spinner big' }) : icon('power'),
        h('h3', {}, busy ? `${label} is starting` : `${label} is ${state.label.toLowerCase()}`),
        h(
          'p',
          {},
          camera.runtime.error ??
            (busy
              ? 'The perception server loads the detector and opens the camera source. This page continues automatically.'
              : `Start the camera to run its perception server and ${purpose}.`),
        ),
        h(
          'div',
          { class: 'row' },
          busy ? null : startButton,
          h('a', { class: 'btn', href: links.settings(siteId) }, icon('settings'), 'Settings'),
          h(
            'a',
            { class: 'btn ghost', href: links.logs(`${siteId}/${camera.id}`) },
            icon('logs'),
            'Logs',
          ),
        ),
      ),
    );
    await new Promise((resolve) => setTimeout(resolve, WAIT_INTERVAL_MS));
  }
}
