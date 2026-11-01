import { getSite, type Site, siteAction, siteState } from '../net/manager';
import { fill, h } from './dom';
import { withBusy } from './feedback';
import { icon } from './icons';
import { links } from './shell';

const WAIT_INTERVAL_MS = 2000;

/**
 * Shown in `content` while a managed site is not running; resolves once it runs. Pages that talk
 * to the site's server (twin, calibration) wait here.
 */
export async function waitForSite(
  content: HTMLElement,
  siteId: string,
  purpose: string,
): Promise<void> {
  const notice = h('div', { class: 'site-waiting' });
  content.append(notice);
  for (;;) {
    let site: Site;
    try {
      site = await getSite(siteId);
    } catch (error) {
      fill(
        notice,
        h(
          'div',
          { class: 'empty' },
          icon('alert'),
          h('h3', {}, 'Site not found'),
          h('p', {}, (error as Error).message),
          h('a', { class: 'btn', href: '/' }, 'Back to overview'),
        ),
      );
      return new Promise<void>(() => undefined);
    }
    if (site.runtime.state === 'running') {
      notice.remove();
      return;
    }
    const state = siteState(site);
    const startButton = h(
      'button',
      { type: 'button', class: 'btn primary' },
      icon('power'),
      'Activate site',
    );
    startButton.addEventListener(
      'click',
      () => void withBusy(startButton, () => siteAction(siteId, 'start'), 'Starting the site…'),
    );
    const busy =
      ['starting', 'stopping', 'backoff'].includes(site.runtime.state) ||
      (site.enabled && site.runtime.state === 'stopped');
    fill(
      notice,
      h(
        'div',
        { class: 'empty' },
        busy ? h('span', { class: 'spinner big' }) : icon('power'),
        h(
          'h3',
          {},
          busy ? `${site.name} is starting` : `${site.name} is ${state.label.toLowerCase()}`,
        ),
        h(
          'p',
          {},
          site.runtime.error ??
            (busy
              ? 'The perception server loads the detector and opens the camera source. This page continues automatically.'
              : `Activate the site to start its perception server and ${purpose}.`),
        ),
        h(
          'div',
          { class: 'row' },
          busy ? null : startButton,
          h('a', { class: 'btn', href: links.settings(siteId) }, icon('settings'), 'Settings'),
          h('a', { class: 'btn ghost', href: links.logs(siteId) }, icon('logs'), 'Logs'),
        ),
      ),
    );
    await new Promise((resolve) => setTimeout(resolve, WAIT_INTERVAL_MS));
  }
}
