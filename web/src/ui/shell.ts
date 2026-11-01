import '../styles/theme.css';
import '../styles/shell.css';

import { listSites, managerAvailable, type Site, siteState, systemInfo } from '../net/manager';
import { type Child, fill, h } from './dom';
import { brandMark, icon, type IconName } from './icons';
import { currentTheme, setTheme } from './theme';

export type Page =
  | 'overview'
  | 'twin'
  | 'calibration'
  | 'settings'
  | 'new-site'
  | 'system'
  | 'logs';

export interface ShellOptions {
  page: Page;
  siteId?: string | null;
  /** Last part of the breadcrumb. */
  title: string;
  /** Content without padding or scrolling (twin, calibration). */
  fullBleed?: boolean;
}

export interface Shell {
  content: HTMLElement;
  /** Whether the console runs behind the site manager (false: a single jp_server). */
  managed: boolean;
  setActions(...actions: Child[]): void;
  setTitle(title: string): void;
  /** Latest site list; listeners are called on every refresh. */
  sites(): Site[];
  onSites(listener: (sites: Site[]) => void): void;
  refresh(): Promise<void>;
}

const REFRESH_MS = 3000;

export const links = {
  overview: '/',
  twin: (id: string) => `/twin.html?site=${encodeURIComponent(id)}`,
  calibration: (id: string) => `/calibrate.html?site=${encodeURIComponent(id)}`,
  settings: (id: string) => `/site.html?site=${encodeURIComponent(id)}`,
  newSite: '/new-site.html',
  system: '/system.html',
  logs: (source?: string) =>
    source ? `/logs.html?source=${encodeURIComponent(source)}` : '/logs.html',
};

function navItem(href: string, name: IconName, label: string, active: boolean, extra?: Child) {
  return h(
    'a',
    { class: `nav-item${active ? ' active' : ''}`, href, title: label },
    icon(name),
    h('span', { class: 'name' }, label),
    extra ?? null,
  );
}

/** Builds the page frame and returns its content area. */
export async function mountShell(options: ShellOptions): Promise<Shell> {
  document.body.classList.add('console');
  const managed = await managerAvailable();
  const nav = h('nav', { class: 'nav' });
  const footer = h('div', { class: 'sidebar-footer' });
  const title = h('strong', {}, options.title);
  const actions = h('div', { class: 'topbar-actions' });
  const content = h('main', { class: `content${options.fullBleed ? ' full' : ''}` });
  const crumbs = h('div', { class: 'crumbs' });
  const themeButton = h('button', {
    type: 'button',
    class: 'btn ghost small icon-only theme-toggle',
  });
  const renderThemeButton = (): void => {
    const light = currentTheme() === 'light';
    fill(themeButton, icon(light ? 'moon' : 'sun'));
    themeButton.title = light ? 'Night mode' : 'Day mode';
  };
  themeButton.addEventListener('click', () => {
    setTheme(currentTheme() === 'light' ? 'dark' : 'light');
    renderThemeButton();
  });
  renderThemeButton();

  const app = h(
    'div',
    { class: 'app' },
    h(
      'aside',
      { class: 'sidebar' },
      h(
        'a',
        { class: 'brand', href: '/' },
        brandMark(),
        h(
          'span',
          { class: 'brand-name' },
          h('strong', {}, 'JunctionPerception'),
          h('span', {}, 'Digital twin console'),
        ),
      ),
      nav,
      footer,
    ),
    h(
      'div',
      { class: 'main' },
      h('header', { class: 'topbar' }, crumbs, actions, themeButton),
      content,
    ),
  );
  document.body.replaceChildren(app);

  let sites: Site[] = [];
  const listeners: ((sites: Site[]) => void)[] = [];

  const renderCrumbs = (): void => {
    const site = sites.find((s) => s.id === options.siteId);
    const parts: Child[] = [];
    if (options.siteId) {
      parts.push(
        h('a', { href: '/' }, 'Sites'),
        icon('chevron'),
        h('a', { href: links.twin(options.siteId) }, site?.name ?? options.siteId),
        icon('chevron'),
      );
    }
    parts.push(title);
    fill(crumbs, parts);
  };

  const renderNav = (): void => {
    const p = options.page;
    const siteItems: Child[] = sites.flatMap((site) => {
      const state = siteState(site);
      const open = site.id === options.siteId;
      const row = h(
        'a',
        {
          class: `nav-item nav-site${open ? ' open' : ''}`,
          href: links.twin(site.id),
          title: `${site.name} · ${state.label}`,
        },
        h('span', { class: `dot ${state.tone}` }),
        h('span', { class: 'name' }, site.name),
        site.runtime.status?.objects !== undefined && site.runtime.state === 'running'
          ? h('span', { class: 'count num' }, String(site.runtime.status.objects))
          : null,
      );
      if (!open) {
        return row;
      }
      return [
        row,
        h(
          'div',
          { class: 'nav-sub' },
          navItem(links.twin(site.id), 'twin', 'Live twin', p === 'twin'),
          navItem(links.calibration(site.id), 'calibration', 'Calibration', p === 'calibration'),
          navItem(links.settings(site.id), 'settings', 'Settings', p === 'settings'),
        ),
      ];
    });
    if (!managed) {
      fill(
        nav,
        h('div', { class: 'nav-label' }, 'This server'),
        navItem('/twin.html', 'twin', 'Live twin', p === 'twin'),
        navItem('/calibrate.html', 'calibration', 'Calibration', p === 'calibration'),
      );
      return;
    }
    fill(
      nav,
      h('div', { class: 'nav-label' }, 'Monitor'),
      navItem('/', 'overview', 'Overview', p === 'overview'),
      navItem(links.logs(), 'logs', 'Logs', p === 'logs'),
      h(
        'div',
        { class: 'nav-label' },
        'Sites',
        h('a', { href: links.newSite, title: 'New site' }, icon('plus')),
      ),
      siteItems.length > 0 ? siteItems : h('div', { class: 'nav-empty' }, 'No sites yet'),
      navItem(links.newSite, 'plus', 'New site', p === 'new-site'),
      h('div', { class: 'nav-label' }, 'Administration'),
      navItem(links.system, 'server', 'System', p === 'system'),
    );
  };

  const renderFooter = async (): Promise<void> => {
    if (!managed) {
      fill(
        footer,
        h(
          'div',
          { class: 'backend-status' },
          h('span', { class: 'dot ok' }),
          h('div', {}, h('strong', {}, 'Single server'), h('span', {}, 'without site manager')),
        ),
      );
      return;
    }
    try {
      const system = await systemInfo();
      const tone = !system.backend_on ? 'danger' : system.sites_running > 0 ? 'ok' : 'warn';
      fill(
        footer,
        h(
          'a',
          { class: 'backend-status', href: links.system },
          h('span', { class: `dot ${tone}` }),
          h(
            'div',
            {},
            h('strong', {}, system.backend_on ? 'Backend running' : 'Backend stopped'),
            h(
              'span',
              {},
              `${system.sites_running}/${system.sites_enabled} sites · ${system.version}`,
            ),
          ),
          icon('chevron'),
        ),
      );
    } catch {
      fill(
        footer,
        h(
          'div',
          { class: 'backend-status' },
          h('span', { class: 'dot danger' }),
          h('div', {}, h('strong', {}, 'Console offline'), h('span', {}, 'reconnecting…')),
        ),
      );
    }
  };

  const refresh = async (): Promise<void> => {
    if (managed) {
      try {
        sites = await listSites();
      } catch {
        // Keep the last list while the manager restarts.
      }
    }
    renderNav();
    renderCrumbs();
    listeners.forEach((listener) => listener(sites));
  };

  await refresh();
  void renderFooter();
  setInterval(() => {
    void refresh();
    void renderFooter();
  }, REFRESH_MS);

  return {
    content,
    managed,
    setActions: (...nodes) => fill(actions, ...nodes),
    setTitle: (text) => {
      title.textContent = text;
      document.title = `${text} · JunctionPerception`;
    },
    sites: () => sites,
    onSites: (listener) => listeners.push(listener),
    refresh,
  };
}
