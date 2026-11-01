/** Night (dark) and day (light) theme of the console, remembered per browser. */
import type { Theme } from '../twin/palette';

const STORAGE_KEY = 'jp-theme';
const EVENT = 'jp-theme';

export function currentTheme(): Theme {
  try {
    return localStorage.getItem(STORAGE_KEY) === 'light' ? 'light' : 'dark';
  } catch {
    return 'dark';
  }
}

function apply(theme: Theme): void {
  document.documentElement.dataset['theme'] = theme;
  document
    .querySelector('meta[name="theme-color"]')
    ?.setAttribute('content', theme === 'light' ? '#f4f6f9' : '#080d17');
}

export function setTheme(theme: Theme): void {
  try {
    localStorage.setItem(STORAGE_KEY, theme);
  } catch {
    // Not remembered in private windows; the page still switches.
  }
  apply(theme);
  window.dispatchEvent(new CustomEvent<Theme>(EVENT, { detail: theme }));
}

export function onThemeChange(listener: (theme: Theme) => void): void {
  window.addEventListener(EVENT, (e) => listener((e as CustomEvent<Theme>).detail));
}

// Applied as soon as the module loads, before the page is built.
apply(currentTheme());
