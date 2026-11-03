/** Drag handles that resize a panel; the width is remembered per browser. */

export interface ResizeOptions {
  /** localStorage key of the width. */
  key: string;
  /** Width without a remembered one, and after a double click on the handle. */
  initial: number;
  min: number;
  /** Largest width, evaluated while dragging (e.g. a share of the window). */
  max: () => number;
  /** 1: the handle is on the right edge (dragging right widens), -1: on the left edge. */
  direction: 1 | -1;
  apply: (width: number) => void;
}

export function storedNumber(key: string): number | null {
  try {
    const value = Number.parseFloat(localStorage.getItem(key) ?? '');
    return Number.isFinite(value) ? value : null;
  } catch {
    return null;
  }
}

export function store(key: string, value: string): void {
  try {
    localStorage.setItem(key, value);
  } catch {
    // Not remembered in private windows; the size still changes.
  }
}

/** Makes `handle` resize a panel by dragging; returns the function that sets the width. */
export function makeResizable(
  handle: HTMLElement,
  options: ResizeOptions,
): (width: number) => void {
  const clamp = (width: number): number =>
    Math.round(Math.min(Math.max(width, options.min), Math.max(options.min, options.max())));
  // The width asked for (remembered or dragged) and the width shown, which fits the window.
  let wanted = storedNumber(options.key) ?? options.initial;
  let width = clamp(wanted);
  const set = (value: number): void => {
    wanted = value;
    width = clamp(value);
    options.apply(width);
  };
  set(wanted);

  let start: { x: number; width: number } | null = null;
  handle.addEventListener('pointerdown', (e) => {
    if (e.button !== 0) {
      return;
    }
    e.preventDefault();
    start = { x: e.clientX, width };
    handle.setPointerCapture(e.pointerId);
    document.body.classList.add('resizing');
  });
  handle.addEventListener('pointermove', (e) => {
    if (start) {
      set(start.width + options.direction * (e.clientX - start.x));
    }
  });
  const end = (): void => {
    if (!start) {
      return;
    }
    start = null;
    document.body.classList.remove('resizing');
    store(options.key, String(width));
  };
  handle.addEventListener('pointerup', end);
  handle.addEventListener('pointercancel', end);
  handle.addEventListener('dblclick', () => {
    set(options.initial);
    store(options.key, String(width));
  });
  // The window may no longer fit the width asked for, or fit it again.
  window.addEventListener('resize', () => {
    width = clamp(wanted);
    options.apply(width);
  });
  return set;
}
