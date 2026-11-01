/** Line icons (24 x 24, stroked with the current colour). */
const PATHS = {
  overview:
    '<rect x="3" y="3" width="7.5" height="7.5" rx="1.5"/><rect x="13.5" y="3" width="7.5" height="7.5" rx="1.5"/><rect x="3" y="13.5" width="7.5" height="7.5" rx="1.5"/><rect x="13.5" y="13.5" width="7.5" height="7.5" rx="1.5"/>',
  twin: '<path d="M12 2.8 20.5 7.5v9L12 21.2 3.5 16.5v-9z"/><path d="M3.5 7.5 12 12.2l8.5-4.7M12 12.2v9"/>',
  calibration:
    '<circle cx="12" cy="12" r="8.5"/><circle cx="12" cy="12" r="2.5"/><path d="M12 1.5v4M12 18.5v4M1.5 12h4M18.5 12h4"/>',
  settings:
    '<path d="M4 6.5h9M17 6.5h3M4 12h3M11 12h9M4 17.5h11M19 17.5h1"/><circle cx="15" cy="6.5" r="2"/><circle cx="9" cy="12" r="2"/><circle cx="17" cy="17.5" r="2"/>',
  plus: '<path d="M12 5v14M5 12h14"/>',
  server:
    '<rect x="3" y="3.5" width="18" height="7" rx="2"/><rect x="3" y="13.5" width="18" height="7" rx="2"/><path d="M7 7h.01M7 17h.01M11 7h6M11 17h6"/>',
  logs: '<path d="m4.5 16.5 5-4.5-5-4.5"/><path d="M12 18.5h7.5"/>',
  play: '<path d="M7 4.5v15l12-7.5z"/>',
  pause: '<path d="M8.5 5v14M15.5 5v14"/>',
  power: '<path d="M12 3v8.5"/><path d="M6.3 6.6a8 8 0 1 0 11.4 0"/>',
  restart: '<path d="M20 12a8 8 0 1 1-2.6-5.9"/><path d="M20 4v4.5h-4.5"/>',
  stop: '<rect x="6" y="6" width="12" height="12" rx="2"/>',
  trash: '<path d="M4 7h16M10 11v6M14 11v6M6 7l1 13h10l1-13M9 7V4h6v3"/>',
  video:
    '<rect x="2.5" y="6" width="13.5" height="12" rx="2.5"/><path d="m16 10.5 5.5-3v9l-5.5-3z"/>',
  stream:
    '<path d="M5 12a7 7 0 0 1 14 0M8 12a4 4 0 0 1 8 0"/><circle cx="12" cy="12" r="1"/><path d="M12 13v8"/>',
  database:
    '<ellipse cx="12" cy="5.5" rx="8" ry="3"/><path d="M4 5.5v13c0 1.7 3.6 3 8 3s8-1.3 8-3v-13M4 12c0 1.7 3.6 3 8 3s8-1.3 8-3"/>',
  replay:
    '<path d="M4 12a8 8 0 1 0 2.4-5.7L4 8.5"/><path d="M4 4v4.5h4.5"/><path d="M12 8v4l3 2"/>',
  pin: '<path d="M12 21s7-6.2 7-11.5a7 7 0 0 0-14 0C5 14.8 12 21 12 21z"/><circle cx="12" cy="9.5" r="2.5"/>',
  layers: '<path d="m12 3 9 5-9 5-9-5z"/><path d="m3 13 9 5 9-5"/>',
  building:
    '<path d="M4 21V5l8-3v19M12 21V9l8 3v9M2 21h20M7 8h1M7 12h1M7 16h1M15 14h1M15 17.5h1"/>',
  check: '<path d="m5 12.5 4.5 4.5L19 7.5"/>',
  alert: '<path d="M12 3.5 21.5 20h-19z"/><path d="M12 10v4.5M12 17.5h.01"/>',
  info: '<circle cx="12" cy="12" r="9"/><path d="M12 11v5.5M12 7.5h.01"/>',
  close: '<path d="M6 6l12 12M18 6 6 18"/>',
  activity: '<path d="M3 12h4l3 7.5 4-15 3 7.5h4"/>',
  cpu: '<rect x="6" y="6" width="12" height="12" rx="2"/><rect x="9.5" y="9.5" width="5" height="5" rx="1"/><path d="M9 2.5V6M15 2.5V6M9 18v3.5M15 18v3.5M2.5 9H6M2.5 15H6M18 9h3.5M18 15h3.5"/>',
  gpu: '<rect x="2.5" y="6" width="19" height="11" rx="2"/><circle cx="9" cy="11.5" r="2.5"/><path d="M15 9.5h3M15 13.5h3M5 17v3M9 17v3"/>',
  memory:
    '<rect x="3" y="7" width="18" height="10" rx="1.5"/><path d="M7 7v10M11 7v10M15 7v10M5 17v3M19 17v3"/>',
  disk: '<rect x="3" y="4" width="18" height="16" rx="2"/><circle cx="12" cy="12" r="4"/><path d="M12 12h.01"/>',
  download: '<path d="M12 4v11M7 10.5l5 5 5-5M5 20h14"/>',
  search: '<circle cx="11" cy="11" r="6.5"/><path d="m20 20-4.3-4.3"/>',
  chevron: '<path d="m9 6 6 6-6 6"/>',
  back: '<path d="M15 6l-6 6 6 6"/>',
  external:
    '<path d="M14 4h6v6M20 4l-9 9M18 14v5a1 1 0 0 1-1 1H5a1 1 0 0 1-1-1V7a1 1 0 0 1 1-1h5"/>',
  recenter: '<path d="M4 9V4h5M20 9V4h-5M4 15v5h5M20 15v5h-5"/><circle cx="12" cy="12" r="2.5"/>',
  route:
    '<circle cx="6" cy="19" r="2.5"/><circle cx="18" cy="5" r="2.5"/><path d="M8.5 19H16a3.5 3.5 0 0 0 0-7H8a3.5 3.5 0 0 1 0-7h7.5"/>',
  walk: '<circle cx="13" cy="4" r="1.8"/><path d="m9 21 2.5-6.5L14 17v4M7 10.5l3-3.5 4 1.5 2 3.5 2.5 1M11.5 14.5 12.5 9"/>',
  conflict: '<path d="M12 3.5 21.5 20h-19z"/><path d="M12 10v4.5M12 17.5h.01"/>',
  pulse: '<circle cx="12" cy="12" r="3"/><circle cx="12" cy="12" r="8" opacity=".45"/>',
  map: '<path d="m3 6.5 6-3 6 3 6-3v14l-6 3-6-3-6 3z"/><path d="M9 3.5v14M15 6.5v14"/>',
  image:
    '<rect x="3" y="4" width="18" height="16" rx="2.5"/><circle cx="9" cy="10" r="2"/><path d="m21 16-5-5-9 9"/>',
  clock: '<circle cx="12" cy="12" r="9"/><path d="M12 7v5l3.5 2"/>',
  wizard:
    '<path d="m4 20 11-11M14 4l1 2 2 1-2 1-1 2-1-2-2-1 2-1zM19 11l.6 1.4L21 13l-1.4.6L19 15l-.6-1.4L17 13l1.4-.6z"/>',
} as const;

export type IconName = keyof typeof PATHS;

export function icon(name: IconName, className = ''): HTMLSpanElement {
  const span = document.createElement('span');
  span.className = `icon ${className}`.trim();
  span.innerHTML = `<svg viewBox="0 0 24 24" aria-hidden="true">${PATHS[name]}</svg>`;
  return span;
}

/** Product mark: a junction (two crossing roads) in a rounded tile. */
export function brandMark(): HTMLSpanElement {
  const span = document.createElement('span');
  span.className = 'brand-mark';
  span.innerHTML =
    '<svg viewBox="0 0 24 24" aria-hidden="true"><path d="M9 2.5h6v6.5h6.5v6H15v6.5H9V15H2.5V9H9z" fill="#fff" opacity=".95"/><path d="M12 4v3M12 17v3M4 12h3M17 12h3" stroke="#2683f0" stroke-width="1.4" stroke-linecap="round"/><circle cx="12" cy="12" r="2" fill="#2683f0"/></svg>';
  return span;
}
