/** Number and time formatting for the console. */

export function formatBytes(bytes: number | null | undefined): string {
  if (bytes === null || bytes === undefined) {
    return '–';
  }
  const units = ['B', 'KB', 'MB', 'GB', 'TB'];
  let value = bytes;
  let unit = 0;
  while (value >= 1024 && unit < units.length - 1) {
    value /= 1024;
    unit++;
  }
  return `${value.toFixed(value >= 100 || unit === 0 ? 0 : 1)} ${units[unit]}`;
}

/** "3 d 4 h", "2 h 05 min", "4 min 12 s", "9 s". */
export function formatDuration(seconds: number | null | undefined): string {
  if (seconds === null || seconds === undefined || !Number.isFinite(seconds)) {
    return '–';
  }
  const s = Math.max(0, Math.floor(seconds));
  const days = Math.floor(s / 86400);
  const hours = Math.floor((s % 86400) / 3600);
  const minutes = Math.floor((s % 3600) / 60);
  if (days > 0) {
    return `${days} d ${hours} h`;
  }
  if (hours > 0) {
    return `${hours} h ${String(minutes).padStart(2, '0')} min`;
  }
  if (minutes > 0) {
    return `${minutes} min ${s % 60} s`;
  }
  return `${s} s`;
}

export function formatNumber(value: number | null | undefined, digits = 0, unit = ''): string {
  if (value === null || value === undefined || !Number.isFinite(value)) {
    return '–';
  }
  return `${value.toFixed(digits)}${unit ? ` ${unit}` : ''}`;
}

/** Clock time of a Unix timestamp in seconds, with milliseconds: "14:03:27.512". */
export function formatClock(unixSeconds: number): string {
  const date = new Date(unixSeconds * 1000);
  const pad = (n: number, width = 2): string => String(n).padStart(width, '0');
  return `${pad(date.getHours())}:${pad(date.getMinutes())}:${pad(date.getSeconds())}.${pad(date.getMilliseconds(), 3)}`;
}

/** Lowercase identifier from a display name: "Munich, Lerchenauer Str." -> "munich-lerchenauer-str". */
export function slugify(name: string): string {
  return name
    .normalize('NFKD')
    .replace(/[̀-ͯ]/g, '')
    .toLowerCase()
    .replace(/ß/g, 'ss')
    .replace(/[^a-z0-9]+/g, '-')
    .replace(/^-+|-+$/g, '')
    .slice(0, 48);
}
