/** Parses "48.137154, 11.576124" (as copied from a map) into latitude and longitude. */
export function parseLatLon(text: string): [number, number] | null {
  const match = /^\s*(-?\d+(?:\.\d+)?)\s*[,;\s]\s*(-?\d+(?:\.\d+)?)\s*$/.exec(text);
  if (!match) {
    return null;
  }
  const lat = Number(match[1]);
  const lon = Number(match[2]);
  return Math.abs(lat) <= 90 && Math.abs(lon) <= 180 ? [lat, lon] : null;
}

/** Latitude and longitude inputs that also take a pasted "lat, lon" pair. */
export function latLonInputs(lat: HTMLInputElement, lon: HTMLInputElement): void {
  for (const input of [lat, lon]) {
    input.addEventListener('paste', (e) => {
      const pair = parseLatLon(e.clipboardData?.getData('text') ?? '');
      if (pair) {
        e.preventDefault();
        lat.value = String(pair[0]);
        lon.value = String(pair[1]);
        lat.dispatchEvent(new Event('input', { bubbles: true }));
      }
    });
  }
}
