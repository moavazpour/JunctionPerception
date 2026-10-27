// Site manager REST API (multi-site demo stack). Absent in the single-camera stack: the panel then hides
// site management and source changes.

export interface SiteInfo {
  id: string;
  name: string;
  source: string;
  kind: "video" | "stream";
  status: string;
  calibrated: boolean;
  device: "auto" | "gpu" | "cpu";
  loop: boolean;
  lens: string;
  position: { lat: number; lon: number; height_m: number; bearing_deg: number };
}

export interface ManagerHealth {
  ok: boolean;
  gpu: boolean;
  detector: string;
  sites: number;
  max_sites: number;
}

export async function api<T = any>(path: string, init?: RequestInit): Promise<T> {
  const r = await fetch(`/api${path}`, init);
  const body = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(body.detail ? String(typeof body.detail === "string" ? body.detail : JSON.stringify(body.detail)) : `${r.status} ${r.statusText}`);
  return body as T;
}

export const json = (method: string, body: unknown): RequestInit => ({
  method,
  headers: { "Content-Type": "application/json" },
  body: JSON.stringify(body),
});

export async function managerHealth(): Promise<ManagerHealth | null> {
  try {
    return await api<ManagerHealth>("/health");
  } catch {
    return null;
  }
}

export function uploadVideo(file: File, onProgress: (pct: number) => void): Promise<string> {
  return new Promise((res, rej) => {
    const fd = new FormData();
    fd.append("file", file);
    const x = new XMLHttpRequest();
    x.open("POST", "/api/videos");
    x.upload.onprogress = (e) => e.lengthComputable && onProgress(Math.round((100 * e.loaded) / e.total));
    x.onload = () => {
      try {
        const b = JSON.parse(x.responseText);
        x.status < 300 ? res(b.video) : rej(new Error(b.detail ?? x.statusText));
      } catch {
        rej(new Error(x.statusText));
      }
    };
    x.onerror = () => rej(new Error("upload failed"));
    x.send(fd);
  });
}

// Address search (OpenStreetMap Nominatim).
export async function geocode(q: string): Promise<{ lat: number; lon: number; name: string } | null> {
  const r = await fetch(`https://nominatim.openstreetmap.org/search?format=json&limit=1&q=${encodeURIComponent(q)}`);
  const hits = await r.json();
  if (!hits.length) return null;
  return { lat: Number(hits[0].lat), lon: Number(hits[0].lon), name: hits[0].display_name };
}
