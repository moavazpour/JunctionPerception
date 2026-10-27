// Sites: create, start, stop and delete perception instances (multi-site stack with the manager API).
// A site is a video file or a live stream plus where the camera is: the installer clicks the camera
// position on an aerial map, then a point it looks at (the viewing direction), and gives the height.
import L from "leaflet";
import "leaflet/dist/leaflet.css";

export interface SiteInfo {
  id: string;
  name: string;
  source: string;
  status: string;
  calibrated: boolean;
  position: { lat: number; lon: number; height_m: number; bearing_deg: number };
}

const api = async (path: string, init?: RequestInit) => {
  const r = await fetch(`/api${path}`, init);
  const body = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(body.detail ? String(body.detail) : `${r.status} ${r.statusText}`);
  return body;
};

export async function managerAvailable(): Promise<boolean> {
  try {
    await api("/health");
    return true;
  } catch {
    return false;
  }
}

export async function listSites(): Promise<SiteInfo[]> {
  return api("/sites");
}

function bearing(a: L.LatLng, b: L.LatLng) {
  const f1 = (a.lat * Math.PI) / 180, f2 = (b.lat * Math.PI) / 180, dl = ((b.lng - a.lng) * Math.PI) / 180;
  const y = Math.sin(dl) * Math.cos(f2), x = Math.cos(f1) * Math.sin(f2) - Math.sin(f1) * Math.cos(f2) * Math.cos(dl);
  return ((Math.atan2(y, x) * 180) / Math.PI + 360) % 360;
}

export class SitesPanel {
  private el: HTMLElement;
  private map: L.Map | null = null;
  private cam: L.LatLng | null = null;
  private look: L.LatLng | null = null;
  private layer = L.layerGroup();

  constructor(private current: string | null) {
    this.el = document.createElement("div");
    this.el.className = "sites-modal hidden";
    this.el.innerHTML = `
      <div class="sites-box">
        <div class="sites-head"><b>Sites</b><button id="sites-close">✕</button></div>
        <table class="sites-list"><thead><tr><th>Name</th><th>Source</th><th>State</th><th>Calibrated</th><th></th></tr></thead><tbody></tbody></table>
        <div class="card">
          <b>New site</b>
          <div class="form">
            <label>Name <input id="ns-name" placeholder="e.g. Georg-Brauchle-Ring east" /></label>
            <label><input type="radio" name="ns-src" value="video" checked /> Video file <input id="ns-file" type="file" accept="video/*" /></label>
            <label><input type="radio" name="ns-src" value="stream" /> Live stream <input id="ns-url" placeholder="rtsp://user:pass@camera/stream" /></label>
            <label>Camera height above the road (m) <input id="ns-height" type="number" value="7" min="1" max="100" step="0.5" /></label>
          </div>
          <div class="ns-search"><input id="ns-q" placeholder="Search an address or place…" /><button id="ns-go">Find</button></div>
          <div class="muted small">On the map: click where the camera is, then click a point it looks at.</div>
          <div id="ns-map" class="ns-map"></div>
          <div id="ns-pos" class="small muted">Camera position: not set</div>
          <div class="mc-buttons"><button id="ns-create">Create and start</button></div>
          <div id="ns-msg" class="small"></div>
        </div>
      </div>`;
    document.body.appendChild(this.el);
    this.el.querySelector("#sites-close")!.addEventListener("click", () => this.hide());
    this.el.querySelector("#ns-create")!.addEventListener("click", () => this.create());
    const go = () => this.search((this.el.querySelector("#ns-q") as HTMLInputElement).value);
    this.el.querySelector("#ns-go")!.addEventListener("click", go);
    this.el.querySelector("#ns-q")!.addEventListener("keydown", (e) => (e as KeyboardEvent).key === "Enter" && go());
  }

  async show() {
    this.el.classList.remove("hidden");
    if (!this.map) {
      this.map = L.map(this.el.querySelector("#ns-map") as HTMLElement, { maxZoom: 22 }).setView([48.1372, 11.5756], 13);
      L.tileLayer("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}", {
        maxNativeZoom: 19, maxZoom: 22, attribution: "Imagery © Esri",
      }).addTo(this.map);
      this.layer.addTo(this.map);
      this.map.on("click", (e: L.LeafletMouseEvent) => {
        if (!this.cam || this.look) {
          this.cam = e.latlng;
          this.look = null;
        } else {
          this.look = e.latlng;
        }
        this.drawPos();
      });
    }
    setTimeout(() => this.map?.invalidateSize(), 50);
    await this.refresh();
  }

  hide() {
    this.el.classList.add("hidden");
  }

  // Address search (OpenStreetMap Nominatim) to jump the map to the site.
  private async search(q: string) {
    if (!q.trim() || !this.map) return;
    try {
      const r = await fetch(`https://nominatim.openstreetmap.org/search?format=json&limit=1&q=${encodeURIComponent(q)}`);
      const hits = await r.json();
      if (!hits.length) return this.msg(`Nothing found for "${q}".`, false);
      this.map.setView([Number(hits[0].lat), Number(hits[0].lon)], 18);
      this.msg(hits[0].display_name);
    } catch {
      this.msg("Address search is not reachable; zoom the map by hand.", false);
    }
  }

  private drawPos() {
    this.layer.clearLayers();
    const pos = this.el.querySelector("#ns-pos")!;
    if (!this.cam) return;
    L.circleMarker(this.cam, { radius: 7, color: "#4e8cff", weight: 3, fillOpacity: 0.8 }).bindTooltip("camera").addTo(this.layer);
    if (this.look) {
      L.polyline([this.cam, this.look], { color: "#ffb020", weight: 3 }).addTo(this.layer);
      pos.textContent = `Camera ${this.cam.lat.toFixed(6)}, ${this.cam.lng.toFixed(6)} · looking ${bearing(this.cam, this.look).toFixed(0)}°`;
    } else {
      pos.textContent = `Camera ${this.cam.lat.toFixed(6)}, ${this.cam.lng.toFixed(6)} · now click a point it looks at`;
    }
  }

  private msg(text: string, ok = true) {
    const m = this.el.querySelector("#ns-msg")!;
    m.className = `small ${ok ? "muted" : "mc-bad"}`;
    m.textContent = text;
  }

  async refresh() {
    const body = this.el.querySelector(".sites-list tbody")!;
    let sites: SiteInfo[] = [];
    try {
      sites = await listSites();
    } catch (e) {
      body.innerHTML = `<tr><td colspan="5" class="mc-bad">${(e as Error).message}</td></tr>`;
      return;
    }
    body.innerHTML = sites.length ? "" : `<tr><td colspan="5" class="muted">No sites yet: create one below.</td></tr>`;
    for (const s of sites) {
      const tr = document.createElement("tr");
      tr.className = s.id === this.current ? "sel" : "";
      tr.innerHTML = `<td>${s.name}</td><td class="small">${s.source}</td><td>${s.status}</td><td>${s.calibrated ? "yes" : "no"}</td>
        <td class="row-btns"><button data-a="view">View</button>
        <button data-a="${s.status === "running" ? "stop" : "start"}">${s.status === "running" ? "Stop" : "Start"}</button>
        <button data-a="delete">Delete</button></td>`;
      tr.querySelectorAll<HTMLButtonElement>("button").forEach((b) =>
        b.addEventListener("click", async (ev) => {
          ev.stopPropagation();
          const a = b.dataset.a!;
          try {
            if (a === "view") location.search = `?site=${encodeURIComponent(s.id)}`;
            else if (a === "delete") {
              if (!confirm(`Delete site "${s.name}" with its video and calibration?`)) return;
              await api(`/sites/${s.id}`, { method: "DELETE" });
            } else await api(`/sites/${s.id}/${a}`, { method: "POST" });
            await this.refresh();
          } catch (e) {
            this.msg((e as Error).message, false);
          }
        }),
      );
      body.appendChild(tr);
    }
  }

  private upload(file: File): Promise<string> {
    return new Promise((res, rej) => {
      const fd = new FormData();
      fd.append("file", file);
      const x = new XMLHttpRequest();
      x.open("POST", "/api/videos");
      x.upload.onprogress = (e) => e.lengthComputable && this.msg(`Uploading ${Math.round((100 * e.loaded) / e.total)} %…`);
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

  private async create() {
    const q = <T extends HTMLElement>(s: string) => this.el.querySelector(s) as T;
    const name = q<HTMLInputElement>("#ns-name").value.trim();
    const kind = (this.el.querySelector('input[name="ns-src"]:checked') as HTMLInputElement).value;
    if (!name) return this.msg("Give the site a name.", false);
    if (!this.cam || !this.look) return this.msg("Click the camera position and a point it looks at on the map.", false);
    const body: Record<string, unknown> = {
      name,
      lat: this.cam.lat,
      lon: this.cam.lng,
      height_m: Number(q<HTMLInputElement>("#ns-height").value) || 7,
      bearing_deg: bearing(this.cam, this.look),
    };
    try {
      if (kind === "video") {
        const f = q<HTMLInputElement>("#ns-file").files?.[0];
        if (!f) return this.msg("Choose a video file.", false);
        body.video = await this.upload(f);
      } else {
        body.stream = q<HTMLInputElement>("#ns-url").value.trim();
        if (!body.stream) return this.msg("Enter the stream URL.", false);
      }
      this.msg("Starting… (the first start builds the TensorRT engine, about a minute)");
      const s = await api("/sites", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
      location.search = `?site=${encodeURIComponent(s.id)}`;
    } catch (e) {
      this.msg((e as Error).message, false);
    }
  }
}
