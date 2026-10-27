// Manual calibration: the installer clicks a point on the road surface in the (pixelated) camera
// snapshot, then the same point on an aerial map. With 4+ pairs the device solves focal length, pose and
// camera position (calibration/manual on MQTT) and publishes the new calibration as usual.
import L from "leaflet";
import "leaflet/dist/leaflet.css";
import type { ManualResultMsg, SnapshotMsg } from "./types";

interface Pair {
  u?: number; // frame pixels
  v?: number;
  lat?: number;
  lon?: number;
}

const COLORS = ["#ff4d4d", "#3ddc84", "#4e8cff", "#ffb020", "#c86bff", "#36d6e7", "#ff7ab6", "#b8e04a"];
const color = (i: number) => COLORS[i % COLORS.length];

export class ManualCalibration {
  private pairs: Pair[] = [];
  private snap: SnapshotMsg | null = null;
  private img = new Image();
  private map: L.Map | null = null;
  private layer = L.layerGroup();
  private canvas: HTMLCanvasElement;

  constructor(private host: HTMLElement, private send: (cmd: object) => void) {
    host.innerHTML = `
      <div class="card">
        <div class="mc-title"><b>Manual calibration</b><button id="mc-full">Full screen</button></div>
        <div class="muted small">Click a point on the road surface in the camera view (lane-marking corners,
          stop-line ends, kerb corners), then the same point on the map. Use at least 4 pairs (6+ is better),
          spread from near to far and across the road.</div>
        <div class="mc-views"><canvas id="mc-img" class="mc-img"></canvas>
        <div id="mc-map" class="mc-map"></div></div>
        <table class="mc-pairs"><thead><tr><th>#</th><th>image (px)</th><th>map</th><th></th></tr></thead><tbody></tbody></table>
        <div class="mc-buttons">
          <button id="mc-solve" disabled>Solve</button>
          <button id="mc-clear">Clear</button>
          <button id="mc-snap">New snapshot</button>
          <button id="mc-auto">Clear calibration</button>
        </div>
        <div id="mc-result" class="small"></div>
      </div>`;
    this.canvas = host.querySelector("#mc-img")!;
    this.canvas.addEventListener("click", (e) => this.onImageClick(e));
    host.querySelector("#mc-full")!.addEventListener("click", (e) => {
      const card = host.querySelector(".card")!;
      card.classList.toggle("mc-fullscreen");
      (e.target as HTMLButtonElement).textContent = card.classList.contains("mc-fullscreen") ? "Close full screen" : "Full screen";
      setTimeout(() => this.map?.invalidateSize(), 50);
    });
    host.querySelector("#mc-solve")!.addEventListener("click", () => this.solve());
    host.querySelector("#mc-clear")!.addEventListener("click", () => {
      this.pairs = [];
      this.redraw();
    });
    host.querySelector("#mc-snap")!.addEventListener("click", () => this.send({ action: "snapshot" }));
    host.querySelector("#mc-auto")!.addEventListener("click", () => {
      if (confirm("Discard the current calibration?")) this.send({ action: "auto" });
    });
    this.img.onload = () => this.redraw();
    this.drawImage();
  }

  // Leaflet needs a visible container; called when the tab is shown.
  show() {
    if (!this.map) {
      const anchor = this.snap?.anchor;
      this.map = L.map(this.host.querySelector("#mc-map") as HTMLElement, { maxZoom: 22 }).setView(
        anchor ? [anchor.lat, anchor.lon] : [48.1372, 11.5756],
        anchor ? 19 : 12,
      );
      const sat = L.tileLayer(
        "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        { maxNativeZoom: 19, maxZoom: 22, attribution: "Imagery © Esri" },
      ).addTo(this.map);
      const osm = L.tileLayer("https://tile.openstreetmap.org/{z}/{x}/{y}.png", {
        maxNativeZoom: 19,
        maxZoom: 22,
        attribution: "© OpenStreetMap contributors",
      });
      L.control.layers({ Aerial: sat, Map: osm }).addTo(this.map);
      this.layer.addTo(this.map);
      this.map.on("click", (e: L.LeafletMouseEvent) => this.onMapClick(e.latlng));
      this.redraw();
    }
    setTimeout(() => this.map?.invalidateSize(), 50);
  }

  setSnapshot(s: SnapshotMsg) {
    const first = !this.snap;
    this.snap = s;
    this.img.src = `data:image/jpeg;base64,${s.jpeg_base64}`;
    if (first && this.map && s.anchor) this.map.setView([s.anchor.lat, s.anchor.lon], 19);
  }

  setResult(m: ManualResultMsg) {
    const r = m.result;
    const el = this.host.querySelector("#mc-result")!;
    el.className = `small ${r.ok ? "mc-ok" : "mc-bad"}`;
    el.textContent = r.message ?? (r.ok ? "done" : "failed");
  }

  private onImageClick(e: MouseEvent) {
    if (!this.snap) return;
    const r = this.canvas.getBoundingClientRect();
    const sx = ((e.clientX - r.left) * this.canvas.width) / r.width;
    const sy = ((e.clientY - r.top) * this.canvas.height) / r.height;
    const p = { u: sx / this.snap.scale, v: sy / this.snap.scale };
    const last = this.pairs[this.pairs.length - 1];
    if (last && last.lat === undefined) Object.assign(last, p); // re-pick the image point of an open pair
    else this.pairs.push(p);
    this.redraw();
  }

  private onMapClick(ll: L.LatLng) {
    const last = this.pairs[this.pairs.length - 1];
    if (!last || last.u === undefined || last.lat !== undefined) {
      this.result("Click the point in the camera view first.", false);
      return;
    }
    last.lat = ll.lat;
    last.lon = ll.lng;
    this.redraw();
  }

  private solve() {
    const pts = this.pairs.filter((p) => p.u !== undefined && p.lat !== undefined);
    this.result(`Solving with ${pts.length} pairs…`, true);
    this.send({ action: "solve", points: pts.map((p) => ({ u: p.u, v: p.v, lat: p.lat, lon: p.lon })) });
  }

  private result(text: string, ok: boolean) {
    const el = this.host.querySelector("#mc-result")!;
    el.className = `small ${ok ? "muted" : "mc-bad"}`;
    el.textContent = text;
  }

  private drawImage() {
    const ctx = this.canvas.getContext("2d")!;
    if (!this.snap || !this.img.complete || !this.img.naturalWidth) {
      this.canvas.width = 960;
      this.canvas.height = 540;
      ctx.fillStyle = "#111";
      ctx.fillRect(0, 0, 960, 540);
      ctx.fillStyle = "#8a93a0";
      ctx.font = "20px system-ui";
      ctx.fillText("Waiting for the calibration snapshot…", 30, 60);
      return;
    }
    this.canvas.width = this.snap.width;
    this.canvas.height = this.snap.height;
    ctx.drawImage(this.img, 0, 0);
    this.pairs.forEach((p, i) => {
      if (p.u === undefined || p.v === undefined) return;
      const x = p.u * this.snap!.scale, y = p.v * this.snap!.scale;
      ctx.strokeStyle = color(i);
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(x - 9, y);
      ctx.lineTo(x + 9, y);
      ctx.moveTo(x, y - 9);
      ctx.lineTo(x, y + 9);
      ctx.stroke();
      ctx.fillStyle = color(i);
      ctx.font = "bold 15px system-ui";
      ctx.fillText(String(i + 1), x + 6, y - 6);
    });
  }

  private redraw() {
    this.drawImage();
    this.layer.clearLayers();
    this.pairs.forEach((p, i) => {
      if (p.lat === undefined || p.lon === undefined) return;
      L.circleMarker([p.lat, p.lon], { radius: 6, color: color(i), weight: 2, fillOpacity: 0.6 })
        .bindTooltip(String(i + 1), { permanent: true, direction: "right", className: "mc-tip" })
        .addTo(this.layer);
    });
    const body = this.host.querySelector(".mc-pairs tbody")!;
    body.innerHTML = "";
    this.pairs.forEach((p, i) => {
      const tr = document.createElement("tr");
      tr.innerHTML = `<td style="color:${color(i)}">${i + 1}</td>
        <td>${p.u !== undefined ? `${p.u.toFixed(0)}, ${p.v!.toFixed(0)}` : "–"}</td>
        <td>${p.lat !== undefined ? `${p.lat.toFixed(6)}, ${p.lon!.toFixed(6)}` : "click on the map"}</td>
        <td><button data-del="${i}">✕</button></td>`;
      body.appendChild(tr);
    });
    body.querySelectorAll<HTMLButtonElement>("[data-del]").forEach((b) =>
      b.addEventListener("click", () => {
        this.pairs.splice(Number(b.dataset.del), 1);
        this.redraw();
      }),
    );
    const complete = this.pairs.filter((p) => p.u !== undefined && p.lat !== undefined).length;
    (this.host.querySelector("#mc-solve") as HTMLButtonElement).disabled = complete < 4;
  }
}
