import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { CSS2DObject, CSS2DRenderer } from "three/examples/jsm/renderers/CSS2DRenderer.js";
import { aerialGround } from "./aerial";
import { PALETTES, buildStylizedMap } from "./osmroads";
import { CLASS_LABEL, LANE_COLORS } from "./store";
import { buildModel, glowTexture } from "./models";
import { geoToEnu } from "./geo";
import type { CameraJson, DetectorsMsg, NearMiss, ObjectClass, OcclusionMsg, RoadUser, SignalsMsg, SiteMap, UserMap } from "./types";

// Ground frame (x east, y north) -> three.js (x east, y up, z south).
const toV3 = (x: number, y: number, h = 0) => new THREE.Vector3(x, h, -y);

interface Actor {
  id: number;
  cls: ObjectClass;
  group: THREE.Group;
  model: THREE.Group;
  glow: THREE.Mesh;
  label: CSS2DObject;
  target: RoadUser;
  received: number;
  seen: number;
}

export class Scene3D {
  readonly renderer: THREE.WebGLRenderer;
  readonly labels: CSS2DRenderer;
  readonly scene = new THREE.Scene();
  readonly camera: THREE.PerspectiveCamera;
  readonly controls: OrbitControls;
  private actors = new Map<number, Actor>();
  private mapGroup = new THREE.Group();
  private zoneMeshes = new Map<string, THREE.Mesh>();
  private occMesh?: THREE.Mesh;
  private eventsGroup = new THREE.Group();
  private sensor = new THREE.Group();
  private signalGroup = new THREE.Group();
  private signalLamps = new Map<string, THREE.MeshStandardMaterial[]>();
  private userMap?: UserMap;
  private glowTex = glowTexture();
  private cam?: CameraJson;
  private aerial?: THREE.Mesh;
  private aerialKey = "";
  private stylized?: THREE.Group;
  private stylizedKey = "";
  private ground!: THREE.Mesh;
  private grid!: THREE.GridHelper;
  private anchor?: { lat: number; lon: number };
  // "stylized": roads drawn from OpenStreetMap; "aerial": satellite imagery (default).
  mapMode: "stylized" | "aerial" = "stylized";
  theme: "dark" | "light" = "dark";
  private hemi!: THREE.HemisphereLight;
  private lastMap?: { map: SiteMap; cam: CameraJson };
  private clock = new THREE.Clock();
  selected: number | null = null;
  showAllLabels = false;
  onSelect: (id: number | null) => void = () => {};

  constructor(private host: HTMLElement) {
    // Logarithmic depth: the ground layers are centimetres apart and hundreds of metres away (no flicker).
    this.renderer = new THREE.WebGLRenderer({ antialias: true, logarithmicDepthBuffer: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    host.appendChild(this.renderer.domElement);
    this.labels = new CSS2DRenderer();
    this.labels.domElement.className = "labels";
    host.appendChild(this.labels.domElement);

    this.scene.background = new THREE.Color(0x0c0e12);
    this.scene.fog = new THREE.Fog(0x0c0e12, 120, 420);
    this.camera = new THREE.PerspectiveCamera(50, 1, 0.5, 2000);
    this.camera.position.set(-30, 45, 40);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.maxPolarAngle = Math.PI * 0.48;

    this.hemi = new THREE.HemisphereLight(0xdfe8ff, 0x1a1c20, 1.1);
    this.scene.add(this.hemi);
    const key = new THREE.DirectionalLight(0xffffff, 1.6);
    key.position.set(-60, 120, 40);
    this.scene.add(key);

    const ground = (this.ground = new THREE.Mesh(
      new THREE.PlaneGeometry(4000, 4000),
      new THREE.MeshStandardMaterial({ color: 0x15181d, roughness: 1 }),
    ));
    ground.rotation.x = -Math.PI / 2;
    ground.position.y = -0.3;
    this.scene.add(ground);
    const grid = (this.grid = new THREE.GridHelper(1200, 120, 0x1f242b, 0x1a1e24));
    grid.position.y = -0.01;
    this.scene.add(grid);
    this.scene.add(this.mapGroup, this.eventsGroup, this.sensor, this.signalGroup);

    this.renderer.domElement.addEventListener("pointerdown", (e) => this.pick(e));
    new ResizeObserver(() => this.resize()).observe(host);
    this.resize();
    this.renderer.setAnimationLoop(() => this.frame());
  }

  private resize() {
    const w = this.host.clientWidth, h = this.host.clientHeight;
    this.renderer.setSize(w, h);
    this.labels.setSize(w, h);
    this.camera.aspect = w / Math.max(h, 1);
    this.camera.updateProjectionMatrix();
  }

  // ------------------------------------------------------------------ map

  // Aerial imagery under the scene, centred on the camera's ground point (loaded once per position).
  private setAerial(cam: CameraJson) {
    if (cam.anchor) this.anchor = cam.anchor;
    this.updateBaseMap();
  }

  // Camera ground position known before calibration (from the snapshot): lets the base map load early.
  setAnchor(lat: number, lon: number) {
    if (!this.anchor) {
      this.anchor = { lat, lon };
      this.updateBaseMap();
    }
  }

  setMapMode(mode: "stylized" | "aerial") {
    this.mapMode = mode;
    this.updateBaseMap();
  }

  setTheme(t: "dark" | "light") {
    if (t === this.theme) return;
    this.theme = t;
    this.hemi.intensity = t === "light" ? 1.6 : 1.1;
    this.hemi.groundColor.setHex(t === "light" ? 0x8a8f96 : 0x1a1c20);
    this.stylizedKey = "";  // redraw the road map in the new palette (OSM data is cached)
    this.updateBaseMap();
    if (this.lastMap) this.setMap(this.lastMap.map, this.lastMap.cam);
  }

  dispose() {
    this.renderer.setAnimationLoop(null);
    this.renderer.dispose();
    this.renderer.domElement.remove();
    this.labels.domElement.remove();
  }

  // Base map under the scene: the stylized OpenStreetMap roads or the aerial imagery (loaded on demand,
  // once per camera position).
  private updateBaseMap() {
    const stylized = this.mapMode === "stylized";
    const STYLE = PALETTES[this.theme];
    const plain = this.theme === "light" ? 0xdfe2e6 : 0x0c0e12;
    const bg = stylized ? STYLE.background : plain;
    this.scene.background = new THREE.Color(bg);
    (this.scene.fog as THREE.Fog).color.setHex(bg);
    (this.ground.material as THREE.MeshStandardMaterial).color.setHex(stylized ? STYLE.ground : this.theme === "light" ? 0xd5d9de : 0x15181d);
    // Until (or unless) the road map is available, the aerial imagery stands in for it.
    const showAerial = !stylized || !this.stylized;
    this.grid.visible = showAerial && !this.aerial;
    if (this.stylized) this.stylized.visible = stylized;
    if (this.aerial) this.aerial.visible = showAerial;
    const a = this.anchor;
    if (!a) return;
    const key = `${a.lat.toFixed(6)},${a.lon.toFixed(6)}`;
    if (stylized && key !== this.stylizedKey) {
      this.stylizedKey = key;
      buildStylizedMap(a.lat, a.lon, STYLE).then((g) => {
        if (key !== this.stylizedKey) return;
        if (!g) {
          // Map data not reachable right now: try again in a while.
          this.stylizedKey = "";
          setTimeout(() => this.updateBaseMap(), 15000);
          return;
        }
        if (this.stylized) this.scene.remove(this.stylized);
        this.stylized = g;
        this.scene.add(g);
        this.updateBaseMap();
      });
    }
    if (showAerial && key !== this.aerialKey) {
      this.aerialKey = key;
      const maxTex = Math.min(this.renderer.capabilities.maxTextureSize, 8192);
      aerialGround(a.lat, a.lon, maxTex).then((m) => {
        if (key !== this.aerialKey) return;
        if (!m) {
          this.aerialKey = "";
          setTimeout(() => this.updateBaseMap(), 15000);
          return;
        }
        if (this.aerial) this.scene.remove(this.aerial);
        this.aerial = m;
        m.visible = this.mapMode === "aerial" || !this.stylized;
        this.grid.visible = false;
        this.scene.add(m);
      });
    }
  }

  setMap(map: SiteMap, cam: CameraJson) {
    this.lastMap = { map, cam };
    this.setAerial(cam);
    this.cam = cam;
    this.mapGroup.clear();
    this.zoneMeshes.clear();
    // Over aerial imagery the lanes are a light overlay, so the real road stays visible.
    // Learned lanes: a light neutral overlay on the base map. Drawn (installer) lanes: one colour each.
    const light = this.theme === "light";
    const neutral = light ? 0x8a9099 : 0x2b2f36;
    let userIdx = 0;
    for (const lane of map.lanes) {
      const user = lane.source === "user";
      const colour = user ? new THREE.Color(LANE_COLORS[userIdx++ % LANE_COLORS.length]).getHex() : neutral;
      const asphalt = new THREE.MeshBasicMaterial({ color: colour, transparent: true, opacity: user ? 0.26 : cam.anchor ? 0.22 : 1, depthWrite: false });
      const lineMat = new THREE.LineDashedMaterial({ color: user ? colour : light ? 0x7d838c : 0xc9ced6, dashSize: 3, gapSize: 6, transparent: true, opacity: user ? 0.9 : 0.5 });
      const pts = lane.centerline.map(([x, y]) => new THREE.Vector2(x, y));
      if (pts.length < 2) continue;
      const left: THREE.Vector3[] = [], right: THREE.Vector3[] = [];
      for (let i = 0; i < pts.length; i++) {
        const a = pts[Math.max(i - 1, 0)], b = pts[Math.min(i + 1, pts.length - 1)];
        const t = b.clone().sub(a).normalize();
        const n = new THREE.Vector2(-t.y, t.x).multiplyScalar(lane.width / 2);
        left.push(toV3(pts[i].x + n.x, pts[i].y + n.y, 0.01));
        right.push(toV3(pts[i].x - n.x, pts[i].y - n.y, 0.01));
      }
      // Ribbon.
      const pos: number[] = [];
      for (let i = 0; i < pts.length - 1; i++) {
        const q = [left[i], right[i], left[i + 1], right[i], right[i + 1], left[i + 1]];
        for (const v of q) pos.push(v.x, v.y, v.z);
      }
      const g = new THREE.BufferGeometry();
      g.setAttribute("position", new THREE.Float32BufferAttribute(pos, 3));
      g.computeVertexNormals();
      const ribbon = new THREE.Mesh(g, asphalt);
      ribbon.material.side = THREE.DoubleSide;
      this.mapGroup.add(ribbon);
      for (const edge of [left, right]) {
        const lg = new THREE.BufferGeometry().setFromPoints(edge.map((v) => v.clone().setY(0.03)));
        const line = new THREE.Line(lg, lineMat);
        line.computeLineDistances();
        this.mapGroup.add(line);
      }
      // Direction chevrons every 25 m.
      const total = pts.reduce((s, p, i) => (i ? s + p.distanceTo(pts[i - 1]) : 0), 0);
      for (let s = 10; s < total - 5; s += 25) {
        const [p, t] = this.along(pts, s);
        const chev = new THREE.Shape();
        chev.moveTo(0.9, 0);
        chev.lineTo(-0.6, 0.6);
        chev.lineTo(-0.3, 0);
        chev.lineTo(-0.6, -0.6);
        chev.closePath();
        const m = new THREE.Mesh(
          new THREE.ShapeGeometry(chev),
          new THREE.MeshBasicMaterial({ color: user ? colour : lane.to_junction ? 0x4e8cff : 0x5b6470, transparent: true, opacity: user ? 0.85 : 0.55 }),
        );
        m.rotation.x = -Math.PI / 2;
        m.rotation.z = Math.atan2(t.y, t.x);
        m.position.copy(toV3(p.x, p.y, 0.04));
        this.mapGroup.add(m);
      }
      if (lane.to_junction && lane.stop_s !== undefined) {
        const [p, t] = this.along(pts, lane.stop_s);
        const bar = new THREE.Mesh(new THREE.PlaneGeometry(0.5, lane.width), new THREE.MeshBasicMaterial({ color: 0xffffff }));
        bar.rotation.x = -Math.PI / 2;
        bar.rotation.z = Math.atan2(t.y, t.x);
        bar.position.copy(toV3(p.x, p.y, 0.05));
        this.mapGroup.add(bar);
      }
    }
    for (const cw of map.crosswalks) this.mapGroup.add(this.zebra(cw));
    for (const z of map.zones) {
      const shape = new THREE.Shape(z.polygon.map(([x, y]) => new THREE.Vector2(x, y)));
      const m = new THREE.Mesh(
        new THREE.ShapeGeometry(shape),
        new THREE.MeshBasicMaterial({ color: 0x3ddc84, transparent: true, opacity: 0.0, depthWrite: false }),
      );
      m.rotation.x = -Math.PI / 2;
      m.position.y = 0.06;
      this.mapGroup.add(m);
      this.zoneMeshes.set(z.id, m);
    }
    this.buildSensor(cam);
    this.setSignalLayout(this.userMap);
  }

  // Traffic light heads at the stop lines they control, floating (no pole), facing the approaching traffic.
  setSignalLayout(um?: UserMap) {
    this.userMap = um;
    this.signalGroup.clear();
    this.signalLamps.clear();
    const anchor = this.cam?.anchor;
    if (!um?.signals?.length || !anchor) return;
    const enu = (p: number[]) => geoToEnu(anchor, p[0], p[1]);
    for (const s of um.signals) {
      const sl = um.stop_lines[s.stop_line];
      if (!sl) continue;
      const a = enu(sl[0]), b = enu(sl[1]);
      const mid = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
      // Travel direction: the drawn lane passing closest to the stop line, else away from the camera.
      let dir: number[] | null = null, best = 1e9;
      for (const l of um.lanes) {
        const pts = l.points.map(enu);
        for (let i = 1; i < pts.length; i++) {
          const d = Math.hypot((pts[i][0] + pts[i - 1][0]) / 2 - mid[0], (pts[i][1] + pts[i - 1][1]) / 2 - mid[1]);
          if (d < best) {
            best = d;
            const L = Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]) || 1;
            dir = [(pts[i][0] - pts[i - 1][0]) / L, (pts[i][1] - pts[i - 1][1]) / L];
          }
        }
      }
      if (!dir || best > 40) {
        const t = [b[0] - a[0], b[1] - a[1]], L = Math.hypot(t[0], t[1]) || 1;
        dir = [-t[1] / L, t[0] / L];
        if (dir[0] * mid[0] + dir[1] * mid[1] < 0) dir = [-dir[0], -dir[1]];
      }
      // On the right-hand end of the stop line, 5 m up.
      const r = [dir[1], -dir[0]];
      const end = (a[0] - mid[0]) * r[0] + (a[1] - mid[1]) * r[1] > 0 ? a : b;
      const g = new THREE.Group();
      const box = new THREE.Mesh(new THREE.BoxGeometry(0.45, 1.2, 0.35), new THREE.MeshStandardMaterial({ color: 0x1f2227 }));
      g.add(box);
      const mats = [0xff3b30, 0xffb020, 0x34c759].map((c, i) => {
        const m = new THREE.MeshStandardMaterial({ color: 0x30333a, emissive: c, emissiveIntensity: 0 });
        const lamp = new THREE.Mesh(new THREE.SphereGeometry(0.13, 14, 10), m);
        lamp.position.set(0, 0.36 - i * 0.36, 0.18);
        g.add(lamp);
        return m;
      });
      g.position.copy(toV3(end[0] + r[0] * 0.8, end[1] + r[1] * 0.8, 5.2));
      g.rotation.y = Math.atan2(-dir[0], dir[1]);
      this.signalGroup.add(g);
      this.signalLamps.set(s.id, mats);
    }
  }

  setSignalStates(msg?: SignalsMsg) {
    for (const [id, mats] of this.signalLamps) {
      const st = msg?.signals.find((x) => x.id === id)?.state ?? "unknown";
      const on = [st === "red" || st === "red_amber", st === "amber" || st === "red_amber", st === "green"];
      mats.forEach((m, i) => {
        m.emissiveIntensity = on[i] ? 2.4 : 0;
        m.color.setHex(on[i] ? 0xffffff : 0x30333a);
      });
    }
  }

  private along(pts: THREE.Vector2[], s: number): [THREE.Vector2, THREE.Vector2] {
    let acc = 0;
    for (let i = 1; i < pts.length; i++) {
      const L = pts[i].distanceTo(pts[i - 1]);
      if (acc + L >= s || i === pts.length - 1) {
        const t = pts[i].clone().sub(pts[i - 1]).normalize();
        return [pts[i - 1].clone().add(t.clone().multiplyScalar(Math.min(Math.max(s - acc, 0), L))), t];
      }
      acc += L;
    }
    return [pts[0], new THREE.Vector2(1, 0)];
  }

  private zebra(poly: [number, number][]): THREE.Group {
    const g = new THREE.Group();
    const pts = poly.map(([x, y]) => new THREE.Vector2(x, y));
    // Stripes parallel to the longest edge, 0.5 m wide every 1 m.
    let best = 0, bi = 0;
    for (let i = 0; i < pts.length; i++) {
      const d = pts[i].distanceTo(pts[(i + 1) % pts.length]);
      if (d > best) [best, bi] = [d, i];
    }
    const a = pts[bi], b = pts[(bi + 1) % pts.length];
    const dir = b.clone().sub(a).normalize();
    const nrm = new THREE.Vector2(-dir.y, dir.x);
    let depth = 0;
    for (const p of pts) depth = Math.max(depth, Math.abs(p.clone().sub(a).dot(nrm)));
    const side = pts.some((p) => p.clone().sub(a).dot(nrm) < -0.1) ? -1 : 1;
    const mat = new THREE.MeshBasicMaterial({ color: 0xe9edf2 });
    for (let s = 0.5; s < best - 0.25; s += 1.0) {
      const c = a.clone().add(dir.clone().multiplyScalar(s)).add(nrm.clone().multiplyScalar((side * depth) / 2));
      const m = new THREE.Mesh(new THREE.PlaneGeometry(0.5, depth), mat);
      m.rotation.x = -Math.PI / 2;
      m.rotation.z = Math.atan2(dir.y, dir.x);
      m.position.copy(toV3(c.x, c.y, 0.05));
      g.add(m);
    }
    return g;
  }

  private buildSensor(cam: CameraJson) {
    this.sensor.clear();
    const h = cam.height_m;
    const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.08, h), new THREE.MeshStandardMaterial({ color: 0x59606a }));
    pole.position.y = h / 2;
    this.sensor.add(pole);
    const head = new THREE.Mesh(new THREE.SphereGeometry(0.35, 16, 12), new THREE.MeshStandardMaterial({ color: 0x4e8cff, emissive: 0x2a5cff, emissiveIntensity: 1.5 }));
    head.position.y = h;
    this.sensor.add(head);
    // Field of view wedge on the ground.
    const fx = cam.K[0][0];
    const half = Math.atan((cam.width / 2) / fx);
    const b = (cam.bearing_deg * Math.PI) / 180;
    const range = 160;
    const wedge = new THREE.Shape();
    wedge.moveTo(0, 0);
    for (let k = -1; k <= 1.001; k += 0.1) {
      const a = b + k * half;
      wedge.lineTo(Math.sin(a) * range, Math.cos(a) * range);
    }
    wedge.closePath();
    const m = new THREE.Mesh(
      new THREE.ShapeGeometry(wedge),
      new THREE.MeshBasicMaterial({ color: 0x4e8cff, transparent: true, opacity: 0.05, depthWrite: false }),
    );
    m.rotation.x = -Math.PI / 2;
    m.position.y = 0.02;
    this.sensor.add(m);
  }

  // ------------------------------------------------------------------ dynamic state

  updateObjects(objs: RoadUser[], now: number) {
    const seen = new Set<number>();
    for (const o of objs) {
      seen.add(o.id);
      let a = this.actors.get(o.id);
      if (!a || a.cls !== o.class) {
        if (a) this.remove(a);
        a = this.create(o);
      }
      a.target = o;
      a.received = now;
      a.seen = now;
    }
    for (const a of [...this.actors.values()]) if (!seen.has(a.id) && now - a.seen > 0.6) this.remove(a);
  }

  private create(o: RoadUser): Actor {
    const group = new THREE.Group();
    const model = buildModel(o.class);
    // Per-actor materials so fading one predicted object does not fade all of them.
    model.traverse((m) => {
      if (m instanceof THREE.Mesh) m.material = (m.material as THREE.Material).clone();
    });
    group.add(model);
    const glow = new THREE.Mesh(
      new THREE.PlaneGeometry(1, 1),
      new THREE.MeshBasicMaterial({ map: this.glowTex, transparent: true, depthWrite: false, blending: THREE.AdditiveBlending }),
    );
    glow.rotation.x = -Math.PI / 2;
    glow.position.y = 0.03;
    glow.visible = false;
    group.add(glow);
    const el = document.createElement("div");
    el.className = "tag";
    const label = new CSS2DObject(el);
    label.position.set(0, 0, 0);
    group.add(label);
    group.position.copy(toV3(o.x, o.y));
    group.rotation.y = o.yaw;
    group.userData.id = o.id;
    this.scene.add(group);
    const a: Actor = { id: o.id, cls: o.class, group, model, glow, label, target: o, received: 0, seen: 0 };
    this.actors.set(o.id, a);
    return a;
  }

  private remove(a: Actor) {
    this.scene.remove(a.group);
    a.label.element.remove();
    this.actors.delete(a.id);
  }

  setDetectors(msg: DetectorsMsg) {
    for (const z of msg.zones) {
      const m = this.zoneMeshes.get(z.id);
      if (!m) continue;
      const mat = m.material as THREE.MeshBasicMaterial;
      mat.opacity = z.occupied ? 0.28 : 0.07;
      mat.color.setHex(z.type === "count" ? 0x4e8cff : 0x3ddc84);
    }
  }

  setOcclusion(msg: OcclusionMsg | null, visible: boolean) {
    if (this.occMesh) {
      this.scene.remove(this.occMesh);
      this.occMesh = undefined;
    }
    if (!msg || !visible) return;
    const o = msg.occlusion;
    const data = new Uint8Array(o.cols * o.rows * 4);
    const colors: Record<number, [number, number, number, number]> = {
      0: [0, 0, 0, 0],
      1: [0, 0, 0, 0],
      2: [160, 90, 255, 110],
      3: [255, 170, 40, 140],
      4: [120, 130, 140, 60],
    };
    let i = 0;
    for (let k = 0; k < o.rle.length; k += 2) {
      const c = colors[o.rle[k]] ?? [0, 0, 0, 0];
      for (let n = 0; n < o.rle[k + 1]; n++, i++) data.set(c, i * 4);
    }
    const tex = new THREE.DataTexture(data, o.cols, o.rows);
    tex.magFilter = THREE.NearestFilter;
    tex.needsUpdate = true;
    const w = o.cols * o.cell, h = o.rows * o.cell;
    const m = new THREE.Mesh(new THREE.PlaneGeometry(w, h), new THREE.MeshBasicMaterial({ map: tex, transparent: true, depthWrite: false }));
    m.rotation.x = -Math.PI / 2;
    m.position.copy(toV3(o.origin[0] + w / 2, o.origin[1] + h / 2, 0.07));
    this.occMesh = m;
    this.scene.add(m);
  }

  flashEvent(e: NearMiss) {
    const color = e.severity === "critical" ? 0xff3b3b : 0xffa726;
    const ring = new THREE.Mesh(
      new THREE.RingGeometry(1.2, 1.6, 40),
      new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.9, side: THREE.DoubleSide, depthWrite: false }),
    );
    ring.rotation.x = -Math.PI / 2;
    ring.position.copy(toV3(e.x, e.y, 0.08));
    ring.userData.born = this.clock.elapsedTime;
    this.eventsGroup.add(ring);
  }

  // ------------------------------------------------------------------ views & picking

  view(kind: "orbit" | "top" | "camera") {
    if (!this.cam) return;
    const b = (this.cam.bearing_deg * Math.PI) / 180;
    const f = new THREE.Vector2(Math.sin(b), Math.cos(b));
    if (kind === "top") {
      const c = toV3(f.x * 70, f.y * 70);
      this.camera.position.set(c.x, 170, c.z + 0.01);
      this.controls.target.copy(c);
    } else if (kind === "camera") {
      // From the real camera position, looking down the road like the video.
      const pitch = (this.cam.pitch_deg * Math.PI) / 180;
      this.camera.position.copy(toV3(0, 0, this.cam.height_m));
      const d = 60;
      this.controls.target.copy(toV3(f.x * d, f.y * d, this.cam.height_m - Math.tan(pitch) * d));
      this.camera.fov = (2 * Math.atan(this.cam.height / 2 / this.cam.K[1][1]) * 180) / Math.PI;
      this.camera.updateProjectionMatrix();
      return;
    } else {
      // Chase view: behind and above the camera, looking along the road (Tesla-style).
      this.camera.position.copy(toV3(-f.x * 25, -f.y * 25, 28));
      this.controls.target.copy(toV3(f.x * 60, f.y * 60, 0));
    }
    this.camera.fov = 50;
    this.camera.updateProjectionMatrix();
  }

  private pick(e: PointerEvent) {
    const r = this.renderer.domElement.getBoundingClientRect();
    const ndc = new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
    const ray = new THREE.Raycaster();
    ray.setFromCamera(ndc, this.camera);
    const hits = ray.intersectObjects([...this.actors.values()].map((a) => a.group), true);
    let id: number | null = null;
    if (hits.length) {
      let o: THREE.Object3D | null = hits[0].object;
      while (o && o.userData.id === undefined) o = o.parent;
      id = o ? (o.userData.id as number) : null;
    }
    this.select(id);
  }

  select(id: number | null) {
    this.selected = id;
    this.onSelect(id);
  }

  // ------------------------------------------------------------------ render loop

  private frame() {
    const dt = Math.min(this.clock.getDelta(), 0.1);
    const now = performance.now() / 1000;
    const k = 1 - Math.exp(-dt * 10);
    for (const a of this.actors.values()) {
      const o = a.target;
      // Dead-reckon from the last message so motion stays smooth between updates.
      const ahead = Math.min(now - a.received, 0.3);
      const px = o.x + Math.cos(o.yaw) * o.speed * ahead;
      const py = o.y + Math.sin(o.yaw) * o.speed * ahead;
      const tgt = toV3(px, py);
      a.group.position.lerp(tgt, k);
      let dy = o.yaw - a.group.rotation.y;
      dy = Math.atan2(Math.sin(dy), Math.cos(dy));
      a.group.rotation.y += dy * k;
      const base = a.model.userData.base as THREE.Vector3;
      a.model.scale.set(o.length / base.x, Math.max(o.height, 0.5) / base.y, o.width / base.z);
      const sel = this.selected === a.id;
      a.glow.visible = sel || o.queued === true;
      a.glow.scale.set(o.length + 2.5, o.width + 2.5, 1);
      (a.glow.material as THREE.MeshBasicMaterial).opacity = sel ? 1 : 0.35;
      const alpha = o.state === "predicted" ? 0.45 : 1;
      if (a.model.userData.alpha !== alpha) {
        a.model.userData.alpha = alpha;
        a.model.traverse((m) => {
          if (m instanceof THREE.Mesh) {
            const mat = m.material as THREE.MeshStandardMaterial;
            mat.transparent = alpha < 1;
            mat.opacity = alpha;
          }
        });
      }
      a.label.position.set(0, Math.max(o.height, 1.5) + 0.8, 0);
      const showLabel = this.showAllLabels || sel;
      a.label.element.style.display = showLabel ? "" : "none";
      if (showLabel) {
        const eta = o.eta !== undefined && o.dist_to_stop !== undefined && o.dist_to_stop > 0 ? ` · ETA ${o.eta.toFixed(1)}s` : "";
        a.label.element.textContent = sel ? `${CLASS_LABEL[o.class]} #${o.id} · ${Math.round(Math.abs(o.speed_kmh))} km/h${eta}` : `${Math.round(Math.abs(o.speed_kmh))} km/h`;
        a.label.element.className = `tag ${sel ? "sel" : ""}`;
      }
    }
    for (const r of [...this.eventsGroup.children]) {
      const age = this.clock.elapsedTime - (r.userData.born as number);
      if (age > 6) {
        this.eventsGroup.remove(r);
        continue;
      }
      const s = 1 + 0.6 * Math.sin(age * 6);
      r.scale.set(s, s, s);
      ((r as THREE.Mesh).material as THREE.MeshBasicMaterial).opacity = Math.max(0, 0.9 - age / 6);
    }
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
    this.labels.render(this.scene, this.camera);
  }
}
