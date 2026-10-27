import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { CSS2DObject, CSS2DRenderer } from "three/examples/jsm/renderers/CSS2DRenderer.js";
import { aerialGround } from "./aerial";
import { STYLE, buildStylizedMap } from "./osmroads";
import { buildModel, glowTexture } from "./models";
import type { CameraJson, DetectorsMsg, NearMiss, ObjectClass, OcclusionMsg, RoadUser, SiteMap } from "./types";

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
  private glowTex = glowTexture();
  private cam?: CameraJson;
  private aerial?: THREE.Mesh;
  private aerialKey = "";
  private stylized?: THREE.Group;
  private stylizedKey = "";
  private ground!: THREE.Mesh;
  private grid!: THREE.GridHelper;
  private anchor?: { lat: number; lon: number };
  // "stylized": roads drawn from OpenStreetMap (default); "aerial": satellite imagery.
  mapMode: "stylized" | "aerial" = "stylized";
  private clock = new THREE.Clock();
  selected: number | null = null;
  showAllLabels = false;
  onSelect: (id: number | null) => void = () => {};

  constructor(private host: HTMLElement) {
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
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

    this.scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x1a1c20, 1.1));
    const key = new THREE.DirectionalLight(0xffffff, 1.6);
    key.position.set(-60, 120, 40);
    this.scene.add(key);

    const ground = (this.ground = new THREE.Mesh(
      new THREE.PlaneGeometry(4000, 4000),
      new THREE.MeshStandardMaterial({ color: 0x15181d, roughness: 1 }),
    ));
    ground.rotation.x = -Math.PI / 2;
    ground.position.y = -0.02;
    this.scene.add(ground);
    const grid = (this.grid = new THREE.GridHelper(1200, 120, 0x1f242b, 0x1a1e24));
    grid.position.y = -0.01;
    this.scene.add(grid);
    this.scene.add(this.mapGroup, this.eventsGroup, this.sensor);

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

  // Base map under the scene: the stylized OpenStreetMap roads or the aerial imagery (loaded on demand,
  // once per camera position).
  private updateBaseMap() {
    const stylized = this.mapMode === "stylized";
    const bg = stylized ? STYLE.background : 0x0c0e12;
    this.scene.background = new THREE.Color(bg);
    (this.scene.fog as THREE.Fog).color.setHex(bg);
    (this.ground.material as THREE.MeshStandardMaterial).color.setHex(stylized ? STYLE.ground : 0x15181d);
    this.grid.visible = !stylized && !this.aerial;
    if (this.stylized) this.stylized.visible = stylized;
    if (this.aerial) this.aerial.visible = !stylized;
    const a = this.anchor;
    if (!a) return;
    const key = `${a.lat.toFixed(6)},${a.lon.toFixed(6)}`;
    if (stylized && key !== this.stylizedKey) {
      this.stylizedKey = key;
      buildStylizedMap(a.lat, a.lon).then((g) => {
        if (!g || key !== this.stylizedKey) return;
        if (this.stylized) this.scene.remove(this.stylized);
        this.stylized = g;
        g.visible = this.mapMode === "stylized";
        this.scene.add(g);
      });
    }
    if (!stylized && key !== this.aerialKey) {
      this.aerialKey = key;
      aerialGround(a.lat, a.lon).then((m) => {
        if (!m || key !== this.aerialKey) return;
        if (this.aerial) this.scene.remove(this.aerial);
        this.aerial = m;
        m.visible = this.mapMode === "aerial";
        this.grid.visible = false;
        this.scene.add(m);
      });
    }
  }

  setMap(map: SiteMap, cam: CameraJson) {
    this.setAerial(cam);
    this.cam = cam;
    this.mapGroup.clear();
    this.zoneMeshes.clear();
    // Over aerial imagery the lanes are a light overlay, so the real road stays visible.
    const asphalt = new THREE.MeshStandardMaterial({ color: 0x2b2f36, roughness: 0.95, transparent: true, opacity: cam.anchor ? 0.35 : 1 });
    const lineMat = new THREE.LineDashedMaterial({ color: 0xc9ced6, dashSize: 3, gapSize: 6 });
    const solidMat = new THREE.LineBasicMaterial({ color: 0xc9ced6 });
    for (const lane of map.lanes) {
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
          new THREE.MeshBasicMaterial({ color: lane.to_junction ? 0x4e8cff : 0x5b6470, transparent: true, opacity: 0.55 }),
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
      void solidMat;
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
        a.label.element.textContent = `${Math.round(o.speed_kmh)} km/h${eta}`;
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
