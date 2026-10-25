// Procedural low-poly road-user models in a clean, light-grey "digital twin" style.
// Every model is built in metres with +x forward, +y up, centred on the footprint; the caller scales
// it to the measured length/width/height.
import * as THREE from "three";
import { RoundedBoxGeometry } from "three/examples/jsm/geometries/RoundedBoxGeometry.js";
import type { ObjectClass } from "./types";

const body = new THREE.MeshStandardMaterial({ color: 0xb4bac2, roughness: 0.42, metalness: 0.25 });
const bodyDark = new THREE.MeshStandardMaterial({ color: 0x8d949c, roughness: 0.5, metalness: 0.2 });
const glass = new THREE.MeshStandardMaterial({ color: 0x1c2229, roughness: 0.15, metalness: 0.6 });
const tyre = new THREE.MeshStandardMaterial({ color: 0x15171a, roughness: 0.9 });
const tail = new THREE.MeshStandardMaterial({ color: 0xff2a2a, emissive: 0xff1010, emissiveIntensity: 2.2 });
const head = new THREE.MeshStandardMaterial({ color: 0xffffff, emissive: 0xe8f0ff, emissiveIntensity: 1.2 });
const person = new THREE.MeshStandardMaterial({ color: 0xe6e8eb, roughness: 0.6 });

function wheel(r: number, w: number): THREE.Mesh {
  const g = new THREE.CylinderGeometry(r, r, w, 18);
  g.rotateX(Math.PI / 2);
  return new THREE.Mesh(g, tyre);
}

function addWheels(g: THREE.Group, l: number, w: number, r: number, axles: number[]) {
  for (const x of axles)
    for (const side of [-1, 1]) {
      const m = wheel(r, 0.22);
      m.position.set(x * l, r, side * (w / 2 - 0.1));
      g.add(m);
    }
}

function lights(g: THREE.Group, l: number, w: number, h: number) {
  const t = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.08, w * 0.8), tail);
  t.position.set(-l / 2 - 0.01, h, 0);
  g.add(t);
  for (const s of [-1, 1]) {
    const hl = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.07, 0.3), head);
    hl.position.set(l / 2 + 0.01, h * 0.95, s * (w / 2 - 0.25));
    g.add(hl);
  }
}

function car(): THREE.Group {
  // Side silhouette of a hatchback/sedan, extruded across the width.
  const L = 4.5, W = 1.85, H = 1.45;
  const s = new THREE.Shape();
  s.moveTo(-L / 2, 0.32);
  s.lineTo(-L / 2, 0.85);
  s.quadraticCurveTo(-L / 2 + 0.15, 1.0, -L / 2 + 0.6, 1.05);
  s.lineTo(-L / 2 + 1.1, H);
  s.lineTo(L / 2 - 1.75, H);
  s.quadraticCurveTo(L / 2 - 1.2, H - 0.05, L / 2 - 0.85, 0.92);
  s.lineTo(L / 2 - 0.1, 0.8);
  s.quadraticCurveTo(L / 2, 0.7, L / 2, 0.5);
  s.lineTo(L / 2, 0.32);
  s.closePath();
  const g = new THREE.Group();
  const geo = new THREE.ExtrudeGeometry(s, { depth: W, bevelEnabled: true, bevelSize: 0.06, bevelThickness: 0.08, bevelSegments: 3 });
  geo.translate(0, 0, -W / 2);
  g.add(new THREE.Mesh(geo, body));
  // Glasshouse.
  const gs = new THREE.Shape();
  gs.moveTo(-L / 2 + 0.7, 1.06);
  gs.lineTo(-L / 2 + 1.15, H - 0.05);
  gs.lineTo(L / 2 - 1.8, H - 0.05);
  gs.lineTo(L / 2 - 1.0, 0.95);
  gs.closePath();
  const gg = new THREE.ExtrudeGeometry(gs, { depth: W + 0.02, bevelEnabled: false });
  gg.translate(0, 0, -(W + 0.02) / 2);
  g.add(new THREE.Mesh(gg, glass));
  addWheels(g, L, W, 0.33, [0.31, -0.3]);
  lights(g, L, W, 0.8);
  g.userData.base = new THREE.Vector3(L, H, W);
  return g;
}

function boxVehicle(L: number, W: number, H: number, opts: { windows?: boolean; cab?: boolean; axles: number[] }): THREE.Group {
  const g = new THREE.Group();
  const clearance = 0.35;
  if (opts.cab) {
    const cab = new THREE.Mesh(new RoundedBoxGeometry(1.9, H - clearance - 0.3, W, 3, 0.15), body);
    cab.position.set(L / 2 - 0.95, clearance + (H - clearance - 0.3) / 2, 0);
    g.add(cab);
    const ws = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.8, W * 0.85), glass);
    ws.position.set(L / 2 + 0.01, H - 1.0, 0);
    g.add(ws);
    const cargo = new THREE.Mesh(new RoundedBoxGeometry(L - 2.1, H - clearance, W, 2, 0.08), bodyDark);
    cargo.position.set(-1.05, clearance + (H - clearance) / 2, 0);
    g.add(cargo);
  } else {
    const b = new THREE.Mesh(new RoundedBoxGeometry(L, H - clearance, W, 3, 0.18), body);
    b.position.set(0, clearance + (H - clearance) / 2, 0);
    g.add(b);
    if (opts.windows) {
      for (const side of [-1, 1]) {
        const wnd = new THREE.Mesh(new THREE.BoxGeometry(L * 0.86, 0.75, 0.04), glass);
        wnd.position.set(0.1, H - 0.85, side * (W / 2 + 0.01));
        g.add(wnd);
      }
      const ws = new THREE.Mesh(new THREE.BoxGeometry(0.04, 1.1, W * 0.9), glass);
      ws.position.set(L / 2 + 0.01, H - 1.0, 0);
      g.add(ws);
    }
  }
  addWheels(g, L, W, 0.48, opts.axles);
  lights(g, L, W, 0.9);
  g.userData.base = new THREE.Vector3(L, H, W);
  return g;
}

function human(): THREE.Group {
  const g = new THREE.Group();
  const torso = new THREE.Mesh(new THREE.CapsuleGeometry(0.17, 0.55, 4, 10), person);
  torso.position.y = 1.15;
  g.add(torso);
  const headM = new THREE.Mesh(new THREE.SphereGeometry(0.12, 14, 10), person);
  headM.position.y = 1.62;
  g.add(headM);
  for (const s of [-1, 1]) {
    const leg = new THREE.Mesh(new THREE.CapsuleGeometry(0.07, 0.6, 3, 8), person);
    leg.position.set(0, 0.42, s * 0.1);
    g.add(leg);
  }
  g.userData.base = new THREE.Vector3(0.6, 1.75, 0.6);
  return g;
}

function rider(seatHeight: number): THREE.Group {
  const g = new THREE.Group();
  const torso = new THREE.Mesh(new THREE.CapsuleGeometry(0.16, 0.5, 4, 10), person);
  torso.position.set(-0.05, seatHeight + 0.4, 0);
  torso.rotation.z = -0.25;
  g.add(torso);
  const headM = new THREE.Mesh(new THREE.SphereGeometry(0.12, 14, 10), person);
  headM.position.set(0.1, seatHeight + 0.85, 0);
  g.add(headM);
  return g;
}

function twoWheeler(kind: "bicycle" | "escooter" | "motorcycle"): THREE.Group {
  const g = new THREE.Group();
  const r = kind === "bicycle" ? 0.34 : kind === "escooter" ? 0.12 : 0.32;
  const L = kind === "bicycle" ? 1.75 : kind === "escooter" ? 1.15 : 2.1;
  for (const x of [-L / 2 + r, L / 2 - r]) {
    const w = new THREE.Mesh(new THREE.TorusGeometry(r, kind === "motorcycle" ? 0.08 : 0.035, 8, 24), tyre);
    w.position.set(x, r, 0);
    g.add(w);
  }
  if (kind === "escooter") {
    const deck = new THREE.Mesh(new THREE.BoxGeometry(0.8, 0.05, 0.16), bodyDark);
    deck.position.set(0, 0.15, 0);
    g.add(deck);
    const stem = new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.025, 1.0), bodyDark);
    stem.position.set(L / 2 - r, 0.65, 0);
    g.add(stem);
    const torso = new THREE.Mesh(new THREE.CapsuleGeometry(0.16, 0.55, 4, 10), person);
    torso.position.set(0, 1.0, 0);
    g.add(torso);
    const legs = new THREE.Mesh(new THREE.CapsuleGeometry(0.1, 0.55, 3, 8), person);
    legs.position.set(0, 0.5, 0);
    g.add(legs);
    const headM = new THREE.Mesh(new THREE.SphereGeometry(0.12, 14, 10), person);
    headM.position.set(0, 1.5, 0);
    g.add(headM);
  } else {
    const frame = new THREE.Mesh(
      kind === "motorcycle" ? new RoundedBoxGeometry(1.2, 0.45, 0.4, 2, 0.1) : new THREE.BoxGeometry(1.0, 0.04, 0.04),
      bodyDark,
    );
    frame.position.set(0, kind === "motorcycle" ? 0.6 : 0.6, 0);
    g.add(frame);
    g.add(rider(kind === "motorcycle" ? 0.8 : 0.85));
  }
  g.userData.base = new THREE.Vector3(L, 1.7, 0.6);
  return g;
}

export function buildModel(cls: ObjectClass): THREE.Group {
  switch (cls) {
    case "car":
      return car();
    case "bus":
      return boxVehicle(12, 2.55, 3.1, { windows: true, axles: [0.3, -0.25] });
    case "truck":
      return boxVehicle(7, 2.5, 3.2, { cab: true, axles: [0.33, -0.25] });
    case "trailer":
      return boxVehicle(10.5, 2.55, 3.4, { axles: [-0.25, -0.35, -0.45] });
    case "pedestrian":
      return human();
    case "bicycle":
    case "escooter":
    case "motorcycle":
      return twoWheeler(cls);
  }
}

// Soft blue glow drawn on the ground under a selected object.
export function glowTexture(): THREE.Texture {
  const c = document.createElement("canvas");
  c.width = c.height = 128;
  const ctx = c.getContext("2d")!;
  const g = ctx.createRadialGradient(64, 64, 8, 64, 64, 64);
  g.addColorStop(0, "rgba(70,140,255,0.95)");
  g.addColorStop(0.55, "rgba(40,110,255,0.45)");
  g.addColorStop(1, "rgba(30,90,255,0)");
  ctx.fillStyle = g;
  ctx.fillRect(0, 0, 128, 128);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}
