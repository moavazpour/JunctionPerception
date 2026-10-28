// Aerial imagery as the ground of the digital twin: Web-Mercator tiles around the camera, stitched on a
// canvas and laid out in the local ENU frame (origin = ground point below the camera, x east, y north).
import * as THREE from "three";

const R = 6378137.0;
const TILE = 256;
const URL = (z: number, x: number, y: number) => `https://mt${(x + y) % 4}.google.com/vt/lyrs=s&x=${x}&y=${y}&z=${z}`;

const lon2x = (lon: number, z: number) => ((lon + 180) / 360) * 2 ** z;
const lat2y = (lat: number, z: number) => {
  const r = (lat * Math.PI) / 180;
  return ((1 - Math.log(Math.tan(r) + 1 / Math.cos(r)) / Math.PI) / 2) * 2 ** z;
};
const x2lon = (x: number, z: number) => (x / 2 ** z) * 360 - 180;
const y2lat = (y: number, z: number) => (Math.atan(Math.sinh(Math.PI * (1 - (2 * y) / 2 ** z))) * 180) / Math.PI;

function load(src: string): Promise<HTMLImageElement | null> {
  return new Promise((res) => {
    const im = new Image();
    im.crossOrigin = "anonymous";
    im.onload = () => res(im);
    im.onerror = () => res(null);
    im.src = src;
  });
}

// Ground mesh of roughly `radius` metres around (lat, lon), at the sharpest zoom (up to `maxZoom`) whose
// stitched image fits one texture; null when the imagery cannot be loaded.
export async function aerialGround(lat: number, lon: number, maxTex = 4096, radius = 260, maxZoom = 20): Promise<THREE.Mesh | null> {
  const dLat = ((radius / R) * 180) / Math.PI;
  const dLon = dLat / Math.cos((lat * Math.PI) / 180);
  let zoom = maxZoom, x0 = 0, x1 = 0, y0 = 0, y1 = 0;
  for (; zoom > 14; zoom--) {
    x0 = Math.floor(lon2x(lon - dLon, zoom));
    x1 = Math.floor(lon2x(lon + dLon, zoom));
    y0 = Math.floor(lat2y(lat + dLat, zoom));
    y1 = Math.floor(lat2y(lat - dLat, zoom));
    if (Math.max(x1 - x0 + 1, y1 - y0 + 1) * TILE <= maxTex) break;
  }
  const nx = x1 - x0 + 1, ny = y1 - y0 + 1;
  const canvas = document.createElement("canvas");
  canvas.width = nx * TILE;
  canvas.height = ny * TILE;
  const ctx = canvas.getContext("2d")!;
  const jobs: Promise<void>[] = [];
  let loaded = 0;
  for (let ty = y0; ty <= y1; ty++)
    for (let tx = x0; tx <= x1; tx++)
      jobs.push(
        load(URL(zoom, tx, ty)).then((im) => {
          if (im) {
            ctx.drawImage(im, (tx - x0) * TILE, (ty - y0) * TILE);
            loaded++;
          }
        }),
      );
  await Promise.all(jobs);
  if (!loaded) return null;
  // Canvas bounds in ENU metres (local tangent plane; error < 1 cm over a few hundred metres).
  const kx = (R * Math.PI * Math.cos((lat * Math.PI) / 180)) / 180, ky = (R * Math.PI) / 180;
  const west = (x2lon(x0, zoom) - lon) * kx, east = (x2lon(x1 + 1, zoom) - lon) * kx;
  const north = (y2lat(y0, zoom) - lat) * ky, south = (y2lat(y1 + 1, zoom) - lat) * ky;
  const tex = new THREE.CanvasTexture(canvas);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 8;
  const mesh = new THREE.Mesh(
    new THREE.PlaneGeometry(east - west, north - south),
    new THREE.MeshBasicMaterial({ map: tex, color: 0xb8bcc4 }),
  );
  mesh.rotation.x = -Math.PI / 2; // plane +y -> scene -z = north
  mesh.position.set((east + west) / 2, -0.015, -(north + south) / 2);
  return mesh;
}
