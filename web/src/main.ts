import * as THREE from 'three';

import { bevFrustum } from './twin/bevFrustum';

// World frame: local ENU, x east, y north, z up, ground at z = 0.
const VIEW_HEIGHT_M = 120;
const CAMERA_ALTITUDE_M = 200;
const GROUND_SIZE_M = 1000;

const canvas = document.querySelector<HTMLCanvasElement>('#twin');
if (!canvas) {
  throw new Error('canvas #twin not found');
}

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0xe9eaec);
scene.add(new THREE.HemisphereLight(0xffffff, 0xd0d2d6, 2.0));
const sun = new THREE.DirectionalLight(0xffffff, 1.0);
sun.position.set(-50, -80, 150);
scene.add(sun);

const ground = new THREE.Mesh(
  new THREE.PlaneGeometry(GROUND_SIZE_M, GROUND_SIZE_M),
  new THREE.MeshStandardMaterial({ color: 0xf2f3f5, roughness: 1.0 }),
);
scene.add(ground);

const camera = new THREE.OrthographicCamera();
camera.up.set(0, 1, 0);
camera.position.set(0, 0, CAMERA_ALTITUDE_M);
camera.lookAt(0, 0, 0);
camera.near = 0.1;
camera.far = CAMERA_ALTITUDE_M * 2;

function resize(): void {
  const width = window.innerWidth;
  const height = window.innerHeight;
  renderer.setSize(width, height, false);
  Object.assign(camera, bevFrustum(VIEW_HEIGHT_M, width / height));
  camera.updateProjectionMatrix();
  renderer.render(scene, camera);
}

window.addEventListener('resize', resize);
resize();
