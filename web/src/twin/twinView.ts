import * as THREE from 'three';

import type { ClassInfo, SceneDescription } from '../net/client';
import type { MapData } from '../map/mapData';
import { buildBuildingLayer, buildOrthoLayer, buildStylizedLayer } from '../map/mapLayers';
import type { WorldState } from '../net/schema';
import { BevCameraRig } from './cameraRig';
import { WorldInterpolator } from './interpolation';
import { buildClassMesh, buildConflictHalo, disposeObject, makeMaterials, PALETTE } from './meshes';

const GROUND_SIZE_M = 2000;
const GRID_SPACING_M = 10;
const GRID_EXTENT_M = 400;
const INITIAL_VIEW_M = 90;
const HALO_PULSE_HZ = 1.2;
const ZONE_OPACITY = 0.45;

export type BaseLayer = 'map' | 'ortho';

interface TrackedMesh {
  objectClass: string;
  highlighted: boolean;
  validated: boolean;
  root: THREE.Group;
  model: THREE.Group;
  halo: THREE.Mesh | null;
}

/** Ids of the objects in active conflict events and in any active event. */
export function eventParticipants(state: WorldState): { conflict: Set<number>; any: Set<number> } {
  const conflict = new Set<number>();
  const any = new Set<number>();
  for (const event of state.events) {
    if (event.state !== 'active') {
      continue;
    }
    for (const id of event.object_ids) {
      any.add(id);
      if (event.type === 'VRU_VEHICLE_CONFLICT') {
        conflict.add(id);
      }
    }
  }
  return { conflict, any };
}

/**
 * Centre of all zone corners; without zones the camera position, or the world origin (the geo
 * anchor) for scenes that are not calibrated yet.
 */
function initialCenter(scene: SceneDescription): { x: number; y: number } {
  const corners = scene.zones.features.flatMap((f) => f.geometry.coordinates[0] ?? []);
  if (corners.length === 0) {
    const [x, y] = scene.camera_ground_position_m ?? [0, 0];
    return { x, y };
  }
  const sum = corners.reduce((acc, [x = 0, y = 0]) => ({ x: acc.x + x, y: acc.y + y }), {
    x: 0,
    y: 0,
  });
  return { x: sum.x / corners.length, y: sum.y / corners.length };
}

/**
 * Top-down digital twin: light ground with a 10 m grid, scene zones, the camera position and one
 * low-poly mesh per road user, moved smoothly between world states.
 */
export class TwinView {
  private readonly renderer: THREE.WebGLRenderer;
  private readonly scene = new THREE.Scene();
  private readonly rig: BevCameraRig;
  private readonly sun = new THREE.DirectionalLight(0xffffff, 1.4);
  private readonly interpolator = new WorldInterpolator();
  private grid: THREE.LineSegments | null = null;
  private stylized: THREE.Group | null = null;
  private buildings: THREE.Group | null = null;
  private ortho: THREE.Mesh | null = null;
  private readonly meshes = new Map<number, TrackedMesh>();
  private readonly classes = new Map<string, ClassInfo>();
  private readonly fallbackClass: ClassInfo = {
    name: 'other',
    length_m: 2,
    width_m: 1,
    height_m: 1.5,
    vru: false,
  };

  constructor(
    private readonly canvas: HTMLCanvasElement,
    description: SceneDescription,
  ) {
    for (const info of description.classes) {
      this.classes.set(info.name, info);
    }
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.scene.background = new THREE.Color(PALETTE.background);

    const center = initialCenter(description);
    const camera = description.camera_ground_position_m;
    this.rig = new BevCameraRig(canvas, {
      centerX: center.x,
      centerY: center.y,
      viewHeightM: INITIAL_VIEW_M,
      // The camera's viewing direction points up on screen, as in the camera image.
      rotationRad: camera
        ? Math.atan2(center.y - camera[1], center.x - camera[0]) - Math.PI / 2
        : 0,
    });
    this.addLights(center);
    this.addGround(center);
    this.addZones(description);
    this.addCameraMarker(description);
    window.addEventListener('resize', () => this.resize());
    this.resize();
    this.renderer.setAnimationLoop(() => this.render());
  }

  /** Adds the map layers of the scene; the stylized map is shown first. */
  setMap(map: MapData): void {
    this.stylized = buildStylizedLayer(map);
    this.buildings = buildBuildingLayer(map);
    this.ortho = buildOrthoLayer(map);
    this.scene.add(this.stylized, this.buildings);
    if (this.ortho) {
      this.scene.add(this.ortho);
    }
    this.showLayer('map');
  }

  /** Base layer: stylized map (with grid) or orthophoto. */
  showLayer(layer: BaseLayer): void {
    if (this.stylized) {
      this.stylized.visible = layer === 'map';
    }
    if (this.ortho) {
      this.ortho.visible = layer === 'ortho';
    }
    if (this.grid) {
      this.grid.visible = layer === 'map';
    }
  }

  showBuildings(visible: boolean): void {
    if (this.buildings) {
      this.buildings.visible = visible;
    }
  }

  /** Feeds a new world state. */
  push(state: WorldState): void {
    this.interpolator.push(state, performance.now());
  }

  private resize(): void {
    this.renderer.setSize(this.canvas.clientWidth, this.canvas.clientHeight, false);
    this.rig.update();
  }

  private addLights(center: { x: number; y: number }): void {
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0xd9dce1, 1.6));
    this.sun.position.set(center.x - 60, center.y - 90, 160);
    this.sun.target.position.set(center.x, center.y, 0);
    this.sun.castShadow = true;
    this.sun.shadow.mapSize.set(4096, 4096);
    this.sun.shadow.radius = 4;
    const extent = 150;
    Object.assign(this.sun.shadow.camera, {
      left: -extent,
      right: extent,
      top: extent,
      bottom: -extent,
      near: 1,
      far: 500,
    });
    this.scene.add(this.sun, this.sun.target);
  }

  private addGround(center: { x: number; y: number }): void {
    const ground = new THREE.Mesh(
      new THREE.PlaneGeometry(GROUND_SIZE_M, GROUND_SIZE_M),
      new THREE.MeshStandardMaterial({ color: PALETTE.ground, roughness: 1 }),
    );
    ground.position.set(center.x, center.y, 0);
    ground.receiveShadow = true;
    this.scene.add(ground);

    const points: THREE.Vector3[] = [];
    const x0 = Math.round(center.x / GRID_SPACING_M) * GRID_SPACING_M;
    const y0 = Math.round(center.y / GRID_SPACING_M) * GRID_SPACING_M;
    for (let d = -GRID_EXTENT_M; d <= GRID_EXTENT_M; d += GRID_SPACING_M) {
      points.push(new THREE.Vector3(x0 + d, y0 - GRID_EXTENT_M, 0.01));
      points.push(new THREE.Vector3(x0 + d, y0 + GRID_EXTENT_M, 0.01));
      points.push(new THREE.Vector3(x0 - GRID_EXTENT_M, y0 + d, 0.01));
      points.push(new THREE.Vector3(x0 + GRID_EXTENT_M, y0 + d, 0.01));
    }
    this.grid = new THREE.LineSegments(
      new THREE.BufferGeometry().setFromPoints(points),
      new THREE.LineBasicMaterial({ color: PALETTE.grid }),
    );
    this.scene.add(this.grid);
  }

  private addZones(description: SceneDescription): void {
    for (const feature of description.zones.features) {
      const ring = feature.geometry.coordinates[0] ?? [];
      const shape = new THREE.Shape(ring.map(([x = 0, y = 0]) => new THREE.Vector2(x, y)));
      const fill = new THREE.Mesh(
        new THREE.ShapeGeometry(shape),
        // Translucent, so the base layer (road markings, orthophoto) stays visible under zones.
        new THREE.MeshStandardMaterial({
          color: PALETTE.zoneFill,
          roughness: 1,
          transparent: true,
          opacity: ZONE_OPACITY,
          depthWrite: false,
        }),
      );
      fill.position.z = 0.015;
      fill.receiveShadow = true;
      const edge = new THREE.LineLoop(
        new THREE.BufferGeometry().setFromPoints(
          ring.map(([x = 0, y = 0]) => new THREE.Vector3(x, y, 0.03)),
        ),
        new THREE.LineBasicMaterial({ color: PALETTE.zoneEdge }),
      );
      this.scene.add(fill, edge);
    }
  }

  private addCameraMarker(description: SceneDescription): void {
    if (!description.camera_ground_position_m) {
      return;
    }
    const [x, y] = description.camera_ground_position_m;
    const marker = new THREE.Mesh(
      new THREE.ConeGeometry(1.2, 2.4, 3),
      new THREE.MeshStandardMaterial({ color: 0x5b6169 }),
    );
    marker.rotation.x = Math.PI / 2;
    marker.position.set(x, y, 1.2);
    this.scene.add(marker);
  }

  private classInfo(name: string): ClassInfo {
    return this.classes.get(name) ?? this.fallbackClass;
  }

  private ensureMesh(
    id: number,
    objectClass: string,
    highlighted: boolean,
    validated: boolean,
  ): TrackedMesh {
    const existing = this.meshes.get(id);
    if (
      existing &&
      existing.objectClass === objectClass &&
      existing.highlighted === highlighted &&
      existing.validated === validated
    ) {
      return existing;
    }
    const root = existing?.root ?? new THREE.Group();
    if (existing) {
      root.remove(existing.model);
      disposeObject(existing.model);
    } else {
      this.scene.add(root);
    }
    const model = buildClassMesh(
      this.classInfo(objectClass),
      makeMaterials(highlighted, validated),
    );
    root.add(model);
    const tracked = {
      objectClass,
      highlighted,
      validated,
      root,
      model,
      halo: existing?.halo ?? null,
    };
    this.meshes.set(id, tracked);
    return tracked;
  }

  private render(): void {
    const state = this.interpolator.current();
    if (state) {
      const now = performance.now();
      const poses = this.interpolator.poses(now);
      const { conflict, any } = eventParticipants(state);
      const pulse = 0.3 + 0.2 * Math.sin((now / 1000) * 2 * Math.PI * HALO_PULSE_HZ);
      for (const object of state.objects) {
        const pose = poses.get(object.id);
        if (!pose) {
          continue;
        }
        const tracked = this.ensureMesh(
          object.id,
          object.class,
          object.vru || any.has(object.id),
          object.validated,
        );
        tracked.root.position.set(pose.x, pose.y, 0);
        tracked.model.rotation.z = pose.heading;
        if (conflict.has(object.id) && !tracked.halo) {
          tracked.halo = buildConflictHalo(this.classInfo(object.class));
          tracked.root.add(tracked.halo);
        } else if (!conflict.has(object.id) && tracked.halo) {
          tracked.root.remove(tracked.halo);
          disposeObject(tracked.halo);
          tracked.halo = null;
        }
        if (tracked.halo) {
          (tracked.halo.material as THREE.MeshBasicMaterial).opacity = pulse;
        }
      }
      const present = new Set(state.objects.map((o) => o.id));
      for (const [id, tracked] of this.meshes) {
        if (!present.has(id)) {
          this.scene.remove(tracked.root);
          disposeObject(tracked.root);
          this.meshes.delete(id);
        }
      }
    }
    this.renderer.render(this.scene, this.rig.camera);
  }
}
