// 3D-fidelity harness: drives the twin page's own TwinView (window.jpBench.twin, ?bench=1) with
// recorded world states on a simulated clock and compares what it draws with the backend state
// and with ground truth. Runtime instrumentation only: no source file of the twin is changed.

// Where bundle.json and osm.json are served (same origin as the twin page).
const BASE = window.FIDELITY_BASE ?? '/sites/tumtraf_s110_south2_s03/map/_fidelity';
const FRAME_MS = 1000 / 60;
const LATENCY_MS = 200; // capture -> browser (production status: 130-250 ms)
const GT_EDGE_MS = 150; // ground truth is not extrapolated beyond its track by more than this

export const results = { status: 'idle', progress: '', records: {}, log: [] };

const yieldNow = () =>
  new Promise((resolve) => {
    const channel = new MessageChannel();
    channel.port1.onmessage = () => resolve();
    channel.port2.postMessage(0);
  });

function gtAt(track, t) {
  if (!track || track.length === 0) return null;
  if (t < track[0][0] - GT_EDGE_MS * 1e6 || t > track[track.length - 1][0] + GT_EDGE_MS * 1e6) return null;
  let lo = 0;
  let hi = track.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (track[mid][0] <= t) lo = mid;
    else hi = mid;
  }
  const a = track[lo];
  const b = track[hi];
  const k = b[0] === a[0] ? 0 : Math.min(Math.max((t - a[0]) / (b[0] - a[0]), 0), 1);
  const near = k < 0.5 ? a : b;
  return { x: a[1] + (b[1] - a[1]) * k, y: a[2] + (b[2] - a[2]) * k, yaw: near[3], l: near[4], w: near[5], cat: near[6] };
}

function corners(x, y, h, l, w) {
  const c = Math.cos(h);
  const s = Math.sin(h);
  return [
    [x + c * l / 2 - s * w / 2, y + s * l / 2 + c * w / 2],
    [x + c * l / 2 + s * w / 2, y + s * l / 2 - c * w / 2],
    [x - c * l / 2 + s * w / 2, y - s * l / 2 - c * w / 2],
    [x - c * l / 2 - s * w / 2, y - s * l / 2 + c * w / 2],
  ];
}

/** Mean corner distance of two equal-size rectangles, ignoring front/back (axis only). */
function axisCornerDistance(a, b) {
  const d = (p, q) => Math.hypot(p[0] - q[0], p[1] - q[1]);
  const same = a.reduce((s, p, i) => s + d(p, b[i]), 0) / 4;
  const flipped = a.reduce((s, p, i) => s + d(p, b[(i + 2) % 4]), 0) / 4;
  return Math.min(same, flipped);
}

const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const axisDeg = (a, b) => {
  const d = Math.abs(wrap(a - b));
  return (Math.min(d, Math.PI - d) * 180) / Math.PI;
};
const fullDeg = (a, b) => (Math.abs(wrap(a - b)) * 180) / Math.PI;

export async function run(options = {}) {
  results.status = 'running';
  const twin = window.jpBench.twin;
  const bundle = await (await fetch(`${BASE}/bundle.json`)).json();
  const osm = await (await fetch(`${BASE}/osm.json`)).json();
  // Class catalog of a scene served on this origin.
  const scene = await (
    await fetch(window.FIDELITY_SCENE ?? '/sites/tumtraf_s110_south2_s03/cameras/south2/api/scene')
  ).json();
  const Vector3 = twin.rig.camera.position.constructor;
  twin.ground.geometry.computeBoundingBox();
  const Box3 = twin.ground.geometry.boundingBox.constructor;
  const realNow = performance.now.bind(performance);
  let sim = 0;
  performance.now = () => sim;
  twin.setActive(false);
  const map = {
    base: '',
    manifest: { scene_id: 'fidelity', radius_m: 250, layers: {}, attribution: {} },
    ortho: null,
    roadmap: null,
    osm,
    buildings: [],
  };
  const description = (camera) => ({
    scene_id: 'fidelity',
    camera_id: camera,
    kind: 'benchmark',
    image_size_px: [1920, 1200],
    calibrated: true,
    camera_ground_position_m: bundle.cameras[camera],
    camera_height_m: 8.6,
    geo_anchor: null,
    map: null,
    zones: { type: 'FeatureCollection', features: [] },
    classes: scene.classes,
  });
  const classOf = (name) => twin.classes.get(name) ?? { length_m: 2, width_m: 1, height_m: 1.5 };

  let siteCount = 0;
  let sharedFlow = null;
  async function replay(name, cameras, delays, measure, fps = 60, skewMs = undefined) {
    const id = `fidelity-${name}-${siteCount++}`;
    // Fusion threshold of this run (builds with timestamp-gated fusion only).
    if (skewMs !== undefined) twin.setFusionMaxSkewMs?.(skewMs);
    twin.addSite({ id, name: id, offset: { x: 20000 + siteCount * 2000, y: 20000 }, transform: null });
    for (const c of cameras) twin.addCamera(id, c, description(c));
    twin.setMap(id, map);
    twin.setMapTransform(id, bundle.map_transform);
    const site = twin.sites.get(id);
    if (sharedFlow) site.flow = sharedFlow;
    else sharedFlow = site.flow;
    const arrivals = cameras
      .flatMap((c) => bundle.states[c].map((s) => ({ T: s.timestamp_ns / 1e6 + LATENCY_MS + (delays[c] ?? 0), c, s })))
      .sort((a, b) => a.T - b.T);
    const recs = { interp: [], fusion: [], lost: [], heading: [], persp: [], bevCheck: [], footprint: [], fev: [] };
    const lastMatch = new Map();
    let k = 0;
    sim = arrivals[0].T - 50;
    twin.render();
    twin.focus(id);
    for (let i = 0; i < 120; i++) twin.rig.tick(0.05);
    const end = arrivals[arrivals.length - 1].T + 1500;
    let frames = 0;
    for (let t = arrivals[0].T; t < end; t += 1000 / fps) {
      while (k < arrivals.length && arrivals[k].T <= t) {
        sim = arrivals[k].T;
        twin.push(id, arrivals[k].c, structuredClone(arrivals[k].s));
        k++;
      }
      sim = t;
      twin.render();
      frames++;
      if (measure) measureFrame(site, t, recs, lastMatch, measure, frames);
      if (frames % 300 === 0) {
        results.progress = `${name}: ${((100 * (t - arrivals[0].T)) / (end - arrivals[0].T)).toFixed(0)} %`;
        await yieldNow();
      }
    }
    recs.counts = twin.fusionCounts?.(id) ?? null;
    site.group.visible = false;
    return recs;
  }

  function memberOf(site, key) {
    // The camera track drawn for a road user: the object the mesh shows, in its camera's state.
    const tm = site.meshes.get(key);
    for (const [camera, feed] of site.cameras) {
      if (feed.latest?.objects.includes(tm.last)) return { camera, feed, object: tm.last };
    }
    return null;
  }

  /** Footprints (builds that draw them): each must sit exactly at its backend observation. */
  function checkFootprints(site, recs) {
    for (const tm of site.meshes.values()) {
      if (!tm.footprints) return;
      tm.footprints.forEach((fp, i) => {
        const o = tm.observed[i]?.object;
        if (!o) return;
        const p = fp.group.position;
        recs.footprint.push({
          lost: tm.lost !== null,
          d: Math.hypot(p.x - o.x, p.y - o.y),
          headingErr: o.heading_rad === null ? null : fullDeg(fp.group.rotation.z, o.heading_rad),
          disc: o.heading_rad === null,
          // Lost road users: the model must be frozen at the last measured position.
          frozen: tm.lost !== null ? Math.hypot(tm.x - tm.last.x, tm.y - tm.last.y) : null,
          lostAgeS: tm.lost !== null ? (sim - tm.lost) / 1000 : null,
        });
      });
    }
  }

  /**
   * Fusion evaluation per labelled road user and frame: which drawn road users (keys) show it,
   * whether both cameras see it, and how far each drawn model is from the ground truth now.
   */
  function evaluateFusion(site, t, recs) {
    const seen = new Map();
    for (const [camera, feed] of site.cameras) {
      const gids = new Set();
      for (const o of feed.latest?.objects ?? []) {
        const gid = bundle.matches[`${camera}:${feed.latest.frame}:${o.id}`];
        if (gid !== undefined) gids.add(gid);
      }
      seen.set(camera, gids);
    }
    const byGid = new Map();
    for (const [key, tm] of site.meshes) {
      if (tm.lost !== null) continue;
      const observed = tm.observed ?? [{ camera: memberOf(site, key)?.camera, object: tm.last }];
      for (const { camera, object } of observed) {
        const feed = site.cameras.get(camera);
        if (!feed?.latest) continue;
        const gid = bundle.matches[`${camera}:${feed.latest.frame}:${object.id}`];
        if (gid === undefined) continue;
        if (!byGid.has(gid)) byGid.set(gid, new Set());
        byGid.get(gid).add(key);
      }
    }
    for (const [gid, keys] of byGid) {
      const g = gtAt(bundle.gt[gid], t * 1e6);
      recs.fev.push({
        t,
        gid,
        keys: [...keys].sort(),
        both: [...seen.values()].every((s) => s.has(gid)),
        errs: g ? [...keys].map((k) => Math.hypot(site.meshes.get(k).x - g.x, site.meshes.get(k).y - g.y)) : [],
      });
    }
  }

  function measureFrame(site, t, recs, lastMatch, measure, frame) {
    if (measure.fusionOnly) {
      // Light mode for threshold sweeps: the fusion evaluation only, at half the frame rate.
      if (frame % 2 === 0) evaluateFusion(site, t, recs);
      return;
    }
    checkFootprints(site, recs);
    if (site.cameras.size > 1) evaluateFusion(site, t, recs);
    const keys = new Map();
    for (const [trackKey, roadUser] of site.fusion.alias) {
      if (!keys.has(roadUser)) keys.set(roadUser, []);
      keys.get(roadUser).push(trackKey);
    }
    for (const [key, tm] of site.meshes) {
      const px = tm.x;
      const py = tm.y;
      if (tm.lost !== null) {
        const known = lastMatch.get(key);
        const g = known ? gtAt(bundle.gt[known.gid], t) : null;
        // Reference: the last backend position of this track (what the BEV showed last).
        const lastPos = { x: tm.last.x, y: tm.last.y };
        recs.lost.push({
          age: t - tm.lost,
          d: Math.hypot(px - lastPos.x, py - lastPos.y),
          speed: tm.last.speed_mps,
          speedStd: tm.last.speed_std_mps,
          err3d: g ? Math.hypot(px - g.x, py - g.y) : null,
          errLast: g ? Math.hypot(lastPos.x - g.x, lastPos.y - g.y) : null,
        });
        continue;
      }
      const m = memberOf(site, key);
      if (!m) continue;
      const o = m.object;
      const capture = m.feed.latest.timestamp_ns;
      const gid = bundle.matches[`${m.camera}:${m.feed.latest.frame}:${o.id}`];
      if (gid !== undefined) lastMatch.set(key, { gid, pos: { x: o.x, y: o.y } });
      const gNow = gid !== undefined ? gtAt(bundle.gt[gid], t * 1e6) : null;
      const gCap = gid !== undefined ? gtAt(bundle.gt[gid], capture) : null;
      const interp = m.feed.interpolator;
      const tParam = Math.min(Math.max((t - interp.latestArrival) / interp.interval, 0), 1);
      const base = {
        vru: o.vru,
        speed: o.speed_mps,
        tParam,
        dBackend: Math.hypot(px - o.x, py - o.y),
        err3dNow: gNow ? Math.hypot(px - gNow.x, py - gNow.y) : null,
        errBevNow: gNow ? Math.hypot(o.x - gNow.x, o.y - gNow.y) : null,
        err3dCap: gCap ? Math.hypot(px - gCap.x, py - gCap.y) : null,
        errBevCap: gCap ? Math.hypot(o.x - gCap.x, o.y - gCap.y) : null,
      };
      if (tm.cameras.length === 1) {
        recs.interp.push(base);
      } else {
        const own = interp.poses(t).get(o.id);
        const times = tm.cameras.map((c) => site.cameras.get(c).latest.timestamp_ns);
        recs.fusion.push({
          ...base,
          cameras: tm.cameras.length,
          dFusion: own ? Math.hypot(px - own.x, py - own.y) : null,
          errMemberNow: own && gNow ? Math.hypot(own.x - gNow.x, own.y - gNow.y) : null,
          skewMs: (Math.max(...times) - Math.min(...times)) / 1e6,
        });
      }
      if (!o.vru && gCap) {
        const info = classOf(o.class);
        const l = info.length_m * tm.model.scale.x;
        const w = info.width_m * tm.model.scale.y;
        const drawn = corners(px, py, tm.drawnHeading, l, w);
        const truthAxis = corners(px, py, gCap.yaw, l, w);
        const ageS = (t - tm.born) / 1000;
        recs.heading.push({
          backend: o.heading_rad === null ? 'none' : 'measured',
          young: ageS < 1.5,
          moving: o.speed_mps > 1.2 && (o.speed_std_mps === null || o.speed_std_mps < Math.max(0.6, 0.5 * o.speed_mps)),
          axisErr3d: axisDeg(tm.drawnHeading, gCap.yaw),
          fullErr3d: fullDeg(tm.drawnHeading, gCap.yaw),
          axisErrBev: o.heading_rad === null ? null : axisDeg(o.heading_rad, gCap.yaw),
          fullErrBev: o.heading_rad === null ? null : fullDeg(o.heading_rad, gCap.yaw),
          drawnVsBackend: o.heading_rad === null ? null : fullDeg(tm.drawnHeading, o.heading_rad),
          cornerShift: axisCornerDistance(drawn, truthAxis),
          zero: o.heading_rad === null && Math.abs(wrap(tm.drawnHeading)) < 1e-6,
        });
      }
    }
    if (measure.perspective && frame % 120 === 0) perspective(site, recs);
  }

  function perspective(site, recs) {
    const rig = twin.rig;
    const canvas = twin.canvas;
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    const toPx = (v) => {
      const n = v.clone().project(rig.camera);
      return { x: ((n.x + 1) / 2) * w, y: ((1 - n.y) / 2) * h, inside: Math.abs(n.x) <= 1 && Math.abs(n.y) <= 1 && n.z < 1 };
    };
    const local = (v) => site.group.worldToLocal(v.clone());
    for (const mode of ['3d', '2d']) {
      rig.setMode(mode);
      for (let i = 0; i < 200; i++) rig.tick(0.05);
      twin.scene.updateMatrixWorld(true);
      rig.camera.updateMatrixWorld(true);
      for (const tm of site.meshes.values()) {
        if (tm.lost !== null) continue;
        const ground = tm.root.getWorldPosition(new Vector3());
        const box = new Box3().setFromObject(tm.model);
        const centre = box.getCenter(new Vector3());
        const top = new Vector3(centre.x, centre.y, box.max.z);
        const g = toPx(ground);
        if (!g.inside) continue;
        const c = toPx(centre);
        const r = toPx(top);
        const apparent = (p) => {
          const hit = rig.groundAt(p.x, p.y);
          if (!hit) return null;
          const a = local(hit);
          return Math.hypot(a.x - tm.x, a.y - tm.y);
        };
        const groundBack = rig.groundAt(g.x, g.y);
        recs.persp.push({
          mode,
          vru: tm.last.vru,
          height: box.max.z - box.min.z,
          fromCentrePx: Math.hypot(g.x - w / 2, g.y - h / 2),
          pxCentre: Math.hypot(c.x - g.x, c.y - g.y),
          pxTop: Math.hypot(r.x - g.x, r.y - g.y),
          mCentre: apparent(c),
          mTop: apparent(r),
          groundError: groundBack ? Math.hypot(local(groundBack).x - tm.x, local(groundBack).y - tm.y) : null,
          metresPerPx: (() => {
            const a = rig.groundAt(g.x, g.y);
            const b = rig.groundAt(g.x + 1, g.y);
            return a && b ? Math.hypot(a.x - b.x, a.y - b.y) : null;
          })(),
        });
      }
    }
    rig.setMode('3d');
    for (let i = 0; i < 200; i++) rig.tick(0.05);
  }

  if (options.only === 'sweep') {
    try {
      await replay('learn', ['south2', 'south1'], {}, null, 20);
      for (const skew of options.skews ?? []) {
        for (const [name, delays] of [
          ['sync', {}],
          ['async300', { south1: 300 }],
        ]) {
          const r = await replay(`${name}-${skew}`, ['south2', 'south1'], delays, { fusionOnly: true }, 60, skew);
          results.records[`sweep_${name}_${skew}`] = { fev: r.fev, counts: r.counts };
        }
      }
      results.status = 'done';
    } catch (error) {
      results.status = `error: ${error.message}`;
    } finally {
      performance.now = realNow;
    }
    return results;
  }
  if (options.only === 'lost') {
    try {
      results.records.lostFix = await replay('lostfix', ['south2'], {}, {});
      results.status = 'done';
    } catch (error) {
      results.status = `error: ${error.message}`;
    } finally {
      performance.now = realNow;
    }
    return results;
  }
  try {
    results.log.push(`canvas ${twin.canvas.clientWidth}x${twin.canvas.clientHeight}`);
    // Warm-up: the browser learns the direction of traffic over time; one pass teaches it.
    await replay('learn', ['south2', 'south1'], {}, null, 20);
    results.records.single = await replay('single', ['south2'], {}, { perspective: true });
    results.records.sync = await replay('sync', ['south2', 'south1'], {}, {});
    results.records.async300 = await replay('async300', ['south2', 'south1'], { south1: 300 }, {});
    results.records.noflow = await (async () => {
      sharedFlow = null; // fresh, empty flow field: what a new browser shows
      const r = await replay('noflow', ['south2'], {}, {});
      return { heading: r.heading };
    })();
    // Fusion threshold sweep (Infinity: merge regardless of time, as before the gate).
    for (const skew of options.skews ?? []) {
      for (const [name, delays] of [
        ['sync', {}],
        ['async300', { south1: 300 }],
      ]) {
        const r = await replay(`${name}-${skew}`, ['south2', 'south1'], delays, {}, 60, skew);
        results.records[`sweep_${name}_${skew}`] = { fev: r.fev, counts: r.counts, fusion: r.fusion };
      }
    }
    results.status = 'done';
  } catch (error) {
    results.status = `error: ${error.message}\n${error.stack}`;
  } finally {
    performance.now = realNow;
    for (const key of Object.keys(localStorage)) {
      if (key.startsWith('jp-flow-fidelity')) localStorage.removeItem(key);
    }
  }
  return results;
}
