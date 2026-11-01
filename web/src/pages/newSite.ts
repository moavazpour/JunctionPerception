import '../styles/pages.css';

import {
  APPROACH_LABELS,
  type Approach,
  createSite,
  type GeoAnchor,
  listInputs,
  type NewSite,
  SOURCE_LABELS,
  sourceKind,
} from '../net/manager';
import { CameraFields, intersectionDiagram } from '../ui/cameraEditor';
import { type Child, fill, h, toggle } from '../ui/dom';
import { toast, withBusy } from '../ui/feedback';
import { slugify } from '../ui/format';
import { icon } from '../ui/icons';
import { links, mountShell } from '../ui/shell';
import { latLonInputs } from '../ui/latLon';

const STEPS = ['Intersection', 'Centre', 'Cameras', 'Review'] as const;
const MAX_CAMERAS = 4;
const DEFAULT_RADIUS_M = 200;
// Suggested arms for the second, third and fourth camera.
const NEXT_APPROACH: Approach[] = ['north', 'south', 'east', 'west'];

async function start(): Promise<void> {
  const shell = await mountShell({ page: 'new-site', title: 'New site' });
  if (!shell.managed) {
    location.replace('/');
    return;
  }
  const inputs = await listInputs().catch(() => null);
  let step = 0;
  let idEdited = false;
  let activate = true;
  let prepareMap = true;

  const name = h('input', {
    class: 'input',
    placeholder: 'e.g. Munich – Leopoldstraße / Ungererstraße',
    maxlength: 80,
  });
  const id = h('input', {
    class: 'input mono',
    placeholder: 'munich-leopoldstrasse',
    maxlength: 48,
  });
  const description = h('textarea', {
    class: 'input',
    rows: 2,
    placeholder: 'Streets, purpose, contact…',
    maxlength: 400,
  });
  name.addEventListener('input', () => {
    if (!idEdited) {
      id.value = slugify(name.value);
    }
  });
  id.addEventListener('input', () => (idEdited = id.value !== ''));
  const lat = h('input', {
    class: 'input num',
    type: 'number',
    step: 0.000001,
    placeholder: '48.137154',
  });
  const lon = h('input', {
    class: 'input num',
    type: 'number',
    step: 0.000001,
    placeholder: '11.576124',
  });
  latLonInputs(lat, lon);
  const alt = h('input', { class: 'input num', type: 'number', step: 0.1, value: 0 });
  const radius = h('input', {
    class: 'input num',
    type: 'number',
    step: 10,
    min: 50,
    max: 1000,
    value: DEFAULT_RADIUS_M,
  });
  const mapLink = h(
    'a',
    { target: '_blank', rel: 'noopener', class: 'btn small ghost' },
    icon('external'),
    'Check on OpenStreetMap',
  );

  const anchor = (): GeoAnchor | null => {
    const la = Number.parseFloat(lat.value);
    const lo = Number.parseFloat(lon.value);
    return Number.isFinite(la) && Number.isFinite(lo)
      ? { lat_deg: la, lon_deg: lo, alt_m: Number.parseFloat(alt.value) || 0 }
      : null;
  };
  const updateMapLink = (): void => {
    const a = anchor();
    mapLink.hidden = !a;
    if (a) {
      mapLink.setAttribute(
        'href',
        `https://www.openstreetmap.org/?mlat=${a.lat_deg}&mlon=${a.lon_deg}#map=19/${a.lat_deg}/${a.lon_deg}`,
      );
    }
  };
  lat.addEventListener('input', updateMapLink);
  lon.addEventListener('input', updateMapLink);
  updateMapLink();

  // Cameras of the intersection, one form each.
  const cameraList = h('div', { class: 'camera-list' });
  const plan = h('div', { class: 'junction-plan-box' });
  const cameraTools = h('div', { class: 'row' });
  const cameras: CameraFields[] = [];
  const renderPlan = (): void =>
    fill(
      plan,
      intersectionDiagram(cameras.map((c) => c.value())),
      h('small', {}, 'Cameras on the arms (north up)'),
    );
  const addCameraForm = (): void => {
    const index = cameras.length;
    const fields = new CameraFields(
      {
        name: `Camera ${index + 1}`,
        approach: index === 0 ? 'centre' : (NEXT_APPROACH[index - 1] ?? 'centre'),
        image_size_px: [1920, 1080],
        source: { mode: 'live', input: '' },
      },
      inputs,
      renderPlan,
    );
    cameras.push(fields);
    renderCameras();
  };
  const renderCameras = (): void => {
    fill(
      cameraList,
      cameras.map((fields, index) => {
        const remove = h(
          'button',
          { type: 'button', class: 'btn small ghost', title: 'Remove this camera' },
          icon('trash'),
        );
        remove.disabled = cameras.length === 1;
        remove.addEventListener('click', () => {
          cameras.splice(index, 1);
          renderCameras();
        });
        return h(
          'article',
          { class: 'camera-panel open' },
          h(
            'header',
            { class: 'camera-panel-head' },
            h('span', { class: 'camera-index' }, String(index + 1)),
            h(
              'div',
              { class: 'camera-title' },
              h('div', {}, h('strong', {}, `Camera ${index + 1}`)),
            ),
            h('span', { class: 'spacer' }),
            remove,
          ),
          h('div', { class: 'camera-panel-body' }, fields.root),
        );
      }),
    );
    const add = h('button', { type: 'button', class: 'btn' }, icon('plus'), 'Add another camera');
    add.disabled = cameras.length >= MAX_CAMERAS;
    add.addEventListener('click', addCameraForm);
    fill(
      cameraTools,
      add,
      h('span', { class: 'muted' }, `${cameras.length} of ${MAX_CAMERAS} cameras`),
    );
    renderPlan();
  };
  addCameraForm();

  const request = (): NewSite => {
    const a = anchor();
    return {
      id: id.value.trim(),
      name: name.value.trim(),
      description: description.value.trim(),
      geo_anchor: a ?? { lat_deg: 0, lon_deg: 0, alt_m: 0 },
      map_radius_m: Number.parseFloat(radius.value) || DEFAULT_RADIUS_M,
      enabled: activate,
      prepare_map: prepareMap,
      cameras: cameras.map((c) => {
        const v = c.value();
        return {
          name: v.name,
          approach: v.approach,
          image_size_px: v.image_size_px,
          source: v.source,
        };
      }),
    };
  };

  /** Problem that keeps the user on the current step. */
  const problem = (): string | null => {
    if (step === 0) {
      if (!name.value.trim()) {
        return 'Give the site a name.';
      }
      if (!/^[a-z0-9][a-z0-9_-]{1,47}$/.test(id.value.trim())) {
        return 'The site ID needs 2–48 characters: lowercase letters, digits, - and _.';
      }
      if (shell.sites().some((s) => s.id === id.value.trim())) {
        return 'A site with this ID exists already.';
      }
    }
    if (step === 1) {
      const a = anchor();
      if (!a) {
        return 'Enter the latitude and longitude of the intersection centre.';
      }
      if (Math.abs(a.lat_deg) > 90 || Math.abs(a.lon_deg) > 180) {
        return 'Latitude must be within ±90°, longitude within ±180°.';
      }
      const r = Number.parseFloat(radius.value);
      if (!(r >= 50 && r <= 1000)) {
        return 'The map radius must be between 50 and 1000 m.';
      }
    }
    if (step === 2) {
      for (const [index, camera] of cameras.entries()) {
        const issue = camera.problem();
        if (issue) {
          return `Camera ${index + 1}: ${issue}`;
        }
      }
    }
    return null;
  };

  const stepper = h('div', { class: 'steps' });
  const body = h('div', { class: 'card-body stack' });
  const nav = h('div', { class: 'wizard-nav' });
  const card = h('section', { class: 'card' }, body, nav);
  const page = h(
    'div',
    { class: 'page narrow' },
    h(
      'div',
      { class: 'page-head' },
      h(
        'div',
        {},
        h('h1', {}, 'New site'),
        h(
          'p',
          {},
          'Add an intersection: where its centre is, the map around it and its one to four cameras.',
        ),
      ),
    ),
    stepper,
    card,
  );
  shell.content.append(page);

  const field = (label: string, input: HTMLElement, help?: string, wide = false): HTMLElement =>
    h(
      'label',
      { class: `field${wide ? ' wide' : ''}` },
      h('span', {}, label),
      input,
      help ? h('small', {}, help) : null,
    );

  const review = (): HTMLElement => {
    const r = request();
    const rows: [string, Child][] = [
      ['Name', r.name],
      ['Site ID', h('code', {}, r.id)],
      [
        'Centre',
        `${r.geo_anchor.lat_deg.toFixed(6)}°, ${r.geo_anchor.lon_deg.toFixed(6)}° · map within ${r.map_radius_m} m`,
      ],
      ...r.cameras.map((c, i): [string, Child] => [
        `Camera ${i + 1}`,
        h(
          'span',
          {},
          `${c.name} · ${APPROACH_LABELS[c.approach]} · ${SOURCE_LABELS[sourceKind(c.source)]} `,
          h('code', {}, c.source.input),
          ` · ${c.image_size_px[0]} × ${c.image_size_px[1]} px`,
        ),
      ]),
    ];
    return h(
      'dl',
      { class: 'kv' },
      rows.flatMap(([k, v]) => [h('dt', {}, k), h('dd', {}, v)]),
    );
  };

  const render = (): void => {
    fill(
      stepper,
      STEPS.map((label, i) =>
        h(
          'div',
          { class: `step${i === step ? ' active' : ''}${i < step ? ' done' : ''}` },
          h('b', {}, i < step ? icon('check') : String(i + 1)),
          h('span', {}, label),
        ),
      ),
    );
    page.classList.toggle('wide', step === 2);
    switch (step) {
      case 0:
        fill(
          body,
          h('h2', {}, 'Name the intersection'),
          h(
            'div',
            { class: 'form-grid' },
            field('Name', name, 'Shown in the console and on the overview.'),
            field('Site ID', id, 'Folder name on the server; cannot be changed later.'),
            field('Description (optional)', description, undefined, true),
          ),
        );
        setTimeout(() => name.focus(), 0);
        break;
      case 1:
        fill(
          body,
          h('h2', {}, 'Centre of the intersection'),
          h(
            'p',
            { class: 'help' },
            'The centre is the origin of the world frame that all cameras of the intersection share. Take it from a map service (right click → coordinates) and paste "latitude, longitude" into either field. The map layers (aerial imagery, roads, crossings, buildings) are downloaded for the area around it.',
          ),
          h(
            'div',
            { class: 'form-grid' },
            field('Latitude (°)', lat),
            field('Longitude (°)', lon),
            field('Altitude (m)', alt, 'Height above sea level; optional.'),
            field('Map radius (m)', radius, 'Area around the centre covered by the map layers.'),
          ),
          h(
            'div',
            { class: 'row' },
            toggle(
              'Download map layers after creating the site',
              prepareMap,
              (on) => (prepareMap = on),
            ),
            h('span', { class: 'spacer' }),
            mapLink,
          ),
        );
        break;
      case 2:
        fill(
          body,
          h('h2', {}, 'Cameras'),
          h(
            'p',
            { class: 'help' },
            'Add every camera that watches the intersection (up to four) and the arm it looks at. Each camera is calibrated on its own afterwards; their road users meet in the one twin of the intersection.',
          ),
          h(
            'div',
            { class: 'cameras-layout' },
            h('div', { class: 'stack', style: 'min-width: 0' }, cameraList, cameraTools),
            plan,
          ),
        );
        break;
      default:
        fill(
          body,
          h('h2', {}, 'Review'),
          review(),
          h(
            'div',
            { class: 'status-line' },
            icon('power'),
            h(
              'div',
              {},
              h('strong', {}, 'Activate after creating'),
              h(
                'span',
                {},
                'Starts the perception server of every camera; needed for calibration.',
              ),
            ),
            toggle('Activate', activate, (on) => (activate = on)),
          ),
        );
    }
    const back = h(
      'button',
      { type: 'button', class: 'btn ghost' },
      icon('back'),
      step === 0 ? 'Cancel' : 'Back',
    );
    back.addEventListener('click', () => {
      if (step === 0) {
        location.href = '/';
      } else {
        step--;
        render();
      }
    });
    const last = step === STEPS.length - 1;
    const next = h(
      'button',
      { type: 'button', class: 'btn primary' },
      last ? icon('check') : null,
      last ? 'Create site' : 'Continue',
      last ? null : icon('chevron'),
    );
    next.addEventListener('click', () => {
      const issue = problem();
      if (issue) {
        toast(issue, 'error');
        return;
      }
      if (!last) {
        step++;
        render();
        return;
      }
      const r = request();
      void withBusy(next, async () => {
        const site = await createSite(r);
        done(site.id, site.name, r, site.cameras[0]?.id);
      });
    });
    fill(nav, back, next);
  };

  const done = (siteId: string, siteName: string, r: NewSite, firstCamera?: string): void => {
    void shell.refresh();
    fill(stepper);
    const nextStep = (
      href: string,
      name: 'calibration' | 'twin' | 'settings',
      title: string,
      text: string,
      primary = false,
    ) =>
      h(
        'a',
        { class: `choice${primary ? ' active' : ''}`, href },
        icon(name),
        h('strong', {}, title),
        h('span', {}, text),
      );
    const count = r.cameras.length;
    fill(
      card,
      h(
        'div',
        { class: 'done-panel' },
        icon('check'),
        h('h2', {}, `${siteName} is ready`),
        h(
          'p',
          {},
          r.enabled
            ? `The perception server${count === 1 ? ' is' : 's of its ' + count + ' cameras are'} starting. Next, calibrate each camera: click a few points in its image and the same points on the map.`
            : 'The site is inactive; activate it in its settings when the camera sources are ready.',
          r.prepare_map ? ' Map layers are being downloaded in the background.' : '',
        ),
        h(
          'div',
          { class: 'next-steps' },
          nextStep(
            links.calibration(siteId, firstCamera),
            'calibration',
            'Calibrate the cameras',
            'Image ↔ map points, zones',
            true,
          ),
          nextStep(links.twin(siteId), 'twin', 'Open the live twin', 'All cameras in one view'),
          nextStep(links.settings(siteId), 'settings', 'Site settings', 'Cameras, location, map'),
        ),
      ),
    );
  };

  render();
}

void start();
