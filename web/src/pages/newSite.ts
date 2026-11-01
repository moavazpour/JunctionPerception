import '../styles/pages.css';

import {
  createSite,
  type GeoAnchor,
  listInputs,
  type NewSite,
  SOURCE_LABELS,
  sourceKind,
} from '../net/manager';
import { type Child, fill, h, toggle } from '../ui/dom';
import { toast, withBusy } from '../ui/feedback';
import { slugify } from '../ui/format';
import { icon } from '../ui/icons';
import { links, mountShell } from '../ui/shell';
import { imageSizeField, sourceEditor } from '../ui/sourceEditor';

const STEPS = ['Site', 'Camera', 'Location', 'Review'] as const;

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
    placeholder: 'Camera position, purpose, contact…',
    maxlength: 400,
  });
  name.addEventListener('input', () => {
    if (!idEdited) {
      id.value = slugify(name.value);
    }
  });
  id.addEventListener('input', () => (idEdited = id.value !== ''));
  const source = sourceEditor(
    { mode: 'live', input: '' },
    inputs,
    () => undefined,
    (probe) => imageSize.set([probe.width, probe.height]),
  );
  const imageSize = imageSizeField([1920, 1080], () => undefined);
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
  const alt = h('input', { class: 'input num', type: 'number', step: 0.1, value: 0 });
  const radius = h('input', {
    class: 'input num',
    type: 'number',
    step: 10,
    min: 50,
    max: 1000,
    value: 250,
  });
  const osmLink = h(
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
  const updateOsmLink = (): void => {
    const a = anchor();
    osmLink.hidden = !a;
    if (a) {
      osmLink.setAttribute(
        'href',
        `https://www.openstreetmap.org/?mlat=${a.lat_deg}&mlon=${a.lon_deg}#map=19/${a.lat_deg}/${a.lon_deg}`,
      );
    }
  };
  lat.addEventListener('input', updateOsmLink);
  lon.addEventListener('input', updateOsmLink);
  updateOsmLink();

  const request = (): NewSite => {
    const a = anchor();
    return {
      id: id.value.trim(),
      name: name.value.trim(),
      description: description.value.trim(),
      source: source.value(),
      image_size_px: imageSize.value(),
      geo_anchor: a,
      map_radius_m: Number.parseFloat(radius.value) || 250,
      enabled: activate,
      prepare_map: prepareMap && a !== null,
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
      return source.problem();
    }
    if (step === 2) {
      const a = anchor();
      if ((lat.value || lon.value) && !a) {
        return 'Enter both latitude and longitude, or leave both empty.';
      }
      if (a && (Math.abs(a.lat_deg) > 90 || Math.abs(a.lon_deg) > 180)) {
        return 'Latitude must be within ±90°, longitude within ±180°.';
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
        h('p', {}, 'Add an intersection: its camera, where it is, and how the twin should run.'),
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
    const kind = sourceKind(r.source);
    const rows: [string, Child][] = [
      ['Name', r.name],
      ['Site ID', h('code', {}, r.id)],
      ['Camera source', SOURCE_LABELS[kind]],
      ['Input', h('code', {}, r.source.input)],
      ['Image size', `${r.image_size_px[0]} × ${r.image_size_px[1]} px`],
      [
        'Location',
        r.geo_anchor
          ? `${r.geo_anchor.lat_deg.toFixed(6)}°, ${r.geo_anchor.lon_deg.toFixed(6)}° · map radius ${r.map_radius_m} m`
          : 'not set (no map layers, no calibration against the map)',
      ],
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
    switch (step) {
      case 0:
        fill(
          body,
          h('h2', {}, 'Name the site'),
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
        fill(body, h('h2', {}, 'Camera source'), source.root, imageSize.root);
        break;
      case 2:
        fill(
          body,
          h('h2', {}, 'Location'),
          h(
            'p',
            { class: 'help' },
            'The geographic anchor is the origin of the twin, usually the centre of the intersection. Take it from a map service (right click → coordinates). Map layers are downloaded for the area around it; without a location, the calibration against the map is not available.',
          ),
          h(
            'div',
            { class: 'form-grid' },
            field('Latitude (°)', lat),
            field('Longitude (°)', lon),
            field('Altitude (m)', alt, 'Height above sea level; optional.'),
            field('Map radius (m)', radius, 'Area covered by the map layers.'),
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
            osmLink,
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
              h('span', {}, 'Starts the perception server; it is needed for the calibration.'),
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
        done(site.id, site.name, r);
      });
    });
    fill(nav, back, next);
  };

  const done = (siteId: string, siteName: string, r: NewSite): void => {
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
            ? 'Its perception server is starting. Next, calibrate the camera: click a few points in the camera image and the same points on the map.'
            : 'The site is inactive; activate it in its settings when the camera source is ready.',
          r.prepare_map ? ' Map layers are being downloaded in the background.' : '',
        ),
        h(
          'div',
          { class: 'next-steps' },
          nextStep(
            links.calibration(siteId),
            'calibration',
            'Calibrate the camera',
            'Image ↔ map points, zones',
            true,
          ),
          nextStep(
            links.twin(siteId),
            'twin',
            'Open the live twin',
            'Detections without calibration',
          ),
          nextStep(
            links.settings(siteId),
            'settings',
            'Site settings',
            'Source, location, playback',
          ),
        ),
      ),
    );
  };

  render();
}

void start();
