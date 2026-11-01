/** Editing a camera of an intersection: name, the arm it watches, source and image size. */
import {
  APPROACH_BEARING,
  APPROACH_LABELS,
  type Approach,
  type Inputs,
  type PlaybackSettings,
  type SiteSource,
  type VideoSettings,
} from '../net/manager';
import { type Child, fill, h, toggle } from './dom';
import { imageSizeField, type SourceEditor, sourceEditor } from './sourceEditor';

// Approaches as laid out on the compass picker, row by row.
const COMPASS: Approach[] = [
  'north-west',
  'north',
  'north-east',
  'west',
  'centre',
  'east',
  'south-west',
  'south',
  'south-east',
];
const SHORT: Record<Approach, string> = {
  north: 'N',
  'north-east': 'NE',
  east: 'E',
  'south-east': 'SE',
  south: 'S',
  'south-west': 'SW',
  west: 'W',
  'north-west': 'NW',
  centre: 'C',
};
const SPEEDS = [0.25, 0.5, 1, 2, 4];
const WIDTHS = [640, 960, 1280, 1920];
const SVG = 'http://www.w3.org/2000/svg';

function svg(tag: string, attrs: Record<string, string | number>, text?: string): SVGElement {
  const element = document.createElementNS(SVG, tag);
  for (const [key, value] of Object.entries(attrs)) {
    element.setAttribute(key, String(value));
  }
  if (text !== undefined) {
    element.textContent = text;
  }
  return element;
}

/**
 * Small plan of an intersection with its cameras on the arms they watch (north up). The camera
 * `active` is highlighted.
 */
export function intersectionDiagram(
  cameras: { name: string; approach: Approach }[],
  active = -1,
): SVGSVGElement {
  const size = 168;
  const c = size / 2;
  const root = svg('svg', {
    viewBox: `0 0 ${size} ${size}`,
    class: 'junction-plan',
    role: 'img',
    'aria-label': 'Cameras on the arms of the intersection',
  }) as SVGSVGElement;
  root.append(
    svg('rect', { x: c - 15, y: 4, width: 30, height: size - 8, rx: 3, class: 'road' }),
    svg('rect', { x: 4, y: c - 15, width: size - 8, height: 30, rx: 3, class: 'road' }),
    svg('path', {
      d: `M${c} 8V${c - 19}M${c} ${c + 19}V${size - 8}M8 ${c}H${c - 19}M${c + 19} ${c}H${size - 8}`,
      class: 'lane',
    }),
    svg('text', { x: c, y: 13, class: 'north' }, 'N'),
  );
  // Cameras on the same arm are spread out a little.
  const seen = new Map<Approach, number>();
  cameras.forEach((camera, index) => {
    const bearing = APPROACH_BEARING[camera.approach];
    const n = seen.get(camera.approach) ?? 0;
    seen.set(camera.approach, n + 1);
    let x = c + (n % 2 === 0 ? 1 : -1) * Math.ceil(n / 2) * 13;
    let y = c;
    if (bearing !== null) {
      const angle = (bearing * Math.PI) / 180;
      const r = 50 + n * 12;
      x = c + r * Math.sin(angle);
      y = c - r * Math.cos(angle);
    }
    const group = svg('g', { class: `cam${index === active ? ' active' : ''}` });
    group.append(
      svg('circle', { cx: x, cy: y, r: 9 }),
      svg('text', { x, y: y + 3.5 }, String(index + 1)),
      svg('title', {}, `${camera.name} · ${APPROACH_LABELS[camera.approach]}`),
    );
    root.append(group);
  });
  return root;
}

/** Compass of the arms of an intersection; returns the element and the chosen approach. */
export function approachPicker(
  initial: Approach,
  onChange: (approach: Approach) => void,
): { root: HTMLElement; value(): Approach } {
  let current = initial;
  const buttons = COMPASS.map((approach) =>
    h(
      'button',
      {
        type: 'button',
        class: approach === current ? 'active' : '',
        title: APPROACH_LABELS[approach],
        onclick: () => {
          current = approach;
          buttons.forEach((b, i) => b.classList.toggle('active', COMPASS[i] === approach));
          label.textContent = APPROACH_LABELS[approach];
          onChange(approach);
        },
      },
      SHORT[approach],
    ),
  );
  const label = h('span', { class: 'approach-label' }, APPROACH_LABELS[current]);
  return {
    root: h('div', { class: 'approach-picker' }, h('div', { class: 'compass' }, buttons), label),
    value: () => current,
  };
}

export interface CameraValues {
  name: string;
  approach: Approach;
  image_size_px: [number, number];
  source: SiteSource;
  playback?: PlaybackSettings;
  video?: VideoSettings;
}

function select<T extends string | number>(values: T[], current: T, label: (v: T) => string) {
  const element = h('select', { class: 'input' });
  const options = values.includes(current) ? values : [...values, current];
  for (const v of options) {
    element.append(h('option', { value: String(v), selected: v === current }, label(v)));
  }
  return element;
}

/** Form fields of one camera; with `streaming`, also playback and camera-image settings. */
export class CameraFields {
  readonly root = h('div', { class: 'stack camera-fields' });
  private readonly name = h('input', { class: 'input', maxlength: 60 });
  private readonly approach: ReturnType<typeof approachPicker>;
  private readonly source: SourceEditor;
  private readonly imageSize: ReturnType<typeof imageSizeField>;
  private streaming: {
    loop: boolean;
    speed: HTMLSelectElement;
    fps: HTMLInputElement;
    width: HTMLSelectElement;
    quality: HTMLInputElement;
  } | null = null;

  constructor(
    initial: CameraValues,
    inputs: Inputs | null,
    onChange: () => void,
    options: { streaming?: boolean; extra?: Child } = {},
  ) {
    this.name.value = initial.name;
    this.name.addEventListener('input', onChange);
    this.approach = approachPicker(initial.approach, onChange);
    this.imageSize = imageSizeField(initial.image_size_px, onChange);
    this.source = sourceEditor(initial.source, inputs, onChange, (probe) =>
      this.imageSize.set([probe.width, probe.height]),
    );
    const streaming = options.streaming && initial.playback && initial.video;
    let streamingFields: HTMLElement | null = null;
    if (streaming && initial.playback && initial.video) {
      const quality = h('input', { type: 'range', min: 30, max: 95, step: 5 });
      quality.value = String(initial.video.jpeg_quality);
      const qualityLabel = h('strong', { class: 'num' }, quality.value);
      quality.addEventListener('input', () => {
        qualityLabel.textContent = quality.value;
        onChange();
      });
      const fps = h('input', { class: 'input num', type: 'number', step: 0.5, min: 0.5, max: 30 });
      fps.value = String(initial.video.max_fps);
      fps.addEventListener('input', onChange);
      const fields = {
        loop: initial.playback.loop,
        speed: select(SPEEDS, initial.playback.speed, (v) => `${v}×`),
        fps,
        width: select(WIDTHS, initial.video.width_px, (v) => `${v} px`),
        quality,
      };
      fields.speed.addEventListener('change', onChange);
      fields.width.addEventListener('change', onChange);
      this.streaming = fields;
      streamingFields = h(
        'div',
        { class: 'form-grid' },
        h('label', { class: 'field' }, h('span', {}, 'Playback speed'), fields.speed),
        h(
          'div',
          { class: 'field' },
          h('span', {}, 'At the end of a recording'),
          toggle('Start again (loop)', fields.loop, (on) => {
            fields.loop = on;
            onChange();
          }),
        ),
        h('label', { class: 'field' }, h('span', {}, 'Camera image rate (fps)'), fps),
        h('label', { class: 'field' }, h('span', {}, 'Camera image width'), fields.width),
        h(
          'label',
          { class: 'field' },
          h('span', {}, 'JPEG quality'),
          h('div', { class: 'row' }, quality, qualityLabel),
        ),
      );
    }
    fill(
      this.root,
      h(
        'div',
        { class: 'camera-head-fields' },
        h(
          'label',
          { class: 'field' },
          h('span', {}, 'Camera name'),
          this.name,
          h('small', {}, 'e.g. "North mast" or "Bridge, looking west"'),
        ),
        h(
          'div',
          { class: 'field' },
          h('span', {}, 'Part of the intersection it watches'),
          this.approach.root,
        ),
      ),
      h('div', { class: 'field' }, h('span', {}, 'Source'), this.source.root),
      this.imageSize.root,
      streamingFields
        ? h(
            'details',
            { class: 'more' },
            h('summary', {}, 'Playback and camera image'),
            streamingFields,
          )
        : null,
      options.extra ?? null,
    );
  }

  value(): CameraValues {
    const values: CameraValues = {
      name: this.name.value.trim(),
      approach: this.approach.value(),
      image_size_px: this.imageSize.value(),
      source: this.source.value(),
    };
    if (this.streaming) {
      values.playback = { loop: this.streaming.loop, speed: Number(this.streaming.speed.value) };
      values.video = {
        max_fps: Number.parseFloat(this.streaming.fps.value) || 5,
        width_px: Number(this.streaming.width.value),
        jpeg_quality: Number(this.streaming.quality.value),
      };
    }
    return values;
  }

  /** Problem with the current entry, or null if it can be saved. */
  problem(): string | null {
    if (!this.name.value.trim()) {
      return 'The camera needs a name.';
    }
    return this.source.problem();
  }
}
