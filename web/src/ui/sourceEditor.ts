import {
  type InputFile,
  type Inputs,
  type ProbeResult,
  probeInput,
  type SiteSource,
  SOURCE_LABELS,
  type SourceKind,
  sourceKind,
  uploadVideo,
} from '../net/manager';
import { fill, h } from './dom';
import { toast } from './feedback';
import { formatBytes, formatDuration } from './format';
import { icon, type IconName } from './icons';

const KINDS: { kind: SourceKind; icon: IconName; text: string }[] = [
  { kind: 'file', icon: 'video', text: 'Recorded video on the server (MP4, MOV, MKV)' },
  { kind: 'stream', icon: 'stream', text: 'Live IP camera over RTSP or HTTP' },
  { kind: 'replay', icon: 'replay', text: 'World states recorded earlier (JSONL)' },
  { kind: 'oracle', icon: 'database', text: 'Ground-truth labels of a benchmark sequence' },
];

let listId = 0;

function datalist(files: InputFile[]): { id: string; element: HTMLDataListElement } {
  const id = `inputs-${++listId}`;
  const element = h(
    'datalist',
    { id },
    files.map((f) => h('option', { value: f.path }, f.size !== null ? formatBytes(f.size) : '')),
  );
  return { id, element };
}

export interface SourceEditor {
  root: HTMLElement;
  value(): SiteSource;
  /** Problem with the current entry, or null if it can be saved. */
  problem(): string | null;
}

/** Choice of the camera source of a site and its video, stream URL or recording. */
export function sourceEditor(
  initial: SiteSource,
  inputs: Inputs | null,
  onChange: () => void,
  /** Called with the size, rate and length of the chosen video or stream once it was opened. */
  onProbe: (result: ProbeResult) => void = () => undefined,
): SourceEditor {
  let kind = sourceKind(initial);
  const values: Record<SourceKind, string> = { file: '', stream: '', replay: '', oracle: '' };
  values[kind] = initial.input;
  const cards = h('div', { class: 'choice-grid' });
  const detail = h('div', { class: 'stack' });
  const probeLine = h('div', { class: 'probe-line' });
  let probeTimer = 0;
  let probeRun = 0;

  /** Opens the chosen video or stream on the server to read its size, after typing stops. */
  const scheduleProbe = (delayMs = 700): void => {
    clearTimeout(probeTimer);
    const value = values[kind];
    if ((kind !== 'file' && kind !== 'stream') || !value) {
      fill(probeLine);
      return;
    }
    probeTimer = window.setTimeout(() => {
      const run = ++probeRun;
      probeLine.className = 'probe-line';
      fill(probeLine, h('span', { class: 'spinner' }), 'Opening the video…');
      probeInput(value)
        .then((result) => {
          if (run !== probeRun) {
            return;
          }
          const length =
            result.frames && result.fps > 0
              ? ` · ${formatDuration(result.frames / result.fps)}`
              : '';
          fill(
            probeLine,
            icon('check'),
            `Detected ${result.width} × ${result.height} px · ${result.fps.toFixed(1)} fps${length}`,
          );
          probeLine.className = 'probe-line ok';
          onProbe(result);
        })
        .catch((error: unknown) => {
          if (run === probeRun) {
            fill(probeLine, icon('alert'), (error as Error).message);
            probeLine.className = 'probe-line error';
          }
        });
    }, delayMs);
  };

  const uploadRow = (): HTMLElement => {
    const file = h('input', {
      type: 'file',
      accept: 'video/*,.mp4,.mov,.mkv,.avi,.m4v',
      hidden: true,
    });
    const bar = h('span', {});
    const progress = h('div', { class: 'meter', hidden: true }, bar);
    const button = h(
      'button',
      { type: 'button', class: 'btn small' },
      icon('upload'),
      'Upload a video',
    );
    button.addEventListener('click', () => file.click());
    file.addEventListener('change', () => {
      const chosen = file.files?.[0];
      if (!chosen) {
        return;
      }
      button.disabled = true;
      progress.hidden = false;
      uploadVideo(chosen, (f) => (bar.style.width = `${(f * 100).toFixed(1)}%`))
        .then(({ path }) => {
          values.file = path;
          inputs?.videos.push({ path, size: chosen.size });
          lists.file.element.append(h('option', { value: path }, formatBytes(chosen.size)));
          renderDetail();
          onChange();
          scheduleProbe(0);
          toast(`Uploaded ${chosen.name}`, 'ok');
        })
        .catch((error: unknown) => toast((error as Error).message, 'error'))
        .finally(() => {
          button.disabled = false;
          progress.hidden = true;
          file.value = '';
        });
    });
    return h(
      'div',
      { class: 'upload-row' },
      button,
      h('small', {}, 'or pick a file from your computer; it is copied to the server.'),
      progress,
      file,
    );
  };
  const lists = {
    file: datalist(inputs?.videos ?? []),
    replay: datalist(inputs?.replays ?? []),
    oracle: datalist(inputs?.sequences ?? []),
  };

  const renderCards = (): void =>
    fill(
      cards,
      KINDS.map((k) =>
        h(
          'button',
          {
            type: 'button',
            class: `choice${k.kind === kind ? ' active' : ''}`,
            onclick: () => {
              kind = k.kind;
              render();
              onChange();
            },
          },
          icon(k.icon),
          h('strong', {}, SOURCE_LABELS[k.kind]),
          h('span', {}, k.text),
        ),
      ),
    );

  const input = (placeholder: string, list?: string): HTMLInputElement => {
    const element = h('input', {
      class: 'input mono',
      placeholder,
      list,
      spellcheck: 'false',
      autocomplete: 'off',
    });
    element.value = values[kind];
    element.addEventListener('input', () => {
      values[kind] = element.value.trim();
      onChange();
      scheduleProbe();
    });
    return element;
  };

  const renderDetail = (): void => {
    const count = (files: InputFile[]) =>
      files.length > 0 ? `${files.length} found on the server` : 'none found';
    switch (kind) {
      case 'file':
        fill(
          detail,
          h(
            'label',
            { class: 'field' },
            h('span', {}, 'Video file'),
            input('custom/intersection.mp4', lists.file.id),
            h(
              'small',
              {},
              `Path below the data root of the server; pick from the list (${count(inputs?.videos ?? [])}) or type it.`,
            ),
          ),
          uploadRow(),
          probeLine,
          lists.file.element,
        );
        break;
      case 'stream':
        fill(
          detail,
          h(
            'label',
            { class: 'field' },
            h('span', {}, 'Stream URL'),
            input('rtsp://user:password@192.168.1.20:554/stream1'),
            h(
              'small',
              {},
              'RTSP, RTSPS, HTTP or HTTPS. Credentials are stored in the site settings on the server.',
            ),
          ),
          h(
            'div',
            { class: 'banner info' },
            icon('info'),
            h('span', {}, 'Live streams have no timeline: pause and seek are not available.'),
          ),
          probeLine,
        );
        break;
      case 'replay':
        fill(
          detail,
          h(
            'label',
            { class: 'field' },
            h('span', {}, 'Recording'),
            input('outputs/<scene>/world.jsonl', lists.replay.id),
            h(
              'small',
              {},
              `JSONL written by jp_run, relative to the repository (${count(inputs?.replays ?? [])}).`,
            ),
          ),
          lists.replay.element,
        );
        break;
      case 'oracle':
        fill(
          detail,
          h(
            'label',
            { class: 'field' },
            h('span', {}, 'Benchmark sequence'),
            input('a9_dataset_r02_s01', lists.oracle.id),
            h(
              'small',
              {},
              `TUMTraf sequence folder below the data root (${count(inputs?.sequences ?? [])}); needs a calibrated benchmark scene.`,
            ),
          ),
          lists.oracle.element,
        );
        break;
    }
  };

  const render = (): void => {
    renderCards();
    renderDetail();
    scheduleProbe(0);
  };
  render();

  return {
    root: h('div', { class: 'stack' }, cards, detail),
    value: () => ({
      mode: kind === 'file' || kind === 'stream' ? 'live' : kind,
      input: values[kind],
    }),
    problem: () => {
      const value = values[kind];
      if (!value) {
        return 'Choose the camera source of the site.';
      }
      if (kind === 'stream' && !/^(rtsp|rtsps|http|https):\/\/.+/.test(value)) {
        return 'A stream URL starts with rtsp://, rtsps://, http:// or https://.';
      }
      if (kind !== 'stream' && value.includes('://')) {
        return 'Choose "Camera stream" for URLs.';
      }
      return null;
    },
  };
}

export const IMAGE_SIZES: { label: string; size: [number, number] }[] = [
  { label: 'Full HD · 1920 × 1080', size: [1920, 1080] },
  { label: 'WUXGA · 1920 × 1200', size: [1920, 1200] },
  { label: 'HD · 1280 × 720', size: [1280, 720] },
  { label: 'QHD · 2560 × 1440', size: [2560, 1440] },
  { label: '4K UHD · 3840 × 2160', size: [3840, 2160] },
];

/** Image size of the camera: presets or a custom width and height. */
export function imageSizeField(
  initial: [number, number],
  onChange: () => void,
): { root: HTMLElement; value(): [number, number]; set(size: [number, number]): void } {
  const select = h('select', { class: 'input' });
  const preset = IMAGE_SIZES.findIndex((p) => p.size[0] === initial[0] && p.size[1] === initial[1]);
  IMAGE_SIZES.forEach((p, i) =>
    select.append(h('option', { value: String(i), selected: i === preset }, p.label)),
  );
  select.append(h('option', { value: 'custom', selected: preset < 0 }, 'Custom size'));
  const width = h('input', { class: 'input num', type: 'number', min: 64, step: 1 });
  const height = h('input', { class: 'input num', type: 'number', min: 64, step: 1 });
  width.value = String(initial[0]);
  height.value = String(initial[1]);
  const custom = h(
    'div',
    { class: 'form-grid' },
    h('label', { class: 'field' }, h('span', {}, 'Width (px)'), width),
    h('label', { class: 'field' }, h('span', {}, 'Height (px)'), height),
  );
  const update = (): void => {
    custom.hidden = select.value !== 'custom';
    const chosen = IMAGE_SIZES[Number(select.value)];
    if (chosen) {
      width.value = String(chosen.size[0]);
      height.value = String(chosen.size[1]);
    }
  };
  select.addEventListener('change', () => {
    update();
    onChange();
  });
  width.addEventListener('input', onChange);
  height.addEventListener('input', onChange);
  update();
  return {
    root: h(
      'div',
      { class: 'stack' },
      h(
        'label',
        { class: 'field' },
        h('span', {}, 'Camera image size'),
        select,
        h('small', {}, 'Detected from the video when it is opened; the calibration refers to it.'),
      ),
      custom,
    ),
    value: () => [Number(width.value), Number(height.value)],
    set: (size) => {
      const index = IMAGE_SIZES.findIndex((p) => p.size[0] === size[0] && p.size[1] === size[1]);
      select.value = index >= 0 ? String(index) : 'custom';
      width.value = String(size[0]);
      height.value = String(size[1]);
      custom.hidden = index >= 0;
      onChange();
    },
  };
}
