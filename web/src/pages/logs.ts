import '../styles/pages.css';

import { fetchLogs, type LogLevel, type LogLine, streamLogs } from '../net/manager';
import { fill, h, param, segmented, toggle } from '../ui/dom';
import { toast } from '../ui/feedback';
import { formatClock } from '../ui/format';
import { icon } from '../ui/icons';
import { mountShell } from '../ui/shell';

const MAX_LINES = 5000;
const RENDER_LINES = 2000;

type LevelFilter = 'all' | 'info' | 'warning' | 'error';
const RANK: Record<LogLevel, number> = {
  trace: 0,
  debug: 1,
  info: 2,
  warning: 3,
  error: 4,
  critical: 5,
};
const MIN_RANK: Record<LevelFilter, number> = { all: 0, info: 2, warning: 3, error: 4 };

function highlight(text: string, query: string): (string | HTMLElement)[] {
  if (!query) {
    return [text];
  }
  const parts: (string | HTMLElement)[] = [];
  const lower = text.toLowerCase();
  let at = 0;
  for (let i = lower.indexOf(query); i >= 0; i = lower.indexOf(query, at)) {
    parts.push(text.slice(at, i), h('mark', {}, text.slice(i, i + query.length)));
    at = i + query.length;
  }
  parts.push(text.slice(at));
  return parts;
}

async function start(): Promise<void> {
  const shell = await mountShell({ page: 'logs', title: 'Logs', fullBleed: true });
  if (!shell.managed) {
    location.replace('/twin.html');
    return;
  }
  let source = param('source') ?? '*';
  let level: LevelFilter = 'all';
  let query = '';
  let follow = true;
  let paused = false;
  let lines: LogLine[] = [];
  let pending: LogLine[] = [];
  let stop: (() => void) | null = null;

  const sourceSelect = h('select', { class: 'input' });
  let sourceKey = '';
  const fillSources = (): void => {
    // Rebuilt only when the sites change, so an open list is not reset.
    const key = shell
      .sites()
      .map((s) => `${s.id}:${s.name}:${s.cameras.map((c) => `${c.id}=${c.name}`).join(',')}`)
      .join('|');
    if (key === sourceKey) {
      return;
    }
    sourceKey = key;
    // A site covers its own lines (map preparation) and those of all its cameras.
    const options: [string, string][] = [
      ['*', 'All sources'],
      ['manager', 'Console service (manager)'],
      ...shell
        .sites()
        .flatMap((s): [string, string][] => [
          [s.id, `Site · ${s.name}${s.cameras.length > 1 ? ' (all cameras)' : ''}`],
          ...(s.cameras.length > 1
            ? s.cameras.map((c): [string, string] => [
                `${s.id}/${c.id}`,
                `    ${s.name} · ${c.name}`,
              ])
            : []),
        ]),
    ];
    if (!options.some(([v]) => v === source)) {
      options.push([source, `Source · ${source}`]);
    }
    fill(
      sourceSelect,
      options.map(([value, label]) => h('option', { value, selected: value === source }, label)),
    );
  };
  fillSources();

  const view = h('div', { class: 'log-view', role: 'log', 'aria-live': 'off' });
  const status = h('div', { class: 'logs-status' });
  const search = h('input', { class: 'input', placeholder: 'Search messages', type: 'search' });
  const levels = segmented<LevelFilter>(
    [
      { value: 'all', label: 'All' },
      { value: 'info', label: 'Info+' },
      { value: 'warning', label: 'Warnings' },
      { value: 'error', label: 'Errors' },
    ],
    level,
    (value) => {
      level = value;
      render();
    },
  );
  levels.root.classList.add('level-filter');
  const pauseButton = h('button', { type: 'button', class: 'btn small' });
  const setPaused = (value: boolean): void => {
    paused = value;
    fill(pauseButton, icon(paused ? 'play' : 'pause'), paused ? 'Resume' : 'Pause');
    if (!paused && pending.length > 0) {
      add(pending);
      pending = [];
    }
    renderStatus();
  };
  pauseButton.addEventListener('click', () => setPaused(!paused));

  const visible = (): LogLine[] =>
    lines.filter(
      (l) =>
        RANK[l.level] >= MIN_RANK[level] &&
        (!query || l.msg.toLowerCase().includes(query) || l.source.includes(query)),
    );

  const row = (line: LogLine): HTMLElement =>
    h(
      'div',
      { class: `log-line ${line.level}` },
      h('span', { class: 't' }, formatClock(line.t)),
      h('span', { class: 'lvl' }, line.level === 'warning' ? 'warn' : line.level),
      h('span', { class: 'src', title: line.source }, line.source),
      h('span', { class: 'msg' }, highlight(line.msg, query)),
    );

  const renderStatus = (): void => {
    const errors = lines.filter((l) => RANK[l.level] >= RANK.error).length;
    const warnings = lines.filter((l) => l.level === 'warning').length;
    fill(
      status,
      h('span', { class: `dot ${stop ? (paused ? 'warn' : 'ok') : 'danger'}` }),
      h('span', {}, stop ? (paused ? `Paused · ${pending.length} new` : 'Live') : 'Disconnected'),
      h('span', {}, `${lines.length} lines`),
      h('span', {}, `${warnings} warnings`),
      h('span', {}, `${errors} errors`),
    );
  };

  const render = (): void => {
    const shown = visible().slice(-RENDER_LINES);
    if (shown.length === 0) {
      fill(
        view,
        h(
          'div',
          { class: 'log-empty' },
          lines.length === 0 ? 'No log lines yet.' : 'No lines match the filter.',
        ),
      );
    } else {
      fill(view, shown.map(row));
    }
    if (follow) {
      view.scrollTop = view.scrollHeight;
    }
    renderStatus();
  };

  const add = (incoming: LogLine[]): void => {
    lines = lines.concat(incoming).slice(-MAX_LINES);
    const matching = incoming.filter(
      (l) => RANK[l.level] >= MIN_RANK[level] && (!query || l.msg.toLowerCase().includes(query)),
    );
    if (view.querySelector('.log-empty')) {
      render();
      return;
    }
    view.append(...matching.map(row));
    while (view.childElementCount > RENDER_LINES) {
      view.firstElementChild?.remove();
    }
    if (follow) {
      view.scrollTop = view.scrollHeight;
    }
    renderStatus();
  };

  const connect = async (): Promise<void> => {
    stop?.();
    stop = null;
    pending = [];
    try {
      lines = await fetchLogs(source, MAX_LINES);
    } catch (error) {
      toast((error as Error).message, 'error');
      lines = [];
    }
    render();
    stop = streamLogs(source, (line) => {
      if (paused) {
        pending.push(line);
        renderStatus();
      } else {
        add([line]);
      }
    });
    renderStatus();
  };

  sourceSelect.addEventListener('change', () => {
    source = sourceSelect.value;
    history.replaceState(
      null,
      '',
      source === '*' ? '/logs.html' : `/logs.html?source=${encodeURIComponent(source)}`,
    );
    void connect();
  });
  let searchTimer = 0;
  search.addEventListener('input', () => {
    clearTimeout(searchTimer);
    searchTimer = window.setTimeout(() => {
      query = search.value.trim().toLowerCase();
      render();
    }, 150);
  });
  view.addEventListener('scroll', () => {
    const atBottom = view.scrollHeight - view.scrollTop - view.clientHeight < 40;
    if (atBottom !== follow) {
      follow = atBottom;
      const input = followToggle.querySelector('input');
      if (input) {
        input.checked = follow;
      }
    }
  });
  const followToggle = toggle('Follow', follow, (on) => {
    follow = on;
    if (on) {
      view.scrollTop = view.scrollHeight;
    }
  });
  const download = h(
    'button',
    { type: 'button', class: 'btn small' },
    icon('download'),
    'Download',
  );
  download.addEventListener('click', () => {
    const text = visible()
      .map((l) => `${new Date(l.t * 1000).toISOString()} [${l.level}] [${l.source}] ${l.msg}`)
      .join('\n');
    const link = h('a', {
      href: URL.createObjectURL(new Blob([text], { type: 'text/plain' })),
      download: `jp-logs-${source === '*' ? 'all' : source}-${new Date().toISOString().slice(0, 19).replace(/:/g, '')}.log`,
    });
    link.click();
    URL.revokeObjectURL(link.href);
  });
  const clear = h(
    'button',
    {
      type: 'button',
      class: 'btn small ghost',
      title: 'Clear the view (the server keeps its log)',
    },
    icon('trash'),
    'Clear view',
  );
  clear.addEventListener('click', () => {
    lines = [];
    render();
  });
  setPaused(false);

  shell.content.append(
    h(
      'div',
      { class: 'logs-page' },
      h(
        'div',
        { class: 'logs-toolbar' },
        sourceSelect,
        levels.root,
        h('div', { class: 'search' }, icon('search'), search),
        h('span', { class: 'spacer' }),
        followToggle,
        pauseButton,
        clear,
        download,
      ),
      view,
      status,
    ),
  );
  shell.onSites(() => fillSources());
  await connect();
}

void start();
