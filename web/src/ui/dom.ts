/** Small DOM helpers: the console is plain TypeScript without a UI framework. */

export type Child = Node | string | number | null | undefined | false;
type AttrValue = string | number | boolean | null | undefined | ((event: Event) => void);

/**
 * Creates an element. Attributes starting with "on" are event listeners, `true` sets a boolean
 * attribute and null, undefined or false leave the attribute out.
 */
export function h<K extends keyof HTMLElementTagNameMap>(
  tag: K,
  attrs: Record<string, AttrValue> = {},
  ...children: (Child | Child[])[]
): HTMLElementTagNameMap[K] {
  const element = document.createElement(tag);
  for (const [key, value] of Object.entries(attrs)) {
    if (value === null || value === undefined || value === false) {
      continue;
    }
    if (typeof value === 'function') {
      element.addEventListener(key.slice(2), value);
    } else {
      element.setAttribute(key, value === true ? '' : String(value));
    }
  }
  element.append(...nodes(children));
  return element;
}

function nodes(children: (Child | Child[])[]): (Node | string)[] {
  return children
    .flat()
    .filter((c): c is Node | string | number => c !== null && c !== undefined && c !== false)
    .map((c) => (typeof c === 'number' ? String(c) : c));
}

/** Replaces the children of an element. */
export function fill(element: Element, ...children: (Child | Child[])[]): void {
  element.replaceChildren(...nodes(children));
}

export function element<T extends HTMLElement>(selector: string, root: ParentNode = document): T {
  const found = root.querySelector<T>(selector);
  if (!found) {
    throw new Error(`missing element ${selector}`);
  }
  return found;
}

/** Query parameter of the page URL, e.g. the site of ?site=munich_5538. */
export function param(name: string): string | null {
  return new URLSearchParams(location.search).get(name);
}

/** Toggle switch: <label class="switch"><input type="checkbox"><span class="track"></span>label</label>. */
export function toggle(
  label: string,
  checked: boolean,
  onChange: (checked: boolean, input: HTMLInputElement) => void,
): HTMLLabelElement {
  const input = h('input', { type: 'checkbox' });
  input.checked = checked;
  input.addEventListener('change', () => onChange(input.checked, input));
  return h('label', { class: 'switch' }, input, h('span', { class: 'track' }), label);
}

/** Segmented control; returns the element and a setter for the active value. */
export function segmented<T extends string>(
  options: { value: T; label: string; icon?: Node; disabled?: boolean }[],
  active: T,
  onChange: (value: T) => void,
): { root: HTMLElement; set: (value: T) => void } {
  const buttons = options.map((option) =>
    h(
      'button',
      {
        type: 'button',
        'data-value': option.value,
        disabled: option.disabled,
        onclick: () => {
          set(option.value);
          onChange(option.value);
        },
      },
      option.icon ?? null,
      option.label,
    ),
  );
  const set = (value: T): void => {
    for (const button of buttons) {
      button.classList.toggle('active', button.dataset['value'] === value);
    }
  };
  set(active);
  return { root: h('div', { class: 'segmented' }, buttons), set };
}
