import { h } from './dom';
import { icon, type IconName } from './icons';

const TOAST_MS = 4500;

/** Short notice in the bottom right corner. */
export function toast(message: string, kind: 'ok' | 'error' | 'info' = 'info'): void {
  let container = document.querySelector<HTMLElement>('.toasts');
  if (!container) {
    container = h('div', { class: 'toasts', role: 'status', 'aria-live': 'polite' });
    document.body.append(container);
  }
  const names: Record<typeof kind, IconName> = { ok: 'check', error: 'alert', info: 'info' };
  const item = h('div', { class: `toast ${kind}` }, icon(names[kind]), h('span', {}, message));
  container.append(item);
  setTimeout(() => item.remove(), kind === 'error' ? TOAST_MS * 2 : TOAST_MS);
}

export interface ConfirmOptions {
  title: string;
  body: string | Node;
  confirm: string;
  danger?: boolean;
  /** The user has to type this text to enable the confirm button. */
  typeToConfirm?: string;
}

/** Modal confirmation; resolves to true when confirmed. */
export function confirmDialog(options: ConfirmOptions): Promise<boolean> {
  return new Promise((resolve) => {
    const confirmButton = h(
      'button',
      { type: 'button', class: `btn ${options.danger ? 'danger' : 'primary'}` },
      options.confirm,
    );
    const cancel = h('button', { type: 'button', class: 'btn ghost' }, 'Cancel');
    const body = h(
      'div',
      { class: 'body' },
      typeof options.body === 'string' ? h('p', {}, options.body) : options.body,
    );
    if (options.typeToConfirm) {
      const expected = options.typeToConfirm;
      const input = h('input', { class: 'input mono', placeholder: expected, autocomplete: 'off' });
      confirmButton.disabled = true;
      input.addEventListener('input', () => (confirmButton.disabled = input.value !== expected));
      body.append(
        h('p', {}, 'Type ', h('strong', { class: 'mono' }, expected), ' to confirm.'),
        input,
      );
      setTimeout(() => input.focus(), 0);
    }
    const dialog = h(
      'div',
      { class: `dialog${options.danger ? ' danger' : ''}`, role: 'dialog', 'aria-modal': 'true' },
      h('header', {}, icon(options.danger ? 'alert' : 'info'), h('h2', {}, options.title)),
      body,
      h('footer', {}, cancel, confirmButton),
    );
    const backdrop = h('div', { class: 'backdrop' }, dialog);
    const close = (result: boolean): void => {
      backdrop.remove();
      window.removeEventListener('keydown', onKey);
      resolve(result);
    };
    const onKey = (e: KeyboardEvent): void => {
      if (e.key === 'Escape') {
        close(false);
      }
    };
    cancel.addEventListener('click', () => close(false));
    confirmButton.addEventListener('click', () => close(true));
    backdrop.addEventListener('click', (e) => {
      if (e.target === backdrop) {
        close(false);
      }
    });
    window.addEventListener('keydown', onKey);
    document.body.append(backdrop);
    if (!options.typeToConfirm) {
      confirmButton.focus();
    }
  });
}

/** Runs an action from a button: disables it meanwhile and reports errors as a toast. */
export async function withBusy(
  button: HTMLButtonElement | null,
  action: () => Promise<unknown>,
  success?: string,
): Promise<boolean> {
  if (button) {
    button.disabled = true;
  }
  try {
    await action();
    if (success) {
      toast(success, 'ok');
    }
    return true;
  } catch (error) {
    toast((error as Error).message, 'error');
    return false;
  } finally {
    if (button) {
      button.disabled = false;
    }
  }
}
