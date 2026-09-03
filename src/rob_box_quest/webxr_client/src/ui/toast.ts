// Toast: короткое уведомление для оператора («руль забрал оператор Telegram»).
//
// Дизайн:
//   - Не блокирует (non-modal): под input'ом ничего, поверх сцены.
//   - Авто-скрытие через `autoHideMs` (default 4 с).
//   - Очередь: новый тост сразу показывается, старые не суммируются
//     (single toast — чтобы не заспамить экран).
//   - Стиль близок к error_overlay (карточка с иконкой), но меньше и
//     ниже по иерархии (warn-уровень, не error).
//
// Использование:
//   const toast = createToast(parentEl);
//   toast.show("Руль забрал оператор Telegram", { level: "warn" });
//   toast.dispose();
//
// Это pure-DOM, без three.js. Тесты в tests/toast.test.ts.

export type ToastLevel = "info" | "warn" | "error";

export interface ToastOptions {
  /** Уровень важности (цвет + иконка). */
  level?: ToastLevel;
  /** Авто-скрыть через N мс. Default 4000. 0 = без авто-скрытия. */
  autoHideMs?: number;
}

export interface Toast {
  show(text: string, options?: ToastOptions): void;
  hide(): void;
  readonly isVisible: boolean;
  readonly isDisposed: boolean;
  dispose(): void;
}

const ICONS: Record<ToastLevel, string> = {
  info: "ⓘ",
  warn: "⚠",
  error: "✖"
};

const AUTO_HIDE_DEFAULT_MS = 4000;

export function createToast(parent: HTMLElement): Toast {
  const root = document.createElement("div");
  root.className = "toast toast--hidden";
  root.setAttribute("data-toast", "");
  root.setAttribute("role", "status");
  root.setAttribute("aria-live", "polite");

  const card = document.createElement("div");
  card.className = "toast__card";

  const icon = document.createElement("div");
  icon.className = "toast__icon";
  icon.setAttribute("aria-hidden", "true");

  const text = document.createElement("div");
  text.className = "toast__text";

  const dismissBtn = document.createElement("button");
  dismissBtn.className = "toast__dismiss";
  dismissBtn.type = "button";
  dismissBtn.textContent = "×";
  dismissBtn.setAttribute("aria-label", "Dismiss");
  dismissBtn.addEventListener("click", () => hide());

  card.appendChild(icon);
  card.appendChild(text);
  card.appendChild(dismissBtn);
  root.appendChild(card);
  parent.appendChild(root);

  let visible = false;
  let disposed = false;
  let hideTimer: ReturnType<typeof setTimeout> | null = null;

  function setLevel(level: ToastLevel): void {
    root.classList.remove("toast--info", "toast--warn", "toast--error");
    root.classList.add(`toast--${level}`);
    icon.textContent = ICONS[level];
  }

  function clearHideTimer(): void {
    if (hideTimer !== null) {
      clearTimeout(hideTimer);
      hideTimer = null;
    }
  }

  function show(textValue: string, options: ToastOptions = {}): void {
    if (disposed) return;
    const level = options.level ?? "info";
    const autoHideMs = options.autoHideMs ?? AUTO_HIDE_DEFAULT_MS;
    setLevel(level);
    text.textContent = textValue;
    root.classList.remove("toast--hidden");
    visible = true;
    clearHideTimer();
    if (autoHideMs > 0) {
      hideTimer = setTimeout(() => hide(), autoHideMs);
    }
  }

  function hide(): void {
    if (disposed || !visible) return;
    root.classList.add("toast--hidden");
    visible = false;
    clearHideTimer();
  }

  function dispose(): void {
    if (disposed) return;
    disposed = true;
    clearHideTimer();
    if (root.parentNode === parent) {
      parent.removeChild(root);
    }
  }

  return {
    show,
    hide,
    get isVisible(): boolean {
      return visible && !disposed;
    },
    get isDisposed(): boolean {
      return disposed;
    },
    dispose
  };
}
