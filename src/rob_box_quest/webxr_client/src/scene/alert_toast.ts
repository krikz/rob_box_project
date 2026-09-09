// Alert toast: показывает оператору robot_alert (BATTERY_LOW / WIFI_WEAK /
// ROBOT_STUCK) в шлеме. Три уровня цвета по level, авто-скрытие ~8 с,
// стек ≤ 3, дубль в status HUD как постоянная метка пока алёрт активен.
//
// Источник истины: docs/architecture/meta-quest-api.md §6
// (robot_alert codes + поля). Контракт сервера (AV-26 acceptance):
//   - поднятие: { type: "robot_alert", code, active:true, level:"warn"|"error",
//                 args:{...}, ts_ms }
//   - снятие:   { type: "robot_alert", code, active:false, level:"info",
//                 args:{...}, ts_ms }
//
// Снятие приходит отдельным событием с тем же code — клиент просто убирает
// алёрт из активных; toast (если ещё висит) дозакрывается по авто-таймеру.

import type { ErrorOverlay } from "../ui/error_overlay";

/** Один алёрт, как приходит от сервера. */
export interface RobotAlert {
  code: string;
  active: boolean;
  level: "warn" | "error" | "info";
  args?: Record<string, unknown>;
  ts_ms: number;
}

/** Коды, для которых у нас есть русский текст. Не найден → покажем сам код. */
const ALERT_TEXT: Record<string, (args: Record<string, unknown>) => string> = {
  BATTERY_LOW: (args) => {
    const pct = typeof args.pct === "number" ? Math.round(args.pct) : null;
    return pct === null ? "Батарея разряжена — пора на зарядку" : `Батарея ${pct}% — пора на зарядку`;
  },
  WIFI_WEAK: () => "Слабый Wi-Fi — связь с роботом нестабильна",
  ROBOT_STUCK: (args) => {
    const cmdLinear = typeof args.cmd_linear === "number" ? args.cmd_linear : null;
    const cmdAngular = typeof args.cmd_angular === "number" ? args.cmd_angular : null;
    if (cmdLinear === null || cmdAngular === null) {
      return "Робот не едет — возможно застрял";
    }
    return `Робот не едет (команда ${cmdLinear.toFixed(2)} м/с, ${cmdAngular.toFixed(2)} рад/с) — возможно застрял`;
  }
};

/** Чистая функция: текст алёрта на русском. Неизвестный код → сам код. */
export function alertText(alert: { code: string; args?: Record<string, unknown> }): string {
  const factory = ALERT_TEXT[alert.code];
  if (!factory) {
    // Не выдумываем описание для неизвестного кода (anti-capability-honest):
    // показываем сам код, чтобы оператор видел, что это что-то новое,
    // и сообщил инженеру. Лучше «BATTERY_CRITICAL» в HUD, чем молчание.
    return alert.code;
  }
  return factory(alert.args ?? {});
}

export interface AlertToastOptions {
  /** Куда монтировать toast-контейнер. */
  parent: HTMLElement;
  /** Сколько миллисекунд показывать toast (default 8000). */
  autoHideMs?: number;
  /** Максимум одновременно видимых toast'ов (default 3). */
  maxVisible?: number;
  /** Инжектимый error-overlay для дублирования критических алёртов. */
  errorOverlay?: ErrorOverlay | null;
}

export interface AlertToast {
  /** Принять событие от сервера; тост появляется/снимается. */
  ingest(event: RobotAlert): void;
  /** Снять ВСЕ активные тосты (например, при reconnect). */
  clear(): void;
  /** Кол-во тостов в стеке (для тестов и HUD-метки). */
  readonly activeCount: number;
  /** Текущий «самый важный» активный алёрт для HUD-метки (error > warn > info). */
  readonly worstActive: RobotAlert | null;
  dispose(): void;
}

const AUTO_HIDE_MS_DEFAULT = 8000;
const MAX_VISIBLE_DEFAULT = 3;
const SEVERITY: Record<RobotAlert["level"], number> = {
  error: 3,
  warn: 2,
  info: 1
};

interface ToastEntry {
  alert: RobotAlert;
  node: HTMLElement;
  /** handle setTimeout для авто-скрытия, чтобы тесты могли
   *  пользоваться vi.useFakeTimers() без патча performance.now(). */
  hideTimer: ReturnType<typeof setTimeout>;
}

export function createAlertToast(opts: AlertToastOptions): AlertToast {
  const autoHideMs = opts.autoHideMs ?? AUTO_HIDE_MS_DEFAULT;
  const maxVisible = opts.maxVisible ?? MAX_VISIBLE_DEFAULT;

  // Контейнер для всех toast'ов: position:fixed сверху-справа, не
  // перекрывает статус-HUD (status_hud.ts — верхний-левый угол стены).
  const root = document.createElement("div");
  root.className = "alert-toast-stack alert-toast-stack--hidden";
  root.setAttribute("data-alert-toast-stack", "");
  opts.parent.appendChild(root);

  const active = new Map<string, ToastEntry>();
  let disposed = false;

  function classify(alert: RobotAlert): "error" | "warn" | "info" {
    // На снятии (active:false) сервер шлёт level="info" — но мы НЕ
    // показываем toast для снятия, только гасим уже видимый.
    if (alert.level === "error" || alert.level === "warn") return alert.level;
    return "info";
  }

  function render(alert: RobotAlert): HTMLElement {
    const node = document.createElement("div");
    node.className = `alert-toast alert-toast--${classify(alert)}`;
    node.setAttribute("data-alert-code", alert.code);
    node.setAttribute("role", alert.level === "error" ? "alert" : "status");

    const text = document.createElement("div");
    text.className = "alert-toast__text";
    text.textContent = alertText(alert);

    const dismissBtn = document.createElement("button");
    dismissBtn.className = "alert-toast__dismiss";
    dismissBtn.type = "button";
    dismissBtn.textContent = "×";
    dismissBtn.setAttribute("aria-label", "Dismiss");
    dismissBtn.addEventListener("click", () => removeEntry(alert.code, false));

    node.appendChild(text);
    node.appendChild(dismissBtn);
    return node;
  }

  function enforceStackLimit(): void {
    // Если превышен лимит — убираем самые старые/наименее важные.
    if (active.size <= maxVisible) return;
    // Без order-метаданных удаляем по severity (наименее важные первые).
    const sorted = [...active.entries()].sort((a, b) => {
      const sev = SEVERITY[b[1].alert.level] - SEVERITY[a[1].alert.level];
      if (sev !== 0) return sev;
      // При равной важности — стабильный порядок по коду (детерминизм тестов).
      return a[0].localeCompare(b[0]);
    });
    const toRemove = sorted.slice(maxVisible);
    for (const [code] of toRemove) removeEntry(code, false);
  }

  function removeEntry(code: string, hideErrorOverlay: boolean): void {
    const entry = active.get(code);
    if (!entry) {
      // Toast'а нет (например, уже истёк) — но снимать error-overlay,
      // если он показан именно из-за этого кода, всё равно нужно.
      if (hideErrorOverlay && opts.errorOverlay?.isVisible) {
        opts.errorOverlay.dismiss();
      }
      syncRootVisibility();
      return;
    }
    clearTimeout(entry.hideTimer);
    if (entry.node.parentNode === root) {
      root.removeChild(entry.node);
    }
    active.delete(code);
    if (hideErrorOverlay && opts.errorOverlay?.isVisible) {
      opts.errorOverlay.dismiss();
    }
    syncRootVisibility();
  }

  function syncRootVisibility(): void {
    if (active.size === 0) {
      root.classList.add("alert-toast-stack--hidden");
    } else {
      root.classList.remove("alert-toast-stack--hidden");
    }
  }

  function ingest(event: RobotAlert): void {
    if (disposed) return;
    if (!event.active) {
      // Снятие: гасим тост и overlay (если был выставлен этим кодом).
      removeEntry(event.code, true);
      return;
    }
    // Активный → пересоздаём запись (новый ts_ms = новый таймер авто-скрытия).
    const existing = active.get(event.code);
    if (existing) {
      clearTimeout(existing.hideTimer);
      if (existing.node.parentNode === root) {
        root.removeChild(existing.node);
      }
      active.delete(event.code);
    }
    const node = render(event);
    root.appendChild(node);
    const hideTimer = setTimeout(() => {
      removeEntry(event.code, false);
    }, autoHideMs);
    active.set(event.code, {
      alert: event,
      node,
      hideTimer
    });
    enforceStackLimit();
    syncRootVisibility();
    // Критические алёрты дублируем в error_overlay (на случай, если
    // оператор смотрит мимо toast-стека).
    if (opts.errorOverlay && event.level === "error") {
      opts.errorOverlay.show(alertText(event));
    }
  }

  function clear(): void {
    for (const code of [...active.keys()]) removeEntry(code, false);
    if (opts.errorOverlay?.isVisible) {
      opts.errorOverlay.dismiss();
    }
  }

  function worstActive(): RobotAlert | null {
    let worst: RobotAlert | null = null;
    let worstSev = 0;
    for (const entry of active.values()) {
      const sev = SEVERITY[entry.alert.level];
      if (sev > worstSev) {
        worstSev = sev;
        worst = entry.alert;
      }
    }
    return worst;
  }

  return {
    ingest,
    clear,
    get activeCount(): number {
      return active.size;
    },
    get worstActive(): RobotAlert | null {
      return worstActive();
    },
    dispose(): void {
      if (disposed) return;
      disposed = true;
      clear();
      if (root.parentNode === opts.parent) {
        opts.parent.removeChild(root);
      }
    }
  };
}
