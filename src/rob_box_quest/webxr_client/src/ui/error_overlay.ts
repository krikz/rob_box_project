// Error overlay: показывает проблемы с подключением (WS disconnect > 5s,
// server errors, protocol version mismatch). Авто-скрытие при
// восстановлении соединения.
//
// Дизайн:
//   - Появляется поверх WebXR-вида (но в 2D HUD — это часть DOM).
//   - Не блокирует input; пользователь может продолжить смотреть стримы
//     пока сервер недоступен.
//   - Таймаут 5 секунд — порог «disconnect, который не восстановится сам»
//     (см. ADR-0027 §3.3 — reconnect backoff 1→30 с, после ~5 с клиент
//     уже явно «потерял» сервер).
//
// Использование:
//   const error = createErrorOverlay(parentEl);
//   error.show("Disconnected", "Reconnecting…");
//   error.setDetail("Last ping 12s ago");
//   error.dismiss();        // скрыть
//   error.onChange(cb);     // подписка на видимость
//   error.dispose();
//
// API:
export interface ErrorOverlay {
  show(headline: string, detail?: string): void;
  /** Обновить деталь (без изменения видимости). */
  setDetail(detail: string): void;
  /** Обновить заголовок без пересоздания overlay. */
  setHeadline(headline: string): void;
  /** Принудительно скрыть (например, после восстановления). */
  dismiss(): void;
  /** Подписаться на изменения видимости. */
  onChange(cb: (visible: boolean) => void): () => void;
  readonly isVisible: boolean;
  readonly isDisposed: boolean;
  dispose(): void;
}

export interface ErrorOverlayOptions {
  /** Класс CSS для уровня важности (info | warn | error). */
  level?: "info" | "warn" | "error";
  /** Автоматически скрыть при вызове dismiss() или при success(true). */
  autoDismiss?: boolean;
}

export function createErrorOverlay(
  parent: HTMLElement,
  options: ErrorOverlayOptions = {}
): ErrorOverlay {
  const root = document.createElement("div");
  root.className = "error-overlay error-overlay--hidden";
  root.setAttribute("data-error-overlay", "");
  root.setAttribute("role", "alert");
  root.setAttribute("aria-live", "assertive");

  const card = document.createElement("div");
  card.className = "error-overlay__card";

  const icon = document.createElement("div");
  icon.className = "error-overlay__icon";
  icon.setAttribute("aria-hidden", "true");
  icon.textContent = "⚠";

  const headline = document.createElement("div");
  headline.className = "error-overlay__headline";

  const detail = document.createElement("div");
  detail.className = "error-overlay__detail";

  const dismissBtn = document.createElement("button");
  dismissBtn.className = "error-overlay__dismiss";
  dismissBtn.type = "button";
  dismissBtn.textContent = "×";
  dismissBtn.setAttribute("aria-label", "Dismiss");
  dismissBtn.addEventListener("click", () => dismiss());

  card.appendChild(icon);
  card.appendChild(headline);
  card.appendChild(detail);
  card.appendChild(dismissBtn);
  root.appendChild(card);
  parent.appendChild(root);

  let visible = false;
  let disposed = false;
  const listeners = new Set<(v: boolean) => void>();

  function setLevel(level: "info" | "warn" | "error"): void {
    root.classList.remove(
      "error-overlay--info",
      "error-overlay--warn",
      "error-overlay--error"
    );
    root.classList.add(`error-overlay--${level}`);
  }
  setLevel(options.level ?? "warn");

  function emit(): void {
    for (const cb of listeners) cb(visible);
  }

  function show(headlineText: string, detailText?: string): void {
    if (disposed) return;
    headline.textContent = headlineText;
    detail.textContent = detailText ?? "";
    detail.hidden = detailText === undefined || detailText === "";
    root.classList.remove("error-overlay--hidden");
    if (!visible) {
      visible = true;
      emit();
    }
  }

  function setHeadline(text: string): void {
    if (disposed) return;
    headline.textContent = text;
  }

  function setDetail(text: string): void {
    if (disposed) return;
    detail.textContent = text;
    detail.hidden = text === "";
  }

  function dismiss(): void {
    if (disposed || !visible) return;
    root.classList.add("error-overlay--hidden");
    visible = false;
    emit();
  }

  function onChange(cb: (v: boolean) => void): () => void {
    listeners.add(cb);
    return () => {
      listeners.delete(cb);
    };
  }

  function dispose(): void {
    if (disposed) return;
    disposed = true;
    listeners.clear();
    if (root.parentNode === parent) {
      parent.removeChild(root);
    }
  }

  return {
    show,
    setHeadline,
    setDetail,
    dismiss,
    onChange,
    dispose,
    get isVisible(): boolean {
      return visible && !disposed;
    },
    get isDisposed(): boolean {
      return disposed;
    }
  };
}

// Утилита: обёртка для трекинга «disconnect > thresholdMs» с авто-show/hide.
// Использование:
//   const watchdog = createDisconnectWatchdog(errorOverlay, { thresholdMs: 5000 });
//   watchdog.markConnected();     // при WELCOME / hello
//   watchdog.markDisconnected();  // при onStateChange !== "connected"
//   watchdog.dispose();
export interface DisconnectWatchdog {
  markConnected(): void;
  markDisconnected(): void;
  dispose(): void;
  readonly elapsedMs: number;
  readonly isInDownState: boolean;
}

export interface DisconnectWatchdogOptions {
  /** Порог в мс, после которого показываем overlay. Default 5000. */
  thresholdMs?: number;
  /** Текст overlay (можно локализовать). */
  headline?: string;
  /** Префикс детали: "{elapsed}s without server ping". */
  detailTemplate?: string;
}

export function createDisconnectWatchdog(
  overlay: ErrorOverlay,
  options: DisconnectWatchdogOptions = {}
): DisconnectWatchdog {
  const thresholdMs = options.thresholdMs ?? 5000;
  const headline = options.headline ?? "Connection lost";
  const detailTemplate =
    options.detailTemplate ?? "No response from server for {seconds}s";

  let inDownState = false;
  let lastConnectTs = 0;
  let thresholdTimer: ReturnType<typeof setTimeout> | null = null;
  let tickInterval: ReturnType<typeof setInterval> | null = null;
  let elapsedMs = 0;
  let disposed = false;

  function clearTimers(): void {
    if (thresholdTimer !== null) {
      clearTimeout(thresholdTimer);
      thresholdTimer = null;
    }
    if (tickInterval !== null) {
      clearInterval(tickInterval);
      tickInterval = null;
    }
  }

  function showAfterThreshold(): void {
    if (disposed || !inDownState) return;
    overlay.show(headline, formatDetail(0));
    tickInterval = setInterval(() => {
      if (disposed || !inDownState) return;
      elapsedMs = performance.now() - lastConnectTs;
      overlay.setDetail(formatDetail(elapsedMs));
    }, 1000);
  }

  function formatDetail(ms: number): string {
    const seconds = Math.floor(ms / 1000);
    return detailTemplate.replace("{seconds}", String(seconds));
  }

  function markConnected(): void {
    if (disposed) return;
    inDownState = false;
    clearTimers();
    elapsedMs = 0;
    overlay.dismiss();
  }

  function markDisconnected(): void {
    if (disposed) return;
    if (inDownState) return;
    inDownState = true;
    lastConnectTs = performance.now();
    elapsedMs = 0;
    if (thresholdTimer !== null) clearTimeout(thresholdTimer);
    thresholdTimer = setTimeout(showAfterThreshold, thresholdMs);
  }

  function dispose(): void {
    if (disposed) return;
    disposed = true;
    clearTimers();
  }

  return {
    markConnected,
    markDisconnected,
    dispose,
    get elapsedMs(): number {
      return elapsedMs;
    },
    get isInDownState(): boolean {
      return inDownState;
    }
  };
}
