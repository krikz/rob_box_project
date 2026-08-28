// Loading overlay: показывается пока грузятся CC0 GLB/HDR ассеты
// (Phase 2.1 Captain Bridge environment). Автоматически скрывается
// когда loadingPromise резолвится. API чистый — без зависимости от Three.js,
// чтобы можно было покрыть unit-тестами в jsdom.
//
// Дизайн: overlay занимает весь viewport, фон — матовое затемнение (как
// PIN-overlay), по центру — spinner + текст прогресса. Никаких
// зависимостей от конкретного bridge.handle: только promise и текст.
//
// Использование:
//   const loading = createLoadingScreen(parentEl, "Loading environment…");
//   loading.hide();        // показать
//   loading.show("Loaded"); // обновить текст и оставить
//   loading.dispose();     // убрать из DOM
//   loading.fail("Network"); // показать ошибку (визуально)
//
// Опции безопасны: каждый метод идемпотентен и не падает после dispose().

export interface LoadingScreenOptions {
  /** Минимальное видимое время (мс) — иначе на быстром кеше
   *  overlay мигает. По умолчанию 250 мс. */
  minVisibleMs?: number;
  /** Автоматически скрыться после resolve(promise). По умолчанию true. */
  autoHide?: boolean;
}

export interface LoadingScreen {
  /** Показать overlay с текстом (по умолчанию "Loading…"). */
  show(text?: string): void;
  /** Скрыть overlay; no-op если уже скрыт или disposed. */
  hide(): void;
  /** Обновить текст без изменения видимости. */
  setText(text: string): void;
  /** Пометить как failed (overlay остаётся, текст = error). */
  fail(reason: string): void;
  /** Привязать promise: при resolve вызовется hide(), при reject — fail(). */
  watch<T>(promise: Promise<T>, label?: string): Promise<T>;
  /** Удалить overlay из DOM. После этого все методы — no-op. */
  dispose(): void;
  readonly isVisible: boolean;
  readonly isDisposed: boolean;
}

export function createLoadingScreen(
  parent: HTMLElement,
  initialText = "Loading…",
  options: LoadingScreenOptions = {}
): LoadingScreen {
  const minVisibleMs = options.minVisibleMs ?? 250;
  const autoHide = options.autoHide ?? true;

  // Корневой контейнер overlay.
  const root = document.createElement("div");
  root.className = "loading-screen";
  root.setAttribute("data-loading-screen", "");
  root.setAttribute("role", "status");
  root.setAttribute("aria-live", "polite");
  root.setAttribute("aria-busy", "true");

  const card = document.createElement("div");
  card.className = "loading-screen__card";

  const spinner = document.createElement("div");
  spinner.className = "loading-screen__spinner";
  spinner.setAttribute("aria-hidden", "true");

  const text = document.createElement("div");
  text.className = "loading-screen__text";
  text.textContent = initialText;

  const progress = document.createElement("div");
  progress.className = "loading-screen__progress";
  // Мини-прогресс (5 точек) — для визуальной обратной связи пока promise
  // не резолвится. Никакой реальной progress-bar не делаем (CC0 GLB
  // грузятся за <1с на dev-машине).
  progress.textContent = "";

  card.appendChild(spinner);
  card.appendChild(text);
  card.appendChild(progress);
  root.appendChild(card);
  parent.appendChild(root);

  let visible = true;
  let disposed = false;
  let firstShowTs = performance.now();

  function show(textValue = initialText): void {
    if (disposed) return;
    text.textContent = textValue;
    root.classList.remove("loading-screen--hidden");
    if (!visible) {
      visible = true;
      firstShowTs = performance.now();
    }
    root.setAttribute("aria-busy", "true");
  }

  function hide(): void {
    if (disposed || !visible) return;
    const elapsed = performance.now() - firstShowTs;
    if (elapsed < minVisibleMs) {
      // Дать overlay отработать минимальное время — иначе мигает.
      const remaining = minVisibleMs - elapsed;
      setTimeout(() => doHide(), remaining);
      return;
    }
    doHide();
  }

  function doHide(): void {
    if (disposed) return;
    root.classList.add("loading-screen--hidden");
    root.setAttribute("aria-busy", "false");
    visible = false;
  }

  function setText(textValue: string): void {
    if (disposed) return;
    text.textContent = textValue;
  }

  function fail(reason: string): void {
    if (disposed) return;
    text.textContent = `Error: ${reason}`;
    root.classList.add("loading-screen--failed");
    root.setAttribute("aria-busy", "false");
  }

  function watch<T>(promise: Promise<T>, label?: string): Promise<T> {
    if (label !== undefined) setText(label);
    return promise.then(
      (value) => {
        if (autoHide) hide();
        return value;
      },
      (err: unknown) => {
        const reason = err instanceof Error ? err.message : String(err);
        fail(reason);
        // Re-throw чтобы caller знал, что promise отвергнут.
        throw err;
      }
    );
  }

  function dispose(): void {
    if (disposed) return;
    disposed = true;
    if (root.parentNode === parent) {
      parent.removeChild(root);
    }
  }

  return {
    show,
    hide,
    setText,
    fail,
    watch,
    dispose,
    get isVisible(): boolean {
      return visible && !disposed;
    },
    get isDisposed(): boolean {
      return disposed;
    }
  };
}
