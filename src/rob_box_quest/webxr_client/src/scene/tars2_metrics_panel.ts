// TARS 2 metrics panel — Captain Bridge (issue #2113, quest #2112).
//
// Side panel по бокам от FRONT CAM, симметрично TARS 1, лицом к оператору.
// На нём оператор видит Grafana-панель (Prometheus / Loki) с метриками
// робота. Источник URL — топик `/avatar/tars/panel_url`, который супервизор
// публикует при LLM tool call `show_metrics(query)` (см. ADR-0060 и
// `tars_panel.py` в rob_box_supervisor).
//
// Технический компромисс (решение зафиксировано в PR #2114/issue #2113,
// см. "TARS2 — честное инженерное решение" в описании PR): настоящий
// `<iframe>` внутри immersive-WebXR НЕ рендерится вообще — Three.js не
// владеет DOM-3D-контекстом iframe, а в immersive-режиме браузер не
// проецирует DOM/overlay на плоскость в мире (DOM-оверлей работает только
// в inline/2D-режиме, не в VR-сессии). «TARS 2 показывает iframe с
// Grafana» в 3D-сцене поэтому недостижимо в принципе, не только сейчас.
//
// Рабочая альтернатива — та же, что уже используют камерные панели
// (video_panel.ts + CompressedImage-тракт): растровое изображение на
// текстуре плоскости. Grafana умеет отдавать PNG панели через
// `/render/d-solo/...` (image-renderer plugin), что можно было бы
// прокачивать тем же путём, что и camera_rear/camera_ceiling. На момент
// этой карточки renderer-plugin на Grafana (katana) не установлен, а
// анонимный доступ выключен (проверено с робота: `/api/health` отвечает,
// но `/api/search` — 401, `GF_AUTH_ANONYMOUS_ENABLED` не задан) — поэтому
// PNG-путь не собран, чтобы не изобретать хождение с чужими кредами.
// Список того, что нужно настроить, — в PR.
//
// Что сделано здесь и сейчас: URL долетает до клиента честно (см.
// main.ts: JSON_EVENT{type:"tars_panel_url"} → setPanelUrl/setState), а
// panel рисует host/path текстом на canvas-preview — это единственный
// рендер, который есть, и он не притворяется живой Grafana-панелью.
// Никакого DOM-iframe эта карточка не создаёт: `onUrlChanged` остаётся
// заведённым API для будущего PNG-тракта (или desktop-only overlay), но
// в main.ts на него ничего не подписано.

import * as THREE from "three";

export interface Tars2MetricsPanelOptions {
  /** Ширина canvas в пикселях (default 512). */
  canvasWidth?: number;
  /** Высота canvas в пикселях (default 384). */
  canvasHeight?: number;
  /** Размер шрифта заголовка (default 18). */
  fontSize?: number;
}

export type Tars2PanelState = "idle" | "loading" | "ok" | "error";

/**
 * Не используется в main.ts на момент issue #2113 (нет DOM-iframe и
 * нет собранного PNG-тракта — см. комментарий в шапке файла). API
 * оставлен как точка расширения для будущего PNG/desktop-overlay пути:
 * при смене URL сюда прилетит `{kind:'url', url}`, при clear — `{kind:
 * 'clear'}`.
 */
export interface Tars2UrlListener {
  (event: { kind: "url"; url: string } | { kind: "clear" }): void;
}

export interface Tars2MetricsPanelHandle {
  readonly mesh: THREE.Mesh;
  /** Текущий URL (или null). */
  readonly currentUrl: string | null;
  /** Состояние («нет URL» / «грузим» / «ок» / «ошибка»). */
  readonly state: Tars2PanelState;
  /**
   * Установить новый URL. Триггерит ре-рендер preview и колбэк
   * `onUrlChanged({kind:'url', url})`. Если URL === текущему — игнор
   * (оператор не должен видеть двойной loading).
   */
  setPanelUrl(url: string): void;
  /** Очистить panel (URL → null, состояние → idle). */
  clear(): void;
  /** Пометить состояние (например, при ошибке iframe load). */
  setState(state: Tars2PanelState): void;
  /** Колбэк, который вызывается при смене URL. Устанавливается извне. */
  setOnUrlChanged(listener: Tars2UrlListener | null): void;
  getStats(): { url: string | null; state: Tars2PanelState };
  dispose(): void;
}

export function createTars2MetricsPanel(
  opts: Tars2MetricsPanelOptions = {}
): Tars2MetricsPanelHandle {
  const canvasWidth = opts.canvasWidth ?? 512;
  const canvasHeight = opts.canvasHeight ?? 384;
  const fontSize = opts.fontSize ?? 18;

  const canvas = document.createElement("canvas");
  canvas.width = canvasWidth;
  canvas.height = canvasHeight;
  const ctx = canvas.getContext("2d");
  if (!ctx) {
    throw new Error("Tars2MetricsPanel: failed to acquire 2D context");
  }

  const texture = new THREE.CanvasTexture(canvas);
  texture.minFilter = THREE.LinearFilter;
  texture.magFilter = THREE.LinearFilter;
  texture.colorSpace = THREE.SRGBColorSpace;

  // 16:9 — по ADR-0074 §4.0 (вариант E: все три экрана Captain Bridge
  // одного aspect ratio, как основной 4.8 × 2.7). Плоскость по умолчанию
  // 1.6 × 0.9 м; реальный размер задаётся снаружи через mesh.scale
  // (см. captain_bridge.ts: TARS_PANEL_SIZE). При смене ширины панели
  // архитектором (диапазон 2.4–3.2 м) PlaneGeometry не меняется — здесь
  // только отношение сторон.
  const geometry = new THREE.PlaneGeometry(1.6, 0.9);
  const material = new THREE.MeshBasicMaterial({
    map: texture,
    side: THREE.DoubleSide,
    toneMapped: false
  });
  const mesh = new THREE.Mesh(geometry, material);

  let currentUrl: string | null = null;
  let state: Tars2PanelState = "idle";
  let onUrlChanged: Tars2UrlListener | null = null;

  function render(): void {
    // Подложка — тёмная палитра Captain Bridge.
    ctx!.fillStyle = "#0a0d11";
    ctx!.fillRect(0, 0, canvasWidth, canvasHeight);

    // Цветной индикатор состояния: сверху, как в TARS 1 (но шире — 8 px).
    const stateColor: Record<Tars2PanelState, string> = {
      idle: "#444a52",
      loading: "#8fd4ff",
      ok: "#2ec27e",
      error: "#e64568"
    };
    ctx!.fillStyle = stateColor[state];
    ctx!.fillRect(0, 0, canvasWidth, 8);

    // Заголовок.
    ctx!.fillStyle = "#8fd4ff";
    ctx!.font = `bold ${Math.round(fontSize * 0.9)}px monospace`;
    ctx!.textBaseline = "top";
    ctx!.fillText("TARS 2 ▸ ", 8, 12);

    // Состояние справа от заголовка.
    ctx!.fillStyle = stateColor[state];
    ctx!.font = `bold ${Math.round(fontSize * 0.8)}px monospace`;
    const labelX = 8 + ctx!.measureText("TARS 2 ▸ ").width;
    ctx!.fillText(state.toUpperCase(), labelX, 14);

    if (currentUrl === null && state === "error") {
      // issue #2113: честное состояние вместо пустоты. avatar_supervisor
      // (tars_panel.py) публикует status="error" на пустой query/
      // неизвестный datasource — url в этом случае пуст, main.ts зовёт
      // setState("error") без setPanelUrl. Показываем это явно, а не
      // молчим и не притворяемся, что панель просто пустая/idle.
      ctx!.fillStyle = "#e64568";
      ctx!.font = `${fontSize}px monospace`;
      ctx!.fillText("Нет доступа к Grafana /", 8, 48);
      ctx!.fillText("ошибка запроса.", 8, 48 + Math.round(fontSize * 1.4));
      ctx!.fillStyle = "#8b98a5";
      ctx!.font = `${Math.round(fontSize * 0.85)}px monospace`;
      ctx!.fillText("Спроси ТАРС ещё раз.", 8, 48 + Math.round(fontSize * 3.0));
    } else if (currentUrl === null) {
      // Пустое состояние: подсказка оператору.
      ctx!.fillStyle = "#8b98a5";
      ctx!.font = `${fontSize}px monospace`;
      ctx!.fillText('Idle.', 8, 48);
      ctx!.fillText('Скажи: "TARS, покажи', 8, 48 + Math.round(fontSize * 1.4));
      ctx!.fillText('  дрейф CPU".', 8, 48 + Math.round(fontSize * 2.8));
    } else {
      // URL — большими буквами, моноширинно, чтобы помещался.
      ctx!.fillStyle = "#e6edf3";
      ctx!.font = `${fontSize}px monospace`;
      const urlY = 48;
      const host = parseUrlHost(currentUrl);
      const path = parseUrlPath(currentUrl);
      ctx!.fillText(host, 8, urlY);
      ctx!.fillStyle = "#8b98a5";
      ctx!.font = `${Math.round(fontSize * 0.85)}px monospace`;
      ctx!.fillText(truncate(path, 64), 8, urlY + Math.round(fontSize * 1.3));
      // Честная подсказка (issue #2113): это текстовый preview URL, не
      // живая Grafana-панель — реального рендера в immersive-WebXR нет
      // (см. комментарий в шапке файла).
      ctx!.fillStyle = "#444a52";
      ctx!.font = `${Math.round(fontSize * 0.75)}px monospace`;
      ctx!.fillText(
        "preview only · рендер Grafana не подключён",
        8,
        canvasHeight - 24
      );
    }

    texture.needsUpdate = true;
  }

  function setPanelUrl(url: string): void {
    if (typeof url !== "string" || url.length === 0) {
      // eslint-disable-next-line no-console
      console.warn("[tars2_metrics_panel] setPanelUrl: empty/non-string url, ignored");
      return;
    }
    if (url === currentUrl) return;
    currentUrl = url;
    state = "loading";
    render();
    const listener = onUrlChanged;
    if (listener) listener({ kind: "url", url });
  }

  function clear(): void {
    if (currentUrl === null && state === "idle") return;
    currentUrl = null;
    state = "idle";
    render();
    const listener = onUrlChanged;
    if (listener) listener({ kind: "clear" });
  }

  function setState(next: Tars2PanelState): void {
    if (state === next) return;
    state = next;
    render();
  }

  function setOnUrlChanged(listener: Tars2UrlListener | null): void {
    onUrlChanged = listener;
  }

  function getStats(): { url: string | null; state: Tars2PanelState } {
    return { url: currentUrl, state };
  }

  function dispose(): void {
    (mesh.geometry as THREE.BufferGeometry).dispose();
    (mesh.material as THREE.Material).dispose();
    texture.dispose();
  }

  render();

  return {
    mesh,
    get currentUrl(): string | null {
      return currentUrl;
    },
    get state(): Tars2PanelState {
      return state;
    },
    setPanelUrl,
    clear,
    setState,
    setOnUrlChanged,
    getStats,
    dispose
  };
}

// ───────────────────────── helpers ─────────────────────────

function parseUrlHost(url: string): string {
  try {
    return new URL(url).host;
  } catch {
    return url.slice(0, 32);
  }
}

function parseUrlPath(url: string): string {
  try {
    return new URL(url).pathname;
  } catch {
    return "";
  }
}

function truncate(s: string, max: number): string {
  if (s.length <= max) return s;
  return `…${s.slice(-(max - 1))}`;
}