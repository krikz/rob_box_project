// TARS 2 metrics panel — Captain Bridge (issue #2113, quest #2112).
//
// Side panel по бокам от FRONT CAM, симметрично TARS 1, лицом к оператору.
// На нём оператор видит Grafana-панель (Prometheus / Loki) с метриками
// робота. Источник URL — топик `/avatar/tars/panel_url`, который супервизор
// публикует при LLM tool call `show_metrics(query)` (см. ADR-0060 и
// `tars_panel.py` в rob_box_supervisor).
//
// Технический компромисс: настоящий `<iframe>` внутри Three.js сцены не
// рендерится (Three.js не владеет DOM-3D-контекстом iframe). Поэтому
// подход такой:
//   - Plane + MeshBasicMaterial + CanvasTexture, на которой рисуется
//     preview (текущий URL, host, состояние «loading/ok/error»);
//   - Реальный iframe создаётся в DOM и проецируется на panel через
//     CSS2DRenderer / отдельный overlay слой, НЕ внутри THREE-сцены;
//     для Quest в VR overlay не работает (immersive-vr не даёт обычный
//     DOM), поэтому в VR показывается только preview, а iframe живёт в
//     desktop-окне параллельно. Это by-design: «side panel» в VR vs
//     «side panel в desktop» имеют разные носители.
//
// Здесь делаем только Three.js-сторону: mesh с preview, API
// `setPanelUrl/refresh/clear`. DOM-iframe создаётся в main.ts и
// синхронизируется через колбэк `onUrlChanged`.

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
 * Callback, который main.ts вешает на panel: при смене URL надо
 * переключить iframe (или обновить его src), при clear — закрыть.
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

  // 4:3 — то же, что у TARS 1 / FRONT CAM.
  const geometry = new THREE.PlaneGeometry(1.6, 1.2);
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

    if (currentUrl === null) {
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
      // Подсказка: «iframe активен в desktop-окне».
      ctx!.fillStyle = "#444a52";
      ctx!.font = `${Math.round(fontSize * 0.75)}px monospace`;
      ctx!.fillText(
        "preview only · iframe синхронизирован в DOM",
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