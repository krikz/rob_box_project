// TARS 2 metrics panel — Captain Bridge (issue #2113, #2184; quest #2112).
//
// Side panel по бокам от FRONT CAM, симметрично TARS 1, лицом к оператору.
// На нём оператор видит телеметрию робота: график Prometheus или хвост логов
// Loki. Источник — топик `/avatar/tars/panel_data`, который супервизор
// публикует после LLM tool call `show_metrics(query)` (см. `tars_panel.py`
// и `metrics_source.py` в rob_box_supervisor).
//
// Почему график рисуется здесь, а не приходит картинкой из Grafana
// (issue #2184):
//
//   * настоящий `<iframe>` внутри immersive-WebXR не рендерится вообще —
//     Three.js не владеет DOM-3D-контекстом iframe, а в immersive-режиме
//     браузер не проецирует DOM/overlay на плоскость в мире. «TARS 2
//     показывает iframe с Grafana» недостижимо в принципе, не только сейчас;
//   * PNG-путь (`/render/d-solo/...`) требует image-renderer plugin, которого
//     на Grafana (katana) нет, и авторизации — анонимный доступ выключен
//     (`/api/search` → 401, проверено 08.09.2026).
//
// Зато Prometheus и Loki отдают JSON по HTTP без авторизации, супервизор их
// уже спросил, а нарисовать ряд точек на canvas-текстуре — ровно то, что этот
// модуль и делает. Никакого DOM здесь по-прежнему нет.
//
// До #2184 панель показывала только host/path пришедшего URL с подписью
// «preview only · рендер Grafana не подключён» — оператор не видел ни одной
// цифры. `setPanelUrl` остался (старые сборки сервера шлют только
// `tars_panel_url`), но теперь это деградированный режим, а не основной.

import * as THREE from "three";

export interface Tars2MetricsPanelOptions {
  /** Ширина canvas в пикселях (default 1280 — 16:9 как у основного экрана). */
  canvasWidth?: number;
  /** Высота canvas в пикселях (default 720 — 16:9). */
  canvasHeight?: number;
  /** Базовый размер шрифта (default 24 — панель 4.8 м читается с ~2 м). */
  fontSize?: number;
}

export type Tars2PanelState = "idle" | "loading" | "ok" | "empty" | "error";

/** Ряд Prometheus: точки `[unix_seconds, value]` по возрастанию времени. */
export interface Tars2Series {
  name: string;
  labels?: Record<string, string>;
  points: [number, number][];
}

/** Строка Loki (новые — первыми). */
export interface Tars2LogLine {
  ts: number;
  line: string;
  labels?: Record<string, string>;
}

/** Полезная нагрузка `/avatar/tars/panel_data` (JSON_EVENT tars_panel_data). */
export interface Tars2PanelData {
  status: "ok" | "empty" | "error" | string;
  datasource?: string;
  query?: string;
  /** Заметка о подмене запроса («cpu» → `rate(...)`) — показываем честно. */
  note?: string;
  /** Одна строка, которую ТАРС произносит вслух; дублируем на экране. */
  summary?: string;
  series?: Tars2Series[];
  lines?: Tars2LogLine[];
  /** Что вообще есть в Prometheus — подсказка на пустом результате. */
  available?: string[];
  url?: string;
  error?: string;
}

/**
 * Колбэк смены URL. Оставлен как точка расширения (desktop-overlay,
 * будущий PNG-тракт); основной путь отрисовки — `setPanelData`.
 */
export interface Tars2UrlListener {
  (event: { kind: "url"; url: string } | { kind: "clear" }): void;
}

export interface Tars2MetricsPanelHandle {
  readonly mesh: THREE.Mesh;
  /** Текущий URL (или null). */
  readonly currentUrl: string | null;
  /** Последние пришедшие данные (или null). */
  readonly currentData: Tars2PanelData | null;
  /** Состояние («нет данных» / «грузим» / «ок» / «пусто» / «ошибка»). */
  readonly state: Tars2PanelState;
  /**
   * Отрисовать реальные данные из `/avatar/tars/panel_data`. Это основной
   * вход панели: график Prometheus, хвост Loki либо честное «данных нет».
   */
  setPanelData(data: Tars2PanelData): void;
  /**
   * Деградированный вход: пришёл только URL (старый сервер без
   * `/avatar/tars/panel_data`). Рисует ссылку текстом — как до #2184.
   */
  setPanelUrl(url: string): void;
  /** Очистить panel (данные и URL → null, состояние → idle). */
  clear(): void;
  /** Пометить состояние (например, при ошибке запроса). */
  setState(state: Tars2PanelState): void;
  /** Колбэк, который вызывается при смене URL. Устанавливается извне. */
  setOnUrlChanged(listener: Tars2UrlListener | null): void;
  getStats(): {
    url: string | null;
    state: Tars2PanelState;
    seriesCount: number;
    pointCount: number;
  };
  dispose(): void;
}

// Палитра линий. Первый цвет — фирменный голубой Captain Bridge; дальше —
// различимые в VR оттенки (проверялись на тёмном фоне #0a0d11).
const SERIES_COLORS = [
  "#8fd4ff",
  "#2ec27e",
  "#f6c177",
  "#e64568",
  "#c77dff",
  "#7ee8fa"
];

export function createTars2MetricsPanel(
  opts: Tars2MetricsPanelOptions = {}
): Tars2MetricsPanelHandle {
  const canvasWidth = opts.canvasWidth ?? 1280;
  const canvasHeight = opts.canvasHeight ?? 720;
  const fontSize = opts.fontSize ?? 24;

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
  // одного aspect ratio, как основной 4.8 × 2.7). Геометрия — ЕДИНИЧНЫЙ
  // план (1×1): реальный размер в метрах задаётся снаружи через
  // mesh.scale.set(width, height, 1) (см. captain_bridge.ts:
  // TARS_PANEL_SIZE). ВАЖНО: не-единичная геометрия здесь (было 1.6×0.9 до
  // bugfix #2142-B) даёт двойное масштабирование — итоговый мировой размер
  // = geometry-size × scale, а не просто scale.
  const geometry = new THREE.PlaneGeometry(1, 1);
  const material = new THREE.MeshBasicMaterial({
    map: texture,
    side: THREE.DoubleSide,
    toneMapped: false
  });
  const mesh = new THREE.Mesh(geometry, material);

  let currentUrl: string | null = null;
  let currentData: Tars2PanelData | null = null;
  let state: Tars2PanelState = "idle";
  let onUrlChanged: Tars2UrlListener | null = null;

  const stateColor: Record<Tars2PanelState, string> = {
    idle: "#444a52",
    loading: "#8fd4ff",
    ok: "#2ec27e",
    empty: "#f6c177",
    error: "#e64568"
  };

  // ─────────────────────── каркас панели ───────────────────────

  function drawChrome(): void {
    ctx!.fillStyle = "#0a0d11";
    ctx!.fillRect(0, 0, canvasWidth, canvasHeight);
    ctx!.fillStyle = stateColor[state];
    ctx!.fillRect(0, 0, canvasWidth, 8);

    ctx!.fillStyle = "#8fd4ff";
    ctx!.font = `bold ${Math.round(fontSize * 0.9)}px monospace`;
    ctx!.textBaseline = "top";
    ctx!.fillText("TARS 2 ▸ ", 12, 16);

    ctx!.fillStyle = stateColor[state];
    ctx!.font = `bold ${Math.round(fontSize * 0.8)}px monospace`;
    const labelX = 12 + ctx!.measureText("TARS 2 ▸ ").width;
    ctx!.fillText(state.toUpperCase(), labelX, 18);
  }

  /** Строка запроса под заголовком; возвращает Y, с которого свободно. */
  function drawQueryLine(data: Tars2PanelData): number {
    const line = fontSize * 1.25;
    let y = 16 + line;
    ctx!.fillStyle = "#e6edf3";
    ctx!.font = `${Math.round(fontSize * 0.8)}px monospace`;
    ctx!.fillText(truncate(data.query ?? "", 78), 12, y);
    y += line * 0.85;
    if (data.note) {
      // Оператор просил «cpu», а ушло `rate(process_cpu_seconds_total[5m])` —
      // об этом надо сказать, иначе он решит, что смотрит на своё выражение.
      ctx!.fillStyle = "#f6c177";
      ctx!.font = `${Math.round(fontSize * 0.65)}px monospace`;
      ctx!.fillText(truncate(data.note, 92), 12, y);
      y += line * 0.7;
    }
    return y + 6;
  }

  // ─────────────────────── график Prometheus ───────────────────────

  function drawChart(series: Tars2Series[], top: number): void {
    const padLeft = 96;
    const padRight = 24;
    const legendH = Math.round(fontSize * 1.5) + 8;
    const plotTop = top;
    const plotBottom = canvasHeight - legendH - 28;
    const plotLeft = padLeft;
    const plotRight = canvasWidth - padRight;

    const flat = series.flatMap((s) => s.points);
    if (flat.length === 0) return;

    let minV = Infinity;
    let maxV = -Infinity;
    let minT = Infinity;
    let maxT = -Infinity;
    for (const [t, v] of flat) {
      if (v < minV) minV = v;
      if (v > maxV) maxV = v;
      if (t < minT) minT = t;
      if (t > maxT) maxT = t;
    }
    // Плоский ряд (например, `up` = 1) без запаса схлопнулся бы в линию по
    // краю области — расширяем диапазон, чтобы он лёг по центру.
    if (!(maxV > minV)) {
      const pad = Math.abs(maxV) > 0 ? Math.abs(maxV) * 0.5 : 1;
      minV -= pad;
      maxV += pad;
    }
    if (!(maxT > minT)) maxT = minT + 1;

    const xOf = (t: number): number =>
      plotLeft + ((t - minT) / (maxT - minT)) * (plotRight - plotLeft);
    const yOf = (v: number): number =>
      plotBottom - ((v - minV) / (maxV - minV)) * (plotBottom - plotTop);

    // Сетка + подписи по оси Y.
    ctx!.strokeStyle = "#1b2129";
    ctx!.lineWidth = 1;
    ctx!.fillStyle = "#8b98a5";
    ctx!.font = `${Math.round(fontSize * 0.6)}px monospace`;
    ctx!.textBaseline = "middle";
    const GRID_ROWS = 4;
    for (let i = 0; i <= GRID_ROWS; i++) {
      const v = minV + ((maxV - minV) * i) / GRID_ROWS;
      const y = yOf(v);
      ctx!.beginPath();
      ctx!.moveTo(plotLeft, y);
      ctx!.lineTo(plotRight, y);
      ctx!.stroke();
      ctx!.fillText(formatValue(v).padStart(9), 8, y);
    }
    ctx!.textBaseline = "top";

    // Ось времени: подписи «-15m» и «now» — абсолютное время в VR читать
    // неудобно, а оператору важна давность.
    ctx!.fillStyle = "#8b98a5";
    ctx!.font = `${Math.round(fontSize * 0.6)}px monospace`;
    const spanMin = Math.max(1, Math.round((maxT - minT) / 60));
    ctx!.fillText(`-${spanMin}m`, plotLeft, plotBottom + 8);
    const nowLabel = "now";
    ctx!.fillText(
      nowLabel,
      plotRight - ctx!.measureText(nowLabel).width,
      plotBottom + 8
    );

    // Линии.
    series.forEach((s, idx) => {
      if (s.points.length === 0) return;
      ctx!.strokeStyle = SERIES_COLORS[idx % SERIES_COLORS.length];
      ctx!.lineWidth = 3;
      ctx!.lineJoin = "round";
      ctx!.beginPath();
      s.points.forEach(([t, v], i) => {
        const x = xOf(t);
        const y = yOf(v);
        if (i === 0) ctx!.moveTo(x, y);
        else ctx!.lineTo(x, y);
      });
      ctx!.stroke();
      // Точка последнего значения — глазу нужна опора на правом краю.
      const last = s.points[s.points.length - 1];
      ctx!.fillStyle = SERIES_COLORS[idx % SERIES_COLORS.length];
      ctx!.beginPath();
      ctx!.arc(xOf(last[0]), yOf(last[1]), 5, 0, Math.PI * 2);
      ctx!.fill();
    });

    drawLegend(series, canvasHeight - legendH - 4);
  }

  function drawLegend(series: Tars2Series[], y: number): void {
    ctx!.font = `${Math.round(fontSize * 0.65)}px monospace`;
    ctx!.textBaseline = "top";
    let x = 12;
    series.forEach((s, idx) => {
      const last = s.points.length ? s.points[s.points.length - 1][1] : NaN;
      const label = `${truncate(s.name, 22)} ${formatValue(last)}`;
      const w = ctx!.measureText(label).width + 26;
      if (x + w > canvasWidth - 12) return;
      ctx!.fillStyle = SERIES_COLORS[idx % SERIES_COLORS.length];
      ctx!.fillRect(x, y + 6, 14, 6);
      ctx!.fillStyle = "#c9d4df";
      ctx!.fillText(label, x + 20, y);
      x += w;
    });
  }

  // ─────────────────────── хвост логов Loki ───────────────────────

  function drawLogs(lines: Tars2LogLine[], top: number): void {
    const rowH = Math.round(fontSize * 0.85);
    ctx!.font = `${Math.round(fontSize * 0.62)}px monospace`;
    ctx!.textBaseline = "top";
    let y = top;
    for (const entry of lines) {
      if (y + rowH > canvasHeight - 12) break;
      ctx!.fillStyle = "#5c6773";
      const stamp = formatClock(entry.ts);
      ctx!.fillText(stamp, 12, y);
      ctx!.fillStyle = "#c9d4df";
      ctx!.fillText(truncate(entry.line.replace(/\s+/g, " "), 108), 96, y);
      y += rowH;
    }
  }

  // ─────────────────────── состояния без данных ───────────────────────

  function drawMessage(
    title: string,
    detail: string[],
    color: string,
    top: number
  ): void {
    ctx!.fillStyle = color;
    ctx!.font = `${fontSize}px monospace`;
    ctx!.textBaseline = "top";
    ctx!.fillText(truncate(title, 62), 12, top);
    ctx!.fillStyle = "#8b98a5";
    ctx!.font = `${Math.round(fontSize * 0.72)}px monospace`;
    detail.forEach((row, i) => {
      ctx!.fillText(truncate(row, 84), 12, top + Math.round(fontSize * 1.5) + i * Math.round(fontSize * 0.95));
    });
  }

  // ─────────────────────── рендер ───────────────────────

  function render(): void {
    drawChrome();

    if (currentData) {
      const top = drawQueryLine(currentData);
      const series = (currentData.series ?? []).filter(
        (s) => s.points && s.points.length > 0
      );
      const lines = currentData.lines ?? [];
      if (currentData.status === "ok" && series.length > 0) {
        drawChart(series, top);
        return;
      }
      if (currentData.status === "ok" && lines.length > 0) {
        drawLogs(lines, top);
        return;
      }
      if (currentData.status === "empty") {
        const available = currentData.available ?? [];
        drawMessage(
          "Данных нет.",
          available.length > 0
            ? ["Есть, например:", ...available.slice(0, 5).map((n) => `  ${n}`)]
            : ["Prometheus ответил пустым результатом."],
          "#f6c177",
          top
        );
        return;
      }
      drawMessage(
        "Метрики не пришли.",
        [currentData.error || currentData.summary || "неизвестная ошибка"],
        "#e64568",
        top
      );
      return;
    }

    const top = 16 + Math.round(fontSize * 1.25);

    if (currentUrl !== null) {
      // Деградированный режим: сервер прислал только ссылку (сборка без
      // /avatar/tars/panel_data). Рисуем её честно — как до #2184.
      ctx!.fillStyle = "#e6edf3";
      ctx!.font = `${Math.round(fontSize * 0.8)}px monospace`;
      ctx!.fillText(parseUrlHost(currentUrl), 12, top);
      ctx!.fillStyle = "#8b98a5";
      ctx!.font = `${Math.round(fontSize * 0.68)}px monospace`;
      ctx!.fillText(truncate(parseUrlPath(currentUrl), 84), 12, top + Math.round(fontSize * 1.1));
      ctx!.fillStyle = "#444a52";
      ctx!.font = `${Math.round(fontSize * 0.6)}px monospace`;
      ctx!.fillText(
        "ссылка без данных · сервер не прислал panel_data",
        12,
        canvasHeight - 28
      );
      return;
    }

    if (state === "error") {
      drawMessage(
        "Нет доступа к метрикам.",
        ["Спроси ТАРС ещё раз."],
        "#e64568",
        top
      );
      return;
    }

    drawMessage(
      "Idle.",
      ['Скажи: "ТАРС, покажи дрейф CPU".'],
      "#8b98a5",
      top
    );
  }

  // ─────────────────────── публичный API ───────────────────────

  function setPanelData(data: Tars2PanelData): void {
    if (data === null || typeof data !== "object") {
      // eslint-disable-next-line no-console
      console.warn("[tars2_metrics_panel] setPanelData: not an object, ignored");
      return;
    }
    currentData = data;
    currentUrl = typeof data.url === "string" && data.url ? data.url : currentUrl;
    state =
      data.status === "ok" ? "ok" : data.status === "empty" ? "empty" : "error";
    render();
  }

  function setPanelUrl(url: string): void {
    if (typeof url !== "string" || url.length === 0) {
      // eslint-disable-next-line no-console
      console.warn("[tars2_metrics_panel] setPanelUrl: empty/non-string url, ignored");
      return;
    }
    if (url === currentUrl && currentData === null) return;
    currentUrl = url;
    // URL сам по себе данных не несёт — сбрасываем их, иначе панель покажет
    // старый график рядом с новой ссылкой и соврёт оператору.
    currentData = null;
    state = "loading";
    render();
    const listener = onUrlChanged;
    if (listener) listener({ kind: "url", url });
  }

  function clear(): void {
    if (currentUrl === null && currentData === null && state === "idle") return;
    currentUrl = null;
    currentData = null;
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

  function getStats(): {
    url: string | null;
    state: Tars2PanelState;
    seriesCount: number;
    pointCount: number;
  } {
    const series = currentData?.series ?? [];
    return {
      url: currentUrl,
      state,
      seriesCount: series.length,
      pointCount: series.reduce((acc, s) => acc + (s.points?.length ?? 0), 0)
    };
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
    get currentData(): Tars2PanelData | null {
      return currentData;
    },
    get state(): Tars2PanelState {
      return state;
    },
    setPanelData,
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

/** Компактная запись значения: «1.23k», «0.0358», «199.27M». */
export function formatValue(v: number): string {
  if (!Number.isFinite(v)) return "—";
  const abs = Math.abs(v);
  if (abs >= 1e9) return `${(v / 1e9).toFixed(2)}G`;
  if (abs >= 1e6) return `${(v / 1e6).toFixed(2)}M`;
  if (abs >= 1e3) return `${(v / 1e3).toFixed(2)}k`;
  if (abs >= 1) return v.toFixed(2);
  if (abs === 0) return "0";
  return v.toFixed(4);
}

/** `HH:MM:SS` из unix-секунд — для хвоста логов. */
function formatClock(tsSeconds: number): string {
  const d = new Date(tsSeconds * 1000);
  const pad = (n: number): string => String(n).padStart(2, "0");
  return `${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}`;
}
