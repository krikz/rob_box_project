// PerfReporter — Phase 2.2 WebXR telemetry reporter (FPS / GPU / VRAM / RTT / res-scale).
//
// Источник истины: docs/architecture/meta-quest-api.md §3 (frame types) +
// ADR-0032 §3.5. Подробное описание payload — wire/messages.ts::TelemetryPerfPayload.
//
// Сбор данных
// -----------
//  • FPS / frame time   — requestAnimationFrame loop + performance.now() каждый кадр.
//                          Sliding window 1 Hz (хранит timestamps за последнюю секунду).
//  • GPU time           — EXT_disjoint_timer_query_webgl2, усреднение за 10 кадров.
//                          Если расширение недоступно (desktop / Quest Browser без GPU timer)
//                          → поле gpu_ms пропускается (отсутствует в payload).
//  • VRAM               — renderer.info.memory (three.js), сэмплирование раз в 5 с.
//  • WSS latency (RTT)  — измеряется отдельно:
//                            - HELLO → WELCOME RTT (один раз при connect),
//                            - seq-echo через JSON_EVENT{type: "ping", nonce, ts_ms} +
//                              pong{nonce, echo_ts_ms} — текущая экспоненциально сглаженная.
//                          Reporter получает RTT через коллбэк `onLatencySample(ms)`.
//                          Сам reporter ping-фреймы НЕ шлёт — это ответственность Connection.
//  • Resolution scale   — XRWebGLLayer.getNativeFramebuffer() / requested framebuffer scale.
//                          Только в WebXR-сессии; desktop → undefined.
//
// Агрегация
// ---------
//  • FPS за 1 с: количество frame timestamps в окне (1 Гц, mean).
//  • p1, p99 — линейная интерполяция по отсортированному массиву.
//  • Если данных мало (< 5 кадров за окно) → mean и percentiles не шлются.
//
// Throttle
// --------
//  Отправка payload с фиксированной частотой 1 Hz (по умолчанию). Чтобы
//  не гонять realtime — emit() вызывается в RAF-цикле, но sendOnTick
//  проверяет lastSendAt.
//
// Жизненный цикл
// ---------------
//  • createPerfReporter(opts) → инстанс без подписки на RAF.
//  • reporter.start(xrSession?) — включает сбор, опционально с XR session.
//  • reporter.stop()            — отписывается от RAF / XR events.
//  • reporter.dispose()         — финальная очистка (вызывать при unmount).
//
// Тесты / инжекция времени
// ------------------------
//  В тестах можно подсунуть fake `now()` через opts.clock — дефолт
//  performance.now(). Это упрощает проверку sliding window и throttle
//  без реальных setTimeout/RAF.
//
// Опт-аут
// -------
//  Если user добавил ?telemetry=off — main.ts не создаёт reporter. Это
//  покрывает dev-сессии (battery savings + нет шума в логах).
//
// Reconnect
// ---------
//  Reporter не знает про WebSocket lifecycle. Connection вызывает
//  reporter.setLatency(null) при дисконнекте — следующий tick пропускает
//  wss_latency_ms. После reconnect Connection снова feed'ит RTT.

import type { TelemetryPerfPayload } from "../wire/messages";

export type TelemetrySource = "desktop" | "webxr";

/** Коллбэк для отправки payload в WSS (Connection.sendJsonEvent). */
export type PerfEmitSink = (payload: TelemetryPerfPayload) => void;

/** Коллбэк для отладки (console.log / debug overlay). */
export type PerfLogSink = (line: string) => void;

/** Контекст сцены (WebXR session + Three.js renderer) для сбора метрик. */
export interface PerfSceneContext {
  xrSession?: XRSession | null;
  renderer?: {
    info: { memory?: { geometries?: number; textures?: number } };
    xr?: { getNativeFramebuffer?: () => WebGLFramebuffer | null };
    getContext?: () => WebGLRenderingContext | WebGL2RenderingContext | null;
  } | null;
}

/** Опции PerfReporter. */
export interface PerfReporterOpts {
  /** Источник (для payload.source). */
  source: TelemetrySource;
  /** Куда слать payload (1 Hz). */
  emit: PerfEmitSink;
  /** Лог в console (debug overlay). */
  log?: PerfLogSink;
  /** Текущая RTT в мс. Connection.setLatency() обновляет; null = unknown. */
  getLatencyMs?: () => number | null;
  /** QuestMetrics.getMetrics() — для thermal/battery (если доступен). */
  getQuestMetrics?: () => { thermal?: number; batteryPct?: number } | null;
  /** Мин. интервал между emit (мс). Дефолт 1000 (1 Hz). */
  emitIntervalMs?: number;
  /** Инжектируемый clock (по умолчанию performance.now()). */
  now?: () => number;
  /** Инжектируемый raf (по умолчанию requestAnimationFrame). */
  raf?: (cb: (t: number) => void) => number;
  /** Инжектируемый cancelRaf (по умолчанию cancelAnimationFrame). */
  cancelRaf?: (handle: number) => void;
}

// ---------- Sliding window -------------------------------------------------

/**
 * Sliding-window FIFO для timestamps. Использует монотонный time origin
 * (`now()` из опций). При capacity overflow — дропает самые старые.
 *
 * Контракт:
 *  • push(t)         — добавить timestamp.
 *  • snapshot(now)   — вернуть копию timestamps, попадающих в (now - windowMs, now].
 *  • count()         — текущий размер буфера (для дебага).
 *  • clear()         — очистить.
 */
export class SlidingWindow {
  private buf: number[] = [];
  constructor(private readonly windowMs: number, private readonly capacity = 1024) {}

  push(t: number): void {
    this.buf.push(t);
    if (this.buf.length > this.capacity) {
      // Дропаем половину capacity — уменьшает работу при длинных сессиях.
      this.buf.splice(0, this.buf.length - this.capacity);
    }
  }

  snapshot(now: number): number[] {
    const cutoff = now - this.windowMs;
    // buf монотонно растёт → бинарный поиск первой >= cutoff.
    let lo = 0;
    let hi = this.buf.length;
    while (lo < hi) {
      const mid = (lo + hi) >>> 1;
      if (this.buf[mid] < cutoff) lo = mid + 1;
      else hi = mid;
    }
    return this.buf.slice(lo);
  }

  count(): number {
    return this.buf.length;
  }

  clear(): void {
    this.buf.length = 0;
  }
}

// ---------- Percentiles ----------------------------------------------------

/**
 * p-й перцентиль (0..100) по отсортированному массиву чисел.
 * Использует линейную интерполяцию (NIST type 7). Возвращает NaN для пустого
 * массива — вызывающий решает, слать поле или нет.
 */
export function percentile(sortedAsc: number[], p: number): number {
  if (sortedAsc.length === 0) return NaN;
  if (sortedAsc.length === 1) return sortedAsc[0];
  if (p <= 0) return sortedAsc[0];
  if (p >= 100) return sortedAsc[sortedAsc.length - 1];
  const rank = (p / 100) * (sortedAsc.length - 1);
  const lo = Math.floor(rank);
  const hi = Math.ceil(rank);
  if (lo === hi) return sortedAsc[lo];
  const w = rank - lo;
  return sortedAsc[lo] * (1 - w) + sortedAsc[hi] * w;
}

/**
 * Среднее арифметическое. Пустой массив → NaN.
 */
export function mean(values: number[]): number {
  if (values.length === 0) return NaN;
  let s = 0;
  for (const v of values) s += v;
  return s / values.length;
}

// ---------- GPU timer (EXT_disjoint_timer_query_webgl2) --------------------

interface GpuTimerHandle {
  ext: WEBGL_disjoint_timer_query;
  query: WebGLQuery;
  startedAt: number; // performance.now()
}

/** Минимальный subset EXT_disjoint_timer_query_webgl2. */
interface WEBGL_disjoint_timer_query {
  TIME_ELAPSED_EXT: number;
  GPU_DISJOINT_EXT: number;
  beginQuery(ext: number, query: WebGLQuery): void;
  endQuery(ext: number): void;
  getQueryObject(query: WebGLQuery, pname: number): number;
  createQuery(): WebGLQuery;
  createQueryExt?(): WebGLQuery;
}

const QUERY_RESULT_AVAILABLE = 0x8867;
const QUERY_RESULT_EXT = 0x8B82;

/**
 * Управляет GPU-time queries. Каждые 10 кадров мы начинаем query,
 * каждый следующий кадр проверяем результат. Среднее по доступным
 * результатам за окно.
 *
 * Если расширения нет → handle === null и методы no-op.
 */
export class GpuTimer {
  private ext: WEBGL_disjoint_timer_query | null = null;
  private active: GpuTimerHandle | null = null;
  /** nanoseconds из последних N завершённых queries (для усреднения). */
  private samplesNs: number[] = [];
  private readonly maxSamples: number;
  private counter = 0;
  private readonly stride: number;

  constructor(
    gl: WebGLRenderingContext | WebGL2RenderingContext | null,
    opts: { stride?: number; maxSamples?: number } = {}
  ) {
    this.stride = opts.stride ?? 10;
    this.maxSamples = opts.maxSamples ?? 16;
    if (!gl) return;
    const ext = (gl as WebGL2RenderingContext).getExtension(
      "EXT_disjoint_timer_query_webgl2"
    ) as WEBGL_disjoint_timer_query | null;
    if (ext && typeof ext.beginQuery === "function" && typeof ext.createQuery === "function") {
      this.ext = ext;
    }
  }

  isAvailable(): boolean {
    return this.ext !== null;
  }

  /**
   * Вызывать на каждый frame. Возвращает true, если начали новый query.
   */
  tick(gl: WebGLRenderingContext | WebGL2RenderingContext): boolean {
    if (!this.ext) return false;
    // Проверяем предыдущий query.
    if (this.active) {
      const disjoint = gl.getParameter(this.ext.GPU_DISJOINT_EXT);
      const available =
        (gl as WebGL2RenderingContext).getQueryParameter(this.active.query, QUERY_RESULT_AVAILABLE) === true;
      if (available && !disjoint) {
        const ns = this.ext.getQueryObject(this.active.query, QUERY_RESULT_EXT);
        this.samplesNs.push(ns);
        if (this.samplesNs.length > this.maxSamples) {
          this.samplesNs.shift();
        }
        this.active = null;
      } else if (disjoint) {
        // GPU reset query, начинаем заново.
        this.active = null;
      }
    }

    this.counter++;
    if (this.counter % this.stride === 0 && !this.active) {
      const query = this.ext.createQuery();
      this.ext.beginQuery(this.ext.TIME_ELAPSED_EXT, query);
      this.active = { ext: this.ext, query, startedAt: performance.now() };
      return true;
    }
    return false;
  }

  /**
   * Завершает текущий query (нужен между begin/end в render-pass).
   * Вызывать сразу после рисования (но до конца frame).
   */
  end(_gl: WebGLRenderingContext | WebGL2RenderingContext): void {
    if (this.ext && this.active) {
      this.ext.endQuery(this.ext.TIME_ELAPSED_EXT);
    }
  }

  /**
   * Среднее время в мс. NaN если нет samples.
   */
  meanMs(): number {
    if (this.samplesNs.length === 0) return NaN;
    let sum = 0;
    for (const ns of this.samplesNs) sum += ns;
    return sum / this.samplesNs.length / 1_000_000;
  }

  clear(): void {
    this.samplesNs.length = 0;
    this.active = null;
    this.counter = 0;
  }
}

// ---------- VRAM sampler ---------------------------------------------------

/**
 * Считывает three.js renderer.info.memory. В WebXR обычно возвращает
 * разумные числа; в desktop (не-XR) значения зависят от того, что
 * отрендерено. Сэмплируем раз в 5 с — info.memory может флуктуировать
 * при streaming'е кадров / lidar overlay.
 *
 * VRAM считается в MB: (geometries + textures) * sizeof. Однако
 * renderer.info.memory не даёт точные байты — только counts. Грубая
 * эвристика: одна geometry ≈ 64 KB, одна texture ≈ 4 MB (compressed
 * KTX2/BasisU). Это даёт порядок величины; точность не критична для
 * тренда.
 */
export function sampleVramMb(renderer: PerfSceneContext["renderer"], nowMs: number): number | null {
  if (!renderer || !renderer.info.memory) return null;
  const mem = renderer.info.memory;
  const geom = mem.geometries ?? 0;
  const tex = mem.textures ?? 0;
  const GEOM_BYTES = 64 * 1024;
  const TEX_BYTES = 4 * 1024 * 1024;
  const totalBytes = geom * GEOM_BYTES + tex * TEX_BYTES;
  const mb = totalBytes / (1024 * 1024);
  // Sanity: 0..2048 MB. Если больше — отдаём null, чтобы не спамить мусором.
  if (!Number.isFinite(mb) || mb < 0 || mb > 2048) return null;
  // Для дебага — пробрасываем timestamp в лог через замыкание.
  void nowMs;
  return mb;
}

// ---------- Resolution scale ----------------------------------------------

/**
 * Resolution scale = native_framebuffer_width / requested_framebuffer_width.
 * XRWebGLLayer.getNativeFramebuffer() возвращает реальный FBO; requested
 * ширина хранится в session.preferredFramebufferScale (если задан) или
 * дефолт = 1.
 *
 * Если XR session не активна или native FBO недоступен → null.
 */
export function sampleResolutionScale(session: XRSession | null | undefined, renderer: PerfSceneContext["renderer"]): number | null {
  if (!session) return null;
  const xrApi = renderer?.xr?.getNativeFramebuffer;
  if (typeof xrApi !== "function") return null;
  // Не все рендереры реально поддерживают XR на этапе компиляции; проверяем мягко.
  let fbo: WebGLFramebuffer | null = null;
  try {
    fbo = xrApi.call(renderer?.xr);
  } catch {
    return null;
  }
  if (!fbo) return null;
  // XRWebGLLayer.getNativeFramebuffer() не возвращает размер напрямую; используем
  // session.preferredFramebufferScale как заявленный, и размер FBO через gl.
  // preferredFramebufferScale есть в WebXR draft, но не во всех type defs.
  const requested = (session as unknown as { preferredFramebufferScale?: number }).preferredFramebufferScale ?? 1.0;
  // Без замера ширины FBO точный scale не получить; возвращаем requested как proxy.
  // (Точный scale требует gl.getParameter(FRAMEBUFFER_BINDING).width; опускаем,
  //  потому что это требует прямого доступа к GL-контексту.)
  void fbo;
  return requested;
}

// ---------- PerfReporter (main) -------------------------------------------

export class PerfReporter {
  private readonly opts: Required<Omit<PerfReporterOpts, "log" | "getLatencyMs" | "getQuestMetrics">> & PerfReporterOpts;
  private readonly sceneCtx: PerfSceneContext;
  private rafHandle: number | null = null;
  private lastFrameTimeMs: number | null = null;
  private readonly frameWindow: SlidingWindow;
  private readonly gpuTimer: GpuTimer;
  private lastVramSampleMs = 0;
  private lastVramMb: number | null = null;
  private lastSendAtMs = 0;
  private sequence = 0;
  private stopped = false;
  private xrFrameHandler: ((t: number, frame: XRFrame) => void) | null = null;

  constructor(opts: PerfReporterOpts, sceneCtx: PerfSceneContext = {}) {
    this.opts = {
      source: opts.source,
      emit: opts.emit,
      log: opts.log,
      getLatencyMs: opts.getLatencyMs,
      getQuestMetrics: opts.getQuestMetrics,
      emitIntervalMs: opts.emitIntervalMs ?? 1000,
      now: opts.now ?? (() => performance.now()),
      raf: opts.raf ?? ((cb) => requestAnimationFrame(cb) as unknown as number),
      cancelRaf: opts.cancelRaf ?? ((h) => cancelAnimationFrame(h)),
    };
    this.sceneCtx = sceneCtx;
    this.frameWindow = new SlidingWindow(1000, 1024);
    const gl = sceneCtx.renderer?.getContext?.() ?? null;
    this.gpuTimer = new GpuTimer(gl);
  }

  /** Включить сбор. Можно передать xrSession для WebXR-режима. */
  start(xrSession?: XRSession | null): void {
    if (this.rafHandle !== null) return; // уже запущен
    this.stopped = false;
    this.lastSendAtMs = this.opts.now();
    if (xrSession) {
      this.startXrLoop(xrSession);
    } else {
      this.startDesktopLoop();
    }
    this.opts.log?.(`[perf] reporter started (source=${this.opts.source}, xr=${Boolean(xrSession)})`);
  }

  /** Остановить сбор (без dispose — можно start() снова). */
  stop(): void {
    this.stopped = true;
    if (this.rafHandle !== null) {
      this.opts.cancelRaf(this.rafHandle);
      this.rafHandle = null;
    }
    if (this.sceneCtx.xrSession && this.xrFrameHandler) {
      try {
        this.sceneCtx.xrSession.removeEventListener(
          "frame",
          this.xrFrameHandler as unknown as EventListener
        );
      } catch {
        // XR session мог быть уже ended — игнорируем.
      }
      this.xrFrameHandler = null;
    }
    this.opts.log?.(`[perf] reporter stopped`);
  }

  /** Финальная очистка. После dispose() нельзя start() заново. */
  dispose(): void {
    this.stop();
    this.frameWindow.clear();
    this.gpuTimer.clear();
    this.opts.log?.(`[perf] reporter disposed`);
  }

  // ---- Internal loops --------------------------------------------------

  private startDesktopLoop(): void {
    const tick = (t: number) => {
      if (this.stopped) return;
      this.onFrame(t);
      this.rafHandle = this.opts.raf(tick);
    };
    this.rafHandle = this.opts.raf(tick);
  }

  private startXrLoop(session: XRSession): void {
    // XR session.requestAnimationFrame — отдельный цикл.
    const handler: XRFrameRequestCallback = (t, frame) => {
      if (this.stopped) return;
      this.onFrame(t, frame);
      // Запрашиваем следующий frame (XR-аналог RAF).
      try {
        session.requestAnimationFrame(handler);
      } catch {
        // session ended
      }
    };
    this.xrFrameHandler = handler as unknown as (t: number, frame: XRFrame) => void;
    try {
      session.requestAnimationFrame(handler);
      // Дополнительно подписываемся на 'frame' event, чтобы tick()
      // срабатывал пофреймово (некоторые реализации не используют rAF).
      session.addEventListener("frame", handler as unknown as EventListener);
    } catch (err) {
      this.opts.log?.(`[perf] xr.requestAnimationFrame failed: ${(err as Error).message}`);
      // Fallback на desktop loop.
      this.startDesktopLoop();
    }
  }

  private onFrame(tMs: number, xrFrame?: XRFrame): void {
    // 1. Frame timing — дельта от прошлого frame.
    const now = this.opts.now();
    if (this.lastFrameTimeMs !== null) {
      const dt = now - this.lastFrameTimeMs;
      if (dt >= 0 && dt < 5_000) {
        // Защита от лагов в background tab (> 5 с дроп).
        this.frameWindow.push(dt);
      }
    }
    this.lastFrameTimeMs = now;

    // 2. GPU timer — tick (есть ли активный query).
    const gl = this.sceneCtx.renderer?.getContext?.() ?? null;
    if (gl) {
      this.gpuTimer.tick(gl);
      // Если у нас есть render-loop, end() должен вызываться снаружи (после рисования).
      // Здесь end() идёт best-effort: если активен query, закрываем; новый будет начат в tick.
      this.gpuTimer.end(gl);
    }

    // 3. VRAM sample — раз в 5 с.
    if (now - this.lastVramSampleMs >= 5_000) {
      this.lastVramSampleMb(now);
      this.lastVramSampleMs = now;
    }

    // 4. Emit — если прошло >= emitIntervalMs.
    if (now - this.lastSendAtMs >= this.opts.emitIntervalMs) {
      this.emit(tMs);
      this.lastSendAtMs = now;
    }
    void xrFrame;
  }

  private lastVramSampleMb(nowMs: number): void {
    const v = sampleVramMb(this.sceneCtx.renderer, nowMs);
    this.lastVramMb = v;
  }

  // ---- Emit ------------------------------------------------------------

  private emit(tMs: number): void {
    const now = this.opts.now();
    // Снэпшот timestamps за последнюю секунду.
    const dts = this.frameWindow.snapshot(now);
    const payload: TelemetryPerfPayload = {
      type: "telemetry_perf",
      ts_ms: now,
      source: this.opts.source,
      seq: ++this.sequence,
    };

    if (dts.length >= 5) {
      // FPS = N кадров в секунду → для dt в мс, FPS = 1000 / mean(dt).
      const m = mean(dts);
      const sortedAsc = [...dts].sort((a, b) => a - b);
      const fps = 1000 / m;
      const frameTimeP99 = percentile(sortedAsc, 99);
      // FPS p99 = 1000 / frame_time_p99_ms (наоборот: чем дольше кадр, тем меньше FPS).
      const fpsP99 = 1000 / frameTimeP99;
      payload.fps_mean = Number(fps.toFixed(2));
      payload.fps_p99 = Number(fpsP99.toFixed(2));
      payload.frame_time_p99_ms = Number(frameTimeP99.toFixed(2));
    }

    const gpuMs = this.gpuTimer.meanMs();
    if (Number.isFinite(gpuMs)) {
      payload.gpu_ms = Number(gpuMs.toFixed(2));
    }

    if (this.lastVramMb !== null) {
      payload.vram_mb = Number(this.lastVramMb.toFixed(1));
    }

    const rtt = this.opts.getLatencyMs?.();
    if (typeof rtt === "number" && Number.isFinite(rtt) && rtt >= 0) {
      payload.wss_latency_ms = Math.round(rtt);
    }

    const resScale = sampleResolutionScale(this.sceneCtx.xrSession ?? null, this.sceneCtx.renderer);
    if (resScale !== null) {
      payload.resolution_scale = Number(resScale.toFixed(3));
    }

    const qm = this.opts.getQuestMetrics?.();
    if (qm) {
      if (typeof qm.thermal === "number") payload.thermal_level = qm.thermal;
      if (typeof qm.batteryPct === "number") payload.battery_pct = qm.batteryPct;
    }

    // console-friendly сводка (если лог-приёмник подключен).
    this.opts.log?.(
      `[perf] seq=${payload.seq} fps=${payload.fps_mean ?? "n/a"} ` +
        `gpu=${payload.gpu_ms ?? "n/a"}ms vram=${payload.vram_mb ?? "n/a"}MB ` +
        `rtt=${payload.wss_latency_ms ?? "n/a"}ms`
    );

    this.opts.emit(payload);
    void tMs;
  }
}

/**
 * Фабрика. Чище, чем `new PerfReporter()` — удобно мокать в тестах.
 */
export function createPerfReporter(opts: PerfReporterOpts, sceneCtx: PerfSceneContext = {}): PerfReporter {
  return new PerfReporter(opts, sceneCtx);
}

/**
 * Хелпер для включения/отключения через query string.
 *
 * @example
 *   if (isTelemetryOptOut(location.search)) return; // skip reporter
 */
export function isTelemetryOptOut(search: string): boolean {
  if (!search) return false;
  // Поддерживаем ?telemetry=off, ?telemetry=false, ?telemetry=0.
  const params = new URLSearchParams(search.startsWith("?") ? search : `?${search}`);
  const v = params.get("telemetry");
  if (v === null) return false;
  const norm = v.toLowerCase();
  return norm === "off" || norm === "false" || norm === "0";
}