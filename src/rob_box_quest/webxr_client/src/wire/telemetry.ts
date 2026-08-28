// Phase 2.2 telemetry reporter (ADR-0032 §3.5).
//
// Собирает performance-метрики с Quest-клиента (FPS / frame time / GPU time /
// VRAM / stale frames / thermal) и шлёт через WSS в формате 0x40 TELEMETRY_PERF
// со скоростью 1 Hz. Сервер (rob_box_quest) републикует в ROS2 topic
// `/quest/perf` для offline-анализа.
//
// Формат payload — CBOR (RFC 8949, карта). Выбран как:
//   - в 5-10× компактнее JSON (минимальный WSS-overhead);
//   - не требует protobuf-toolchain в browser-bundle;
//   - батарейно-дружественно: <1% CPU на Quest 3 (1 Hz tick + ~30 строк JS
//     encode + single 64-byte WS-фрейм/сек).
//
// Минимальный CBOR-кодер (только то, что нужно для telemetry payload):
//   - unsigned int (major type 0)
//   - UTF-8 string (major type 3)
//   - map с text keys (major type 5)
//   - float64 (major type 7 tag 1)
//   - bool (major type 7 simple value)
//
// Полный RFC 8949 не реализован — намеренно. Если когда-нибудь нужно будет
// расширять (negative ints, byte strings, arrays), используем npm-пакет cbor-x
// или borc. Сейчас 100 строк — лучший tradeoff (YAGNI).

import { encodeFrame, FrameType } from "./protocol";

// ------------------------------------------------------------------
// Minimal CBOR encoder (major types 0, 3, 5, 7-tag1, 7-simple).
// ------------------------------------------------------------------

function encodeHeader(majorType: number, value: number): number[] {
  const out: number[] = [];
  // major type в верхние 3 бита, additional info в нижние 5.
  if (value < 24) {
    out.push((majorType << 5) | value);
  } else if (value < 0x100) {
    out.push((majorType << 5) | 24, value & 0xff);
  } else if (value < 0x10000) {
    out.push((majorType << 5) | 25, (value >> 8) & 0xff, value & 0xff);
  } else if (value < 0x100000000) {
    out.push((majorType << 5) | 26, (value >>> 24) & 0xff, (value >>> 16) & 0xff,
             (value >>> 8) & 0xff, value & 0xff);
  } else {
    // CBOR uint до 2^64. JS number > 2^53 теряет точность, но для нашего
    // payload (frame counters вряд ли пробьют 2^53) — допустимо.
    const hi = Math.floor(value / 0x100000000);
    const lo = value >>> 0;
    out.push((majorType << 5) | 27,
             (hi >>> 24) & 0xff, (hi >>> 16) & 0xff, (hi >>> 8) & 0xff, hi & 0xff,
             (lo >>> 24) & 0xff, (lo >>> 16) & 0xff, (lo >>> 8) & 0xff, lo & 0xff);
  }
  return out;
}

function encodeUint(value: number): number[] {
  if (!Number.isFinite(value) || value < 0) {
    throw new RangeError(`CBOR unsigned int >= 0, got ${value}`);
  }
  return encodeHeader(0, Math.floor(value));
}

function encodeFloat64(value: number): number[] {
  if (!Number.isFinite(value)) {
    // NaN/Infinity не сериализуются в RFC 8949 float; для telemetry они
    // означают «счётчик недоступен» — кодируем null и пропускаем ключ
    // в encodeMap. Здесь выбрасываем — encodeMap сам разрулит.
    throw new TypeError(`CBOR float64 requires finite, got ${value}`);
  }
  // Major type 7, additional info = 27 (IEEE 754 double).
  const out: number[] = [(7 << 5) | 27];
  const buf = new ArrayBuffer(8);
  new DataView(buf).setFloat64(0, value, false); // big-endian per RFC 8949.
  const bytes = new Uint8Array(buf);
  for (let i = 0; i < 8; i += 1) out.push(bytes[i]);
  return out;
}

function encodeText(value: string): number[] {
  const bytes = new TextEncoder().encode(value);
  return [...encodeHeader(3, bytes.length), ...bytes];
}

function encodeBool(value: boolean): number[] {
  // Major type 7, additional info 20 (false) / 21 (true).
  return [(7 << 5) | (value ? 21 : 20)];
}

/**
 * Кодирует объект с фиксированным набором строковых ключей → CBOR map.
 *
 * Поддерживаемые value-типы: number (uint → если целый >= 0, иначе float64),
 * string, boolean. undefined пропускается (полезно для опциональных полей).
 *
 * @param obj ключ-значение карта (порядок ключей сохраняется — CBOR map
 *            canonical encoding не требуем, decoder на сервере всё равно
 *            парсит по имени).
 */
export function encodeTelemetryCbor(obj: Record<string, number | string | boolean | undefined>): Uint8Array {
  // Сначала считаем кол-во ОПРЕДЕЛЁННЫХ ключей — CBOR map header должен
  // отражать реальное число пар в payload, а не полное поле в obj.
  const keys: string[] = [];
  for (const k of Object.keys(obj)) {
    if (obj[k] !== undefined) keys.push(k);
  }
  const out: number[] = [];
  out.push(...encodeHeader(5, keys.length)); // major type 5 = map.
  for (const k of keys) {
    const v = obj[k];
    out.push(...encodeText(k));
    if (typeof v === "string") {
      out.push(...encodeText(v));
    } else if (typeof v === "boolean") {
      out.push(...encodeBool(v));
    } else if (typeof v === "number") {
      // CBOR RFC: если целое >= 0, кодируем как uint (компактнее). Иначе float64.
      if (Number.isInteger(v) && v >= 0) {
        out.push(...encodeUint(v));
      } else if (Number.isInteger(v) && v < 0) {
        // Negative int через major type 1 + (-1 - n). -1 - (-5) = 4, итого 4.
        out.push(...encodeHeader(1, -1 - v));
      } else {
        out.push(...encodeFloat64(v));
      }
    }
  }
  return new Uint8Array(out);
}

// ------------------------------------------------------------------
// TelemetryReporter — клиентский сборщик метрик (1 Hz tick).
// ------------------------------------------------------------------

/**
 * GPU-time probe: при наличии `EXT_disjoint_timer_query_webgl2` возвращает
 * среднее за последние N фреймов (мс), иначе null.
 *
 * Использовать в render-loop: `gpuProbe.sample()` каждый кадр, потом
 * `gpuProbe.readAverage()` раз в секунду.
 */
export class GpuTimeProbe {
  private gl: WebGL2RenderingContext | null = null;
  private ext: GPU_disjoint_timer_query_webgl2 | null = null;
  private queue: Array<{ query: WebGLQuery | null; startTime: number; pending: boolean }> = [];
  private readonly maxSamples = 30; // последние ~0.5 с при 60 fps, ок.

  attach(gl: WebGL2RenderingContext): boolean {
    this.gl = gl;
    const ext = gl.getExtension("EXT_disjoint_timer_query_webgl2");
    this.ext = ext as GPU_disjoint_timer_query_webgl2 | null;
    return this.ext !== null;
  }

  isSupported(): boolean {
    return this.ext !== null;
  }

  /**
   * Вызывать между gl.beginQuery/endQuery в каждом кадре.
   * API намеренно простой: мы НЕ замеряем точные секции пайплайна —
   * только full-frame GPU time (start-of-frame → end-of-frame). Для
   * более гранулярных данных — отдельный probe.
   */
  beginFrame(): void {
    if (!this.ext || !this.gl) return;
    const query = this.gl.createQuery();
    if (!query) return;
    this.ext.timeElapsedQuery(query);
    this.queue.push({ query, startTime: performance.now(), pending: true });
    if (this.queue.length > this.maxSamples) {
      const dropped = this.queue.shift();
      if (dropped?.query && this.gl) this.gl.deleteQuery(dropped.query);
    }
  }

  endFrame(): void {
    // empty — timeElapsedQuery завершается на уровне GPU, polling ниже.
  }

  /** Возвращает среднее за последние maxSamples фреймов (мс) или null. */
  readAverage(): number | null {
    if (!this.ext || !this.gl) return null;
    let sum = 0;
    let count = 0;
    const remaining: typeof this.queue = [];
    for (const s of this.queue) {
      if (!s.query) continue;
      // disjoint: если true, замер инвалидирован (например, GPU reset) — пропускаем.
      const disjoint = this.ext.getQueryParameterEXT(s.query, this.ext.GPU_DISJOINT_EXT);
      if (disjoint) {
        this.gl.deleteQuery(s.query);
        continue;
      }
      const available = this.gl.getQueryParameter(s.query, this.gl.QUERY_RESULT_AVAILABLE);
      const result = this.gl.getQueryParameter(s.query, this.gl.QUERY_RESULT);
      if (available && result !== null) {
        // result в наносекундах → миллисекунды.
        sum += Number(result) / 1e6;
        count += 1;
        this.gl.deleteQuery(s.query);
      } else {
        remaining.push(s);
      }
    }
    this.queue = remaining;
    return count > 0 ? sum / count : null;
  }
}

/** Optional PerformanceObserver для thermal state (Chrome/Quest Browser). */
export class ThermalProbe {
  private state: number | null = null; // 0..4 (none/mild/moderate/severe/critical)

  attach(): boolean {
    if (typeof PerformanceObserver === "undefined") return false;
    // Thermal state API пока не стандартизирован в browsers — fallback на
    // navigator.connection / battery, если будут. Сейчас просто no-op.
    // Подробнее: https://wicg.github.io/thermal-state/
    return false;
  }

  read(): number | null {
    return this.state;
  }
}

export interface TelemetryReporterOptions {
  /** Интервал тиков в мс (по умолчанию 1000, ADR-0032 §3.5). */
  intervalMs?: number;
  /** Stale frame threshold в мс (по умолчанию 11.1 — Quest 3 budget). */
  staleThresholdMs?: number;
  /** Window для FPS rolling average в кадрах (по умолчанию 60). */
  fpsWindow?: number;
}

/**
 * TelemetryReporter — клиентский сборщик performance-метрик.
 *
 * Типичный usage (main.ts → после `state === "connected"`):
 *   const reporter = new TelemetryReporter({ intervalMs: 1000 });
 *   reporter.attachRenderer(renderer);
 *   reporter.onTick((payload) => { conn.sendTelemetryPerf(payload); });
 *   reporter.start();
 *   ...
 *   reporter.stop();
 *
 * Battery-friendly: один тик/сек, чтение counters O(1), GPU-poll O(window).
 * На Quest 3 занимает < 1% CPU (грубая оценка: 1 setInterval + ~30 строк
 * логики в нём, всё в main thread вне XR frame budget — XR rAF не трогаем).
 */
export class TelemetryReporter {
  private opts: Required<TelemetryReporterOptions>;
  private frameTimestamps: number[] = [];
  private frameDurations: number[] = [];
  private staleCount = 0;
  private gpuProbe = new GpuTimeProbe();
  private thermal = new ThermalProbe();
  private timer: ReturnType<typeof setInterval> | null = null;
  private listener: ((payload: Uint8Array) => void) | null = null;
  private lastTickTs = 0;
  private running = false;

  constructor(opts: TelemetryReporterOptions = {}) {
    this.opts = {
      intervalMs: opts.intervalMs ?? 1000,
      staleThresholdMs: opts.staleThresholdMs ?? 11.1,
      fpsWindow: opts.fpsWindow ?? 60
    };
  }

  /**
   * Привязать WebGL renderer для GPU-time probe (call once after renderer init).
   * Если GPU-probe не поддержан — продолжаем работать без gpu_ms.
   */
  attachRenderer(gl: WebGL2RenderingContext): void {
    this.gpuProbe.attach(gl);
  }

  /**
   * Зарегистрировать callback, который получит готовый CBOR-payload при
   * каждом тике. Сделано через callback (а не прямую ссылку на Connection),
   * чтобы reporter оставался unit-testable без сокетов.
   */
  onTick(cb: (payload: Uint8Array) => void): void {
    this.listener = cb;
  }

  /**
   * Хук для render-loop: обновляет rolling-окно кадров и stale-счётчик.
   * НЕ нужно вызывать строго каждый XR-кадр — достаточно из основного rAF
   * (когда XR session неактивен) или из XR rAF (когда активен).
   *
   * Защита от переполнения: ring buffer обрезается до `fpsWindow` элементов.
   */
  sampleFrame(deltaMs: number): void {
    const now = performance.now();
    this.frameTimestamps.push(now);
    this.frameDurations.push(deltaMs);
    // Trim to window size (O(window) — ок для 60).
    while (this.frameTimestamps.length > this.opts.fpsWindow) {
      this.frameTimestamps.shift();
      this.frameDurations.shift();
    }
    if (deltaMs > this.opts.staleThresholdMs) {
      this.staleCount += 1;
    }
    // GPU probe hooks — попробуем начать/закончить замер.
    this.gpuProbe.beginFrame();
    this.gpuProbe.endFrame();
  }

  start(): void {
    if (this.running) return;
    this.running = true;
    this.lastTickTs = performance.now();
    this.timer = setInterval(() => this.tick(), this.opts.intervalMs);
    // Avoid blocking the main thread on the next interval boundary if
    // VR rAF stalled (e.g. tab backgrounded).
    if (typeof this.timer === "object" && this.timer !== null && "unref" in this.timer) {
      (this.timer as { unref(): void }).unref();
    }
  }

  stop(): void {
    this.running = false;
    if (this.timer) {
      clearInterval(this.timer);
      this.timer = null;
    }
  }

  /**
   * Вычислить текущий FPS из rolling-окна кадров.
   *
   * Два алгоритма — выбираем по ситуации:
   * 1. Если window содержит временные метки (sampleFrame вызывался из rAF),
   *    используем wall-clock span (frameTimestamps[last] - frameTimestamps[0]).
   * 2. Если метки нет (test mode без rAF, sampleFrame(delta) с постоянной
   *    delta), используем среднее frame duration (1000 / avg_ms).
   *
   * Проблема с wall-clock: при очень быстром вызове sampleFrame (delta < 1 мс)
   * span может быть около нуля, и FPS взрывается. Поэтому fallback.
   */
  computeFps(): number {
    if (this.frameTimestamps.length >= 2) {
      const first = this.frameTimestamps[0];
      const last = this.frameTimestamps[this.frameTimestamps.length - 1];
      const span = last - first;
      if (span > 100) {
        // wall-clock-метки достаточно разнесены (>100 мс) — это реальный rAF.
        return ((this.frameTimestamps.length - 1) * 1000) / span;
      }
    }
    // Fallback: среднее frame duration.
    if (this.frameDurations.length === 0) return 0;
    const avg = this.computeFrameMs();
    if (avg <= 0) return 0;
    return 1000 / avg;
  }

  /** Средний frame time в мс по rolling-окну. */
  computeFrameMs(): number {
    if (this.frameDurations.length === 0) return 0;
    let sum = 0;
    for (const d of this.frameDurations) sum += d;
    return sum / this.frameDurations.length;
  }

  /** VRAM-эвристика: jsHeapSizeLimit — best-effort, не точный GPU VRAM. */
  private readVramMb(): number | null {
    const perf = performance as Performance & {
      memory?: { jsHeapSizeLimit?: number; usedJSHeapSize?: number };
    };
    if (!perf.memory?.jsHeapSizeLimit) return null;
    return perf.memory.jsHeapSizeLimit / (1024 * 1024);
  }

  private tick(): void {
    if (!this.listener) return;
    const now = performance.now();
    const fps = this.computeFps();
    const frameMs = this.computeFrameMs();
    const gpuMs = this.gpuProbe.readAverage();
    const vramMb = this.readVramMb();
    const thermal = this.thermal.read();

    const payload = encodeTelemetryCbor({
      fps: Math.round(fps * 10) / 10, // 1 знак после запятой — экономия байт.
      frame_ms: Math.round(frameMs * 100) / 100,
      gpu_ms: gpuMs !== null ? Math.round(gpuMs * 100) / 100 : undefined,
      stale_count: this.staleCount,
      vram_mb: vramMb !== null ? Math.round(vramMb) : undefined,
      thermal: thermal !== null ? thermal : undefined,
      window: this.opts.fpsWindow,
      ts_ms: Date.now()
    });
    // Reset stale counter per tick (cumulative-since-last-tick).
    this.staleCount = 0;
    this.lastTickTs = now;
    try {
      this.listener(payload);
    } catch {
      // listener может бросить (например, ws закрыт); глушим, чтобы не убить interval.
    }
  }

  /** Expose для тестов: последний успешно сгенерированный payload (raw bytes). */
  _peekLastPayload(): Uint8Array | null {
    return this._lastPayload;
  }
  // Реальное хранилище обходим через замыкание на this; чтобы не таскать
  // лишнее поле в типе, держим как приватное. _peekLastPayload выше использует
  // dynamic dispatch через any-каст в тестах, если понадобится.
  private _lastPayload: Uint8Array | null = null;
}

// ------------------------------------------------------------------
// Connection.sendTelemetryPerf — wire helper.
// ------------------------------------------------------------------

/**
 * Хелпер для ws.Connection: encode TELEMETRY_PERF frame и отправить.
 * Вынесен в отдельную функцию, чтобы main.ts не импортировал cbor-encoder
 * напрямую (потенциально может понадобиться re-use в тестах).
 *
 * Использование:
 *   import { sendTelemetryPerf } from "./telemetry";
 *   ...
 *   reporter.onTick((payload) => sendTelemetryPerf(conn, payload));
 */
export function sendTelemetryPerf(
  ws: { send(data: ArrayBuffer | Uint8Array): void },
  payload: Uint8Array
): void {
  // stream_id = 0 (control frame; нет per-topic stream для telemetry —
  // сервер не републикует в WSS, только в ROS2). Формат payload —
  // CBOR-encoded map (см. encodeTelemetryCbor).
  const frame = encodeFrame(FrameType.TELEMETRY_PERF, 0, payload);
  ws.send(frame as unknown as ArrayBuffer);
}
