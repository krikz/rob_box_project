// FPS-счётчик: скользящее среднее по последним N кадрам.
//
// Чистая логика без Three.js и DOM. `frameTimes` — массив performance.now()
// (или любой монотонной шкалы), в секундах или миллисекундах — единицы
// те же, что и в ответе.
//
// Использование в сцене:
//   const fps = createFpsMeter({ windowSize: 60 });
//   loop() {
//     fps.push(performance.now());
//     if (fps.shouldUpdate(500)) { hud.setFps(fps.value()); fps.markUpdated(); }
//   }
//
// Тесты: tests/fps_meter.test.ts — равномерный поток, дропнутые кадры,
// пустой массив, еденица-измерения.

export interface FpsMeterOptions {
  /** Сколько последних кадров держать в окне. Default 60. */
  windowSize?: number;
}

export class FpsMeter {
  private readonly windowSize: number;
  private times: number[] = [];
  private lastUpdateMs = 0;

  constructor(opts: FpsMeterOptions = {}) {
    this.windowSize = Math.max(2, opts.windowSize ?? 60);
  }

  /** Добавить момент времени кадра. */
  push(nowMs: number): void {
    if (!Number.isFinite(nowMs)) return;
    this.times.push(nowMs);
    while (this.times.length > this.windowSize) this.times.shift();
  }

  /** Текущий FPS. 0 — данных недостаточно. */
  value(): number {
    return fpsFromFrameTimes(this.times, this.windowSize);
  }

  /**
   * Пора ли обновлять значение (для HUD-строки, чтобы цифра не
   * дрожала 90 раз в секунду). `intervalMs` — минимум между
   * обновлениями. Возвращает true, если с момента последнего
   * markUpdated прошло ≥ intervalMs.
   */
  shouldUpdate(intervalMs: number, nowMs: number = 0): boolean {
    if (this.lastUpdateMs === 0) return true;
    return nowMs - this.lastUpdateMs >= intervalMs;
  }

  /** Отметить момент последнего обновления (после чтения value). */
  markUpdated(nowMs: number): void {
    this.lastUpdateMs = nowMs;
  }

  /** Сбросить (например, при suspend страницы). */
  reset(): void {
    this.times = [];
    this.lastUpdateMs = 0;
  }

  /** Текущее число кадров в окне (для тестов). */
  size(): number {
    return this.times.length;
  }
}

/**
 * Считаем FPS как 1 / средний-интервал-между-кадрами.
 *
 *   window     — массив моментов кадров в одной монотонной шкале.
 *   maxFrames  — сколько последних интервалов использовать. Если не
 *                задан, берём весь массив. Нужен для горячего
 *                использования, чтобы не считать среднее каждый раз.
 *
 * Дропнутые кадры обрабатываются автоматически: длинный интервал → низкий
 * FPS. Пустой или одиночный массив → 0 (недостаточно данных).
 */
export function fpsFromFrameTimes(
  frameTimes: ReadonlyArray<number>,
  maxFrames?: number
): number {
  if (!Array.isArray(frameTimes) || frameTimes.length < 2) return 0;
  // Берём окно: либо последние maxFrames+1 моментов, либо весь массив.
  let slice: ReadonlyArray<number>;
  if (maxFrames !== undefined && frameTimes.length > maxFrames + 1) {
    slice = frameTimes.slice(frameTimes.length - maxFrames - 1);
  } else {
    slice = frameTimes;
  }
  if (slice.length < 2) return 0;
  let sumDelta = 0;
  let count = 0;
  for (let i = 1; i < slice.length; i += 1) {
    const a = slice[i - 1];
    const b = slice[i];
    if (!Number.isFinite(a) || !Number.isFinite(b)) continue;
    const d = b - a;
    if (d <= 0) continue;
    sumDelta += d;
    count += 1;
  }
  if (count === 0 || sumDelta <= 0) return 0;
  const avg = sumDelta / count;
  // FPS = 1000/avg (если вход в мс). Если вход в секундах, получится
  // странно, но мы внутри проекта всегда передаём ms через performance.now()
  // — пусть вызывающий решает. Возвращаем 1000/avg как «frames per
  // миллисекунду-шкалы», что для ms равно fps.
  if (avg > 1) {
    // Скорее всего миллисекунды.
    return 1000 / avg;
  }
  // Малые числа — секунды.
  return 1 / avg;
}
