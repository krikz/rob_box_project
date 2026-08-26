// Deadman safety timer (Phase 2 §3.8):
//
//   - Если grip отпущен > DEADMAN_RELEASE_MS (500ms) → publish emergency_stop.
//   - Timer reset при каждом grip press.
//   - За 200ms до trigger (т.е. на 300ms после release) → HUD warning
//     "Release B to stop!" (вызывающий код решает, что показать).
//
// Чистая логика без DOM/XR — тестируется через инжектируемый now() и
// callback'и. Реальные setTimeout не используются — caller тикает по
// animation frame и зовёт check().

export const DEADMAN_RELEASE_MS = 500;
export const DEADMAN_WARNING_MS = 300;

export type DeadmanEvent =
  | { kind: "warning"; remainingMs: number }
  | { kind: "triggered"; elapsedMs: number };

export interface DeadmanTimerOptions {
  /** Текущее время в мс. Дефолт Date.now. */
  now?: () => number;
  /** Release threshold (мс). Дефолт 500. */
  releaseMs?: number;
  /** Warning threshold (мс). Дефолт 300. */
  warningMs?: number;
}

export class DeadmanTimer {
  private lastGripAtMs: number | null = null;
  private warned = false;
  private triggered = false;
  private readonly now: () => number;
  private readonly releaseMs: number;
  private readonly warningMs: number;

  constructor(opts: DeadmanTimerOptions = {}) {
    this.now = opts.now ?? Date.now;
    this.releaseMs = opts.releaseMs ?? DEADMAN_RELEASE_MS;
    this.warningMs = opts.warningMs ?? DEADMAN_WARNING_MS;
    if (this.warningMs > this.releaseMs) {
      throw new Error(`warningMs (${this.warningMs}) must be <= releaseMs (${this.releaseMs})`);
    }
  }

  /** Вызывать при каждом grip press (rising edge). */
  gripPressed(): void {
    this.lastGripAtMs = this.now();
    this.warned = false;
    this.triggered = false;
  }

  /** Вызывать при grip release (falling edge). Запоминает момент release. */
  gripReleased(): void {
    this.lastGripAtMs = this.now();
    this.warned = false;
    this.triggered = false;
  }

  /** Сбросить (например, при смене режима или reconnect). */
  reset(): void {
    this.lastGripAtMs = null;
    this.warned = false;
    this.triggered = false;
  }

  /** Текущее состояние grip (нажат ли прямо сейчас). */
  isGripHeld(): boolean {
    return this.lastGripAtMs != null && !this.triggered;
  }

  /** Время с момента release (или null, если grip не отпускался). */
  elapsedSinceRelease(nowMs?: number): number | null {
    if (this.lastGripAtMs == null) return null;
    const now = nowMs ?? this.now();
    return now - this.lastGripAtMs;
  }

  /**
   * Тик — вызывается на animation frame. Возвращает событие, если
   * произошёл warning или trigger. После trigger timer переходит в
   * "выстреливший" режим и больше не тикает (нужен reset() или
   * gripPressed() для повторного использования).
   */
  check(nowMs?: number): DeadmanEvent | null {
    if (this.triggered) return null;
    const elapsed = this.elapsedSinceRelease(nowMs);
    if (elapsed == null) return null;

    if (elapsed >= this.releaseMs) {
      this.triggered = true;
      return { kind: "triggered", elapsedMs: elapsed };
    }

    if (!this.warned && elapsed >= this.warningMs) {
      this.warned = true;
      return { kind: "warning", remainingMs: this.releaseMs - elapsed };
    }

    return null;
  }

  /** Был ли триггер уже выпущен (для UI / защиты от двойной отправки). */
  hasTriggered(): boolean {
    return this.triggered;
  }
}