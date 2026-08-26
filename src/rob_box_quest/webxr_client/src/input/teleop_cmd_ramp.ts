// Teleop command ramp handoff (Phase 2 §3.3):
//
// При переключении режима текущий teleop cmd не должен "прыгать" — плавный
// handoff за RAMP_MS (50ms по умолчанию). Применяется между источниками
// (XR → desktop fallback, teleop → explore и т.п.) — outgoing cmd ramp'ится
// к 0, новый cmd ramp'ится от 0 к target. Caller тикает по animation frame.
//
// Чистая логика, без Three.js/DOM. Тестируется через инжектируемый now().

export const RAMP_MS = 50;

export interface RampedInput {
  linear: number; // -1..1
  angular: number; // -1..1
}

export interface RampedOutput {
  linear: number;
  angular: number;
  /** true если ramp ещё активен (значения не равны target). */
  ramping: boolean;
}

export interface RampOptions {
  /** Длительность ramp в мс. Default 50. */
  rampMs?: number;
  /** Текущее время в мс (инжектируется в тестах). */
  now?: () => number;
}

function smoothstep(t: number): number {
  if (t <= 0) return 0;
  if (t >= 1) return 1;
  return t * t * (3 - 2 * t);
}

export class TeleopCmdRamp {
  private current: RampedInput = { linear: 0, angular: 0 };
  private start: RampedInput = { linear: 0, angular: 0 };
  private target: RampedInput = { linear: 0, angular: 0 };
  private rampStartMs = 0;
  private readonly rampMs: number;
  private readonly now: () => number;

  constructor(opts: RampOptions = {}) {
    this.rampMs = opts.rampMs ?? RAMP_MS;
    this.now = opts.now ?? (() => (typeof performance !== "undefined" ? performance.now() : Date.now()));
  }

  /**
   * Установить новый target. Если он сильно отличается от текущего current —
   * запускается ramp длительностью rampMs: current идёт плавно от current
   * к target через smoothstep.
   */
  setTarget(next: RampedInput): void {
    if (
      Math.abs(next.linear - this.target.linear) < 1e-3 &&
      Math.abs(next.angular - this.target.angular) < 1e-3
    ) {
      // Same target — ничего не делаем.
      return;
    }
    // Стартуем ramp от ТЕКУЩЕГО current (если ramp ещё активен — продолжаем
    // плавный handoff, не "прыгаем").
    this.start = { ...this.current };
    this.target = { ...next };
    this.rampStartMs = this.now();
  }

  /** Прямо установить current без ramp (snap). */
  snap(v: RampedInput): void {
    this.current = { ...v };
    this.start = { ...v };
    this.target = { ...v };
    this.rampStartMs = this.now();
  }

  /** Прочитать текущее значение. */
  getCurrent(): RampedInput {
    return { ...this.current };
  }

  /** Прочитать target. */
  getTarget(): RampedInput {
    return { ...this.target };
  }

  /** Ramp активен (значения ещё не дошли до target). */
  isRamping(): boolean {
    if (this.rampMs <= 0) return false;
    const elapsed = this.now() - this.rampStartMs;
    if (elapsed >= this.rampMs) return false;
    return (
      Math.abs(this.current.linear - this.target.linear) > 1e-3 ||
      Math.abs(this.current.angular - this.target.angular) > 1e-3
    );
  }

  /** Тик — вызывается на каждом frame. Возвращает ramped значения для отправки. */
  tick(): RampedOutput {
    const elapsed = this.now() - this.rampStartMs;
    if (this.rampMs <= 0 || elapsed >= this.rampMs) {
      // Завершили ramp.
      this.current = { ...this.target };
      return { ...this.current, ramping: false };
    }
    const t = smoothstep(elapsed / this.rampMs);
    this.current = {
      linear: this.start.linear + (this.target.linear - this.start.linear) * t,
      angular: this.start.angular + (this.target.angular - this.start.angular) * t
    };
    return { ...this.current, ramping: true };
  }

  reset(): void {
    this.current = { linear: 0, angular: 0 };
    this.start = { linear: 0, angular: 0 };
    this.target = { linear: 0, angular: 0 };
    this.rampStartMs = this.now();
  }
}