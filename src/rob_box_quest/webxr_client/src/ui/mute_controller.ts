// src/ui/mute_controller.ts
//
// Push-to-mute: когда активирован (XR-кнопка или горячая клавиша),
// голосовой passthrough ВРЕМЕННО отключается. Persistent — пока не
// тогглим обратно. Не путать с push-to-talk (правый grip удерживаем —
// говорим): mute = «микрофон выключен пока не тогглим».
//
// Это pure state + edge-detection, без DOM. UI-обвязка (HUD-кнопка,
// XR-кнопка, горячая клавиша) дёргает press()/release(). main.ts читает
// isMuted() и блокирует voice passthrough.

export type MuteStateChangeListener = (muted: boolean) => void;

export interface MuteControllerOptions {
  initial?: boolean;
  /** Длительность удержания (мс) для регистрации press→toggle. */
  pressDurationMs?: number;
}

export class MuteController {
  private muted: boolean;
  private listeners: MuteStateChangeListener[] = [];
  private pressStartMs: number | null = null;
  private readonly pressDurationMs: number;

  constructor(opts: MuteControllerOptions = {}) {
    this.muted = opts.initial ?? false;
    this.pressDurationMs = opts.pressDurationMs ?? 350;
  }

  isMuted(): boolean {
    return this.muted;
  }

  /** Кнопка/клавиша нажата — начинаем отсчёт удержания. */
  press(): void {
    if (this.pressStartMs === null) {
      this.pressStartMs = Date.now();
    }
  }

  /** Кнопка/клавиша отпущена — если удержана дольше порога, тогглим mute. */
  release(): boolean {
    if (this.pressStartMs === null) return false;
    const held = Date.now() - this.pressStartMs;
    this.pressStartMs = null;
    if (held >= this.pressDurationMs) {
      this.toggle();
      return true;
    }
    return false;
  }

  /** Прямое переключение (без учёта удержания). */
  toggle(): boolean {
    return this.setMuted(!this.muted);
  }

  setMuted(muted: boolean): boolean {
    if (this.muted === muted) return false;
    this.muted = muted;
    for (const fn of this.listeners) fn(muted);
    return true;
  }

  subscribe(fn: MuteStateChangeListener): () => void {
    this.listeners.push(fn);
    return () => {
      const i = this.listeners.indexOf(fn);
      if (i >= 0) this.listeners.splice(i, 1);
    };
  }
}