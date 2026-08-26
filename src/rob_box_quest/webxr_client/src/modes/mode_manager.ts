// Mode switcher FSM для Captain Bridge (Phase 2 §3).
//
// Четыре режима:
//   - explore (default): только камера и panels. Teleop + voice ВЫКЛ.
//   - teleop: WASD / right-stick. Voice ВЫКЛ (safety).
//   - voice:  микрофон → STT pipeline. Teleop ВЫКЛ.
//   - mixed:  teleop + voice параллельно (для AV-11 epic).
//
// Детерминированные переходы (явные, без неявных сайд-эффектов):
//   - requestMode(target)         — UI / горячая клавиша M / controller A.
//   - reportTeleopIntent(bool)    — auto-upgrade voice→mixed при teleop-вводе.
//   - reportVoiceActive(bool)     — auto-downgrade mixed→teleop при потере voice.
//   - reportDeadmanReleased()     — auto-downgrade mixed→teleop если нет voice.
//
// Правила §3.3:
//   - Переключение не теряет текущий teleop cmd — handoff плавный
//     (это ответственность caller через reportTeleopIntent; FSM только
//     хранит состояние).
//   - В voice + teleop key → upgrade mixed.
//   - В mixed + grip released >500ms + no voice → downgrade teleop.
//
// API чистый (без зависимостей от Three.js/XR/DOM), тестируется в изоляции.

export type CaptainMode = "explore" | "teleop" | "voice" | "mixed";

export const CAPTAIN_MODES: readonly CaptainMode[] = ["explore", "teleop", "voice", "mixed"] as const;

export const DEFAULT_MODE: CaptainMode = "explore";

export interface ModeChangeEvent {
  readonly prev: CaptainMode;
  readonly next: CaptainMode;
  readonly reason: ModeChangeReason;
  readonly atMs: number;
}

export type ModeChangeReason =
  | "ui_select"
  | "hotkey"
  | "controller_button"
  | "auto_upgrade_teleop"
  | "auto_downgrade_voice"
  | "auto_downgrade_deadman";

export type ModeListener = (ev: ModeChangeEvent) => void;

export interface ModeManagerOptions {
  /** Начальный режим; дефолт DEFAULT_MODE. */
  initial?: CaptainMode;
  /** Текущее время в мс (инжектится в тестах). Дефолт Date.now(). */
  now?: () => number;
}

export class ModeManager {
  private current: CaptainMode;
  private readonly now: () => number;
  private readonly listeners = new Set<ModeListener>();
  /** Текущее состояние voice (для решения auto-downgrade). */
  private voiceActive = false;
  /** Сколько раз был запрошен teleop-intent в текущем режиме (флаг). */
  private teleopIntentFlag = false;

  constructor(opts: ModeManagerOptions = {}) {
    this.current = opts.initial ?? DEFAULT_MODE;
    this.now = opts.now ?? Date.now;
  }

  getMode(): CaptainMode {
    return this.current;
  }

  /**
   * Подписка на изменения. Возвращает unsubscribe.
   * Слушатель вызывается синхронно в момент перехода.
   */
  subscribe(listener: ModeListener): () => void {
    this.listeners.add(listener);
    return () => {
      this.listeners.delete(listener);
    };
  }

  /**
   * Явный запрос на смену режима (UI dropdown, M-key, controller A).
   * Если mode === current, ничего не делает и возвращает false.
   */
  requestMode(target: CaptainMode, reason: ModeChangeReason = "ui_select"): boolean {
    if (!isValidMode(target)) return false;
    if (target === this.current) return false;
    return this.transition(target, reason);
  }

  /** Циклический переход к следующему режиму: explore → teleop → voice → mixed → explore. */
  cycleNext(reason: ModeChangeReason = "hotkey"): boolean {
    const idx = CAPTAIN_MODES.indexOf(this.current);
    const next = CAPTAIN_MODES[(idx + 1) % CAPTAIN_MODES.length];
    return this.transition(next, reason);
  }

  /**
   * Сообщить, что пользователь хочет управлять teleop (WASD / thumbstick / grip).
   * В voice → upgrade в mixed. Идемпотентно.
   */
  reportTeleopIntent(): boolean {
    this.teleopIntentFlag = true;
    if (this.current === "voice") {
      return this.transition("mixed", "auto_upgrade_teleop");
    }
    // Если уже в mixed/teleop — no-op.
    return false;
  }

  /** Сообщить, что voice-пайплайн сейчас активен (микрофон → STT). */
  setVoiceActive(active: boolean): boolean {
    this.voiceActive = active;
    // mixed + voice потерян → downgrade teleop.
    if (!active && this.current === "mixed") {
      return this.transition("teleop", "auto_downgrade_voice");
    }
    return false;
  }

  isVoiceActive(): boolean {
    return this.voiceActive;
  }

  hasTeleopIntent(): boolean {
    return this.teleopIntentFlag;
  }

  /**
   * Сообщить, что deadman отпущен. В mixed без активного voice — downgrade teleop.
   * Вызывающий код (deadman_timer) сам решает, прошло ли >500ms; FSM только
   * принимает решение о переходе.
   */
  reportDeadmanReleased(): boolean {
    if (this.current === "mixed" && !this.voiceActive) {
      return this.transition("teleop", "auto_downgrade_deadman");
    }
    return false;
  }

  /** Сбросить флаги и вернуться в DEFAULT_MODE (например, при reconnect). */
  reset(): void {
    this.teleopIntentFlag = false;
    this.voiceActive = false;
    if (this.current !== DEFAULT_MODE) {
      this.transition(DEFAULT_MODE, "ui_select");
    }
  }

  private transition(next: CaptainMode, reason: ModeChangeReason): boolean {
    const prev = this.current;
    if (prev === next) return false;
    this.current = next;
    const ev: ModeChangeEvent = { prev, next, reason, atMs: this.now() };
    // Копируем listeners — иначе listener, который отписывается внутри, сломает итерацию.
    for (const l of [...this.listeners]) {
      try {
        l(ev);
      } catch {
        // Не должно падать — но и крашить UI из-за listener'а тоже нельзя.
        // eslint-disable-next-line no-console
        console.warn("[mode_manager] listener threw", ev);
      }
    }
    return true;
  }
}

export function isValidMode(m: unknown): m is CaptainMode {
  return typeof m === "string" && (CAPTAIN_MODES as readonly string[]).includes(m);
}