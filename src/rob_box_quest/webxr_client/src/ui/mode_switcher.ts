// src/ui/mode_switcher.ts
//
// Mode switcher для Captain Bridge — 4 режима взаимодействия с роботом:
//
//   explore: passive, никаких команд не шлётся (без arm — teleop_disabled).
//            Voice passthrough работает (это «рация», не зависит от mode).
//   teleop:  arm-button включает движение. Voice passthrough работает.
//   voice:   voice passthrough (radio и robot-voice) разрешён.
//            Teleop явно DISARM (рука или стик не управляют роботом).
//   mixed:   teleop + voice одновременно (для AV-11 epic).
//
// Mode — это фильтр между input events и outgoing commands:
//   - explore → teleop-команды НЕ шлются (даже если arm=true)
//   - voice   → teleop-команды НЕ шлются (даже если arm=true)
//   - teleop  → voice passthrough работает (можно говорить и рулить)
//   - mixed   → всё разрешено
//
// Это pure state machine — без DOM, без Three.js. UI-обвязка (HUD-кнопки,
// клавиатура 1-4) делает setMode(); движок (main.ts) читает currentMode()
// и пропускает команды через `shouldEmitTeleop()` / `shouldEmitVoice()`.

export type BridgeMode = "explore" | "teleop" | "voice" | "mixed";

export const BRIDGE_MODES: readonly BridgeMode[] = [
  "explore",
  "teleop",
  "voice",
  "mixed"
] as const;

const DEFAULT_MODE: BridgeMode = "explore";

/** Subset of mode where teleop commands (linear/angular/emergency) are emitted. */
export function teleopEnabled(mode: BridgeMode): boolean {
  return mode === "teleop" || mode === "mixed";
}

/** Subset of mode where voice passthrough (radio + robot_voice) is allowed. */
export function voiceEnabled(mode: BridgeMode): boolean {
  return mode === "voice" || mode === "mixed" || mode === "teleop" || mode === "explore";
}

/**
 * Раскладка клавишру по умолчанию: 1=explore, 2=teleop, 3=voice, 4=mixed.
 * Возвращает новый mode или null если клавиша не привязана.
 */
export function modeFromKey(key: string): BridgeMode | null {
  switch (key) {
    case "1":
      return "explore";
    case "2":
      return "teleop";
    case "3":
      return "voice";
    case "4":
      return "mixed";
    default:
      return null;
  }
}

/** Человекочитаемая метка для HUD/логов. */
export function modeLabel(mode: BridgeMode): string {
  switch (mode) {
    case "explore":
      return "EXPLORE";
    case "teleop":
      return "TELEOP";
    case "voice":
      return "VOICE";
    case "mixed":
      return "MIXED";
  }
}

export type ModeChangeListener = (mode: BridgeMode, prev: BridgeMode) => void;

export interface ModeSwitcherOptions {
  initial?: BridgeMode;
}

export class ModeSwitcher {
  private mode: BridgeMode;
  private listeners: ModeChangeListener[] = [];

  constructor(opts: ModeSwitcherOptions = {}) {
    this.mode = opts.initial ?? DEFAULT_MODE;
  }

  current(): BridgeMode {
    return this.mode;
  }

  /** Установить режим. Если mode совпадает с текущим — no-op (listener не зовётся). */
  setMode(next: BridgeMode): boolean {
    if (next === this.mode) return false;
    const prev = this.mode;
    this.mode = next;
    for (const fn of this.listeners) fn(next, prev);
    return true;
  }

  /** Перейти к следующему режиму по кругу (explore → teleop → voice → mixed → explore). */
  cycle(): BridgeMode {
    const idx = BRIDGE_MODES.indexOf(this.mode);
    const next = BRIDGE_MODES[(idx + 1) % BRIDGE_MODES.length];
    this.setMode(next);
    return this.mode;
  }

  /** Подписка на смену режима. Возвращает unsubscribe. */
  subscribe(fn: ModeChangeListener): () => void {
    this.listeners.push(fn);
    return () => {
      const i = this.listeners.indexOf(fn);
      if (i >= 0) this.listeners.splice(i, 1);
    };
  }

  /** Пропускать ли teleop-команду в WSS для текущего режима? */
  shouldEmitTeleop(): boolean {
    return teleopEnabled(this.mode);
  }

  /** Пропускать ли voice passthrough в WSS для текущего режима? */
  shouldEmitVoice(): boolean {
    return voiceEnabled(this.mode);
  }
}