// Desktop WASD/Space/E fallback для teleop (дизайн §6).
// Активируется автоматически, если XR недоступен.
//
// Phase 2 расширения:
//   - KeyM  → cycle next Captain Mode (через ModeManager).
//   - Shift → camera orbit mode (mouse → camera orbit).
//   - Escape → immediate emergency stop.
//   - Boost (Space held) → 1.5x linear speed (применяется через fsm).
//
// Boost multiplier применяется через `setLinear(linear * BOOST_MULT)` при
// зажатом Space — TeleopFSM получает уже масштабированное значение.

import type { TeleopFSM } from "./teleop_fsm";
import type { ModeManager } from "../modes/mode_manager";

export const BOOST_MULT = 1.5;

export interface DesktopTeleopHandle {
  destroy(): void;
  isActive(): boolean;
}

export interface DesktopTeleopOptions {
  fsm: TeleopFSM;
  /** ModeManager — опциональный. Если передан, KeyM циклит режимы. */
  modeManager?: ModeManager;
  /** Boost активен (вызывающий код может сбросить, например, в режиме explore). */
  boostEnabled?: () => boolean;
  element?: HTMLElement | Window;
}

const KEY_BINDINGS: Record<string, "linear+" | "linear-" | "angular+" | "angular-" | "deadman" | "emergency"> = {
  KeyW: "linear+",
  KeyS: "linear-",
  KeyA: "angular+",
  KeyD: "angular-",
  ArrowUp: "linear+",
  ArrowDown: "linear-",
  ArrowLeft: "angular+",
  ArrowRight: "angular-",
  Space: "deadman",
  KeyE: "emergency"
};

const SPECIAL_KEYS = new Set(["KeyM", "Escape", "ShiftLeft", "ShiftRight"]);

export function createDesktopTeleop(opts: DesktopTeleopOptions): DesktopTeleopHandle {
  const fsm = opts.fsm;
  const modeManager = opts.modeManager;
  const isBoostAllowed = opts.boostEnabled ?? (() => true);
  const target: HTMLElement | Window = opts.element ?? window;
  const pressed = new Set<string>();
  /** Линейная/угловая скорости *без* boost (для масштабирования). */
  let rawLinear = 0;
  let rawAngular = 0;

  function isInTextField(tgt: EventTarget | null): boolean {
    if (!tgt) return false;
    const el = tgt as HTMLElement | null;
    if (!el) return false;
    return (
      el.tagName === "INPUT" ||
      el.tagName === "TEXTAREA" ||
      el.tagName === "SELECT" ||
      (el as HTMLElement).isContentEditable === true
    );
  }

  function onKeyDown(ev: KeyboardEvent): void {
    if (isInTextField(ev.target)) return;
    // Режим-цикл.
    if (ev.code === "KeyM" && modeManager) {
      ev.preventDefault();
      modeManager.cycleNext("hotkey");
      return;
    }
    // Escape → emergency немедленно.
    if (ev.code === "Escape") {
      ev.preventDefault();
      pressed.add("Escape");
      return;
    }
    const binding = KEY_BINDINGS[ev.code];
    if (!binding) return;
    ev.preventDefault();
    if (binding === "emergency") {
      pressed.add(ev.code);
      return;
    }
    pressed.add(ev.code);
    refresh();
  }

  function onKeyUp(ev: KeyboardEvent): void {
    const binding = KEY_BINDINGS[ev.code];
    if (binding || SPECIAL_KEYS.has(ev.code)) {
      pressed.delete(ev.code);
      if (binding) refresh();
    }
  }

  function refresh(): void {
    const linear =
      (pressed.has("KeyW") || pressed.has("ArrowUp") ? 1 : 0) +
      (pressed.has("KeyS") || pressed.has("ArrowDown") ? -1 : 0);
    const angular =
      (pressed.has("KeyA") || pressed.has("ArrowLeft") ? 1 : 0) +
      (pressed.has("KeyD") || pressed.has("ArrowRight") ? -1 : 0);
    rawLinear = linear;
    rawAngular = angular;
    const deadman = pressed.has("Space");
    const boost = deadman && isBoostAllowed();
    const mult = boost ? BOOST_MULT : 1.0;
    fsm.setLinear(linear * mult);
    fsm.setAngular(angular);
    fsm.setDeadman(deadman);
  }

  function consumeEmergency(): boolean {
    if (pressed.has("KeyE")) {
      pressed.delete("KeyE");
      return true;
    }
    if (pressed.has("Escape")) {
      pressed.delete("Escape");
      return true;
    }
    return false;
  }

  /** Текущие raw-значения (без boost) — для отладки / UI. */
  function getRaw(): { linear: number; angular: number; boost: boolean; deadman: boolean } {
    const deadman = pressed.has("Space");
    return {
      linear: rawLinear,
      angular: rawAngular,
      boost: deadman && isBoostAllowed(),
      deadman
    };
  }

  target.addEventListener("keydown", onKeyDown as EventListener);
  target.addEventListener("keyup", onKeyUp as EventListener);
  (target as unknown as { __questDesktopTeleop?: unknown }).__questDesktopTeleop = { consumeEmergency, getRaw };

  return {
    destroy(): void {
      target.removeEventListener("keydown", onKeyDown as EventListener);
      target.removeEventListener("keyup", onKeyUp as EventListener);
      pressed.clear();
    },
    isActive(): boolean {
      return pressed.size > 0;
    }
  };
}