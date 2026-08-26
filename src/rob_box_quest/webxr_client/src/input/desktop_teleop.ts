// Desktop WASD/Space/E fallback для teleop (дизайн §6 + Phase 2 §3.9).
// Активируется автоматически, если XR недоступен.
//
// Phase 2 §3.9 bindings:
//   - WASD / Arrow keys → movement (linear/angular)
//   - Space            → boost (1.5x linear speed, требует boost-allowed gate)
//   - E                → interact (panel under cursor — UI сам решает, что)
//   - Shift (held)     → camera-orbit mode (mouse → camera orbit, если реализовано)
//   - Escape           → immediate emergency stop
//   - KeyM             → cycle next Captain Mode (через ModeManager)
//
// Boost multiplier применяется через `setLinear(linear * BOOST_MULT)` при
// зажатом Space — TeleopFSM получает уже масштабированное значение.
//
// Safety: даже когда XR активен, desktop teleop можно держать активным
// (например, для дев-сценария без XR). Каждый режим всё равно гейтит
// boost через boostEnabled callback из main.ts (teleop/mixed only).

import type { TeleopFSM } from "./teleop_fsm";
import type { ModeManager } from "../modes/mode_manager";

export const BOOST_MULT = 1.5;

export type DesktopAction =
  | "emergency"
  | "interact"
  | "camera_orbit_enter"
  | "camera_orbit_exit";

export interface DesktopTeleopHandle {
  destroy(): void;
  isActive(): boolean;
  /** Подписка на сигнал "interact" (E-key rising edge) — UI ловит через subscribe. */
  onInteract: (cb: () => void) => () => void;
  /** Подписка на сигнал "emergency" rising edge. */
  onEmergencyEdge: (cb: (source: "escape" | "e_key") => void) => () => void;
}

export interface DesktopTeleopOptions {
  fsm: TeleopFSM;
  /** ModeManager — опциональный. Если передан, KeyM циклит режимы. */
  modeManager?: ModeManager;
  /** Boost активен (вызывающий код может сбросить, например, в режиме explore). */
  boostEnabled?: () => boolean;
  element?: HTMLElement | Window;
  /** Callback на rising-edge interact (E-key). Используется UI. */
  onInteract?: () => void;
  /** Callback на rising-edge emergency. Используется UI/caller для отправки stop_emergency. */
  onEmergency?: (source: "escape" | "e_key") => void;
}

const KEY_BINDINGS: Record<string, "linear+" | "linear-" | "angular+" | "angular-"> = {
  KeyW: "linear+",
  KeyS: "linear-",
  KeyA: "angular+",
  KeyD: "angular-",
  ArrowUp: "linear+",
  ArrowDown: "linear-",
  ArrowLeft: "angular+",
  ArrowRight: "angular-"
};

export function createDesktopTeleop(opts: DesktopTeleopOptions): DesktopTeleopHandle {
  const fsm = opts.fsm;
  const modeManager = opts.modeManager;
  const isBoostAllowed = opts.boostEnabled ?? (() => true);
  const target: HTMLElement | Window = opts.element ?? window;
  const pressed = new Set<string>();
  /** Edge-флаги (consumed once). */
  let emergencyEdge = false;
  let interactEdge = false;
  let cameraOrbitActive = false;
  /** Линейная/угловая скорости *без* boost (для масштабирования). */
  let rawLinear = 0;
  let rawAngular = 0;
  /** Реальное состояние deadman (Space зажат). */
  let deadmanHeld = false;
  /** Текущий source последней emergency edge (для consumeEmergency). */
  let lastEmergencySource: "escape" | "e_key" = "escape";

  const interactListeners = new Set<() => void>();
  const emergencyListeners = new Set<(source: "escape" | "e_key") => void>();

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
    // Escape → emergency немедленно (rising edge).
    if (ev.code === "Escape" && !pressed.has("Escape")) {
      ev.preventDefault();
      emergencyEdge = true;
      lastEmergencySource = "escape";
      for (const cb of emergencyListeners) {
        try { cb("escape"); } catch { /* swallow */ }
      }
    }
    // E-key rising edge → interact.
    if (ev.code === "KeyE" && !pressed.has("KeyE")) {
      ev.preventDefault();
      interactEdge = true;
      for (const cb of interactListeners) {
        try { cb(); } catch { /* swallow */ }
      }
    }
    // Shift (rising edge) → camera orbit.
    if ((ev.code === "ShiftLeft" || ev.code === "ShiftRight") && !cameraOrbitActive) {
      cameraOrbitActive = true;
    }
    const binding = KEY_BINDINGS[ev.code];
    if (!binding) return;
    ev.preventDefault();
    pressed.add(ev.code);
    refresh();
  }

  function onKeyUp(ev: KeyboardEvent): void {
    if (KEY_BINDINGS[ev.code]) {
      pressed.delete(ev.code);
      refresh();
      return;
    }
    if (ev.code === "Space") {
      pressed.delete("Space");
      deadmanHeld = false;
      refresh();
      return;
    }
    if (ev.code === "KeyE" || ev.code === "Escape") {
      pressed.delete(ev.code);
      return;
    }
    if (ev.code === "ShiftLeft" || ev.code === "ShiftRight") {
      if (!pressed.has("ShiftLeft") && !pressed.has("ShiftRight")) {
        cameraOrbitActive = false;
      }
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
    // Space boost: §3.9 Space = boost, не deadman.
    const boost = pressed.has("Space") && isBoostAllowed();
    const mult = boost ? BOOST_MULT : 1.0;
    // Deadman (XR-стиль) — на desktop Space выполняет функцию "hold-to-be-alive".
    // Без Space — линейная/угловая скорости = 0 (safety).
    deadmanHeld = pressed.has("Space");
    if (!deadmanHeld) {
      fsm.setLinear(0);
      fsm.setAngular(0);
      fsm.setDeadman(false);
    } else {
      fsm.setLinear(linear * mult);
      fsm.setAngular(angular);
      fsm.setDeadman(true);
    }
  }

  function consumeEmergencyEdge(): { source: "escape" | "e_key" } | null {
    if (!emergencyEdge) return null;
    emergencyEdge = false;
    return { source: lastEmergencySource };
  }

  function consumeInteractEdge(): boolean {
    if (!interactEdge) return false;
    interactEdge = false;
    return true;
  }

  /** Текущие raw-значения (без boost) — для отладки / UI. */
  function getRaw(): { linear: number; angular: number; boost: boolean; deadman: boolean } {
    const boost = pressed.has("Space") && isBoostAllowed();
    return {
      linear: rawLinear,
      angular: rawAngular,
      boost,
      deadman: deadmanHeld
    };
  }

  /** Boost-scaled linear (после применения BOOST_MULT) — для ramp'а. */
  function getBoostedLinear(): number {
    const boost = pressed.has("Space") && isBoostAllowed();
    return rawLinear * (boost ? BOOST_MULT : 1.0);
  }

  target.addEventListener("keydown", onKeyDown as EventListener);
  target.addEventListener("keyup", onKeyUp as EventListener);
  (target as unknown as {
    __questDesktopTeleop?: unknown;
  }).__questDesktopTeleop = {
    consumeEmergency: consumeEmergencyEdge,
    consumeInteract: consumeInteractEdge,
    getRaw,
    isCameraOrbitActive: () => cameraOrbitActive,
    getIntent: () => ({ linear: getBoostedLinear(), angular: rawAngular }),
    isDeadmanHeld: () => deadmanHeld
  };

  // Прокидываем внешние callbacks (для интеграции в main.ts).
  if (opts.onInteract) interactListeners.add(opts.onInteract);
  if (opts.onEmergency) emergencyListeners.add(opts.onEmergency);

  return {
    destroy(): void {
      target.removeEventListener("keydown", onKeyDown as EventListener);
      target.removeEventListener("keyup", onKeyUp as EventListener);
      pressed.clear();
      interactListeners.clear();
      emergencyListeners.clear();
    },
    isActive(): boolean {
      return pressed.size > 0;
    },
    onInteract(cb): () => void {
      interactListeners.add(cb);
      return () => {
        interactListeners.delete(cb);
      };
    },
    onEmergencyEdge(cb): () => void {
      emergencyListeners.add(cb);
      return () => {
        emergencyListeners.delete(cb);
      };
    }
  };
}