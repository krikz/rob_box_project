// Quest XR controllers teleop (дизайн §6 + Phase 2 §3).
//
// §3.4 bindings (стандартный XR gamepad mapping):
//   Left thumbstick X/Y (axes[0]/[1]) → movement (linear + angular)
//   Right thumbstick X/Y (axes[2]/[3]) → camera rotate (yaw/pitch)
//   Left grip  → boost (1.5x linear) + deadman=true
//   Right B    → emergency edge
//   Left A     → cycle next Captain Mode (modeCycleEdge)
//
// §3.6: right stick rotation применяется через camera controller (lerp damping).
// Reset camera — отдельный one-shot edge (`consumeResetCameraEdge`).
//
// Реализация через WebXR `input_sources` API. Вызывающий код подписывается
// на onChange и передаёт в FSM через tick(). Boost multiplier применяется
// к fsm.setLinear() при зажатом grip.

import type { TeleopFSM } from "./teleop_fsm";
import type { ModeManager } from "../modes/mode_manager";
import { DeadmanTimer } from "./deadman_timer";

export const BOOST_MULT = 1.5;
const MODE_BUTTON_INDEX = 4; // Meta Quest A-button (left controller standard mapping)
const GRIP_BUTTON_INDEX = 1;
const EMERGENCY_BUTTON_INDEX = 5; // Meta B (right controller)
// §3.4: Left thumbstick = axes[0]/[1] (movement), Right thumbstick = axes[2]/[3] (camera).
const LEFT_STICK_X = 0;
const LEFT_STICK_Y = 1;
const RIGHT_STICK_X = 2;
const RIGHT_STICK_Y = 3;
const DEADZONE = 0.12;

export interface XrTeleopHandle {
  destroy(): void;
  isActive(): boolean;
}

export interface XrInputState {
  linear: number;
  angular: number;
  /** Правая ручка — yaw (поворот камеры вокруг Y). -1..1 */
  cameraYaw: number;
  /** Правая ручка — pitch (наклон камеры). -1..1 */
  cameraPitch: number;
  deadman: boolean;
  emergencyEdge: boolean;
  /** Событие mode-cycle от A-кнопки (one-shot, consume). */
  modeCycleEdge: boolean;
  /** Rising-edge reset-camera (one-shot, consume). Alias для modeCycle, чтобы
   * UI мог различать "cycle" и "reset" если понадобится. */
  resetCameraEdge: boolean;
}

export interface XrTeleopOptions {
  fsm: TeleopFSM;
  session: XRSession;
  /** ModeManager — если передан, A-кнопка циклит режим. */
  modeManager?: ModeManager;
  /** Callback на emergency (например, чтобы отправить JSON_CMD). */
  onEmergency?: (source: "controller_b" | "ui_button") => void;
  /** Callback на warning deadman (300ms). */
  onDeadmanWarning?: (remainingMs: number) => void;
  /** Callback на triggered deadman (>500ms). */
  onDeadmanTriggered?: (elapsedMs: number) => void;
  /** Тикнуть deadman timer. Если не задан — timer не используется. */
  shouldTickDeadman?: () => boolean;
}

function applyDeadzone(v: number): number {
  if (Math.abs(v) < DEADZONE) return 0;
  const sign = v < 0 ? -1 : 1;
  return sign * ((Math.abs(v) - DEADZONE) / (1 - DEADZONE));
}

/** Поля, которые читаются из XRInputSource.gamepad. */
interface GamepadLike {
  axes: ArrayLike<number>;
  buttons: ArrayLike<{ value?: number; pressed?: boolean }>;
}

/** True, если источник — left controller (по .handedness). */
function isLeftHand(source: XRInputSource): boolean {
  return source.handedness === "left";
}

/**
 * Полль одного XRInputSource: обновляет fsm (linear/angular/deadman +
 * boost) и фиксирует emergency/mode-cycle edges.
 *
 * Left controller: left thumbstick + grip + A-button.
 * Right controller: right thumbstick (camera) + B-button (emergency).
 *
 * Параметр `intentSink` (если передан) — сюда пишется computed intent
 * (linear/angular/deadman после boost). Используется caller'ом для ramp'а.
 * Если intentSink не передан, intent пишется в fsm напрямую (старая логика).
 */
export function pollXrInput(
  state: XrInputState,
  source: XRInputSource,
  fsm: TeleopFSM,
  intentSink?: { linear: number; angular: number; deadman: boolean } | null,
  _opts?: XrTeleopOptions
): void {
  if (!source.gamepad) return;
  const gp = source.gamepad as unknown as GamepadLike;

  if (isLeftHand(source)) {
    // Movement — left thumbstick, boost если гrip зажат.
    const gripVal = gp.buttons[GRIP_BUTTON_INDEX]?.value ?? 0;
    const grip = gripVal > 0.5;
    const lx = applyDeadzone(gp.axes[LEFT_STICK_X] ?? 0);
    const ly = applyDeadzone(gp.axes[LEFT_STICK_Y] ?? 0);
    const linear = ly * (grip ? BOOST_MULT : 1.0);
    const angular = -lx;
    if (intentSink) {
      intentSink.linear = linear;
      intentSink.angular = angular;
      intentSink.deadman = grip;
    } else {
      fsm.setLinear(linear);
      fsm.setAngular(angular);
      fsm.setDeadman(grip);
    }

    state.linear = fsm.getState() === "idle" ? 0 : ly;
    state.angular = fsm.getState() === "idle" ? 0 : lx;
    state.deadman = grip;

    if (gp.buttons[MODE_BUTTON_INDEX]?.pressed) {
      state.modeCycleEdge = true;
      state.resetCameraEdge = true; // A-button = mode cycle AND camera reset (по §3.6)
    }
  } else {
    // Right controller — camera rotate + B emergency.
    const rx = applyDeadzone(gp.axes[RIGHT_STICK_X] ?? 0);
    const ry = applyDeadzone(gp.axes[RIGHT_STICK_Y] ?? 0);
    state.cameraYaw = rx;
    state.cameraPitch = ry;

    if (gp.buttons[EMERGENCY_BUTTON_INDEX]?.pressed) {
      state.emergencyEdge = true;
    }
  }
}

export function createXrTeleop(opts: XrTeleopOptions): XrTeleopHandle {
  const state: XrInputState = {
    linear: 0,
    angular: 0,
    cameraYaw: 0,
    cameraPitch: 0,
    deadman: false,
    emergencyEdge: false,
    modeCycleEdge: false,
    resetCameraEdge: false
  };
  const deadmanTimer = new DeadmanTimer();

  const handler = (_ev: XRInputSourcesChangeEvent) => {
    void _ev;
  };
  opts.session.addEventListener("inputsourceschange", handler);

  // Вспомогательные методы, доступные вызывающему коду.
  (opts as unknown as { __xrDeadmanTimer?: DeadmanTimer; __xrTeleopState?: XrInputState }).__xrDeadmanTimer = deadmanTimer;
  (opts as unknown as { __xrTeleopState?: XrInputState }).__xrTeleopState = state;

  return {
    destroy(): void {
      opts.session.removeEventListener("inputsourceschange", handler);
      deadmanTimer.reset();
    },
    isActive(): boolean {
      return state.deadman;
    }
  };
}

/** Извлечь state + tick deadman timer (вызывающий код использует в loop). */
export function tickXrTeleop(opts: XrTeleopOptions): {
  consumeEmergency(): boolean;
  consumeModeCycleEdge(): boolean;
  consumeResetCameraEdge(): boolean;
  getCameraAxes(): { yaw: number; pitch: number };
  tickDeadman(gripHeld: boolean): { warning?: number; triggered?: number };
} {
  const x = opts as unknown as {
    __xrDeadmanTimer?: DeadmanTimer;
    __xrTeleopState?: XrInputState;
  };
  const timer = x.__xrDeadmanTimer ?? new DeadmanTimer();
  const state = x.__xrTeleopState;
  return {
    consumeEmergency(): boolean {
      if (state?.emergencyEdge) {
        state.emergencyEdge = false;
        opts.onEmergency?.("controller_b");
        return true;
      }
      return false;
    },
    consumeModeCycleEdge(): boolean {
      if (state?.modeCycleEdge && opts.modeManager) {
        state.modeCycleEdge = false;
        opts.modeManager.cycleNext("controller_button");
        return true;
      }
      if (state) state.modeCycleEdge = false;
      return false;
    },
    consumeResetCameraEdge(): boolean {
      if (state?.resetCameraEdge) {
        state.resetCameraEdge = false;
        return true;
      }
      return false;
    },
    getCameraAxes(): { yaw: number; pitch: number } {
      return { yaw: state?.cameraYaw ?? 0, pitch: state?.cameraPitch ?? 0 };
    },
    tickDeadman(gripHeld: boolean): { warning?: number; triggered?: number } {
      if (gripHeld) {
        timer.gripPressed();
        return {};
      }
      const ev = timer.check();
      if (!ev) return {};
      if (ev.kind === "warning") {
        opts.onDeadmanWarning?.(ev.remainingMs);
        return { warning: ev.remainingMs };
      }
      opts.onDeadmanTriggered?.(ev.elapsedMs);
      opts.onEmergency?.("controller_b");
      return { triggered: ev.elapsedMs };
    }
  };
}

/** Хелпер: тикнуть deadman timer (вызывающий код зовёт из animation loop). */
export function tickDeadmanTimer(
  timer: DeadmanTimer,
  gripHeld: boolean,
  opts: { onWarning?: (remainingMs: number) => void; onTriggered?: (elapsedMs: number) => void }
): void {
  if (gripHeld) {
    timer.gripPressed();
    return;
  }
  // Если grip не зажат и timer в состоянии "released", пусть он
  // считает elapsed. Каждый кадр вызывающий код зовёт check().
  const ev = timer.check();
  if (!ev) return;
  if (ev.kind === "warning") opts.onWarning?.(ev.remainingMs);
  else if (ev.kind === "triggered") opts.onTriggered?.(ev.elapsedMs);
}