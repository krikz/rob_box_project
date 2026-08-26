// Quest XR controllers teleop (дизайн §6 + Phase 2 §3):
//   Left thumbstick Y → linear.x (boost если grip зажат)
//   Left thumbstick X → angular.z
//   Grip (любой)       → deadman=true (дополнительно — boost)
//   B (любой)          → emergency stop (deadman timer → если grip
//                        отпущен >500ms — также emergency)
//   A (left controller) → cycle next Captain Mode
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
const EMERGENCY_BUTTON_INDEX = 5; // Meta B
const THUMBSTICK_AXIS_X = 2;
const THUMBSTICK_AXIS_Y = 3;
const DEADZONE = 0.12;

export interface XrTeleopHandle {
  destroy(): void;
  isActive(): boolean;
}

export interface XrInputState {
  linear: number;
  angular: number;
  deadman: boolean;
  emergencyEdge: boolean;
  /** Событие mode-cycle от A-кнопки (one-shot, consume). */
  modeCycleEdge: boolean;
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

/**
 * Полль одного XRInputSource: обновляет fsm (linear/angular/deadman +
 * boost) и фиксирует emergency/mode-cycle edges.
 */
export function pollXrInput(
  state: XrInputState,
  source: XRInputSource,
  fsm: TeleopFSM,
  _opts: XrTeleopOptions
): void {
  if (source.gamepad) {
    const gp = source.gamepad as unknown as GamepadLike;
    const gripVal = gp.buttons[GRIP_BUTTON_INDEX]?.value ?? 0;
    const grip = gripVal > 0.5;

    // linear с boost если grip зажат (только для X — strafe тоже boosted).
    const tx = applyDeadzone(gp.axes[THUMBSTICK_AXIS_X] ?? 0);
    const ty = applyDeadzone(gp.axes[THUMBSTICK_AXIS_Y] ?? 0);
    fsm.setLinear(ty * (grip ? BOOST_MULT : 1.0));
    fsm.setAngular(-tx);
    fsm.setDeadman(grip);

    state.linear = fsm.getState() === "idle" ? 0 : ty;
    state.angular = fsm.getState() === "idle" ? 0 : tx;
    state.deadman = grip;

    if (gp.buttons[EMERGENCY_BUTTON_INDEX]?.pressed) {
      state.emergencyEdge = true;
    }
    if (gp.buttons[MODE_BUTTON_INDEX]?.pressed) {
      state.modeCycleEdge = true;
    }
  }
}

export function createXrTeleop(opts: XrTeleopOptions): XrTeleopHandle {
  const state: XrInputState = {
    linear: 0,
    angular: 0,
    deadman: false,
    emergencyEdge: false,
    modeCycleEdge: false
  };
  const deadmanTimer = new DeadmanTimer();

  // Каждый кадр xr-loop может звать pollXrInput. Этот же callback
  // обновляет deadman timer (через caller).
  const handler = (_ev: XRInputSourcesChangeEvent) => {
    void _ev;
  };
  opts.session.addEventListener("inputsourceschange", handler);

  // Вспомогательные методы, доступные вызывающему коду.
  (opts as unknown as { __xrDeadmanTimer?: DeadmanTimer }).__xrDeadmanTimer = deadmanTimer;

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