// Quest XR controllers teleop (дизайн §6):
//   Left thumbstick Y → linear.x
//   Left thumbstick X → angular.z
//   Grip (любой)       → deadman=true
//   B (любой)          → emergency stop
//
// Реализация через WebXR `input_sources` API. Вызывающий код
// подписывается на onChange и передаёт в FSM через tick().

import type { TeleopFSM } from "./teleop_fsm";

export interface XrTeleopHandle {
  destroy(): void;
  isActive(): boolean;
}

export interface XrInputState {
  linear: number; // -1..1
  angular: number; // -1..1
  deadman: boolean; // grip зажат
  emergencyEdge: boolean; // B нажата (один раз, debounce)
}

const DEADMAN_GAMEPAD_BUTTON_INDEX = 1; // grip
const EMERGENCY_GAMEPAD_BUTTON_INDEX = 5; // B
const THUMBSTICK_AXIS_X = 2;
const THUMBSTICK_AXIS_Y = 3;
const DEADZONE = 0.12;

export function pollXrInput(state: XrInputState, source: XRInputSource, fsm: TeleopFSM): void {
  if (source.gamepad) {
    const gp = source.gamepad;
    const grip = (gp.buttons[DEADMAN_GAMEPAD_BUTTON_INDEX]?.value ?? 0) > 0.5;
    fsm.setDeadman(grip);

    // thumbstick (axes 2/3 для стандартного Quest mapping).
    const tx = gp.axes[THUMBSTICK_AXIS_X] ?? 0;
    const ty = gp.axes[THUMBSTICK_AXIS_Y] ?? 0;
    fsm.setLinear(applyDeadzone(ty));
    fsm.setAngular(applyDeadzone(-tx));
  }

  // Проверка B-кнопки (button index 5).
  if (source.gamepad?.buttons[EMERGENCY_GAMEPAD_BUTTON_INDEX]?.pressed) {
    state.emergencyEdge = true;
  }

  state.linear = fsm.getState() === "idle" ? 0 : (source.gamepad?.axes[THUMBSTICK_AXIS_Y] ?? 0);
  state.angular = fsm.getState() === "idle" ? 0 : (source.gamepad?.axes[THUMBSTICK_AXIS_X] ?? 0);
  state.deadman = (source.gamepad?.buttons[DEADMAN_GAMEPAD_BUTTON_INDEX]?.value ?? 0) > 0.5;
}

function applyDeadzone(v: number): number {
  if (Math.abs(v) < DEADZONE) return 0;
  // Re-scale outside the deadzone.
  const sign = v < 0 ? -1 : 1;
  return sign * ((Math.abs(v) - DEADZONE) / (1 - DEADZONE));
}

export function createXrTeleop(opts: { fsm: TeleopFSM; session: XRSession }): XrTeleopHandle {
  const state: XrInputState = { linear: 0, angular: 0, deadman: false, emergencyEdge: false };
  const handler = (_ev: XRInputSourcesChangeEvent) => {
    // Можно было бы переподписаться, но проще поллить каждый кадр через
    // requestAnimationFrame — вызывающий код вызывает pollXrInput().
    void _ev;
  };
  opts.session.addEventListener("inputsourceschange", handler);
  return {
    destroy(): void {
      opts.session.removeEventListener("inputsourceschange", handler);
    },
    isActive(): boolean {
      return state.deadman;
    }
  };
}