// Quest XR controllers teleop (дизайн §6):
//   Thumbstick (любой) → linear.x / angular.z
//   Grip/squeeze (любой) → deadman=true
//   B/Y (любой)          → emergency stop
//
// Реализация через WebXR `input_sources` + Gamepad. pollXrInput() читает
// состояние одного XRInputSource; агрегацию по контроллерам и edge-trigger
// emergency делает main.ts. ВАЖНО: в immersive-vr window.requestAnimationFrame
// заморожен браузером, поэтому pollXrInput() вызывается из XR-кадрового
// цикла (session.requestAnimationFrame).
//
// Маппинг "xr-standard" профиля Quest Touch (Meta docs):
//   buttons: 0=trigger, 1=squeeze, 2=thumbstick press, 3=A/X, 4=B/Y, 5=thumbrest
//   axes:    0/1=touchpad, 2/3=thumbstick (x, y)

import type { TeleopFSM } from "./teleop_fsm";

export interface XrTeleopHandle {
  destroy(): void;
  isActive(): boolean;
}

// Кнопки gamepad "xr-standard" (Meta docs, Quest Touch):
//   0=trigger, 1=squeeze(grip), 2=thumbstick press, 3=A/X, 4=B/Y, 5=thumbrest.
const DEADMAN_GAMEPAD_BUTTON_INDEX = 1; // squeeze/grip
const EMERGENCY_GAMEPAD_BUTTON_INDEX = 4; // B/Y (5 = thumbrest — НЕ emergency)
const THUMBSTICK_AXIS_X = 2;
const THUMBSTICK_AXIS_Y = 3;
const DEADZONE = 0.12;

/** Результат поллинга одного XRInputSource (FSM не мутируем — агрегация в main.ts). */
export interface XrPollResult {
  linear: number; // -1..1 (уже с deadzone)
  angular: number; // -1..1 (уже с deadzone)
  deadman: boolean; // grip зажат
  emergency: boolean; // B/Y нажата (level; edge-trigger делает caller)
}

export function pollXrInput(source: XRInputSource): XrPollResult {
  const gp = source.gamepad;
  if (!gp) {
    return { linear: 0, angular: 0, deadman: false, emergency: false };
  }
  const deadman = (gp.buttons[DEADMAN_GAMEPAD_BUTTON_INDEX]?.value ?? 0) > 0.5;
  // thumbstick (axes 2/3 для стандартного Quest mapping).
  const tx = gp.axes[THUMBSTICK_AXIS_X] ?? 0;
  const ty = gp.axes[THUMBSTICK_AXIS_Y] ?? 0;
  return {
    linear: applyDeadzone(ty),
    angular: applyDeadzone(-tx),
    deadman,
    emergency: gp.buttons[EMERGENCY_GAMEPAD_BUTTON_INDEX]?.pressed ?? false
  };
}

function applyDeadzone(v: number): number {
  if (Math.abs(v) < DEADZONE) return 0;
  // Re-scale outside the deadzone.
  const sign = v < 0 ? -1 : 1;
  return sign * ((Math.abs(v) - DEADZONE) / (1 - DEADZONE));
}

export function createXrTeleop(opts: { fsm: TeleopFSM; session: XRSession }): XrTeleopHandle {
  // Поллинг и FSM-агрегацию делает main.ts в XR-кадровом цикле; здесь лишь
  // отслеживаем наличие контроллеров для isActive().
  let active = opts.session.inputSources.length > 0;
  const handler = (_ev: XRInputSourcesChangeEvent) => {
    void _ev;
    active = opts.session.inputSources.length > 0;
  };
  opts.session.addEventListener("inputsourceschange", handler);
  return {
    destroy(): void {
      opts.session.removeEventListener("inputsourceschange", handler);
      active = false;
    },
    isActive(): boolean {
      return active;
    }
  };
}