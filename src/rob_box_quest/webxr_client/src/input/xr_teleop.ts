// Quest XR controllers teleop (дизайн §6): thumbstick → движение, клик
// правого стика → arm/disarm (toggle), B/Y → emergency stop. Маппинг
// «какая кнопка за что отвечает» живёт в teleop_config.ts (DEFAULT_BINDINGS)
// — тут только чтение gamepad.
//
// pollXrInput() читает состояние одного XRInputSource; агрегацию по
// контроллерам и edge-trigger emergency делает main.ts. ВАЖНО: в immersive-vr
// window.requestAnimationFrame заморожен браузером, поэтому pollXrInput()
// вызывается из XR-кадрового цикла (session.requestAnimationFrame).

import type { TeleopFSM } from "./teleop_fsm";
import { DEFAULT_BINDINGS, type TeleopBindings } from "./teleop_config";

export interface XrTeleopHandle {
  destroy(): void;
  isActive(): boolean;
}

/** Результат поллинга одного XRInputSource (FSM не мутируем — агрегация в main.ts). */
export interface XrPollResult {
  linear: number; // -1..1 (уже с deadzone)
  angular: number; // -1..1 (уже с deadzone)
  armPress: boolean; // клик правого стика (level; toggle делает caller)
  emergency: boolean; // B/Y нажата (level; edge-trigger делает caller)
  ptt: boolean; // правый grip (рация): push-to-talk
  robotPtt: boolean; // левый grip (робот-голос): STT → LLM → TTS
}

export function pollXrInput(source: XRInputSource, bindings: TeleopBindings = DEFAULT_BINDINGS): XrPollResult {
  const gp = source.gamepad;
  if (!gp) {
    return { linear: 0, angular: 0, armPress: false, emergency: false, ptt: false, robotPtt: false };
  }
  const armPress =
    source.handedness === bindings.armHandedness &&
    (gp.buttons[bindings.armButton]?.pressed ?? false);
  const ptt =
    source.handedness === bindings.pttHandedness &&
    (gp.buttons[bindings.pttButton]?.value ?? 0) > 0.5;
  const robotPtt =
    source.handedness === bindings.robotPttHandedness &&
    (gp.buttons[bindings.robotPttButton]?.value ?? 0) > 0.5;
  const tx = gp.axes[bindings.angularAxis] ?? 0;
  const ty = gp.axes[bindings.linearAxis] ?? 0;
  return {
    linear: applySign(applyDeadzone(ty, bindings.deadzone), bindings.invertLinear),
    angular: applySign(applyDeadzone(tx, bindings.deadzone), bindings.invertAngular),
    armPress,
    emergency: gp.buttons[bindings.emergencyButton]?.pressed ?? false,
    ptt,
    robotPtt
  };
}

function applyDeadzone(v: number, deadzone: number): number {
  if (Math.abs(v) < deadzone) return 0;
  // Re-scale outside the deadzone.
  const sign = v < 0 ? -1 : 1;
  return sign * ((Math.abs(v) - deadzone) / (1 - deadzone));
}

function applySign(v: number, invert: boolean): number {
  if (v === 0) return 0; // избегаем -0 (Object.is различает)
  return invert ? -v : v;
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