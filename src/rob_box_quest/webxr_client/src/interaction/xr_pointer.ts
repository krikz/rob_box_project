// XR-контроллер → PointerRay.
//
// Луч берём из `targetRaySpace` кадра (это и есть штатный «указательный»
// луч контроллера, тот же, что рисует ray в сцене), нажатие — trigger
// (кнопка 0 oculus-touch-v2). Trigger свободен: grip'ы заняты голосом
// (рация / робот-голос), клик стика — arm/disarm, B/Y — emergency.
//
// Правый контроллер имеет приоритет: если оператор целится обоими,
// «главной» считаем правую руку, как и в остальном UI мостика.

import { GAMEPAD_BUTTONS } from "../input/teleop_config";
import type { PointerRay } from "./pointer";

/** Кнопка выбора: trigger (индекс 0 по oculus-touch-v2). */
export const POINTER_BUTTON = GAMEPAD_BUTTONS.trigger;

/** Порог срабатывания trigger'а — он аналоговый (0..1). */
export const POINTER_PRESS_THRESHOLD = 0.5;

export function isPointerPressed(source: XRInputSource): boolean {
  const gp = source.gamepad;
  if (!gp) return false;
  const btn = gp.buttons[POINTER_BUTTON];
  if (!btn) return false;
  return btn.pressed || btn.value > POINTER_PRESS_THRESHOLD;
}

/**
 * Луч из XR-кадра. `null`, если у источника нет targetRaySpace или поза
 * в этом кадре недоступна (трекинг потерян) — вызывающий передаёт `null`
 * в PointerSystem, и наведение корректно снимается.
 */
export function xrPointerRay(
  frame: XRFrame,
  refSpace: XRReferenceSpace,
  sources: readonly XRInputSource[]
): PointerRay | null {
  const source = pickSource(sources);
  if (!source || !source.targetRaySpace) return null;
  const pose = frame.getPose(source.targetRaySpace, refSpace);
  if (!pose) return null;
  const m = pose.transform.matrix; // column-major 4x4
  // Направление указателя — -Z локальной оси targetRaySpace.
  return {
    origin: { x: m[12], y: m[13], z: m[14] },
    direction: { x: -m[8], y: -m[9], z: -m[10] },
    pressed: isPointerPressed(source)
  };
}

/** Правая рука приоритетнее левой; иначе — первый источник с gamepad. */
export function pickSource(sources: readonly XRInputSource[]): XRInputSource | null {
  let fallback: XRInputSource | null = null;
  for (const s of sources) {
    if (!s.gamepad) continue;
    if (s.handedness === "right") return s;
    if (!fallback) fallback = s;
  }
  return fallback;
}
