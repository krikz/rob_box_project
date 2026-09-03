// XR-контроллер → PointerRay.
//
// Луч берём из `targetRaySpace` кадра (это и есть штатный «указательный»
// луч контроллера, тот же, что рисует ray в сцене), нажатие — trigger
// (кнопка 0 oculus-touch-v2). Trigger свободен: grip'ы заняты голосом
// (рация / робот-голос), клик стика — arm/disarm, B/Y — emergency.
//
// Две вещи здесь сделаны против ложных срабатываний, которые оператор
// ловил на мостике:
//
//   1. **Активная рука, а не всегда правая.** Раньше `pickSource` жёстко
//      возвращала правый контроллер. Луч при этом рисовался на обоих, и
//      оператор, целясь левой рукой, нажимал её trigger — а клик уходил
//      туда, куда в этот момент смотрела правая, лежащая на колене. Теперь
//      «главной» становится рука, которая жмёт, и остаётся ей, пока не
//      нажмёт другая.
//   2. **Гистерезис на аналоговом trigger'е.** Порог был один (0.5) и в
//      обе стороны: рука с зажатым grip'ом (голосовой PTT) непроизвольно
//      подтягивает указательный палец, значение дребезжит вокруг порога, и
//      каждый переход читался как отдельный клик. Порог нажатия поднят, а
//      порог отпускания опущен — дребезг в зазор больше не проваливается.

import { GAMEPAD_BUTTONS } from "../input/teleop_config";
import type { PointerRay } from "./pointer";

/** Кнопка выбора: trigger (индекс 0 по oculus-touch-v2). */
export const POINTER_BUTTON = GAMEPAD_BUTTONS.trigger;

/**
 * Порог срабатывания trigger'а — он аналоговый (0..1).
 * Оставлен для обратной совместимости с `isPointerPressed` (одиночная
 * проверка без истории); для кадрового опроса используйте
 * `createPointerPressGate`, у которого порогов два.
 */
export const POINTER_PRESS_THRESHOLD = 0.5;

/** Гистерезис: нажатие засчитывается выше этого значения… */
export const POINTER_PRESS_ON_THRESHOLD = 0.65;
/** …а отпускание — только ниже этого. Зазор гасит дребезг. */
export const POINTER_PRESS_OFF_THRESHOLD = 0.3;

/** Сырое значение trigger'а (0..1); `pressed` от рантайма считаем за 1. */
export function pointerTriggerValue(source: XRInputSource): number {
  const gp = source.gamepad;
  if (!gp) return 0;
  const btn = gp.buttons[POINTER_BUTTON];
  if (!btn) return 0;
  if (btn.pressed) return 1;
  return typeof btn.value === "number" && Number.isFinite(btn.value) ? btn.value : 0;
}

export function isPointerPressed(source: XRInputSource): boolean {
  return pointerTriggerValue(source) > POINTER_PRESS_THRESHOLD;
}

/**
 * Гистерезисный фильтр нажатия. Хранит одно состояние (нажато/нет) и
 * переключает его только при выходе за соответствующий порог.
 */
export interface PointerPressGate {
  /** Обновить по значению trigger'а и вернуть текущее состояние. */
  update(value: number): boolean;
  /** Текущее состояние без обновления. */
  isPressed(): boolean;
  /** Сбросить (потеря трекинга, выход из VR). */
  reset(): void;
}

export function createPointerPressGate(): PointerPressGate {
  let pressed = false;
  return {
    update(value: number): boolean {
      if (pressed) {
        if (value < POINTER_PRESS_OFF_THRESHOLD) pressed = false;
      } else if (value >= POINTER_PRESS_ON_THRESHOLD) {
        pressed = true;
      }
      return pressed;
    },
    isPressed(): boolean {
      return pressed;
    },
    reset(): void {
      pressed = false;
    }
  };
}

/**
 * Правая рука приоритетнее левой; иначе — первый источник с gamepad.
 *
 * Это выбор «по умолчанию», без учёта того, кто сейчас жмёт: им пользуется
 * `xrPointerRay`, когда ни один trigger не тронут. Динамику активной руки
 * держит `createXrPointerSource` (см. ниже).
 */
export function pickSource(sources: readonly XRInputSource[]): XRInputSource | null {
  let fallback: XRInputSource | null = null;
  for (const s of sources) {
    if (!s.gamepad) continue;
    if (s.handedness === "right") return s;
    if (!fallback) fallback = s;
  }
  return fallback;
}

/**
 * Выбрать источник луча с учётом того, какая рука жмёт. Порядок:
 *   1. рука, чей trigger сейчас продавлен сильнее всего (если вообще тронут);
 *   2. `preferred` — рука, бывшая активной в прошлом кадре (липкость: луч
 *      не должен перескакивать между руками, пока оператор ничего не жмёт);
 *   3. `pickSource` — правая, затем любая с gamepad.
 */
export function pickActiveSource(
  sources: readonly XRInputSource[],
  preferred: XRInputSource | null
): XRInputSource | null {
  let best: XRInputSource | null = null;
  let bestValue = 0;
  for (const s of sources) {
    if (!s.gamepad) continue;
    const v = pointerTriggerValue(s);
    if (v > bestValue) {
      bestValue = v;
      best = s;
    }
  }
  // Любое касание trigger'а (не только полное нажатие) уже говорит, какой
  // рукой оператор целится — раньше, чем гистерезис засчитает клик.
  if (best && bestValue > 0.08) return best;
  if (preferred && sources.includes(preferred) && preferred.gamepad) return preferred;
  return pickSource(sources);
}

/**
 * Кадровый источник PointerRay: держит активную руку и её press-gate.
 *
 * Отдельный объект (а не свободная функция), потому что и «какая рука
 * главная», и гистерезис — это состояние между кадрами. Создаётся на вход
 * в VR-сессию, `reset()` — на выход.
 */
export interface XrPointerSource {
  /** Луч для этого кадра; `null` — трекинг потерян / контроллеров нет. */
  ray(frame: XRFrame, refSpace: XRReferenceSpace, sources: readonly XRInputSource[]): PointerRay | null;
  /** Активная рука прошлого кадра (для подсветки контроллера в сцене). */
  activeSource(): XRInputSource | null;
  reset(): void;
}

export function createXrPointerSource(): XrPointerSource {
  let active: XRInputSource | null = null;
  const gate = createPointerPressGate();

  return {
    ray(frame, refSpace, sources) {
      const source = pickActiveSource(sources, active);
      if (source !== active) {
        // Сменилась рука — нажатие старой не должно «дотечь» до новой.
        gate.reset();
        active = source;
      }
      if (!source || !source.targetRaySpace) return null;
      const pose = frame.getPose(source.targetRaySpace, refSpace);
      if (!pose) return null;
      const m = pose.transform.matrix; // column-major 4x4
      return {
        origin: { x: m[12], y: m[13], z: m[14] },
        // Направление указателя — -Z локальной оси targetRaySpace.
        direction: { x: -m[8], y: -m[9], z: -m[10] },
        pressed: gate.update(pointerTriggerValue(source))
      };
    },
    activeSource(): XRInputSource | null {
      return active;
    },
    reset(): void {
      active = null;
      gate.reset();
    }
  };
}

/**
 * Луч из XR-кадра без сохранения состояния между кадрами (правая рука,
 * одиночный порог). Оставлен для совместимости; интерактивный путь ходит
 * через `createXrPointerSource`.
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
  const m = pose.transform.matrix;
  return {
    origin: { x: m[12], y: m[13], z: m[14] },
    direction: { x: -m[8], y: -m[9], z: -m[10] },
    pressed: isPointerPressed(source)
  };
}
