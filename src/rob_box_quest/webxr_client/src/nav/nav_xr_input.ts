// Кнопка прицела nav-цели в VR (issue #3151): A/X — на любой руке.
//
// Раскладка контроллеров мостика (teleop_config.ts): trigger — указатель,
// grip'ы — голос, стик — телеоп и ARM, B/Y — аварийный стоп. Свободна A/X
// (индекс 4 oculus-touch-v2) — ей и взводится прицел. Любая рука: оператор
// целится той рукой, которой удобно, и взводит той же.

import { GAMEPAD_BUTTONS } from "../input/teleop_config";

export const NAV_AIM_BUTTON = GAMEPAD_BUTTONS.aX;

interface GamepadLike {
  buttons: ReadonlyArray<{ pressed: boolean } | undefined>;
}

/** Зажата ли A/X хотя бы на одном контроллере. */
export function navAimPressed(sources: ReadonlyArray<{ gamepad?: GamepadLike | null }>): boolean {
  return sources.some((s) => s.gamepad?.buttons[NAV_AIM_BUTTON]?.pressed ?? false);
}

/** Детектор фронта: `true` ровно в кадр нажатия. */
export function createEdge(): (level: boolean) => boolean {
  let prev = false;
  return (level: boolean): boolean => {
    const rising = level && !prev;
    prev = level;
    return rising;
  };
}
