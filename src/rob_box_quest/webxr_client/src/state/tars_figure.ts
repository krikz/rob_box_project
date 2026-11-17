// Персонаж ТАРС 1 (issue #3253, Ш1): четыре вертикальных «монолита» в духе
// ТАРС из «Интерстеллара». Чистая функция `(activity, nowMs) → поза`, без
// Three.js и DOM — тестируется без часов, как `tarsActivityView`
// (state/tars_activity.ts). Рисует позу панель (scene/tars1_text_panel.ts).

import { tarsActivityView, type TarsActivity } from "./tars_activity";

/** Сколько сегментов у персонажа. */
export const TARS_SEGMENTS = 4;

/** Период «дыхания» в покое, мс. */
export const TARS_BREATH_PERIOD_MS = 4000;

/** Один сегмент-монолит. Все величины безразмерные. */
export interface TarsSegmentPose {
  /** Сдвиг по горизонтали в ширинах сегмента (раздвигание колонн). */
  dx: number;
  /** Сдвиг по вертикали в долях полной высоты фигуры; минус = вверх. */
  dy: number;
  /** Высота сегмента, 0..1 от полной высоты фигуры. */
  h: number;
  /** Яркость/свечение, 0..1. */
  glow: number;
}

export type TarsFigurePose = TarsSegmentPose[];

const TWO_PI = Math.PI * 2;

/** Поза персонажа в момент nowMs. Детерминирована. */
export function tarsFigurePose(a: TarsActivity, nowMs: number): TarsFigurePose {
  const pose: TarsFigurePose = [];
  switch (a) {
    case "listening": {
      // Колонны слегка раздвигаются и светятся в такт pulse из tarsActivityView.
      const p = tarsActivityView("listening", nowMs).pulse;
      for (let i = 0; i < TARS_SEGMENTS; i += 1) {
        pose.push({ dx: (i - 1.5) * 0.35 * p, dy: 0, h: 0.9, glow: 0.45 + 0.55 * p });
      }
      return pose;
    }
    case "thinking": {
      // Перебор: сегменты по очереди приподнимаются волной, шаг 300 мс.
      const step = nowMs / 300;
      for (let i = 0; i < TARS_SEGMENTS; i += 1) {
        const ph = (((step - i) % TARS_SEGMENTS) + TARS_SEGMENTS) % TARS_SEGMENTS;
        const bump = ph < 1 ? Math.sin(Math.PI * ph) : 0;
        pose.push({ dx: 0, dy: -0.18 * bump, h: 0.9, glow: 0.4 + 0.6 * bump });
      }
      return pose;
    }
    case "speaking": {
      // Амплитуда эквалайзера (те же формулы, что у bars в tarsActivityView).
      for (let i = 0; i < TARS_SEGMENTS; i += 1) {
        const amp = 0.25 + 0.75 * Math.abs(Math.sin(nowMs / 180 + i * 1.7));
        pose.push({ dx: 0, dy: 0, h: 0.3 + 0.65 * amp, glow: 0.4 + 0.6 * amp });
      }
      return pose;
    }
    default: {
      // STANDBY: медленное «дыхание», едва заметное.
      for (let i = 0; i < TARS_SEGMENTS; i += 1) {
        const b = Math.sin((nowMs / TARS_BREATH_PERIOD_MS) * TWO_PI - i * 0.35);
        pose.push({ dx: 0, dy: 0.012 * b, h: 0.9 + 0.02 * b, glow: 0.32 + 0.14 * b });
      }
      return pose;
    }
  }
}
