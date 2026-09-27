// pointer_beam + xr_pointer: чистая часть визуализации указателя и
// защиты от ложных срабатываний.
//
// Три вещи, из-за которых оператор «не видел луч и жал не туда»:
// длина луча, активная рука и дребезг аналогового trigger'а. WebGL в
// jsdom нет, поэтому проверяем ровно вычисления, а не отрисовку.

import { describe, it, expect } from "vitest";
import {
  BEAM_MISS_LENGTH_M,
  BEAM_MIN_LENGTH_M,
  beamLength,
  beamState
} from "../src/interaction/pointer_beam";
import {
  POINTER_BUTTON,
  POINTER_PRESS_OFF_THRESHOLD,
  POINTER_PRESS_ON_THRESHOLD,
  createPointerPressGate,
  pickActiveSource,
  pointerTriggerValue
} from "../src/interaction/xr_pointer";
import type { PointerHit } from "../src/interaction/pointer";

function hit(over: Partial<PointerHit> = {}): PointerHit {
  return { id: null, distanceM: null, point: null, pressed: false, ...over };
}

describe("beamLength", () => {
  it("попали — луч ровно до цели", () => {
    expect(beamLength(hit({ id: "vpl:stt", distanceM: 2.4 }))).toBeCloseTo(2.4);
  });

  it("не попали — фиксированная длина в пустоту", () => {
    expect(beamLength(hit())).toBe(BEAM_MISS_LENGTH_M);
  });

  it("длина в пустоту достаёт до самых дальних панелей (экран-стена на 3.9 м)", () => {
    expect(BEAM_MISS_LENGTH_M).toBeGreaterThan(3.9);
  });

  it("цель вплотную не схлопывает луч в точку", () => {
    expect(beamLength(hit({ id: "x", distanceM: 0 }))).toBe(BEAM_MIN_LENGTH_M);
  });

  it("битое расстояние не роняет отрисовку", () => {
    expect(beamLength(hit({ id: "x", distanceM: Number.NaN }))).toBe(BEAM_MISS_LENGTH_M);
  });
});

describe("beamState", () => {
  it("пусто / наведение / нажатие", () => {
    expect(beamState(hit())).toBe("miss");
    expect(beamState(hit({ id: "vpl:llm", distanceM: 2 }))).toBe("hover");
    expect(beamState(hit({ id: "vpl:llm", distanceM: 2, pressed: true }))).toBe("press");
  });
});

describe("гистерезис trigger'а", () => {
  const gate = () => createPointerPressGate();

  it("нажатие засчитывается только выше верхнего порога", () => {
    const g = gate();
    expect(g.update(POINTER_PRESS_ON_THRESHOLD - 0.01)).toBe(false);
    expect(g.update(POINTER_PRESS_ON_THRESHOLD)).toBe(true);
  });

  it("дребезг в зазоре не порождает второй клик", () => {
    const g = gate();
    g.update(0.9);
    // Палец «поплыл» — но пока значение выше нижнего порога, нажатие держится.
    for (const v of [0.5, 0.45, 0.6, 0.4]) {
      expect(g.update(v), `отпустило на ${v}`).toBe(true);
    }
    expect(g.update(POINTER_PRESS_OFF_THRESHOLD - 0.01)).toBe(false);
  });

  it("пороги действительно образуют зазор", () => {
    expect(POINTER_PRESS_ON_THRESHOLD).toBeGreaterThan(POINTER_PRESS_OFF_THRESHOLD);
  });

  it("reset снимает нажатие (выход из VR, потеря трекинга)", () => {
    const g = gate();
    g.update(1);
    g.reset();
    expect(g.isPressed()).toBe(false);
  });
});

describe("активная рука", () => {
  const src = (handedness: string, trigger = 0) => {
    const buttons = Array.from({ length: 6 }, () => ({ pressed: false, value: 0 }));
    buttons[POINTER_BUTTON] = { pressed: false, value: trigger };
    return { handedness, gamepad: { buttons, axes: [] } } as unknown as XRInputSource;
  };

  it("читает аналоговое значение trigger'а", () => {
    expect(pointerTriggerValue(src("right", 0.42))).toBeCloseTo(0.42);
    expect(pointerTriggerValue({ handedness: "left" } as XRInputSource)).toBe(0);
  });

  it("жмущая рука главнее правой по умолчанию", () => {
    const left = src("left", 0.8);
    const right = src("right", 0);
    expect(pickActiveSource([left, right], null)).toBe(left);
  });

  it("без нажатий держится за прошлую активную руку (луч не скачет)", () => {
    const left = src("left", 0);
    const right = src("right", 0);
    expect(pickActiveSource([left, right], left)).toBe(left);
  });

  it("без нажатий и без истории — правая, как раньше", () => {
    const left = src("left", 0);
    const right = src("right", 0);
    expect(pickActiveSource([left, right], null)).toBe(right);
  });

  it("исчезнувший контроллер не остаётся активным", () => {
    const gone = src("left", 0);
    const right = src("right", 0);
    expect(pickActiveSource([right], gone)).toBe(right);
  });
});
