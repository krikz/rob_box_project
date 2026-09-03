// Тесты FPS-счётчика (AV-25 / B4).
//
// Acceptance: чистая функция `fpsFromFrameTimes` + `FpsMeter`-класс.
// Сценарии:
//   - равномерный поток (60 кадров за 1 секунду → 60 fps);
//   - дропнутые кадры (между парой times большой интервал → FPS падает);
//   - пустой/одиночный массив → 0;
//   - окно (windowSize) учитывается в среднем;
//   - FpsMeter.shouldUpdate уважает intervalMs (fake-timers).

import { describe, it, expect, vi } from "vitest";
import { FpsMeter, fpsFromFrameTimes } from "../src/scene/fps_meter";

describe("fpsFromFrameTimes — чистая функция", () => {
  it("returns 0 for empty array", () => {
    expect(fpsFromFrameTimes([])).toBe(0);
  });

  it("returns 0 for single-element array", () => {
    expect(fpsFromFrameTimes([1000])).toBe(0);
  });

  it("returns 0 for arrays with only non-finite values", () => {
    expect(fpsFromFrameTimes([Infinity, NaN as unknown as number, NaN as unknown as number])).toBe(0);
  });

  it("returns 0 when intervals are non-positive", () => {
    expect(fpsFromFrameTimes([100, 100, 100])).toBe(0);
    expect(fpsFromFrameTimes([100, 50])).toBe(0);
  });

  it("computes correct fps for uniform 60 fps stream", () => {
    // 60 кадров за 1000мс = 60 fps.
    const times: number[] = [];
    for (let i = 0; i <= 60; i += 1) times.push(i * (1000 / 60));
    expect(fpsFromFrameTimes(times)).toBeCloseTo(60, 0);
  });

  it("computes correct fps for uniform 30 fps stream", () => {
    const times: number[] = [];
    for (let i = 0; i <= 30; i += 1) times.push(i * (1000 / 30));
    expect(fpsFromFrameTimes(times)).toBeCloseTo(30, 0);
  });

  it("fps drops when frames are dropped (jittery stream)", () => {
    // Чередование: 16мс, 16мс, 200мс (три дропа). Средний интервал ~77мс
    // → ~13 fps.
    const times = [0, 16, 32, 232, 248, 264, 464, 480, 496, 696];
    const fps = fpsFromFrameTimes(times);
    expect(fps).toBeGreaterThan(5);
    expect(fps).toBeLessThan(20);
  });

  it("respects maxFrames window (last N intervals only)", () => {
    // Первые 10 кадров с интервалом 100мс (10 fps), потом 60 кадров с
    // интервалом 16.67мс (60 fps). Без maxFrames среднее ≈ 36 fps.
    // С maxFrames=30 берём последние 30 интервалов → 60 fps.
    const times: number[] = [];
    for (let i = 0; i <= 10; i += 1) times.push(i * 100); // 10 интервалов
    for (let i = 1; i <= 60; i += 1) times.push(1000 + i * (1000 / 60));
    const all = fpsFromFrameTimes(times);
    expect(all).toBeGreaterThan(30);
    expect(all).toBeLessThan(50);
    const windowed = fpsFromFrameTimes(times, 30);
    expect(windowed).toBeGreaterThan(55);
  });

  it("treats large numbers as ms (default heuristic)", () => {
    // 50 кадров за 1000мс = 50 fps, ожидание при ms-шкале.
    const times: number[] = [];
    for (let i = 0; i <= 50; i += 1) times.push(i * 20);
    expect(fpsFromFrameTimes(times)).toBeCloseTo(50, 0);
  });
});

describe("FpsMeter — класс", () => {
  it("starts empty and returns 0", () => {
    const m = new FpsMeter();
    expect(m.size()).toBe(0);
    expect(m.value()).toBe(0);
  });

  it("push + value accumulates frames and slides window", () => {
    const m = new FpsMeter({ windowSize: 5 });
    m.push(0);
    m.push(20); // 50 fps интервал
    m.push(40);
    m.push(60);
    m.push(80);
    m.push(100);
    expect(m.size()).toBe(5); // окно не растёт за windowSize
    expect(m.value()).toBeCloseTo(50, 0);
  });

  it("ignores non-finite pushes", () => {
    const m = new FpsMeter();
    m.push(NaN);
    m.push(Infinity);
    m.push(0);
    m.push(20);
    expect(m.size()).toBe(2);
    expect(m.value()).toBeCloseTo(50, 0);
  });

  it("reset() clears state", () => {
    const m = new FpsMeter();
    m.push(0);
    m.push(20);
    m.reset();
    expect(m.size()).toBe(0);
    expect(m.value()).toBe(0);
  });

  it("shouldUpdate respects intervalMs (fake-timers)", () => {
    vi.useFakeTimers();
    try {
      const m = new FpsMeter();
      // До первого update — всегда true (lastUpdateMs === 0).
      expect(m.shouldUpdate(500, 1000)).toBe(true);
      m.markUpdated(1000);
      // Сразу после markUpdated — false.
      expect(m.shouldUpdate(500, 1100)).toBe(false);
      // Через 499мс — false.
      expect(m.shouldUpdate(500, 1499)).toBe(false);
      // Через 500мс — true.
      expect(m.shouldUpdate(500, 1500)).toBe(true);
      m.markUpdated(1500);
      // Снова false.
      expect(m.shouldUpdate(500, 1999)).toBe(false);
    } finally {
      vi.useRealTimers();
    }
  });
});
