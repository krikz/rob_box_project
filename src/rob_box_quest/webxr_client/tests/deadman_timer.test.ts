import { describe, it, expect, beforeEach } from "vitest";
import {
  DeadmanTimer,
  DEADMAN_RELEASE_MS,
  DEADMAN_WARNING_MS
} from "../src/input/deadman_timer";

describe("DeadmanTimer", () => {
  let now: number;
  let timer: DeadmanTimer;

  beforeEach(() => {
    now = 1000;
    timer = new DeadmanTimer({ now: () => now });
  });

  it("default thresholds are 500ms release / 300ms warning", () => {
    expect(DEADMAN_RELEASE_MS).toBe(500);
    expect(DEADMAN_WARNING_MS).toBe(300);
  });

  it("check() returns null before any release", () => {
    expect(timer.check()).toBeNull();
    now += 1000;
    expect(timer.check()).toBeNull();
  });

  it("emits warning event at 300ms after release", () => {
    timer.gripPressed();
    now += 100;
    timer.gripReleased();
    now += 300;
    const ev = timer.check();
    expect(ev).not.toBeNull();
    expect(ev!.kind).toBe("warning");
    if (ev!.kind === "warning") {
      expect(ev!.remainingMs).toBeCloseTo(200);
    }
  });

  it("warning is emitted exactly once (not every frame after)", () => {
    timer.gripPressed();
    timer.gripReleased();
    now += 300;
    expect(timer.check()?.kind).toBe("warning");
    now += 50;
    expect(timer.check()).toBeNull();
    now += 50;
    expect(timer.check()).toBeNull();
  });

  it("emits triggered event at 500ms after release", () => {
    timer.gripPressed();
    now += 100;
    timer.gripReleased();
    now += 500;
    const ev = timer.check();
    expect(ev).not.toBeNull();
    expect(ev!.kind).toBe("triggered");
    if (ev!.kind === "triggered") {
      expect(ev!.elapsedMs).toBeGreaterThanOrEqual(500);
    }
    expect(timer.hasTriggered()).toBe(true);
  });

  it("does not emit duplicate triggered events", () => {
    timer.gripPressed();
    timer.gripReleased();
    now += 500;
    expect(timer.check()?.kind).toBe("triggered");
    expect(timer.check()).toBeNull();
    now += 200;
    expect(timer.check()).toBeNull();
  });

  it("gripPressed resets triggered state and prevents emergency", () => {
    timer.gripPressed();
    timer.gripReleased();
    now += 600;
    expect(timer.check()?.kind).toBe("triggered");
    // Пользователь нажал grip снова — флаг сброшен.
    timer.gripPressed();
    now += 100;
    expect(timer.check()).toBeNull();
    expect(timer.hasTriggered()).toBe(false);
  });

  it("reset() clears all state", () => {
    timer.gripPressed();
    timer.gripReleased();
    now += 600;
    timer.check();
    timer.reset();
    expect(timer.hasTriggered()).toBe(false);
    expect(timer.check()).toBeNull();
  });

  it("isGripHeld reflects held state when not yet triggered", () => {
    timer.gripPressed();
    expect(timer.isGripHeld()).toBe(true);
    timer.gripReleased();
    expect(timer.isGripHeld()).toBe(true); // пока не сработал — "держится" для FSM
    now += 600;
    timer.check();
    expect(timer.isGripHeld()).toBe(false);
  });

  it("elapsedSinceRelease is null before any release", () => {
    expect(timer.elapsedSinceRelease()).toBeNull();
  });

  it("elapsedSinceRelease grows linearly after release", () => {
    timer.gripPressed();
    now += 50;
    timer.gripReleased();
    now += 100;
    expect(timer.elapsedSinceRelease()).toBe(100);
    now += 50;
    expect(timer.elapsedSinceRelease()).toBe(150);
  });

  it("accepts custom thresholds", () => {
    const t = new DeadmanTimer({ now: () => now, releaseMs: 1000, warningMs: 500 });
    t.gripPressed();
    t.gripReleased();
    now += 600;
    expect(t.check()?.kind).toBe("warning");
    now += 400;
    expect(t.check()?.kind).toBe("triggered");
  });

  it("rejects warningMs > releaseMs at construction", () => {
    expect(() => new DeadmanTimer({ releaseMs: 100, warningMs: 200 })).toThrow(/warningMs/);
  });
});