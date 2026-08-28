// tests/mute_controller.test.ts
import { describe, it, expect } from "vitest";
import { MuteController } from "../src/ui/mute_controller";

describe("MuteController", () => {
  it("starts unmuted by default", () => {
    expect(new MuteController().isMuted()).toBe(false);
  });

  it("respects initial state", () => {
    expect(new MuteController({ initial: true }).isMuted()).toBe(true);
  });

  it("toggle flips state", () => {
    const c = new MuteController();
    expect(c.toggle()).toBe(true); // became muted
    expect(c.isMuted()).toBe(true);
    expect(c.toggle()).toBe(true); // became unmuted
    expect(c.isMuted()).toBe(false);
  });

  it("setMuted is idempotent", () => {
    const c = new MuteController();
    expect(c.setMuted(false)).toBe(false);
    expect(c.setMuted(true)).toBe(true);
    expect(c.setMuted(true)).toBe(false);
  });

  it("listener fires on change only", () => {
    const c = new MuteController();
    let fires = 0;
    c.subscribe(() => (fires += 1));
    c.setMuted(false);
    expect(fires).toBe(0);
    c.setMuted(true);
    expect(fires).toBe(1);
  });

  it("press/release with hold < threshold does not toggle", () => {
    const c = new MuteController({ pressDurationMs: 100 });
    c.press();
    // симулируем «отпустили сразу» через явный release без ожидания.
    // Контракт: pressStartMs=null после release, но held=0 → no toggle.
    // Тут нет sleep — проверяем что немедленный release ничего не делает.
    // Чтобы реально проверить удержание, используем фейк Date.
    const origNow = Date.now;
    Date.now = () => 1000;
    c.release();
    Date.now = () => 1050; // 50ms удержания < 100ms
    // Сейчас pressStartMs уже null, второй release — no-op.
    expect(c.release()).toBe(false);
    expect(c.isMuted()).toBe(false);
    Date.now = origNow;
  });

  it("press/release with hold >= threshold toggles", async () => {
    const c = new MuteController({ pressDurationMs: 50 });
    const origNow = Date.now;
    Date.now = () => 2000;
    c.press();
    Date.now = () => 2200; // 200ms удержания
    expect(c.release()).toBe(true);
    expect(c.isMuted()).toBe(true);
    Date.now = origNow;
  });

  it("unsubscribe stops receiving", () => {
    const c = new MuteController();
    let fires = 0;
    const off = c.subscribe(() => (fires += 1));
    c.setMuted(true);
    off();
    c.setMuted(false);
    expect(fires).toBe(1);
  });

  it("multiple presses are coalesced (idempotent press)", () => {
    const c = new MuteController();
    c.press();
    c.press(); // double press без release
    // release() сбрасывает pressStartMs, затем следующий release() — no-op.
    expect(c.release()).toBe(false); // held=0
  });
});