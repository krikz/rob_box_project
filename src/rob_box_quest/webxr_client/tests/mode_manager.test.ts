import { describe, it, expect, beforeEach } from "vitest";
import {
  ModeManager,
  CAPTAIN_MODES,
  DEFAULT_MODE,
  type CaptainMode
} from "../src/modes/mode_manager";

describe("ModeManager", () => {
  let now: number;
  let mm: ModeManager;

  beforeEach(() => {
    now = 1000;
    mm = new ModeManager({ initial: "explore", now: () => now });
  });

  it("starts in DEFAULT_MODE (explore)", () => {
    const fresh = new ModeManager({ now: () => now });
    expect(fresh.getMode()).toBe(DEFAULT_MODE);
    expect(fresh.getMode()).toBe("explore");
  });

  it("exposes all 4 modes in CAPTAIN_MODES", () => {
    expect(CAPTAIN_MODES).toEqual(["explore", "teleop", "voice", "mixed"]);
  });

  it("requestMode transitions and notifies listener", () => {
    const events: { prev: CaptainMode; next: CaptainMode; reason: string }[] = [];
    mm.subscribe((e) => events.push({ prev: e.prev, next: e.next, reason: e.reason }));
    expect(mm.requestMode("teleop", "ui_select")).toBe(true);
    expect(mm.getMode()).toBe("teleop");
    expect(events).toEqual([{ prev: "explore", next: "teleop", reason: "ui_select" }]);
  });

  it("requestMode to same mode is no-op and returns false", () => {
    const events: unknown[] = [];
    mm.subscribe((e) => events.push(e));
    expect(mm.requestMode("explore")).toBe(false);
    expect(events).toHaveLength(0);
  });

  it("cycleNext walks through all modes and wraps around", () => {
    const visited: CaptainMode[] = [mm.getMode()];
    for (let i = 0; i < CAPTAIN_MODES.length; i += 1) {
      expect(mm.cycleNext("hotkey")).toBe(true);
      visited.push(mm.getMode());
    }
    expect(visited).toEqual(["explore", "teleop", "voice", "mixed", "explore"]);
  });

  it("reportTeleopIntent upgrades voice → mixed, no-op elsewhere", () => {
    mm.requestMode("voice", "ui_select");
    expect(mm.reportTeleopIntent()).toBe(true);
    expect(mm.getMode()).toBe("mixed");
    // idempotent
    expect(mm.reportTeleopIntent()).toBe(false);

    // В explore/teleop/mixed — no-op.
    mm.requestMode("explore", "ui_select");
    expect(mm.reportTeleopIntent()).toBe(false);
    expect(mm.getMode()).toBe("explore");

    mm.requestMode("teleop", "ui_select");
    expect(mm.reportTeleopIntent()).toBe(false);
    expect(mm.getMode()).toBe("teleop");
  });

  it("setVoiceActive(false) downgrades mixed → teleop", () => {
    mm.requestMode("teleop", "ui_select");
    mm.setVoiceActive(true);
    mm.requestMode("mixed", "ui_select"); // прямое переключение
    expect(mm.getMode()).toBe("mixed");
    expect(mm.setVoiceActive(false)).toBe(true);
    expect(mm.getMode()).toBe("teleop");
  });

  it("setVoiceActive(false) is no-op outside mixed", () => {
    mm.requestMode("voice", "ui_select");
    expect(mm.setVoiceActive(false)).toBe(false);
    expect(mm.getMode()).toBe("voice");
  });

  it("reportDeadmanReleased downgrades mixed → teleop when no voice", () => {
    mm.requestMode("teleop", "ui_select");
    mm.requestMode("mixed", "ui_select");
    expect(mm.reportDeadmanReleased()).toBe(true);
    expect(mm.getMode()).toBe("teleop");
  });

  it("reportDeadmanReleased is no-op when voice is active in mixed", () => {
    mm.requestMode("teleop", "ui_select");
    mm.setVoiceActive(true);
    mm.requestMode("mixed", "ui_select");
    expect(mm.reportDeadmanReleased()).toBe(false);
    expect(mm.getMode()).toBe("mixed");
  });

  it("subscribe returns an unsubscribe handle", () => {
    const events: unknown[] = [];
    const unsub = mm.subscribe((e) => events.push(e));
    mm.requestMode("teleop");
    expect(events).toHaveLength(1);
    unsub();
    mm.requestMode("voice");
    expect(events).toHaveLength(1);
  });

  it("listener throw does not break the FSM (other listeners still fire)", () => {
    let secondFired = false;
    mm.subscribe(() => {
      throw new Error("boom");
    });
    mm.subscribe(() => {
      secondFired = true;
    });
    mm.requestMode("teleop");
    expect(secondFired).toBe(true);
  });

  it("reset() returns to DEFAULT_MODE and clears flags", () => {
    mm.requestMode("teleop");
    mm.setVoiceActive(true);
    mm.reportTeleopIntent();
    mm.reset();
    expect(mm.getMode()).toBe(DEFAULT_MODE);
    expect(mm.isVoiceActive()).toBe(false);
    expect(mm.hasTeleopIntent()).toBe(false);
  });

  it("now() is called on each transition (for atMs)", () => {
    let calls = 0;
    const mm2 = new ModeManager({ now: () => (calls += 1) * 10 });
    const events: number[] = [];
    mm2.subscribe((e) => events.push(e.atMs));
    mm2.requestMode("teleop");
    expect(events).toEqual([10]);
    mm2.cycleNext();
    expect(events).toEqual([10, 20]);
  });
});