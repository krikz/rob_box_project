// tests/mode_switcher.test.ts
import { describe, it, expect } from "vitest";
import {
  ModeSwitcher,
  BRIDGE_MODES,
  modeFromKey,
  modeLabel,
  teleopEnabled,
  voiceEnabled,
  type BridgeMode
} from "../src/ui/mode_switcher";

describe("ModeSwitcher", () => {
  it("starts in explore by default", () => {
    const s = new ModeSwitcher();
    expect(s.current()).toBe("explore");
  });

  it("respects initial mode", () => {
    const s = new ModeSwitcher({ initial: "mixed" });
    expect(s.current()).toBe("mixed");
  });

  it("setMode is idempotent on same value (no listener fire)", () => {
    const s = new ModeSwitcher();
    let fires = 0;
    s.subscribe(() => (fires += 1));
    expect(s.setMode("explore")).toBe(false);
    expect(fires).toBe(0);
    expect(s.setMode("teleop")).toBe(true);
    expect(fires).toBe(1);
    expect(s.setMode("teleop")).toBe(false);
    expect(fires).toBe(1);
  });

  it("cycle walks explore → teleop → voice → mixed → explore", () => {
    const s = new ModeSwitcher();
    expect(s.cycle()).toBe("teleop");
    expect(s.cycle()).toBe("voice");
    expect(s.cycle()).toBe("mixed");
    expect(s.cycle()).toBe("explore");
  });

  it("subscribe receives (next, prev)", () => {
    const s = new ModeSwitcher();
    const events: Array<[BridgeMode, BridgeMode]> = [];
    s.subscribe((next, prev) => events.push([next, prev]));
    s.setMode("teleop");
    s.setMode("mixed");
    expect(events).toEqual([
      ["teleop", "explore"],
      ["mixed", "teleop"]
    ]);
  });

  it("unsubscribe stops receiving events", () => {
    const s = new ModeSwitcher();
    let fires = 0;
    const off = s.subscribe(() => (fires += 1));
    s.setMode("teleop");
    off();
    s.setMode("voice");
    expect(fires).toBe(1);
  });

  it("shouldEmitTeleop/shouldEmitVoice match filter helpers", () => {
    const s = new ModeSwitcher();
    const cases: Array<[BridgeMode, boolean, boolean]> = [
      ["explore", false, true], // voice (radio) works, teleop off
      ["teleop", true, true],
      ["voice", false, true],
      ["mixed", true, true]
    ];
    for (const [mode, teleop, voice] of cases) {
      s.setMode(mode);
      expect(s.shouldEmitTeleop()).toBe(teleop);
      expect(s.shouldEmitVoice()).toBe(voice);
    }
  });
});

describe("mode filter helpers", () => {
  it("teleopEnabled", () => {
    expect(teleopEnabled("explore")).toBe(false);
    expect(teleopEnabled("teleop")).toBe(true);
    expect(teleopEnabled("voice")).toBe(false);
    expect(teleopEnabled("mixed")).toBe(true);
  });

  it("voiceEnabled — passthrough works in all modes (radio intercom is always allowed)", () => {
    expect(voiceEnabled("explore")).toBe(true);
    expect(voiceEnabled("teleop")).toBe(true);
    expect(voiceEnabled("voice")).toBe(true);
    expect(voiceEnabled("mixed")).toBe(true);
  });
});

describe("modeFromKey", () => {
  it("maps 1-4 to modes, ignores other keys", () => {
    expect(modeFromKey("1")).toBe("explore");
    expect(modeFromKey("2")).toBe("teleop");
    expect(modeFromKey("3")).toBe("voice");
    expect(modeFromKey("4")).toBe("mixed");
    expect(modeFromKey("5")).toBeNull();
    expect(modeFromKey("q")).toBeNull();
    expect(modeFromKey("")).toBeNull();
  });
});

describe("modeLabel", () => {
  it("returns uppercase label", () => {
    expect(modeLabel("explore")).toBe("EXPLORE");
    expect(modeLabel("teleop")).toBe("TELEOP");
    expect(modeLabel("voice")).toBe("VOICE");
    expect(modeLabel("mixed")).toBe("MIXED");
  });
});

describe("BRIDGE_MODES order", () => {
  it("matches HUD layout (explore, teleop, voice, mixed)", () => {
    expect(BRIDGE_MODES).toEqual(["explore", "teleop", "voice", "mixed"]);
  });
});