import { describe, it, expect } from "vitest";
import { bridgeHotkey } from "../src/input/bridge_hotkeys";
import { WALK_KEYS } from "../src/input/desktop_walk";

const ev = (key: string, code: string, mods: Record<string, boolean> = {}) => ({ key, code, ...mods });

describe("bridgeHotkey", () => {
  it("maps R / V / G / Shift+G / P", () => {
    expect(bridgeHotkey(ev("r", "KeyR"))).toBe("reset_layout");
    expect(bridgeHotkey(ev("V", "KeyV"))).toBe("tts_picker");
    expect(bridgeHotkey(ev("g", "KeyG"))).toBe("nav_aim");
    expect(bridgeHotkey(ev("G", "KeyG", { shiftKey: true }))).toBe("nav_cancel");
    expect(bridgeHotkey(ev("p", "KeyP"))).toBe("streams_panel");
  });

  it("G and P work in the Russian layout (by physical key)", () => {
    expect(bridgeHotkey(ev("п", "KeyG"))).toBe("nav_aim");
    expect(bridgeHotkey(ev("з", "KeyP"))).toBe("streams_panel");
  });

  it("P with Ctrl/Cmd/Alt is left to the browser (print etc.); autorepeat ignored", () => {
    expect(bridgeHotkey(ev("p", "KeyP", { ctrlKey: true }))).toBeNull();
    expect(bridgeHotkey(ev("p", "KeyP", { metaKey: true }))).toBeNull();
    expect(bridgeHotkey(ev("p", "KeyP", { altKey: true }))).toBeNull();
    expect(bridgeHotkey(ev("p", "KeyP", { repeat: true }))).toBeNull();
  });

  it("does not steal walk / teleop / other panel keys", () => {
    for (const code of WALK_KEYS) {
      const key = code.startsWith("Key") ? code.slice(3).toLowerCase() : "Shift";
      expect(bridgeHotkey(ev(key, code))).toBeNull();
    }
    for (const [key, code] of [
      ["e", "KeyE"],
      ["m", "KeyM"],
      ["h", "KeyH"],
      [" ", "Space"],
      ["ArrowUp", "ArrowUp"]
    ]) {
      expect(bridgeHotkey(ev(key, code))).toBeNull();
    }
  });
});
