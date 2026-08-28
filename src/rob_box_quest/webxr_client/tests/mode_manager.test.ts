import { describe, it, expect, vi } from "vitest";
import { createModeManager } from "../src/ui/mode_manager";

describe("createModeManager", () => {
  it("starts with default snapshot", () => {
    const mm = createModeManager();
    expect(mm.snapshot()).toEqual({
      voiceMode: "off",
      teleopState: "disarmed",
      currentVoice: null,
      currentPreset: null
    });
  });

  it("setVoiceMode updates snapshot and fires listener", () => {
    const mm = createModeManager();
    const cb = vi.fn();
    mm.on(cb);
    mm.setVoiceMode("robot_voice");
    expect(mm.snapshot().voiceMode).toBe("robot_voice");
    expect(cb).toHaveBeenCalledTimes(1);
    expect(cb.mock.calls[0][0].voiceMode).toBe("robot_voice");
  });

  it("does not fire listener when value unchanged", () => {
    const mm = createModeManager();
    const cb = vi.fn();
    mm.on(cb);
    mm.setVoiceMode("off"); // уже off
    expect(cb).not.toHaveBeenCalled();
  });

  it("setTeleopState / setCurrentVoice / setCurrentPreset update independent fields", () => {
    const mm = createModeManager();
    mm.setTeleopState("armed");
    mm.setCurrentVoice("alena");
    mm.setCurrentPreset("friendly");
    expect(mm.snapshot()).toEqual({
      voiceMode: "off",
      teleopState: "armed",
      currentVoice: "alena",
      currentPreset: "friendly"
    });
  });

  it("on() returns unsubscribe function", () => {
    const mm = createModeManager();
    const cb = vi.fn();
    const off = mm.on(cb);
    mm.setVoiceMode("radio");
    expect(cb).toHaveBeenCalledTimes(1);
    off();
    mm.setVoiceMode("off");
    expect(cb).toHaveBeenCalledTimes(1);
  });

  it("listener exception does not break other listeners", () => {
    const mm = createModeManager();
    const bad = vi.fn(() => {
      throw new Error("listener boom");
    });
    const good = vi.fn();
    mm.on(bad);
    mm.on(good);
    // Suppress expected console.warn from listener-throw handler.
    const warnSpy = vi.spyOn(console, "warn").mockImplementation(() => {});
    mm.setVoiceMode("robot_voice");
    expect(bad).toHaveBeenCalled();
    expect(good).toHaveBeenCalled();
    warnSpy.mockRestore();
  });

  it("snapshot() returns a copy (caller can't mutate internal state)", () => {
    const mm = createModeManager();
    const snap = mm.snapshot();
    snap.voiceMode = "robot_voice";
    // Внутреннее состояние не изменилось.
    expect(mm.snapshot().voiceMode).toBe("off");
  });

  it("reset() returns to defaults and fires listeners", () => {
    const mm = createModeManager({ voiceMode: "radio", currentVoice: "x" });
    const cb = vi.fn();
    mm.on(cb);
    mm.reset();
    expect(mm.snapshot().voiceMode).toBe("off");
    expect(mm.snapshot().currentVoice).toBeNull();
    expect(cb).toHaveBeenCalledTimes(1);
  });

  it("accepts partial initial snapshot", () => {
    const mm = createModeManager({
      teleopState: "armed",
      currentPreset: "whisper"
    });
    expect(mm.snapshot()).toEqual({
      voiceMode: "off",
      teleopState: "armed",
      currentVoice: null,
      currentPreset: "whisper"
    });
  });
});
