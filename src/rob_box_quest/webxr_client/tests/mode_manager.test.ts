import { describe, it, expect, vi } from "vitest";
import { createModeManager } from "../src/ui/mode_manager";

describe("createModeManager", () => {
  it("starts with default snapshot (no defaults)", () => {
    const mm = createModeManager();
    expect(mm.snapshot()).toEqual({
      voiceMode: "off",
      teleopState: "disarmed",
      currentVoice: null,
      currentPreset: null,
      currentLanguage: null
    });
  });

  it("uses defaults when no explicit initial preset/language is given", () => {
    const mm = createModeManager(undefined, {
      preset: "technical",
      language: "ru"
    });
    expect(mm.snapshot()).toEqual({
      voiceMode: "off",
      teleopState: "disarmed",
      currentVoice: null,
      currentPreset: "technical",
      currentLanguage: "ru"
    });
  });

  it("explicit initial overrides defaults (initial wins)", () => {
    const mm = createModeManager(
      { currentPreset: "lenin", currentLanguage: "en" },
      { preset: "technical", language: "ru" }
    );
    expect(mm.snapshot().currentPreset).toBe("lenin");
    expect(mm.snapshot().currentLanguage).toBe("en");
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

  it("setTeleopState / setCurrentVoice / setCurrentPreset / setCurrentLanguage update independent fields", () => {
    const mm = createModeManager();
    mm.setTeleopState("armed");
    mm.setCurrentVoice("alena");
    mm.setCurrentPreset("friendly");
    mm.setCurrentLanguage("en");
    expect(mm.snapshot()).toEqual({
      voiceMode: "off",
      teleopState: "armed",
      currentVoice: "alena",
      currentPreset: "friendly",
      currentLanguage: "en"
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

  it("reset() returns to initial+defaults (preset+language fall back to defaults)", () => {
    const mm = createModeManager(
      { voiceMode: "radio", currentVoice: "x" },
      { preset: "technical", language: "ru" }
    );
    const cb = vi.fn();
    mm.on(cb);
    mm.setCurrentPreset("lenin");
    mm.setCurrentLanguage("en");
    mm.reset();
    // После reset(): voiceMode/currentVoice — из initial (radio/x),
    // preset/language — из defaults (technical/ru), потому что initial
    // их не задавал.
    expect(mm.snapshot().voiceMode).toBe("radio");
    expect(mm.snapshot().currentVoice).toBe("x");
    expect(mm.snapshot().currentPreset).toBe("technical");
    expect(mm.snapshot().currentLanguage).toBe("ru");
    // 2 emit от setPreset + setLanguage + 1 от reset = 3.
    expect(cb).toHaveBeenCalledTimes(3);
  });

  it("reset() with no initial returns to defaults (full reset)", () => {
    const mm = createModeManager(undefined, {
      preset: "technical",
      language: "ru"
    });
    mm.setCurrentPreset("lenin");
    mm.setVoiceMode("robot_voice");
    mm.reset();
    expect(mm.snapshot().voiceMode).toBe("off");
    expect(mm.snapshot().currentVoice).toBeNull();
    expect(mm.snapshot().currentPreset).toBe("technical"); // default
    expect(mm.snapshot().currentLanguage).toBe("ru"); // default
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
      currentPreset: "whisper",
      currentLanguage: null
    });
  });

  it("does not fire listener when setCurrentLanguage value unchanged", () => {
    const mm = createModeManager(undefined, { language: "ru" });
    const cb = vi.fn();
    mm.on(cb);
    mm.setCurrentLanguage("ru"); // уже ru
    expect(cb).not.toHaveBeenCalled();
  });
});
