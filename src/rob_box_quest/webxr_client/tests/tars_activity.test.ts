import { describe, it, expect } from "vitest";
import {
  EMPTY_TARS_SIGNALS,
  TARS_TTL_MS,
  createTarsActivityTracker,
  deriveTarsActivity,
  tarsActivityView
} from "../src/state/tars_activity";

const T0 = 1_000_000;

describe("deriveTarsActivity — сигналы → состояние", () => {
  it("без сигналов — idle", () => {
    expect(deriveTarsActivity(EMPTY_TARS_SIGNALS, T0)).toBe("idle");
  });

  it("PTT зажат — listening", () => {
    expect(deriveTarsActivity({ ...EMPTY_TARS_SIGNALS, pttHeld: true }, T0)).toBe("listening");
  });

  it("voice_state listening/thinking мост → listening/thinking", () => {
    expect(
      deriveTarsActivity({ ...EMPTY_TARS_SIGNALS, voiceState: { state: "listening", atMs: T0 } }, T0 + 10)
    ).toBe("listening");
    expect(
      deriveTarsActivity({ ...EMPTY_TARS_SIGNALS, voiceState: { state: "thinking", atMs: T0 } }, T0 + 10)
    ).toBe("thinking");
  });

  it("voice_state speaking — голос робота, НЕ «ТАРС говорит»", () => {
    expect(
      deriveTarsActivity({ ...EMPTY_TARS_SIGNALS, voiceState: { state: "speaking", atMs: T0 } }, T0)
    ).toBe("idle");
  });

  it("tars_state accepted/thinking → thinking, speaking → speaking", () => {
    const at = (stage: string) => ({ ...EMPTY_TARS_SIGNALS, tarsStage: { stage, atMs: T0 } });
    expect(deriveTarsActivity(at("accepted"), T0)).toBe("thinking");
    expect(deriveTarsActivity(at("thinking"), T0)).toBe("thinking");
    expect(deriveTarsActivity(at("speaking"), T0)).toBe("speaking");
    expect(deriveTarsActivity(at("idle"), T0)).toBe("idle");
  });

  it("чанк operator_tts_audio держит speaking TTL и гаснет", () => {
    const s = { ...EMPTY_TARS_SIGNALS, ttsChunkAtMs: T0 };
    expect(deriveTarsActivity(s, T0 + TARS_TTL_MS.ttsChunk)).toBe("speaking");
    expect(deriveTarsActivity(s, T0 + TARS_TTL_MS.ttsChunk + 1)).toBe("idle");
  });

  it("tars1_text streaming → speaking; streaming=false → нет", () => {
    expect(
      deriveTarsActivity({ ...EMPTY_TARS_SIGNALS, tars1Stream: { on: true, atMs: T0 } }, T0 + 5)
    ).toBe("speaking");
    expect(
      deriveTarsActivity({ ...EMPTY_TARS_SIGNALS, tars1Stream: { on: false, atMs: T0 } }, T0 + 5)
    ).toBe("idle");
  });

  it("приоритет: speaking > thinking > listening", () => {
    const all = {
      pttHeld: true,
      voiceState: { state: "thinking", atMs: T0 },
      tarsStage: { stage: "speaking", atMs: T0 },
      ttsChunkAtMs: null,
      tars1Stream: null
    };
    expect(deriveTarsActivity(all, T0)).toBe("speaking");
    expect(deriveTarsActivity({ ...all, tarsStage: null }, T0)).toBe("thinking");
    expect(deriveTarsActivity({ ...all, tarsStage: null, voiceState: null }, T0)).toBe("listening");
  });

  it("потерянный idle: thinking протухает по TTL, экран не застревает", () => {
    const s = { ...EMPTY_TARS_SIGNALS, tarsStage: { stage: "thinking", atMs: T0 } };
    expect(deriveTarsActivity(s, T0 + TARS_TTL_MS.thinking)).toBe("thinking");
    expect(deriveTarsActivity(s, T0 + TARS_TTL_MS.thinking + 1)).toBe("idle");
  });
});

describe("createTarsActivityTracker", () => {
  it("tars_state idle обрывает хвост звука", () => {
    const t = createTarsActivityTracker();
    t.noteTtsChunk(T0);
    expect(t.state(T0 + 100)).toBe("speaking");
    t.noteTarsStage("idle", T0 + 200);
    expect(t.state(T0 + 300)).toBe("idle");
  });

  it("сценарий: PTT → thinking → speaking → idle", () => {
    const t = createTarsActivityTracker();
    t.notePtt(true);
    expect(t.state(T0)).toBe("listening");
    t.notePtt(false);
    t.noteTarsStage("accepted", T0 + 1000);
    expect(t.state(T0 + 1001)).toBe("thinking");
    t.noteTtsChunk(T0 + 3000);
    expect(t.state(T0 + 3001)).toBe("speaking");
    t.noteTarsStage("idle", T0 + 6000);
    expect(t.state(T0 + 6001)).toBe("idle");
  });
});

describe("tarsActivityView", () => {
  it("лейблы состояний", () => {
    expect(tarsActivityView("listening", 0).label).toBe("LISTENING");
    expect(tarsActivityView("thinking", 0).label).toBe("THINKING");
    expect(tarsActivityView("speaking", 0).label).toBe("SPEAKING");
    expect(tarsActivityView("idle", 0).label).toBe("STANDBY");
  });

  it("thinking крутит спиннер, speaking даёт 5 столбиков в 0..1, listening пульсирует", () => {
    expect(tarsActivityView("thinking", 0).glyph).not.toBe(tarsActivityView("thinking", 130).glyph);
    const bars = tarsActivityView("speaking", 500).bars;
    expect(bars).toHaveLength(5);
    for (const b of bars) {
      expect(b).toBeGreaterThan(0);
      expect(b).toBeLessThanOrEqual(1);
    }
    const p = tarsActivityView("listening", 200).pulse;
    expect(p).toBeGreaterThanOrEqual(0.1);
    expect(p).toBeLessThanOrEqual(1);
    expect(tarsActivityView("idle", 0).bars).toEqual([]);
  });
});
