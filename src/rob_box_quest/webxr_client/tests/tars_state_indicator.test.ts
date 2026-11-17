import { describe, it, expect } from "vitest";
import {
  formatTarsStatePresentation,
  normalizeTarsStage,
  parseTarsStateEvent
} from "../src/ui/tars_state_indicator";

describe("tars_state_indicator — парсинг и маппинг", () => {
  it("валидный payload парсится в TarsStateEvent", () => {
    const event = parseTarsStateEvent({
      type: "tars_state",
      stage: "accepted",
      request_id: "req-1",
      text: "ТАРС, ты здесь",
      ts_ms: 1234567
    });
    expect(event).toEqual({
      stage: "accepted",
      requestId: "req-1",
      text: "ТАРС, ты здесь",
      tsMs: 1234567
    });
  });

  it("payload без type=tars_state → null", () => {
    expect(parseTarsStateEvent({ type: "operator_tts_audio" })).toBeNull();
    expect(parseTarsStateEvent({})).toBeNull();
    expect(parseTarsStateEvent(null)).toBeNull();
    expect(parseTarsStateEvent("строка")).toBeNull();
  });

  it("неизвестный stage нормализуется в idle", () => {
    expect(normalizeTarsStage("garbage")).toBe("idle");
    expect(normalizeTarsStage(null)).toBe("idle");
    expect(normalizeTarsStage(123)).toBe("idle");
  });

  it("presentation: accepted — голубой ACCEPTED", () => {
    expect(formatTarsStatePresentation("accepted")).toMatchObject({
      label: "ACCEPTED",
      color: "#3b8eea",
      ariaText: "ТАРС: команда принята"
    });
  });

  it("presentation: speaking — зелёный SPEAKING", () => {
    expect(formatTarsStatePresentation("speaking")).toMatchObject({
      label: "SPEAKING",
      color: "#2ec27e"
    });
  });

  it("presentation: thinking — оранжевый THINKING", () => {
    expect(formatTarsStatePresentation("thinking")).toMatchObject({
      label: "THINKING",
      color: "#f5a623"
    });
  });

  it("presentation: idle — серый IDLE", () => {
    expect(formatTarsStatePresentation("idle")).toMatchObject({
      label: "IDLE",
      color: "#8b98a5"
    });
  });
});
