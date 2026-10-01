import { describe, expect, it } from "vitest";
import { parseTarsStatus } from "../src/state/tars_status";
import { buildInfoRows, INFO_DASH } from "../src/state/tars1_layout";

describe("parseTarsStatus (#3253 Ш3)", () => {
  it("берёт tts/topic/nearby, остальное игнорирует", () => {
    const f = parseTarsStatus({
      type: "tars_status",
      tts: "minimax · voice1",
      topic: "DJ · рок",
      nearby: "Борис",
      ts_ms: 1,
      secret: "x"
    });
    expect(f).toEqual({ tts: "minimax · voice1", topic: "DJ · рок", nearby: "Борис" });
  });

  it("не tars_status и мусор → null", () => {
    expect(parseTarsStatus({ type: "tars1_text" })).toBeNull();
    expect(parseTarsStatus(null)).toBeNull();
    expect(parseTarsStatus("x")).toBeNull();
  });

  it("пустые и не-строки отбрасываются", () => {
    expect(parseTarsStatus({ type: "tars_status", tts: "  ", topic: 5, nearby: null })).toEqual({});
  });

  it("отсутствующие поля в панели — прочерк, llm и wake всегда прочерк", () => {
    const f = parseTarsStatus({ type: "tars_status", tts: "yandex" });
    const rows = buildInfoRows({ ...f });
    const m = new Map([...rows.service, ...rows.context].map((x) => [x.label, x.value]));
    expect(m.get("tts")).toBe("yandex");
    for (const k of ["llm", "wake", "тема", "рядом"]) expect(m.get(k)).toBe(INFO_DASH);
  });

  it("новый статус заменяет старый целиком: пропавшее поле снова прочерк", () => {
    const a = parseTarsStatus({ type: "tars_status", nearby: "Борис" });
    const b = parseTarsStatus({ type: "tars_status", tts: "yandex" });
    expect(a).toEqual({ nearby: "Борис" });
    expect(b).toEqual({ tts: "yandex" });
  });
});
