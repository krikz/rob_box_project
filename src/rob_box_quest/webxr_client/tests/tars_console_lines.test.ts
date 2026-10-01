import { describe, expect, it } from "vitest";
import {
  CONSOLE_STYLES,
  ConsoleBuffer,
  parseConsoleEvent
} from "../src/state/tars_console_lines";

describe("parseConsoleEvent (#3253 Ш4)", () => {
  it("operator и event разбираются, пробелы схлопываются", () => {
    expect(parseConsoleEvent({ type: "tars_console", kind: "operator", text: "  включи   свет " })).toEqual({
      kind: "operator",
      text: "включи свет"
    });
    expect(parseConsoleEvent({ type: "tars_console", kind: "event", text: "tool: play_music", ts_ms: 1 })).toEqual({
      kind: "event",
      text: "tool: play_music"
    });
  });

  it("чужие типы, вид tars, пустой текст и мусор → null", () => {
    expect(parseConsoleEvent({ type: "tars1_text", text: "x" })).toBeNull();
    expect(parseConsoleEvent({ type: "tars_console", kind: "tars", text: "x" })).toBeNull();
    expect(parseConsoleEvent({ type: "tars_console", kind: "operator", text: "  " })).toBeNull();
    expect(parseConsoleEvent({ type: "tars_console", kind: "event", text: 5 })).toBeNull();
    expect(parseConsoleEvent(null)).toBeNull();
    expect(parseConsoleEvent("x")).toBeNull();
  });
});

describe("CONSOLE_STYLES", () => {
  it("у видов разные префиксы и цвета; ТАРС — как раньше «> » зелёным", () => {
    expect(CONSOLE_STYLES.tars.prefix).toBe("> ");
    expect(CONSOLE_STYLES.tars.textColor).toBe("#39ff88");
    expect(CONSOLE_STYLES.operator.prefix).toBe("you> ");
    expect(CONSOLE_STYLES.operator.textColor).toBe("#33e0ff");
    const colors = new Set(Object.values(CONSOLE_STYLES).map((s) => s.textColor));
    const prefixes = new Set(Object.values(CONSOLE_STYLES).map((s) => s.prefix));
    expect(colors.size).toBe(3);
    expect(prefixes.size).toBe(3);
  });
});

describe("ConsoleBuffer", () => {
  it("поток ТАРС склеивается в открытую строку, \\n закрывает", () => {
    const b = new ConsoleBuffer(32);
    b.appendTars("При");
    b.appendTars("вет\nкак ");
    b.appendTars("дела");
    expect(b.lines.map((l) => [l.kind, l.text])).toEqual([
      ["tars", "Привет"],
      ["tars", "как дела"]
    ]);
  });

  it("фраза оператора встаёт отдельной строкой; следующая реплика ТАРС — новой", () => {
    const b = new ConsoleBuffer(32);
    b.pushLine("operator", "включи музыку");
    b.appendTars("Включаю");
    b.pushLine("event", "tool: play_music");
    b.appendTars("Готово");
    expect(b.lines.map((l) => [l.kind, l.text])).toEqual([
      ["operator", "включи музыку"],
      ["tars", "Включаю"],
      ["event", "tool: play_music"],
      ["tars", "Готово"]
    ]);
  });

  it("недописанная реплика ТАРС не склеивается с фразой оператора", () => {
    const b = new ConsoleBuffer(32);
    b.appendTars("Слушаю");
    b.pushLine("operator", "стоп");
    b.appendTars("Ок");
    expect(b.lines.map((l) => l.text)).toEqual(["Слушаю", "стоп", "Ок"]);
  });

  it("кольцо: больше maxLines — старые уходят; clear возвращает пустую строку", () => {
    const b = new ConsoleBuffer(3);
    for (let i = 0; i < 6; i += 1) b.pushLine("operator", `фраза ${i}`);
    expect(b.length).toBe(3);
    expect(b.lines[0].text).toBe("фраза 3");
    b.clear();
    expect(b.length).toBe(1);
    expect(b.lines[0]).toEqual({ kind: "tars", text: "" });
  });

  it("пустые вставки — no-op", () => {
    const b = new ConsoleBuffer(8);
    b.appendTars("");
    b.pushLine("operator", "");
    expect(b.length).toBe(1);
  });
});
