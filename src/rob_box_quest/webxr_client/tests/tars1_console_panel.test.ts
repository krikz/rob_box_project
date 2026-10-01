// Консоль ТАРС 1 в обе стороны (#3253 Ш4): виды строк в панели — префикс,
// цвет, typewriter только для реплик ТАРС. Canvas — запись fillText.

import { describe, expect, it } from "vitest";
import { createTars1TextPanel } from "../src/scene/tars1_text_panel";

function recordingPanel(opts: { typewriterCps?: number } = {}) {
  const texts: { t: string; color: string }[] = [];
  const ctx = {
    fillStyle: "",
    fillRect: () => {},
    fillText(this: { fillStyle: string }, t: string) {
      texts.push({ t, color: String(this.fillStyle) });
    },
    measureText: (t: string) => ({ width: t.length * 7 })
  };
  const orig = HTMLCanvasElement.prototype.getContext;
  HTMLCanvasElement.prototype.getContext = function () {
    return ctx as unknown as CanvasRenderingContext2D;
  } as unknown as typeof HTMLCanvasElement.prototype.getContext;
  try {
    const panel = createTars1TextPanel({ canvasWidth: 1280, canvasHeight: 720, ...opts });
    return { panel, texts };
  } finally {
    HTMLCanvasElement.prototype.getContext = orig;
  }
}

describe("tars1_text_panel — консоль в обе стороны (#3253 Ш4)", () => {
  it("appendLine(operator): «you> » и текст оператора — своим цветом", () => {
    const { panel, texts } = recordingPanel();
    panel.appendLine("operator", "включи музыку");
    expect(texts.filter((x) => x.t === "you> ").pop()?.color).toBe("#1b8ea6");
    expect(texts.filter((x) => x.t === "включи музыку").pop()?.color).toBe("#33e0ff");
  });

  it("реплика ТАРС — «> » и зелёный; событие — свой префикс и цвет", () => {
    const { panel, texts } = recordingPanel();
    panel.append("Привет");
    panel.appendLine("event", "tool: set_volume");
    expect(texts.filter((x) => x.t === "Привет").pop()?.color).toBe("#39ff88");
    expect(texts.filter((x) => x.t === "> ").pop()?.color).toBe("#1f9c55");
    expect(texts.filter((x) => x.t === "· ").pop()).toBeTruthy();
    expect(texts.filter((x) => x.t === "tool: set_volume").pop()?.color).toBe("#ffc94d");
  });

  it("typewriter только для ТАРС: оператор сразу, хвост ТАРС дописан мгновенно", () => {
    const { panel } = recordingPanel({ typewriterCps: 50 });
    panel.append("длинная реплика");
    expect(panel.getStats().pending).toBeGreaterThan(0);
    panel.appendLine("operator", "стоп");
    expect(panel.getStats().pending).toBe(0);
    expect(panel.getStats().lineCount).toBe(2);
    panel.appendLine("operator", "ещё");
    expect(panel.getStats().pending).toBe(0);
    expect(panel.getStats().lineCount).toBe(3);
  });

  it("пустая фраза — no-op; appendLine(tars) идёт через typewriter как append", () => {
    const { panel } = recordingPanel({ typewriterCps: 50 });
    panel.appendLine("operator", "   ");
    expect(panel.getStats().lineCount).toBe(1);
    panel.appendLine("tars", "ответ");
    expect(panel.getStats().pending).toBe(5);
  });
});
