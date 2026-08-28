// tests/audio_hud.test.ts
import { describe, it, expect } from "vitest";
import {
  drawAudioHud,
  rmsLevel,
  smoothLevel,
  type AudioHudState,
  type MicState
} from "../src/scene/audio_hud";

/**
 * Запись вызовов ctx в массив — для ассертов «нарисовали иконку» / «нарисовали текст».
 * Минимальный mock: достаточно fillStyle/fillRect/clearRect/strokeStyle/lineWidth/
 * beginPath/moveTo/lineTo/closePath/fill/stroke/quadraticCurveTo/arc/font/textBaseline/fillText.
 */
function makeMockCtx(): {
  ctx: CanvasRenderingContext2D;
  calls: Array<{ op: string, args: unknown[] }>;
} {
  const calls: Array<{ op: string, args: unknown[] }> = [];
  const rec = (op: string) =>
    (...args: unknown[]) => {
      calls.push({ op, args });
    };
  // Часть аргументов — функции (quadraticCurveTo/arc) — мокнуть не нужно,
  // vitest/jsdom рисует имитацию; recordOp всё равно их зовёт.
  const ctx = {
    fillStyle: "",
    strokeStyle: "",
    lineWidth: 1,
    font: "",
    textBaseline: "",
    clearRect: rec("clearRect"),
    fillRect: rec("fillRect"),
    fillText: rec("fillText"),
    fill: rec("fill"),
    stroke: rec("stroke"),
    beginPath: rec("beginPath"),
    closePath: rec("closePath"),
    moveTo: rec("moveTo"),
    lineTo: rec("lineTo"),
    quadraticCurveTo: rec("quadraticCurveTo"),
    arc: rec("arc")
  } as unknown as CanvasRenderingContext2D;
  return { ctx, calls };
}

const GEOM = { width: 512, height: 96 };

describe("rmsLevel", () => {
  it("zero for empty array", () => {
    expect(rmsLevel(new Int16Array(0))).toBe(0);
  });

  it("zero for silence", () => {
    expect(rmsLevel(new Int16Array([0, 0, 0, 0]))).toBe(0);
  });

  it("computes RMS of sine-like samples", () => {
    const N = 1024;
    const pcm = new Int16Array(N);
    for (let i = 0; i < N; i += 1) {
      pcm[i] = Math.round(Math.sin((i / N) * Math.PI * 2) * 16384);
    }
    const r = rmsLevel(pcm);
    // RMS чистой синусоиды амплитуды A = A/sqrt(2). 16384 → ~11585.
    expect(r).toBeGreaterThan(0.4);
    expect(r).toBeLessThan(0.6);
  });

  it("returns 1 for full-scale square wave", () => {
    const N = 100;
    const pcm = new Int16Array(N);
    for (let i = 0; i < N; i += 1) pcm[i] = i % 2 === 0 ? 32767 : -32768;
    expect(rmsLevel(pcm)).toBeCloseTo(1, 2);
  });
});

describe("smoothLevel", () => {
  it("clamps NaN/inf to 0", () => {
    expect(smoothLevel(0.5, NaN)).toBeGreaterThanOrEqual(0);
    expect(smoothLevel(0.5, -0.5)).toBe(0.5);
    expect(smoothLevel(0.5, 1.5)).toBe(1);
  });

  it("uses attack on rising, release on falling", () => {
    // rising: prev=0, target=1 → 0 + 1*0.6 = 0.6
    expect(smoothLevel(0, 1, 0.6, 0.15)).toBeCloseTo(0.6, 5);
    // falling: prev=1, target=0 → 1 + (-1)*0.15 = 0.85
    expect(smoothLevel(1, 0, 0.6, 0.15)).toBeCloseTo(0.85, 5);
  });
});

describe("drawAudioHud — fillStyle color contract", () => {
  function lastFillStyleForRect(ctx: CanvasRenderingContext2D, calls: Array<{ op: string, args: unknown[] }>): string {
    let color = "";
    for (const c of calls) {
      if (c.op === "fillRect") color = ctx.fillStyle;
    }
    return color;
  }

  it("uses green meter fill for idle/talk, muted color for muted", () => {
    const states: Array<{ s: MicState, fillContains: string }> = [
      { s: "idle", fillContains: "rgb(46, 194, 126)" }, // #2ec27e
      { s: "talk", fillContains: "rgb(46, 194, 126)" },
      { s: "muted", fillContains: "rgb(58, 65, 80)" } // #3a4150
    ];
    for (const { s, fillContains } of states) {
      const { ctx, calls } = makeMockCtx();
      drawAudioHud(ctx, { micState: s, level: 0.9 }, GEOM);
      // Последний fillRect перед завершением — level fill (см. drawAudioHud).
      expect(ctx.fillStyle).toContain(fillContains);
      expect(calls.some((c) => c.op === "fillText")).toBe(true);
    }
  });

  it("clamps level > 1 to meter full-width", () => {
    const { ctx, calls } = makeMockCtx();
    drawAudioHud(ctx, { micState: "talk", level: 5.0 }, GEOM);
    // Найти fillRect с шириной ≈ meterW (после clearRect/fillRect(track)/fillRect(fill)).
    const rects = calls.filter((c) => c.op === "fillRect");
    expect(rects.length).toBeGreaterThanOrEqual(3); // bg, track, fill
    // Последний rect — level fill: width не превышает meterW.
    const last = rects[rects.length - 1];
    expect(last).toBeDefined();
    const [, , w] = last!.args as [number, number, number];
    const meterW = GEOM.width - 96 - 16;
    expect(w as number).toBeCloseTo(meterW, 1);
  });

  it("does not draw peak marker below threshold", () => {
    const { ctx, calls } = makeMockCtx();
    drawAudioHud(ctx, { micState: "talk", level: 0.5 }, GEOM);
    // Peak marker рисуется fillRect со стилем COLORS.meterPeak (#e8c547).
    // Не проверяем стиль, только что peak-rect не появился: ищем fillRect
    // ПОСЛЕ того, как fillStyle стал peak (если был бы peak — появился бы 4-й fillRect).
    const rects = calls.filter((c) => c.op === "fillRect");
    expect(rects.length).toBe(3); // bg, track, fill — без peak.
  });

  it("draws peak marker above 0.85 in talk state", () => {
    const { ctx, calls } = makeMockCtx();
    drawAudioHud(ctx, { micState: "talk", level: 0.95 }, GEOM);
    const rects = calls.filter((c) => c.op === "fillRect");
    expect(rects.length).toBe(4); // bg, track, fill, peak
  });

  it("does not draw peak marker in muted state even above 0.85", () => {
    const { ctx, calls } = makeMockCtx();
    drawAudioHud(ctx, { micState: "muted", level: 0.95 }, GEOM);
    const rects = calls.filter((c) => c.op === "fillRect");
    expect(rects.length).toBe(3);
  });
});

describe("drawAudioHud — label text contract", () => {
  it("renders IDLE/TALK/MUTED label via fillText", () => {
    for (const [s, label] of [
      ["idle", "IDLE"],
      ["talk", "TALK"],
      ["muted", "MUTED"]
    ] as Array<[MicState, string]>) {
      const { ctx, calls } = makeMockCtx();
      drawAudioHud(ctx, { micState: s, level: 0 }, GEOM);
      const textCalls = calls.filter((c) => c.op === "fillText");
      expect(textCalls.length).toBe(1);
      const text = textCalls[0]!.args[0] as string;
      expect(text).toBe(label);
    }
  });
});

describe("drawAudioHud — state variants render different mic icon", () => {
  it("muted icon adds a diagonal line (extra stroke)", () => {
    const idle = makeMockCtx();
    drawAudioHud(idle.ctx, { micState: "idle", level: 0 }, GEOM);
    const muted = makeMockCtx();
    drawAudioHud(muted.ctx, { micState: "muted", level: 0 }, GEOM);
    // muted добавляет диагональную черту (moveTo + lineTo + stroke) поверх иконки.
    const idleMoveTo = idle.calls.filter((c) => c.op === "moveTo").length;
    const mutedMoveTo = muted.calls.filter((c) => c.op === "moveTo").length;
    expect(mutedMoveTo).toBeGreaterThan(idleMoveTo);
  });
});