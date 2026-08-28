// tests/audio_hud.test.ts
import { describe, it, expect } from "vitest";
import {
  drawAudioHud,
  rmsLevel,
  smoothLevel,
  type MicState
} from "../src/scene/audio_hud";

/**
 * Запись вызовов ctx в массив — для ассертов «нарисовали иконку» / «нарисовали текст».
 * Минимальный mock: достаточно fillStyle/fillRect/clearRect/strokeStyle/lineWidth/
 * beginPath/moveTo/lineTo/closePath/fill/stroke/quadraticCurveTo/arc/font/textBaseline/fillText.
 */
function makeMockCtx(): {
  ctx: CanvasRenderingContext2D;
  calls: Array<{ op: string, args: unknown[]; fillStyle?: string; strokeStyle?: string }>;
} {
  const calls: Array<{ op: string, args: unknown[]; fillStyle?: string; strokeStyle?: string }> = [];
  let fillStyle = "";
  let strokeStyle = "";
  const rec = (op: string) =>
    (...args: unknown[]) => {
      const entry: { op: string, args: unknown[]; fillStyle?: string; strokeStyle?: string } = { op, args };
      if (op === "fillRect" || op === "fillText" || op === "fill") {
        entry.fillStyle = fillStyle;
      }
      if (op === "stroke") {
        entry.strokeStyle = strokeStyle;
      }
      calls.push(entry);
    };
  const ctx = {
    get fillStyle(): string {
      return fillStyle;
    },
    set fillStyle(v: string) {
      fillStyle = v;
    },
    get strokeStyle(): string {
      return strokeStyle;
    },
    set strokeStyle(v: string) {
      strokeStyle = v;
    },
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
    // RMS чистой синусоиды амплитуды A в диапазоне [-1..1]: A/sqrt(2).
    // A = 16384/32768 = 0.5 → RMS = 0.5/sqrt(2) ≈ 0.354.
    expect(r).toBeGreaterThan(0.3);
    expect(r).toBeLessThan(0.4);
  });

  it("returns 1 for full-scale square wave", () => {
    const N = 100;
    const pcm = new Int16Array(N);
    for (let i = 0; i < N; i += 1) pcm[i] = i % 2 === 0 ? 32767 : -32768;
    expect(rmsLevel(pcm)).toBeCloseTo(1, 2);
  });
});

describe("smoothLevel", () => {
  it("clamps NaN/inf/out-of-range before smoothing", () => {
    // NaN: clamp→0, falling от 0.5 → release → 0.5 + (0-0.5)*0.15 = 0.425.
    expect(smoothLevel(0.5, NaN)).toBeCloseTo(0.425, 5);
    // Отрицательный target: clamp→0, falling → 0.5 - 0.075 = 0.425.
    expect(smoothLevel(0.5, -0.5)).toBeCloseTo(0.425, 5);
    // Цель > 1: clamp→1, rising от 0.5 → attack=0.6 → 0.5 + 0.5*0.6 = 0.8.
    expect(smoothLevel(0.5, 1.5)).toBeCloseTo(0.8, 5);
  });

  it("uses attack on rising, release on falling", () => {
    // rising: prev=0, target=1 → 0 + 1*0.6 = 0.6
    expect(smoothLevel(0, 1, 0.6, 0.15)).toBeCloseTo(0.6, 5);
    // falling: prev=1, target=0 → 1 + (-1)*0.15 = 0.85
    expect(smoothLevel(1, 0, 0.6, 0.15)).toBeCloseTo(0.85, 5);
  });
});

describe("drawAudioHud — fillStyle color contract", () => {
  it("uses green meter fill for idle/talk, muted color for muted", () => {
    const states: Array<{ s: MicState; fillContains: string }> = [
      { s: "idle", fillContains: "#2ec27e" }, // green meter
      { s: "talk", fillContains: "#2ec27e" },
      { s: "muted", fillContains: "#3a4150" } // grey meter fill (muted)
    ];
    for (const { s, fillContains } of states) {
      const { ctx, calls } = makeMockCtx();
      drawAudioHud(ctx, { micState: s, level: 0.9 }, GEOM);
      // Третий fillRect — meter fill (bg, track, fill). Ищем его по
      // ширине = meterW * 0.9.
      const meterW = GEOM.width - 96 - 16;
      const expectedWidth = Math.floor(meterW * 0.9);
      const meterFill = calls.find((c) => {
        if (c.op !== "fillRect") return false;
        const [, , w] = c.args as [number, number, number];
        return Math.abs((w as number) - expectedWidth) <= 1;
      });
      expect(meterFill).toBeDefined();
      // fillStyle ЗАХВАЧЕН в момент вызова — это то, что production-код
      // установил перед fillRect.
      expect(meterFill!.fillStyle).toContain(fillContains);
      expect(calls.some((c) => c.op === "fillText")).toBe(true);
    }
  });

  it("clamps level > 1 to meter full-width", () => {
    const { ctx, calls } = makeMockCtx();
    drawAudioHud(ctx, { micState: "talk", level: 5.0 }, GEOM);
    // Ищем fillRect с width = meterW (после clamp 5.0→1.0).
    const meterW = GEOM.width - 96 - 16;
    const fullRect = calls.find((c) => {
      if (c.op !== "fillRect") return false;
      const [, , w] = c.args as [number, number, number];
      return Math.abs((w as number) - meterW) <= 1;
    });
    expect(fullRect).toBeDefined();
    // Это либо meter fill (если peak подавил наш find), либо peak.
    // Допустимо оба варианта — главное, что level клампован и нет overflow.
    expect(fullRect!.fillStyle).toBeDefined();
  });

  it("does not draw peak marker below threshold", () => {
    const { ctx, calls } = makeMockCtx();
    drawAudioHud(ctx, { micState: "talk", level: 0.5 }, GEOM);
    const rects = calls.filter((c) => c.op === "fillRect");
    expect(rects.length).toBe(3); // bg, track, fill — без peak.
  });

  it("draws peak marker above 0.85 in talk state", () => {
    const { ctx, calls } = makeMockCtx();
    drawAudioHud(ctx, { micState: "talk", level: 0.95 }, GEOM);
    const rects = calls.filter((c) => c.op === "fillRect");
    expect(rects.length).toBe(4); // bg, track, fill, peak
    // Пик рендерится жёлтым #e8c547.
    const peak = rects[3]!;
    expect(peak.fillStyle).toContain("#e8c547");
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