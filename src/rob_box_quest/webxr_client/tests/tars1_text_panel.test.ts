// Тесты для TARS 1 text panel — pure-логика: append / clear / streaming,
// кольцевой буфер, рендер canvas (smoke: canvas создан, текст записан).
//
// Three.js Mesh мы не проверяем — он уже покрыт в других тестах
// (status_hud.test.ts, voice_state_indicator.test.ts), и jsdom не даёт
// WebGL. Здесь — только контракт handle'а.
//
// ``HTMLCanvasElement.prototype.getContext`` в jsdom не реализован
// (см. jsdom/lib/jsdom/browser/not-implemented.js) — ставим минимальный
// mock, чтобы ``canvas.getContext('2d')`` возвращал объект-заглушку. Без
// этого createTars1TextPanel падает на первом же canvas-вызове (issue
// #2113, vitest-job fail был именно на этом).

import { describe, it, expect, beforeAll, beforeEach } from "vitest";
import {
  createTars1TextPanel,
  type Tars1TextPanelHandle
} from "../src/scene/tars1_text_panel";

beforeAll(() => {
  // jsdom: getContext не реализован по умолчанию. Заменяем на stub с
  // минимумом методов, которые дёргает наш canvas-рендер.
  const stubCtx = {
    fillStyle: "",
    font: "",
    textBaseline: "",
    fillRect: () => {},
    fillText: () => {},
    measureText: (text: string) => ({
      width: text.length * 7 // грубая оценка, для тестов достаточно
    }),
    clearRect: () => {},
    get fillStyle_(): string {
      return "";
    }
  } as unknown as CanvasRenderingContext2D;
  HTMLCanvasElement.prototype.getContext = function (
    _type: string
  ): CanvasRenderingContext2D | null {
    return stubCtx;
  };
});

describe("tars1_text_panel", () => {
  let panel: Tars1TextPanelHandle;

  beforeEach(() => {
    panel = createTars1TextPanel({ canvasWidth: 256, canvasHeight: 96 });
  });

  it("создаёт canvas и mesh", () => {
    expect(panel.mesh).toBeTruthy();
    expect(panel.getStats().streaming).toBe(false);
  });

  it("append дописывает текст (без \n — реплика одна строка)", () => {
    panel.append("Привет");
    panel.append(", как дела?");
    // Без \n — всё в одной строке; stats считает только зафиксированные
    // строки плюс «активную» (одна).
    const stats = panel.getStats();
    expect(stats.streaming).toBe(false);
    expect(stats.lineCount).toBe(1);
  });

  it("\\n закрывает строку, partial переходит в буфер", () => {
    panel.append("Первая строка\n");
    panel.append("Вторая строка\n");
    panel.append("Третья");
    const stats = panel.getStats();
    // 2 закрытых + 1 активная (хвост без \n)
    expect(stats.lineCount).toBe(3);
  });

  it("setStreaming включает/выключает индикатор", () => {
    panel.setStreaming(true);
    expect(panel.getStats().streaming).toBe(true);
    panel.setStreaming(false);
    expect(panel.getStats().streaming).toBe(false);
  });

  it("clear стирает буфер и сбрасывает streaming", () => {
    panel.append("foo\nbar\n");
    panel.setStreaming(true);
    panel.clear();
    const stats = panel.getStats();
    expect(stats.lineCount).toBe(1);
    expect(stats.streaming).toBe(false);
  });

  it("кольцевой буфер: больше maxLines — старые строки уходят", () => {
    const small = createTars1TextPanel({ maxLines: 3 });
    for (let i = 0; i < 10; i += 1) {
      small.append(`строка-${i}\n`);
    }
    // maxLines = 3 → активная строка (последняя без \n) + 2 предыдущих.
    // В нашей реализации 10 append'ов с \n → 10 строк → после shift'ов 3.
    expect(small.getStats().lineCount).toBeLessThanOrEqual(3);
  });

  it("dispose() освобождает canvas texture", () => {
    // Просто не должно бросать — Three.js-ресурсы сложно проверить в jsdom,
    // мы фиксируем только контракт «не падает».
    expect(() => panel.dispose()).not.toThrow();
  });

  it("пустой append — no-op", () => {
    const before = panel.getStats().lineCount;
    panel.append("");
    const after = panel.getStats().lineCount;
    expect(after).toBe(before);
  });
});