// vad_gate: RMS-гейт + hangover (ADR-0054, issue #1992, шаг 5a-impl).
//
// DoD требует минимум 2 теста:
//   1. silence → гейт закрыт, чанки НЕ отправляются;
//   2. voice → чанки отправляются И hangover 200мс удерживает гейт на
//      короткой паузе между словами (а не рвёт фразу).
//
// Дополнительно: rmsInt16 отдельно (формула RMS), детерминированные
// переходы состояний, защита от регрессий по hangoverChunks.

import { describe, it, expect } from "vitest";
import {
  createVadGateState,
  rmsInt16,
  shouldSendChunk,
  DEFAULT_VAD_HANGOVER,
  DEFAULT_VAD_THRESHOLD
} from "../src/input/vad_gate";

// Размер чанка как у voice_capture.ts (20 мс @ 16 кГц = 320 семплов).
// Дублируем тут, чтобы тест можно было читать без импорта input-модуля
// (иначе vitest warning о cyclic). Значение — контракт ADR-0054.
const VOICE_CHUNK_SAMPLES = 320;

describe("rmsInt16", () => {
  it("RMS пустого чанка = 0 (защита от деления на ноль)", () => {
    expect(rmsInt16(new Int16Array(0))).toBe(0);
  });

  it("RMS константного сигнала = его амплитуда", () => {
    // Заполняем 320 семплов значением 1000 → RMS тоже 1000 (среднее квадрата
    // константы = квадрат константы, sqrt даёт константу).
    const pcm = new Int16Array(320).fill(1000);
    expect(rmsInt16(pcm)).toBe(1000);
  });

  it("RMS sin-образного сигнала ≈ 0.707 × амплитуды", () => {
    // sin → среднеквадратичное = a / sqrt(2). Проверяем только целую грубую
    // границу — вычисление с плавающей точкой даст крошечную погрешность.
    const n = 320;
    const amp = 1000;
    const pcm = new Int16Array(n);
    for (let i = 0; i < n; i++) {
      pcm[i] = Math.round(amp * Math.sin((2 * Math.PI * i) / 16));
    }
    const rms = rmsInt16(pcm);
    expect(rms).toBeGreaterThan(700);
    expect(rms).toBeLessThan(710);
  });
});

describe("createVadGateState", () => {
  it("дефолты: threshold=300, hangover=10 чанков (= 200 мс @ 20мс/чанк)", () => {
    const s = createVadGateState();
    expect(s.threshold).toBe(DEFAULT_VAD_THRESHOLD);
    expect(s.hangoverChunks).toBe(DEFAULT_VAD_HANGOVER);
    expect(DEFAULT_VAD_HANGOVER * 20).toBe(200); // контракт ADR-0054
    expect(s.hangover).toBe(0);
  });

  it("можно переопределить threshold и hangover", () => {
    const s = createVadGateState({ threshold: 500, hangoverChunks: 5 });
    expect(s.threshold).toBe(500);
    expect(s.hangoverChunks).toBe(5);
  });
});

describe("shouldSendChunk (DoD ADR-0054)", () => {
  it("DoD #1: silence → гейт закрыт, чанки НЕ отправляются", () => {
    let state = createVadGateState();
    // 50 чанков чистого нуля (= 1 секунда тишины).
    for (let i = 0; i < 50; i++) {
      const pcm = new Int16Array(VOICE_CHUNK_SAMPLES); // все нули
      const r = shouldSendChunk(state, pcm);
      expect(r.send).toBe(false);
      state = r.next;
    }
    // Hangover не должен был «накопиться» из ничего.
    expect(state.hangover).toBe(0);
  });

  it("DoD #2: voice → чанки отправляются + hangover 200мс держит гейт на короткой паузе", () => {
    let state = createVadGateState();
    const loud = new Int16Array(VOICE_CHUNK_SAMPLES).fill(2000); // RMS=2000 >> 300
    const silence = new Int16Array(VOICE_CHUNK_SAMPLES); // RMS=0

    // 1. Голос 5 чанков (= 100 мс).
    for (let i = 0; i < 5; i++) {
      const r = shouldSendChunk(state, loud);
      expect(r.send).toBe(true);
      state = r.next;
    }
    // После каждого loud-чанка hangover перезапускается на полное окно (10).
    expect(state.hangover).toBe(DEFAULT_VAD_HANGOVER);

    // 2. Пауза 8 чанков (= 160 мс) < 200 мс hangover: ВСЕ должны пройти.
    for (let i = 0; i < 8; i++) {
      const r = shouldSendChunk(state, silence);
      expect(r.send).toBe(true);
      state = r.next;
    }
    // Осталось 10 - 8 = 2 чанка hangover.
    expect(state.hangover).toBe(2);

    // 3. Ещё 3 тихих чанка: первые 2 пройдут (hangover=2,1 → 1,0),
    //    3-й уже НЕ пройдёт (hangover=0).
    for (let i = 0; i < 2; i++) {
      const r = shouldSendChunk(state, silence);
      expect(r.send).toBe(true);
      state = r.next;
    }
    expect(state.hangover).toBe(0);
    const finalR = shouldSendChunk(state, silence);
    expect(finalR.send).toBe(false); // гейт закрыт
    state = finalR.next;
    expect(state.hangover).toBe(0);
  });

  it("новый loud-чанк перезапускает hangover на полное окно (не декрементирует)", () => {
    let state = createVadGateState();
    const loud = new Int16Array(VOICE_CHUNK_SAMPLES).fill(2000);
    const silence = new Int16Array(VOICE_CHUNK_SAMPLES);

    // Открыли гейт, потратили половину hangover.
    state = shouldSendChunk(state, loud).next;
    for (let i = 0; i < 5; i++) {
      state = shouldSendChunk(state, silence).next;
    }
    expect(state.hangover).toBe(5); // 10 - 5

    // Новый loud — hangover снова 10.
    state = shouldSendChunk(state, loud).next;
    expect(state.hangover).toBe(DEFAULT_VAD_HANGOVER);
  });

  it("пустой чанк (0 семплов) не меняет state (дроп, hangover не трогаем)", () => {
    let state = createVadGateState({ hangoverChunks: 3 });
    state = { ...state, hangover: 2 };
    const r = shouldSendChunk(state, new Int16Array(0));
    expect(r.send).toBe(false);
    expect(r.next.hangover).toBe(2); // НЕ обнулили
  });

  it("порог: чанк чуть ниже порога считается тишиной", () => {
    let state = createVadGateState({ threshold: 1000, hangoverChunks: 0 });
    // RMS = 999 < 1000 → drop. Hangover=0 не спасёт.
    const pcm = new Int16Array(VOICE_CHUNK_SAMPLES).fill(999);
    const r = shouldSendChunk(state, pcm);
    expect(r.send).toBe(false);
    expect(r.next.hangover).toBe(0);
  });

  it("порог: чанк ровно на пороге — голос (>=, не строго больше)", () => {
    let state = createVadGateState({ threshold: 1000, hangoverChunks: 0 });
    const pcm = new Int16Array(VOICE_CHUNK_SAMPLES).fill(1000);
    const r = shouldSendChunk(state, pcm);
    expect(r.send).toBe(true);
    // Hangover должен был перезапуститься на hangoverChunks (0 тут).
    expect(r.next.hangover).toBe(0);
  });
});
