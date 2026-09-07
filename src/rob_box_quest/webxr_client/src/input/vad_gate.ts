// src/input/vad_gate.ts
//
// RMS VAD-гейт с hangover (ADR-0054 / issue #1992, шаг 5a-impl).
//
// Назначение: always-on wake-поток шлема НЕ должен отправлять на сервер
// фоновый шум (тишина, ветер, шум VR-охлаждения). Решение — гейт на
// клиенте: считаем RMS каждого чанка int16 PCM; выше порога →
// "голос есть" → чанк уходит в WS. Слабый hangover 200 мс удерживает
// гейт открытым, чтобы короткие паузы внутри фразы не рвали звук.
//
// Решение — ЧИСТАЯ ФУНКЦИЯ (без побочных эффектов, без часов, без I/O),
// чтобы её можно было покрыть детерминированными unit-тестами
// (tests/vad_gate.test.ts — минимум 2 теста, требует DoD ADR-0054):
//   1. silence не отправляется;
//   2. voice отправляется + hangover 200мс удерживает гейт на короткой паузе.
//
// ПОЧЕМУ RMS, А НЕ PEAK/zero-crossing: RMS даёт стабильную оценку мощности
// сигнала, не зависит от единичных пиков (шаги, щелчки контроллера).
// Типичный порог для шлема: RMS > 200..400 на int16 шкале (≈ −42..−36 dBFS).
// Конкретное значение (RMS_THRESHOLD = 300) — дефолт для Quest 3; оператор
// с шумным окружением может поднять через VoiceListenCmd.reason (forward-compat).
//
// Hangover: после того как гейт открылся, держим его открытым ещё
// HANGOVER_CHUNKS чанков (при шаге 20 мс и 10 чанках = 200 мс). Это
// классический VAD-приём: детектор «голоса» — пороговый, детектор «конца
// фразы» — hangover. Не используем энергетический буфер, потому что
// STT-движок сам решит, фраза это или нет, по более длинному окну.

export interface VadGateOptions {
  /** Порог RMS (int16 шкала, 0..32767). Дефолт 300 (~ −40 dBFS). */
  threshold?: number;
  /** Сколько чанков после последнего «голоса» держать гейт открытым. */
  hangoverChunks?: number;
}

export interface VadGateState {
  /** Чанков осталось в hangover-окне (0 = гейт сейчас закрыт). */
  hangover: number;
  /** Текущий порог (для UI/диагностики; immutable, конфиг). */
  threshold: number;
  /** Сколько чанков hangover-окна задано (для UI/диагностики). */
  hangoverChunks: number;
}

export const DEFAULT_VAD_THRESHOLD = 300;
export const DEFAULT_VAD_HANGOVER = 10; // 10 × 20 ms = 200 ms (ADR-0054)

/**
 * Создать начальное состояние гейта.
 * Используйте `Object.freeze`-обёртку на клиенте, если нужно иммутабельность.
 */
export function createVadGateState(opts: VadGateOptions = {}): VadGateState {
  return {
    hangover: 0,
    threshold: opts.threshold ?? DEFAULT_VAD_THRESHOLD,
    hangoverChunks: opts.hangoverChunks ?? DEFAULT_VAD_HANGOVER
  };
}

/**
 * RMS int16 PCM чанка (квадратный корень из среднего квадрата).
 *
 * int16 → делим на 32768 → float, считаем RMS, возвращаем в шкале
 * 0..32767. Это нужно, чтобы порог был в понятных единицах и сравнивался
 * с RMS напрямую (а не с float 0..1).
 */
export function rmsInt16(pcm: Int16Array): number {
  if (pcm.length === 0) return 0;
  // Sum-of-squares через double (overflow int32 на 32768²·320 ≈ 3.4e11 — ok).
  let sumSq = 0;
  for (let i = 0; i < pcm.length; i++) {
    const v = pcm[i];
    sumSq += v * v;
  }
  const mean = sumSq / pcm.length;
  // Math.sqrt можно заменить на приближение для горячего пути; в клиенте
  // wake это ~50 чанков/с — Math.sqrt дешевле, чем сложность приближения.
  return Math.sqrt(mean);
}

/**
 * Решение гейта по одному чанку: обновить state, вернуть «отправить?».
 *
 * Правила (ADR-0054 §3):
 *   - chunk RMS ≥ threshold → голос есть, гейт открыт, перезапуск hangover;
 *   - chunk RMS < threshold И hangover > 0 → тишина, но гейт ещё открыт
 *     (hangover обратный отсчёт), чанк ОТПРАВЛЯЕТСЯ;
 *   - chunk RMS < threshold И hangover == 0 → тишина, гейт закрыт,
 *     чанк НЕ отправляется (drop);
 *   - пустой чанк → drop, state не меняется (нет смысла крутить счётчик).
 *
 * Чистая функция: нет I/O, нет Date.now(). Состояние передаётся
 * параметром и возвращается. Вызывающий код (voice_capture.ts)
 * накапливает выходные чанки в своём буфере.
 */
export function shouldSendChunk(
  state: VadGateState,
  pcm: Int16Array
): { send: boolean; next: VadGateState } {
  if (pcm.length === 0) {
    // Пустой чанк (такого быть не должно — push() фильтрует, но защитимся):
    // дроп без изменения state. Это не сбрасывает hangover — корректно:
    // пустой чанк не должен «обнулять» уже открытый гейт.
    return { send: false, next: state };
  }

  const rms = rmsInt16(pcm);

  if (rms >= state.threshold) {
    // Голос. Гейт открыт, hangover перезапущен на полное окно.
    return {
      send: true,
      next: { ...state, hangover: state.hangoverChunks }
    };
  }

  // RMS ниже порога. Если есть hangover — гейт ещё открыт (слабый сигнал
  // в паузе между словами), шлём чанк и декрементируем.
  if (state.hangover > 0) {
    const nextHangover = state.hangover - 1;
    return {
      send: true,
      next: { ...state, hangover: nextHangover }
    };
  }

  // Тишина, гейт закрыт — drop.
  return { send: false, next: state };
}
