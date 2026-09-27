// utterance_progress: «где сейчас моя реплика» — чистый редьюсер.
//
// Главное, что здесь проверяется, — что `idle` от моста читается
// по-разному в зависимости от того, что делал оператор: до реплики это
// покой, сразу после неё — «обрабатываю», после речи робота — «готово».
// Именно эта развилка и закрывает дыру, в которой оператор ждал вслепую.

import { describe, it, expect } from "vitest";
import {
  DONE_LINGER_MS,
  INITIAL_UTTERANCE_PROGRESS,
  STALL_TIMEOUT_MS,
  utteranceProgressReducer,
  utteranceProgressView,
  type UtteranceProgressAction,
  type UtteranceProgressState
} from "../src/state/utterance_progress";

function run(
  actions: UtteranceProgressAction[],
  from: UtteranceProgressState = INITIAL_UTTERANCE_PROGRESS
): UtteranceProgressState {
  return actions.reduce(utteranceProgressReducer, from);
}

const T0 = 1_000_000;

describe("полный путь реплики", () => {
  it("грипп → отпустил → обработка → речь → готово", () => {
    let s = run([{ kind: "ptt_start", atMs: T0 }]);
    expect(s.stage).toBe("recording");

    s = run([{ kind: "ptt_stop", atMs: T0 + 2000 }], s);
    expect(s.stage).toBe("sending");
    expect(s.sentAtMs).toBe(T0 + 2000);

    // Ключевая ветка: мост шлёт `idle`, но реплика уже ушла — значит
    // сейчас идёт распознавание/переписывание, а не покой.
    s = run([{ kind: "voice_state", state: "idle", detail: "none", atMs: T0 + 2300 }], s);
    expect(s.stage).toBe("thinking");

    s = run([{ kind: "voice_state", state: "speaking", detail: "none", atMs: T0 + 4000 }], s);
    expect(s.stage).toBe("speaking");

    s = run([{ kind: "voice_state", state: "idle", detail: "none", atMs: T0 + 7000 }], s);
    expect(s.stage).toBe("done");
  });

  it("«готово» гаснет само через DONE_LINGER_MS", () => {
    let s = run([
      { kind: "ptt_start", atMs: T0 },
      { kind: "ptt_stop", atMs: T0 + 1000 },
      { kind: "voice_state", state: "speaking", detail: "none", atMs: T0 + 2000 },
      { kind: "voice_state", state: "idle", detail: "none", atMs: T0 + 3000 }
    ]);
    expect(s.stage).toBe("done");
    s = run([{ kind: "tick", atMs: T0 + 3000 + DONE_LINGER_MS - 1 }], s);
    expect(s.stage).toBe("done");
    s = run([{ kind: "tick", atMs: T0 + 3000 + DONE_LINGER_MS }], s);
    expect(s.stage).toBe("idle");
  });
});

describe("idle до реплики — это покой, а не обработка", () => {
  it("мост шлёт idle, пока оператор ничего не говорил", () => {
    const s = run([{ kind: "voice_state", state: "idle", detail: "none", atMs: T0 }]);
    expect(s.stage).toBe("idle");
  });

  it("listening без гриппа (робот услышал сам) — не «обрабатываю»", () => {
    const s = run([{ kind: "voice_state", state: "listening", detail: "none", atMs: T0 }]);
    expect(s.stage).toBe("listening");
  });
});

describe("зависшая реплика", () => {
  it("молчание дольше STALL_TIMEOUT_MS → «нет ответа»", () => {
    let s = run([
      { kind: "ptt_start", atMs: T0 },
      { kind: "ptt_stop", atMs: T0 + 1000 }
    ]);
    s = run([{ kind: "tick", atMs: T0 + 1000 + STALL_TIMEOUT_MS - 1 }], s);
    expect(s.stage).toBe("sending");
    s = run([{ kind: "tick", atMs: T0 + 1000 + STALL_TIMEOUT_MS }], s);
    expect(s.stage).toBe("stalled");
  });

  it("таймаут отсчитывается от отправки, а не от смены стадии", () => {
    // Мост подтвердил обработку через 10 c; до порога остаётся ещё 2 c,
    // а не полные 12 — иначе «нет ответа» не наступит никогда, пока мост
    // изредка шлёт кадры.
    let s = run([
      { kind: "ptt_start", atMs: T0 },
      { kind: "ptt_stop", atMs: T0 + 1000 },
      { kind: "voice_state", state: "idle", detail: "none", atMs: T0 + 11000 }
    ]);
    expect(s.stage).toBe("thinking");
    s = run([{ kind: "tick", atMs: T0 + 1000 + STALL_TIMEOUT_MS }], s);
    expect(s.stage).toBe("stalled");
  });

  it("во время записи таймаута нет — оператор говорит сколько хочет", () => {
    let s = run([{ kind: "ptt_start", atMs: T0 }]);
    s = run([{ kind: "tick", atMs: T0 + 10 * STALL_TIMEOUT_MS }], s);
    expect(s.stage).toBe("recording");
  });
});

describe("устойчивость", () => {
  it("битый кадр (unknown) стадию не двигает", () => {
    const before = run([
      { kind: "ptt_start", atMs: T0 },
      { kind: "ptt_stop", atMs: T0 + 500 }
    ]);
    const after = run([{ kind: "voice_state", state: "unknown", detail: "none", atMs: T0 + 600 }], before);
    expect(after.stage).toBe("sending");
  });

  it("отпускание без записи (рация) стадию не трогает", () => {
    const s = run([{ kind: "ptt_stop", atMs: T0 }]);
    expect(s).toBe(INITIAL_UTTERANCE_PROGRESS);
  });

  it("разрыв связи сбрасывает стадию: висящее «обрабатываю» соврало бы", () => {
    const s = run([
      { kind: "ptt_start", atMs: T0 },
      { kind: "ptt_stop", atMs: T0 + 500 },
      { kind: "disconnected" }
    ]);
    expect(s.stage).toBe("idle");
    expect(s.sentAtMs).toBe(0);
  });

  it("speaking не откатывается обратно в listening", () => {
    const s = run([
      { kind: "ptt_start", atMs: T0 },
      { kind: "ptt_stop", atMs: T0 + 500 },
      { kind: "voice_state", state: "speaking", detail: "none", atMs: T0 + 1000 },
      { kind: "voice_state", state: "listening", detail: "none", atMs: T0 + 1200 }
    ]);
    expect(s.stage).toBe("speaking");
  });
});

describe("utteranceProgressView", () => {
  it("покой: все ступени погашены, счётчика нет", () => {
    const v = utteranceProgressView(INITIAL_UTTERANCE_PROGRESS, T0);
    expect(v.level).toBe("idle");
    expect(v.elapsedS).toBeNull();
    expect(Object.values(v.steps).every((x) => x === "idle")).toBe(true);
  });

  it("обработка: голос и STT пройдены, LLM активен", () => {
    const s = run([
      { kind: "ptt_start", atMs: T0 },
      { kind: "ptt_stop", atMs: T0 + 1000 },
      { kind: "voice_state", state: "idle", detail: "none", atMs: T0 + 1200 }
    ]);
    const v = utteranceProgressView(s, T0 + 4200);
    expect(v.steps).toEqual({ voice: "done", stt: "done", llm: "active", tts: "idle" });
    expect(v.elapsedS).toBe(3);
    expect(v.level).toBe("warn");
  });

  it("detail=denied перекрывает стадию", () => {
    const s = run([
      { kind: "ptt_start", atMs: T0 },
      { kind: "voice_state", state: "listening", detail: "denied", atMs: T0 + 100 }
    ]);
    const v = utteranceProgressView(s, T0 + 200);
    expect(v.level).toBe("bad");
    expect(v.label).toContain("МИКРОФОН");
  });

  it("у каждой стадии есть непустая подпись и подсказка", () => {
    const stages: UtteranceProgressState[] = [
      INITIAL_UTTERANCE_PROGRESS,
      run([{ kind: "ptt_start", atMs: T0 }]),
      run([{ kind: "ptt_start", atMs: T0 }, { kind: "ptt_stop", atMs: T0 + 1 }])
    ];
    for (const s of stages) {
      const v = utteranceProgressView(s, T0 + 10);
      expect(v.label.length).toBeGreaterThan(0);
      expect(v.hint.length).toBeGreaterThan(0);
    }
  });
});
