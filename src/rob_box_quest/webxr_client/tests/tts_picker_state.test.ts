// AV-27 / issue #1919: чистая логика TTS picker'а (src/state/tts_picker_state.ts).
//
// Меню рисуется на canvas, которого в jsdom нет (см. комментарий в
// stream_menu.test.ts / bridge_environment.test.ts), поэтому здесь тестируется
// только редьюсер + маршрутизация id целей указателя + форматирование строк.
// Именно на них держится поведение UI: пять состояний из карточки
// (loading / empty / populated / previewing / applying), честный текст
// пустого состояния и лок UI на время set_voice.

import { describe, it, expect } from "vitest";
import {
  APPLY_TARGET_ID,
  CLOSE_TARGET_ID,
  EMPTY_VOICES_TEXT,
  INITIAL_TTS_PICKER_STATE,
  LAUNCH_TARGET_ID,
  NEXT_PAGE_TARGET_ID,
  PAGE_SIZE,
  PREV_PAGE_TARGET_ID,
  STOP_TARGET_ID,
  TTS_TARGET_PREFIX,
  canApply,
  canStopPreview,
  isInteractive,
  newPreviewRequestId,
  pageCount,
  parseTtsTargetId,
  previewTargetId,
  selectTargetId,
  ttsFooterText,
  ttsHeaderText,
  ttsPageInfo,
  ttsPickerReducer,
  ttsRowViews,
  type TtsPickerAction,
  type TtsPickerState
} from "../src/state/tts_picker_state";
import type { VoiceInfo } from "../src/wire/messages";

const VOICES: VoiceInfo[] = [
  {
    voice_id: "alena",
    display_name: "Алёна",
    language: "ru-RU",
    gender: "female",
    provider: "yandex",
    presets: ["standard", "friendly"]
  },
  {
    voice_id: "male-qn-qingse",
    display_name: "Qingse",
    language: "zh-CN",
    gender: "male",
    provider: "minimax"
  }
];

/** Прогнать цепочку действий от начального состояния. */
function run(actions: TtsPickerAction[], from: TtsPickerState = INITIAL_TTS_PICKER_STATE): TtsPickerState {
  return actions.reduce((st, a) => ttsPickerReducer(st, a), from);
}

const opened = (): TtsPickerState => run([{ kind: "open" }]);

const populated = (): TtsPickerState =>
  run([
    { kind: "open" },
    { kind: "voice_list", voices: VOICES, activeVoice: "alena", activeProvider: "yandex" }
  ]);

describe("ttsPickerReducer — состояние 1: loading", () => {
  it("open переводит в loading (список запрашивается заново)", () => {
    const st = opened();
    expect(st.phase).toBe("loading");
    expect(st.voices).toEqual([]);
    expect(st.error).toBeNull();
  });

  it("повторный open — no-op (тот же объект, лишнего рендера нет)", () => {
    const st = opened();
    expect(ttsPickerReducer(st, { kind: "open" })).toBe(st);
  });

  it("footer в loading честно говорит, что ждём сервер", () => {
    expect(ttsFooterText(opened()).text).toBe("waiting for voice_list…");
  });

  it("строк в loading нет — выдумывать голоса нельзя", () => {
    expect(ttsRowViews(opened())).toEqual([]);
  });
});

describe("ttsPickerReducer — состояние 2: empty", () => {
  it("voices: [] → phase=empty", () => {
    const st = run([{ kind: "open" }, { kind: "voice_list", voices: [] }]);
    expect(st.phase).toBe("empty");
    expect(st.voices).toEqual([]);
  });

  it("footer печатает точный текст из карточки", () => {
    const st = run([{ kind: "open" }, { kind: "voice_list", voices: [] }]);
    expect(ttsFooterText(st).text).toBe(EMPTY_VOICES_TEXT);
    expect(EMPTY_VOICES_TEXT).toBe("Provider does not expose a voice list");
  });

  it("в empty строк нет (никаких придуманных записей)", () => {
    const st = run([{ kind: "open" }, { kind: "voice_list", voices: [] }]);
    expect(ttsRowViews(st)).toEqual([]);
  });

  it("header в empty показывает прочерк, а не нулевой голос", () => {
    const st = run([{ kind: "open" }, { kind: "voice_list", voices: [] }]);
    expect(ttsHeaderText(st)).toBe("TTS VOICES · —");
  });
});

describe("ttsPickerReducer — состояние 3: populated", () => {
  it("voice_list наполняет список и активный голос/провайдер", () => {
    const st = populated();
    expect(st.phase).toBe("ready");
    expect(st.voices).toHaveLength(2);
    expect(st.currentVoiceId).toBe("alena");
    expect(st.activeProvider).toBe("yandex");
  });

  it("активный голос подсвечен, остальные — нет", () => {
    const rows = ttsRowViews(populated());
    expect(rows.map((r) => [r.voiceId, r.current])).toEqual([
      ["alena", true],
      ["male-qn-qingse", false]
    ]);
  });

  it("строка показывает display_name + язык и provider/gender/presets", () => {
    const rows = ttsRowViews(populated());
    expect(rows[0].title).toBe("Алёна · ru-RU");
    expect(rows[0].subtitle).toBe("yandex · female · standard/friendly");
    expect(rows[1].subtitle).toBe("minimax · male");
  });

  it("select выделяет существующий голос", () => {
    const st = ttsPickerReducer(populated(), { kind: "select", voiceId: "male-qn-qingse" });
    expect(st.selectedVoiceId).toBe("male-qn-qingse");
    expect(canApply(st)).toBe(true);
  });

  it("select неизвестного голоса игнорируется", () => {
    const base = populated();
    expect(ttsPickerReducer(base, { kind: "select", voiceId: "nope" })).toBe(base);
  });

  it("header показывает провайдера и текущий голос", () => {
    expect(ttsHeaderText(populated())).toBe("TTS VOICES · yandex · now: alena");
  });

  it("новый voice_list сбрасывает выбор, если голоса больше нет в списке", () => {
    const st = run(
      [
        { kind: "select", voiceId: "male-qn-qingse" },
        { kind: "voice_list", voices: [VOICES[0]], activeVoice: "alena" }
      ],
      populated()
    );
    expect(st.selectedVoiceId).toBeNull();
  });

  it("новый voice_list сохраняет выбор, если голос остался", () => {
    const st = run(
      [
        { kind: "select", voiceId: "alena" },
        { kind: "voice_list", voices: VOICES, activeVoice: "alena" }
      ],
      populated()
    );
    expect(st.selectedVoiceId).toBe("alena");
  });
});

describe("ttsPickerReducer — состояние 4: previewing", () => {
  it("preview_sent создаёт прогресс на нужном голосе", () => {
    const st = run([{ kind: "preview_sent", requestId: "r1", voiceId: "alena" }], populated());
    expect(st.preview).toEqual({
      requestId: "r1",
      voiceId: "alena",
      received: 0,
      total: 0,
      done: false
    });
    expect(canStopPreview(st)).toBe(true);
  });

  it("preview_audio считает чанки и запоминает total", () => {
    const st = run(
      [
        { kind: "preview_sent", requestId: "r1", voiceId: "alena" },
        { kind: "preview_audio", requestId: "r1", seq: 0, total: 3 },
        { kind: "preview_audio", requestId: "r1", seq: 1, total: 3 }
      ],
      populated()
    );
    expect(st.preview).toMatchObject({ received: 2, total: 3, done: false });
    expect(ttsFooterText(st).text).toBe("PREVIEW alena: chunk 2/3…");
  });

  it("чанк чужого request_id игнорируется (несколько preview параллельно)", () => {
    const base = run([{ kind: "preview_sent", requestId: "r1", voiceId: "alena" }], populated());
    expect(ttsPickerReducer(base, { kind: "preview_audio", requestId: "r2", seq: 0, total: 1 })).toBe(base);
  });

  it("preview_done помечает завершение, STOP гаснет", () => {
    const st = run(
      [
        { kind: "preview_sent", requestId: "r1", voiceId: "alena" },
        { kind: "preview_audio", requestId: "r1", seq: 0, total: 1 },
        { kind: "preview_done", requestId: "r1" }
      ],
      populated()
    );
    expect(st.preview?.done).toBe(true);
    expect(canStopPreview(st)).toBe(false);
    expect(ttsFooterText(st)).toEqual({ text: "PREVIEW alena: done", level: "ok" });
  });

  it("строка голоса помечена previewing только пока preview не завершён", () => {
    const running = run([{ kind: "preview_sent", requestId: "r1", voiceId: "alena" }], populated());
    expect(ttsRowViews(running)[0].previewing).toBe(true);
    const done = ttsPickerReducer(running, { kind: "preview_done", requestId: "r1" });
    expect(ttsRowViews(done)[0].previewing).toBe(false);
  });

  it("preview_error чистит прогресс и показывает reason (честный FAIL)", () => {
    const st = run(
      [
        { kind: "preview_sent", requestId: "r1", voiceId: "alena" },
        { kind: "preview_error", requestId: "r1", reason: "preview_synthesis_not_implemented_in_mvp" }
      ],
      populated()
    );
    expect(st.preview).toBeNull();
    expect(ttsFooterText(st)).toEqual({
      text: "ERROR: preview_synthesis_not_implemented_in_mvp",
      level: "bad"
    });
  });

  it("preview_error чужого request_id не трогает активный preview", () => {
    const base = run([{ kind: "preview_sent", requestId: "r1", voiceId: "alena" }], populated());
    expect(ttsPickerReducer(base, { kind: "preview_error", requestId: "r2", reason: "x" })).toBe(base);
  });

  it("preview_stopped (кнопка STOP) снимает прогресс", () => {
    const st = run(
      [
        { kind: "preview_sent", requestId: "r1", voiceId: "alena" },
        { kind: "preview_stopped" }
      ],
      populated()
    );
    expect(st.preview).toBeNull();
    expect(canStopPreview(st)).toBe(false);
  });
});

describe("ttsPickerReducer — состояние 5: applying", () => {
  it("apply_sent лочит UI", () => {
    const st = run(
      [{ kind: "select", voiceId: "alena" }, { kind: "apply_sent", voiceId: "alena" }],
      populated()
    );
    expect(st.applyingVoiceId).toBe("alena");
    expect(isInteractive(st)).toBe(false);
    expect(canApply(st)).toBe(false);
    expect(ttsHeaderText(st)).toBe("APPLYING alena…");
  });

  it("второй apply_sent до ответа игнорируется", () => {
    const locked = run([{ kind: "apply_sent", voiceId: "alena" }], populated());
    expect(ttsPickerReducer(locked, { kind: "apply_sent", voiceId: "male-qn-qingse" })).toBe(locked);
  });

  it("select во время лока игнорируется", () => {
    const locked = run([{ kind: "apply_sent", voiceId: "alena" }], populated());
    expect(ttsPickerReducer(locked, { kind: "select", voiceId: "male-qn-qingse" })).toBe(locked);
  });

  it("voice_set_ack снимает лок и переносит активный голос", () => {
    const st = run(
      [
        { kind: "select", voiceId: "male-qn-qingse" },
        { kind: "apply_sent", voiceId: "male-qn-qingse" },
        { kind: "voice_set_ack", voiceId: "male-qn-qingse", preset: "friendly" }
      ],
      populated()
    );
    expect(st.applyingVoiceId).toBeNull();
    expect(st.currentVoiceId).toBe("male-qn-qingse");
    expect(st.selectedVoiceId).toBeNull();
    expect(ttsRowViews(st).find((r) => r.current)?.voiceId).toBe("male-qn-qingse");
  });

  it("voice_set_nack снимает лок и показывает ERROR инлайном", () => {
    const st = run(
      [
        { kind: "apply_sent", voiceId: "ghost" },
        { kind: "voice_set_nack", voiceId: "ghost", reason: "voice_unavailable" }
      ],
      populated()
    );
    expect(st.applyingVoiceId).toBeNull();
    expect(st.currentVoiceId).toBe("alena"); // активный голос НЕ поменялся
    expect(ttsFooterText(st)).toEqual({ text: "ERROR: voice_unavailable", level: "bad" });
  });

  it("voice_set_nack.available попадает в текст ошибки (не более 4 штук)", () => {
    const st = run(
      [
        { kind: "apply_sent", voiceId: "ghost" },
        {
          kind: "voice_set_nack",
          reason: "voice_unavailable",
          available: ["alena", "filipp", "ermil", "jane", "omazh"]
        }
      ],
      populated()
    );
    expect(ttsFooterText(st).text).toBe(
      "ERROR: voice_unavailable (доступно: alena, filipp, ermil, jane)"
    );
  });
});

describe("ttsPickerReducer — close / disconnect", () => {
  it("close скрывает меню и чистит preview/ошибку", () => {
    const st = run(
      [
        { kind: "preview_sent", requestId: "r1", voiceId: "alena" },
        { kind: "close" }
      ],
      populated()
    );
    expect(st.phase).toBe("closed");
    expect(st.preview).toBeNull();
    expect(st.error).toBeNull();
  });

  it("disconnected обнуляет активный голос (после разрыва он не факт, ADR-0018)", () => {
    const st = run([{ kind: "disconnected" }], populated());
    expect(st.phase).toBe("loading");
    expect(st.currentVoiceId).toBeNull();
    expect(st.applyingVoiceId).toBeNull();
    expect(st.preview).toBeNull();
  });

  it("disconnected на закрытом меню его не открывает", () => {
    const st = ttsPickerReducer(INITIAL_TTS_PICKER_STATE, { kind: "disconnected" });
    expect(st.phase).toBe("closed");
  });

  it("disconnected снимает лок apply (ответа уже не будет)", () => {
    const st = run([{ kind: "apply_sent", voiceId: "alena" }, { kind: "disconnected" }], populated());
    expect(st.applyingVoiceId).toBeNull();
    expect(isInteractive(st)).toBe(true);
  });
});

describe("parseTtsTargetId — маршрутизация клика указателя", () => {
  it("строка голоса", () => {
    expect(parseTtsTargetId(selectTargetId("alena"))).toEqual({ kind: "select", voiceId: "alena" });
  });

  it("кнопка PREVIEW строки", () => {
    expect(parseTtsTargetId(previewTargetId("alena"))).toEqual({ kind: "preview", voiceId: "alena" });
  });

  it("кнопки меню", () => {
    expect(parseTtsTargetId(APPLY_TARGET_ID)).toEqual({ kind: "apply" });
    expect(parseTtsTargetId(STOP_TARGET_ID)).toEqual({ kind: "stop" });
    expect(parseTtsTargetId(CLOSE_TARGET_ID)).toEqual({ kind: "close" });
    expect(parseTtsTargetId(LAUNCH_TARGET_ID)).toEqual({ kind: "launch" });
  });

  it("id панели и строки stream_menu — не наши (клик уходит дальше)", () => {
    expect(parseTtsTargetId("p1")).toBeNull();
    expect(parseTtsTargetId("menu:camera_rear")).toBeNull();
    expect(parseTtsTargetId("main_screen")).toBeNull();
  });

  it("не путает префикс в середине id", () => {
    expect(parseTtsTargetId("panel_tts:apply")).toBeNull();
  });

  it("пустой voice_id → null (не создаём безымянную цель)", () => {
    expect(parseTtsTargetId(`${TTS_TARGET_PREFIX}voice:`)).toBeNull();
    expect(parseTtsTargetId(`${TTS_TARGET_PREFIX}preview:`)).toBeNull();
  });

  it("неизвестный суффикс → null", () => {
    expect(parseTtsTargetId(`${TTS_TARGET_PREFIX}wat`)).toBeNull();
  });

  it("id голоса с двоеточием внутри разбирается целиком", () => {
    expect(parseTtsTargetId(selectTargetId("minimax:male-qn"))).toEqual({
      kind: "select",
      voiceId: "minimax:male-qn"
    });
  });
});

describe("newPreviewRequestId", () => {
  it("использует переданный uuid-генератор", () => {
    expect(newPreviewRequestId(Math.random, () => "uuid-1")).toBe("uuid-1");
  });

  it("fallback без crypto даёт непустую строку с префиксом pv-", () => {
    const saved = globalThis.crypto;
    delete (globalThis as { crypto?: unknown }).crypto;
    try {
      const id = newPreviewRequestId(() => 0.5);
      expect(id.startsWith("pv-")).toBe(true);
      expect(id.length).toBeGreaterThan(4);
    } finally {
      Object.defineProperty(globalThis, "crypto", { value: saved, configurable: true });
    }
  });
});

describe("footer подсказки в обычном состоянии", () => {
  it("без выбора — подсказка про PREVIEW/APPLY", () => {
    expect(ttsFooterText(populated())).toEqual({
      text: "pick a voice · PREVIEW / APPLY",
      level: "ok"
    });
  });

  it("с выбором — что применится", () => {
    const st = ttsPickerReducer(populated(), { kind: "select", voiceId: "alena" });
    expect(ttsFooterText(st).text).toBe("selected alena → APPLY");
  });

  it("ошибка приоритетнее прогресса preview", () => {
    const st = run(
      [
        { kind: "apply_sent", voiceId: "ghost" },
        { kind: "voice_set_nack", reason: "tts_unreachable" }
      ],
      populated()
    );
    expect(ttsFooterText(st).level).toBe("bad");
  });
});

// ---------------------------------------------------------------------------
// Листание списка голосов.
//
// У yandex 11 голосов, у minimax 10 (tts_voice_registry.PROVIDER_VOICES), а в
// меню помещается 8. До листания последние просто не отображались, и выбрать
// их было нечем — меню молча врало, что голосов ровно восемь.
// ---------------------------------------------------------------------------

describe("листание списка голосов", () => {
  const many = (n: number): VoiceInfo[] =>
    Array.from({ length: n }, (_, i) => ({
      voice_id: `v${i}`,
      display_name: `Voice ${i}`,
      language: "ru",
      gender: "male" as const
    }));

  function ready(n: number): TtsPickerState {
    return [
      { kind: "open" } as TtsPickerAction,
      { kind: "voice_list", voices: many(n) } as TtsPickerAction
    ].reduce(ttsPickerReducer, INITIAL_TTS_PICKER_STATE);
  }

  it("первая страница — ровно PAGE_SIZE строк", () => {
    const s = ready(11);
    expect(ttsRowViews(s)).toHaveLength(PAGE_SIZE);
    expect(ttsRowViews(s)[0].voiceId).toBe("v0");
  });

  it("вторая страница показывает остаток, а не обрезает список", () => {
    const s = ttsPickerReducer(ready(11), { kind: "page", delta: 1 });
    const rows = ttsRowViews(s);
    expect(rows).toHaveLength(3);
    expect(rows.map((r) => r.voiceId)).toEqual(["v8", "v9", "v10"]);
  });

  it("каждый голос достижим хотя бы на одной странице", () => {
    let s = ready(11);
    const seen = new Set<string>();
    for (let i = 0; i < pageCount(11); i++) {
      for (const r of ttsRowViews(s)) seen.add(r.voiceId);
      s = ttsPickerReducer(s, { kind: "page", delta: 1 });
    }
    expect(seen.size).toBe(11);
  });

  it("за границы списка не уходит", () => {
    const first = ready(11);
    expect(ttsPickerReducer(first, { kind: "page", delta: -1 })).toBe(first);
    const last = ttsPickerReducer(first, { kind: "page", delta: 1 });
    expect(ttsPickerReducer(last, { kind: "page", delta: 1 })).toBe(last);
  });

  it("короткий список — одна страница, стрелки не нужны", () => {
    const info = ttsPageInfo(ready(5));
    expect(info.pages).toBe(1);
    expect(info.hasPrev).toBe(false);
    expect(info.hasNext).toBe(false);
  });

  it("укоротившийся список не оставляет страницу за своим концом", () => {
    const s = ttsPickerReducer(ready(11), { kind: "page", delta: 1 });
    expect(s.page).toBe(1);
    const shrunk = ttsPickerReducer(s, { kind: "voice_list", voices: many(3) });
    expect(shrunk.page).toBe(0);
    expect(ttsRowViews(shrunk)).toHaveLength(3);
  });

  it("во время apply листать нельзя — UI залочен", () => {
    const locked = ttsPickerReducer(ready(11), { kind: "apply_sent", voiceId: "v0" });
    expect(ttsPickerReducer(locked, { kind: "page", delta: 1 })).toBe(locked);
  });

  it("id стрелок разбираются в действие листания", () => {
    expect(parseTtsTargetId(PREV_PAGE_TARGET_ID)).toEqual({ kind: "page", delta: -1 });
    expect(parseTtsTargetId(NEXT_PAGE_TARGET_ID)).toEqual({ kind: "page", delta: 1 });
  });

  it("футер сообщает, что список длиннее страницы", () => {
    const text = ttsFooterText(ready(11)).text;
    expect(text).toContain("11");
    expect(text).toContain("листать");
  });
});
