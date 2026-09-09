// Wire-cmd helper для панели пайплайна (t_80e7aa1e, шаг 4б).
//
// Карточка t_80e7aa1e: «конфиг панели пайплайна не доезжает до супервизора
// — грип повторяет дословно». До этой правки панель слала ТОЛЬКО:
//   * `voice_mode` (на тумблеры STT/LLM/style_off, через sendStyleChange /
//     ensureLlmFormalize в main.ts:404-465);
//   * `set_voice` (на пресет/язык, через sendStyleChange в main.ts:444/457 —
//     этот cmd меняет voice_preset на dialogue_node, ЛИЧНОСТЬ, не grip pipeline).
//
// Супервизор ждёт конфиг grip-пайплайна на топике /avatar/voice_pipeline
// (см. supervisor_node.py:2634 _on_grip_voice_pipeline). Прямого
// канала от панели к супервизору не было — отсюда «грип сидит на default
// (Без стиля, llm_enabled=False), оператор выбирает стиль в панели,
// робот повторяет дословно».
//
// Новая команда `voice_pipeline` (см. ws_server.py) валидирует whitelist
// preset/language и публикует JSON в /avatar/voice_pipeline. Этот модуль —
// чистый тестируемый «что отправить при данном состоянии панели».
//
// Тест test_voice_pipeline_cmd.ts прибивает три инварианта:
//   1) лингва-форма (lang/en/ru) → {llm_enabled, preset, language};
//   2) «Без стиля» (style_off или llm=false) → llm_enabled=false, preset="";
//   3) сервер по `voice_pipeline` получит ровно то, что supervisor ждёт
//      (поля и типы).

import type { VoiceLanguage, VoicePresetId } from "./messages";

/** Снимок текущего состояния панели пайплайна (pure, сериализуемый). */
export interface VoicePipelinePanelState {
  /** STT-ступень вкл/выкл. */
  sttOn: boolean;
  /** LLM-ступень вкл/выкл. */
  llmOn: boolean;
  /** Активный пресет стиля речи (id из PRESET_ORDER или "" для Без стиля). */
  preset: VoicePresetId | "";
  /** Активный язык вывода. */
  language: VoiceLanguage;
}

/**
 * Построить JSON-cmd `voice_pipeline` для отправки на сервер.
 *
 * Сервер (ws_server.py:2237) интерпретирует так:
 *   * `llm_enabled` — bool, гейт LLM-ступени. False ⇒ грип произносит
 *     дословно, 0 вызовов LLM (см. supervisor_node.py:578 default).
 *   * `preset`      — id из VOICE_PRESET_IDS или "" (GRIP_OFF_PRESETS).
 *     "" трактуется как «Без стиля» (см. supervisor_node.py:359).
 *   * `language`    — id из VOICE_LANGUAGES.
 *
 * NB: эта функция не выдумывает новых комбинаций — `preset` берётся из
 * state как есть. Если клиент хочет «Без стиля», он кладёт `preset: ""`.
 */
export function buildVoicePipelineCmd(
  state: VoicePipelinePanelState,
  tsMs: number = Date.now()
): { cmd: "voice_pipeline"; ts_ms: number; llm_enabled: boolean; preset: string; language: string } {
  return {
    cmd: "voice_pipeline",
    ts_ms: tsMs,
    llm_enabled: state.llmOn,
    preset: state.preset,
    language: state.language,
  };
}
