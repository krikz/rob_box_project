// Тесты для wire-cmd helper панели пайплайна (t_80e7aa1e, шаг 4б).
//
// Покрывают чистую логику построения JSON-cmd `voice_pipeline` на стороне
// клиента. Сервер и supervisor валидируются в Python-тестах
// (test_ws_server_voice.py + test_grip_pipeline.py). Здесь — что клиент
// шлёт ровно то, что supervisor ждёт.

import { describe, it, expect } from "vitest";
import {
  buildVoicePipelineCmd,
  type VoicePipelinePanelState
} from "../src/wire/voice_pipeline_cmd";
import type { VoiceLanguage, VoicePresetId } from "../src/wire/messages";

describe("buildVoicePipelineCmd", () => {
  it("llm=true + preset='translate' + lang='en' → три контрактных поля", () => {
    const state: VoicePipelinePanelState = {
      sttOn: true,
      llmOn: true,
      preset: "translate",
      language: "en"
    };
    const cmd = buildVoicePipelineCmd(state, 12345);
    expect(cmd).toEqual({
      cmd: "voice_pipeline",
      ts_ms: 12345,
      llm_enabled: true,
      preset: "translate",
      language: "en"
    });
  });

  it("«Без стиля» = preset='' + llm=false → сервер увидит default пайплайна", () => {
    // Семантика: оператор жмёт «Без стиля» в панели → pipelineLlmOn=false,
    // preset="". Это default пайплайна грипа (supervisor_node.py:578).
    // Серверный whitelist (ws_server._validate_voice_pipeline_payload)
    // принимает пустой preset, и supervisor его трактует как «Без стиля»
    // (GRIP_OFF_PRESETS, supervisor_node.py:359).
    const state: VoicePipelinePanelState = {
      sttOn: true,
      llmOn: false,
      preset: "",
      language: "ru"
    };
    const cmd = buildVoicePipelineCmd(state, 1000);
    expect(cmd.llm_enabled).toBe(false);
    expect(cmd.preset).toBe("");
    expect(cmd.language).toBe("ru");
  });

  it("STT выкл, LLM выкл — рация (passthrough) → llm=false, preset='', язык по умолчанию", () => {
    // «Рация» в panel = stt=false (STT_TARGET_ID). LLM-ступень тут
    // неактуальна, но buildVoicePipelineCmd шлёт ровно то, что в state —
    // чтобы сервер не догадывался о нашей UI-семантике «passthrough».
    const state: VoicePipelinePanelState = {
      sttOn: false,
      llmOn: false,
      preset: "",
      language: "ru"
    };
    const cmd = buildVoicePipelineCmd(state);
    expect(cmd.llm_enabled).toBe(false);
    expect(cmd.preset).toBe("");
  });

  it("все поля cmd стабильны по ключам (контракт с ws_server)", () => {
    // Тест-страховка от переименования полей: ws_server ждёт ровно
    // llm_enabled / preset / language (иначе _on_grip_voice_pipeline
    // молча падает в default, см. supervisor_node.py:2650-2665).
    const state: VoicePipelinePanelState = {
      sttOn: true,
      llmOn: true,
      preset: "lenin",
      language: "fr"
    };
    const cmd = buildVoicePipelineCmd(state);
    expect(Object.keys(cmd).sort()).toEqual(
      ["cmd", "language", "llm_enabled", "preset", "ts_ms"].sort()
    );
  });

  it("типы полей совпадают с тем, что supervisor парсит", () => {
    // _on_grip_voice_pipeline делает bool(data.get('llm_enabled', False))
    // и str(...).get('preset', ''). Тест фиксирует типы клиента.
    const state: VoicePipelinePanelState = {
      sttOn: true,
      llmOn: true,
      preset: "business",
      language: "de"
    };
    const cmd = buildVoicePipelineCmd(state);
    expect(typeof cmd.llm_enabled).toBe("boolean");
    expect(typeof cmd.preset).toBe("string");
    expect(typeof cmd.language).toBe("string");
  });

  it("preset='translate' в whitelist сервера", () => {
    // ws_server.VOICE_PRESET_IDS = ('technical','street','caveman',
    // 'business','philosopher','lenin','translate'). «translate» — там,
    // потому что yaml.voice_presets.yaml его декларирует. Сервер не
    // отбрасывает, supervisor не отбрасывает.
    const state: VoicePipelinePanelState = {
      sttOn: true,
      llmOn: true,
      preset: "translate" as VoicePresetId,
      language: "en" as VoiceLanguage
    };
    const cmd = buildVoicePipelineCmd(state);
    expect(cmd.preset).toBe("translate");
  });

  it("ts_ms по умолчанию — Date.now() (число, не строка)", () => {
    const before = Date.now();
    const cmd = buildVoicePipelineCmd({
      sttOn: true,
      llmOn: false,
      preset: "",
      language: "ru"
    });
    const after = Date.now();
    expect(typeof cmd.ts_ms).toBe("number");
    expect(cmd.ts_ms).toBeGreaterThanOrEqual(before);
    expect(cmd.ts_ms).toBeLessThanOrEqual(after);
  });
});
