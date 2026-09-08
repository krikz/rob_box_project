// Ручные типы сообщений JSON_CMD / JSON_EVENT (docs/architecture/meta-quest-api.md §5, §6).
//
// Сгенерированная часть контракта (XxxCmd / XxxEvent интерфейсы для
// admin_logs, teleop_twist, voice_pipeline, …, и discriminated union-ы
// JsonCmdGenerated / JsonEventGenerated) живёт в `protocol_generated.ts` —
// это машинное отражение `rob_box_core._bridge_protocol_data` (ADR-0080 §2.2).
//
// Здесь остаётся только то, что либо ещё НЕ внесено в каталог, либо
// несёт UI-специфику (helper-типы для picker'ов и панелей):
//
//   * HelloMsg / WelcomeMsg / SubscribeMsg / UnsubscribeMsg — control-frame
//     shapes. Они в CONTROL_FRAMES каталога, но c per-field комментариями,
//     которые генератор не повторяет.
//   * VoiceInfo / VoicePresetInfo — DTO, в wire уходит как part of
//     voice_list/voice_presets events. Каталог ссылается на них по имени
//     (list[VoiceInfo], list[VoicePresetInfo]) — TS-определения обязаны
//     жить где-то; UI импортирует отсюда.
//   * VoicePttMode / VoiceWireMode — клиентские helper-типы для UI-кода,
//     которых нет в каталоге (там это inline-литералы внутри payload).
//   * VoicePresetLegacy / VoicePresetId / VoicePreset / VoiceLanguage —
//     старые ID пресетов (Phase 1/2.0), AV-28 §P7 новые стили речи.
//   * ErrorMsg — структура ERROR-frame payload (UI его рисует как toast).
//   * StreamMeta — клиентская нормализация подписки.
//   * Fallback-вариант `{ cmd: string; ts_ms: number; [k: string]: unknown }`
//     для forward-compat с неизвестными payload'ами от старого сервера.

import type {
  JsonCmdGenerated,
  JsonEventGenerated,
  TeleopTwistCmd,
  TeleopHeartbeatCmd,
  StopEmergencyCmd,
  VoicePttStartCmd,
  VoicePttStopCmd,
  VoiceModeCmd,
  StreamSelectCmd,
  StreamListCmd,
  ListVoicesCmd,
  SetVoiceCmd,
  PreviewVoiceCmd,
  SetPanelTopicCmd,
  PingCmd,
  AdminLogsCmd,
  AdminLogsStopCmd,
  UiButtonCmd,
  VoicePipelineCmd,
  VoiceListenStartCmd,
  VoiceListenStopCmd,
  // Supervisor v2 (msgpack JSON-equivalent — meta-quest-api.md §5.1)
  SupervisorSetModeCmd,
  SupervisorAcquireFloorCmd,
  SupervisorReleaseFloorCmd,
  SupervisorGetStateCmd,
} from "./protocol_generated";

// Re-export generated interfaces under their old names so existing
// `import type { TeleopTwistCmd } from "../wire/messages"` keep working.
export type {
  TeleopTwistCmd,
  TeleopHeartbeatCmd,
  StopEmergencyCmd,
  VoicePttStartCmd,
  VoicePttStopCmd,
  VoiceModeCmd,
  StreamSelectCmd,
  StreamListCmd,
  ListVoicesCmd,
  SetVoiceCmd,
  PreviewVoiceCmd,
  SetPanelTopicCmd,
  PingCmd,
  AdminLogsCmd,
  AdminLogsStopCmd,
  UiButtonCmd,
  VoicePipelineCmd,
  VoiceListenStartCmd,
  VoiceListenStopCmd,
  SupervisorSetModeCmd,
  SupervisorAcquireFloorCmd,
  SupervisorReleaseFloorCmd,
  SupervisorGetStateCmd,
  JsonCmdGenerated,
  JsonEventGenerated,
};

// ──────────────────── Control frames (HELLO / WELCOME / SUBSCRIBE / …) ────────────────────

export interface HelloMsg {
  client_version: string;
  capabilities: string[];
  session_pin: string;
}

export interface WelcomeMsg {
  server_version: string;
  session_id: string;
  server_time_ms: number;
  robot_status?: Record<string, unknown>;
}

// ──────────────────── Subscribe / Unsubscribe (control frames) ────────────────────

export interface SubscribeMsg {
  topic: string;
  quality?: "low" | "med" | "high";
}

export interface UnsubscribeMsg {
  topic: string;
}

// ──────────────────── Voice helper types (UI-only) ────────────────────

// Голос: режим PTT. "radio" = голос оператора → динамик робота (рация);
// "robot_voice" = голос оператора → STT → LLM → TTS голосом робота.
export type VoicePttMode = "radio" | "robot_voice";

// Клиентский режим voice_input_mode (UI/логика сцены). В payload'е
// voice_mode это литеральный union внутри JsonEventGenerated, но
// scene-код предпочитает именованный type для narrowing.
export type VoiceWireMode = "off" | "passthrough" | "ttts_proxy" | "stt_llm" | "llm_formalize";

// Phase 2 §4.3+§4.5: set_voice { voice_id, preset? }. preset ∈ standard|friendly|authoritative|whisper.
//
// AV-28 §P7 (formalize-режим): preset теперь ссылается на стиль речи
// (technical/street/caveman/business/philosopher/lenin) из
// src/rob_box_voice/config/voice_presets.yaml. Сервер мапит его на
// конкретный промпт dialogue_node. Чтобы не ломать старый контракт
// "standard|friendly|...", принимаемый сервером, расширяем тип через
// литеральный union (полный список) — TS-strict его примет.
export type VoicePresetLegacy = "standard" | "friendly" | "authoritative" | "whisper";
/** AV-28 §P7: ID пресета стиля речи (voice_presets.yaml: presets.<id>). */
export type VoicePresetId =
  | "technical"
  | "street"
  | "caveman"
  | "business"
  | "philosopher"
  | "lenin"
  /**
   * Нейтральный пресет: не накладывает свой стиль, только убирает оговорки
   * и переводит реплику на выбранный язык. Не путать с выключенной
   * LLM-ступенью — там реплика уходит дословно и на исходном языке.
   */
  | "translate";
/** Совместный тип — клиент шлёт либо старый, либо новый ID. */
export type VoicePreset = VoicePresetLegacy | VoicePresetId;
/** AV-28 §P7: ID языка из voice_presets.yaml: ключи languages. */
export type VoiceLanguage = "ru" | "en" | "fr" | "de" | "zh" | "hi";

/**
 * AV-28 §P7: контракт-описание пресета (UI рисует кнопки из этого списка).
 * Сервер шлёт его в JSON_EVENT{type:"voice_presets"} либо как часть
 * voice_list.voices[].presets[] (см. VoiceInfo).
 */
export interface VoicePresetInfo {
  id: VoicePresetId;
  /** Локализованное имя для UI (русский). */
  name: string;
}

// Структура описания голоса из voice-pipeline.
export interface VoiceInfo {
  voice_id: string;
  display_name: string;
  language: string;
  gender: "male" | "female" | "neutral";
  description?: string;
  // Доступные пресеты (subset of VoicePreset).
  presets?: VoicePreset[];
  // AV-27 / issue #1919 — провайдер, у которого этот голос есть
  // (yandex | minimax | silero). Опциональное поле: для cross-provider UI
  // (например, фильтр по провайдеру) — сервер присылает его в voice_list;
  // до AV-27 голоса в LLM-контексте уже различались по провайдеру, но wire
  // был без provider. Добавляется как minor type-bump (см. design
  // t_5b9d5d0c §89-114 «Adding provider to the existing client VoiceInfo type»).
  provider?: string;
}

// ──────────────────── Discriminated unions (hand-written fallback + generated) ────────────────────

export type JsonCmd = JsonCmdGenerated | { cmd: string; ts_ms: number; [k: string]: unknown };
export type JsonEvent =
  // Сгенерированный union покрывает большинство событий (см. protocol_generated.ts).
  // Ряд типов, добавленных позже (#2184 и т.п.), пока не заехали в каталог —
  // докидываем их руками до следующего regen (tools/gen_bridge_protocol_ts.py).
  | JsonEventGenerated
  // issue #2184 — TARS 2 metrics panel: РЯДЫ ТОЧЕК из Prometheus (series)
  // или строки из Loki (lines). Именно это клиент рисует на экране TARS 2;
  // tars_panel_url выше остался ссылкой «доглядеть с десктопа».
  // status="empty" — запрос корректен, но данных нет: панель обязана
  // сказать это словами, а не показать пустой график.
  | {
      type: "tars_panel_data";
      request_id: string;
      status: "ok" | "empty" | "error" | string;
      datasource: string;
      query: string;
      note: string;
      summary: string;
      series: TarsPanelSeries[];
      lines: TarsPanelLogLine[];
      url: string;
      error: string;
      ts_ms: number;
    }
  | { type: string; ts_ms?: number; [k: string]: unknown };

// ──────────────────── Error frame + stream metadata ────────────────────

/** Один ряд Prometheus: точки ``[unix_seconds, value]`` по возрастанию ts. */
export interface TarsPanelSeries {
  name: string;
  labels: Record<string, string>;
  points: [number, number][];
}

/** Одна строка Loki (новые — первыми). */
export interface TarsPanelLogLine {
  ts: number;
  line: string;
  labels: Record<string, string>;
}

export interface ErrorMsg {
  code: string;
  message: string;
}

export interface StreamMeta {
  topic: string;
  topic_id: number;
  kind: "ros_topic" | "camera_direct";
  source: string;
  default_quality: string;
  description?: string;
}
