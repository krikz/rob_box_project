// Типы сообщений JSON_CMD / JSON_EVENT (docs/architecture/meta-quest-api.md §5, §6).

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

export interface SubscribeMsg {
  topic: string;
  quality?: "low" | "med" | "high";
}

export interface UnsubscribeMsg {
  topic: string;
}

export interface TeleopTwistCmd {
  cmd: "teleop_twist";
  ts_ms: number;
  seq: number;
  linear: { x: number; y: number; z: number };
  angular: { x: number; y: number; z: number };
  deadman: boolean;
}

export interface StopEmergencyCmd {
  cmd: "stop_emergency";
  ts_ms: number;
  source: "controller_b" | "ui_button" | "client_lost";
}

// Голос: режим PTT. "radio" = голос оператора → динамик робота (рация);
// "robot_voice" = голос оператора → STT → LLM → TTS голосом робота.
export type VoicePttMode = "radio" | "robot_voice";

export interface VoicePttStartCmd {
  cmd: "voice_ptt_start";
  ts_ms: number;
  mode?: VoicePttMode;
}

export interface VoicePttStopCmd {
  cmd: "voice_ptt_stop";
  ts_ms: number;
  mode?: VoicePttMode;
}

// Смена режима голоса (meta-quest-api §5). Супервизор применяет его как
// voice_input_mode на dialogue_node (ADR-0028 S5).
export type VoiceWireMode = "off" | "passthrough" | "ttts_proxy" | "stt_llm" | "llm_formalize";

export interface VoiceModeCmd {
  cmd: "voice_mode";
  ts_ms: number;
  mode: VoiceWireMode;
}

export interface StreamSelectCmd {
  cmd: "stream_select";
  ts_ms: number;
  topic: string;
}

export interface StreamListCmd {
  cmd: "stream_list";
  ts_ms: number;
}

// Phase 2 §4.1: список голосов через `list_voices` (запрашивается из voice-pipeline).
export interface ListVoicesCmd {
  cmd: "list_voices";
  ts_ms: number;
}

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
  | "lenin";
/** Совместный тип — клиент шлёт либо старый, либо новый ID. */
export type VoicePreset = VoicePresetLegacy | VoicePresetId;
/** AV-28 §P7: ID языка из voice_presets.yaml: languages[]. */
export type VoiceLanguage = "ru" | "en";

export interface SetVoiceCmd {
  cmd: "set_voice";
  ts_ms: number;
  voice_id: string;
  preset?: VoicePreset;
  /**
   * AV-28 §P7: желаемый язык вывода. Сервер применяет его как
   * voice_output_language на dialogue_node. Если поле отсутствует,
   * сервер берёт default_language из voice_presets.yaml.
   */
  language?: VoiceLanguage;
}

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

// Phase 2 §4.2: preview_voice { voice_id, text } → сервер шлёт audio bytes обратно
// через JSON_EVENT{type:"preview_voice_audio", format, seq} + BINARY_FRAME.
export interface PreviewVoiceCmd {
  cmd: "preview_voice";
  ts_ms: number;
  voice_id: string;
  text: string;
  request_id: string;
}

// Phase 2 §6.2: drag-from-gui → drop-on-panel: set_panel_topic { panel_id, topic }.
export interface SetPanelTopicCmd {
  cmd: "set_panel_topic";
  ts_ms: number;
  panel_id: string;
  topic: string;
}

export type JsonCmd =
  | TeleopTwistCmd
  | StopEmergencyCmd
  | VoicePttStartCmd
  | VoicePttStopCmd
  | VoiceModeCmd
  | StreamSelectCmd
  | StreamListCmd
  | ListVoicesCmd
  | SetVoiceCmd
  | PreviewVoiceCmd
  | SetPanelTopicCmd
  | { cmd: string; ts_ms: number; [k: string]: unknown };

// Структура описания голоса из voice-pipeline.
export interface VoiceInfo {
  voice_id: string;
  display_name: string;
  language: string;
  gender: "male" | "female" | "neutral";
  description?: string;
  // Доступные пресеты (subset of VoicePreset).
  presets?: VoicePreset[];
}

export type JsonEvent =
  | { type: "subscribe_ack"; topic: string; stream_id: number; quality: string; kind?: string }
  | { type: "subscribe_nack"; topic: string; reason: string }
  | { type: "heartbeat"; ts_ms: number }
  | { type: "voice_state"; state: string; ts_ms: number; utterance_id?: string }
  | { type: "voice_mode_ack"; mode: string; ts_ms: number }
  | { type: "safety_stop"; reason: string; ts_ms: number }
  | { type: "robot_alert"; level: "warn" | "error"; code: string; args?: Record<string, unknown>; ts_ms: number }
  | { type: "stream_list"; items: Array<Record<string, unknown>>; ts_ms: number }
  | { type: "stream_select_ack"; topic: string; stream_id: number | null; kind?: string }
  | { type: "voice_list"; voices: VoiceInfo[]; ts_ms: number }
  | {
      type: "voice_presets";
      presets: VoicePresetInfo[];
      languages: VoiceLanguage[];
      default_preset: VoicePresetId;
      default_language: VoiceLanguage;
      ts_ms: number;
    }
  | {
      type: "voice_set_ack";
      voice_id: string;
      preset: VoicePreset;
      language: VoiceLanguage;
      ts_ms: number;
    }
  | {
      type: "voice_set_nack";
      voice_id?: string;
      preset?: VoicePreset;
      language?: VoiceLanguage;
      reason: string;
      ts_ms: number;
    }
  | {
      type: "preview_voice_audio";
      request_id: string;
      format: "mp3" | "opus" | "wav";
      content_type: string;
      seq: number;
      total: number;
      ts_ms: number;
    }
  | { type: "preview_voice_done"; request_id: string; ts_ms: number }
  | { type: "preview_voice_error"; request_id: string; reason: string; ts_ms: number }
  | { type: "ping"; ts_ms: number; nonce?: string }
  | { type: "pong"; ts_ms: number; nonce?: string }
  | { type: string; [k: string]: unknown };

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