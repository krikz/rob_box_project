// ⚠️  GENERATED FILE — DO NOT EDIT BY HAND.
//
// Source of truth:
//   src/rob_box_core/rob_box_core/_bridge_protocol_data.py
//
// Regenerate with:
//   python tools/gen_bridge_protocol_ts.py
//
// Detect drift in CI with:
//   python tools/gen_bridge_protocol_ts.py --check
// (.github/workflows/G-Bridge-Protocol-Drift.yml)
//
// Hand-written parts of ``messages.ts`` import from this file; this file
// must NEVER be edited to fix a server contract bug — instead fix the
// catalog and re-run the generator (ADR-0080 §2.2).

// JSON_CMD — client → server (meta-quest-api.md §5)
  /** Phase 2 R14. Сервер читает docker logs / journald и шлёт admin_logs_chunk. */
  export interface AdminLogsCmd {
    cmd: "admin_logs";
    ts_ms: number;
    service: "dialogue_node" | "rob_box_quest" | "all";
    tail?: number;
    follow?: boolean;
  }
  /** Останавливает follow-стриминг admin_logs. */
  export interface AdminLogsStopCmd {
    cmd: "admin_logs_stop";
    ts_ms: number;
  }
  /** Phase 2 §4.1. Запрос списка доступных голосов → JSON_EVENT voice_list. */
  export interface ListVoicesCmd {
    cmd: "list_voices";
    ts_ms: number;
  }
  /** Latency probe (§7). Клиент шлёт JSON_EVENT ping (см. EVENTS), JSON_CMD ping — отступление от контракта, но ws_server принимает оба. */
  export interface PingCmd {
    cmd: "ping";
    ts_ms: number;
    nonce?: string;
  }
  /** Phase 2 §4.2. Синтез фразы голосом → BINARY_FRAME + preview_voice_audio/_done/_error. */
  export interface PreviewVoiceCmd {
    cmd: "preview_voice";
    ts_ms: number;
    voice_id: string;
    text: string;
    request_id: string;
  }
  /** Phase 2 §6.2. Меняет топик, рендеримый в данной panel. */
  export interface SetPanelTopicCmd {
    cmd: "set_panel_topic";
    ts_ms: number;
    panel_id: string;
    topic: string;
  }
  /** Phase 2 §4.3 + AV-28 §P7. Меняет голос/preset/language TTS-пайплайна. */
  export interface SetVoiceCmd {
    cmd: "set_voice";
    ts_ms: number;
    voice_id: string;
    preset?: "standard" | "friendly" | "authoritative" | "whisper" | "technical" | "street" | "caveman" | "business" | "philosopher" | "lenin";
    language?: "ru" | "en";
  }
  /** Публикует в /safety/emergency_stop. Всегда в обход teleop_floor-гейта (AV-19). */
  export interface StopEmergencyCmd {
    cmd: "stop_emergency";
    ts_ms: number;
    source: "controller_b" | "ui_button" | "client_lost";
  }
  /** Phase 2 R10. Сервер отвечает JSON_EVENT stream_list со списком доступных топиков. */
  export interface StreamListCmd {
    cmd: "stream_list";
    ts_ms: number;
  }
  /** Phase 2 §6.2 (companion to set_panel_topic). */
  export interface StreamSelectCmd {
    cmd: "stream_select";
    ts_ms: number;
    topic: string;
  }
  /** §5.1 — JSON-эквивалент 0x31 ACQUIRE_FLOOR (msgpack). v1 сессия → ERROR PROTOCOL_VERSION. */
  export interface SupervisorAcquireFloorCmd {
    cmd: "supervisor_acquire_floor";
    ts_ms: number;
    seq?: number;
    client_id: string;
    floor: "teleop" | "voice";
  }
  /** §5.1 — poll-эквивалент 0x33 STATE_UPDATE. */
  export interface SupervisorGetStateCmd {
    cmd: "supervisor_get_state";
    ts_ms: number;
  }
  /** §5.1 — JSON-эквивалент 0x32 RELEASE_FLOOR (msgpack). */
  export interface SupervisorReleaseFloorCmd {
    cmd: "supervisor_release_floor";
    ts_ms: number;
    seq?: number;
    client_id: string;
    floor: "teleop" | "voice";
  }
  /** §5.1 — JSON-эквивалент 0x30 SET_MODE (msgpack). */
  export interface SupervisorSetModeCmd {
    cmd: "supervisor_set_mode";
    ts_ms: number;
    seq?: number;
    client_id: string;
    mode: "off" | "telegram_active" | "avatar_present" | "mixed" | "teleop_only" | "voice_only";
  }
  /** AV-19 / ADR-0028 §4.4 S10. Шлётся клиентом 10 Гц пока ARM + teleop_floor. */
  export interface TeleopHeartbeatCmd {
    cmd: "teleop_heartbeat";
    ts_ms: number;
    seq: number;
  }
  /** Twist-команда (Twist.linear/angular + deadman). AV-19: при require_teleop_floor=true без своего floor → ERROR FLOOR_HELD ≤1 Гц. */
  export interface TeleopTwistCmd {
    cmd: "teleop_twist";
    ts_ms: number;
    seq: number;
    linear: {
      x: number,
      y: number,
      z: number,
    };
    angular: {
      x: number,
      y: number,
      z: number,
    };
    deadman: boolean;
  }
  /** Phase 1/2. Кнопка UI → ROS-сервис через registry (phase-2). */
  export interface UiButtonCmd {
    cmd: "ui_button";
    ts_ms: number;
    button: string;
    press: boolean;
  }
  /** Включает постоянное прослушивание микрофона шлема (для wake-gate). Раньше был частью voice_ptt_start; вынесен отдельно (voice-vr 09). */
  export interface VoiceListenStartCmd {
    cmd: "voice_listen_start";
    ts_ms: number;
  }
  /** Выключает прослушивание микрофона шлема (voice-vr 09). */
  export interface VoiceListenStopCmd {
    cmd: "voice_listen_stop";
    ts_ms: number;
  }
  /** Переключает voice_input_mode в dialogue_node (ADR-0028 §5). */
  export interface VoiceModeCmd {
    cmd: "voice_mode";
    ts_ms: number;
    mode: "off" | "passthrough" | "ttts_proxy" | "stt_llm" | "llm_formalize";
  }
  /** Конфиг пайплайна грипа (issue #1989) → /avatar/voice_pipeline. Whitelist (_validate_voice_pipeline_payload): preset ∈ VOICE_PRESET_IDS, language ∈ {ru, en, fr, de, zh, hi}, llm_enabled ∈ {true,false}. */
  export interface VoicePipelineCmd {
    cmd: "voice_pipeline";
    ts_ms: number;
    llm_enabled: boolean;
    preset: string;
    language: "ru" | "en" | "fr" | "de" | "zh" | "hi";
  }
  /** Phase 2.1+ (§11.3). Заменяет старый voice_ptt{state:start|stop}. mode определяет маршрут аудио (radio=passthrough, robot_voice=STT→LLM→TTS). */
  export interface VoicePttStartCmd {
    cmd: "voice_ptt_start";
    ts_ms: number;
    mode?: "radio" | "robot_voice";
  }
  /** §11.3. Edge-triggered stop. mode обязан совпадать с voice_ptt_start. */
  export interface VoicePttStopCmd {
    cmd: "voice_ptt_stop";
    ts_ms: number;
    mode?: "radio" | "robot_voice";
  }

// JSON_EVENT — server → client (meta-quest-api.md §6)
  /** Чанк docker logs / journald в ответ на admin_logs (follow=true). */
  export interface AdminLogsChunkEvent {
    type: "admin_logs_chunk";
    service: "dialogue_node" | "rob_box_quest" | "all";
    lines: Array<string>;
    ts_ms: number;
  }
  /** Финал стриминга admin_logs. */
  export interface AdminLogsEndEvent {
    type: "admin_logs_end";
    service: string;
    ts_ms: number;
  }
  /** Phase 2. Ответ на supervisor_get_state — текущий AvatarState. */
  export interface AvatarStateAckEvent {
    type: "avatar_state_ack";
    state: Record<string, unknown>;
    ts_ms: number;
  }
  /** Phase 2. Ошибка получения state. */
  export interface AvatarStateNackEvent {
    type: "avatar_state_nack";
    reason: string;
    ts_ms: number;
  }
  /** AV-19. Сервер сообщает, что наша сессия больше не держит teleop_floor. Клиент обязан DISARM-нуться (teleop_fsm.setHasFloor(false)). */
  export interface FloorLostEvent {
    type: "floor_lost";
    floor: "teleop" | "voice";
    reason?: string;
    ts_ms: number;
  }
  /** Server → client heartbeat каждые 200 мс (§7). 3 пропуска → CONNECTION LOST. */
  export interface HeartbeatEvent {
    type: "heartbeat";
    ts_ms: number;
  }
  /** ADR-0055 / issue #1993 — обратный канал ТАРС в шлем через BINARY_FRAME + JSON_EVENT. */
  export interface OperatorTtsAudioEvent {
    type: "operator_tts_audio";
    request_id: string;
    format: "pcm_s16le" | "mp3" | "opus" | "wav";
    content_type: string;
    seq: number;
    total: number;
    ts_ms: number;
  }
  /** ADR-0055 — финал обратного канала. */
  export interface OperatorTtsDoneEvent {
    type: "operator_tts_done";
    request_id: string;
    ts_ms: number;
  }
  /** ADR-0055 — ошибка обратного канала. */
  export interface OperatorTtsErrorEvent {
    type: "operator_tts_error";
    request_id: string;
    reason: string;
    ts_ms: number;
  }
  /** Latency probe (§7). Сервер шлёт в ответ на JSON_EVENT ping клиента. JSON_CMD ping (см. COMMANDS) — отступление от контракта. */
  export interface PingEvent {
    type: "ping";
    ts_ms: number;
    nonce?: string;
  }
  /** Ответ сервера на ping (§7). */
  export interface PongEvent {
    type: "pong";
    ts_ms: number;
    nonce?: string;
  }
  /** Phase 2 §4.2. Первый чанк preview_voice (BINARY_FRAME несёт сам звук). */
  export interface PreviewVoiceAudioEvent {
    type: "preview_voice_audio";
    request_id: string;
    format: "mp3" | "opus" | "wav";
    content_type: string;
    seq: number;
    total: number;
    ts_ms: number;
  }
  /** Phase 2 §4.2. Финал preview_voice. */
  export interface PreviewVoiceDoneEvent {
    type: "preview_voice_done";
    request_id: string;
    ts_ms: number;
  }
  /** Phase 2 §4.2. Ошибка preview_voice. */
  export interface PreviewVoiceErrorEvent {
    type: "preview_voice_error";
    request_id: string;
    reason: string;
    ts_ms: number;
  }
  /** AV-26 R7 — алёрт на изменении (edge semantics, гистерезис, выдержка 10 с). */
  export interface RobotAlertEvent {
    type: "robot_alert";
    code: string;
    level: "warn" | "error" | "info";
    active?: boolean;
    args?: Record<string, unknown>;
    ts_ms: number;
  }
  /** Сервер принудительно остановил движение (controller_b или client_lost). */
  export interface SafetyStopEvent {
    type: "safety_stop";
    reason: "controller_b" | "client_lost";
    ts_ms: number;
  }
  /** Phase 2 R10. Список доступных топиков в ответ на cmd=stream_list. */
  export interface StreamListEvent {
    type: "stream_list";
    topics: Array<string>;
    ts_ms: number;
  }
  /** Phase 2 §6.2. Подтверждение stream_select: новый (или null) stream_id. */
  export interface StreamSelectAckEvent {
    type: "stream_select_ack";
    topic: string;
    stream_id?: number;
    kind?: string;
  }
  /** Подтверждение SUBSCRIBE с назначенным server-side stream_id. */
  export interface SubscribeAckEvent {
    type: "subscribe_ack";
    topic: string;
    stream_id: number;
    quality: string;
    kind?: string;
  }
  /** Отказ SUBSCRIBE (TOPIC_UNKNOWN / TOPIC_NOT_AVAILABLE_YET / etc). */
  export interface SubscribeNackEvent {
    type: "subscribe_nack";
    topic: string;
    reason: string;
  }
  /** §5.1 / §11.2. Ответ на supervisor_get_state (poll) либо broadcast. */
  export interface SupervisorStateEvent {
    type: "supervisor_state";
    state: Record<string, unknown>;
    ts_ms: number;
  }
  /** issue #2113 / #2112 — Captain Bridge. TARS 1 text panel echo. */
  export interface Tars1TextEvent {
    type: "tars1_text";
    request_id: string;
    text: string;
    streaming: boolean;
    done: boolean;
    ts_ms: number;
  }
  /** issue #2113 — TARS 2 metrics panel: URL Grafana от avatar_supervisor. */
  export interface TarsPanelUrlEvent {
    type: "tars_panel_url";
    request_id: string;
    url: string;
    status: string;
    error: string;
    ts_ms: number;
  }
  /** Phase 2 §4.1. Список VoiceInfo в ответ на cmd=list_voices. AV-27 добавил provider. */
  export interface VoiceListEvent {
    type: "voice_list";
    voices: Array<"VoiceInfo">;
    active_provider?: string;
    active_voice?: string;
    ts_ms: number;
  }
  /** Подтверждение voice_mode. */
  export interface VoiceModeAckEvent {
    type: "voice_mode_ack";
    mode: string;
    ts_ms: number;
  }
  /** Подтверждение voice_pipeline. */
  export interface VoicePipelineAckEvent {
    type: "voice_pipeline_ack";
    ts_ms: number;
  }
  /** Отказ voice_pipeline (rate_limited / invalid_preset / invalid_language / unknown_field). */
  export interface VoicePipelineNackEvent {
    type: "voice_pipeline_nack";
    reason: string;
    ts_ms: number;
  }
  /** Список пресетов/языков, доступных dialogue_node (boot-time UI seed). */
  export interface VoicePresetsEvent {
    type: "voice_presets";
    presets: Array<"VoicePresetInfo">;
    languages: Array<string>;
    default_preset: string;
    default_language: string;
    ts_ms: number;
  }
  /** Подтверждение set_voice. */
  export interface VoiceSetAckEvent {
    type: "voice_set_ack";
    voice_id: string;
    preset: string;
    language: string;
    ts_ms: number;
  }
  /** Отказ set_voice (voice_unavailable / rate_limited / invalid_*). AV-27: available[] подсказка. */
  export interface VoiceSetNackEvent {
    type: "voice_set_nack";
    voice_id?: string;
    preset?: string;
    language?: string;
    reason: string;
    available?: Array<string>;
    ts_ms: number;
  }
  /** §6 — состояние голосового канала робота. */
  export interface VoiceStateEvent {
    type: "voice_state";
    state: "idle" | "listening" | "thinking" | "speaking" | "denied";
    ts_ms: number;
    utterance_id?: string;
    holder_id?: string;
    detail?: string;
  }

/** Discriminated union — every JSON_CMD the server accepts. */
export type JsonCmdGenerated = AdminLogsCmd | AdminLogsStopCmd | ListVoicesCmd | PingCmd | PreviewVoiceCmd | SetPanelTopicCmd | SetVoiceCmd | StopEmergencyCmd | StreamListCmd | StreamSelectCmd | SupervisorAcquireFloorCmd | SupervisorGetStateCmd | SupervisorReleaseFloorCmd | SupervisorSetModeCmd | TeleopHeartbeatCmd | TeleopTwistCmd | UiButtonCmd | VoiceListenStartCmd | VoiceListenStopCmd | VoiceModeCmd | VoicePipelineCmd | VoicePttStartCmd | VoicePttStopCmd;

/** Discriminated union — every JSON_EVENT the server emits. */
export type JsonEventGenerated = AdminLogsChunkEvent | AdminLogsEndEvent | AvatarStateAckEvent | AvatarStateNackEvent | FloorLostEvent | HeartbeatEvent | OperatorTtsAudioEvent | OperatorTtsDoneEvent | OperatorTtsErrorEvent | PingEvent | PongEvent | PreviewVoiceAudioEvent | PreviewVoiceDoneEvent | PreviewVoiceErrorEvent | RobotAlertEvent | SafetyStopEvent | StreamListEvent | StreamSelectAckEvent | SubscribeAckEvent | SubscribeNackEvent | SupervisorStateEvent | Tars1TextEvent | TarsPanelUrlEvent | VoiceListEvent | VoiceModeAckEvent | VoicePipelineAckEvent | VoicePipelineNackEvent | VoicePresetsEvent | VoiceSetAckEvent | VoiceSetNackEvent | VoiceStateEvent;

/** All ``cmd`` discriminant values the server knows. */
export type CommandName = "admin_logs" | "admin_logs_stop" | "list_voices" | "ping" | "preview_voice" | "set_panel_topic" | "set_voice" | "stop_emergency" | "stream_list" | "stream_select" | "supervisor_acquire_floor" | "supervisor_get_state" | "supervisor_release_floor" | "supervisor_set_mode" | "teleop_heartbeat" | "teleop_twist" | "ui_button" | "voice_listen_start" | "voice_listen_stop" | "voice_mode" | "voice_pipeline" | "voice_ptt_start" | "voice_ptt_stop";

/** All ``type`` discriminant values the server emits. */
export type EventName = "admin_logs_chunk" | "admin_logs_end" | "avatar_state_ack" | "avatar_state_nack" | "floor_lost" | "heartbeat" | "operator_tts_audio" | "operator_tts_done" | "operator_tts_error" | "ping" | "pong" | "preview_voice_audio" | "preview_voice_done" | "preview_voice_error" | "robot_alert" | "safety_stop" | "stream_list" | "stream_select_ack" | "subscribe_ack" | "subscribe_nack" | "supervisor_state" | "tars1_text" | "tars_panel_url" | "voice_list" | "voice_mode_ack" | "voice_pipeline_ack" | "voice_pipeline_nack" | "voice_presets" | "voice_set_ack" | "voice_set_nack" | "voice_state";

// Catalog tuples — single source of truth for whitelists.
  export const MODES = ["off", "telegram_active", "avatar_present", "mixed", "teleop_only", "voice_only"] as const;
  export const FLOORS = ["teleop", "voice"] as const;
  export const VOICE_PRESETS = ["standard", "friendly", "authoritative", "whisper", "technical", "street", "caveman", "business", "philosopher", "lenin"] as const;
  export const VOICE_LANGUAGES = ["ru", "en", "fr", "de", "zh", "hi"] as const;
  export const QUALITY_LEVELS = ["low", "med", "high"] as const;
  export const SUBPROTOCOLS_OFFERED = ["v2", "v1"] as const;
  export const ERROR_CODES = [
    { code: "AUTH_FAIL" },
    { code: "BAD_PAYLOAD" },
    { code: "RATE_LIMIT" },
    { code: "TOPIC_UNKNOWN" },
    { code: "TOPIC_NOT_AVAILABLE_YET" },
    { code: "INTERNAL" },
    { code: "PROTOCOL_VERSION" },
    { code: "FLOOR_HELD" },
    { code: "MODE_CONFLICT" }
  ] as const;
  export const STREAMS = [
    { topic: "camera_rear", topic_id: 4097, kind: "camera_direct", default_quality: "med" },
    { topic: "camera_front", topic_id: 4098, kind: "camera_direct", default_quality: "med" },
    { topic: "lidar_2d", topic_id: 4353, kind: "ros_topic", default_quality: "high" },
    { topic: "lidar_3d", topic_id: 4354, kind: "ros_topic", default_quality: "low" },
    { topic: "map_2d", topic_id: 4355, kind: "ros_topic", default_quality: "low" },
    { topic: "robot_status", topic_id: 4609, kind: "ros_topic", default_quality: "high" },
    { topic: "voice_state", topic_id: 4610, kind: "ros_topic", default_quality: "high" },
    { topic: "person_detections", topic_id: 4865, kind: "ros_topic", default_quality: "med" }
  ] as const;
