// ⚠️  GENERATED FILE — DO NOT EDIT BY HAND.
//
// Source of truth:
//   src/rob_box_core/rob_box_core/bridge_protocol.py
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
  export interface PingCmd {
    cmd: "ping";
    ts_ms: number;
  }
  export interface StreamListCmd {
    cmd: "stream_list";
    ts_ms: number;
  }
  export interface StreamSelectCmd {
    cmd: "stream_select";
    ts_ms: number;
    topic: string;
  }
  export interface TeleopTwistCmd {
    cmd: "teleop_twist";
    ts_ms: number;
    seq?: number;
    linear: {
      x: number;
      y: number;
      z: number;
    };
    angular: {
      x: number;
      y: number;
      z: number;
    };
    deadman: boolean;
  }
  export interface TeleopHeartbeatCmd {
    cmd: "teleop_heartbeat";
    ts_ms: number;
    seq: number;
  }
  export interface StopEmergencyCmd {
    cmd: "stop_emergency";
    ts_ms: number;
    source?: "controller_b" | "ui_button" | "client_lost";
  }
  export interface VoicePttStartCmd {
    cmd: "voice_ptt_start";
    ts_ms: number;
    client_id?: string;
    mode?: "radio" | "robot_voice";
  }
  export interface VoicePttStopCmd {
    cmd: "voice_ptt_stop";
    ts_ms: number;
    mode?: "radio" | "robot_voice";
  }
  export interface VoiceModeCmd {
    cmd: "voice_mode";
    ts_ms: number;
    mode: "off" | "passthrough" | "ttts_proxy" | "stt_llm" | "llm_formalize";
  }
  export interface VoiceListenStartCmd {
    cmd: "voice_listen_start";
    ts_ms: number;
  }
  export interface VoiceListenStopCmd {
    cmd: "voice_listen_stop";
    ts_ms: number;
  }
  export interface SupervisorSetModeCmd {
    cmd: "supervisor_set_mode";
    ts_ms: number;
    client_id: string;
    mode: "off" | "telegram_active" | "avatar_present" | "mixed" | "teleop_only" | "voice_only";
  }
  export interface SupervisorAcquireFloorCmd {
    cmd: "supervisor_acquire_floor";
    ts_ms: number;
    client_id: string;
    floor: "teleop" | "voice";
  }
  export interface SupervisorReleaseFloorCmd {
    cmd: "supervisor_release_floor";
    ts_ms: number;
    client_id: string;
    floor: "teleop" | "voice";
  }
  export interface SupervisorGetStateCmd {
    cmd: "supervisor_get_state";
    ts_ms: number;
  }
  export interface ListVoicesCmd {
    cmd: "list_voices";
    ts_ms: number;
  }
  export interface SetVoiceCmd {
    cmd: "set_voice";
    ts_ms: number;
    voice_id?: string;
    preset?: "standard" | "friendly" | "authoritative" | "whisper" | "technical" | "street" | "caveman" | "business" | "philosopher" | "lenin";
    language?: "ru" | "en";
  }
  export interface VoicePipelineCmd {
    cmd: "voice_pipeline";
    ts_ms: number;
    llm_enabled?: boolean;
    preset?: string;
    language?: "ru" | "en" | "fr" | "de" | "zh" | "hi";
  }
  export interface PreviewVoiceCmd {
    cmd: "preview_voice";
    ts_ms: number;
    voice_id: string;
    text: string;
    request_id: string;
  }
  export interface AvatarSetModeCmd {
    cmd: "avatar_set_mode";
    ts_ms: number;
    mode: string;
    reason?: string;
  }
  export interface AvatarAcquireFloorCmd {
    cmd: "avatar_acquire_floor";
    ts_ms: number;
    kind: string;
  }
  export interface AvatarReleaseFloorCmd {
    cmd: "avatar_release_floor";
    ts_ms: number;
    kind: string;
  }
  export interface SetPanelTopicCmd {
    cmd: "set_panel_topic";
    ts_ms: number;
    panel_id: string;
    topic: string;
  }
  export interface UiButtonCmd {
    cmd: "ui_button";
    ts_ms: number;
    button: string;
    press: boolean;
  }
  export interface AdminLogsCmd {
    cmd: "admin_logs";
    ts_ms: number;
    service: "dialogue_node" | "rob_box_quest" | "all";
    tail?: number;
    follow?: boolean;
  }
  export interface AdminLogsStopCmd {
    cmd: "admin_logs_stop";
    ts_ms: number;
  }

// JSON_EVENT — server → client (meta-quest-api.md §6)
  export interface SubscribeAckEvent {
    type: "subscribe_ack";
    topic: string;
    stream_id: number;
    quality: string;
    kind?: string;
  }
  export interface SubscribeNackEvent {
    type: "subscribe_nack";
    topic: string;
    reason: string;
  }
  export interface HeartbeatEvent {
    type: "heartbeat";
    ts_ms: number;
  }
  export interface PingEvent {
    type: "ping";
    ts_ms: number;
    nonce?: string;
  }
  export interface PongEvent {
    type: "pong";
    ts_ms: number;
    nonce?: string;
  }
  export interface StreamListEvent {
    type: "stream_list";
    topics: Array<string>;
    ts_ms: number;
  }
  export interface StreamSelectAckEvent {
    type: "stream_select_ack";
    topic: string;
    stream_id?: number;
    kind?: string;
  }
  export interface VoiceStateEvent {
    type: "voice_state";
    state: "idle" | "listening" | "thinking" | "speaking" | "denied";
    ts_ms: number;
    utterance_id?: string;
    holder_id?: string;
    detail?: string;
  }
  export interface VoiceModeAckEvent {
    type: "voice_mode_ack";
    mode: string;
    ts_ms: number;
  }
  export interface VoiceListenAckEvent {
    type: "voice_listen_ack";
    active: boolean;
    ts_ms: number;
  }
  export interface VoiceListEvent {
    type: "voice_list";
    voices: Array<"VoiceInfo">;
    active_provider?: string;
    active_voice?: string;
    ts_ms: number;
  }
  export interface VoiceSetAckEvent {
    type: "voice_set_ack";
    voice_id: string;
    preset: string;
    language: string;
    ts_ms: number;
  }
  export interface VoiceSetNackEvent {
    type: "voice_set_nack";
    voice_id?: string;
    preset?: string;
    language?: string;
    reason: string;
    available?: Array<string>;
    ts_ms: number;
  }
  export interface VoicePipelineAckEvent {
    type: "voice_pipeline_ack";
    ts_ms: number;
  }
  export interface VoicePipelineNackEvent {
    type: "voice_pipeline_nack";
    reason: string;
    ts_ms: number;
  }
  export interface PreviewVoiceAudioEvent {
    type: "preview_voice_audio";
    request_id: string;
    format: "mp3" | "opus" | "wav";
    content_type: string;
    seq: number;
    total: number;
    ts_ms: number;
  }
  export interface PreviewVoiceDoneEvent {
    type: "preview_voice_done";
    request_id: string;
    ts_ms: number;
  }
  export interface PreviewVoiceErrorEvent {
    type: "preview_voice_error";
    request_id: string;
    reason: string;
    ts_ms: number;
  }
  export interface SupervisorStateEvent {
    type: "supervisor_state";
    state: Record<string, unknown>;
    ts_ms: number;
  }
  export interface SafetyStopEvent {
    type: "safety_stop";
    reason: "controller_b" | "client_lost";
    ts_ms: number;
  }
  export interface RobotAlertEvent {
    type: "robot_alert";
    code: string;
    level: "warn" | "error" | "info";
    active?: boolean;
    args?: Record<string, unknown>;
    ts_ms: number;
  }
  export interface FloorLostEvent {
    type: "floor_lost";
    floor: "teleop" | "voice";
    reason?: string;
    ts_ms: number;
  }
  export interface AdminLogsChunkEvent {
    type: "admin_logs_chunk";
    service: "dialogue_node" | "rob_box_quest" | "all";
    lines: Array<string>;
    ts_ms: number;
  }
  export interface AdminLogsEndEvent {
    type: "admin_logs_end";
    service: string;
    ts_ms: number;
  }

/** Discriminated union — every JSON_CMD the server accepts. */
export type JsonCmdGenerated = PingCmd | StreamListCmd | StreamSelectCmd | TeleopTwistCmd | TeleopHeartbeatCmd | StopEmergencyCmd | VoicePttStartCmd | VoicePttStopCmd | VoiceModeCmd | VoiceListenStartCmd | VoiceListenStopCmd | SupervisorSetModeCmd | SupervisorAcquireFloorCmd | SupervisorReleaseFloorCmd | SupervisorGetStateCmd | ListVoicesCmd | SetVoiceCmd | VoicePipelineCmd | PreviewVoiceCmd | AvatarSetModeCmd | AvatarAcquireFloorCmd | AvatarReleaseFloorCmd | SetPanelTopicCmd | UiButtonCmd | AdminLogsCmd | AdminLogsStopCmd;

/** Discriminated union — every JSON_EVENT the server emits. */
export type JsonEventGenerated = SubscribeAckEvent | SubscribeNackEvent | HeartbeatEvent | PingEvent | PongEvent | StreamListEvent | StreamSelectAckEvent | VoiceStateEvent | VoiceModeAckEvent | VoiceListenAckEvent | VoiceListEvent | VoiceSetAckEvent | VoiceSetNackEvent | VoicePipelineAckEvent | VoicePipelineNackEvent | PreviewVoiceAudioEvent | PreviewVoiceDoneEvent | PreviewVoiceErrorEvent | SupervisorStateEvent | SafetyStopEvent | RobotAlertEvent | FloorLostEvent | AdminLogsChunkEvent | AdminLogsEndEvent;

/** All ``cmd`` discriminant values the server knows. */
export type CommandName = "ping" | "stream_list" | "stream_select" | "teleop_twist" | "teleop_heartbeat" | "stop_emergency" | "voice_ptt_start" | "voice_ptt_stop" | "voice_mode" | "voice_listen_start" | "voice_listen_stop" | "supervisor_set_mode" | "supervisor_acquire_floor" | "supervisor_release_floor" | "supervisor_get_state" | "list_voices" | "set_voice" | "voice_pipeline" | "preview_voice" | "avatar_set_mode" | "avatar_acquire_floor" | "avatar_release_floor" | "set_panel_topic" | "ui_button" | "admin_logs" | "admin_logs_stop";

/** All ``type`` discriminant values the server emits. */
export type EventName = "subscribe_ack" | "subscribe_nack" | "heartbeat" | "ping" | "pong" | "stream_list" | "stream_select_ack" | "voice_state" | "voice_mode_ack" | "voice_listen_ack" | "voice_list" | "voice_set_ack" | "voice_set_nack" | "voice_pipeline_ack" | "voice_pipeline_nack" | "preview_voice_audio" | "preview_voice_done" | "preview_voice_error" | "supervisor_state" | "safety_stop" | "robot_alert" | "floor_lost" | "admin_logs_chunk" | "admin_logs_end";

// Catalog tuples — single source of truth for whitelists.
  export const MODES = ["off", "telegram_active", "avatar_present", "mixed", "teleop_only", "voice_only"] as const;
  export const FLOORS = ["teleop", "voice"] as const;
  export const VOICE_PRESETS = ["technical", "street", "caveman", "business", "philosopher", "lenin", "translate"] as const;
  export const VOICE_LANGUAGES = ["ru", "en", "fr", "de", "zh", "hi"] as const;
  export const SUBPROTOCOLS_OFFERED = ["v1", "v2"] as const;

  export const ERROR_CODES = ["AUTH_FAIL", "BAD_PAYLOAD", "TOPIC_UNKNOWN", "RATE_LIMIT", "PROTOCOL_VERSION", "FLOOR_HELD", "MODE_CONFLICT", "INTERNAL", "UNKNOWN_COMMAND"] as const;
