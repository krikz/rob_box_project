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

export interface StreamSelectCmd {
  cmd: "stream_select";
  ts_ms: number;
  topic: string;
}

export interface StreamListCmd {
  cmd: "stream_list";
  ts_ms: number;
}

export type JsonCmd =
  | TeleopTwistCmd
  | StopEmergencyCmd
  | StreamSelectCmd
  | StreamListCmd
  | { cmd: string; ts_ms: number; [k: string]: unknown };

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