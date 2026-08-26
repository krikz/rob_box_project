// Типы WSS-сообщений (docs/architecture/meta-quest-api.md §5, §6).
//
// Phase 2.2 — добавляем TELEMETRY_PERF event. Подробный формат payload —
// см. TelemetryPerfPayload ниже.

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

// ----- Telemetry (Phase 2.2) -----------------------------------------------

/**
 * Phase 2.2 telemetry payload (см. ADR-0032 §3.5).
 *
 * Слайд-window агрегаты за 1 с:
 *  - fps_mean / fps_p99: кадры в секунду (средний / 99-й перцентиль)
 *  - frame_time_p99_ms: 99-й перцентиль frame time (мс)
 *  - gpu_ms: среднее GPU time (мс) из EXT_disjoint_timer_query_webgl2
 *  - vram_mb: текстуры + геометрия в renderer.info.memory (MB)
 *  - wss_latency_ms: текущая RTT (мс) — последний HELLO→WELCOME или seq-echo
 *  - resolution_scale: отношение native / requested XR framebuffer
 *  - stale_frames: количество dropped frame за последнюю секунду
 *  - thermal_level: 0..4 (0 = nominal, 4 = critical; из QuestMetrics)
 *  - battery_pct: 0..100 (из getBattery() или QuestMetrics)
 *
 * Поля опциональны — клиент шлёт только то, что смог собрать (XR-сессия
 * или desktop). Сервер принимает partial payload.
 */
export interface TelemetryPerfPayload {
  type: "telemetry_perf";
  ts_ms: number;
  // Source: "desktop" или "webxr" — помогает серверу фильтровать.
  source: "desktop" | "webxr";
  session_id?: string;
  // Sliding-window 1 Hz aggregates:
  fps_mean?: number;
  fps_p99?: number;
  frame_time_p99_ms?: number;
  gpu_ms?: number;
  vram_mb?: number;
  wss_latency_ms?: number;
  resolution_scale?: number;
  stale_frames?: number;
  // Quest-specific (через QuestMetrics / navigator.getBattery):
  thermal_level?: number;
  battery_pct?: number;
  // Sequence для дедупликации на сервере:
  seq?: number;
}

/**
 * Event, который клиент шлёт серверу с throttle 1 Hz через JSON_EVENT.
 * Сервер парсит payload, валидирует seq, пишет в ROS2 /quest/perf.
 */
export type TelemetryEvent = TelemetryPerfPayload;

export type JsonEvent =
  | { type: "heartbeat"; ts_ms: number }
  | { type: "voice_state"; state: string; ts_ms: number; utterance_id?: string }
  | { type: "ping"; ts_ms: number; nonce?: string }
  | { type: "pong"; ts_ms: number; nonce?: string }
  | { type: "supervisor_state"; state: Record<string, unknown>; ts_ms: number }
  | TelemetryEvent
  | { type: string; [k: string]: unknown };

export interface StreamMeta {
  topic: string;
  topic_id: number;
  kind: "ros_topic" | "camera_direct";
  source: string;
  default_quality: string;
  description?: string;
}

// ----- Commands (клиент → сервер) -------------------------------------------

export interface TeleopTwistCmd {
  cmd: "teleop_twist";
  ts_ms: number;
  seq: number;
  linear: { x: number; y: number; z: number };
  angular: { x: number; y: number; z: number };
  deadman: boolean;
}

export type JsonCmd = TeleopTwistCmd | { cmd: string; ts_ms: number; [k: string]: unknown };

/**
 * Извлечь timestamp из события TelemetryPerfPayload.
 */
export function telemetryEventTimestamp(ev: TelemetryEvent): number {
  return ev.ts_ms;
}