// Мок-робот для симулятора мостика (issue #3149): серверная сторона
// НАСТОЯЩЕГО протокола rob_box_quest прямо в браузере, без бэкенда.
//
// Клиент подключается к нему обычным `Connection` — через штатный шов
// `ConnectionOptions.WebSocketCtor` (тот же, которым пользуются тесты с
// FakeWebSocket). Никакой логики соединения тут не дублируется: мок
// принимает кадры wire-формата (wire/protocol.ts) и отвечает кадрами того
// же формата, как ws_server.py.
//
// Что умеет (зеркало ws_server.py):
//   HELLO → WELCOME (или ERROR{AUTH_FAIL}, если задан `pin` и он не совпал);
//   JSON_EVENT ping → pong с эхом ts_ms;
//   SUBSCRIBE → subscribe_ack{stream_id} | ERROR{TOPIC_UNKNOWN};
//   UNSUBSCRIBE; JSON_CMD по таблице `commands` (unknown → ERROR{UNKNOWN_COMMAND});
//   SET_MODE / ACQUIRE_FLOOR / RELEASE_FLOOR → STATE_UPDATE;
//   SUBSCRIBE.max_hz (#3150) — прореживание стрима + эхо в subscribe_ack;
//   nav_goal / nav_cancel + стрим nav_path (#3151) — sim_nav.ts.
//
// Расширение (Nav2, max_hz — отдельными карточками):
//   * новая команда — `robot.registerCommand("nav_goal", handler)`;
//   * новый стрим — `robot.registerStream("nav_path", source)`;
//   * частота стрима считается в одном месте — `subscriptionPeriodMs()`,
//     туда и ляжет `max_hz` из SUBSCRIBE (запись подписки уже хранит
//     исходное сообщение целиком).

import {
  decodeFrame,
  encodeFrame,
  encodeJsonFrame,
  FrameType,
  type DecodedFrame
} from "../wire/protocol";
import { decodeMsgpackMap } from "../wire/msgpack";
import {
  encodeLidar2d,
  encodeMap2d,
  encodeRobotStatus,
  encodeSupervisorState,
  encodeVoiceState,
  gridToPng
} from "./sim_encoders";
import {
  createDefaultWorld,
  createGrid,
  createRobotState,
  integrateScan,
  scanLidar,
  stepRobot,
  type SimGrid,
  type SimRobotState,
  type SimScan,
  type SimWorld
} from "./sim_world";
import { cameraKindForTopic, type CameraRenderer } from "./sim_camera";
import { SimNavigator } from "./sim_nav";
import { encodeNavPath } from "./sim_encoders";

// ────────────────────────── типы расширения ──────────────────────────

export interface MockSubscription {
  topic: string;
  streamId: number;
  quality: string;
  /** Исходный SUBSCRIBE целиком. */
  request: Record<string, unknown>;
  /** SUBSCRIBE.max_hz после нормализации (parseMaxHz); null — без лимита. */
  maxHz: number | null;
  nextDueMs: number;
  /** Кадр ещё кодируется (async-камера) — следующий не начинаем. */
  busy: boolean;
  /** Память стрима между кадрами (например, какую ревизию карты уже слали). */
  memo: Record<string, unknown>;
}

export interface MockStreamSource {
  topicId: number;
  kind: "ros_topic" | "camera_direct";
  source: string;
  description: string;
  defaultQuality: "low" | "med" | "high";
  /** Номинальная частота, Гц; 0 — только по событию (onSubscribe/push). */
  rateHz: number;
  produce?(
    robot: MockRobot,
    sub: MockSubscription
  ): Uint8Array | null | Promise<Uint8Array | null>;
  /** Снапшот сразу после subscribe_ack (как voice_state у сервера). */
  onSubscribe?(robot: MockRobot, session: MockSession, sub: MockSubscription): void;
}

export interface MockCommandContext {
  robot: MockRobot;
  session: MockSession;
  cmd: Record<string, unknown>;
}

export type MockCommandHandler = (ctx: MockCommandContext) => void;

export interface MockRobotOptions {
  world?: SimWorld;
  /** Часы (мс). По умолчанию Date.now. */
  now?: () => number;
  /** Рендер камер; `null` — видеокадров нет (jsdom). */
  cameraRenderer?: CameraRenderer | null;
  /** Если задан — HELLO с другим PIN получает AUTH_FAIL. По умолчанию любой. */
  pin?: string | null;
  /** Задержка доставки кадров в одну сторону, мс (для правдоподобного RTT). */
  latencyMs?: number;
  /** Шаг физики, мс. */
  physicsStepMs?: number;
}

/** Сколько держится E-STOP в симуляторе (у робота — до нового HELLO). */
export const SIM_EMERGENCY_HOLD_MS = 3000;

const SERVER_STREAM_ID_BASE = 0x1000;

// ────────────────────────── сессия (сервер одной вкладки) ──────────────────────────

export class MockSession {
  readonly sessionId: string;
  authenticated = false;
  /** Когда последний раз слали периодический STATE_UPDATE. */
  lastStateUpdateMs = 0;
  readonly subscriptions = new Map<string, MockSubscription>();
  private closed = false;

  constructor(
    private readonly robot: MockRobot,
    private readonly deliver: (bytes: Uint8Array) => void,
    private readonly closeSocket: (code: number, reason: string) => void
  ) {
    this.sessionId = `sim-${Math.random().toString(16).slice(2, 10)}`;
  }

  isClosed(): boolean {
    return this.closed;
  }

  markClosed(): void {
    this.closed = true;
    this.subscriptions.clear();
  }

  sendFrame(type: FrameType, streamId: number, payload: Uint8Array): void {
    if (this.closed) return;
    this.deliver(encodeFrame(type, streamId, payload));
  }

  sendJson(type: FrameType, streamId: number, obj: unknown): void {
    if (this.closed) return;
    this.deliver(encodeJsonFrame(type, streamId, obj));
  }

  sendEvent(event: Record<string, unknown>): void {
    this.sendJson(FrameType.JSON_EVENT, 0, { ts_ms: this.robot.now(), ...event });
  }

  sendError(code: string, message: string): void {
    this.sendJson(FrameType.ERROR, 0, { code, message });
  }

  handleFrame(raw: Uint8Array): void {
    if (this.closed) return;
    let frame: DecodedFrame;
    try {
      frame = decodeFrame(raw);
    } catch (err) {
      this.sendError("BAD_PAYLOAD", (err as Error).message);
      return;
    }
    if (!this.authenticated) {
      if (frame.type !== FrameType.HELLO) {
        this.sendError("BAD_PAYLOAD", "expected HELLO");
        return;
      }
      this.onHello(frame);
      return;
    }
    switch (frame.type) {
      case FrameType.SUBSCRIBE:
        this.onSubscribe(parseJson(frame));
        return;
      case FrameType.UNSUBSCRIBE: {
        const topic = parseJson(frame).topic;
        if (typeof topic === "string") this.subscriptions.delete(topic);
        return;
      }
      case FrameType.JSON_EVENT: {
        const ev = parseJson(frame);
        if (ev.type === "ping") {
          this.sendEvent({ type: "pong", ts_ms: ev.ts_ms ?? null, server_ts_ms: this.robot.now() });
        }
        return;
      }
      case FrameType.JSON_CMD:
        this.robot.dispatchCommand(this, parseJson(frame));
        return;
      case FrameType.SET_MODE:
      case FrameType.ACQUIRE_FLOOR:
      case FrameType.RELEASE_FLOOR:
        this.robot.applySupervisorFrame(this, frame.type, decodeMsgpackMap(frame.payload) ?? {});
        return;
      case FrameType.VOICE_AUDIO:
        // Голос оператора симулятору некуда девать — молча глотаем, как
        // NoOpBridge сервера.
        return;
      case FrameType.GOODBYE:
        this.closeSocket(1000, "goodbye");
        return;
      default:
        this.sendError("BAD_PAYLOAD", `unexpected frame type 0x${frame.type.toString(16)}`);
    }
  }

  private onHello(frame: DecodedFrame): void {
    const hello = parseJson(frame);
    const pin = hello.session_pin;
    if (typeof pin !== "string" || pin.length === 0) {
      this.sendError("BAD_PAYLOAD", "session_pin required");
      return;
    }
    const expected = this.robot.pin;
    if (expected !== null && pin !== expected) {
      this.sendError("AUTH_FAIL", "wrong PIN");
      return;
    }
    this.authenticated = true;
    this.robot.onSessionHello(this);
    this.sendJson(FrameType.WELCOME, 0, {
      server_version: "sim-0.1.0",
      session_id: this.sessionId,
      server_time_ms: this.robot.now(),
      teleop_floor_held_by: this.sessionId
    });
    this.robot.sendSupervisorState(this);
  }

  private onSubscribe(msg: Record<string, unknown>): void {
    const topic = msg.topic;
    const source = typeof topic === "string" ? this.robot.getStream(topic) : undefined;
    if (typeof topic !== "string" || !source) {
      this.sendError("TOPIC_UNKNOWN", `topic '${String(topic)}' not in registry`);
      return;
    }
    const q = msg.quality;
    const quality = q === "low" || q === "med" || q === "high" ? q : source.defaultQuality;
    // Как ws_server: повторный SUBSCRIBE идемпотентен по stream_id, но
    // обновляет лимит частоты (отсутствие max_hz = снять лимит).
    const maxHz = parseMaxHz(msg.max_hz);
    let sub = this.subscriptions.get(topic);
    if (!sub) {
      sub = {
        topic,
        streamId: this.robot.allocateStreamId(),
        quality,
        request: msg,
        maxHz,
        nextDueMs: 0,
        busy: false,
        memo: {}
      };
      this.subscriptions.set(topic, sub);
    } else {
      sub.request = msg;
      sub.maxHz = maxHz;
      sub.quality = quality;
    }
    const ack: Record<string, unknown> = {
      type: "subscribe_ack",
      topic,
      stream_id: sub.streamId,
      quality,
      kind: source.kind
    };
    if (maxHz !== null) ack.max_hz = maxHz;
    this.sendEvent(ack);
    source.onSubscribe?.(this.robot, this, sub);
  }
}

function parseJson(frame: DecodedFrame): Record<string, unknown> {
  try {
    const v = JSON.parse(new TextDecoder().decode(frame.payload));
    return v && typeof v === "object" && !Array.isArray(v) ? (v as Record<string, unknown>) : {};
  } catch {
    return {};
  }
}

// ────────────────────────── робот ──────────────────────────

export class MockRobot {
  readonly world: SimWorld;
  readonly state: SimRobotState;
  readonly grid: SimGrid;
  readonly pin: string | null;
  readonly latencyMs: number;
  readonly now: () => number;
  cameraRenderer: CameraRenderer | null;

  /** Последний скан лидара (обновляется в `tick` на 10 Гц). */
  lastScan: SimScan;
  batteryPct = 87;
  voiceState: "idle" | "listening" | "thinking" | "speaking" = "idle";
  emergencyUntilMs = 0;
  supervisorMode = "avatar_present";
  supervisorVersion = 1;
  supervisorSinceMs: number;
  /** Nav2 симулятора (sim_nav.ts): цель, план, pure-pursuit. */
  readonly nav: SimNavigator;
  /** Последний teleop_twist — телеоп перебивает навигацию (twist_mux 90 > 10). */
  lastTeleopMs = -Infinity;

  private readonly sessions = new Set<MockSession>();
  private readonly commands = new Map<string, MockCommandHandler>();
  private readonly streams = new Map<string, MockStreamSource>();
  private nextStreamId = SERVER_STREAM_ID_BASE;
  private timer: ReturnType<typeof setInterval> | null = null;
  private lastTickMs = 0;
  private lastScanMs = -Infinity;
  private readonly physicsStepMs: number;

  constructor(opts: MockRobotOptions = {}) {
    this.world = opts.world ?? createDefaultWorld();
    this.now = opts.now ?? (() => Date.now());
    this.state = createRobotState(this.world);
    this.grid = createGrid(this.world);
    this.pin = opts.pin ?? null;
    this.latencyMs = opts.latencyMs ?? 0;
    this.cameraRenderer = opts.cameraRenderer ?? null;
    this.physicsStepMs = opts.physicsStepMs ?? 20;
    this.supervisorSinceMs = this.now();
    this.lastScan = scanLidar(this.world, this.state);
    integrateScan(this.grid, this.state, this.lastScan);
    this.nav = new SimNavigator({
      grid: this.grid,
      pose: () => this.state,
      drive: (v, w) => this.setCommandVelocity(v, w),
      emit: (ev) => this.broadcastEvent(ev)
    });
    for (const [name, h] of Object.entries(DEFAULT_COMMANDS)) this.commands.set(name, h);
    for (const [topic, s] of Object.entries(DEFAULT_STREAMS)) this.streams.set(topic, s);
  }

  // ── расширение ──
  registerCommand(name: string, handler: MockCommandHandler): void {
    this.commands.set(name, handler);
  }

  registerStream(topic: string, source: MockStreamSource): void {
    this.streams.set(topic, source);
  }

  getStream(topic: string): MockStreamSource | undefined {
    return this.streams.get(topic);
  }

  streamTopics(): string[] {
    return [...this.streams.keys()];
  }

  // ── жизненный цикл ──
  start(): void {
    if (this.timer) return;
    this.lastTickMs = this.now();
    this.timer = setInterval(() => this.tick(), this.physicsStepMs);
  }

  stop(): void {
    if (this.timer) clearInterval(this.timer);
    this.timer = null;
  }

  attach(session: MockSession): void {
    this.sessions.add(session);
  }

  detach(session: MockSession): void {
    session.markClosed();
    this.sessions.delete(session);
  }

  sessionCount(): number {
    return this.sessions.size;
  }

  /** JSON_EVENT всем подключённым сессиям (nav_status у сервера — так же). */
  broadcastEvent(event: Record<string, unknown>): void {
    for (const s of this.sessions) if (s.authenticated) s.sendEvent(event);
  }

  allocateStreamId(): number {
    const sid = this.nextStreamId;
    this.nextStreamId = sid >= 0xffff ? SERVER_STREAM_ID_BASE : sid + 1;
    return sid;
  }

  onSessionHello(_session: MockSession): void {
    // Как bridge.reset() сервера на HELLO: новая сессия снимает E-STOP.
    this.emergencyUntilMs = 0;
  }

  isEmergency(): boolean {
    return this.now() < this.emergencyUntilMs;
  }

  mode(): string {
    if (this.isEmergency()) return "emergency_stop";
    const moving = Math.abs(this.state.v) > 0.01 || Math.abs(this.state.w) > 0.01;
    // Едет Nav2, а не оператор — как у сервера (quest_node: teleop_active
    // только от телеопа), это не «teleop».
    if (moving && (this.teleopOverridesNav() || !this.nav.activeGoal())) return "teleop";
    return "idle";
  }

  /** RSSI падает с удалением от «точки доступа» в комнате A. */
  wifiRssi(): number {
    const d = Math.hypot(this.state.x - 4, this.state.y - 3);
    return Math.round(-38 - 18 * Math.log10(Math.max(1, d)) + (Math.random() - 0.5) * 3);
  }

  // ── команды ──
  dispatchCommand(session: MockSession, cmd: Record<string, unknown>): void {
    const name = cmd.cmd;
    const handler = typeof name === "string" ? this.commands.get(name) : undefined;
    if (!handler) {
      session.sendError("UNKNOWN_COMMAND", `unknown JSON_CMD: '${String(name)}' (sim)`);
      return;
    }
    handler({ robot: this, session, cmd });
  }

  setCommandVelocity(v: number, w: number): void {
    if (this.isEmergency()) return;
    this.state.cmdV = v;
    this.state.cmdW = w;
    this.state.lastCmdMs = this.now();
  }

  /** teleop_twist оператора (в отличие от команд навигатора). */
  teleopCommand(v: number, w: number): void {
    this.lastTeleopMs = this.now();
    this.setCommandVelocity(v, w);
  }

  /** Телеоп сейчас перебивает навигацию (twist_mux: quest timeout 0.5 с). */
  teleopOverridesNav(): boolean {
    return this.now() - this.lastTeleopMs <= TELEOP_MUX_TIMEOUT_MS;
  }

  emergencyStop(): void {
    // quest_node.emergency_stop (#3151): аварийный стоп отменяет и nav-цель.
    this.nav.cancel(this.now());
    this.emergencyUntilMs = this.now() + SIM_EMERGENCY_HOLD_MS;
    this.state.cmdV = 0;
    this.state.cmdW = 0;
    this.state.v = 0;
    this.state.w = 0;
  }

  // ── supervisor ──
  sendSupervisorState(session: MockSession): void {
    session.sendFrame(
      FrameType.STATE_UPDATE,
      0,
      encodeSupervisorState({
        mode: this.supervisorMode,
        teleopHolder: session.sessionId,
        voiceHolder: null,
        sinceMs: this.supervisorSinceMs,
        lastEvent: "sim",
        version: this.supervisorVersion
      })
    );
  }

  applySupervisorFrame(session: MockSession, type: FrameType, body: Record<string, unknown>): void {
    if (type === FrameType.SET_MODE && typeof body.mode === "string") {
      this.setSupervisorMode(body.mode);
    }
    this.sendSupervisorState(session);
  }

  setSupervisorMode(mode: string): void {
    this.supervisorMode = mode;
    this.supervisorVersion += 1;
    this.supervisorSinceMs = this.now();
  }

  // ── такт ──
  tick(): void {
    const now = this.now();
    const dt = Math.min(0.1, Math.max(0, (now - this.lastTickMs) / 1000));
    this.lastTickMs = now;
    if (this.isEmergency()) {
      this.state.cmdV = 0;
      this.state.cmdW = 0;
    } else {
      this.nav.step(now, this.teleopOverridesNav());
    }
    stepRobot(this.world, this.state, dt, now);
    const moving = Math.abs(this.state.v) + Math.abs(this.state.w) > 0.01;
    this.batteryPct = Math.max(5, this.batteryPct - dt * (moving ? 0.05 : 0.01));
    if (now - this.lastScanMs >= 100) {
      this.lastScanMs = now;
      this.lastScan = scanLidar(this.world, this.state);
      integrateScan(this.grid, this.state, this.lastScan);
    }
    for (const session of this.sessions) this.pumpStreams(session, now);
  }

  private pumpStreams(session: MockSession, now: number): void {
    for (const sub of session.subscriptions.values()) {
      const source = this.streams.get(sub.topic);
      if (!source?.produce) continue;
      const period = subscriptionPeriodMs(source, sub);
      if (period <= 0 || sub.busy || now < sub.nextDueMs) continue;
      sub.nextDueMs = now + period;
      const out = source.produce(this, sub);
      if (out instanceof Promise) {
        sub.busy = true;
        out.then(
          (bytes) => {
            sub.busy = false;
            if (bytes && session.subscriptions.get(sub.topic) === sub) {
              session.sendFrame(FrameType.BINARY_FRAME, sub.streamId, bytes);
            }
          },
          () => {
            sub.busy = false;
          }
        );
      } else if (out) {
        session.sendFrame(FrameType.BINARY_FRAME, sub.streamId, out);
      }
    }
    // Supervisor-снапшот раз в 5 с (как периодический STATE_UPDATE).
    if (now - session.lastStateUpdateMs >= 5000) {
      session.lastStateUpdateMs = now;
      this.sendSupervisorState(session);
    }
  }
}

/**
 * Период кадров подписки, мс. Единственное место, где считается частота:
 * номинал стрима, прореженный до SUBSCRIBE.max_hz (#3150), если лимит
 * строже номинала. Событийные стримы (rateHz = 0) лимит не включает.
 */
export function subscriptionPeriodMs(source: MockStreamSource, sub: MockSubscription): number {
  if (source.rateHz <= 0) return 0;
  const hz = sub.maxHz !== null ? Math.min(source.rateHz, sub.maxHz) : source.rateHz;
  return 1000 / hz;
}

/** Потолок max_hz (stream_rate.py MAX_HZ_CEILING). */
export const MAX_HZ_CEILING = 120;

/**
 * SUBSCRIBE.max_hz → Гц или null (без лимита) — зеркало stream_rate.py
 * parse_max_hz: null для отсутствия, bool, не-числа, NaN/inf, ≤ 0, > 120.
 */
export function parseMaxHz(raw: unknown): number | null {
  if (typeof raw !== "number" || !Number.isFinite(raw)) return null;
  if (raw <= 0 || raw > MAX_HZ_CEILING) return null;
  return raw;
}

/** twist_mux: quest timeout 0.5 с — столько телеоп держит приоритет над nav. */
export const TELEOP_MUX_TIMEOUT_MS = 500;

// ────────────────────────── таблица команд ──────────────────────────

function num(v: unknown): number {
  return typeof v === "number" && Number.isFinite(v) ? v : 0;
}

const SIM_VOICES = [
  { voice_id: "sim_robot", display_name: "Робот (sim)", language: "ru", gender: "neutral", provider: "sim" },
  { voice_id: "sim_alt", display_name: "Второй голос (sim)", language: "ru", gender: "female", provider: "sim" }
];

export const DEFAULT_COMMANDS: Readonly<Record<string, MockCommandHandler>> = Object.freeze({
  ping: ({ session, cmd }) => session.sendEvent({ type: "pong", ts_ms: cmd.ts_ms ?? null }),
  stream_list: ({ robot, session }) =>
    session.sendEvent({
      type: "stream_list",
      items: robot.streamTopics().map((topic) => {
        const s = robot.getStream(topic)!;
        return {
          topic,
          topic_id: s.topicId,
          kind: s.kind,
          source: s.source,
          default_quality: s.defaultQuality,
          description: s.description
        };
      })
    }),
  stream_select: ({ robot, session, cmd }) => {
    const topic = cmd.topic;
    const src = typeof topic === "string" ? robot.getStream(topic) : undefined;
    if (!src || typeof topic !== "string") {
      session.sendError("TOPIC_UNKNOWN", `topic '${String(topic)}' not in registry`);
      return;
    }
    session.sendEvent({
      type: "stream_select_ack",
      topic,
      stream_id: session.subscriptions.get(topic)?.streamId ?? null,
      kind: src.kind
    });
  },
  teleop_twist: ({ robot, cmd }) => {
    const lin = (cmd.linear ?? {}) as Record<string, unknown>;
    const ang = (cmd.angular ?? {}) as Record<string, unknown>;
    // deadman=false → стоп (FSM шлёт такой twist при отпускании Space).
    if (cmd.deadman === false) robot.teleopCommand(0, 0);
    else robot.teleopCommand(num(lin.x), num(ang.z));
  },
  teleop_heartbeat: () => undefined,
  stop_emergency: ({ robot }) => robot.emergencyStop(),
  list_voices: ({ session }) =>
    session.sendEvent({
      type: "voice_list",
      voices: SIM_VOICES,
      active_provider: "sim",
      active_voice: SIM_VOICES[0].voice_id
    }),
  set_voice: ({ session, cmd }) =>
    session.sendEvent({
      type: "voice_set_ack",
      voice_id: String(cmd.voice_id ?? SIM_VOICES[0].voice_id),
      preset: String(cmd.preset ?? "technical"),
      language: String(cmd.language ?? "ru")
    }),
  preview_voice: ({ session, cmd }) =>
    session.sendEvent({
      type: "preview_voice_error",
      request_id: String(cmd.request_id ?? ""),
      reason: "sim: нет синтеза речи в симуляторе"
    }),
  voice_mode: ({ session, cmd }) => session.sendEvent({ type: "voice_mode_ack", mode: String(cmd.mode ?? "off") }),
  voice_pipeline: ({ session }) => session.sendEvent({ type: "voice_pipeline_ack" }),
  voice_listen_start: ({ session }) => session.sendEvent({ type: "voice_listen_ack", active: true }),
  voice_listen_stop: ({ session }) => session.sendEvent({ type: "voice_listen_ack", active: false }),
  voice_ptt_start: ({ robot, session }) => {
    robot.voiceState = "listening";
    session.sendEvent({ type: "voice_state", state: "listening", holder_id: session.sessionId });
  },
  voice_ptt_stop: ({ robot, session }) => {
    robot.voiceState = "idle";
    session.sendEvent({ type: "voice_state", state: "idle" });
  },
  supervisor_set_mode: supervisorCmd,
  supervisor_acquire_floor: supervisorCmd,
  supervisor_release_floor: supervisorCmd,
  supervisor_get_state: supervisorCmd,
  avatar_set_mode: supervisorCmd,
  avatar_acquire_floor: supervisorCmd,
  avatar_release_floor: supervisorCmd,
  nav_goal: navGoalCmd,
  nav_cancel: ({ robot, session }) => {
    const hadGoal = robot.nav.cancel(robot.now());
    session.sendEvent({ type: "nav_cancel_ack", had_goal: hadGoal });
  }
});

/** Причины nav_goal_nack (core/nav_goal.py NACK_*). */
export const NAV_NACK = Object.freeze({
  badPayload: "bad_payload",
  badFrame: "bad_frame",
  emergency: "emergency_active"
});

/**
 * nav_goal (ws_server._json_cmd_nav_goal + parse_nav_goal): frame обязан
 * быть "map", seq — целое, x/y/yaw — конечные числа; E-STOP → nack. Floor
 * у сессии симулятора всегда свой, floor_held не бывает.
 */
function navGoalCmd({ robot, session, cmd }: MockCommandContext): void {
  const rawSeq = cmd.seq;
  const seq = typeof rawSeq === "number" && Number.isInteger(rawSeq) ? rawSeq : null;
  const nack = (reason: string): void => {
    const ev: Record<string, unknown> = { type: "nav_goal_nack", reason };
    if (seq !== null) ev.seq = seq;
    session.sendEvent(ev);
  };
  if (cmd.frame !== "map") return nack(NAV_NACK.badFrame);
  const finite = (v: unknown): v is number => typeof v === "number" && Number.isFinite(v);
  if (seq === null || !finite(cmd.x) || !finite(cmd.y) || !finite(cmd.yaw)) return nack(NAV_NACK.badPayload);
  if (robot.isEmergency()) return nack(NAV_NACK.emergency);
  const yaw = Math.atan2(Math.sin(cmd.yaw), Math.cos(cmd.yaw));
  robot.nav.start({ seq, x: cmd.x, y: cmd.y, yaw: yaw === -Math.PI ? Math.PI : yaw }, robot.now());
  session.sendEvent({ type: "nav_goal_ack", seq });
}

function supervisorCmd({ robot, session, cmd }: MockCommandContext): void {
  if (
    (cmd.cmd === "supervisor_set_mode" || cmd.cmd === "avatar_set_mode") &&
    typeof cmd.mode === "string"
  ) {
    robot.setSupervisorMode(cmd.mode);
  }
  robot.sendSupervisorState(session);
}

// ────────────────────────── таблица стримов ──────────────────────────

function cameraStream(topicId: number, source: string, description: string): MockStreamSource {
  return {
    topicId,
    kind: "ros_topic",
    source,
    description: `SIM: ${description}`,
    defaultQuality: "med",
    rateHz: 10,
    produce(robot, sub) {
      const kind = cameraKindForTopic(sub.topic);
      if (!robot.cameraRenderer || !kind) return null;
      return robot.cameraRenderer({
        world: robot.world,
        robot: robot.state,
        topic: sub.topic,
        kind,
        nowMs: robot.now(),
        mode: robot.mode()
      });
    }
  };
}

/**
 * Полный кадр карты (с PNG, ~336 КБ несжатого RGBA) — только при значимом
 * изменении и не чаще раза в 5 с (quest_node.py MAP_PNG_MIN_PERIOD_S = 5).
 */
export const MAP_PNG_MIN_PERIOD_MS = 5000;
/** «Значимо» — изменилось хотя бы столько клеток (5 см) с прошлого PNG. */
export const MAP_PNG_MIN_CHANGED_CELLS = 25;
/** Мелкие изменения всё же доезжают, но не чаще раза в 30 с. */
export const MAP_PNG_MINOR_PERIOD_MS = 30_000;
/** Без изменений — PNG раз в минуту (подписка и так начинается с PNG). */
export const MAP_PNG_KEEPALIVE_MS = 60_000;

/**
 * Нужен ли PNG в очередном кадре map_2d. Первый кадр подписки — всегда
 * (BINARY_FRAME не latched: иначе новый подписчик стоял бы на пустом полу).
 */
export function mapPngDue(
  now: number,
  last: { ms: number; changedCells: number } | null,
  changedCells: number
): boolean {
  if (!last) return true;
  const since = now - last.ms;
  const delta = changedCells - last.changedCells;
  if (delta >= MAP_PNG_MIN_CHANGED_CELLS) return since >= MAP_PNG_MIN_PERIOD_MS;
  if (delta > 0) return since >= MAP_PNG_MINOR_PERIOD_MS;
  return since >= MAP_PNG_KEEPALIVE_MS;
}

export const DEFAULT_STREAMS: Readonly<Record<string, MockStreamSource>> = Object.freeze({
  camera_rear: cameraStream(0x1001, "/camera/camera/color/image_raw", "OAK-D color — вид вперёд"),
  camera_front: cameraStream(0x1002, "/camera/front/image_raw", "передняя камера"),
  camera_oak_color: cameraStream(0x1003, "oak:color", "OAK-D color (depthai)"),
  camera_oak_depth: cameraStream(0x1004, "/camera/camera/depth/image_rect_raw", "глубина OAK-D"),
  camera_ceiling: cameraStream(0x1005, "/ceiling_camera/image_raw/compressed", "потолочная камера"),
  lidar_2d: {
    topicId: 0x1101,
    kind: "ros_topic",
    source: "/scan",
    description: "SIM: 2D LiDAR 360°, 10 Hz",
    defaultQuality: "high",
    rateHz: 10,
    produce: (robot) => encodeLidar2d(robot.lastScan)
  },
  map_2d: {
    topicId: 0x1103,
    kind: "ros_topic",
    source: "/rtabmap/map",
    description: "SIM: карта открывается по мере осмотра",
    defaultQuality: "low",
    rateHz: 5,
    produce(robot, sub) {
      const now = robot.now();
      const last = (sub.memo.png as { ms: number; changedCells: number } | undefined) ?? null;
      let png: Uint8Array | null = null;
      if (mapPngDue(now, last, robot.grid.changedCells)) {
        png = gridToPng(robot.grid);
        sub.memo.png = { ms: now, changedCells: robot.grid.changedCells };
      }
      const { x, y, yaw } = robot.state;
      return encodeMap2d({ grid: robot.grid, robot: { x, y, yaw }, tsMs: now, png });
    }
  },
  robot_status: {
    topicId: 0x1201,
    kind: "ros_topic",
    source: "aggregation",
    description: "SIM: 1 Hz battery/wifi/mode/vel",
    defaultQuality: "med",
    rateHz: 1,
    produce: (robot) =>
      encodeRobotStatus({
        batteryPct: robot.batteryPct,
        batteryV: 22.2 + (robot.batteryPct / 100) * 3,
        wifiRssi: robot.wifiRssi(),
        mode: robot.mode(),
        velLinear: robot.state.v,
        velAngular: robot.state.w,
        tsMs: robot.now()
      })
  },
  nav_path: {
    topicId: 0x1104,
    kind: "ros_topic",
    source: "/plan",
    description: "SIM: глобальный план Nav2 (≤ 2 Гц)",
    defaultQuality: "low",
    // NAV_PATH_MIN_PERIOD_S = 0.5; кадр уходит только при смене плана
    // (пересчёт раз в 1 с) и один пустой (n = 0) по завершении цели.
    rateHz: 2,
    produce(robot, sub) {
      const rev = robot.nav.pathRevision;
      if (sub.memo.navRevision === rev) return null;
      const first = sub.memo.navRevision === undefined;
      sub.memo.navRevision = rev;
      const pts = robot.nav.currentPath();
      // Свежая подписка без цели — молчим, как сервер без /plan.
      if (first && pts.length === 0) return null;
      return encodeNavPath({ frame: "map", points: pts, tsMs: robot.now() });
    }
  },
  voice_state: {
    topicId: 0x1202,
    kind: "ros_topic",
    source: "/voice/dialogue/state",
    description: "SIM: состояние диалога (idle)",
    defaultQuality: "med",
    rateHz: 0.2,
    produce: (robot) => encodeVoiceState(robot.voiceState, robot.now()),
    onSubscribe(robot, session, sub) {
      session.sendFrame(FrameType.BINARY_FRAME, sub.streamId, encodeVoiceState(robot.voiceState, robot.now()));
    }
  }
});

// ────────────────────────── WebSocket-шов ──────────────────────────

type Listener = (ev: unknown) => void;

/**
 * Конструктор, совместимый с `ConnectionOptions.WebSocketCtor`: каждый
 * `new` — новая «вкладка» у мок-робота. Реализует ровно то, чем
 * пользуется Connection: readyState, protocol, binaryType, send, close,
 * addEventListener(open|message|close|error).
 */
export function createMockWebSocketCtor(
  robot: MockRobot
): new (url: string, protocols?: string | string[]) => WebSocket {
  class MockRobotSocket {
    readyState = 0;
    protocol = "";
    binaryType: BinaryType = "blob";
    readonly url: string;
    private readonly listeners = new Map<string, Set<Listener>>();
    private readonly session: MockSession;

    constructor(url: string, protocols?: string | string[]) {
      this.url = url;
      const requested = Array.isArray(protocols) ? protocols : protocols ? [protocols] : [];
      this.session = new MockSession(
        robot,
        (bytes) => this.deliver(bytes),
        (code, reason) => this.finish(code, reason)
      );
      setTimeout(() => {
        if (this.readyState !== 0) return;
        this.readyState = 1;
        // Сервер выбирает v2, если клиент его предложил (как ws_server).
        this.protocol = requested.includes("robbox-quest-v2")
          ? "robbox-quest-v2"
          : requested[0] ?? "";
        robot.attach(this.session);
        this.emit("open", { type: "open" });
      }, robot.latencyMs);
    }

    addEventListener(name: string, fn: Listener): void {
      let set = this.listeners.get(name);
      if (!set) {
        set = new Set();
        this.listeners.set(name, set);
      }
      set.add(fn);
    }

    removeEventListener(name: string, fn: Listener): void {
      this.listeners.get(name)?.delete(fn);
    }

    send(data: ArrayBuffer | ArrayBufferView): void {
      if (this.readyState !== 1) throw new Error("InvalidStateError: socket not open");
      const bytes =
        data instanceof ArrayBuffer
          ? new Uint8Array(data.slice(0))
          : new Uint8Array(data.buffer.slice(data.byteOffset, data.byteOffset + data.byteLength));
      this.later(() => this.session.handleFrame(bytes));
    }

    close(code = 1000, reason = ""): void {
      if (this.readyState >= 2) return;
      this.readyState = 2;
      this.later(() => this.finish(code, reason));
    }

    private finish(code: number, reason: string): void {
      if (this.readyState === 3) return;
      this.readyState = 3;
      robot.detach(this.session);
      this.emit("close", { type: "close", code, reason, wasClean: true });
    }

    private deliver(bytes: Uint8Array): void {
      this.later(() => {
        if (this.readyState !== 1) return;
        // Connection читает ev.data как ArrayBuffer — отдаём ровно его.
        const buf = bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength);
        this.emit("message", { type: "message", data: buf });
      });
    }

    private later(fn: () => void): void {
      if (robot.latencyMs > 0) setTimeout(fn, robot.latencyMs);
      else queueMicrotask(fn);
    }

    private emit(name: string, ev: unknown): void {
      for (const fn of [...(this.listeners.get(name) ?? [])]) fn(ev);
    }
  }
  return MockRobotSocket as unknown as new (url: string, protocols?: string | string[]) => WebSocket;
}
