// WSS-клиент rob_box_quest.
//
// Lifecycle:
//   connecting → authenticating → connected → reconnecting → ...
//
// Handshake (api.md §3):
//   client HELLO      →
//   server WELCOME    → клиент сохраняет session_id и серверное время.
//   client SUBSCRIBE  → server subscribe_ack { stream_id }
//   ... loop data ...
//
// Подписки маппятся в topic через stream_id, потому что BINARY_FRAME несёт
// только stream_id в заголовке (api.md §4 обещал 4-байтовый topic_id, но
// реальный сервер шлёт JPEG/LiDAR as-is без префикса — см. protocol.ts TODO).

import {
  decodeFrame,
  encodeFrame,
  encodeJsonFrame,
  FrameType,
  type DecodedFrame
} from "./protocol";
import type {
  HelloMsg,
  JsonCmd,
  JsonEvent,
  StreamMeta,
  SubscribeMsg,
  UnsubscribeMsg
} from "./messages";

export type ConnectionState =
  | "idle"
  | "connecting"
  | "authenticating"
  | "connected"
  | "reconnecting"
  | "closed"
  | "auth_failed";

export interface ConnectionListeners {
  onStateChange?: (state: ConnectionState, info?: string) => void;
  onBinaryFrame?: (streamId: number, payload: Uint8Array) => void;
  onJsonEvent?: (event: JsonEvent) => void;
  onStreamList?: (items: StreamMeta[]) => void;
  onWelcome?: (sessionId: string, serverTimeMs: number) => void;
  onError?: (code: string, message: string) => void;
}

export interface ConnectionOptions {
  url: string;
  subprotocol?: string; // "robbox-quest-v1" (Phase 1)
  clientVersion: string;
  capabilities?: string[];
  pin: string;
  // Watchdog: если нет ничего на сокете > timeoutMs → закрыть + reconnect.
  watchdogTimeoutMs?: number; // дефолт 600
  // Heartbeat: клиент шлёт ping каждые intervalMs (api.md §7 рекомендует 5с;
  // сервер ждёт 3 пропуска по 200мс heartbeat → trip). Здесь мы шлём ping
  // каждые 1 с для надёжности (4 подряд пропуска точно триггернут watchdog).
  pingIntervalMs?: number;
  // Reconnect exponential backoff.
  reconnectInitialMs?: number; // 1000
  reconnectMaxMs?: number; // 30000
  // Флаг для тестов: пропускать реальные reconnect.
  autoReconnect?: boolean;
  // Опционально — конструктор WebSocket (для тестов с FakeWebSocket).
  WebSocketCtor?: new (url: string, protocols?: string | string[]) => WebSocket;
}

// Множество stream_id, занятых на сервере (см. session.py allocate_stream_id
// 0x0001..0x0FFF для client-initiated, 0x1000..0xFFFF для server-initiated).
// Для команд (HELLO/SUBSCRIBE) stream_id в диапазоне client-initiated:
//   HELLO = 0, SUBSCRIBE = следующий свободный.
export const STREAM_ID_HELLO = 0x0000;
// Control frames (HELLO/WELCOME/GOODBYE/ERROR) используют stream_id = 0.

export class Connection {
  private ws: WebSocket | null = null;
  private opts: Required<Omit<ConnectionOptions, "pin" | "WebSocketCtor">> & { pin: string };
  private WebSocketCtor: new (url: string, protocols?: string | string[]) => WebSocket;
  private listeners: ConnectionListeners;
  private state: ConnectionState = "idle";
  private nextClientStreamId = 1; // 0x0001..0x0FFF (control = 0)
  private readonly maxClientStreamId = 0x0fff;
  private pingTimer: ReturnType<typeof setInterval> | null = null;
  private watchdogTimer: ReturnType<typeof setTimeout> | null = null;
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null;
  private reconnectAttempt = 0;
  private closedByUser = false;
  // stream_id → topic, присылается в subscribe_ack.
  private streamIdToTopic = new Map<number, string>();
  // topic → stream_id (для дедупа повторных SUBSCRIBE).
  private topicToStreamId = new Map<string, number>();
  // topic → quality
  private topicToQuality = new Map<string, string>();

  constructor(opts: ConnectionOptions, listeners: ConnectionListeners = {}) {
    this.listeners = listeners;
    this.opts = {
      url: opts.url,
      subprotocol: opts.subprotocol ?? "robbox-quest-v1",
      clientVersion: opts.clientVersion,
      capabilities: opts.capabilities ?? ["webxr"],
      pin: opts.pin,
      watchdogTimeoutMs: opts.watchdogTimeoutMs ?? 600,
      pingIntervalMs: opts.pingIntervalMs ?? 1000,
      reconnectInitialMs: opts.reconnectInitialMs ?? 1000,
      reconnectMaxMs: opts.reconnectMaxMs ?? 30_000,
      autoReconnect: opts.autoReconnect ?? true
    };
    // Тесты могут подсунуть свой WebSocket-класс (FakeWebSocket).
    this.WebSocketCtor = opts.WebSocketCtor ?? (globalThis as unknown as { WebSocket: typeof WebSocket }).WebSocket;
  }

  getState(): ConnectionState {
    return this.state;
  }

  // map доступен только для чтения (снаружи — для панелей / lidar).
  getTopicForStream(streamId: number): string | undefined {
    return this.streamIdToTopic.get(streamId);
  }

  listSubscribed(): Array<{ topic: string; stream_id: number; quality: string }> {
    const out: Array<{ topic: string; stream_id: number; quality: string }> = [];
    for (const [topic, sid] of this.topicToStreamId.entries()) {
      out.push({
        topic,
        stream_id: sid,
        quality: this.topicToQuality.get(topic) ?? "med"
      });
    }
    return out;
  }

  connect(): void {
    this.closedByUser = false;
    this.openSocket();
  }

  close(): void {
    this.closedByUser = true;
    this.clearTimers();
    if (this.ws) {
      try {
        this.ws.close(1000, "client_close");
      } catch {
        // ignore
      }
      this.ws = null;
    }
    this.setState("closed");
  }

  send(cmd: JsonCmd): void {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      throw new Error("connection not open");
    }
    // JSON_CMD использует stream_id 0 (control), см. ws_server.py.
    const bytes = encodeJsonFrame(FrameType.JSON_CMD, 0, cmd);
    this.ws.send(bytes as unknown as ArrayBuffer);
  }

  private openSocket(): void {
    this.clearTimers();
    this.setState(this.reconnectAttempt > 0 ? "reconnecting" : "connecting");

    let ws: WebSocket;
    try {
      ws = new this.WebSocketCtor(this.opts.url, this.opts.subprotocol);
    } catch (err) {
      this.scheduleReconnect(`socket construct failed: ${(err as Error).message}`);
      return;
    }
    ws.binaryType = "arraybuffer";
    this.ws = ws;

    ws.addEventListener("open", () => {
      this.setState("authenticating");
      const hello: HelloMsg = {
        client_version: this.opts.clientVersion,
        capabilities: this.opts.capabilities,
        session_pin: this.opts.pin
      };
      const bytes = encodeJsonFrame(FrameType.HELLO, STREAM_ID_HELLO, hello);
      ws.send(bytes as unknown as ArrayBuffer);
      this.feedRx(); // HELLO — это тоже RX, watchdog можно сбросить.
    });

    ws.addEventListener("message", (ev) => {
      this.feedRx();
      const buf = ev.data as ArrayBuffer;
      const bytes = new Uint8Array(buf);
      let frame: DecodedFrame;
      try {
        frame = decodeFrame(bytes);
      } catch (err) {
        this.listeners.onError?.("BAD_PAYLOAD", (err as Error).message);
        return;
      }
      this.dispatchFrame(frame);
    });

    ws.addEventListener("close", (ev) => {
      this.clearTimers();
      this.ws = null;
      if (this.state === "auth_failed") {
        // terminal: не реконнектимся, юзер должен сменить PIN.
        return;
      }
      if (this.closedByUser) {
        this.setState("closed");
        return;
      }
      const why = ev.reason || `code=${ev.code}`;
      this.scheduleReconnect(`socket closed: ${why}`);
    });

    ws.addEventListener("error", () => {
      // close сам придёт следом.
    });

    this.startWatchdog();
  }

  private dispatchFrame(frame: DecodedFrame): void {
    if (frame.type === FrameType.WELCOME) {
      this.handleWelcome(frame);
      return;
    }
    if (frame.type === FrameType.GOODBYE) {
      this.ws?.close(1000, "goodbye");
      return;
    }
    if (frame.type === FrameType.ERROR) {
      this.handleError(frame);
      return;
    }
    if (frame.type === FrameType.JSON_EVENT) {
      this.handleJsonEvent(frame);
      return;
    }
    if (frame.type === FrameType.BINARY_FRAME) {
      this.listeners.onBinaryFrame?.(frame.streamId, frame.payload);
      return;
    }
    // SUBSCRIBE / UNSUBSCRIBE / JSON_CMD от сервера не ожидаются в Phase 1.
    this.listeners.onError?.("BAD_PAYLOAD", `unexpected server frame type ${frame.type}`);
  }

  private handleWelcome(frame: DecodedFrame): void {
    try {
      const obj = JSON.parse(new TextDecoder().decode(frame.payload));
      this.setState("connected");
      this.reconnectAttempt = 0;
      this.listeners.onWelcome?.(obj.session_id, obj.server_time_ms);
      this.startPing();
    } catch (err) {
      this.listeners.onError?.("BAD_PAYLOAD", `welcome parse: ${(err as Error).message}`);
    }
  }

  private handleError(frame: DecodedFrame): void {
    let obj: { code?: string; message?: string } = {};
    try {
      obj = JSON.parse(new TextDecoder().decode(frame.payload));
    } catch {
      // ignore
    }
    const code = obj.code ?? "INTERNAL";
    const msg = obj.message ?? "unknown error";
    this.listeners.onError?.(code, msg);
    if (code === "AUTH_FAIL") {
      this.closedByUser = true;
      this.clearTimers();
      try {
        this.ws?.close(4001, "auth_fail");
      } catch {
        // ignore
      }
      this.setState("auth_failed");
    }
  }

  private handleJsonEvent(frame: DecodedFrame): void {
    let ev: JsonEvent;
    try {
      ev = JSON.parse(new TextDecoder().decode(frame.payload));
    } catch {
      return;
    }
    const type = (ev as { type?: string }).type;
    if (type === "subscribe_ack") {
      const ack = ev as { topic: string; stream_id: number; quality?: string };
      this.streamIdToTopic.set(ack.stream_id, ack.topic);
      this.topicToStreamId.set(ack.topic, ack.stream_id);
      this.topicToQuality.set(ack.topic, ack.quality ?? "med");
    } else if (type === "stream_list") {
      const items = (ev as { items?: Array<Record<string, unknown>> }).items ?? [];
      const meta: StreamMeta[] = items.map((it) => ({
        topic: String(it.topic ?? ""),
        topic_id: Number(it.topic_id ?? 0),
        kind: (it.kind as "ros_topic" | "camera_direct") ?? "ros_topic",
        source: String(it.source ?? ""),
        default_quality: String(it.default_quality ?? "med"),
        description: it.description ? String(it.description) : undefined
      }));
      this.listeners.onStreamList?.(meta);
    }
    this.listeners.onJsonEvent?.(ev);
  }

  // ----------------------------------------------------------------
  // Подписки: после успешного WELCOME.
  // ----------------------------------------------------------------

  subscribe(topic: string, quality?: "low" | "med" | "high"): void {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return;
    if (this.topicToStreamId.has(topic)) {
      // уже подписаны — идемпотентно (см. ws_server._on_subscribe).
      return;
    }
    const sid = this.allocateClientStreamId();
    const msg: SubscribeMsg = quality ? { topic, quality } : { topic };
    const bytes = encodeJsonFrame(FrameType.SUBSCRIBE, sid, msg);
    this.ws.send(bytes as unknown as ArrayBuffer);
  }

  unsubscribe(topic: string): void {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return;
    const sid = this.topicToStreamId.get(topic);
    if (sid === undefined) return;
    const msg: UnsubscribeMsg = { topic };
    const bytes = encodeJsonFrame(FrameType.UNSUBSCRIBE, sid, msg);
    this.ws.send(bytes as unknown as ArrayBuffer);
    this.topicToStreamId.delete(topic);
    this.topicToQuality.delete(topic);
    this.streamIdToTopic.delete(sid);
  }

  requestStreamList(): void {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return;
    const bytes = encodeJsonFrame(FrameType.JSON_CMD, 0, {
      cmd: "stream_list",
      ts_ms: Date.now()
    });
    this.ws.send(bytes as unknown as ArrayBuffer);
  }

  // ----------------------------------------------------------------
  // Watchdog + ping.
  // ----------------------------------------------------------------

  private feedRx(): void {
    if (this.watchdogTimer) clearTimeout(this.watchdogTimer);
    this.watchdogTimer = setTimeout(() => {
      // Сервер молчит > timeout → закрыть (сервер сам триггернёт, но
      // подстрахуемся и закроем с нашей стороны).
      this.scheduleReconnect("watchdog trip (no RX)");
    }, this.opts.watchdogTimeoutMs + this.opts.pingIntervalMs);
  }

  private startPing(): void {
    if (this.pingTimer) clearInterval(this.pingTimer);
    this.pingTimer = setInterval(() => {
      if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return;
      // Per docs/architecture/meta-quest-api.md §7 — клиент шлёт
      // JSON_EVENT{type:"ping"} (раньше ошибочно было JSON_CMD{cmd:"ping"},
      // сервер его игнорировал и рвал сессию по watchdog через 600 мс).
      const bytes = encodeJsonFrame(FrameType.JSON_EVENT, 0, {
        type: "ping",
        ts_ms: Date.now()
      });
      try {
        this.ws.send(bytes as unknown as ArrayBuffer);
      } catch {
        // ignore
      }
    }, this.opts.pingIntervalMs);
  }

  private startWatchdog(): void {
    this.feedRx();
  }

  // ----------------------------------------------------------------
  // Reconnect.
  // ----------------------------------------------------------------

  private scheduleReconnect(reason: string): void {
    if (!this.opts.autoReconnect || this.closedByUser) return;
    if (this.state === "auth_failed" || this.state === "closed") return;
    const delay = Math.min(
      this.opts.reconnectMaxMs,
      this.opts.reconnectInitialMs * Math.pow(2, this.reconnectAttempt)
    );
    this.reconnectAttempt += 1;
    this.listeners.onError?.("RECONNECT", `${reason} (attempt ${this.reconnectAttempt}, next ${delay}ms)`);
    this.setState("reconnecting");
    this.reconnectTimer = setTimeout(() => this.openSocket(), delay);
  }

  private clearTimers(): void {
    if (this.pingTimer) {
      clearInterval(this.pingTimer);
      this.pingTimer = null;
    }
    if (this.watchdogTimer) {
      clearTimeout(this.watchdogTimer);
      this.watchdogTimer = null;
    }
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
  }

  private setState(s: ConnectionState, info?: string): void {
    if (this.state === s) return;
    this.state = s;
    this.listeners.onStateChange?.(s, info);
  }

  private allocateClientStreamId(): number {
    const sid = this.nextClientStreamId;
    this.nextClientStreamId = sid >= this.maxClientStreamId ? 1 : sid + 1;
    return sid;
  }

  // expose для тестов: проверка распределения stream_id.
  _peekNextClientStreamId(): number {
    return this.nextClientStreamId;
  }
}

// Удобный парсер JSON в payload (используется в unit-тестах).
export function encodeJsonFrameForTest(type: FrameType, streamId: number, obj: unknown): Uint8Array {
  return encodeJsonFrame(type, streamId, obj);
}

export { encodeFrame };