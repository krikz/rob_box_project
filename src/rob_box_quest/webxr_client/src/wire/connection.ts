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
import { decodeMsgpackMap, encodeMsgpackMap } from "./msgpack";
import { parseSupervisorState } from "../state/supervisor_state";
import type { SupervisorState } from "../state/supervisor_state";

export type ConnectionState =
  | "idle"
  | "connecting"
  | "authenticating"
  | "connected"
  | "reconnecting"
  | "closed"
  | "auth_failed";

/** Какая версия subprotocol реально выбрана сервером. */
export type NegotiatedSubprotocol = "v1" | "v2";

export interface ConnectionListeners {
  onStateChange?: (state: ConnectionState, info?: string) => void;
  onBinaryFrame?: (streamId: number, payload: Uint8Array) => void;
  onJsonEvent?: (event: JsonEvent) => void;
  /** Свежий round-trip из ping/pong — для HUD (Wave 3.A). */
  onRtt?: (rttMs: number) => void;
  onStreamList?: (items: StreamMeta[]) => void;
  /**
   * WELCOME от сервера. AV-19 (issue #1911): ``teleopFloorHeldBy`` —
   * client_id текущего держателя teleop_floor (``null`` если никто);
   * клиент сверяет с ``sessionId`` (== own session_id), чтобы сразу
   * определить hasFloor (см. teleop_fsm.setHasFloor).
   */
  onWelcome?: (
    sessionId: string,
    serverTimeMs: number,
    teleopFloorHeldBy: string | null
  ) => void;
  /**
   * Сервер прислал `STATE_UPDATE` (frame 0x33). Если сервер на v1 —
   * колбэк никогда не сработает (подробности в `meta-quest-api.md`
   * §11: v1 клиент `STATE_UPDATE` не получает).
   */
  onSupervisorState?: (state: SupervisorState) => void;
  /**
   * Сервер ответил `ERROR{FLOOR_HELD}` / `MODE_CONFLICT` на supervisor-
   * команду. Подробности в `meta-quest-api.md` §8.
   */
  onSupervisorError?: (code: "FLOOR_HELD" | "MODE_CONFLICT", message: string, meta?: Record<string, unknown>) => void;
  onError?: (code: string, message: string) => void;
}

export interface ConnectionOptions {
  url: string;
  /**
   * Subprotocol по умолчанию `"robbox-quest-v2"` (AV-17). Сервер на
   * v1 может быть принудительно выбран через опцию ниже — нужно для
   * отладки старого rob_box_quest без avatar_supervisor.
   */
  subprotocol?: string; // "robbox-quest-v1" | "robbox-quest-v2"
  /**
   * Если true — откатиться на v1 даже если subprotocol явно не задан.
   * Полезно для e2e со старым сервером; прод-код пусть оставляет false.
   */
  forceV1?: boolean;
  clientVersion: string;
  capabilities?: string[];
  pin: string;
  // Watchdog: если нет ничего на сокете > timeoutMs → закрыть + reconnect.
  watchdogTimeoutMs?: number; // дефолт 600
  // Heartbeat: клиент шлёт ping каждые intervalMs (api.md §7 рекомендует 5с;
  // сервер ждёт 3 пропуска по 200мс heartbeat → trip). Здесь мы шлём ping
  // каждые 1 с для надёжности (4 подряд пропуска точно триггернут watchdog).
  pingIntervalMs?: number; // дефолт 250 — быстрее серверного WATCHDOG_TIMEOUT_S (600 мс) с большим запасом.
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
  // Последний измеренный round-trip (ping → pong), null пока pong не пришёл.
  private rttMs: number | null = null;
  // stream_id → topic, присылается в subscribe_ack.
  private streamIdToTopic = new Map<number, string>();
  // topic → stream_id (для дедупа повторных SUBSCRIBE).
  private topicToStreamId = new Map<string, number>();
  // topic → quality
  private topicToQuality = new Map<string, string>();
  /**
   * Какая версия subprotocol реально выбрана сервером. До открытия
   * сокета = `null`; после — `"v1"` (наш v1 fallback) или `"v2"`.
   * Используется в guard'ах supervisor-методов: `sendAcquireFloor` etc.
   * работают только если `negotiatedVersion === "v2"`.
   */
  private negotiatedVersion: NegotiatedSubprotocol | null = null;

  constructor(opts: ConnectionOptions, listeners: ConnectionListeners = {}) {
    this.listeners = listeners;
    // По умолчанию — v2 (Phase 2). v1 — только если явно через forceV1
    // (e2e со старым rob_box_quest) или subprotocol=...v1.
    const requestedSubprotocol = opts.forceV1
      ? "robbox-quest-v1"
      : opts.subprotocol ?? "robbox-quest-v2";
    this.opts = {
      url: opts.url,
      subprotocol: requestedSubprotocol,
      forceV1: opts.forceV1 ?? false,
      clientVersion: opts.clientVersion,
      capabilities: opts.capabilities ?? ["webxr"],
      pin: opts.pin,
      watchdogTimeoutMs: opts.watchdogTimeoutMs ?? 600,
      pingIntervalMs: opts.pingIntervalMs ?? 250,
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

  /** Последний RTT в мс (ping → pong), `null` пока pong не приходил. */
  getRttMs(): number | null {
    return this.rttMs;
  }

  /**
   * Версия subprotocol, выбранная сервером после open. `null` до open.
   * Если клиент запрашивал v2, а сервер ответил v1 (Phase 1 fallback) —
   * вернётся `"v1"`, и supervisor-команды будут no-op.
   */
  getNegotiatedVersion(): NegotiatedSubprotocol | null {
    return this.negotiatedVersion;
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

  sendVoiceAudio(payload: Uint8Array, streamId = 0): void {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return;
    // VOICE_AUDIO (0x13, client→server): сырой int16 PCM 16 kHz mono (рация).
    const bytes = encodeFrame(FrameType.VOICE_AUDIO, streamId, payload);
    this.ws.send(bytes as unknown as ArrayBuffer);
  }

  // ----------------------------------------------------------------
  // Supervisor-команды (Phase 2, subprotocol robbox-quest-v2).
  // API: meta-quest-api.md §3 (0x30–0x32) + ADR-0028 §4.4.
  //
  // Все три — no-op, если сервер не выбрал v2 (`negotiatedVersion`).
  // Это graceful degradation: v1 сервер (Phase 1 rob_box_quest без
  // avatar_supervisor) не понимает эти frame-типы, слать 0x30..0x32 —
  // шум в его логе + риск гонок.
  // ----------------------------------------------------------------

  /**
   * @returns `true` если команда реально отправлена; `false` если
   * сервер на v1 / сокет не открыт / state не готов.
   */
  private canSendSupervisor(): boolean {
    return (
      this.negotiatedVersion === "v2" &&
      this.ws !== null &&
      this.ws.readyState === WebSocket.OPEN
    );
  }

  /** `SET_MODE` (0x30). @returns true если отправлено. */
  sendSetMode(clientId: string, mode: string): boolean {
    if (!this.canSendSupervisor()) return false;
    const payload = encodeMsgpackMap({ client_id: clientId, mode });
    const bytes = encodeFrame(FrameType.SET_MODE, 0, payload);
    this.ws!.send(bytes as unknown as ArrayBuffer);
    return true;
  }

  /** `ACQUIRE_FLOOR` (0x31). @returns true если отправлено. */
  sendAcquireFloor(clientId: string, floor: "teleop" | "voice"): boolean {
    if (!this.canSendSupervisor()) return false;
    const payload = encodeMsgpackMap({ client_id: clientId, floor });
    const bytes = encodeFrame(FrameType.ACQUIRE_FLOOR, 0, payload);
    this.ws!.send(bytes as unknown as ArrayBuffer);
    return true;
  }

  /** `RELEASE_FLOOR` (0x32). @returns true если отправлено. */
  sendReleaseFloor(clientId: string, floor: "teleop" | "voice"): boolean {
    if (!this.canSendSupervisor()) return false;
    const payload = encodeMsgpackMap({ client_id: clientId, floor });
    const bytes = encodeFrame(FrameType.RELEASE_FLOOR, 0, payload);
    this.ws!.send(bytes as unknown as ArrayBuffer);
    return true;
  }

  // expose для тестов: getter последней применённой negotiated-версии.
  _peekNegotiatedVersion(): NegotiatedSubprotocol | null {
    return this.negotiatedVersion;
  }

  private openSocket(): void {
    this.clearTimers();
    // Сброс подписок на новый сокет: после reconnect сервер создаёт НОВУЮ
    // сессию с пустым `subscribed`, старые stream_id/topic недействительны.
    // Иначе subscribe() увидит topicToStreamId и не отправит SUBSCRIBE,
    // и сессия останется без стримов (чёрный экран после реконнекта).
    this.streamIdToTopic.clear();
    this.topicToStreamId.clear();
    this.topicToQuality.clear();
    // После разрыва связи STATE_UPDATE мы больше не получали → supervisor-
    // state неизвестен (UI должен показать `?`, не выдумывать). Новый
    // сокет = новый серверный цикл STATE_UPDATE → неизвестно сброшено.
    this.negotiatedVersion = null;
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
      // Сервер выбирает subprotocol через Sec-WebSocket-Protocol; клиент
      // читает факт выбора из `ws.protocol` (MDN: «the name of the sub-
      // protocol the server selected»). Если сервер ответил v1, а мы
      // просили v2 — supervisor-команды (0x30–0x33) в этом сокете не
      // работают, см. meta-quest-api.md §11.
      const negotiatedRaw = ws.protocol ?? "";
      const negotiated: NegotiatedSubprotocol =
        negotiatedRaw === "robbox-quest-v1" ? "v1" : negotiatedRaw === "robbox-quest-v2" ? "v2" : this.negotiatedVersion ?? "v1";
      this.negotiatedVersion = negotiated;
      console.log("[quest] WS open: negotiated subprotocol =", negotiated, {
        url: this.opts.url,
        requested: this.opts.subprotocol
      });
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
    console.log("[quest] RX frame type=0x" + frame.type.toString(16) + " streamId=" + frame.streamId);
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
    if (frame.type === FrameType.STATE_UPDATE) {
      this.handleSupervisorState(frame.payload);
      return;
    }
    // SUBSCRIBE / UNSUBSCRIBE / JSON_CMD от сервера не ожидаются в Phase 1.
    // SET_MODE / ACQUIRE_FLOOR / RELEASE_FLOOR — это client→server
    // команды; если сервер прислал их обратно, это баг протокола.
    if (
      frame.type === FrameType.SET_MODE ||
      frame.type === FrameType.ACQUIRE_FLOOR ||
      frame.type === FrameType.RELEASE_FLOOR
    ) {
      this.listeners.onError?.("BAD_PAYLOAD", `unexpected server→client supervisor frame 0x${frame.type.toString(16)}`);
      return;
    }
    this.listeners.onError?.("BAD_PAYLOAD", `unexpected server frame type ${frame.type}`);
  }

  private handleSupervisorState(payload: Uint8Array): void {
    const map = decodeMsgpackMap(payload);
    if (!map) {
      this.listeners.onError?.("BAD_PAYLOAD", "STATE_UPDATE not a msgpack map");
      return;
    }
    const state = parseSupervisorState(map);
    if (!state) {
      this.listeners.onError?.("BAD_PAYLOAD", "STATE_UPDATE missing required fields");
      return;
    }
    this.listeners.onSupervisorState?.(state);
  }

  private handleWelcome(frame: DecodedFrame): void {
    try {
      const obj = JSON.parse(new TextDecoder().decode(frame.payload)) as {
        session_id?: string;
        server_time_ms?: number;
        // AV-19: optional floor-claim, может отсутствовать у Phase 1
        // серверов (которые ещё не знают про AV-19). Тогда мы остаёмся
        // в оптимистичном default и ждём первого FLOOR_HELD/floor_lost.
        teleop_floor_held_by?: string | null;
      };
      console.log("[quest] WELCOME received", obj);
      this.setState("connected");
      this.reconnectAttempt = 0;
      const heldBy =
        typeof obj.teleop_floor_held_by === "string"
          ? obj.teleop_floor_held_by
          : null;
      this.listeners.onWelcome?.(obj.session_id ?? "", obj.server_time_ms ?? 0, heldBy);
      this.startPing();
    } catch (err) {
      console.log("[quest] WELCOME parse error", err);
      this.listeners.onError?.("BAD_PAYLOAD", `welcome parse: ${(err as Error).message}`);
    }
  }

  private handleError(frame: DecodedFrame): void {
    let obj: { code?: string; message?: string; [k: string]: unknown } = {};
    try {
      obj = JSON.parse(new TextDecoder().decode(frame.payload));
    } catch {
      // ignore
    }
    const code = obj.code ?? "INTERNAL";
    const msg = obj.message ?? "unknown error";
    // Supervisor-ошибки — отдельный канал, чтобы UI мог показать
    // тост «руль сейчас у оператора Telegram», не путая с обычным
    // серверным error (meta-quest-api.md §8).
    if (code === "FLOOR_HELD" || code === "MODE_CONFLICT") {
      const meta: Record<string, unknown> = {};
      for (const [k, v] of Object.entries(obj)) {
        if (k !== "code" && k !== "message") meta[k] = v;
      }
      this.listeners.onSupervisorError?.(code, msg, meta);
      return;
    }
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
    if (type === "pong") {
      // RTT считаем по своим часам: сервер эхом возвращает наш ts_ms
      // (meta-quest-api.md §6/§7), поэтому рассинхрон часов не мешает.
      const sent = (ev as { ts_ms?: number }).ts_ms;
      if (typeof sent === "number" && Number.isFinite(sent)) {
        this.rttMs = Math.max(0, Date.now() - sent);
        this.listeners.onRtt?.(this.rttMs);
      }
    } else if (type === "subscribe_ack") {
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
    console.log("[quest] startPing: interval=", this.opts.pingIntervalMs, "ms");
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
    console.log("[quest] state:", this.state, "->", s, info ? `(${info})` : "");
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