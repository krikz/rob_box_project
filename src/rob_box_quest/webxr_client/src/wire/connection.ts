// Минимальный WSS-клиент для Phase 2.2 telemetry.
//
// Только то, что нужно PerfReporter:
//   • connect → HELLO → WELCOME → connected
//   • JSON_EVENT {type: "telemetry_perf"} → сервер
//   • ping/pong для RTT (с EMA-сглаживанием)
//   • reconnect с exponential backoff
//
// Полный client (teleop, video panels, lidar) добавляется в Phase 2.3+
// карточках. Здесь — узкий слой для telemetry.

import { encodeJsonFrame, FrameType } from "./protocol";
import type { HelloMsg, TelemetryPerfPayload, WelcomeMsg } from "./messages";

export type ConnectionState = "idle" | "connecting" | "connected" | "reconnecting" | "closed";

export interface ConnectionOpts {
  url: string;
  subprotocol?: string;
  clientVersion: string;
  pin: string;
  /** Round-trip time EMA — устанавливается сервером через ping/pong. */
  rttEmaMs?: number;
  /** Latency callback для reporter.getLatencyMs(). */
  onLatencyChange?: (ms: number | null) => void;
  /** State change для reporter и UI. */
  onStateChange?: (state: ConnectionState) => void;
  /** Inject WebSocket для тестов. */
  WebSocketCtor?: new (url: string, protocols?: string | string[]) => WebSocket;
  /** Backoff config. */
  reconnectInitialMs?: number;
  reconnectMaxMs?: number;
}

const STREAM_ID_CONTROL = 0x0000;

/**
 * Лёгкая обёртка над WebSocket с reconnect и RTT-трекингом.
 */
export class TelemetryConnection {
  private ws: WebSocket | null = null;
  private state: ConnectionState = "idle";
  private closedByUser = false;
  private reconnectAttempt = 0;
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null;
  private pingTimer: ReturnType<typeof setInterval> | null = null;
  private rttEmaMs: number | null = null;
  private readonly opts: Required<Omit<ConnectionOpts, "onStateChange" | "onLatencyChange" | "WebSocketCtor">>;
  private readonly listeners: Pick<ConnectionOpts, "onStateChange" | "onLatencyChange">;
  private readonly WebSocketCtor: new (url: string, protocols?: string | string[]) => WebSocket;
  private pendingPings = new Map<string, number>(); // nonce → send time
  private sessionId: string | null = null;

  constructor(opts: ConnectionOpts) {
    this.opts = {
      url: opts.url,
      subprotocol: opts.subprotocol ?? "robbox-quest-v1",
      clientVersion: opts.clientVersion,
      pin: opts.pin,
      rttEmaMs: opts.rttEmaMs ?? 0.5,
      reconnectInitialMs: opts.reconnectInitialMs ?? 1000,
      reconnectMaxMs: opts.reconnectMaxMs ?? 30_000,
    };
    this.listeners = {
      onStateChange: opts.onStateChange,
      onLatencyChange: opts.onLatencyChange,
    };
    const globalWS = (globalThis as unknown as { WebSocket: typeof WebSocket }).WebSocket;
    this.WebSocketCtor = opts.WebSocketCtor ?? globalWS;
  }

  getState(): ConnectionState {
    return this.state;
  }

  getLatencyMs(): number | null {
    return this.rttEmaMs;
  }

  getSessionId(): string | null {
    return this.sessionId;
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

  sendTelemetry(payload: TelemetryPerfPayload): boolean {
    if (!this.ws || this.ws.readyState !== 1 /* OPEN */) return false;
    try {
      const bytes = encodeJsonFrame(FrameType.JSON_EVENT, STREAM_ID_CONTROL, payload);
      this.ws.send(bytes as unknown as ArrayBuffer);
      return true;
    } catch {
      return false;
    }
  }

  private openSocket(): void {
    this.setState(this.reconnectAttempt > 0 ? "reconnecting" : "connecting");
    let ws: WebSocket;
    try {
      ws = new this.WebSocketCtor(this.opts.url, this.opts.subprotocol);
    } catch {
      this.scheduleReconnect();
      return;
    }
    ws.binaryType = "arraybuffer";
    this.ws = ws;

    ws.addEventListener("open", () => {
      this.reconnectAttempt = 0;
      // HELLO — для WELCOME RTT.
      const t0 = performance.now();
      const hello: HelloMsg = {
        client_version: this.opts.clientVersion,
        capabilities: ["telemetry"],
        session_pin: this.opts.pin,
      };
      const bytes = encodeJsonFrame(FrameType.HELLO, STREAM_ID_CONTROL, hello);
      ws.send(bytes as unknown as ArrayBuffer);
      this.pendingPings.set("__hello__", t0);

      // ping loop — каждую секунду для RTT (быстрее, чем 5 с в api.md §7,
      // потому что для telemetry 5 с — слишком грубо).
      this.pingTimer = setInterval(() => this.sendPing(), 1000);
    });

    ws.addEventListener("message", (ev) => {
      const buf = new Uint8Array(ev.data as ArrayBuffer);
      this.handleFrame(buf);
    });

    ws.addEventListener("close", () => {
      this.clearTimers();
      this.ws = null;
      this.listeners.onLatencyChange?.(null);
      if (!this.closedByUser) {
        this.scheduleReconnect();
      }
    });

    ws.addEventListener("error", () => {
      // close придёт следом.
    });
  }

  private handleFrame(buf: Uint8Array): void {
    // Минимальный парсер: type byte + 4 bytes streamId + LEB128 len + payload.
    if (buf.length < 5) return;
    const type = buf[0];
    // const streamId = new DataView(buf.buffer, buf.byteOffset).getUint32(1, true);
    let p = 5;
    let len = 0;
    let shift = 0;
    while (p < buf.length) {
      const b = buf[p++];
      len |= (b & 0x7f) << shift;
      shift += 7;
      if ((b & 0x80) === 0) break;
    }
    const payload = buf.subarray(p, p + len);
    const text = new TextDecoder().decode(payload);
    let obj: { type: string; [k: string]: unknown };
    try {
      obj = JSON.parse(text);
    } catch {
      return;
    }

    if (type === 0x02 /* WELCOME */) {
      const w = obj as unknown as WelcomeMsg;
      this.sessionId = w.session_id;
      const t0 = this.pendingPings.get("__hello__");
      if (typeof t0 === "number") {
        const rtt = performance.now() - t0;
        this.updateLatency(rtt);
        this.pendingPings.delete("__hello__");
      }
      this.setState("connected");
    } else if (type === 0x12 /* JSON_EVENT */) {
      if (obj.type === "pong") {
        const nonce = obj.nonce as string | undefined;
        if (nonce) {
          const t0 = this.pendingPings.get(nonce);
          if (typeof t0 === "number") {
            const rtt = performance.now() - t0;
            this.updateLatency(rtt);
            this.pendingPings.delete(nonce);
          }
        }
      }
    }
  }

  private sendPing(): void {
    if (!this.ws || this.ws.readyState !== 1) return;
    const nonce = `${Date.now()}-${Math.random().toString(36).slice(2, 8)}`;
    this.pendingPings.set(nonce, performance.now());
    // Cap на размер map (если pong не приходят).
    if (this.pendingPings.size > 64) {
      const firstKey = this.pendingPings.keys().next().value;
      if (firstKey) this.pendingPings.delete(firstKey);
    }
    const bytes = encodeJsonFrame(FrameType.JSON_EVENT, STREAM_ID_CONTROL, {
      type: "ping",
      ts_ms: Date.now(),
      nonce,
    });
    try {
      this.ws.send(bytes as unknown as ArrayBuffer);
    } catch {
      // ignore
    }
  }

  private updateLatency(sampleMs: number): void {
    if (sampleMs < 0 || !Number.isFinite(sampleMs)) return;
    if (this.rttEmaMs === null) {
      this.rttEmaMs = sampleMs;
    } else {
      this.rttEmaMs = this.rttEmaMs * (1 - this.opts.rttEmaMs) + sampleMs * this.opts.rttEmaMs;
    }
    this.listeners.onLatencyChange?.(this.rttEmaMs);
  }

  private scheduleReconnect(): void {
    if (this.closedByUser) return;
    this.reconnectAttempt++;
    const base = Math.min(
      this.opts.reconnectMaxMs,
      this.opts.reconnectInitialMs * Math.pow(2, this.reconnectAttempt - 1)
    );
    // Jitter ±25%.
    const jitter = base * (0.75 + Math.random() * 0.5);
    this.reconnectTimer = setTimeout(() => this.openSocket(), jitter);
    this.setState("reconnecting");
  }

  private clearTimers(): void {
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
    if (this.pingTimer) {
      clearInterval(this.pingTimer);
      this.pingTimer = null;
    }
    this.pendingPings.clear();
  }

  private setState(s: ConnectionState): void {
    if (this.state === s) return;
    this.state = s;
    this.listeners.onStateChange?.(s);
  }
}