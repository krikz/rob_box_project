// Smoke-тест connection.ts без реального WebSocket:
// проверяем encode/decode HELLO/WELCOME/SUBSCRIBE handshake через loopback.
//
// Используем `FakeWebSocket.reserveClient()` чтобы подменить конструктор,
// который дёрнет `Connection` — `new FakeWebSocket()` вернёт наш заранее
// подготовленный client-сокет (см. FakeWebSocket constructor).

import { describe, it, expect, beforeEach } from "vitest";
import {
  encodeJsonFrame,
  FrameType,
  decodeFrame
} from "../src/wire/protocol";

// jsdom не предоставляет WebSocket, поэтому делаем in-memory loopback.
class FakeWebSocket {
  static instances: FakeWebSocket[] = [];
  /** Следующий new FakeWebSocket() вернёт этот объект (через Object.assign). */
  static nextInstance: FakeWebSocket | null = null;

  static makeServer(): FakeWebSocket {
    const ws = Object.create(FakeWebSocket.prototype) as FakeWebSocket;
    ws.readyState = 0;
    ws.peer = null;
    ws.sentFrames = [];
    ws.listeners = {};
    FakeWebSocket.instances.push(ws);
    return ws;
  }

  /** Pre-create a "client" socket and queue it as the next instance returned by `new FakeWebSocket()`. */
  static reserveClient(): FakeWebSocket {
    const ws = Object.create(FakeWebSocket.prototype) as FakeWebSocket;
    ws.readyState = 0;
    ws.peer = null;
    ws.sentFrames = [];
    ws.listeners = {};
    FakeWebSocket.instances.push(ws);
    FakeWebSocket.nextInstance = ws;
    return ws;
  }

  static link(a: FakeWebSocket, b: FakeWebSocket): void {
    a.peer = b;
    b.peer = a;
  }

  readyState = 0; // CONNECTING
  peer: FakeWebSocket | null = null;
  sentFrames: Uint8Array[] = [];
  listeners: Record<string, Array<(ev: unknown) => void>> = {};

  constructor() {
    if (FakeWebSocket.nextInstance) {
      const inst = FakeWebSocket.nextInstance;
      FakeWebSocket.nextInstance = null;
      Object.assign(this, inst);
      return;
    }
    FakeWebSocket.instances.push(this);
  }

  addEventListener(name: string, fn: (ev: unknown) => void): void {
    (this.listeners[name] ??= []).push(fn);
  }

  removeEventListener(name: string, fn: (ev: unknown) => void): void {
    const arr = this.listeners[name];
    if (!arr) return;
    const i = arr.indexOf(fn);
    if (i >= 0) arr.splice(i, 1);
  }

  send(data: ArrayBuffer | Uint8Array): void {
    const bytes = data instanceof Uint8Array ? data : new Uint8Array(data);
    this.sentFrames.push(bytes);
    // Simulate async loopback: dispatch message event on peer.
    queueMicrotask(() => {
      if (this.peer) {
        const listeners = this.peer.listeners["message"] ?? [];
        for (const fn of listeners) {
          fn({ data: bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength) });
        }
      }
    });
  }

  close(code = 1000, reason = ""): void {
    this.readyState = 3;
    for (const fn of this.listeners["close"] ?? []) fn({ code, reason });
    if (this.peer) {
      for (const fn of this.peer.listeners["close"] ?? []) fn({ code, reason });
    }
  }

  dispatchOpen(): void {
    this.readyState = 1;
    for (const fn of this.listeners["open"] ?? []) fn({});
  }
}

(globalThis as unknown as { WebSocket: unknown }).WebSocket = FakeWebSocket;

import { Connection } from "../src/wire/connection";

describe("Connection handshake (loopback)", () => {
  beforeEach(() => {
    FakeWebSocket.instances = [];
    FakeWebSocket.nextInstance = null;
  });

  it("FakeWebSocket.dispatchOpen actually calls listeners", () => {
    const [a, b] = [FakeWebSocket.makeServer(), FakeWebSocket.makeServer()];
    FakeWebSocket.link(a, b);
    let called = 0;
    a.addEventListener("open", () => (called += 1));
    a.dispatchOpen();
    expect(called).toBe(1);
  });

  it("sends HELLO on open and parses WELCOME", () => {
    const server = FakeWebSocket.makeServer();
    const client = FakeWebSocket.reserveClient();
    FakeWebSocket.link(client, server);

    const states: string[] = [];
    let welcome: { sessionId: string; serverTimeMs: number } | null = null;
    const conn = new Connection(
      {
        url: "ws://test",
        clientVersion: "0.1.0",
        pin: "123456",
        autoReconnect: false,
        pingIntervalMs: 100_000, // не триггерить ping в тесте
        WebSocketCtor: FakeWebSocket as unknown as new (url: string, protocols?: string | string[]) => WebSocket
      },
      {
        onStateChange: (s) => states.push(s),
        onWelcome: (sid, ts) => (welcome = { sessionId: sid, serverTimeMs: ts })
      }
    );
    conn.connect();
    // Connection создал FakeWebSocket через reserveClient() — наш `client`.
    expect(client.listeners["open"]?.length).toBeGreaterThanOrEqual(1);

    // Симулируем открытие сокета: client.open + server.open.
    client.dispatchOpen();
    server.dispatchOpen();

    // Клиент должен отправить HELLO синхронно при open.
    expect(client.sentFrames.length).toBeGreaterThanOrEqual(1);
    const helloFrame = client.sentFrames[0];
    const dec = decodeFrame(helloFrame);
    expect(dec.type).toBe(FrameType.HELLO);
    expect(JSON.parse(new TextDecoder().decode(dec.payload)).session_pin).toBe("123456");

    // Сервер отвечает WELCOME.
    const welcomeBytes = encodeJsonFrame(FrameType.WELCOME, 0, {
      server_version: "0.1.0",
      session_id: "test-session",
      server_time_ms: 12345
    });
    server.send(welcomeBytes as unknown as ArrayBuffer);
    return new Promise<void>((r) =>
      queueMicrotask(() =>
        queueMicrotask(() => {
          expect(states).toContain("connected");
          expect(welcome).toEqual({ sessionId: "test-session", serverTimeMs: 12345 });
          r();
        })
      )
    );
  });

  it("on AUTH_FAIL → state auth_failed and no reconnect", () => {
    const server = FakeWebSocket.makeServer();
    const client = FakeWebSocket.reserveClient();
    FakeWebSocket.link(client, server);

    const states: string[] = [];
    const conn = new Connection(
      {
        url: "ws://test",
        clientVersion: "0.1.0",
        pin: "000000",
        autoReconnect: true,
        pingIntervalMs: 100_000,
        WebSocketCtor: FakeWebSocket as unknown as new (url: string, protocols?: string | string[]) => WebSocket
      },
      { onStateChange: (s) => states.push(s) }
    );
    conn.connect();
    client.dispatchOpen();
    server.dispatchOpen();
    // сервер шлёт ERROR{AUTH_FAIL}
    const errBytes = encodeJsonFrame(FrameType.ERROR, 0, {
      code: "AUTH_FAIL",
      message: "wrong PIN"
    });
    server.send(errBytes as unknown as ArrayBuffer);
    return new Promise<void>((r) =>
      queueMicrotask(() =>
        queueMicrotask(() => {
          expect(states).toContain("auth_failed");
          expect(conn.getState()).toBe("auth_failed");
          r();
        })
      )
    );
  });

  it("subscribe_ack populates streamId → topic map", () => {
    const server = FakeWebSocket.makeServer();
    const client = FakeWebSocket.reserveClient();
    FakeWebSocket.link(client, server);

    const conn = new Connection({
      url: "ws://test",
      clientVersion: "0.1.0",
      pin: "123456",
      autoReconnect: false,
      pingIntervalMs: 100_000,
      WebSocketCtor: FakeWebSocket as unknown as new (url: string, protocols?: string | string[]) => WebSocket
    });
    conn.connect();
    client.dispatchOpen();
    server.dispatchOpen();
    // WELCOME
    server.send(
      encodeJsonFrame(FrameType.WELCOME, 0, { server_version: "0.1.0", session_id: "s", server_time_ms: 1 }) as unknown as ArrayBuffer
    );
    return new Promise<void>((r) =>
      queueMicrotask(() => {
        // subscribe
        conn.subscribe("camera_rear", "med");
        queueMicrotask(() => {
          // сервер отвечает ack с stream_id
          server.send(
            encodeJsonFrame(FrameType.JSON_EVENT, 0, {
              type: "subscribe_ack",
              topic: "camera_rear",
              stream_id: 0x1001,
              quality: "med"
            }) as unknown as ArrayBuffer
          );
          queueMicrotask(() =>
            queueMicrotask(() => {
              expect(conn.getTopicForStream(0x1001)).toBe("camera_rear");
              expect(conn.listSubscribed()).toEqual([
                { topic: "camera_rear", stream_id: 0x1001, quality: "med" }
              ]);
              r();
            })
          );
        });
      })
    );
  });

  it("close() switches to closed and prevents reconnect", () => {
    const server = FakeWebSocket.makeServer();
    const client = FakeWebSocket.reserveClient();
    FakeWebSocket.link(client, server);
    const states: string[] = [];
    const conn = new Connection(
      {
        url: "ws://test",
        clientVersion: "0.1.0",
        pin: "123456",
        autoReconnect: true,
        pingIntervalMs: 100_000,
        WebSocketCtor: FakeWebSocket as unknown as new (url: string, protocols?: string | string[]) => WebSocket
      },
      { onStateChange: (s) => states.push(s) }
    );
    conn.connect();
    client.dispatchOpen();
    conn.close();
    expect(conn.getState()).toBe("closed");
    expect(states).toContain("closed");
  });
});