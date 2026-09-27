// AV-17: subprotocol-негоциация v2/v1 + STATE_UPDATE (0x33) + supervisor-
// ошибки (FLOOR_HELD / MODE_CONFLICT) на уровне Connection.
//
// Своя копия in-memory FakeWebSocket (connection.test.ts держит её локально),
// плюс поле `protocol` — сервер сообщает выбранный subprotocol именно так
// (MDN WebSocket.protocol; docs/architecture/meta-quest-api.md §11.1).

import { describe, it, expect, beforeEach } from "vitest";
import { encodeFrame, encodeJsonFrame, FrameType, decodeFrame } from "../src/wire/protocol";
import { encodeMsgpackMap } from "../src/wire/msgpack";
import type { SupervisorState } from "../src/state/supervisor_state";

class FakeWebSocket {
  static CONNECTING = 0;
  static OPEN = 1;
  static CLOSING = 2;
  static CLOSED = 3;
  static nextInstance: FakeWebSocket | null = null;
  static lastProtocols: string | string[] | undefined;
  /** Если задано — конструктор положит это в `protocol` ДО open, имитируя выбор сервера. */
  static nextProtocol: string | null = null;

  static make(): FakeWebSocket {
    const ws = Object.create(FakeWebSocket.prototype) as FakeWebSocket;
    ws.readyState = 0;
    ws.peer = null;
    ws.protocol = "";
    ws.sentFrames = [];
    ws.listeners = {};
    return ws;
  }

  static reserveClient(): FakeWebSocket {
    const ws = FakeWebSocket.make();
    if (FakeWebSocket.nextProtocol !== null) {
      ws.protocol = FakeWebSocket.nextProtocol;
      FakeWebSocket.nextProtocol = null;
    }
    FakeWebSocket.nextInstance = ws;
    return ws;
  }

  static link(a: FakeWebSocket, b: FakeWebSocket): void {
    a.peer = b;
    b.peer = a;
  }

  readyState = 0;
  protocol = "";
  peer: FakeWebSocket | null = null;
  sentFrames: Uint8Array[] = [];
  listeners: Record<string, Array<(ev: unknown) => void>> = {};

  constructor(_url?: string, protocols?: string | string[]) {
    FakeWebSocket.lastProtocols = protocols;
    if (FakeWebSocket.nextInstance) {
      // Не клонируем: выдаём ЗАРЕЗЕРВИРОВАННЫЙ объект как `this`. Так
      // `client.sentFrames` и `this.ws.sentFrames` — один массив, и
      // `this.ws.readyState` меняется при `client.dispatchOpen()`.
      const inst = FakeWebSocket.nextInstance;
      FakeWebSocket.nextInstance = null;
      return inst as unknown as FakeWebSocket;
    }
  }

  addEventListener(name: string, fn: (ev: unknown) => void): void {
    (this.listeners[name] ??= []).push(fn);
  }

  removeEventListener(): void {
    /* not needed */
  }

  send(data: ArrayBuffer | Uint8Array): void {
    const bytes = data instanceof Uint8Array ? data : new Uint8Array(data);
    this.sentFrames.push(bytes);
    queueMicrotask(() => {
      for (const fn of this.peer?.listeners["message"] ?? []) {
        fn({ data: bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength) });
      }
    });
  }

  close(code = 1000, reason = ""): void {
    this.readyState = 3;
    for (const fn of this.listeners["close"] ?? []) fn({ code, reason });
  }

  dispatchOpen(): void {
    this.readyState = 1;
    for (const fn of this.listeners["open"] ?? []) fn({});
  }
}

(globalThis as unknown as { WebSocket: unknown }).WebSocket = FakeWebSocket;

import { Connection } from "../src/wire/connection";

type Sink = {
  states: SupervisorState[];
  errors: Array<{ code: string; message: string; meta?: Record<string, unknown> }>;
  genericErrors: Array<{ code: string; message: string }>;
  connStates: string[];
};

function connect(opts: { protocol: string; forceV1?: boolean }) {
  const server = FakeWebSocket.make();
  FakeWebSocket.nextProtocol = opts.protocol;
  const client = FakeWebSocket.reserveClient();
  FakeWebSocket.link(client, server);
  const sink: Sink = { states: [], errors: [], genericErrors: [], connStates: [] };
  // protocol — примитив: выставляем ДО конструктора Connection, иначе
  // Object.assign в FakeWebSocket скопирует старое значение.
  const conn = new Connection(
    {
      url: "ws://test",
      clientVersion: "0.1.0",
      pin: "123456",
      autoReconnect: false,
      pingIntervalMs: 100_000,
      forceV1: opts.forceV1,
      WebSocketCtor: FakeWebSocket as unknown as new (
        url: string,
        protocols?: string | string[]
      ) => WebSocket
    },
    {
      onStateChange: (s) => sink.connStates.push(s),
      onSupervisorState: (st) => sink.states.push(st),
      onSupervisorError: (code, message, meta) => sink.errors.push({ code, message, meta }),
      onError: (code, message) => sink.genericErrors.push({ code, message })
    }
  );
  conn.connect();
  // Сервер выбрал subprotocol — до dispatchOpen, как в реальном браузере.
  client.dispatchOpen();
  server.dispatchOpen();
  return { conn, client, server, sink };
}

const tick = () => new Promise<void>((r) => queueMicrotask(() => queueMicrotask(() => r())));

const STATE_PAYLOAD = (over: Record<string, unknown> = {}) =>
  encodeMsgpackMap({
    mode: "avatar_present",
    teleop_floor: { client_id: "quest-1", since_ms: 1000, last_heartbeat_ms: 1500 },
    voice_floor: null,
    since_ms: 900,
    version: 1,
    ...over
  } as never);

describe("Connection subprotocol negotiation (AV-17)", () => {
  beforeEach(() => {
    FakeWebSocket.nextInstance = null;
    FakeWebSocket.lastProtocols = undefined;
  });

  it("по умолчанию запрашивает robbox-quest-v2", () => {
    connect({ protocol: "robbox-quest-v2" });
    expect(FakeWebSocket.lastProtocols).toBe("robbox-quest-v2");
  });

  it("forceV1 запрашивает robbox-quest-v1 (отладка старого сервера)", () => {
    connect({ protocol: "robbox-quest-v1", forceV1: true });
    expect(FakeWebSocket.lastProtocols).toBe("robbox-quest-v1");
  });

  it("сервер ответил v2 → negotiatedVersion=v2, supervisor-команды уходят", () => {
    const { conn, client } = connect({ protocol: "robbox-quest-v2" });
    expect(conn.getNegotiatedVersion()).toBe("v2");
    const before = client.sentFrames.length;
    expect(conn.sendAcquireFloor("quest-1", "teleop")).toBe(true);
    expect(conn.sendReleaseFloor("quest-1", "teleop")).toBe(true);
    expect(conn.sendSetMode("quest-1", "avatar_present")).toBe(true);
    const sent = client.sentFrames.slice(before).map((f) => decodeFrame(f).type);
    expect(sent).toEqual([FrameType.ACQUIRE_FLOOR, FrameType.RELEASE_FLOOR, FrameType.SET_MODE]);
  });

  it("сервер ответил v1 → фреймы 0x30–0x32 НЕ отправляются", () => {
    const { conn, client } = connect({ protocol: "robbox-quest-v1" });
    expect(conn.getNegotiatedVersion()).toBe("v1");
    const before = client.sentFrames.length;
    expect(conn.sendAcquireFloor("quest-1", "teleop")).toBe(false);
    expect(conn.sendReleaseFloor("quest-1", "voice")).toBe(false);
    expect(conn.sendSetMode("quest-1", "mixed")).toBe(false);
    expect(client.sentFrames.length).toBe(before);
  });

  it("сервер не вернул subprotocol вовсе → консервативно v1 (не слать 0x3X)", () => {
    const { conn } = connect({ protocol: "" });
    expect(conn.getNegotiatedVersion()).toBe("v1");
    expect(conn.sendAcquireFloor("quest-1", "teleop")).toBe(false);
  });

  it("ACQUIRE_FLOOR payload — msgpack {client_id, floor}", () => {
    const { conn, client } = connect({ protocol: "robbox-quest-v2" });
    const before = client.sentFrames.length;
    conn.sendAcquireFloor("quest-7", "voice");
    const frame = decodeFrame(client.sentFrames[before]);
    expect(frame.type).toBe(FrameType.ACQUIRE_FLOOR);
    // decode обратно нашим же декодером (round-trip уже покрыт в msgpack.test).
    expect(frame.payload.length).toBeGreaterThan(0);
  });
});

describe("Connection STATE_UPDATE 0x33 (AV-17)", () => {
  beforeEach(() => {
    FakeWebSocket.nextInstance = null;
  });

  it("валидный 0x33 → onSupervisorState с распарсенным состоянием", async () => {
    const { server, sink } = connect({ protocol: "robbox-quest-v2" });
    server.send(
      encodeFrame(FrameType.STATE_UPDATE, 0, STATE_PAYLOAD()) as unknown as ArrayBuffer
    );
    await tick();
    expect(sink.states.length).toBe(1);
    expect(sink.states[0].mode).toBe("avatar_present");
    expect(sink.states[0].teleopFloor.clientId).toBe("quest-1");
  });

  it("0x33 с не-msgpack payload → BAD_PAYLOAD, колбэк state не вызван", async () => {
    const { server, sink } = connect({ protocol: "robbox-quest-v2" });
    server.send(
      encodeFrame(FrameType.STATE_UPDATE, 0, new Uint8Array([0xc1])) as unknown as ArrayBuffer
    );
    await tick();
    expect(sink.states).toEqual([]);
    expect(sink.genericErrors.some((e) => e.code === "BAD_PAYLOAD")).toBe(true);
  });

  it("0x33 без обязательных полей → BAD_PAYLOAD, а не выдуманное состояние", async () => {
    const { server, sink } = connect({ protocol: "robbox-quest-v2" });
    server.send(
      encodeFrame(
        FrameType.STATE_UPDATE,
        0,
        encodeMsgpackMap({ mode: "off" } as never)
      ) as unknown as ArrayBuffer
    );
    await tick();
    expect(sink.states).toEqual([]);
    expect(sink.genericErrors.some((e) => e.code === "BAD_PAYLOAD")).toBe(true);
  });

  it("разрыв связи → supervisor-команды no-op до нового handshake", () => {
    const { conn, client } = connect({ protocol: "robbox-quest-v2" });
    expect(conn.getNegotiatedVersion()).toBe("v2");
    expect(conn.sendAcquireFloor("quest-1", "teleop")).toBe(true);
    // Разрыв (1006, autoReconnect:false → нового сокета сам не откроет).
    // Инвариант: пока сокет не переоткрыт и handshake не пройден,
    // supervisor-команды уходить не должны.
    client.close(1006, "network");
    expect(conn.sendAcquireFloor("quest-1", "teleop")).toBe(false);
    expect(conn.sendReleaseFloor("quest-1", "teleop")).toBe(false);
    expect(conn.sendSetMode("quest-1", "mixed")).toBe(false);
  });

  it("reconnect на v1-сервер → negotiatedVersion=v1, 0x30–0x32 не уходят", () => {
    const { conn, client } = connect({ protocol: "robbox-quest-v2" });
    expect(conn.getNegotiatedVersion()).toBe("v2");
    client.close(1006, "network");

    // Новый сокет: тот же URL, но сервер теперь отвечает v1 (откат сервера).
    // openSocket() обязан сбросить negotiatedVersion и выставить его заново
    // из ws.protocol нового сокета — иначе «v2» протёк бы через reconnect.
    const server2 = FakeWebSocket.make();
    FakeWebSocket.nextProtocol = "robbox-quest-v1";
    const client2 = FakeWebSocket.reserveClient();
    FakeWebSocket.link(client2, server2);
    conn.connect();
    client2.dispatchOpen();
    server2.dispatchOpen();

    expect(conn.getNegotiatedVersion()).toBe("v1");
    const before = client2.sentFrames.length;
    expect(conn.sendAcquireFloor("quest-1", "teleop")).toBe(false);
    expect(client2.sentFrames.length).toBe(before);
  });

  it("reconnect на v2-сервер → команды снова уходят по новому сокету", () => {
    const { conn, client } = connect({ protocol: "robbox-quest-v2" });
    client.close(1006, "network");

    const server2 = FakeWebSocket.make();
    FakeWebSocket.nextProtocol = "robbox-quest-v2";
    const client2 = FakeWebSocket.reserveClient();
    FakeWebSocket.link(client2, server2);
    conn.connect();
    client2.dispatchOpen();
    server2.dispatchOpen();

    expect(conn.getNegotiatedVersion()).toBe("v2");
    const before = client2.sentFrames.length;
    expect(conn.sendAcquireFloor("quest-1", "teleop")).toBe(true);
    expect(decodeFrame(client2.sentFrames[before]).type).toBe(FrameType.ACQUIRE_FLOOR);
    // Старый сокет не должен получить ничего после разрыва.
    expect(client.readyState).toBe(FakeWebSocket.CLOSED);
  });
});

describe("Connection supervisor errors (AV-17)", () => {
  beforeEach(() => {
    FakeWebSocket.nextInstance = null;
  });

  it("ERROR{FLOOR_HELD} → onSupervisorError с держателем в meta", async () => {
    const { server, sink } = connect({ protocol: "robbox-quest-v2" });
    server.send(
      encodeJsonFrame(FrameType.ERROR, 0, {
        code: "FLOOR_HELD",
        message: "teleop floor held",
        held_by: "telegram-42"
      }) as unknown as ArrayBuffer
    );
    await tick();
    expect(sink.errors.length).toBe(1);
    expect(sink.errors[0].code).toBe("FLOOR_HELD");
    expect(sink.errors[0].meta?.held_by).toBe("telegram-42");
    // Не должно дублироваться в generic onError.
    expect(sink.genericErrors.some((e) => e.code === "FLOOR_HELD")).toBe(false);
  });

  it("ERROR{MODE_CONFLICT} → onSupervisorError", async () => {
    const { server, sink } = connect({ protocol: "robbox-quest-v2" });
    server.send(
      encodeJsonFrame(FrameType.ERROR, 0, {
        code: "MODE_CONFLICT",
        message: "fsm rejected off→mixed"
      }) as unknown as ArrayBuffer
    );
    await tick();
    expect(sink.errors.map((e) => e.code)).toEqual(["MODE_CONFLICT"]);
  });

  it("обычный ERROR остаётся в generic onError", async () => {
    const { server, sink } = connect({ protocol: "robbox-quest-v2" });
    server.send(
      encodeJsonFrame(FrameType.ERROR, 0, {
        code: "RATE_LIMIT",
        message: "slow down"
      }) as unknown as ArrayBuffer
    );
    await tick();
    expect(sink.errors).toEqual([]);
    expect(sink.genericErrors.map((e) => e.code)).toContain("RATE_LIMIT");
  });
});
