// Unit-тесты для PreviewPlayer — без реального AudioContext.
// Используем заглушку AudioContext с decodeAudioData и createBufferSource.

import { describe, it, expect, beforeEach } from "vitest";
import { PreviewPlayer, makePreviewRequestId } from "../src/audio/preview_player";

class FakeAudioBufferSourceNode {
  buffer: unknown = null;
  onended: (() => void) | null = null;
  connected = false;
  started = false;
  stopped = false;
  connect(_dest: unknown): void {
    this.connected = true;
  }
  disconnect(): void {
    this.connected = false;
  }
  start(): void {
    this.started = true;
  }
  stop(): void {
    this.stopped = true;
    if (this.onended) this.onended();
  }
}

class FakeAudioBuffer {
  constructor(public duration: number) {}
}

class FakeAudioContext {
  destination = {};
  decodeResult: AudioBuffer | { duration: number } | Error = new FakeAudioBuffer(1.0);
  decodeCalls = 0;
  sources: FakeAudioBufferSourceNode[] = [];
  closed = false;
  decodeAudioData(_buf: ArrayBuffer): Promise<AudioBuffer> {
    this.decodeCalls += 1;
    if (this.decodeResult instanceof Error) return Promise.reject(this.decodeResult);
    return Promise.resolve(this.decodeResult as unknown as AudioBuffer);
  }
  createBufferSource(): AudioBufferSourceNode {
    const src = new FakeAudioBufferSourceNode();
    this.sources.push(src);
    return src as unknown as AudioBufferSourceNode;
  }
  close(): Promise<void> {
    this.closed = true;
    return Promise.resolve();
  }
}

describe("PreviewPlayer", () => {
  let player: PreviewPlayer;
  // Хранилище ВСЕХ созданных FakeAudioContext, чтобы тесты могли
  // проверять, что play() дёргает decodeAudioData на свежем инстансе.
  let instances: FakeAudioContext[];

  beforeEach(() => {
    instances = [];
    const Ctor = class extends FakeAudioContext {
      constructor() {
        super();
        instances.push(this);
      }
    };
    player = new PreviewPlayer({
      AudioContextCtor: Ctor as unknown as typeof AudioContext
    });
  });

  function lastCtx(): FakeAudioContext {
    const c = instances[instances.length - 1];
    if (!c) throw new Error("no AudioContext instance created");
    return c;
  }

  it("initial state is idle and no AudioContext created yet", () => {
    expect(player.getState()).toBe("idle");
    expect(instances.length).toBe(0);
  });

  it("play creates AudioContext on first call and decodes bytes", async () => {
    const transitions: string[] = [];
    const errors: string[] = [];
    player.setListeners({
      onPlayingChange: (s) => transitions.push(s),
      onError: (r) => errors.push(r)
    });
    await player.play(new Uint8Array([1, 2, 3, 4]), "audio/mpeg");
    const ctx = lastCtx();
    expect(ctx.decodeCalls).toBe(1);
    expect(errors).toEqual([]);
    expect(player.getState()).toBe("playing");
    expect(transitions).toEqual(["playing"]);
    expect(player.getLastPlayedContentType()).toBe("audio/mpeg");
    expect(player.getLastPlayedDurationMs()).toBe(1000);
  });

  it("stop() switches state to stopped and stops current source", async () => {
    await player.play(new Uint8Array([1, 2]), "audio/mpeg");
    expect(player.getState()).toBe("playing");
    const ctx = lastCtx();
    const src = ctx.sources[0];
    expect(src).toBeDefined();
    player.stop();
    expect(src.stopped).toBe(true);
    expect(player.getState()).toBe("stopped");
  });

  it("play while playing → stop previous and start new (no concurrent sources)", async () => {
    await player.play(new Uint8Array([1, 2]), "audio/mpeg");
    const ctx1 = lastCtx();
    const first = ctx1.sources[0];
    expect(first.started).toBe(true);
    await player.play(new Uint8Array([3, 4]), "audio/opus");
    expect(first.stopped).toBe(true);
    // Второй source создан в ТОМ ЖЕ context (ensureContext переиспользует).
    expect(ctx1.sources.length).toBe(2);
    expect(ctx1.sources[1].started).toBe(true);
    expect(player.getLastPlayedContentType()).toBe("audio/opus");
  });

  it("empty bytes → fail with error state, no decode call", async () => {
    const errors: string[] = [];
    player.setListeners({ onError: (r) => errors.push(r) });
    await player.play(new Uint8Array(0), "audio/mpeg");
    expect(player.getState()).toBe("error");
    expect(errors.length).toBe(1);
    expect(errors[0]).toBe("empty_audio");
    expect(instances.length).toBe(0);
  });

  it("decode error → fail with decode_error reason", async () => {
    const errors: string[] = [];
    player.setListeners({ onError: (r) => errors.push(r) });
    // First play to create AudioContext.
    await player.play(new Uint8Array([1, 2]), "audio/mpeg");
    const ctx = lastCtx();
    // Now make subsequent decodes reject.
    ctx.decodeResult = new Error("bad-mp3");
    await player.play(new Uint8Array([5, 6]), "audio/mpeg");
    expect(player.getState()).toBe("error");
    // Последний push в errors — decode_error (предыдущие успехи ошибок не дают).
    expect(errors.length).toBeGreaterThanOrEqual(1);
    expect(errors[errors.length - 1]).toContain("decode_error");
  });

  it("stop when idle → no-op (does not crash)", () => {
    expect(() => player.stop()).not.toThrow();
    expect(player.getState()).toBe("idle");
  });

  it("stop after onended → state stays 'stopped'", async () => {
    await player.play(new Uint8Array([1, 2]), "audio/mpeg");
    const src = lastCtx().sources[0];
    player.stop();
    // Теперь симулируем браузерный onended.
    if (src.onended) src.onended();
    // Состояние уже 'stopped', onended не должен ничего менять.
    expect(player.getState()).toBe("stopped");
  });

  it("onended during playing → state becomes idle (natural end)", async () => {
    await player.play(new Uint8Array([1, 2]), "audio/mpeg");
    const src = lastCtx().sources[0];
    // Не вызываем stop(), симулируем natural end через onended.
    if (src.onended) src.onended();
    expect(player.getState()).toBe("idle");
  });

  it("disableAudioContext=true → throws on play", async () => {
    const p = new PreviewPlayer({
      enableAudioContext: false
    });
    const errors: string[] = [];
    p.setListeners({ onError: (r) => errors.push(r) });
    await p.play(new Uint8Array([1, 2]), "audio/mpeg");
    expect(p.getState()).toBe("error");
    expect(errors[0]).toBe("decode_error: AudioContext disabled");
  });

  it("dispose() closes context and clears listeners", async () => {
    await player.play(new Uint8Array([1, 2]), "audio/mpeg");
    const ctx = lastCtx();
    player.dispose();
    expect(ctx.closed).toBe(true);
  });

  it("makePreviewRequestId: unique-ish format", () => {
    const id = makePreviewRequestId();
    expect(id.startsWith("pv_")).toBe(true);
    expect(id.length).toBeGreaterThan(10);
    const id2 = makePreviewRequestId();
    expect(id).not.toBe(id2);
  });

  it("regression: stop() twice → no error", async () => {
    await player.play(new Uint8Array([1, 2]), "audio/mpeg");
    player.stop();
    expect(() => player.stop()).not.toThrow();
  });

  it("regression: play after stop works", async () => {
    await player.play(new Uint8Array([1, 2]), "audio/mpeg");
    player.stop();
    await player.play(new Uint8Array([5, 6, 7]), "audio/opus");
    expect(player.getState()).toBe("playing");
    expect(player.getLastPlayedContentType()).toBe("audio/opus");
  });
});
