// Диагностика: воспроизводит РЕАЛЬНЫЙ порядок вызовов из main.ts
// (case "operator_tts_audio"), а не порядок, который используют
// существующие тесты operator_audio_sink.test.ts.
//
// main.ts делает: onMeta(...) → void play(requestId) СРАЗУ (синхронно,
// до прихода BINARY_FRAME с байтами) — см. main.ts:1074-1082.
// BINARY_FRAME — отдельное WS-сообщение, обрабатывается ПОЗЖЕ через
// onBinaryFrame → operatorAudioSink.onChunk(payload) (main.ts:1349).
//
// Существующие тесты вызывают onChunk() ДО play() — это НЕ тот порядок,
// который реально происходит в проде. Этот тест использует настоящий
// порядок: onMeta → play() → onChunk (bytes приходят позже).

import { describe, it, expect, vi } from "vitest";
import { createOperatorAudioSink } from "../src/ui/operator_audio_sink";

function makeFakeCtx() {
  const sources: Array<{ started: boolean; connected: boolean }> = [];
  const ctx = {
    destination: {},
    state: "running" as AudioContextState,
    decodeAudioData: vi.fn(async () => ({ duration: 1 }) as unknown as AudioBuffer),
    createBuffer: vi.fn((_c: number, _len: number, _sr: number) => {
      return { copyToChannel: () => undefined } as unknown as AudioBuffer;
    }),
    resume: vi.fn(async () => undefined),
    createBufferSource: vi.fn(() => {
      const src = { started: false, connected: false };
      sources.push(src);
      return {
        set buffer(_b: unknown) {},
        get buffer() { return null; },
        connect: () => { src.connected = true; },
        start: () => { src.started = true; },
        stop: () => {},
        disconnect: () => {},
        onended: null
      } as unknown as AudioBufferSourceNode;
    }),
    close: vi.fn(async () => undefined)
  };
  return { ctx, sources };
}

describe("REPRO: реальный порядок main.ts (meta -> play() -> binary chunk)", () => {
  it("play() вызванный СРАЗУ после onMeta (до onChunk) — чанк теряется", async () => {
    const { ctx, sources } = makeFakeCtx();
    const sink = createOperatorAudioSink({
      AudioContextCtor: function () { return ctx; } as unknown as new () => AudioContext
    });

    // main.ts: case "operator_tts_audio" → onMeta(...) затем void play(...)
    sink.onMeta("req1", "audio/pcm", 0, 1, 22050);
    void sink.play("req1"); // main.ts:1082 — СИНХРОННО, ДО прихода BINARY_FRAME

    // ... позже приходит отдельное WS-сообщение BINARY_FRAME со сырыми байтами.
    const consumed = sink.onChunk(new Uint8Array([1, 2, 3, 4]));

    console.log("[repro] onChunk consumed:", consumed);
    console.log("[repro] bufferedBytes(req1):", sink.bufferedBytes("req1"));
    console.log("[repro] sources.length:", sources.length);
    console.log("[repro] source[0]?.started:", sources[0]?.started);

    // Ожидаемое (баг): чанк отброшен (мета "потеряна" — play() уже сбросил
    // currentRequestId), НИ ОДИН source не создан → ТИШИНА в шлеме.
    expect(consumed).toBe(false);
    expect(sources.length).toBe(0);
  });

  it("ПОСЛЕ ФИКСА: play() вызванный ПОСЛЕ onChunk() — чанк играет", async () => {
    const { ctx, sources } = makeFakeCtx();
    const sink = createOperatorAudioSink({
      AudioContextCtor: function () { return ctx; } as unknown as new () => AudioContext
    });

    // Новый порядок main.ts: onMeta(...) → onChunk(bytes) на BINARY_FRAME
    // → play() ТОЛЬКО если onChunk вернул true (байты реально накоплены).
    sink.onMeta("req1", "audio/pcm", 0, 1, 22050);
    const consumed = sink.onChunk(new Uint8Array([1, 2, 3, 4]));
    expect(consumed).toBe(true);
    if (consumed) await sink.play("req1");

    console.log("[repro-fixed] sources.length:", sources.length);
    console.log("[repro-fixed] source[0]?.started:", sources[0]?.started);

    expect(sources.length).toBe(1);
    expect(sources[0].started).toBe(true);
    expect(sources[0].connected).toBe(true);
  });
});
