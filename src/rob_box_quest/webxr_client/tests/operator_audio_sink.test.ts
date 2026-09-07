// ADR-0055 / issue #1993 — проигрыватель обратного аудио ТАРС в шлем
// (src/ui/operator_audio_sink.ts).
//
// Контракт симметричен preview_audio_sink.test.ts: те же проверки
// буферизации, склейки и проигрывания, плюс две вещи, специфичные
// для обратного канала:
//
//   1. ``error(reason)`` — сбрасывает буфер (а не молча, как ``stop()``)
//      и логирует reason. Сервер сейчас его не шлёт, но тип заведён
//      для forward-compat.
//   2. Barge-in: ``stop()`` ДОЛЖЕН быть достаточен для тишины в шлеме
//      (preview-sink тоже это умеет, но тут это контракт по ADR-0055
//      §webxr_client).

import { describe, it, expect, vi } from "vitest";
import {
  createOperatorAudioSink,
  OPERATOR_TTS_MAX_BYTES
} from "../src/ui/operator_audio_sink";

interface FakeSource {
  buffer: unknown;
  connected: boolean;
  started: boolean;
  stopped: boolean;
}

/** Минимальный AudioContext: запоминает, что декодировали и что играли. */
function makeFakeCtx() {
  const decoded: ArrayBuffer[] = [];
  const sources: FakeSource[] = [];
  let failDecode = false;
  const ctx = {
    destination: {},
    decodeAudioData: vi.fn(async (ab: ArrayBuffer) => {
      if (failDecode) throw new Error("bad format");
      decoded.push(ab);
      return { duration: 1 } as unknown as AudioBuffer;
    }),
    createBufferSource: vi.fn(() => {
      const src: FakeSource = { buffer: null, connected: false, started: false, stopped: false };
      sources.push(src);
      return {
        set buffer(b: unknown) { src.buffer = b; },
        get buffer() { return src.buffer; },
        connect: () => { src.connected = true; },
        start: () => { src.started = true; },
        stop: () => { src.stopped = true; },
        disconnect: () => { src.connected = false; }
      } as unknown as AudioBufferSourceNode;
    }),
    close: vi.fn(async () => undefined)
  };
  return {
    decoded,
    sources,
    ctx,
    failNextDecode(): void {
      failDecode = true;
    }
  };
}

function sinkWithFake() {
  const fake = makeFakeCtx();
  const sink = createOperatorAudioSink({
    AudioContextCtor: function () {
      return fake.ctx;
    } as unknown as new () => AudioContext
  });
  return { sink, fake };
}

describe("operator audio sink — приём чанков (ADR-0055 §webxr_client)", () => {
  it("байты без меты отбрасываются (не играем мусор)", () => {
    const { sink } = sinkWithFake();
    expect(sink.onChunk(new Uint8Array([1, 2, 3]))).toBe(false);
  });

  it("мета + байты копятся под своим request_id", () => {
    const { sink } = sinkWithFake();
    sink.onMeta("op1", "audio/pcm", 0, 2);
    expect(sink.onChunk(new Uint8Array([1, 2, 3]))).toBe(true);
    sink.onMeta("op1", "audio/pcm", 1, 2);
    expect(sink.onChunk(new Uint8Array([4, 5]))).toBe(true);
    expect(sink.bufferedBytes("op1")).toBe(5);
  });

  it("превышение OPERATOR_TTS_MAX_BYTES роняет реплику целиком", () => {
    const { sink } = sinkWithFake();
    sink.onMeta("op1", "audio/pcm", 0, 1);
    expect(sink.onChunk(new Uint8Array(OPERATOR_TTS_MAX_BYTES + 1))).toBe(false);
    expect(sink.bufferedBytes("op1")).toBe(0);
  });

  it("копия байтов: мутация исходного буфера сокета не портит накопленное", async () => {
    const { sink, fake } = sinkWithFake();
    const shared = new Uint8Array([1, 2, 3]);
    sink.onMeta("op1", "audio/pcm", 0, 1);
    sink.onChunk(shared);
    shared[0] = 99;
    await sink.play("op1");
    // Декодировали исходный байт 1, а не мутированный 99.
    expect([...new Uint8Array(fake.decoded[0])]).toEqual([1, 2, 3]);
  });
});

describe("operator audio sink — проигрывание и barge-in (ADR-0055 §barge-in)", () => {
  it("play склеивает чанки по порядку и запускает source", async () => {
    const { sink, fake } = sinkWithFake();
    sink.onMeta("op1", "audio/pcm", 0, 2);
    sink.onChunk(new Uint8Array([1, 2]));
    sink.onMeta("op1", "audio/pcm", 1, 2);
    sink.onChunk(new Uint8Array([3, 4, 5]));
    await sink.play("op1");
    expect(fake.decoded).toHaveLength(1);
    expect([...new Uint8Array(fake.decoded[0])]).toEqual([1, 2, 3, 4, 5]);
    expect(fake.sources[0]).toMatchObject({ started: true, connected: true });
  });

  it("barge-in: stop() прерывает проигрывание и выбрасывает накопленное", async () => {
    const { sink, fake } = sinkWithFake();
    sink.onMeta("op1", "audio/pcm", 0, 1);
    sink.onChunk(new Uint8Array([1]));
    await sink.play("op1");
    // Источник уже играет.
    expect(fake.sources[0].started).toBe(true);

    // Новый request_id пришёл, но не успел доиграть — PTT нажат.
    sink.onMeta("op2", "audio/pcm", 0, 1);
    sink.onChunk(new Uint8Array([7, 7, 7]));
    sink.stop();
    // Проигрывание оборвано, буфер выброшен — оператор слышит тишину
    // ДО того, как сервер обработает STOP и прекратит слать чанки.
    expect(fake.sources[0].stopped).toBe(true);
    expect(sink.bufferedBytes("op2")).toBe(0);
  });

  it("barge-in: после stop() второй чанк того же request_id игнорируется", () => {
    const { sink } = sinkWithFake();
    sink.onMeta("op1", "audio/pcm", 0, 2);
    sink.onChunk(new Uint8Array([1]));
    sink.stop();
    // currentRequestId сброшен — следующий чанк отбрасывается (мета потеряна).
    expect(sink.onChunk(new Uint8Array([2]))).toBe(false);
  });

  it("error(reason) сбрасывает буфер и логирует причину", async () => {
    const { sink, fake } = sinkWithFake();
    const warn = vi.spyOn(console, "warn").mockImplementation(() => undefined);
    sink.onMeta("op1", "audio/pcm", 0, 1);
    sink.onChunk(new Uint8Array([1, 2]));
    await sink.play("op1");
    expect(fake.sources[0].started).toBe(true);

    sink.onMeta("op2", "audio/pcm", 0, 1);
    sink.onChunk(new Uint8Array([7]));
    sink.error("synth_failed");
    expect(sink.bufferedBytes("op2")).toBe(0);
    expect(fake.sources[0].stopped).toBe(true);
    // Лог содержит reason — для диагностики в headset HUD.
    expect(warn).toHaveBeenCalledWith(
      "[quest] operator_tts error:",
      "synth_failed"
    );
    warn.mockRestore();
  });

  it("новый play останавливает предыдущий source (не наслаиваем голоса)", async () => {
    const { sink, fake } = sinkWithFake();
    sink.onMeta("op1", "audio/pcm", 0, 1);
    sink.onChunk(new Uint8Array([1]));
    await sink.play("op1");
    sink.onMeta("op2", "audio/pcm", 0, 1);
    sink.onChunk(new Uint8Array([2]));
    await sink.play("op2");
    expect(fake.sources[0].stopped).toBe(true);
    expect(fake.sources[1].started).toBe(true);
  });

  it("dispose закрывает контекст и чистит всё", async () => {
    const { sink, fake } = sinkWithFake();
    sink.onMeta("op1", "audio/pcm", 0, 1);
    sink.onChunk(new Uint8Array([1]));
    await sink.play("op1");
    sink.dispose();
    expect(fake.ctx.close).toHaveBeenCalled();
    expect(sink.bufferedBytes("op1")).toBe(0);
  });
});

describe("operator audio sink — без AudioContext в окружении", () => {
  it("play не падает, если AudioContext недоступен", async () => {
    const sink = createOperatorAudioSink({
      AudioContextCtor: undefined as unknown as new () => AudioContext
    });
    sink.onMeta("op1", "audio/pcm", 0, 1);
    sink.onChunk(new Uint8Array([1, 2]));
    await expect(sink.play("op1")).resolves.toBeUndefined();
    sink.dispose();
  });
});
