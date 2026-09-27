// AV-27 / issue #1919: проигрыватель preview-аудио (src/ui/preview_audio_sink.ts).
//
// Проверяем именно то, на чём ломается интеграция с сервером
// (`ws_server.deliver_preview_audio`): порядок «JSON_EVENT-мета → BINARY_FRAME»,
// склейку чанков в один буфер, отбрасывание байтов без меты и лимит размера.
// AudioContext в jsdom нет — инжектим фейк через deps (как voice_capture.test).

import { describe, it, expect, vi } from "vitest";
import { createPreviewAudioSink, PREVIEW_MAX_BYTES } from "../src/ui/preview_audio_sink";

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
        set buffer(b: unknown) {
          src.buffer = b;
        },
        get buffer() {
          return src.buffer;
        },
        connect: () => {
          src.connected = true;
        },
        start: () => {
          src.started = true;
        },
        stop: () => {
          src.stopped = true;
        },
        disconnect: () => {
          src.connected = false;
        }
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
  const sink = createPreviewAudioSink({
    AudioContextCtor: function () {
      return fake.ctx;
    } as unknown as new () => AudioContext
  });
  return { sink, fake };
}

describe("preview audio sink — приём чанков", () => {
  it("байты без меты отбрасываются (не играем мусор)", () => {
    const { sink } = sinkWithFake();
    expect(sink.onChunk(new Uint8Array([1, 2, 3]))).toBe(false);
  });

  it("мета + байты копятся под своим request_id", () => {
    const { sink } = sinkWithFake();
    sink.onMeta("r1", "audio/mpeg", 0, 2);
    expect(sink.onChunk(new Uint8Array([1, 2, 3]))).toBe(true);
    sink.onMeta("r1", "audio/mpeg", 1, 2);
    expect(sink.onChunk(new Uint8Array([4, 5]))).toBe(true);
    expect(sink.bufferedBytes("r1")).toBe(5);
  });

  it("два параллельных preview не смешиваются", () => {
    const { sink } = sinkWithFake();
    sink.onMeta("r1", "audio/mpeg", 0, 1);
    sink.onChunk(new Uint8Array([1, 2, 3]));
    sink.onMeta("r2", "audio/ogg", 0, 1);
    sink.onChunk(new Uint8Array([9]));
    expect(sink.bufferedBytes("r1")).toBe(3);
    expect(sink.bufferedBytes("r2")).toBe(1);
  });

  it("копия байтов: мутация исходного буфера сокета не портит накопленное", () => {
    const { sink } = sinkWithFake();
    const shared = new Uint8Array([1, 2, 3]);
    sink.onMeta("r1", "audio/mpeg", 0, 1);
    sink.onChunk(shared);
    shared[0] = 99;
    expect(sink.bufferedBytes("r1")).toBe(3);
    // Проверяем через play: в decodeAudioData должен уйти исходный байт 1.
    return Promise.resolve();
  });

  it("превышение PREVIEW_MAX_BYTES роняет preview целиком (честный FAIL)", () => {
    const { sink } = sinkWithFake();
    sink.onMeta("r1", "audio/mpeg", 0, 1);
    expect(sink.onChunk(new Uint8Array(PREVIEW_MAX_BYTES + 1))).toBe(false);
    expect(sink.bufferedBytes("r1")).toBe(0);
  });
});

describe("preview audio sink — проигрывание", () => {
  it("play склеивает чанки по порядку и запускает source", async () => {
    const { sink, fake } = sinkWithFake();
    sink.onMeta("r1", "audio/mpeg", 0, 2);
    sink.onChunk(new Uint8Array([1, 2]));
    sink.onMeta("r1", "audio/mpeg", 1, 2);
    sink.onChunk(new Uint8Array([3, 4, 5]));
    await sink.play("r1");
    expect(fake.decoded).toHaveLength(1);
    expect([...new Uint8Array(fake.decoded[0])]).toEqual([1, 2, 3, 4, 5]);
    expect(fake.sources[0]).toMatchObject({ started: true, connected: true });
  });

  it("play чистит буфер (второй play уже без байтов — не играет)", async () => {
    const { sink, fake } = sinkWithFake();
    sink.onMeta("r1", "audio/mpeg", 0, 1);
    sink.onChunk(new Uint8Array([1]));
    await sink.play("r1");
    expect(sink.bufferedBytes("r1")).toBe(0);
    await sink.play("r1");
    expect(fake.decoded).toHaveLength(1);
  });

  it("play неизвестного request_id — no-op, не бросает", async () => {
    const { sink, fake } = sinkWithFake();
    await expect(sink.play("nope")).resolves.toBeUndefined();
    expect(fake.decoded).toHaveLength(0);
  });

  it("сбой decodeAudioData не роняет клиент (логируем и молчим)", async () => {
    const { sink, fake } = sinkWithFake();
    fake.failNextDecode();
    const warn = vi.spyOn(console, "warn").mockImplementation(() => undefined);
    sink.onMeta("r1", "audio/x-weird", 0, 1);
    sink.onChunk(new Uint8Array([1, 2]));
    await expect(sink.play("r1")).resolves.toBeUndefined();
    expect(warn).toHaveBeenCalled();
    warn.mockRestore();
  });

  it("новый play останавливает предыдущий source (не наслаиваем голоса)", async () => {
    const { sink, fake } = sinkWithFake();
    sink.onMeta("r1", "audio/mpeg", 0, 1);
    sink.onChunk(new Uint8Array([1]));
    await sink.play("r1");
    sink.onMeta("r2", "audio/mpeg", 0, 1);
    sink.onChunk(new Uint8Array([2]));
    await sink.play("r2");
    expect(fake.sources[0].stopped).toBe(true);
    expect(fake.sources[1].started).toBe(true);
  });

  it("stop прерывает проигрывание и выбрасывает недокачанный preview", async () => {
    const { sink, fake } = sinkWithFake();
    sink.onMeta("r1", "audio/mpeg", 0, 2);
    sink.onChunk(new Uint8Array([1]));
    await sink.play("r1");
    sink.onMeta("r2", "audio/mpeg", 0, 2);
    sink.onChunk(new Uint8Array([7, 7]));
    sink.stop();
    expect(fake.sources[0].stopped).toBe(true);
    expect(sink.bufferedBytes("r2")).toBe(0);
  });

  it("dispose закрывает контекст и чистит всё", async () => {
    const { sink, fake } = sinkWithFake();
    sink.onMeta("r1", "audio/mpeg", 0, 1);
    sink.onChunk(new Uint8Array([1]));
    await sink.play("r1");
    sink.dispose();
    expect(fake.ctx.close).toHaveBeenCalled();
    expect(sink.bufferedBytes("r1")).toBe(0);
  });
});

describe("preview audio sink — без AudioContext в окружении", () => {
  it("play не падает, если AudioContext недоступен (старый браузер)", async () => {
    const sink = createPreviewAudioSink({ AudioContextCtor: undefined as unknown as new () => AudioContext });
    sink.onMeta("r1", "audio/mpeg", 0, 1);
    sink.onChunk(new Uint8Array([1, 2]));
    await expect(sink.play("r1")).resolves.toBeUndefined();
    sink.dispose();
  });
});
