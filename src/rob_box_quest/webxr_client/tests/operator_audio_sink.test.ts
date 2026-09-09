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
  const buffers: Array<{ channel: number[] }> = [];
  const sources: FakeSource[] = [];
  let failDecode = false;
  const ctx = {
    destination: {},
    state: "running" as AudioContextState,
    decodeAudioData: vi.fn(async (ab: ArrayBuffer) => {
      if (failDecode) throw new Error("bad format");
      decoded.push(ab);
      return { duration: 1 } as unknown as AudioBuffer;
    }),
    // ADR-0078: теперь PCM идёт через createBuffer; старые тесты
    // не проверяют содержимое буфера, поэтому достаточно заглушки.
    createBuffer: vi.fn(
      (_channels: number, _length: number, _sampleRate: number): AudioBuffer => {
        const buf = { channel: [] as number[] };
        buffers.push(buf);
        return {
          copyToChannel: (data: Float32Array, channel: number) => {
            if (channel === 0) buf.channel = Array.from(data);
          }
        } as unknown as AudioBuffer;
      }
    ),
    resume: vi.fn(async () => undefined),
    createBufferSource: vi.fn(() => {
      const src: FakeSource = { buffer: null, connected: false, started: false, stopped: false };
      sources.push(src);
      return {
        set buffer(b: unknown) { src.buffer = b; },
        get buffer() { return src.buffer; },
        connect: () => { src.connected = true; },
        start: () => { src.started = true; },
        stop: () => { src.stopped = true; },
        disconnect: () => { src.connected = false; },
        onended: null
      } as unknown as AudioBufferSourceNode;
    }),
    close: vi.fn(async () => undefined)
  };
  return {
    decoded,
    buffers,
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
    sink.onMeta("op1", "audio/pcm", 0, 1, 16000);
    sink.onChunk(shared);
    shared[0] = 99;
    await sink.play("op1");
    // ADR-0078: PCM идёт через createBuffer+copyToChannel, не decodeAudioData.
    // Проверяем, что в channel записаны исходные int16-семплы [1, 2, 3], а не
    // мутированный [99, 2, 3].
    const createBuffer = fake.ctx.createBuffer as ReturnType<typeof vi.fn>;
    expect(createBuffer).toHaveBeenCalled();
    // createBuffer вернул объект с copyToChannel; vi.fn(mockFn) с реализацией
    // возвращает значение напрямую (синхронно), но mock.results[0] не всегда
    // заполняется — возьмём последний вызов и подменим — проще через явный
    // вызов и сравнение с тем, что copyToChannel записал.
    const buf = createBuffer.mock.results[0]?.value as
      | { channel: number[] }
      | undefined;
    // Если vi.fn не вернул результат через .results — fallback на buffers[0].
    const fakeBuffers = (fake as unknown as {
      buffers?: Array<{ channel: number[] }>;
    }).buffers;
    const storedBuf =
      buf && buf.channel !== undefined
        ? buf
        : fakeBuffers && fakeBuffers[0];
    expect(storedBuf).toBeDefined();
    if (!storedBuf) throw new Error("storedBuf is undefined");
    const ch = storedBuf.channel;
    expect(ch.length).toBeGreaterThan(0);
    // [1,2,3] — 3 байта. samples = floor(3/2) = 1 (неполный последний байт
    // отбрасывается). Этот один int16 = 0x0201 (LE) = 513.
    // 513/32768 ≈ 0.01565.
    expect(ch[0]).toBeCloseTo(513 / 32768, 4);
    // decodeAudioData НЕ звался (PCM-путь).
    expect(fake.ctx.decodeAudioData).not.toHaveBeenCalled();
  });
});

describe("operator audio sink — проигрывание и barge-in (ADR-0055 §barge-in)", () => {
  it("play склеивает чанки по порядку и запускает source", async () => {
    const { sink, fake } = sinkWithFake();
    sink.onMeta("op1", "audio/pcm", 0, 2, 16000);
    sink.onChunk(new Uint8Array([1, 2]));
    sink.onMeta("op1", "audio/pcm", 1, 2, 16000);
    sink.onChunk(new Uint8Array([3, 4, 5]));
    await sink.play("op1");
    // ADR-0078: createBuffer с правильным количеством семплов.
    const createBuffer = fake.ctx.createBuffer as ReturnType<typeof vi.fn>;
    expect(createBuffer).toHaveBeenCalled();
    // [1,2] + [3,4,5] = 5 байт. samples = floor(5/2) = 2.
    expect(createBuffer).toHaveBeenCalledWith(1, 2, 16000);
    // Source запущен и подключён.
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

  it("новый play ставит второй чанк в очередь (ADR-0078 §3.1)", async () => {
    const { sink, fake } = sinkWithFake();
    sink.onMeta("op1", "audio/pcm", 0, 1, 16000);
    sink.onChunk(new Uint8Array([1]));
    await sink.play("op1");
    // Первый source играет.
    expect(fake.sources[0].started).toBe(true);
    expect(fake.sources[0].stopped).toBe(false);
    // Второй play — в очередь, источник не останавливается и не стартует.
    sink.onMeta("op2", "audio/pcm", 0, 1, 16000);
    sink.onChunk(new Uint8Array([2]));
    await sink.play("op2");
    expect(fake.sources).toHaveLength(1);
    expect(fake.sources[0].stopped).toBe(false);
    expect(fake.sources[0].started).toBe(true);
    // queuedCount = 2 (текущий + 1 в очереди).
    expect(sink.queuedCount()).toBe(2);
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

// ── ADR-0078 / issue #2162 follow-up — ТАРС в шлем ────────────────────

interface FakeBuffer {
  numberOfChannels: number;
  length: number;
  sampleRate: number;
  /** Записанные Int16-значения по каналу 0 (нормализованные к [-1,1]). */
  channel: number[];
  copyChannel?: number;
  copyBuffer?: ArrayBufferLike;
  copyOffset?: number;
}

function makeFakeCtxWithPCM(opts?: { suspendFirst?: boolean }) {
  const decoded: ArrayBuffer[] = [];
  const buffers: FakeBuffer[] = [];
  const sources: FakeSource[] = [];
  /** Реальные source-обёртки (AudioBufferSourceNode-фейки) для вызова stop()/onended. */
  const sourceObjs: AudioBufferSourceNode[] = [];
  let suspended = !!opts?.suspendFirst;
  const ctx = {
    destination: {},
    state: suspended ? ("suspended" as const) : ("running" as const),
    decodeAudioData: vi.fn(async (ab: ArrayBuffer) => {
      decoded.push(ab);
      return { duration: 1 } as unknown as AudioBuffer;
    }),
    createBuffer: vi.fn(
      (channels: number, length: number, sampleRate: number): AudioBuffer => {
        const buf: FakeBuffer = {
          numberOfChannels: channels,
          length,
          sampleRate,
          channel: []
        };
        buffers.push(buf);
        return buf as unknown as AudioBuffer;
      }
    ),
    resume: vi.fn(async () => {
      suspended = false;
      (ctx as { state: AudioContextState }).state = "running";
    }),
    createBufferSource: vi.fn((): AudioBufferSourceNode => {
      const src: FakeSource = {
        buffer: null,
        connected: false,
        started: false,
        stopped: false
      };
      sources.push(src);
      const onended = { fn: null as null | (() => void) };
      const sourceObj: AudioBufferSourceNode = {
        set buffer(b: AudioBuffer | null) { src.buffer = b; },
        get buffer(): AudioBuffer | null { return src.buffer as AudioBuffer | null; },
        connect: () => { src.connected = true; },
        start: () => { src.started = true; },
        stop: () => {
          src.stopped = true;
          // имитируем WebAudio: при stop() onended срабатывает асинхронно.
          if (onended.fn) Promise.resolve().then(() => onended.fn?.());
        },
        disconnect: () => { src.connected = false; },
        set onended(fn: (() => void) | null) { onended.fn = fn; },
        get onended(): (() => void) | null { return onended.fn; }
      } as unknown as AudioBufferSourceNode;
      sourceObjs.push(sourceObj);
      return sourceObj;
    }),
    close: vi.fn(async () => undefined)
  };
  function wrapBuffers(): void {
    const original = ctx.createBuffer;
    (ctx as { createBuffer: typeof original }).createBuffer = vi.fn(
      (channels: number, length: number, sampleRate: number): AudioBuffer => {
        const buf = original(channels, length, sampleRate) as unknown as FakeBuffer;
        (buf as unknown as AudioBuffer).copyToChannel = vi.fn(
          (data: Float32Array, channel: number) => {
            const arr = Array.from(data);
            if (channel === 0) (buf as FakeBuffer).channel = arr;
          }
        );
        return buf as unknown as AudioBuffer;
      }
    );
  }
  return { decoded, buffers, sources, sourceObjs, ctx, wrapBuffers };
}

describe("operator audio sink — ADR-0078: PCM AudioBuffer из Int16", () => {
  it("audio/pcm с sample_rate собирает AudioBuffer руками (НЕ decodeAudioData)", async () => {
    const wrapped = makeFakeCtxWithPCM();
    wrapped.wrapBuffers();
    const sink2 = createOperatorAudioSink({
      AudioContextCtor: function () {
        return wrapped.ctx;
      } as unknown as new () => AudioContext
    });
    // 4 байта int16 LE = 2 семпла: 0x0001, 0x7FFF (max positive).
    sink2.onMeta("p1", "audio/pcm", 0, 1, 16000);
    sink2.onChunk(new Uint8Array([0x01, 0x00, 0xff, 0x7f]));
    await sink2.play("p1");
    expect(wrapped.buffers).toHaveLength(1);
    expect(wrapped.buffers[0]).toMatchObject({
      numberOfChannels: 1,
      length: 2,
      sampleRate: 16000
    });
    // Нормализация: 0x0001 / 32768 ≈ 0.0000305…, 0x7FFF / 32768 ≈ 0.9999694…
    const ch = wrapped.buffers[0].channel;
    expect(ch).toHaveLength(2);
    expect(ch[0]).toBeCloseTo(1 / 32768, 5);
    expect(ch[1]).toBeCloseTo(32767 / 32768, 4);
    // decodeAudioData НЕ звался.
    expect(wrapped.ctx.decodeAudioData).not.toHaveBeenCalled();
    // Source запущен и подключён.
    expect(wrapped.sources[0]).toMatchObject({ started: true, connected: true });
  });

  it("не PCM формат (audio/mpeg) — fallback на decodeAudioData", async () => {
    const wrapped = makeFakeCtxWithPCM();
    wrapped.wrapBuffers();
    const sink = createOperatorAudioSink({
      AudioContextCtor: function () {
        return wrapped.ctx;
      } as unknown as new () => AudioContext
    });
    sink.onMeta("m1", "audio/mpeg", 0, 1);
    sink.onChunk(new Uint8Array([0x49, 0x44, 0x33])); // псевдо-mp3 заголовок
    await sink.play("m1");
    // PCM-путь НЕ сработал.
    expect(wrapped.buffers).toHaveLength(0);
    // decodeAudioData вызван.
    expect(wrapped.ctx.decodeAudioData).toHaveBeenCalled();
    expect(wrapped.sources[0].started).toBe(true);
  });

  it("AudioContext suspended → resume() перед start()", async () => {
    const wrapped = makeFakeCtxWithPCM({ suspendFirst: true });
    wrapped.wrapBuffers();
    const sink = createOperatorAudioSink({
      AudioContextCtor: function () {
        return wrapped.ctx;
      } as unknown as new () => AudioContext
    });
    sink.onMeta("r1", "audio/pcm", 0, 1, 16000);
    sink.onChunk(new Uint8Array([0x00, 0x00, 0x00, 0x00]));
    await sink.play("r1");
    expect(wrapped.ctx.resume).toHaveBeenCalled();
    // После resume — start() вызван.
    expect(wrapped.sources[0].started).toBe(true);
  });
});

describe("operator audio sink — ADR-0078: queue mode (самодостаточные чанки)", () => {
  it("новый play() не играет сразу, если предыдущий source ещё играет", async () => {
    const wrapped = makeFakeCtxWithPCM();
    wrapped.wrapBuffers();
    const sink = createOperatorAudioSink({
      AudioContextCtor: function () {
        return wrapped.ctx;
      } as unknown as new () => AudioContext
    });
    sink.onMeta("q1", "audio/pcm", 0, 1, 16000);
    sink.onChunk(new Uint8Array([0x00, 0x00]));
    await sink.play("q1");
    // Второй чанк приходит, пока первый ещё играет.
    sink.onMeta("q2", "audio/pcm", 0, 1, 16000);
    sink.onChunk(new Uint8Array([0x10, 0x00]));
    await sink.play("q2");
    // Сразу стартует только q1.
    expect(wrapped.sources).toHaveLength(1);
    expect(wrapped.sources[0].started).toBe(true);
    expect(wrapped.sources[0].stopped).toBe(false);
    // q2 в очереди — буфер создан, но source ещё не создан.
    expect(wrapped.buffers).toHaveLength(2);
    // Имитируем конец первого source (WebAudio onended):
    // стоп на обёртке → асинхронно срабатывает onended → следующий из очереди.
    wrapped.sourceObjs[0].stop();
    await new Promise((r) => setTimeout(r, 10));
    expect(wrapped.sources).toHaveLength(2);
    expect(wrapped.sources[1].started).toBe(true);
  });

  it("stop() очищает и текущий source, и всю очередь", async () => {
    const wrapped = makeFakeCtxWithPCM();
    wrapped.wrapBuffers();
    const sink = createOperatorAudioSink({
      AudioContextCtor: function () {
        return wrapped.ctx;
      } as unknown as new () => AudioContext
    });
    sink.onMeta("s1", "audio/pcm", 0, 1, 16000);
    sink.onChunk(new Uint8Array([0x00, 0x00]));
    await sink.play("s1");
    // Второй в очереди.
    sink.onMeta("s2", "audio/pcm", 0, 1, 16000);
    sink.onChunk(new Uint8Array([0x10, 0x00]));
    await sink.play("s2");
    expect(wrapped.buffers).toHaveLength(2);
    expect(wrapped.sources).toHaveLength(1);

    sink.stop();
    // Текущий source остановлен.
    expect(wrapped.sources[0].stopped).toBe(true);
    // Очередь выброшена: даже после onended никакой новый source не появится.
    wrapped.sourceObjs[0].stop(); // имитация «источник доиграл до конца»
    await new Promise((r) => setTimeout(r, 10));
    expect(wrapped.sources).toHaveLength(1);
    // pending очищен, queuedCount() = 0.
    expect(sink.queuedCount()).toBe(0);
    expect(sink.bufferedBytes("s1")).toBe(0);
  });
});
