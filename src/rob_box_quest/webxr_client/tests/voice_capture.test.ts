// voice_capture: захват микрофона → int16 PCM 16 kHz mono (рация).
// Проверяем: resample 48k→16k, нарезка на ~20мс чанки, освобождение
// getUserMedia-трека на stop(), ошибки getUserMedia, тишина не рвёт стрим.
//
// После рефакторинга на AudioWorklet (issue #1991, шаг 05a-0) источник чанков
// — порт AudioWorkletNode. В jsdom настоящего AudioWorklet нет, поэтому
// подсовываем fake-фабрику и руками дёргаем port.onmessage — это эквивалент
// того, что worklet делает в audio-thread'е.

import { describe, it, expect, vi } from "vitest";
import {
  createVoiceCapture,
  resampleToInt16,
  VOICE_CHUNK_SAMPLES,
  VOICE_SAMPLE_RATE,
  type AudioWorkletNodeLike
} from "../src/input/voice_capture";

interface FakeDeps {
  track: { stop: ReturnType<typeof vi.fn> };
  stream: MediaStream;
  getUserMedia: ReturnType<typeof vi.fn>;
  audioWorklet: { addModule: ReturnType<typeof vi.fn> };
  audioCtx: {
    sampleRate: number;
    destination: unknown;
    audioWorklet: { addModule: ReturnType<typeof vi.fn> };
    createMediaStreamSource: ReturnType<typeof vi.fn>;
    close: ReturnType<typeof vi.fn>;
  };
  /** Фабрика fake-нод: создаёт ноду с портом, который тест может дёргать. */
  nodes: Array<{
    port: {
      onmessage: ((ev: { data: unknown }) => void) | null;
      postMessage: ReturnType<typeof vi.fn>;
      close: ReturnType<typeof vi.fn>;
    };
    connect: ReturnType<typeof vi.fn>;
    disconnect: ReturnType<typeof vi.fn>;
  }>;
}

function makeFakeDeps(): FakeDeps {
  const track = { stop: vi.fn() };
  const stream = { getTracks: () => [track] } as unknown as MediaStream;
  const getUserMedia = vi.fn().mockResolvedValue(stream);
  const audioWorklet = { addModule: vi.fn().mockResolvedValue(undefined) };
  const nodes: FakeDeps["nodes"] = [];
  const audioCtx: FakeDeps["audioCtx"] = {
    sampleRate: 48000,
    destination: {},
    // В jsdom AudioContext не имеет audioWorklet по умолчанию — подкладываем
    // fake, чтобы production-код мог дёрнуть .addModule().
    audioWorklet,
    createMediaStreamSource: vi.fn().mockReturnValue({ connect: vi.fn(), disconnect: vi.fn() }),
    close: vi.fn().mockResolvedValue(undefined)
  };
  return { track, stream, getUserMedia, audioWorklet, audioCtx, nodes };
}

function makeCapture(
  deps: FakeDeps,
  onChunk: (c: Int16Array) => void,
  onError?: (e: Error) => void
): ReturnType<typeof createVoiceCapture> {
  function FakeCtor(): AudioContext {
    return deps.audioCtx as unknown as AudioContext;
  }
  const createNode = (_ctx: AudioContext, _name: string): AudioWorkletNodeLike => {
    const port = {
      onmessage: null as ((ev: { data: unknown }) => void) | null,
      postMessage: vi.fn(),
      close: vi.fn()
    };
    const node = { port, connect: vi.fn(), disconnect: vi.fn() };
    deps.nodes.push(node);
    return node;
  };
  return createVoiceCapture({
    onChunk,
    onError,
    deps: {
      getUserMedia: deps.getUserMedia,
      AudioContextCtor: FakeCtor as unknown as new () => AudioContext,
      // Передаём любую непустую строку — fake addModule ничего с ней не делает.
      audioWorkletModuleUrl: "blob:fake-worklet-url"
    },
    createAudioWorkletNode: createNode
  });
}

/** Эмулирует один вызов AudioWorkletProcessor.process(): пушит Float32 в порт. */
function drive(node: FakeDeps["nodes"][number], sampleRate: number, seconds: number, value = 0.5): void {
  if (!node.port.onmessage) throw new Error("worklet port not wired");
  const n = Math.floor(sampleRate * seconds);
  const pcm = new Float32Array(n).fill(value);
  node.port.onmessage({ data: { type: "chunk", pcm } });
}

describe("resampleToInt16", () => {
  it("48k→16k keeps 3:1 length ratio and converts float→int16", () => {
    const out = resampleToInt16(new Float32Array(48000).fill(0.5), 48000, 16000);
    expect(out.length).toBe(16000);
    expect(out[0]).toBe(16384); // round(0.5 * 32767)
    expect(out[16000 - 1]).toBe(16384);
  });

  it("identity when inputRate == outputRate", () => {
    const input = new Float32Array([1, -0.5, 0, 2]);
    const out = resampleToInt16(input, 4, 4);
    expect(Array.from(out)).toEqual([32767, -16384, 0, 32767]); // 2.0 clamped to 1.0
  });
});

describe("createVoiceCapture", () => {
  it("emits int16 PCM chunks of 20ms and releases track on stop()", async () => {
    const deps = makeFakeDeps();
    const chunks: Int16Array[] = [];
    const cap = makeCapture(deps, (c) => chunks.push(c));

    await cap.start();

    expect(deps.getUserMedia).toHaveBeenCalledWith({
      audio: { echoCancellation: true, noiseSuppression: true }
    });
    expect(deps.audioWorklet.addModule).toHaveBeenCalledWith("blob:fake-worklet-url");
    expect(deps.nodes.length).toBe(1);
    expect(cap.isCapturing()).toBe(true);

    // 1 секунда аудио @ 48k → 16000 samples → 50 чанков по 320.
    drive(deps.nodes[0], 48000, 1);
    expect(chunks.length).toBe(50);
    expect(chunks[0]).toBeInstanceOf(Int16Array);
    expect(chunks[0].length).toBe(VOICE_CHUNK_SAMPLES);
    expect(chunks[0][0]).toBe(16384);

    // AudioWorklet-node НЕ подключаем к ctx.destination (эхо-петля в VR-ушах).
    // ScriptProcessor раньше подключали вынужденно — иначе onaudioprocess не
    // вызывался. У AudioWorklet такой зависимости нет.
    expect(deps.nodes[0].connect).not.toHaveBeenCalledWith(deps.audioCtx.destination);

    cap.stop();
    expect(deps.track.stop).toHaveBeenCalled();
    expect(deps.audioCtx.close).toHaveBeenCalled();
    // Порт отписан и закрыт, чтобы worklet не звал мёртвый onmessage после stop().
    expect(deps.nodes[0].port.onmessage).toBeNull();
    expect(deps.nodes[0].port.close).toHaveBeenCalled();
    expect(deps.nodes[0].disconnect).toHaveBeenCalled();
    expect(cap.isCapturing()).toBe(false);
  });

  it("stop() before start() is a safe no-op", () => {
    const deps = makeFakeDeps();
    const cap = makeCapture(deps, () => {});
    expect(() => cap.stop()).not.toThrow();
    expect(cap.isCapturing()).toBe(false);
    expect(deps.getUserMedia).not.toHaveBeenCalled();
  });

  it("reports getUserMedia failure via onError and does not capture", async () => {
    const deps = makeFakeDeps();
    deps.getUserMedia.mockRejectedValue(new Error("mic denied"));
    const onError = vi.fn();
    const cap = makeCapture(deps, () => {}, onError);

    await cap.start();

    expect(onError).toHaveBeenCalledWith(expect.any(Error));
    expect(cap.isCapturing()).toBe(false);
    // Worklet не успел подгрузиться — фабрика нод не вызвана.
    expect(deps.nodes.length).toBe(0);
  });

  it("keeps emitting silence chunks while capturing (no 300ms cut-off mid-hold)", async () => {
    const deps = makeFakeDeps();
    const chunks: Int16Array[] = [];
    const cap = makeCapture(deps, (c) => chunks.push(c));

    await cap.start();
    // Держим grip молча: чанки тишины (нули) продолжают идти.
    drive(deps.nodes[0], 48000, 1, 0);
    expect(chunks.length).toBe(50);
    expect(chunks[0][0]).toBe(0);
    expect(chunks[0].length).toBe(VOICE_CHUNK_SAMPLES);
  });

  it("ignores malformed worklet messages (defensive against worklet drift)", async () => {
    const deps = makeFakeDeps();
    const chunks: Int16Array[] = [];
    const cap = makeCapture(deps, (c) => chunks.push(c));

    await cap.start();
    const port = deps.nodes[0].port;
    if (!port.onmessage) throw new Error("port not wired");
    // Неправильный тип / нет pcm / pcm не Float32Array — всё игнорируем.
    port.onmessage({ data: { type: "ping" } });
    port.onmessage({ data: { type: "chunk" } });
    port.onmessage({ data: { type: "chunk", pcm: new Int16Array(10) } });
    port.onmessage({ data: null });
    expect(chunks.length).toBe(0);

    // После этого нормальный чанк всё ещё проходит.
    drive(deps.nodes[0], 48000, 0.02);
    expect(chunks.length).toBe(1);
  });
});

// keep VOICE_SAMPLE_RATE referenced (documented contract for server side).
describe("constants", () => {
  it("VOICE_SAMPLE_RATE is 16000 and chunk is 20ms", () => {
    expect(VOICE_SAMPLE_RATE).toBe(16000);
    expect(VOICE_CHUNK_SAMPLES).toBe(320);
  });
});

describe("WORKLET_SOURCE", () => {
  it("declares registerProcessor('voice-capture-processor') — worklet contract for audio-thread", async () => {
    const { WORKLET_SOURCE } = await import("../src/input/voice_capture");
    // Worklet-код НЕ выполняется в jsdom (vitest), но компилируется как
    // обычный JS. Если кто-то его сломает — addModule в браузере тихо упадёт
    // без понятного стектрейса. Поэтому хотя бы проверим синтаксис.
    expect(WORKLET_SOURCE).toMatch(/class\s+VoiceCaptureProcessor\s+extends\s+AudioWorkletProcessor/);
    expect(WORKLET_SOURCE).toMatch(/registerProcessor\(\s*["']voice-capture-processor["']/);
    // new Function компилирует тело без выполнения — SyntaxError поймает
    // опечатки вроде незакрытой скобки, которые проглядел бы grep.
    expect(() => {
      new Function(WORKLET_SOURCE);
    }).not.toThrow();
  });
});
