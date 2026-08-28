// voice_capture: захват микрофона → int16 PCM 16 kHz mono (рация).
// Проверяем: resample 48k→16k, нарезка на ~20мс чанки, освобождение
// getUserMedia-трека на stop().

import { describe, it, expect, vi } from "vitest";
import {
  createVoiceCapture,
  resampleToInt16,
  VOICE_CHUNK_SAMPLES,
  VOICE_SAMPLE_RATE
} from "../src/input/voice_capture";

interface FakeDeps {
  track: { stop: ReturnType<typeof vi.fn> };
  stream: MediaStream;
  getUserMedia: ReturnType<typeof vi.fn>;
  processor: {
    connect: ReturnType<typeof vi.fn>;
    disconnect: ReturnType<typeof vi.fn>;
    onaudioprocess: ((ev: { inputBuffer: { getChannelData(c: number): Float32Array } }) => void) | null;
  };
  audioCtx: {
    sampleRate: number;
    destination: unknown;
    createMediaStreamSource: ReturnType<typeof vi.fn>;
    createScriptProcessor: ReturnType<typeof vi.fn>;
    close: ReturnType<typeof vi.fn>;
  };
}

function makeFakeDeps(): FakeDeps {
  const track = { stop: vi.fn() };
  const stream = { getTracks: () => [track] } as unknown as MediaStream;
  const getUserMedia = vi.fn().mockResolvedValue(stream);
  const processor: FakeDeps["processor"] = {
    connect: vi.fn(),
    disconnect: vi.fn(),
    onaudioprocess: null
  };
  const audioCtx: FakeDeps["audioCtx"] = {
    sampleRate: 48000,
    destination: {},
    createMediaStreamSource: vi.fn().mockReturnValue({ connect: vi.fn(), disconnect: vi.fn() }),
    createScriptProcessor: vi.fn().mockReturnValue(processor),
    close: vi.fn().mockResolvedValue(undefined)
  };
  return { track, stream, getUserMedia, processor, audioCtx };
}

function makeCapture(
  deps: FakeDeps,
  onChunk: (c: Int16Array) => void,
  onError?: (e: Error) => void
): ReturnType<typeof createVoiceCapture> {
  // Обычная function (не стрелка): production-код делает `new deps.AudioContextCtor()`.
  function FakeCtor(): AudioContext {
    return deps.audioCtx as unknown as AudioContext;
  }
  return createVoiceCapture({
    onChunk,
    onError,
    deps: {
      getUserMedia: deps.getUserMedia,
      AudioContextCtor: FakeCtor as unknown as new () => AudioContext
    }
  });
}

function drive(processor: FakeDeps["processor"], sampleRate: number, seconds: number, value = 0.5): void {
  const n = Math.floor(sampleRate * seconds);
  const buf = { getChannelData: () => new Float32Array(n).fill(value) };
  if (!processor.onaudioprocess) throw new Error("onaudioprocess not wired");
  processor.onaudioprocess({ inputBuffer: buf });
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
    expect(cap.isCapturing()).toBe(true);

    // 1 секунда аудио @ 48k → 16000 samples → 50 чанков по 320.
    drive(deps.processor, 48000, 1);
    expect(chunks.length).toBe(50);
    expect(chunks[0]).toBeInstanceOf(Int16Array);
    expect(chunks[0].length).toBe(VOICE_CHUNK_SAMPLES);
    expect(chunks[0][0]).toBe(16384);

    cap.stop();
    expect(deps.track.stop).toHaveBeenCalled();
    expect(deps.audioCtx.close).toHaveBeenCalled();
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
  });

  it("keeps emitting silence chunks while capturing (no 300ms cut-off mid-hold)", async () => {
    const deps = makeFakeDeps();
    const chunks: Int16Array[] = [];
    const cap = makeCapture(deps, (c) => chunks.push(c));

    await cap.start();
    // Держим grip молча: чанки тишины (нули) продолжают идти.
    drive(deps.processor, 48000, 1, 0);
    expect(chunks.length).toBe(50);
    expect(chunks[0][0]).toBe(0);
    expect(chunks[0].length).toBe(VOICE_CHUNK_SAMPLES);
  });
});

// keep VOICE_SAMPLE_RATE referenced (documented contract for server side).
describe("constants", () => {
  it("VOICE_SAMPLE_RATE is 16000 and chunk is 20ms", () => {
    expect(VOICE_SAMPLE_RATE).toBe(16000);
    expect(VOICE_CHUNK_SAMPLES).toBe(320);
  });
});
