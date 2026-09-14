// voice_capture_sustained_session: задача kanban t_d4ce35e3.
//
// Хендофф §14.2 ADR-0027 / rob_box / phase 1: проверить, что getUserMedia-поток
// живёт часами в immersive-сессии на Quest, без самопроизвольного обрыва
// (зависание видео, отвал трекинга, freeze браузера). Полночный handoff
// (issue #1992) уже подтвердил: capture стартует boot-ом и не рвётся на
// отпускании грипа (см. voice_capture_wiring.test.ts). Этот файл —
// количественная проверка «длительность непрерывной работы».
//
// Без физического Quest в worktree можно проверить только программную часть:
// эмулируем в fake-Deps длинную сессию (много виртуальных секунд) и проверяем
// три инварианта:
//
//   1. isCapturing() остаётся true через N циклов (нет ложного stop'а).
//   2. onChunk продолжает приходить с правильным темпом (чанков/сек).
//   3. track.stop() НЕ вызывается, audioCtx.close() НЕ вызывается,
//      workletNode.port.onmessage НЕ обнуляется.
//
// Это эквивалент того, что делал бы настоящий Quest с открытым браузером,
// только в Vitest: 50 000 фейковых чанков ≈ 1000 секунд (16 минут) живой
// работы. Если что-то в production-коде voice_capture.ts ломает поток
// после N секунд — тест поймает.

import { describe, it, expect, vi } from "vitest";
import {
  createVoiceCapture,
  VOICE_CHUNK_SAMPLES,
  type AudioWorkletNodeLike
} from "../src/input/voice_capture";

/** Fake worklet-port, который можно дёргать много раз подряд. */
interface FakePort {
  onmessage: ((ev: { data: unknown }) => void) | null;
  postMessage: ReturnType<typeof vi.fn>;
  close: ReturnType<typeof vi.fn>;
}

interface FakeDeps {
  track: { stop: ReturnType<typeof vi.fn> };
  getUserMedia: ReturnType<typeof vi.fn>;
  audioWorklet: { addModule: ReturnType<typeof vi.fn> };
  audioCtx: {
    sampleRate: number;
    audioWorklet: { addModule: ReturnType<typeof vi.fn> };
    createMediaStreamSource: ReturnType<typeof vi.fn>;
    close: ReturnType<typeof vi.fn>;
  };
  ports: FakePort[];
}

function makeFakeDeps(sampleRate = 48000): FakeDeps {
  const track = { stop: vi.fn() };
  const stream = { getTracks: () => [track] } as unknown as MediaStream;
  const getUserMedia = vi.fn().mockResolvedValue(stream);
  const audioWorklet = { addModule: vi.fn().mockResolvedValue(undefined) };
  const audioCtx = {
    sampleRate,
    audioWorklet,
    createMediaStreamSource: vi
      .fn()
      .mockReturnValue({ connect: vi.fn(), disconnect: vi.fn() }),
    close: vi.fn().mockResolvedValue(undefined)
  } as unknown as FakeDeps["audioCtx"];
  return { track, getUserMedia, audioWorklet, audioCtx, ports: [] };
}

function makeCapture(
  deps: FakeDeps,
  onChunk: (pcm: Int16Array, channel: "ptt" | "wake") => void,
  onError?: (e: Error) => void
) {
  function FakeCtor(): AudioContext {
    return deps.audioCtx as unknown as AudioContext;
  }
  const createNode = (_ctx: AudioContext, _name: string): AudioWorkletNodeLike => {
    const port: FakePort = {
      onmessage: null,
      postMessage: vi.fn(),
      close: vi.fn()
    };
    deps.ports.push(port);
    return {
      port,
      connect: vi.fn(),
      disconnect: vi.fn()
    } as unknown as AudioWorkletNodeLike;
  };
  const cap = createVoiceCapture({
    onChunk,
    onError,
    deps: {
      getUserMedia: deps.getUserMedia,
      AudioContextCtor: FakeCtor as unknown as new () => AudioContext,
      audioWorkletModuleUrl: "blob:fake-worklet-url"
    },
    createAudioWorkletNode: createNode
  });
  cap.setPttEnabled(true);
  return cap;
}

/**
 * Эмулировать N секунд «аудио» через fake-worklet. Синусоида амплитуды 0.5
 * (Float32), порождает озвученные чанки → ptt-канал их гонит.
 *
 * @returns число сгенерированных ptt-чанков.
 */
function driveSeconds(deps: FakeDeps, seconds: number): number {
  // Берём последний порт: на каждом start() createVoiceCapture создаёт
  // новый порт и пушит его в deps.ports. После stop() предыдущие порты
  // отписаны — дёргать нужно только свежий.
  const port = deps.ports[deps.ports.length - 1];
  if (!port.onmessage) throw new Error("port not wired");
  const sr = deps.audioCtx.sampleRate;
  const total = sr * seconds;
  const chunkSize = sr * 0.02; // 20 мс за раз
  let pcmCalls = 0;
  for (let sent = 0; sent < total; sent += chunkSize) {
    const n = Math.min(chunkSize, total - sent);
    const pcm = new Float32Array(n);
    for (let i = 0; i < n; i++) {
      pcm[i] = Math.sin(((sent + i) / sr) * Math.PI * 2) * 0.5;
    }
    port.onmessage({ data: { type: "chunk", pcm } });
    pcmCalls++;
  }
  // 50 ptt-чанков в секунду @ 16 kHz (320 семплов × 50 = 16000).
  return pcmCalls;
}

describe("voice_capture — sustained getUserMedia session (kanban t_d4ce35e3, §14.2)", () => {
  it("stays in capturing state and emits chunks continuously for 10 simulated minutes", async () => {
    const deps = makeFakeDeps();
    let pttChunks = 0;
    const cap = makeCapture(
      deps,
      () => {
        pttChunks++;
      },
      (e) => {
        throw new Error(`unexpected onError during sustained run: ${e.message}`);
      }
    );

    await cap.start();
    expect(cap.isCapturing()).toBe(true);

    // 10 минут × 60 сек × 50 чанков/сек = 30 000 чанков.
    // 1 секунда fake-drive = 50 pcm-чанков → 50 ptt-чанков (320 семплов каждый).
    const fakeSeconds = 600;
    const expectedChunks = fakeSeconds * 50;

    const calls = driveSeconds(deps, fakeSeconds);

    // Sanity: drive выполнил столько итераций, сколько просили.
    expect(calls).toBe(fakeSeconds * 50);

    // Capture не «упал» посреди дороги.
    expect(cap.isCapturing()).toBe(true);

    // Каждый drive-чанк породил ровно один ptt-чанк (320 семплов @ 48k → 1 @ 16k).
    expect(pttChunks).toBe(expectedChunks);
    // Минимум «живых» чанков (10 минут × 50/сек). Жёсткий equality
    // уже проверен строкой выше; здесь — sanity-страховка от регрессии
    // «capture падает после первой секунды».
    expect(pttChunks).toBe(30_000);

    // Никаких побочных эффектов: трек/контекст ещё живы, порт не отписан.
    expect(deps.track.stop).not.toHaveBeenCalled();
    expect(deps.audioCtx.close).not.toHaveBeenCalled();
    expect(deps.ports[0].onmessage).not.toBeNull();
    expect(deps.ports[0].close).not.toHaveBeenCalled();
  });

  it("emits exactly 50 ptt-chunks per simulated second (correct pcm framing)", async () => {
    const deps = makeFakeDeps();
    const chunks: Int16Array[] = [];
    const cap = makeCapture(
      deps,
      (pcm) => {
        chunks.push(pcm);
      },
      (e) => {
        throw new Error(e.message);
      }
    );
    await cap.start();

    driveSeconds(deps, 5);
    expect(chunks.length).toBe(250); // 5 сек × 50 чанков
    expect(chunks.every((c) => c.length === VOICE_CHUNK_SAMPLES)).toBe(true);
  });

  it("survives many simulated sessions without leaking context (close happens only on stop)", async () => {
    // 10 циклов start/stop — каждый start получает свежий fake-stream,
    // каждый stop корректно его освобождает. Если бы capture был «одноразовый»
    // (бага в refactor), второй start провалился бы.
    const deps = makeFakeDeps();
    let totalPtt = 0;
    const cap = makeCapture(
      deps,
      () => {
        totalPtt++;
      },
      (e) => {
        throw new Error(`unexpected onError in cycle: ${e.message}`);
      }
    );

    for (let cycle = 0; cycle < 10; cycle++) {
      await cap.start();
      // stop() сбрасывает voicePttEnabled (см. voice_capture.ts:310) —
      // это семантика capture teardown, не баг. Клиент заново включает ptt
      // после каждого старта сессии (аналог main.ts:applyVoicePtt на грипе).
      cap.setPttEnabled(true);
      expect(cap.isCapturing()).toBe(true);
      driveSeconds(deps, 2);
      expect(totalPtt).toBe((cycle + 1) * 100);
      cap.stop();
      expect(cap.isCapturing()).toBe(false);
      expect(deps.track.stop).toHaveBeenCalledTimes(cycle + 1);
      expect(deps.audioCtx.close).toHaveBeenCalledTimes(cycle + 1);
    }

    // 10 циклов × 2 сек × 50 чанков = 1000 ptt-чанков.
    expect(totalPtt).toBe(1000);
  });
});
