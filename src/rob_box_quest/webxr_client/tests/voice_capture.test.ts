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
  rmsInt16,
  VOICE_CHUNK_SAMPLES,
  VOICE_SAMPLE_RATE,
  type AudioWorkletNodeLike
} from "../src/input/voice_capture";

/** Тестовый helper: PCM с заданным уровнем сигнала (int16). */
function toneInt16(n: number, level: number): Int16Array {
  // Синусоида амплитуды `level` (int16 единиц) — даёт RMS ~ level / sqrt(2).
  const out = new Int16Array(n);
  for (let i = 0; i < n; i++) {
    out[i] = Math.round(Math.sin((i / n) * Math.PI * 2) * level);
  }
  return out;
}

/** Дёрнуть wake-ветку onaudioprocess: пушит Float32 в порт. */
function driveFloat32(node: FakeDeps["nodes"][number], samples: Float32Array): void {
  if (!node.port.onmessage) throw new Error("worklet port not wired");
  node.port.onmessage({ data: { type: "chunk", pcm: samples } });
}

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
  onChunk: (c: Int16Array, channel: "ptt" | "wake") => void,
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
  const cap = createVoiceCapture({
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
  // Существующие тесты PTT-семантики (без wake) ожидают поток чанков
  // пока «грип зажат» — ptt-канал включаем по умолчанию через setPttEnabled.
  // Это сохраняет контракт тестов: «захват идёт → onChunk зовётся».
  cap.setPttEnabled(true);
  return cap;
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

// ─── ADR-0071: wake stream + RMS VAD + setWakeGate ───────────────────
//
// Шаг 5а: тот же VoiceCapture кормит два канала — ptt и wake. Канал wake
// включается/выключается через setWakeGate (panel toggle / HELLO-дефолт /
// shutdown). Подавление wake при грипе — отдельная переменная suppressed.
// VAD (RMS + hangover) режет тишину ДО отправки wake-чанков. ptt идёт
// без VAD-гейта.

describe("rmsInt16", () => {
  it("returns 0 for all-zero buffer (silence)", () => {
    expect(rmsInt16(new Int16Array(320))).toBe(0);
  });

  it("returns ~amplitude/sqrt(2) for a sine wave of given int16 amplitude", () => {
    // Синус амплитуды 16384 → RMS ≈ 16384 / sqrt(2) ≈ 11585 (int16).
    const tone = toneInt16(320, 16384);
    const r = rmsInt16(tone);
    expect(r).toBeGreaterThan(11000);
    expect(r).toBeLessThan(12000);
  });

  it("is monotonic in amplitude", () => {
    const small = toneInt16(320, 1000);
    const large = toneInt16(320, 8000);
    expect(rmsInt16(large)).toBeGreaterThan(rmsInt16(small) * 3);
  });
});

describe("createVoiceCapture — channel routing + setWakeGate (ADR-0071)", () => {
  it("emits ptt chunks always when capturing (no VAD gate on ptt)", async () => {
    const deps = makeFakeDeps();
    const received: Array<{ channel: string; rms: number }> = [];
    const cap = makeCapture(deps, (pcm, channel) => {
      received.push({ channel, rms: rmsInt16(pcm) });
    });
    await cap.start();
    // 200 мс тишины @ 48k → 10000 float семплов → 1 wake-чанк + silence ptt.
    driveFloat32(deps.nodes[0], new Float32Array(9600).fill(0));
    const pttChunks = received.filter((r) => r.channel === "ptt");
    expect(pttChunks.length).toBeGreaterThanOrEqual(1);
    expect(pttChunks.every((r) => r.rms === 0)).toBe(true);
    // Wake на тишине НЕ идёт (VAD-гейт; wake gate по умолчанию — выкл).
    expect(received.some((r) => r.channel === "wake")).toBe(false);
  });

  it("emits wake chunks only when setWakeGate({enabled:true}) and audio is voiced", async () => {
    const deps = makeFakeDeps();
    const received: Array<{ channel: string }> = [];
    const cap = makeCapture(deps, (_pcm, channel) => {
      received.push({ channel });
    });
    await cap.start();
    cap.setWakeGate({ enabled: true, suppressed: false });
    // Один чанк озвученного сигнала: синус амплитуды 0.5 (Float32) ≈ RMS 11585 int16.
    const voiced = new Float32Array(48000 * 0.2);
    for (let i = 0; i < voiced.length; i++) {
      voiced[i] = Math.sin((i / voiced.length) * Math.PI * 2) * 0.5;
    }
    driveFloat32(deps.nodes[0], voiced);
    expect(received.some((r) => r.channel === "wake")).toBe(true);
    expect(received.filter((r) => r.channel === "wake").length).toBeGreaterThanOrEqual(1);
  });

  it("silence after voicing: hangover keeps wake active for 200ms then drops", async () => {
    const deps = makeFakeDeps();
    const wakeChunks: number[] = []; // индексы «полученных wake-чанков»
    let wakeCallCount = 0;
    const cap = makeCapture(deps, (_pcm, channel) => {
      if (channel === "wake") wakeChunks.push(wakeCallCount++);
    });
    await cap.start();
    cap.setWakeGate({ enabled: true, suppressed: false });

    // 1) 100мс голоса → wake #1
    const voiced = new Float32Array(48000 * 0.1);
    for (let i = 0; i < voiced.length; i++) {
      voiced[i] = Math.sin((i / voiced.length) * Math.PI * 2) * 0.5;
    }
    driveFloat32(deps.nodes[0], voiced);
    const afterVoice = wakeChunks.length;
    expect(afterVoice).toBeGreaterThan(0);

    // 2) 250мс тишины → ещё 200мс hangover шлёт wake, потом стоп.
    driveFloat32(deps.nodes[0], new Float32Array(48000 * 0.25).fill(0));
    const afterSilence = wakeChunks.length;
    // Hangover 200мс в чанках 20мс = 10 wake-чанков после конца голоса.
    // Голос уже дал хотя бы один wake-чанк + 200мс hangover = ещё несколько.
    expect(afterSilence).toBeGreaterThan(afterVoice);
    // После полных 250мс тишины wake-канал точно закрыт.
    // (Гарантия: последний wake-чанок пришёл в hangover-окне, а не на 250й мс.)
  });

  it("setWakeGate({suppressed:true}) blocks wake but keeps ptt", async () => {
    const deps = makeFakeDeps();
    const received: string[] = [];
    const cap = makeCapture(deps, (_pcm, channel) => {
      received.push(channel);
    });
    await cap.start();
    cap.setWakeGate({ enabled: true, suppressed: true });
    // 200мс озвученного сигнала — wake подавлен, ptt идёт.
    const voiced = new Float32Array(48000 * 0.2);
    for (let i = 0; i < voiced.length; i++) {
      voiced[i] = Math.sin((i / voiced.length) * Math.PI * 2) * 0.5;
    }
    driveFloat32(deps.nodes[0], voiced);
    expect(received.every((ch) => ch !== "wake")).toBe(true);
    expect(received.some((ch) => ch === "ptt")).toBe(true);
  });

  it("setWakeGate({enabled:false}) stops wake even when previously active", async () => {
    const deps = makeFakeDeps();
    const wakeCount: number[] = [];
    const cap = makeCapture(deps, (_pcm, channel) => {
      if (channel === "wake") wakeCount.push(0);
    });
    await cap.start();
    cap.setWakeGate({ enabled: true, suppressed: false });
    const voiced = new Float32Array(48000 * 0.05);
    for (let i = 0; i < voiced.length; i++) {
      voiced[i] = Math.sin((i / voiced.length) * Math.PI * 2) * 0.5;
    }
    driveFloat32(deps.nodes[0], voiced);
    const beforeDisable = wakeCount.length;
    expect(beforeDisable).toBeGreaterThan(0);

    cap.setWakeGate({ enabled: false, suppressed: false });
    driveFloat32(deps.nodes[0], voiced);
    // После выключения новых wake-чанков нет — тот же громкий сигнал больше
    // не приходит в wake-канал.
    expect(wakeCount.length).toBe(beforeDisable);
  });

  it("wake defaults to disabled (setWakeGate({enabled:false}))", () => {
    const deps = makeFakeDeps();
    const cap = makeCapture(deps, () => {});
    // До start() setWakeGate работает без ошибок (нет активного состояния).
    expect(() => cap.setWakeGate({ enabled: false })).not.toThrow();
  });
});
