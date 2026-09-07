// src/input/voice_capture.ts
//
// Рация (voice passthrough): захват микрофона → int16 PCM 16 kHz mono.
// getUserMedia + AudioWorklet (real-time audio thread) → resample 48k→16k →
// чанки ~20 мс.
//
// Почему AudioWorklet, а не ScriptProcessorNode:
//   ScriptProcessorNode — deprecated и бежит В MAIN-THREAD. Раньше это
//   конкурировало с three.js render loop → дропы кадров в VR. AudioWorklet
//   исполняется в отдельном audio rendering thread (см. ADR-0051 §2.7 и
//   target-operator-agent-and-dialogue §7.2 "Технический долг").
//
// PTT-семантика: start() при зажатом grip, stop() при отпускании. Пока
// захват активен, чанки идут непрерывно (включая тишину-нули), поэтому
// sound_node не рвёт стрим своим 300мс watchdog'ом посреди разговора.

export const VOICE_SAMPLE_RATE = 16000;
export const VOICE_CHUNK_SAMPLES = 320; // 20 мс @ 16 kHz

export interface AudioWorkletLike {
  addModule(url: string): Promise<void>;
}

export interface AudioWorkletNodeLike {
  port: {
    postMessage(message: unknown, transfer?: Transferable[]): void;
    onmessage: ((ev: { data: unknown }) => void) | null;
    close?: () => void;
  };
  connect(dest: unknown): void;
  disconnect(): void;
}

export interface VoiceCaptureDeps {
  getUserMedia: (constraints: MediaStreamConstraints) => Promise<MediaStream>;
  AudioContextCtor: new () => AudioContext;
  /** URL модуля worklet'а (в проде — blob URL из WORKLET_SOURCE). */
  audioWorkletModuleUrl: string;
}

/**
 * Исходник AudioWorkletProcessor'а. Заворачивается в Blob URL на лету и
 * загружается через `audioWorklet.addModule()` — без отдельного bundler-шага.
 *
 * Контракт процессора: на вход 1 канал Float32 (любой sampleRate AudioContext),
 * на выход — port.postMessage({ type: "chunk", pcm: Float32Array }).
 */
export const WORKLET_SOURCE = `
class VoiceCaptureProcessor extends AudioWorkletProcessor {
  process(inputs) {
    const input = inputs[0];
    if (!input || input.length === 0) return true;
    const channel = input[0];
    // Копируем: AudioWorklet переиспользует входные буферы между вызовами.
    const out = new Float32Array(channel.length);
    out.set(channel);
    this.port.postMessage({ type: "chunk", pcm: out }, [out.buffer]);
    return true;
  }
}
registerProcessor("voice-capture-processor", VoiceCaptureProcessor);
`;

export interface VoiceCaptureOptions {
  onChunk: (pcm: Int16Array) => void;
  onError?: (err: Error) => void;
  /** Внедряемые зависимости — для unit-тестов без реального микрофона. */
  deps?: Partial<VoiceCaptureDeps>;
  /**
   * Фабрика AudioWorkletNode (тесты подменяют на fake). В проде не нужна —
   * берётся прямо с audioCtx.
   */
  createAudioWorkletNode?: (ctx: AudioContext, name: string) => AudioWorkletNodeLike;
}

export interface VoiceCapture {
  start(): Promise<void>;
  stop(): void;
  isCapturing(): boolean;
}

/** float [-1..1] → int16 [-32768..32767] (симметрично). */
export function floatToInt16(v: number): number {
  const clamped = Math.max(-1, Math.min(1, v));
  const scaled = Math.trunc(clamped * 32768);
  return Math.max(-32768, Math.min(32767, scaled));
}

/** Линейная интерполяция inputRate → outputRate + float→int16. */
export function resampleToInt16(input: Float32Array, inputRate: number, outputRate: number): Int16Array {
  const outLen = Math.floor((input.length * outputRate) / inputRate);
  const out = new Int16Array(outLen);
  if (outLen === 0 || input.length === 0) return out;
  const ratio = inputRate / outputRate;
  for (let i = 0; i < outLen; i++) {
    const pos = i * ratio;
    const idx = Math.floor(pos);
    const frac = pos - idx;
    const a = input[idx];
    const b = idx + 1 < input.length ? input[idx + 1] : a;
    out[i] = floatToInt16(a + (b - a) * frac);
  }
  return out;
}

export function createVoiceCapture(opts: VoiceCaptureOptions): VoiceCapture {
  // Дефолтный фабричный путь: берём AudioWorkletNode прямо с контекста.
  // Тесты подменяют через opts.createAudioWorkletNode.
  const defaultCreateNode = (ctx: AudioContext, name: string): AudioWorkletNodeLike =>
    new AudioWorkletNode(ctx, name) as unknown as AudioWorkletNodeLike;
  const createNode = opts.createAudioWorkletNode ?? defaultCreateNode;

  // Дефолтный URL — blob с inline-исходником worklet'а. В jsdom URL.createObjectURL
  // недоступен, поэтому берём дефолт лениво и даём тестам возможность
  // переопределить через opts.deps.audioWorkletModuleUrl.
  const defaultModuleUrl =
    typeof URL !== "undefined" && typeof URL.createObjectURL === "function"
      ? URL.createObjectURL(new Blob([WORKLET_SOURCE], { type: "application/javascript" }))
      : "";

  const deps: VoiceCaptureDeps = {
    getUserMedia: (constraints) => navigator.mediaDevices.getUserMedia(constraints),
    AudioContextCtor: (globalThis as unknown as { AudioContext: new () => AudioContext }).AudioContext,
    audioWorkletModuleUrl: defaultModuleUrl,
    ...opts.deps
  };

  let ctx: AudioContext | null = null;
  let stream: MediaStream | null = null;
  let source: MediaStreamAudioSourceNode | null = null;
  let workletNode: AudioWorkletNodeLike | null = null;
  let capturing = false;
  // Остаток после нарезки на VOICE_CHUNK_SAMPLES (int16 семплы).
  let pending = new Int16Array(0);

  function push(pcm: Int16Array): void {
    if (pcm.length === 0) return;
    const merged = new Int16Array(pending.length + pcm.length);
    merged.set(pending, 0);
    merged.set(pcm, pending.length);
    let off = 0;
    while (merged.length - off >= VOICE_CHUNK_SAMPLES) {
      opts.onChunk(merged.slice(off, off + VOICE_CHUNK_SAMPLES));
      off += VOICE_CHUNK_SAMPLES;
    }
    pending = merged.slice(off);
  }

  async function start(): Promise<void> {
    if (capturing) return;
    try {
      const s = await deps.getUserMedia({ audio: { echoCancellation: true, noiseSuppression: true } });
      const c = new deps.AudioContextCtor();
      const aw = (c as unknown as { audioWorklet: AudioWorkletLike }).audioWorklet;
      await aw.addModule(deps.audioWorkletModuleUrl);
      const src = c.createMediaStreamSource(s);
      const node = createNode(c, "voice-capture-processor");
      node.port.onmessage = (ev: { data: unknown }) => {
        const data = ev.data as { type?: string; pcm?: Float32Array };
        if (!data || data.type !== "chunk" || !(data.pcm instanceof Float32Array)) return;
        push(resampleToInt16(data.pcm, c.sampleRate, VOICE_SAMPLE_RATE));
      };
      src.connect(node as unknown as AudioNode);
      // Worklet-node НЕ подключаем к destination: иначе в VR-ушах звучит
      // собственный голос с микрофона (эхо-петля). ScriptProcessor раньше
      // подключался к destination только потому, что иначе onaudioprocess
      // не вызывался. У AudioWorklet такой зависимости нет.
      stream = s;
      ctx = c;
      source = src;
      workletNode = node;
      capturing = true;
    } catch (err) {
      opts.onError?.(err instanceof Error ? err : new Error(String(err)));
    }
  }

  function stop(): void {
    if (!capturing) return;
    capturing = false;
    if (workletNode) {
      workletNode.port.onmessage = null;
      workletNode.port.close?.();
    }
    try {
      workletNode?.disconnect();
    } catch {
      // ignore
    }
    try {
      source?.disconnect();
    } catch {
      // ignore
    }
    for (const track of stream?.getTracks() ?? []) {
      track.stop();
    }
    void ctx?.close();
    stream = null;
    ctx = null;
    source = null;
    workletNode = null;
    pending = new Int16Array(0);
  }

  return { start, stop, isCapturing: () => capturing };
}
