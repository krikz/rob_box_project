// src/input/voice_capture.ts
//
// Рация (voice passthrough): захват микрофона → int16 PCM 16 kHz mono.
// getUserMedia + ScriptProcessorNode → resample 48k→16k → чанки ~20 мс.
//
// PTT-семантика: start() при зажатом grip, stop() при отпускании. Пока
// захват активен, чанки идут непрерывно (включая тишину-нули), поэтому
// sound_node не рвёт стрим своим 300мс watchdog'ом посреди разговора.

export const VOICE_SAMPLE_RATE = 16000;
export const VOICE_CHUNK_SAMPLES = 320; // 20 мс @ 16 kHz

// Буфер ScriptProcessorNode: 4096 @ 48 kHz ≈ 85 мс, после resample ≈ 1365 семплов.
const PROCESSOR_BUFFER = 4096;

export interface VoiceCaptureDeps {
  getUserMedia: (constraints: MediaStreamConstraints) => Promise<MediaStream>;
  AudioContextCtor: new () => AudioContext;
}

export interface VoiceCaptureOptions {
  onChunk: (pcm: Int16Array) => void;
  onError?: (err: Error) => void;
  /** Внедряемые зависимости — для unit-тестов без реального микрофона. */
  deps?: Partial<VoiceCaptureDeps>;
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
  const deps: VoiceCaptureDeps = {
    getUserMedia: (constraints) => navigator.mediaDevices.getUserMedia(constraints),
    AudioContextCtor: (globalThis as unknown as { AudioContext: new () => AudioContext }).AudioContext,
    ...opts.deps
  };

  let ctx: AudioContext | null = null;
  let stream: MediaStream | null = null;
  let source: MediaStreamAudioSourceNode | null = null;
  let processor: ScriptProcessorNode | null = null;
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
      const src = c.createMediaStreamSource(s);
      const proc = c.createScriptProcessor(PROCESSOR_BUFFER, 1, 1);
      proc.onaudioprocess = (ev) => {
        const input = ev.inputBuffer.getChannelData(0);
        push(resampleToInt16(input, c.sampleRate, VOICE_SAMPLE_RATE));
      };
      src.connect(proc);
      proc.connect(c.destination);
      stream = s;
      ctx = c;
      source = src;
      processor = proc;
      capturing = true;
    } catch (err) {
      opts.onError?.(err instanceof Error ? err : new Error(String(err)));
    }
  }

  function stop(): void {
    if (!capturing) return;
    capturing = false;
    try {
      processor?.disconnect();
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
    processor = null;
    pending = new Int16Array(0);
  }

  return { start, stop, isCapturing: () => capturing };
}
