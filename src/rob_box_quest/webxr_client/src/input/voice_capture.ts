// src/input/voice_capture.ts
//
// Два канала с одного mic-захвата (ADR-0071, шаг 5а — wake stream):
//   - ptt: включается при зажатом грипе (applyVoicePtt в main.ts),
//          шлёт непрерывный поток int16 PCM 16 kHz mono в VOICE_AUDIO/stream_id=1.
//          Без VAD — sound_node не рвёт стрим своим 300мс watchdog'ом.
//   - wake: всегда-включённый поток (по умолчанию), шлёт VOICE_AUDIO/stream_id=2,
//          отфильтрованный RMS-VAD с hangover 200мс (тишина не идёт в WS).
//          Подавляется при зажатом грипе (одна фраза — один маршрут).
//          Включается/выключается через setWakeGate.
//
// getUserMedia + AudioWorklet (real-time audio thread) → resample 48k→16k →
// чанки ~20 мс.
//
// Почему AudioWorklet, а не ScriptProcessorNode:
//   ScriptProcessorNode — deprecated и бежит В MAIN-THREAD. Раньше это
//   конкурировало с three.js render loop → дропы кадров в VR. AudioWorklet
//   исполняется в отдельном audio rendering thread (см. ADR-0051 §2.7 и
//   target-operator-agent-and-dialogue §7.2 "Технический долг").

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
  onChunk: (pcm: Int16Array, channel: "ptt" | "wake") => void;
  onError?: (err: Error) => void;
  /** Внедряемые зависимости — для unit-тестов без реального микрофона. */
  deps?: Partial<VoiceCaptureDeps>;
  /**
   * Фабрика AudioWorkletNode (тесты подменяют на fake). В проде не нужна —
   * берётся прямо с audioCtx.
   */
  createAudioWorkletNode?: (ctx: AudioContext, name: string) => AudioWorkletNodeLike;
  /**
   * VAD-параметры для wake-канала (ADR-0071 §2.1).
   * rmsThreshold — int16 единицы (~ 0.0061 = 200 в диапазоне [-32768..32767]).
   * hangoverMs — после последнего голоса шлём ещё N мс (200 мс по умолчанию).
   * ptt-канал VAD НЕ гейтится — там нужен непрерывный поток для sound_node.
   */
  vad?: { rmsThreshold?: number; hangoverMs?: number };
}

export interface VoiceCapture {
  start(): Promise<void>;
  stop(): void;
  isCapturing(): boolean;
  /**
   * ADR-0071 §2.2 + §2.3: управление wake-каналом.
   *   enabled=true   → wake-канал активен (после VAD-gate).
   *   suppressed=true → wake подавлен при зажатом грипе (gate до VAD).
   * Вызывается из main.ts: HELLO-дефолт → {enabled:true, suppressed:false};
   * panel toggle → {enabled:false/true}; applyVoicePtt → {suppressed:radio||robot}.
   * Idempotent — повторный вызов с тем же значением ничего не делает.
   */
  setWakeGate(opts: { enabled?: boolean; suppressed?: boolean }): void;
  /**
   * Включение/выключение ptt-канала (грип зажат). Вызывается из
   * applyVoicePtt. ptt идёт без VAD — непрерывный поток для sound_node.
   */
  setPttEnabled(on: boolean): void;
}

/** float [-1..1] → int16 [-32768..32767] (симметрично). */
export function floatToInt16(v: number): number {
  const clamped = Math.max(-1, Math.min(1, v));
  const scaled = Math.trunc(clamped * 32768);
  return Math.max(-32768, Math.min(32767, scaled));
}

/**
 * RMS int16 PCM (ADR-0071 §2.1).
 *   r = sqrt( sum(pcm[i]^2) / N )
 * Pure function (exported для unit-тестов). Возвращает 0..32767.
 * Синус амплитуды A даёт RMS ≈ A / sqrt(2) — поэтому порог 200 ≈ 0.0061 от
 * максимума, что отсеивает микрофонный шум и оставляет голос.
 */
export function rmsInt16(pcm: Int16Array): number {
  if (pcm.length === 0) return 0;
  let sumSq = 0;
  for (let i = 0; i < pcm.length; i++) {
    const v = pcm[i];
    sumSq += v * v;
  }
  return Math.round(Math.sqrt(sumSq / pcm.length));
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

  // ── VAD-параметры (ADR-0071 §2.1) ────────────────────────────────────────
  // rmsThreshold — стартовая точка (200 инт16 единиц ≈ 0.0061); на замере
  // шлема подбирается. hangoverMs 200мс — компромисс «не рвать слоги».
  // ptt-канал гейтится по VOICE_PTT_ENABLED (при грипе).
  const VAD_RMS_THRESHOLD_DEFAULT = 200;
  const VAD_HANGOVER_MS_DEFAULT = 200;
  const rmsThreshold = opts.vad?.rmsThreshold ?? VAD_RMS_THRESHOLD_DEFAULT;
  const hangoverSamplesTotal = Math.max(
    0,
    ((opts.vad?.hangoverMs ?? VAD_HANGOVER_MS_DEFAULT) * VOICE_SAMPLE_RATE) / 1000
  );

  let ctx: AudioContext | null = null;
  let stream: MediaStream | null = null;
  let source: MediaStreamAudioSourceNode | null = null;
  let workletNode: AudioWorkletNodeLike | null = null;
  let capturing = false;
  // Остаток после нарезки на VOICE_CHUNK_SAMPLES (int16 семплы).
  let pending = new Int16Array(0);

  // ── Gate state (ADR-0071 §2.2, §2.3) ─────────────────────────────────────
  // enabled   — wake-канал разрешён (panel / HELLO-дефолт).
  // suppressed — wake подавлен при зажатом грипе (gate ДО VAD).
  // voicePttEnabled — грип зажат (ptt-канал активен). Не путать с gate.enabled!
  let wakeGate: { enabled: boolean; suppressed: boolean } = { enabled: false, suppressed: false };
  let voicePttEnabled = false;
  // Сколько ещё семплов шлём wake после последнего голоса (hangover).
  let hangoverSamplesLeft = 0;

  /**
   * Нарезать накопленный PCM на чанки и разослать по каналам:
   *   - ptt: при voicePttEnabled — всегда (без VAD).
   *   - wake: при wakeGate.enabled && !wakeGate.suppressed && speechActive —
   *          после RMS-VAD с hangover.
   *
   * `speechActive` = true если RMS чанка ≥ порога ИЛИ hangover ещё не
   * истёк. После тишины hangoverSamplesLeft убывает; когда обнулится —
   * wake больше не идёт, пока не появится новый голос.
   */
  function push(pcm: Int16Array): void {
    if (pcm.length === 0) return;
    const merged = new Int16Array(pending.length + pcm.length);
    merged.set(pending, 0);
    merged.set(pcm, pending.length);
    let off = 0;
    while (merged.length - off >= VOICE_CHUNK_SAMPLES) {
      const chunk = merged.slice(off, off + VOICE_CHUNK_SAMPLES);
      // RMS считаем один раз для обоих каналов (дёшево — O(N) на 320 семплов).
      const rms = rmsInt16(chunk);
      const isSpeech = rms >= rmsThreshold;
      if (isSpeech) {
        hangoverSamplesLeft = hangoverSamplesTotal;
      } else if (hangoverSamplesLeft > 0) {
        hangoverSamplesLeft = Math.max(0, hangoverSamplesLeft - chunk.length);
      }
      const speechActive = isSpeech || hangoverSamplesLeft > 0;
      // ptt-канал: шлём непрерывно пока грип зажат (даже тишину — иначе
      // sound_node watchdog'у покажется «клиент отвалился»).
      if (voicePttEnabled) {
        opts.onChunk(chunk, "ptt");
      }
      // wake-канал: VAD-гейт + panel toggle + grip suppression.
      if (
        wakeGate.enabled &&
        !wakeGate.suppressed &&
        speechActive
      ) {
        opts.onChunk(chunk, "wake");
      }
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
    hangoverSamplesLeft = 0;
    voicePttEnabled = false;
    // wakeGate оставляем как был — panel/HELLO-стейт, не сбрасывается на
    // stop() (он сбрасывается только явным setWakeGate или shutdown).
  }

  /**
   * ADR-0071 §2.2/§2.3. Idempotent — повторный вызов с теми же значениями
   * ничего не делает. Поддерживает частичные апдейты (только enabled / только
   * suppressed). Дёргать можно до start() (gate просто запоминается) и
   * после — применяется к следующим чанкам.
   *
   * Сброс hangover на enable=true: чтобы не слать «хвост» прошлой фразы
   * сразу после включения. В UI это незаметно (VAD наберёт 200мс тишины
   * сам), но экономим 200мс ложного wake-трафика.
   */
  function setWakeGate(patch: { enabled?: boolean; suppressed?: boolean }): void {
    const next = {
      enabled: patch.enabled !== undefined ? patch.enabled : wakeGate.enabled,
      suppressed: patch.suppressed !== undefined ? patch.suppressed : wakeGate.suppressed
    };
    if (next.enabled === wakeGate.enabled && next.suppressed === wakeGate.suppressed) return;
    wakeGate = next;
    if (wakeGate.enabled && !wakeGate.suppressed) {
      hangoverSamplesLeft = 0;
    }
  }

  /**
   * Внутренний вызов из main.ts:applyVoicePtt — включает/выключает ptt-канал.
   * Не часть публичного API (для тестов — тонкая обвязка через setWakeGate
   * для grip suppression, ptt-включение делает сам main.ts).
   */
  function setPttEnabled(on: boolean): void {
    voicePttEnabled = on;
  }

  return { start, stop, isCapturing: () => capturing, setWakeGate, setPttEnabled };
}
