// Проигрыватель обратного аудио ТАРС в шлем оператора
// (ADR-0055 / issue #1993, шаг 5б; ADR-0078 / issue #2162 — автоплей).
//
// Контракт симметричен preview-каналу (см. preview_audio_sink.ts), но
// ADR-0078 меняет поведение для обратного канала:
//
//   Сервер шлёт ЧАНК ЗА ЧАНКОМ (operator_tts_audio). Каждый чанк
//   САМОДОСТАТОЧНЫЙ — играется СРАЗУ по приходу байт, ставится в
//   **последовательную очередь** (следующий стартует, когда предыдущий
//   source закончился — AudioBufferSourceNode.onended).
//
//   JSON_EVENT{type:"operator_tts_audio", request_id, format, content_type,
//              sample_rate?, seq, total, ts_ms}
//   BINARY_FRAME (stream_id = 0)
//   ... повтор для каждого чанка ...
//   operator_tts_done / operator_tts_error — НЕ публикуются в норме
//     (зарезервированы для forward-compat с приоритетной очередью 07a /
//      flush при supervised shutdown).
//
// BINARY_FRAME идёт со stream_id=0 (как и preview), и не имеет
// topic'а в subscribe_ack — main.ts зеркалит нулевой stream в оба
// sink'а (preview + operator). Каждый sink знает свой `currentRequestId`,
// поэтому «чужие» чанки просто отбрасываются (sink.onChunk вернёт false).
//
// Barge-in (локальный): `stop()` обрывает проигрывание **и всю очередь**.
// Зовётся в main.ts на `voice_ptt_start` ДО отправки команды на сервер
// (оператор должен услышать тишину раньше, чем сервер обработает STOP).
//
// Формат: pcm_s16le (int16 LE PCM, тот же SR, что /voice/audio/speech)
// собирается руками через AudioBuffer.createBuffer + copyToChannel; иначе
// (audio/mpeg / audio/ogg / audio/wav) — fallback на decodeAudioData.

export interface OperatorAudioDeps {
  /** Конструктор AudioContext — в тестах подменяется на фейк. */
  AudioContextCtor: new () => AudioContext;
}

export interface OperatorAudioSink {
  /** Пришла мета очередного чанка: запоминаем request_id/тип/частоту. */
  onMeta(
    requestId: string,
    contentType: string,
    seq: number,
    total: number,
    sampleRate?: number
  ): void;
  /** Пришли байты (BINARY_FRAME stream_id=0). `false` — мета не приходила. */
  onChunk(payload: Uint8Array): boolean;
  /** Запустить проигрывание накопленного request_id (или поставить в очередь). */
  play(requestId: string): Promise<void>;
  /** Barge-in: обрезать проигрывание + всю очередь. */
  stop(): void;
  /** Сервер сказал error: выбросить буфер и очередь, ничего не играть. */
  error(reason: string): void;
  /** Сколько байт накоплено для request_id (для тестов/диагностики). */
  bufferedBytes(requestId: string): number;
  /** Размер очереди (для тестов/диагностики). */
  queuedCount(): number;
  dispose(): void;
}

interface PendingOperator {
  contentType: string;
  sampleRate?: number;
  chunks: Uint8Array[];
  bytes: number;
}

/** Мусор не копим: одна реплика — максимум 8 МБ (60 c mp3 @ 128 kbps). */
export const OPERATOR_TTS_MAX_BYTES = 8 * 1024 * 1024;

/**
 * Делим PCM-Float32-данные, делим int16-байты на 32768.
 * WebAudio AudioBuffer.createBuffer принимает float32 в [-1, 1].
 * Для положительных: 0x7FFF → 32767/32768 ≈ 0.9999; 0x0001 → 1/32768 ≈ 0.00003.
 * Для отрицательных: 0x8000 → -32768/32768 = -1.0; 0xFFFF → -1/32768 ≈ -0.00003.
 */
function pcmS16LEToFloat32(bytes: Uint8Array): Float32Array {
  const samples = Math.floor(bytes.byteLength / 2);
  const view = new DataView(
    bytes.buffer,
    bytes.byteOffset,
    bytes.byteLength
  );
  // ArrayBuffer-backed Float32Array (не SharedArrayBuffer) — copyToChannel
  // требует именно ArrayBuffer в типе Float32Array<ArrayBuffer>.
  const ab = new ArrayBuffer(samples * 4);
  const out = new Float32Array(ab);
  for (let i = 0; i < samples; i++) {
    // Little-endian int16. Нормализация: s16 / 32768 → [-1, ~0.99997].
    out[i] = view.getInt16(i * 2, true) / 32768;
  }
  return out;
}

/** Содержимое contentType — PCM (raw int16)? */
function isPcmContentType(contentType: string): boolean {
  const lc = contentType.toLowerCase();
  return lc === "audio/pcm" || lc === "audio/pcm_s16le" || lc.startsWith("audio/pcm");
}

export function createOperatorAudioSink(
  deps?: Partial<OperatorAudioDeps>
): OperatorAudioSink {
  const AudioContextCtor =
    deps?.AudioContextCtor ??
    (globalThis as unknown as { AudioContext?: new () => AudioContext }).AudioContext;

  const pending = new Map<string, PendingOperator>();
  // request_id последней пришедшей меты — к нему относятся следующие байты.
  let currentRequestId: string | null = null;
  let ctx: AudioContext | null = null;
  // Текущий играющий source (если есть). Очередь — FIFO готовых AudioBuffer-ов
  // (предсобранных в play() и ждущих, пока предыдущий source.onended выстрелит).
  let currentSource: AudioBufferSourceNode | null = null;
  // Буферы, ожидающие своей очереди на запуск.
  const queue: AudioBuffer[] = [];

  function ensureCtx(): AudioContext | null {
    if (!AudioContextCtor) return null;
    if (!ctx) ctx = new AudioContextCtor();
    return ctx;
  }

  function onMeta(
    requestId: string,
    contentType: string,
    _seq: number,
    _total: number,
    sampleRate?: number
  ): void {
    currentRequestId = requestId;
    const existing = pending.get(requestId);
    if (existing) {
      // Уточняем content_type и sample_rate если пришли новые.
      existing.contentType = contentType || existing.contentType;
      if (sampleRate !== undefined) existing.sampleRate = sampleRate;
      return;
    }
    pending.set(requestId, {
      contentType,
      sampleRate,
      chunks: [],
      bytes: 0
    });
  }

  function onChunk(payload: Uint8Array): boolean {
    if (currentRequestId === null) return false;
    const entry = pending.get(currentRequestId);
    if (!entry) return false;
    if (entry.bytes + payload.byteLength > OPERATOR_TTS_MAX_BYTES) {
      // Честно роняем реплику, а не молча пишем половину: слишком
      // большой ответ — это баг сервера, и играть обрезок хуже, чем
      // не играть. Barge-in на это тоже не сработает — но sink.stop()
      // пользователь ещё успеет позвать.
      pending.delete(currentRequestId);
      if (currentRequestId !== null) currentRequestId = null;
      return false;
    }
    // Копируем: payload — вид на буфер сокета, он переиспользуется.
    entry.chunks.push(new Uint8Array(payload));
    entry.bytes += payload.byteLength;
    return true;
  }

  function concat(entry: PendingOperator): Uint8Array {
    const out = new Uint8Array(entry.bytes);
    let off = 0;
    for (const c of entry.chunks) {
      out.set(c, off);
      off += c.byteLength;
    }
    return out;
  }

  /** Один source отыграл — пускаем следующий из очереди. */
  function onCurrentSourceEnded(): void {
    currentSource = null;
    flushQueue();
  }

  function flushQueue(): void {
    if (currentSource !== null) return; // ещё играет
    const next = queue.shift();
    if (!next) return;
    startSource(next);
  }

  function startSource(buffer: AudioBuffer): void {
    const audioCtx = ensureCtx();
    if (!audioCtx) return;
    if (currentSource !== null) {
      // Защита: если кто-то забыл flushQueue, не плодим второй.
      try {
        currentSource.stop();
      } catch {
        // ignore
      }
    }
    const src = audioCtx.createBufferSource();
    src.buffer = buffer;
    src.connect(audioCtx.destination);
    // onended → следующий из очереди. Но! Если был stop()/error() —
    // мы хотим очистить всё. Используем флаг cancelled.
    src.onended = () => {
      // Если это «наш» source, а не заменённый — пускаем очередь.
      if (currentSource === src) onCurrentSourceEnded();
    };
    currentSource = src;
    void (async () => {
      try {
        // Quest — immersive VR, автоплей-политика режет аудио до
        // user gesture. AudioContext приходит suspended, start() тогда
        // ничего не сделает. Делаем resume() перед start().
        if (audioCtx.state === "suspended") {
          await audioCtx.resume();
        }
        src.start();
      } catch (err) {
        // eslint-disable-next-line no-console
        console.warn("[quest] operator_tts start failed:", (err as Error).message);
        if (currentSource === src) {
          currentSource = null;
          flushQueue();
        }
      }
    })();
  }

  /** Собрать AudioBuffer: PCM — руками, иначе — decodeAudioData. */
  async function buildAudioBuffer(
    entry: PendingOperator
  ): Promise<AudioBuffer | null> {
    const audioCtx = ensureCtx();
    if (!audioCtx) return null;
    const bytes = concat(entry);
    if (isPcmContentType(entry.contentType)) {
      // PCM int16 LE. sampleRate обязателен — если не пришёл, используем
      // 16000 как честный fallback (tts_node.audio_output_sample_rate
      // по умолчанию). Записываем в лог, чтобы отлавливать баги сервера.
      const sr =
        entry.sampleRate && entry.sampleRate > 0 ? entry.sampleRate : 16000;
      if (!entry.sampleRate || entry.sampleRate <= 0) {
        // eslint-disable-next-line no-console
        console.warn(
          "[quest] operator_tts PCM without sample_rate in meta — fallback 16000"
        );
      }
      const samples = Math.floor(bytes.byteLength / 2);
      const buf = audioCtx.createBuffer(1, samples, sr);
      const channel = pcmS16LEToFloat32(bytes);
      // copyToChannel ожидает Float32Array<ArrayBuffer>; channel уже
      // ArrayBuffer-backed (см. pcmS16LEToFloat32), но TS-тайпгвард
      // хочет явный каст (переменная `channel` пока что выводится как
      // Float32Array<ArrayBufferLike>, т.к. исходный bytes.buffer — общий).
      buf.copyToChannel(channel as Float32Array<ArrayBuffer>, 0);
      return buf;
    }
    // Не-PCM: decodeAudioData (mp3/opus/wav).
    const ab = bytes.buffer.slice(
      bytes.byteOffset,
      bytes.byteOffset + bytes.byteLength
    ) as ArrayBuffer;
    try {
      return await audioCtx.decodeAudioData(ab);
    } catch (err) {
      // eslint-disable-next-line no-console
      console.warn(
        "[quest] operator_tts decode failed:",
        (err as Error).message,
        entry.contentType
      );
      return null;
    }
  }

  async function play(requestId: string): Promise<void> {
    const entry = pending.get(requestId);
    pending.delete(requestId);
    if (currentRequestId === requestId) currentRequestId = null;
    if (!entry || entry.bytes === 0) return;
    const buf = await buildAudioBuffer(entry);
    if (!buf) return;
    queue.push(buf);
    flushQueue();
  }

  function stopSource(): void {
    if (!currentSource) return;
    const src = currentSource;
    currentSource = null;
    try {
      src.stop();
    } catch {
      // уже остановлен
    }
    try {
      src.disconnect();
    } catch {
      // ignore
    }
    // Не зовём onCurrentSourceEnded() — stop() хочет тишину, а не
    // следующий из очереди.
  }

  function stop(): void {
    stopSource();
    queue.length = 0;
    pending.clear();
    currentRequestId = null;
  }

  function error(reason: string): void {
    // eslint-disable-next-line no-console
    console.warn("[quest] operator_tts error:", reason);
    queue.length = 0;
    pending.clear();
    currentRequestId = null;
    stopSource();
  }

  function bufferedBytes(requestId: string): number {
    return pending.get(requestId)?.bytes ?? 0;
  }

  function queuedCount(): number {
    return queue.length + (currentSource ? 1 : 0);
  }

  function dispose(): void {
    stop();
    if (ctx) {
      void ctx.close();
      ctx = null;
    }
  }

  return {
    onMeta,
    onChunk,
    play,
    stop,
    error,
    bufferedBytes,
    queuedCount,
    dispose
  };
}
