// Проигрыватель обратного аудио ТАРС в шлем оператора
// (ADR-0055 / issue #1993, шаг 5б).
//
// Контракт симметричен preview-каналу (см. preview_audio_sink.ts), но
// это **речь в шлем** — никакого UI-pipeline (`dispatchTts`, picker,
// preview-чанков) здесь нет. Сервер шлёт:
//
//   JSON_EVENT{type:"operator_tts_audio", request_id, format, content_type, seq, total}
//   BINARY_FRAME (stream_id = 0)
//   ... повтор для seq = 1..total-1 ...
//   (done/error пока не публикуются сервером — см. ADR-0055 §ws_server,
//    но client-типы заведены для forward-compat с приоритетной очередью
//    07a.)
//
// BINARY_FRAME идёт со stream_id=0 (как и preview), и не имеет
// topic'а в subscribe_ack — main.ts зеркалит нулевой stream в оба
// sink'а (preview + operator). Каждый sink знает свой `currentRequestId`,
// поэтому «чужие» чанки просто отбрасываются (sink.onChunk вернёт false).
//
// Barge-in (локальный): `stop()` обрывает проигрывание и сбрасывает
// буфер — зовётся в main.ts на `voice_ptt_start` ДО отправки команды
// на сервер (оператор должен услышать тишину раньше, чем сервер
// обработает STOP).
//
// Формат: pcm_s16le — самый частый (ADR-0055 §tts_node), но mp3/opus/wav
// тоже валидны (через decodeAudioData).

export interface OperatorAudioDeps {
  /** Конструктор AudioContext — в тестах подменяется на фейк. */
  AudioContextCtor: new () => AudioContext;
}

export interface OperatorAudioSink {
  /** Пришла мета очередного чанка: запоминаем request_id/тип для байтов. */
  onMeta(requestId: string, contentType: string, seq: number, total: number): void;
  /** Пришли байты (BINARY_FRAME stream_id=0). `false` — мета не приходила. */
  onChunk(payload: Uint8Array): boolean;
  /** Сервер сказал done: склеиваем и играем. Возвращает promise проигрывания. */
  play(requestId: string): Promise<void>;
  /** Barge-in: обрезать проигрывание + буфер. */
  stop(): void;
  /** Сервер сказал error: выбросить буфер, ничего не играть. */
  error(reason: string): void;
  /** Сколько байт накоплено для request_id (для тестов/диагностики). */
  bufferedBytes(requestId: string): number;
  dispose(): void;
}

interface PendingOperator {
  contentType: string;
  chunks: Uint8Array[];
  bytes: number;
}

/** Мусор не копим: одна реплика — максимум 8 МБ (60 c mp3 @ 128 kbps). */
export const OPERATOR_TTS_MAX_BYTES = 8 * 1024 * 1024;

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
  let source: AudioBufferSourceNode | null = null;

  function ensureCtx(): AudioContext | null {
    if (!AudioContextCtor) return null;
    if (!ctx) ctx = new AudioContextCtor();
    return ctx;
  }

  function onMeta(
    requestId: string,
    contentType: string,
    _seq: number,
    _total: number
  ): void {
    currentRequestId = requestId;
    const existing = pending.get(requestId);
    if (existing) {
      existing.contentType = contentType || existing.contentType;
      return;
    }
    pending.set(requestId, { contentType, chunks: [], bytes: 0 });
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

  async function play(requestId: string): Promise<void> {
    const entry = pending.get(requestId);
    pending.delete(requestId);
    if (currentRequestId === requestId) currentRequestId = null;
    if (!entry || entry.bytes === 0) return;
    const audioCtx = ensureCtx();
    if (!audioCtx) return;
    const bytes = concat(entry);
    // Копия в свой ArrayBuffer: decodeAudioData забирает буфер себе (detach).
    const ab = bytes.buffer.slice(
      bytes.byteOffset,
      bytes.byteOffset + bytes.byteLength
    ) as ArrayBuffer;
    let buffer: AudioBuffer;
    try {
      buffer = await audioCtx.decodeAudioData(ab);
    } catch (err) {
      // eslint-disable-next-line no-console
      console.warn(
        "[quest] operator_tts decode failed:",
        (err as Error).message,
        entry.contentType
      );
      return;
    }
    stopSource();
    const src = audioCtx.createBufferSource();
    src.buffer = buffer;
    src.connect(audioCtx.destination);
    src.start();
    source = src;
  }

  function stopSource(): void {
    if (!source) return;
    try {
      source.stop();
    } catch {
      // уже остановлен
    }
    try {
      source.disconnect();
    } catch {
      // ignore
    }
    source = null;
  }

  function stop(): void {
    stopSource();
    if (currentRequestId !== null) pending.delete(currentRequestId);
    currentRequestId = null;
  }

  function error(reason: string): void {
    // eslint-disable-next-line no-console
    console.warn("[quest] operator_tts error:", reason);
    if (currentRequestId !== null) pending.delete(currentRequestId);
    currentRequestId = null;
    stopSource();
  }

  function bufferedBytes(requestId: string): number {
    return pending.get(requestId)?.bytes ?? 0;
  }

  function dispose(): void {
    stopSource();
    pending.clear();
    currentRequestId = null;
    if (ctx) {
      void ctx.close();
      ctx = null;
    }
  }

  return { onMeta, onChunk, play, stop, error, bufferedBytes, dispose };
}
