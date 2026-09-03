// Проигрыватель preview-аудио TTS picker'а (AV-27 / issue #1919).
//
// Как приходит аудио (meta-quest-api.md §4.2 + §5, ws_server.deliver_preview_audio):
//   JSON_EVENT{type:"preview_voice_audio", request_id, format, content_type, seq, total}
//   BINARY_FRAME (stream_id = 0, payload = сырые байты этого чанка)
//   …повтор для seq = 1..total-1…
//   JSON_EVENT{type:"preview_voice_done"}  ← можно играть
// или JSON_EVENT{type:"preview_voice_error", reason}
//
// Ключевой момент: BINARY_FRAME preview'а идёт со `stream_id = 0`, у него нет
// topic'а в subscribe_ack — поэтому `main.ts` роутит нулевой stream сюда,
// а не в видео-панели. Порядок «мета → байты» сервер гарантирует (обе
// отправки планируются в один loop подряд), поэтому чанк подписывается
// последней пришедшей метой.
//
// Почему decodeAudioData, а не <audio src=blob:>: в immersive-vr сессии
// WebAudio — единственный надёжный путь (Quest браузер режет автоплей
// media-элементов вне user-activation, а WebAudio-контекст уже разбужен
// микрофонным захватом рации). Формат берём из `content_type`: mp3/opus/wav
// декодирует сам браузер.

export interface PreviewAudioDeps {
  /** Конструктор AudioContext — в тестах подменяется на фейк. */
  AudioContextCtor: new () => AudioContext;
}

export interface PreviewAudioSink {
  /** Пришла мета очередного чанка: запоминаем request_id/тип для байтов. */
  onMeta(requestId: string, contentType: string, seq: number, total: number): void;
  /** Пришли байты (BINARY_FRAME stream_id=0). `false` — мета не приходила. */
  onChunk(payload: Uint8Array): boolean;
  /** Сервер сказал done: склеиваем и играем. Возвращает promise проигрывания. */
  play(requestId: string): Promise<void>;
  /** Остановить проигрывание и выбросить накопленные чанки request_id. */
  stop(): void;
  /** Сколько байт накоплено для request_id (для тестов/диагностики). */
  bufferedBytes(requestId: string): number;
  dispose(): void;
}

interface PendingPreview {
  contentType: string;
  chunks: Uint8Array[];
  bytes: number;
}

/** Мусор не копим: один preview — максимум 4 МБ (30 c mp3 @ 128 kbps ≈ 480 КБ). */
export const PREVIEW_MAX_BYTES = 4 * 1024 * 1024;

export function createPreviewAudioSink(deps?: Partial<PreviewAudioDeps>): PreviewAudioSink {
  const AudioContextCtor =
    deps?.AudioContextCtor ??
    (globalThis as unknown as { AudioContext?: new () => AudioContext }).AudioContext;

  const pending = new Map<string, PendingPreview>();
  // request_id последней пришедшей меты — к нему относятся следующие байты.
  let currentRequestId: string | null = null;
  let ctx: AudioContext | null = null;
  let source: AudioBufferSourceNode | null = null;

  function ensureCtx(): AudioContext | null {
    if (!AudioContextCtor) return null;
    if (!ctx) ctx = new AudioContextCtor();
    return ctx;
  }

  function onMeta(requestId: string, contentType: string, _seq: number, _total: number): void {
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
    if (entry.bytes + payload.byteLength > PREVIEW_MAX_BYTES) {
      // Честно роняем preview, а не молча пишем половину: слишком большой
      // ответ — это баг сервера, и играть обрезок хуже, чем не играть.
      pending.delete(currentRequestId);
      currentRequestId = null;
      return false;
    }
    // Копируем: payload — вид на буфер сокета, он переиспользуется.
    entry.chunks.push(new Uint8Array(payload));
    entry.bytes += payload.byteLength;
    return true;
  }

  function concat(entry: PendingPreview): Uint8Array {
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
    const ab = bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength) as ArrayBuffer;
    let buffer: AudioBuffer;
    try {
      buffer = await audioCtx.decodeAudioData(ab);
    } catch (err) {
      // eslint-disable-next-line no-console
      console.warn("[quest] preview decode failed:", (err as Error).message, entry.contentType);
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

  return { onMeta, onChunk, play, stop, bufferedBytes, dispose };
}
