// PreviewPlayer: проигрывает аудио-байты (mp3/opus/wav) через WebAudio API.
//
// Phase 2 §4.2: при preview_voice → клиент собирает чанки BINARY_FRAME,
// ассоциированные с JSON_EVENT{type:"preview_voice_audio", request_id},
// в один Blob, затем decodeAudioData → AudioBufferSourceNode → destination.
//
// Тестируемый контракт:
//   - play(bytes, contentType): void
//   - stop(): void (прерывает текущий source)
//   - onPlayingChange: (cb) → callback когда состояние playing меняется
//   - onError: (cb) → callback когда ошибка decode/play
//   - lastPlayedContentType / lastPlayedDurationMs — для UI/тестов
//
// Аудио-контекст лениво создаётся при первом вызове play() (многие браузеры
// требуют user gesture для AudioContext.resume()).

export type PreviewState = "idle" | "playing" | "stopped" | "error";

export interface PreviewPlayerOptions {
  // Inject для тестов: AudioContext / fetch-стратегия для blob URL.
  AudioContextCtor?: typeof AudioContext;
  // Можно переопределить, если в jsdom нет AudioContext.
  enableAudioContext?: boolean;
}

export interface PreviewPlayerListeners {
  onPlayingChange?: (state: PreviewState) => void;
  onError?: (reason: string) => void;
}

export class PreviewPlayer {
  private ctx: AudioContext | null = null;
  private currentSource: AudioBufferSourceNode | null = null;
  private state: PreviewState = "idle";
  private listeners: PreviewPlayerListeners = {};
  private lastPlayedContentType: string | null = null;
  private lastPlayedDurationMs = 0;
  private readonly opts: Required<Omit<PreviewPlayerOptions, "AudioContextCtor">> & {
    AudioContextCtor: typeof AudioContext | null;
  };

  constructor(opts: PreviewPlayerOptions = {}) {
    this.opts = {
      AudioContextCtor: opts.AudioContextCtor ?? null,
      enableAudioContext: opts.enableAudioContext ?? true
    };
  }

  setListeners(listeners: PreviewPlayerListeners): void {
    this.listeners = listeners;
  }

  getState(): PreviewState {
    return this.state;
  }

  getLastPlayedContentType(): string | null {
    return this.lastPlayedContentType;
  }

  getLastPlayedDurationMs(): number {
    return this.lastPlayedDurationMs;
  }

  /** Запустить проигрывание. bytes — Uint8Array (mp3/opus/wav). */
  async play(bytes: Uint8Array, contentType: string): Promise<void> {
    if (!bytes || bytes.byteLength === 0) {
      this.fail("empty_audio");
      return;
    }
    // Прерываем предыдущий preview, если играет.
    this.stop();
    try {
      const ctx = this.ensureContext();
      // Копия в ArrayBuffer для decodeAudioData (типобезопасно).
      const buf = bytes.slice().buffer;
      const audioBuffer = await ctx.decodeAudioData(buf);
      const src = ctx.createBufferSource();
      src.buffer = audioBuffer;
      src.connect(ctx.destination);
      src.onended = (): void => {
        // onended сработает и при stop() — но мы уже выставили state=stopped там.
        if (this.currentSource === src) {
          this.currentSource = null;
          if (this.state === "playing") {
            this.setState("idle");
          }
        }
      };
      this.currentSource = src;
      this.lastPlayedContentType = contentType;
      this.lastPlayedDurationMs = Math.round(audioBuffer.duration * 1000);
      src.start(0);
      this.setState("playing");
    } catch (err) {
      this.fail(`decode_error: ${(err as Error).message}`);
    }
  }

  /** Прервать текущий preview. */
  stop(): void {
    const src = this.currentSource;
    if (src) {
      this.currentSource = null;
      try {
        src.stop();
      } catch {
        // уже остановлен — ignore
      }
      try {
        src.disconnect();
      } catch {
        // ignore
      }
    }
    if (this.state === "playing") {
      this.setState("stopped");
    }
  }

  /** Полная очистка (при destroy). */
  dispose(): void {
    this.stop();
    if (this.ctx) {
      this.ctx.close().catch(() => undefined);
      this.ctx = null;
    }
    this.listeners = {};
  }

  private ensureContext(): AudioContext {
    if (this.ctx) return this.ctx;
    if (!this.opts.enableAudioContext) {
      throw new Error("AudioContext disabled");
    }
    const Ctor = this.opts.AudioContextCtor ?? (globalThis as unknown as { AudioContext?: typeof AudioContext }).AudioContext;
    if (!Ctor) {
      throw new Error("AudioContext not available in this environment");
    }
    this.ctx = new Ctor();
    return this.ctx;
  }

  private setState(s: PreviewState): void {
    if (this.state === s) return;
    this.state = s;
    this.listeners.onPlayingChange?.(s);
  }

  private fail(reason: string): void {
    this.setState("error");
    this.listeners.onError?.(reason);
  }
}

/** Helper: генерирует стабильный request_id для preview_voice. */
export function makePreviewRequestId(): string {
  // Достаточно для уникальности в одном клиенте.
  return `pv_${Date.now().toString(36)}_${Math.random().toString(36).slice(2, 8)}`;
}
