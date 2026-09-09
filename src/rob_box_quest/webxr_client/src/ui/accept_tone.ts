// ADR-0078 / issue #2162 follow-up — звуковой акцепт-тон в шлеме.
//
// Когда ТАРС принимает команду оператора (wake + STT), оператор слышит
// короткий тональный сигнал в шлеме — ДО голоса ТАРС. Это подтверждает,
// что wake-word услышан, и даёт обратную связь «жди, думаю» (а не
// «молчит, не понял»).
//
// Технически: WebAudio OscillatorNode + GainNode с ADSR-огибающей.
// Синтез ЛОКАЛЬНЫЙ (не через сеть) — иначе:
//   - лаг до 100+ мс (потеря момента «принято»);
//   - сетевой мусор при потерях даёт «жужжание»;
//   - зависимость от сервера.
//
// Форма: 880 Гц, ~100 мс, attack 10 мс, release 80 мс. ADSR руками,
// чтобы звук был мягкий (не «дзынь»).
//
// AudioContext: используем тот же, что и у operator_audio_sink (sink
// уже мог его создать микрофонным захватом рации — квест-immersive
// автоплей-политика). Если глобального нет — создаём свой.

export interface AcceptToneOptions {
  AudioContextCtor?: new () => AudioContext;
  /** Частота тона в Гц (default 880). */
  frequencyHz?: number;
  /** Полная длительность тона с release в мс (default 100). */
  durationMs?: number;
}

export interface AcceptTonePlayer {
  /** Проиграть тон. Бросает, если AudioContext недоступен. */
  play(): Promise<void>;
  dispose(): void;
}

/** Мусор: лимит частей для воспроизведения (защита от спама). */
export const ACCEPT_TONE_MAX_PER_SECOND = 5;

export function createAcceptTonePlayer(
  opts: AcceptToneOptions = {}
): AcceptTonePlayer {
  const frequencyHz = opts.frequencyHz ?? 880;
  const durationMs = opts.durationMs ?? 100;
  const AudioContextCtor =
    opts.AudioContextCtor ??
    (globalThis as unknown as { AudioContext?: new () => AudioContext })
      .AudioContext;

  let ctx: AudioContext | null = null;
  // Rate-limit: не более ACCEPT_TONE_MAX_PER_SECOND тонов в секунду.
  const recent: number[] = [];

  function ensureCtx(): AudioContext | null {
    if (!AudioContextCtor) return null;
    if (!ctx) ctx = new AudioContextCtor();
    return ctx;
  }

  async function play(): Promise<void> {
    const audioCtx = ensureCtx();
    if (!audioCtx) return;
    const now = performance.now();
    // Чистим старые timestamps (> 1 сек).
    while (recent.length > 0 && now - recent[0] > 1000) recent.shift();
    if (recent.length >= ACCEPT_TONE_MAX_PER_SECOND) return;
    recent.push(now);

    const osc = audioCtx.createOscillator();
    const gain = audioCtx.createGain();
    osc.type = "sine";
    osc.frequency.value = frequencyHz;
    const t0 = audioCtx.currentTime;
    const attack = 0.01;
    const release = Math.max(0.05, (durationMs - 10) / 1000);
    // ADSR (без sustain/release-hold — короткий «тик»):
    //   0     → 0.0  (silence)
    //   attack→ peak  (10 мс ramp up)
    //   release→ 0.0  (остаток — ramp down)
    gain.gain.setValueAtTime(0, t0);
    gain.gain.linearRampToValueAtTime(0.25, t0 + attack);
    gain.gain.linearRampToValueAtTime(0, t0 + attack + release);
    osc.connect(gain);
    gain.connect(audioCtx.destination);
    if (audioCtx.state === "suspended") {
      await audioCtx.resume();
    }
    osc.start(t0);
    osc.stop(t0 + attack + release + 0.01);
  }

  function dispose(): void {
    if (ctx) {
      void ctx.close();
      ctx = null;
    }
  }

  return { play, dispose };
}
