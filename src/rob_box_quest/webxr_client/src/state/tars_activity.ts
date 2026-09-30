// Что делает ТАРС «прямо сейчас» — СЛУШАЕТ / ДУМАЕТ / ГОВОРИТ / ждёт.
//
// Чистая логика без Three.js/DOM: main.ts кормит трекер теми сигналами,
// которые клиент УЖЕ получает (новых топиков и серверных правок нет), а
// панель ТАРС 1 рисует результат. Что из чего выводится (и что НЕ видно):
//
//   ГОВОРИТ  ← tars_state=speaking | tars1_text streaming | чанк
//              operator_tts_audio пришёл недавно (сервер operator_tts_done
//              в норме не шлёт, поэтому у аудио — TTL, а не «до done»).
//   ДУМАЕТ   ← tars_state=accepted|thinking | voice_state=thinking.
//   СЛУШАЕТ  ← грип/PTT зажат у оператора | voice_state=listening.
//   ЖДЁТ     ← ничего из вышеперечисленного свежее TTL.
//
// Каждый сигнал протухает по своему TTL: если сервер потерял `idle`, экран
// не застрянет на «ДУМАЕТ» навсегда. Приоритет: говорит > думает > слушает.
// voice_state=speaking намеренно НЕ считается «ТАРС говорит»: это голос
// самого робота (личность), а не ТАРС-в-шлем.

export type TarsActivity = "idle" | "listening" | "thinking" | "speaking";

/** Сколько живёт сигнал без подтверждения, мс. */
export const TARS_TTL_MS = {
  /** Чанк operator_tts_audio: реплики идут кусками, между ними паузы < 1 с. */
  ttsChunk: 2000,
  /** tars_state accepted/thinking без последующего speaking/idle. */
  thinking: 30_000,
  /** tars_state=speaking без idle (сервер done не шлёт). */
  speaking: 20_000,
  /** voice_state listening/thinking (обновляется мостом при смене). */
  voiceState: 60_000,
  /** tars1_text streaming=true без done. */
  tars1Stream: 20_000
} as const;

export interface TarsSignals {
  pttHeld: boolean;
  /** voice_state моста (stage робота) + когда пришёл. */
  voiceState: { state: string; atMs: number } | null;
  /** tars_state события: stage + когда пришёл. */
  tarsStage: { stage: string; atMs: number } | null;
  /** Время прихода последнего чанка operator_tts_audio. */
  ttsChunkAtMs: number | null;
  /** tars1_text: streaming=true (без done) + когда пришёл. */
  tars1Stream: { on: boolean; atMs: number } | null;
}

export const EMPTY_TARS_SIGNALS: TarsSignals = {
  pttHeld: false,
  voiceState: null,
  tarsStage: null,
  ttsChunkAtMs: null,
  tars1Stream: null
};

const fresh = (at: number, now: number, ttl: number): boolean => now - at >= 0 && now - at <= ttl;

/** Чистая функция «сигналы → состояние». */
export function deriveTarsActivity(s: TarsSignals, nowMs: number): TarsActivity {
  const stage = s.tarsStage;
  if (
    (stage && stage.stage === "speaking" && fresh(stage.atMs, nowMs, TARS_TTL_MS.speaking)) ||
    (s.tars1Stream && s.tars1Stream.on && fresh(s.tars1Stream.atMs, nowMs, TARS_TTL_MS.tars1Stream)) ||
    (s.ttsChunkAtMs !== null && fresh(s.ttsChunkAtMs, nowMs, TARS_TTL_MS.ttsChunk))
  ) {
    return "speaking";
  }
  const vs = s.voiceState;
  if (
    (stage &&
      (stage.stage === "accepted" || stage.stage === "thinking") &&
      fresh(stage.atMs, nowMs, TARS_TTL_MS.thinking)) ||
    (vs && vs.state === "thinking" && fresh(vs.atMs, nowMs, TARS_TTL_MS.voiceState))
  ) {
    return "thinking";
  }
  if (s.pttHeld || (vs && vs.state === "listening" && fresh(vs.atMs, nowMs, TARS_TTL_MS.voiceState))) {
    return "listening";
  }
  return "idle";
}

/** Изменяемая обёртка над сигналами: main.ts зовёт note*, читает state(). */
export interface TarsActivityTracker {
  notePtt(held: boolean): void;
  noteVoiceState(state: string, atMs: number): void;
  noteTarsStage(stage: string, atMs: number): void;
  noteTtsChunk(atMs: number): void;
  noteTars1Stream(on: boolean, atMs: number): void;
  state(nowMs: number): TarsActivity;
}

export function createTarsActivityTracker(): TarsActivityTracker {
  let s: TarsSignals = EMPTY_TARS_SIGNALS;
  return {
    notePtt: (held) => {
      s = { ...s, pttHeld: held };
    },
    noteVoiceState: (state, atMs) => {
      s = { ...s, voiceState: { state, atMs } };
    },
    noteTarsStage: (stage, atMs) => {
      // idle гасит и хвост звука: реплика закончилась.
      s = { ...s, tarsStage: { stage, atMs }, ttsChunkAtMs: stage === "idle" ? null : s.ttsChunkAtMs };
    },
    noteTtsChunk: (atMs) => {
      s = { ...s, ttsChunkAtMs: atMs };
    },
    noteTars1Stream: (on, atMs) => {
      s = { ...s, tars1Stream: { on, atMs } };
    },
    state: (nowMs) => deriveTarsActivity(s, nowMs)
  };
}

// ── Представление (без canvas) ──────────────────────────────────────────

export interface TarsActivityView {
  label: string;
  color: string;
  /** Символ-анимация для текущего кадра (спиннер). "" если нет. */
  glyph: string;
  /** 0..1 — яркость пульса (LISTENING) или 1. */
  pulse: number;
  /** Высоты 5 столбиков эквалайзера 0..1 (SPEAKING) или []. */
  bars: number[];
}

const SPINNER = ["|", "/", "-", "\\"];

/** Вид индикатора в момент nowMs. Детерминирован — тестируется без часов. */
export function tarsActivityView(a: TarsActivity, nowMs: number): TarsActivityView {
  switch (a) {
    case "listening":
      return {
        label: "LISTENING",
        color: "#33e0ff",
        glyph: "",
        pulse: 0.55 + 0.45 * Math.sin((nowMs / 1000) * Math.PI * 2 * 1.2),
        bars: []
      };
    case "thinking":
      return {
        label: "THINKING",
        color: "#ffb640",
        glyph: SPINNER[Math.floor(nowMs / 120) % SPINNER.length],
        pulse: 1,
        bars: []
      };
    case "speaking": {
      const bars: number[] = [];
      for (let i = 0; i < 5; i += 1) {
        bars.push(0.25 + 0.75 * Math.abs(Math.sin(nowMs / 180 + i * 1.7)));
      }
      return { label: "SPEAKING", color: "#39ff88", glyph: "", pulse: 1, bars };
    }
    default:
      return { label: "STANDBY", color: "#3f6f5a", glyph: "", pulse: 1, bars: [] };
  }
}
