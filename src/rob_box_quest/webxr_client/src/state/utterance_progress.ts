// «Где сейчас моё сообщение» — прогресс одной реплики оператора.
//
// Проблема, которую решает модуль: оператор зажал грипп, сказал фразу,
// отпустил — и дальше тишина. HUD показывает voice_state (IDLE /
// LISTENING / SPEAKING), но между «отпустил» и «робот заговорил» проходят
// секунды, в которые бэкенд честно шлёт `idle`. Оператор видит IDLE и не
// понимает: реплику потеряли, её распознают или LLM ещё думает.
//
// Бэкенд отдельной «thinking»-фазы не отдаёт: streams/voice_state.py
// маппит DialogueStateKind {IDLE, LISTENING, DIALOGUE, SILENCED} в
// {idle, listening, speaking, idle+silenced}. Ждать расширения контракта
// не надо — недостающее знает сам клиент: это ОН зажимал грипп и ОН
// знает, что аудио уже отправлено. Поэтому стадию собираем из двух
// источников:
//
//   • локальные события PTT (ptt_start / ptt_stop) — что сделал оператор;
//   • voice_state с моста                          — что делает робот.
//
// Ничего не выдумываем: `thinking` выставляется только когда мы ТОЧНО
// отправили реплику и после этого получили от моста `idle` — то есть
// робот уже не слушает, но ещё не говорит. Если моста нет вовсе, стадия
// честно уходит в `stalled` по таймауту, а не висит «отправляю…».
//
// Чистый модуль: ни three.js, ни DOM — только состояние и переходы
// (тот же приём, что в `state/tts_picker_state.ts`). Рисует его панель
// голосового пайплайна (`scene/voice_pipeline_panel.ts`).

import type { VoiceDetail, VoiceState } from "../ui/voice_state_indicator";

/**
 * Стадии пути реплики. Порядок в массиве = порядок в прогресс-строке UI.
 *
 *   recording → sending → thinking → speaking → done
 *
 * `listening` — подтверждение моста, что робот слышит нас прямо сейчас;
 * в прогресс-строке она делит колонку с `recording` (для оператора это
 * одно и то же: «мой голос идёт в робота»).
 */
export type UtteranceStage =
  | "idle"
  | "recording"
  | "listening"
  | "sending"
  | "thinking"
  | "speaking"
  | "done"
  | "stalled";

/** Колонки прогресс-строки (UI рисует ровно их, слева направо). */
export const PROGRESS_STEPS = ["voice", "stt", "llm", "tts"] as const;
export type ProgressStep = (typeof PROGRESS_STEPS)[number];

export interface UtteranceProgressState {
  stage: UtteranceStage;
  /** Активный detail voice_state (denied/silenced) — сильнее стадии. */
  detail: VoiceDetail;
  /** Когда началась текущая стадия (ms epoch). 0 — стадия `idle`. */
  stageSinceMs: number;
  /** Когда оператор отпустил грипп (ms epoch). 0 — реплики в пути нет. */
  sentAtMs: number;
}

export const INITIAL_UTTERANCE_PROGRESS: UtteranceProgressState = Object.freeze({
  stage: "idle" as UtteranceStage,
  detail: "none" as VoiceDetail,
  stageSinceMs: 0,
  sentAtMs: 0
});

export type UtteranceProgressAction =
  /** Оператор зажал грипп робот-голоса. */
  | { kind: "ptt_start"; atMs: number }
  /** Оператор отпустил грипп: аудио ушло, ждём робота. */
  | { kind: "ptt_stop"; atMs: number }
  /** Кадр voice_state с моста. */
  | { kind: "voice_state"; state: VoiceState; detail: VoiceDetail; atMs: number }
  /** Ход времени: переводит done → idle и «в пути» → stalled. */
  | { kind: "tick"; atMs: number }
  /** Соединение потеряно — стадия больше не факт. */
  | { kind: "disconnected" };

/** Сколько держим «✓ готово», прежде чем вернуться в покой. */
export const DONE_LINGER_MS = 2500;

/**
 * Через сколько молчания реплика считается застрявшей. 12 с — это заметно
 * больше типичного STT+LLM+TTS круга, но всё ещё в пределах терпения
 * оператора: лучше честное «робот не отвечает», чем вечное «отправляю…».
 */
export const STALL_TIMEOUT_MS = 12000;

/** Стадии, в которых реплика оператора «в пути» и ждёт робота. */
const IN_FLIGHT: ReadonlySet<UtteranceStage> = new Set<UtteranceStage>([
  "sending",
  "thinking"
]);

export function utteranceProgressReducer(
  state: UtteranceProgressState,
  action: UtteranceProgressAction
): UtteranceProgressState {
  switch (action.kind) {
    case "ptt_start":
      return { ...state, stage: "recording", stageSinceMs: action.atMs, sentAtMs: 0 };

    case "ptt_stop":
      // Отпустили не во время записи (например, рация) — не наша реплика.
      if (state.stage !== "recording" && state.stage !== "listening") return state;
      return { ...state, stage: "sending", stageSinceMs: action.atMs, sentAtMs: action.atMs };

    case "voice_state": {
      // detail сильнее стадии: «микрофон не дали» / «робот замьючен»
      // оператор должен увидеть немедленно, в какой бы фазе мы ни были.
      const detail = action.detail;
      const next = nextStageForVoiceState(state.stage, action.state);
      if (next === state.stage && detail === state.detail) return state;
      return {
        ...state,
        detail,
        stage: next,
        stageSinceMs: next === state.stage ? state.stageSinceMs : action.atMs
      };
    }

    case "tick": {
      if (state.stage === "done" && action.atMs - state.stageSinceMs >= DONE_LINGER_MS) {
        return { ...INITIAL_UTTERANCE_PROGRESS, detail: state.detail };
      }
      if (IN_FLIGHT.has(state.stage) && action.atMs - state.sentAtMs >= STALL_TIMEOUT_MS) {
        return { ...state, stage: "stalled", stageSinceMs: action.atMs };
      }
      return state;
    }

    case "disconnected":
      return { ...INITIAL_UTTERANCE_PROGRESS };

    default:
      return state;
  }
}

/**
 * Ядро маппинга «состояние моста → стадия реплики». Вынесено отдельно,
 * потому что вся неочевидность фичи именно здесь: один и тот же `idle`
 * от моста означает разное в зависимости от того, что оператор делал.
 */
function nextStageForVoiceState(stage: UtteranceStage, state: VoiceState): UtteranceStage {
  switch (state) {
    case "listening":
      // Мост подтвердил, что слышит. Если мы уже отпустили грипп —
      // назад в «слушает» не откатываемся: реплика уже в пути.
      return stage === "recording" || stage === "idle" ? "listening" : stage;

    case "thinking":
      // Мост сейчас эту фазу не шлёт (см. streams/voice_state.py), но
      // контракт её объявляет — примем без правок, если появится.
      return "thinking";

    case "speaking":
      return "speaking";

    case "idle":
      // Ключевая ветка. `idle` после того, как реплика ушла, — это и есть
      // невидимая раньше пауза «распознаю + думаю».
      if (stage === "sending" || stage === "listening") return "thinking";
      // `idle` после того, как робот говорил, — реплика отработана.
      if (stage === "speaking") return "done";
      // `idle` в середине «думаю» ничего нового не сообщает.
      if (stage === "thinking") return "thinking";
      if (stage === "recording") return "recording";
      return stage === "done" || stage === "stalled" ? stage : "idle";

    case "unknown":
    default:
      // Битый кадр стадию не двигает — see parseVoiceState (возвращает
      // "unknown" вместо падения).
      return stage;
  }
}

// ---------------------------------------------------------------------------
// Представление для UI (чистое форматирование, тестируется без canvas).
// ---------------------------------------------------------------------------

export type StepStatus = "idle" | "active" | "done" | "bad";

export interface UtteranceProgressView {
  /** Крупная строка статуса — что происходит прямо сейчас. */
  label: string;
  /** Уточнение под строкой (может быть пустым). */
  hint: string;
  level: "ok" | "warn" | "bad" | "idle";
  /** Состояние каждой колонки прогресс-строки. */
  steps: Record<ProgressStep, StepStatus>;
  /** Секунды в текущей стадии; null — счётчик не нужен. */
  elapsedS: number | null;
}

const STAGE_LABEL: Readonly<Record<UtteranceStage, string>> = {
  idle: "ГОТОВ",
  recording: "ЗАПИСЬ",
  listening: "СЛУШАЮ",
  sending: "ОТПРАВЛЕНО",
  thinking: "ОБРАБАТЫВАЮ",
  speaking: "ГОВОРЮ",
  done: "ГОТОВО",
  stalled: "НЕТ ОТВЕТА"
};

const STAGE_HINT: Readonly<Record<UtteranceStage, string>> = {
  idle: "зажми грипп и говори",
  recording: "говори — микрофон открыт",
  listening: "робот слышит тебя",
  sending: "реплика ушла роботу",
  thinking: "распознаю и переписываю",
  speaking: "робот произносит реплику",
  done: "реплика отработана",
  stalled: "робот не ответил — проверь связь"
};

/**
 * Разложить стадию на колонки прогресс-строки. `done` у колонки означает
 * «эта ступень уже пройдена этой репликой», `active` — «идёт прямо сейчас».
 */
function stepsFor(stage: UtteranceStage): Record<ProgressStep, StepStatus> {
  const s = (voice: StepStatus, stt: StepStatus, llm: StepStatus, tts: StepStatus) => ({
    voice,
    stt,
    llm,
    tts
  });
  switch (stage) {
    case "recording":
    case "listening":
      return s("active", "idle", "idle", "idle");
    case "sending":
      return s("done", "active", "idle", "idle");
    case "thinking":
      return s("done", "done", "active", "idle");
    case "speaking":
      return s("done", "done", "done", "active");
    case "done":
      return s("done", "done", "done", "done");
    case "stalled":
      return s("done", "bad", "bad", "bad");
    case "idle":
    default:
      return s("idle", "idle", "idle", "idle");
  }
}

export function utteranceProgressView(
  state: UtteranceProgressState,
  nowMs: number
): UtteranceProgressView {
  // detail перекрывает стадию: это не «где реплика», а «почему её не будет».
  if (state.detail === "denied") {
    return {
      label: "МИКРОФОН НЕ ДАН",
      hint: "браузер не пустил к микрофону",
      level: "bad",
      steps: stepsFor("stalled"),
      elapsedS: null
    };
  }
  if (state.detail === "silenced") {
    return {
      label: "РОБОТ МОЛЧИТ",
      hint: "включён режим тишины",
      level: "warn",
      steps: stepsFor("idle"),
      elapsedS: null
    };
  }
  const level: UtteranceProgressView["level"] =
    state.stage === "stalled"
      ? "bad"
      : state.stage === "idle"
        ? "idle"
        : state.stage === "sending" || state.stage === "thinking"
          ? "warn"
          : "ok";
  // Счётчик показываем только там, где ожидание неограниченно и оператору
  // важно понимать, сколько он уже ждёт.
  const showElapsed =
    state.stage === "recording" ||
    state.stage === "sending" ||
    state.stage === "thinking" ||
    state.stage === "stalled";
  const elapsedS =
    showElapsed && state.stageSinceMs > 0
      ? Math.max(0, Math.floor((nowMs - state.stageSinceMs) / 1000))
      : null;
  return {
    label: STAGE_LABEL[state.stage],
    hint: STAGE_HINT[state.stage],
    level,
    steps: stepsFor(state.stage),
    elapsedS
  };
}
