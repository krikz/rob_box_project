// ADR-0078 / issue #2162 follow-up — видимая индикация цикла ТАРС.
//
// Отдельный индикатор от voice_state_indicator.ts: тот — про STT/LLM/TTS
// пайплайн **личности** робота, а tars_state — про стадии ТАРС-в-шлем:
//   accepted   — wake принят, синтезируем тон в шлеме
//   thinking   — супервизор думает (между accepted и speaking)
//   speaking   — реплика ТАРС пошла в шлем (оператор слышит голос)
//   idle       — цикл завершён
//
// Контракт события от моста: JSON_EVENT
//   {type:"tars_state", stage, request_id?, text?, ts_ms}
//
// Маппинг stage → цвет/лейбл — в файле, чтобы можно было подменить
// стилистику без правки парсера. Чистая функция — тестируется без
// Three.js (как в voice_state_indicator.ts).

export type TarsStage = "accepted" | "thinking" | "speaking" | "idle";

export const VALID_TARS_STAGES: ReadonlyArray<TarsStage> = [
  "accepted",
  "thinking",
  "speaking",
  "idle"
] as const;

export interface TarsStateEvent {
  stage: TarsStage;
  requestId?: string;
  text?: string;
  tsMs: number;
}

export interface TarsStatePresentation {
  label: string;
  color: string;
  ariaText: string;
}

const COLOR_ACCEPTED = "#3b8eea"; // голубой — wake принят
const COLOR_THINKING = "#f5a623"; // оранжевый — супервизор думает
const COLOR_SPEAKING = "#2ec27e"; // зелёный — реплика идёт
const COLOR_IDLE = "#8b98a5"; // серый — завершено

export function normalizeTarsStage(raw: unknown): TarsStage {
  if (typeof raw !== "string") return "idle";
  return (VALID_TARS_STAGES as readonly string[]).includes(raw)
    ? (raw as TarsStage)
    : "idle";
}

export function formatTarsStatePresentation(stage: TarsStage): TarsStatePresentation {
  switch (stage) {
    case "accepted":
      return {
        label: "ACCEPTED",
        color: COLOR_ACCEPTED,
        ariaText: "ТАРС: команда принята"
      };
    case "thinking":
      return {
        label: "THINKING",
        color: COLOR_THINKING,
        ariaText: "ТАРС: думаю"
      };
    case "speaking":
      return {
        label: "SPEAKING",
        color: COLOR_SPEAKING,
        ariaText: "ТАРС: говорю"
      };
    case "idle":
    default:
      return {
        label: "IDLE",
        color: COLOR_IDLE,
        ariaText: "ТАРС: ожидаю"
      };
  }
}

/** Парсер payload JSON_EVENT {type:"tars_state", stage, request_id, text, ts_ms}. */
export function parseTarsStateEvent(payload: unknown): TarsStateEvent | null {
  if (!payload || typeof payload !== "object") return null;
  const obj = payload as Record<string, unknown>;
  if (obj.type !== "tars_state") return null;
  const stage = normalizeTarsStage(obj.stage);
  const requestId = typeof obj.request_id === "string" ? obj.request_id : undefined;
  const text = typeof obj.text === "string" ? obj.text : undefined;
  const tsMs =
    typeof obj.ts_ms === "number" && Number.isFinite(obj.ts_ms) ? obj.ts_ms : 0;
  return { stage, requestId, text, tsMs };
}
