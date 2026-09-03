// Чистая модель состояния TTS picker'а (AV-27 / issue #1919, карточка t_a4cdf35f).
//
// Wire-контракт — `wire/messages.ts` (list_voices / set_voice / preview_voice +
// события voice_list / voice_set_ack / voice_set_nack / preview_voice_audio /
// preview_voice_done / preview_voice_error). Здесь НЕТ ни three.js, ни DOM:
// только состояние и переходы, чтобы 3D-меню (`scene/tts_picker_menu.ts`) и
// `main.ts` были тонкими применителями (тот же приём, что в
// `state/supervisor_state.ts` → `supervisorEffect`).
//
// Пять UI-состояний карточки:
//   1. loading    — открыли меню, отправили list_voices, ответа ещё нет;
//   2. empty      — сервер ответил `{voices: []}` → честный текст
//                   EMPTY_VOICES_TEXT, никаких выдуманных голосов;
//   3. ready      — список есть, текущий голос подсвечен;
//   4. previewing — идёт preview: progress по seq/total + кнопка STOP;
//   5. applying   — set_voice отправлен, UI залочен до ack/nack.
//
// Честность (ADR-0018): пока сервер не ответил, `currentVoiceId` = null и в
// UI прочерк, а не «первый голос из списка». Ошибку сервера показываем
// текстом reason, а не «применилось».

import type { VoiceInfo, VoicePreset } from "../wire/messages";

/** Текст пустого состояния. Дословно из карточки — не менять на «нет голосов». */
export const EMPTY_VOICES_TEXT = "Provider does not expose a voice list";

/** Фраза, которой озвучиваем preview. Короткая: preview — не спектакль. */
export const PREVIEW_TEXT = "Привет, оператор. Это проверка голоса.";

/** Префикс id целей указателя — чтобы не путать со `menu:` (stream_menu). */
export const TTS_TARGET_PREFIX = "tts:";

export type TtsPickerPhase = "closed" | "loading" | "empty" | "ready";

export interface TtsPreviewProgress {
  requestId: string;
  voiceId: string;
  /** Сколько чанков audio уже пришло. */
  received: number;
  /** Сколько обещал сервер в `total` (0 — ещё не знаем). */
  total: number;
  /** true — сервер прислал preview_voice_done. */
  done: boolean;
}

export interface TtsPickerState {
  phase: TtsPickerPhase;
  voices: ReadonlyArray<VoiceInfo>;
  /** Активный голос по данным сервера (voice_list.active_voice / voice_set_ack). */
  currentVoiceId: string | null;
  /** Активный провайдер (voice_list.active_provider). null — сервер не сказал. */
  activeProvider: string | null;
  /** Что оператор выделил в списке (ещё не применено). */
  selectedVoiceId: string | null;
  /** Не-null — set_voice отправлен, ждём ack/nack: UI залочен. */
  applyingVoiceId: string | null;
  /** Текущий preview (или null). */
  preview: TtsPreviewProgress | null;
  /** Инлайновая ошибка (nack / preview_error). null — ошибок нет. */
  error: string | null;
}

export const INITIAL_TTS_PICKER_STATE: TtsPickerState = Object.freeze({
  phase: "closed",
  voices: Object.freeze([]) as ReadonlyArray<VoiceInfo>,
  currentVoiceId: null,
  activeProvider: null,
  selectedVoiceId: null,
  applyingVoiceId: null,
  preview: null,
  error: null
});

export type TtsPickerAction =
  | { kind: "open" }
  | { kind: "close" }
  | { kind: "select"; voiceId: string }
  | {
      kind: "voice_list";
      voices: ReadonlyArray<VoiceInfo>;
      activeVoice?: string | null;
      activeProvider?: string | null;
    }
  | { kind: "apply_sent"; voiceId: string }
  | { kind: "voice_set_ack"; voiceId: string; preset?: VoicePreset }
  | { kind: "voice_set_nack"; voiceId?: string | null; reason: string; available?: ReadonlyArray<string> }
  | { kind: "preview_sent"; requestId: string; voiceId: string }
  | { kind: "preview_audio"; requestId: string; seq: number; total: number }
  | { kind: "preview_done"; requestId: string }
  | { kind: "preview_error"; requestId: string; reason: string }
  /** Оператор нажал STOP (или ушёл из меню) — локальная остановка. */
  | { kind: "preview_stopped" }
  /** Соединение потеряно: список/активный голос больше не факт. */
  | { kind: "disconnected" };

/**
 * Переход состояния. Чистая функция: тот же state на входе → тот же на
 * выходе, никаких side-effect'ов (команды на сокет отправляет `main.ts`).
 */
export function ttsPickerReducer(state: TtsPickerState, action: TtsPickerAction): TtsPickerState {
  switch (action.kind) {
    case "open":
      if (state.phase !== "closed") return state;
      // Открытие всегда идёт через loading: список запрашиваем заново
      // (провайдер мог смениться), но старые voices не выбрасываем —
      // они рисуются приглушённо, пока не пришёл свежий voice_list.
      return { ...state, phase: "loading", error: null };

    case "close":
      if (state.phase === "closed") return state;
      return { ...state, phase: "closed", preview: null, error: null };

    case "select": {
      if (!isInteractive(state)) return state;
      if (!state.voices.some((v) => v.voice_id === action.voiceId)) return state;
      if (state.selectedVoiceId === action.voiceId) return state;
      return { ...state, selectedVoiceId: action.voiceId, error: null };
    }

    case "voice_list": {
      const voices = [...action.voices];
      const activeVoice = action.activeVoice ?? null;
      const activeProvider = action.activeProvider ?? null;
      // Сервер прислал список — выбор оператора оставляем, только если
      // такой голос в новом списке ещё существует.
      const selected =
        state.selectedVoiceId && voices.some((v) => v.voice_id === state.selectedVoiceId)
          ? state.selectedVoiceId
          : null;
      return {
        ...state,
        phase: voices.length === 0 ? "empty" : "ready",
        voices,
        currentVoiceId: activeVoice ?? state.currentVoiceId,
        activeProvider: activeProvider ?? state.activeProvider,
        selectedVoiceId: selected,
        error: null
      };
    }

    case "apply_sent":
      if (state.applyingVoiceId !== null) return state;
      return { ...state, applyingVoiceId: action.voiceId, error: null };

    case "voice_set_ack":
      return {
        ...state,
        applyingVoiceId: null,
        currentVoiceId: action.voiceId,
        selectedVoiceId: null,
        error: null
      };

    case "voice_set_nack": {
      const available = action.available && action.available.length > 0
        ? ` (доступно: ${action.available.slice(0, 4).join(", ")})`
        : "";
      return {
        ...state,
        applyingVoiceId: null,
        error: `ERROR: ${action.reason}${available}`
      };
    }

    case "preview_sent":
      return {
        ...state,
        preview: {
          requestId: action.requestId,
          voiceId: action.voiceId,
          received: 0,
          total: 0,
          done: false
        },
        error: null
      };

    case "preview_audio": {
      const p = state.preview;
      if (!p || p.requestId !== action.requestId) return state;
      return {
        ...state,
        preview: {
          ...p,
          received: p.received + 1,
          total: action.total > 0 ? action.total : p.total
        }
      };
    }

    case "preview_done": {
      const p = state.preview;
      if (!p || p.requestId !== action.requestId) return state;
      return { ...state, preview: { ...p, done: true } };
    }

    case "preview_error": {
      const p = state.preview;
      if (p && p.requestId !== action.requestId) return state;
      return { ...state, preview: null, error: `ERROR: ${action.reason}` };
    }

    case "preview_stopped":
      if (state.preview === null) return state;
      return { ...state, preview: null };

    case "disconnected":
      // Список голосов и активный голос после разрыва — не факт (ADR-0018):
      // возвращаемся в loading, но меню не закрываем.
      return {
        ...state,
        phase: state.phase === "closed" ? "closed" : "loading",
        currentVoiceId: null,
        applyingVoiceId: null,
        preview: null,
        error: null
      };

    default:
      return state;
  }
}

/** Меню открыто и не залочено (не идёт apply) — можно жать строки/кнопки. */
export function isInteractive(state: TtsPickerState): boolean {
  return state.phase !== "closed" && state.applyingVoiceId === null;
}

/** Кнопка APPLY активна: выделен голос, меню не залочено. */
export function canApply(state: TtsPickerState): boolean {
  return isInteractive(state) && state.selectedVoiceId !== null;
}

/** Кнопка STOP активна: идёт preview (и он ещё не завершён сервером). */
export function canStopPreview(state: TtsPickerState): boolean {
  return state.preview !== null && !state.preview.done;
}

// ---------------------------------------------------------------------------
// Цели указателя (PointerSystem): id → действие.
// ---------------------------------------------------------------------------

export type TtsPickerTarget =
  | { kind: "select"; voiceId: string }
  | { kind: "preview"; voiceId: string }
  | { kind: "apply" }
  | { kind: "stop" }
  | { kind: "close" };

export function selectTargetId(voiceId: string): string {
  return `${TTS_TARGET_PREFIX}voice:${voiceId}`;
}

export function previewTargetId(voiceId: string): string {
  return `${TTS_TARGET_PREFIX}preview:${voiceId}`;
}

export const APPLY_TARGET_ID = `${TTS_TARGET_PREFIX}apply`;
export const STOP_TARGET_ID = `${TTS_TARGET_PREFIX}stop`;
export const CLOSE_TARGET_ID = `${TTS_TARGET_PREFIX}close`;

/**
 * Разобрать id цели указателя. `null` — цель не наша (панель, строка
 * stream_menu и т.п.), обработчик должен пропустить её дальше.
 */
export function parseTtsTargetId(id: string): TtsPickerTarget | null {
  if (!id.startsWith(TTS_TARGET_PREFIX)) return null;
  const rest = id.slice(TTS_TARGET_PREFIX.length);
  if (rest === "apply") return { kind: "apply" };
  if (rest === "stop") return { kind: "stop" };
  if (rest === "close") return { kind: "close" };
  if (rest.startsWith("voice:")) {
    const voiceId = rest.slice("voice:".length);
    return voiceId ? { kind: "select", voiceId } : null;
  }
  if (rest.startsWith("preview:")) {
    const voiceId = rest.slice("preview:".length);
    return voiceId ? { kind: "preview", voiceId } : null;
  }
  return null;
}

// ---------------------------------------------------------------------------
// Форматирование для отрисовки (canvas-текстуры рисует scene/tts_picker_menu).
// ---------------------------------------------------------------------------

export interface TtsRowView {
  voiceId: string;
  /** Основная строка: display_name + язык. */
  title: string;
  /** Вторая строка: provider + gender + tags/пресеты. */
  subtitle: string;
  /** Этот голос активен на сервере. */
  current: boolean;
  /** Этот голос выделен оператором. */
  selected: boolean;
  /** По этой строке сейчас идёт preview. */
  previewing: boolean;
}

/** Строки списка для рендера. Пустой массив в состояниях loading/empty. */
export function ttsRowViews(state: TtsPickerState): TtsRowView[] {
  if (state.phase !== "ready") return [];
  return state.voices.map((v) => ({
    voiceId: v.voice_id,
    title: `${v.display_name || v.voice_id}${v.language ? ` · ${v.language}` : ""}`,
    subtitle: rowSubtitle(v),
    current: state.currentVoiceId === v.voice_id,
    selected: state.selectedVoiceId === v.voice_id,
    previewing: state.preview?.voiceId === v.voice_id && !state.preview.done
  }));
}

function rowSubtitle(v: VoiceInfo): string {
  const parts: string[] = [];
  if (v.provider) parts.push(v.provider);
  if (v.gender) parts.push(v.gender);
  if (v.presets && v.presets.length > 0) parts.push(v.presets.join("/"));
  else if (v.description) parts.push(v.description);
  return parts.join(" · ");
}

/**
 * Заголовок меню: одна строка, честно отражающая фазу. Прочерк вместо
 * выдуманного значения (тот же принцип, что в status HUD).
 */
export function ttsHeaderText(state: TtsPickerState): string {
  if (state.applyingVoiceId !== null) return `APPLYING ${state.applyingVoiceId}…`;
  if (state.phase === "loading") return "TTS VOICES · loading…";
  if (state.phase === "empty") return "TTS VOICES · —";
  const provider = state.activeProvider ?? "—";
  const current = state.currentVoiceId ?? "—";
  return `TTS VOICES · ${provider} · now: ${current}`;
}

/**
 * Строка футера (прогресс preview / ошибка / подсказка). Возвращает текст и
 * уровень для цвета: ошибка красная, прогресс жёлтый, обычное — серое.
 */
export function ttsFooterText(state: TtsPickerState): { text: string; level: "ok" | "warn" | "bad" } {
  if (state.error) return { text: state.error, level: "bad" };
  const p = state.preview;
  if (p) {
    if (p.done) return { text: `PREVIEW ${p.voiceId}: done`, level: "ok" };
    const total = p.total > 0 ? `/${p.total}` : "";
    return { text: `PREVIEW ${p.voiceId}: chunk ${p.received}${total}…`, level: "warn" };
  }
  if (state.phase === "empty") return { text: EMPTY_VOICES_TEXT, level: "warn" };
  if (state.phase === "loading") return { text: "waiting for voice_list…", level: "warn" };
  if (state.selectedVoiceId) return { text: `selected ${state.selectedVoiceId} → APPLY`, level: "ok" };
  return { text: "pick a voice · PREVIEW / APPLY", level: "ok" };
}

// ---------------------------------------------------------------------------
// request_id для preview_voice.
// ---------------------------------------------------------------------------

/**
 * `request_id` обязателен (meta-quest-api.md §5): по нему клиент матчит
 * параллельные preview. Берём crypto.randomUUID, если он есть; иначе —
 * счётчик + Math.random (детерминируемо подменяется в тестах).
 */
export function newPreviewRequestId(
  rnd: () => number = Math.random,
  uuid?: () => string
): string {
  const gen =
    uuid ??
    (typeof globalThis.crypto !== "undefined" && typeof globalThis.crypto.randomUUID === "function"
      ? () => globalThis.crypto.randomUUID()
      : null);
  if (gen) return gen();
  return `pv-${Date.now().toString(36)}-${Math.floor(rnd() * 0xffffff).toString(16)}`;
}
