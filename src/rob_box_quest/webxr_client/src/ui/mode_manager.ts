// Client-side mode manager: tracks local UI mode (audio / voice / teleop).
// Архитектурное решение (Phase 2 интеграция): серверный ModeManager FSM
// (ADR-0028, `rob_box_supervisor`) держит ground truth — но клиенту
// нужен ОТДЕЛЬНЫЙ лёгкий стор для отслеживания «что я сейчас хочу»
// (открыт voice picker, отправляю teleop, в режиме radio vs robot_voice).
//
// Не путать с серверным avatar_supervisor.ModeManager (FSM
// off/telegram_active/avatar_present/mixed, ADR-0028 §3). Это
// чисто UI-state — какая кнопка подсвечена, какой preset выбран.
//
// Использование:
//   const mm = createModeManager();
//   mm.on("voice", (mode) => …);        // "radio" | "robot_voice" | "off"
//   mm.setVoiceMode("robot_voice");      // → emit
//   mm.currentVoice();                   // → string | null
//   mm.setCurrentVoice("alena");
//   mm.currentPreset();                  // → VoicePreset | null
//   mm.setPreset("friendly");
//   mm.currentLanguage();                // → VoiceLanguage | null
//   mm.setLanguage("ru");
//
// Подписки — observer pattern, listener'ы вызываются синхронно.

import type { VoicePreset, VoiceLanguage, VoicePresetId } from "../wire/messages";

export type ClientVoiceMode = "off" | "radio" | "robot_voice";
export type ClientTeleopState = "disarmed" | "armed";

export interface ClientModeSnapshot {
  voiceMode: ClientVoiceMode;
  teleopState: ClientTeleopState;
  currentVoice: string | null;
  currentPreset: VoicePreset | null;
  currentLanguage: VoiceLanguage | null;
}

/**
 * AV-28 §P7: дефолтные значения UI (отображаются до того, как сервер
 * ответит списком пресетов). Совпадают с default_preset /
 * default_language в src/rob_box_voice/config/voice_presets.yaml.
 */
export interface ClientModeDefaults {
  preset?: VoicePresetId;
  language?: VoiceLanguage;
}

export type ModeListener = (snap: ClientModeSnapshot) => void;

export interface ClientModeManager {
  setVoiceMode(mode: ClientVoiceMode): void;
  setTeleopState(state: ClientTeleopState): void;
  setCurrentVoice(voiceId: string | null): void;
  setCurrentPreset(preset: VoicePreset | null): void;
  setCurrentLanguage(language: VoiceLanguage | null): void;
  /** Записать факт WELCOME / STATE_UPDATE от сервера. */
  snapshot(): ClientModeSnapshot;
  /** Подписаться на изменения. Возвращает unsubscribe. */
  on(listener: ModeListener): () => void;
  /** Сбросить все поля в дефолт. */
  reset(): void;
}

const DEFAULT_SNAPSHOT: ClientModeSnapshot = Object.freeze({
  voiceMode: "off",
  teleopState: "disarmed",
  currentVoice: null,
  currentPreset: null,
  currentLanguage: null
});

export function createModeManager(
  initial?: Partial<ClientModeSnapshot>,
  defaults?: ClientModeDefaults
): ClientModeManager {
  // Дефолтный snapshot собирается из defaults (если переданы) — это
  // позволяет UI до ответа сервера показать корректно подсвеченные
  // кнопки (technical + ru, как в voice_presets.yaml).
  const initialSnapshot: ClientModeSnapshot = {
    ...DEFAULT_SNAPSHOT,
    ...(defaults?.preset ? { currentPreset: defaults.preset } : {}),
    ...(defaults?.language ? { currentLanguage: defaults.language } : {}),
    ...initial
  };
  let snap: ClientModeSnapshot = initialSnapshot;
  const listeners = new Set<ModeListener>();

  function emit(prev: ClientModeSnapshot): void {
    if (prev === snap) return;
    const copy: ClientModeSnapshot = { ...snap };
    for (const cb of listeners) {
      try {
        cb(copy);
      } catch (err) {
        // eslint-disable-next-line no-console
        console.warn("[mode-manager] listener threw:", err);
      }
    }
  }

  function setVoiceMode(mode: ClientVoiceMode): void {
    if (snap.voiceMode === mode) return;
    const prev = snap;
    snap = { ...snap, voiceMode: mode };
    emit(prev);
  }

  function setTeleopState(state: ClientTeleopState): void {
    if (snap.teleopState === state) return;
    const prev = snap;
    snap = { ...snap, teleopState: state };
    emit(prev);
  }

  function setCurrentVoice(voiceId: string | null): void {
    if (snap.currentVoice === voiceId) return;
    const prev = snap;
    snap = { ...snap, currentVoice: voiceId };
    emit(prev);
  }

  function setCurrentPreset(preset: VoicePreset | null): void {
    if (snap.currentPreset === preset) return;
    const prev = snap;
    snap = { ...snap, currentPreset: preset };
    emit(prev);
  }

  function setCurrentLanguage(language: VoiceLanguage | null): void {
    if (snap.currentLanguage === language) return;
    const prev = snap;
    snap = { ...snap, currentLanguage: language };
    emit(prev);
  }

  function getSnapshot(): ClientModeSnapshot {
    return { ...snap };
  }

  function on(listener: ModeListener): () => void {
    listeners.add(listener);
    return () => {
      listeners.delete(listener);
    };
  }

  function reset(): void {
    const prev = snap;
    snap = { ...initialSnapshot };
    emit(prev);
  }

  return {
    setVoiceMode,
    setTeleopState,
    setCurrentVoice,
    setCurrentPreset,
    setCurrentLanguage,
    snapshot: getSnapshot,
    on,
    reset
  };
}
