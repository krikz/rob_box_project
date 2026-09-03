// Voice State Indicator (AV-20 / §4-bis аудита 30.08).
//
// Мост backend'а публикует voice_state (0x1202) через msgpack:
//     {state: "idle"|"listening"|"thinking"|"speaking",
//      ts_ms: int, detail?: "silenced"|...}
//
// Этот модуль:
//   1. Парсит payload в типизированный VoiceState + опциональный VoiceDetail.
//   2. Маппит состояние на визуальные свойства (цвет, лейбл) и текстовое
//      описание для a11y (DOM aria-live="polite").
//   3. Рисует Three.js-спрайт (как status_hud.ts и arm-state sprite) —
//      всегда повёрнут к оператору, читается из любой позы.
//
// Контракт состояний:
//   - "idle"       → серый, "IDLE"        (робот молчит, не слушает).
//   - "listening"  → голубой, "LISTENING" (STT фаза).
//   - "thinking"   → оранжевый, "THINKING" (LLM фаза, если backend
//                                              решит её отделить от speaking).
//   - "speaking"   → зелёный, "SPEAKING"  (TTS фаза).
//   - detail="denied"   → красный, "MIC DENIED" — отдельный визуальный
//                          стиль + детальный текст, потому что «микрофон
//                          занят» оператор должен заметить мгновенно.
//   - detail="silenced" → серый, "MUTED"   — робот принудительно молчит.
//
// Неизвестный state / битый payload → "unknown" (серый, "—"), чтобы
// silent downgrade не сломал UI и был виден в логе.
//
// Использование:
//   const indicator = createVoiceStateIndicator();
//   scene.add(indicator.sprite);
//   indicator.setState(parseVoiceState(payload));
//   indicator.dispose();
//
// Логика разделена так же, как в status_hud.ts: формат и парсер — чистые,
// рисование — Three.js + canvas. Тесты формата/парсера не требуют canvas.

import * as THREE from "three";
import { decodeMsgpackMap, type MsgpackValue } from "../wire/msgpack";

/**
 * Допустимые voice_state по meta-quest-api.md §4 + детализированные
 * варианты (detail), которые мы показываем в HUD как ОТДЕЛЬНЫЙ визуальный
 * режим — не как цвет надписи.
 *
 * Экспортируем тип, чтобы тесты и другие подсистемы (например,
 * voice_picker в Phase 3) могли переиспользовать.
 */
export type VoiceState =
  | "idle"
  | "listening"
  | "thinking"
  | "speaking"
  | "unknown";

export type VoiceDetail = "none" | "denied" | "silenced";

/** Распарсенный кадр voice_state (или unknown + лог на битом payload). */
export interface VoiceStateFrame {
  state: VoiceState;
  detail: VoiceDetail;
  tsMs: number;
}

/**
 * Допустимые bridge-state'ы с бэкенда (streams/voice_state.py BRIDGE_STATES).
 * Если backend расширит список — здесь тоже расширяем одной строкой +
 * тест, чтобы UI не падал на новых значениях.
 */
const BRIDGE_STATES: ReadonlyArray<VoiceState> = [
  "idle",
  "listening",
  "thinking",
  "speaking"
] as const;

const VALID_DETAILS: ReadonlyArray<VoiceDetail> = [
  "none",
  "denied",
  "silenced"
] as const;

function normalizeState(raw: unknown): VoiceState {
  if (typeof raw !== "string") return "unknown";
  // Backend (streams/voice_state.py) нормализует FSM-state в lowercase
  // литералы из BRIDGE_STATES. Uppercase от старого upstream НЕ
  // поддерживаем: пусть парсер вернёт unknown и backend залогирует
  // WARNING, чем мы будем молча принимать невалидный формат.
  return (BRIDGE_STATES as readonly string[]).includes(raw)
    ? (raw as VoiceState)
    : "unknown";
}

function normalizeDetail(raw: unknown): VoiceDetail {
  if (typeof raw !== "string") return "none";
  return (VALID_DETAILS as readonly string[]).includes(raw)
    ? (raw as VoiceDetail)
    : "none";
}

/**
 * Распарсить msgpack-payload voice_state (0x1202).
 *
 * Возвращает `null` если payload битый (не map) — это сигнал
 * вызывающему коду пропустить кадр, а не падать (см. parseRobotStatus).
 */
export function parseVoiceState(payload: Uint8Array): VoiceStateFrame | null {
  const map = decodeMsgpackMap(payload);
  if (!map) return null;
  const stateRaw = (map as { state?: MsgpackValue }).state;
  const detailRaw = (map as { detail?: MsgpackValue }).detail;
  const tsRaw = (map as { ts_ms?: MsgpackValue }).ts_ms;
  const tsMs = typeof tsRaw === "number" && Number.isFinite(tsRaw) ? tsRaw : 0;
  return {
    state: normalizeState(stateRaw),
    detail: normalizeDetail(detailRaw),
    tsMs
  };
}

/**
 * Описание voice_state для HUD: человекочитаемая метка + цветовая фаза.
 * Чистая функция — тестируется без Three.js.
 */
export interface VoiceStatePresentation {
  /** Короткий лейбл, который пишем в спрайт. */
  label: string;
  /** Цвет текста/индикатора (HEX). */
  color: string;
  /** Текст для скринридера (aria-live). Чуть длиннее, чем label. */
  ariaText: string;
  /** Уровень важности — пока не используется, но оставлен для совместимости с status_hud.ts API. */
  level: "ok" | "warn" | "bad" | "unknown";
}

const COLOR_IDLE = "#8b98a5"; // серый — нейтральный
const COLOR_LISTENING = "#3b8eea"; // голубой — STT активен
const COLOR_THINKING = "#f5a623"; // оранжевый — LLM фаза
const COLOR_SPEAKING = "#2ec27e"; // зелёный — TTS фаза
const COLOR_BAD = "#e01b24"; // красный — отказ/ошибка

export function formatVoiceStatePresentation(frame: VoiceStateFrame): VoiceStatePresentation {
  // Приоритет: detail. Это «сильнее» state — даже если backend ещё не
  // успел прислать speaking, явный "denied" важнее.
  if (frame.detail === "denied") {
    return {
      label: "MIC DENIED",
      color: COLOR_BAD,
      ariaText: "Robot voice: microphone access denied",
      level: "bad"
    };
  }
  if (frame.detail === "silenced") {
    return {
      label: "MUTED",
      color: COLOR_IDLE,
      ariaText: "Robot voice is muted",
      level: "warn"
    };
  }
  switch (frame.state) {
    case "listening":
      return {
        label: "LISTENING",
        color: COLOR_LISTENING,
        ariaText: "Robot voice: listening",
        level: "ok"
      };
    case "thinking":
      return {
        label: "THINKING",
        color: COLOR_THINKING,
        ariaText: "Robot voice: thinking",
        level: "ok"
      };
    case "speaking":
      return {
        label: "SPEAKING",
        color: COLOR_SPEAKING,
        ariaText: "Robot voice: speaking",
        level: "ok"
      };
    case "idle":
      return {
        label: "IDLE",
        color: COLOR_IDLE,
        ariaText: "Robot voice: idle",
        level: "ok"
      };
    case "unknown":
    default:
      return {
        label: "—",
        color: COLOR_IDLE,
        ariaText: "Robot voice: unknown",
        level: "unknown"
      };
  }
}

// ------------------------------------------------------------------
// Three.js HUD-объект (sprite + DOM live-region для a11y).
// ------------------------------------------------------------------

export interface VoiceStateIndicator {
  readonly sprite: THREE.Sprite;
  /** Обновить состояние HUD. `null` — данных ещё нет (рисуем прочерк). */
  setState(frame: VoiceStateFrame | null): void;
  /**
   * Ручная установка уровня (например, "denied" из локального источника —
   * navigator.mediaDevices.getUserMedia().catch). Используется клиентом,
   * если он сам знает, что микрофон недоступен (на Quest не пустит
   * WebXR-сессию, но при первом запуске браузер может отказать).
   */
  setLocalDetail(detail: VoiceDetail): void;
  dispose(): void;
}

export interface VoiceStateIndicatorOptions {
  /** Позиция спрайта в сцене (по умолчанию — центр стены над экраном). */
  position?: { x: number; y: number; z: number };
  /** Размер спрайта в метрах (default 1.1 × 0.5). */
  scale?: { x: number; y: number };
  /**
   * Куда монтировать visually-hidden aria-live-регион.
   * По умолчанию document.body.
   */
  ariaParent?: HTMLElement;
}

export function createVoiceStateIndicator(
  opts: VoiceStateIndicatorOptions = {}
): VoiceStateIndicator {
  const canvas = document.createElement("canvas");
  canvas.width = 512;
  canvas.height = 240;
  const ctx2d = canvas.getContext("2d");
  if (!ctx2d) {
    throw new Error("voice_state_indicator: failed to acquire 2D context");
  }
  const ctx: CanvasRenderingContext2D = ctx2d;
  const texture = new THREE.CanvasTexture(canvas);
  texture.minFilter = THREE.LinearFilter;
  texture.magFilter = THREE.LinearFilter;

  const sprite = new THREE.Sprite(
    new THREE.SpriteMaterial({ map: texture, depthTest: false, transparent: true })
  );
  const pos = opts.position ?? { x: 0, y: 2.85, z: -3.85 };
  const scale = opts.scale ?? { x: 1.1, y: 0.5 };
  sprite.position.set(pos.x, pos.y, pos.z);
  sprite.scale.set(scale.x, scale.y, 1);

  // a11y: visually-hidden live-region, который скринридер прочитает при
  // смене состояния. Делаем его DOM-элементом, а не частью canvas'а —
  // иначе OCR/screen-reader не получит текст.
  const ariaParent = opts.ariaParent ?? document.body;
  const live = document.createElement("div");
  live.className = "voice-state-indicator__live";
  live.setAttribute("role", "status");
  live.setAttribute("aria-live", "polite");
  live.setAttribute("aria-atomic", "true");
  // visually-hidden по Bootstrap-конвенции (читается скринридером,
  // невидим для зрячих).
  live.style.position = "absolute";
  live.style.width = "1px";
  live.style.height = "1px";
  live.style.padding = "0";
  live.style.margin = "-1px";
  live.style.overflow = "hidden";
  live.style.clip = "rect(0,0,0,0)";
  live.style.whiteSpace = "nowrap";
  live.style.border = "0";
  ariaParent.appendChild(live);

  let currentFrame: VoiceStateFrame | null = null;
  let localDetail: VoiceDetail = "none";
  let disposed = false;

  function effectiveFrame(): VoiceStateFrame | null {
    if (!currentFrame) return null;
    if (currentFrame.detail !== "none") return currentFrame;
    // Локальный detail (например, getUserMedia отказал) перекрывает bridge state.
    if (localDetail !== "none") {
      return { ...currentFrame, detail: localDetail };
    }
    return currentFrame;
  }

  function draw(): void {
    const frame = effectiveFrame();
    const presentation = formatVoiceStatePresentation(
      frame ?? { state: "unknown", detail: "none", tsMs: 0 }
    );
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    // Тёмная подложка — единый стиль с status_hud и arm-state sprite.
    ctx.fillStyle = "rgba(10, 13, 17, 0.72)";
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    // Цветной индикатор слева.
    ctx.fillStyle = presentation.color;
    ctx.fillRect(0, 0, 16, canvas.height);
    // Текст.
    ctx.fillStyle = presentation.color;
    ctx.font = "bold 56px monospace";
    ctx.textBaseline = "middle";
    ctx.fillText(presentation.label, 44, canvas.height / 2);
    texture.needsUpdate = true;
    // aria-live: текст для скринридера. Меняем текст ТОЛЬКО при смене,
    // чтобы не спамить.
    const nextAria = presentation.ariaText;
    if (live.textContent !== nextAria) {
      live.textContent = nextAria;
    }
  }

  draw();

  return {
    sprite,
    setState(frame: VoiceStateFrame | null): void {
      if (disposed) return;
      currentFrame = frame;
      draw();
    },
    setLocalDetail(detail: VoiceDetail): void {
      if (disposed) return;
      localDetail = detail;
      draw();
    },
    dispose(): void {
      if (disposed) return;
      disposed = true;
      texture.dispose();
      (sprite.material as THREE.SpriteMaterial).dispose();
      if (live.parentNode === ariaParent) {
        ariaParent.removeChild(live);
      }
    }
  };
}
