// 3D-панель голосового пайплайна оператора (docs/design/dialogue-mode-spec
// §3.6 + docs/plans/2026-08-29-wave2-worker-prompts.md W6-2).
//
// Оператор видит и настраивает свой голосовой путь:
//
//   мой голос → STT → LLM → TTS → динамик
//
// Быстрые настройки ступеней:
//   - STT  вкл/выкл  (выкл = рация, сырой голос);
//   - LLM  вкл/выкл  (выкл = STT→TTS дословно); настройки: стиль + язык;
//   - TTS  голос     (открывает 3D-меню TTS picker, AV-27).
//
// Это НЕ DOM: в immersive-vr DOM не виден. Панель — canvas-текстура на
// Plane-меше + отдельные меши-кнопки на слое указателя (тот же приём, что
// в `supervisor_panel.ts`). Формат/разбор и геометрия — чистые функции
// (тестируются без Three.js), рисование — canvas + CanvasTexture.
//
// Живая телеметрия ступеней (§3.7 спека) — отдельная карточка, сюда не
// входит (W6-2: «Не в скоупе: живая телеметрия ступеней»).

import * as THREE from "three";
import { panelGeometry } from "./supervisor_panel";
import {
  PROGRESS_STEPS,
  type ProgressStep,
  type StepStatus,
  type UtteranceProgressView
} from "../state/utterance_progress";
import type {
  VoiceLanguage,
  VoicePreset,
  VoicePresetId,
  VoicePresetInfo
} from "../wire/messages";

// ───────────────────────── источник правды (чистые функции) ─────────────────────────

/** Префикс id целей указателя — отличаем от `sup:` (supervisor) и `tts:` (TTS picker). */
export const PIPELINE_TARGET_PREFIX = "vpl:";

export const STT_TARGET_ID = `${PIPELINE_TARGET_PREFIX}stt`;
export const LLM_TARGET_ID = `${PIPELINE_TARGET_PREFIX}llm`;
export const TTS_TARGET_ID = `${PIPELINE_TARGET_PREFIX}tts`;
/**
 * «Без стиля»: реплика уходит дословно, LLM-ступень выключается. Это не
 * седьмой пресет (в voice_presets.yaml его нет и быть не должно), а выход
 * из режима переписывания — поэтому отдельный id, а не `preset:none`.
 */
export const STYLE_OFF_TARGET_ID = `${PIPELINE_TARGET_PREFIX}style_off`;
/** id цели указателя для перетаскивания панели (весь фон, не кнопки). */
export const PIPELINE_DRAG_TARGET_ID = `${PIPELINE_TARGET_PREFIX}drag`;

export function presetTargetId(preset: VoicePresetId): string {
  return `${PIPELINE_TARGET_PREFIX}preset:${preset}`;
}

export function langTargetId(language: VoiceLanguage): string {
  return `${PIPELINE_TARGET_PREFIX}lang:${language}`;
}

/** Действие панели, о котором мостик сообщает наружу (bootstrap). */
export type VoicePipelineAction =
  | { kind: "stt" }
  | { kind: "llm" }
  | { kind: "tts" }
  | { kind: "preset"; preset: VoicePresetId }
  | { kind: "lang"; language: VoiceLanguage }
  /** Выключить переписывание: робот произносит сказанное дословно. */
  | { kind: "style_off" };

/**
 * Разобрать id цели указателя. `null` — цель не наша, обработчик должен
 * пропустить её дальше по цепочке (tts picker / supervisor / панели).
 */
export function parsePipelineTargetId(id: string): VoicePipelineAction | null {
  if (!id.startsWith(PIPELINE_TARGET_PREFIX)) return null;
  const rest = id.slice(PIPELINE_TARGET_PREFIX.length);
  if (rest === "stt") return { kind: "stt" };
  if (rest === "llm") return { kind: "llm" };
  if (rest === "tts") return { kind: "tts" };
  if (rest === "style_off") return { kind: "style_off" };
  if (rest.startsWith("preset:")) {
    const preset = rest.slice("preset:".length);
    return preset ? { kind: "preset", preset: preset as VoicePresetId } : null;
  }
  if (rest.startsWith("lang:")) {
    const language = rest.slice("lang:".length);
    return language ? { kind: "lang", language: language as VoiceLanguage } : null;
  }
  return null;
}

/**
 * HUD-метка текущего выбора стиля речи (ADR-0027 §3.4.1). Префикс `ST:`
 * именно для стиля речи (AV-28), чтобы не путать с `voice_id` (TTS picker,
 * AV-27) — другой «слой», отдельный выбор голоса.
 *
 *   renderHud(null, null)     = "ST:--"
 *   renderHud("lenin", null)  = "ST:LENIN"
 *   renderHud("lenin", "ru")  = "ST:LENIN@RU"
 */
export function renderHud(
  preset: VoicePreset | null,
  language: VoiceLanguage | null
): string {
  if (!preset) return "ST:--";
  const st = String(preset).toUpperCase();
  return language ? `ST:${st}@${String(language).toUpperCase()}` : `ST:${st}`;
}

/**
 * Порядок языков вывода = порядок кнопок. Совпадает с ключами `languages`
 * в src/rob_box_voice/config/voice_presets.yaml и с VOICE_LANGUAGES в
 * ws_server.py: расширение языка — правка всех трёх мест, иначе кнопка
 * будет ловить NACK.
 */
export const LANG_ORDER: ReadonlyArray<VoiceLanguage> = [
  "ru",
  "en",
  "fr",
  "de",
  "zh",
  "hi"
];

/** Короткая подпись кнопки языка (ISO-код: влезает в узкую кнопку). */
export const LANG_LABELS: Readonly<Record<VoiceLanguage, string>> = {
  ru: "RU",
  en: "EN",
  fr: "FR",
  de: "DE",
  zh: "ZH",
  hi: "HI"
};

/** Ширина канваса-текстуры панели. Высота считается из раскладки (ниже). */
export const CANVAS_W = 512;

/** Геометрия панели: спереди-справа (+60°) от оператора. */
export const VOICE_PIPELINE_ANGLE_DEG = 60;
export const VOICE_PIPELINE_RADIUS_M = 2.4;
export const VOICE_PIPELINE_Y_M = 1.4;
export const VOICE_PIPELINE_W_M = 0.95;

export interface Rect {
  x: number;
  y: number;
  w: number;
  h: number;
}

export interface ButtonRect {
  /** Полный id для PointerSystem (`vpl:preset:<id>` и т.п.). */
  id: string;
  /** Логическое имя (для тестов). */
  buttonId: string;
  rect: Rect;
}

export interface PipelineLayout {
  header: Rect;
  /** Строка «где сейчас моя реплика» — статус + 4 ступени пути. */
  progress: Rect;
  stt: Rect;
  llm: Rect;
  tts: Rect;
  sectionLabel: Rect;
  /** Кнопка «Без стиля» — отдельной строкой над чипами пресетов. */
  noStyle: ButtonRect;
  presetChips: ButtonRect[];
  langButtons: ButtonRect[];
  loading: Rect;
}

// ───────────────────────── раскладка (детерминированная) ─────────────────────────

const PAD_X = 24;
const PAD_Y = 20;
const HEADER_H = 56;
/**
 * Прогресс-строка — самый крупный элемент панели. Она отвечает на вопрос
 * «я сказал, и что дальше?», ради которого оператор вообще смотрит на
 * панель во время разговора, поэтому места ей отдано больше, чем любой
 * настройке: крупный статус + подсказка + 4 ступени.
 */
const PROGRESS_H = 124;
const TOGGLE_H = 44;
const TOGGLE_GAP = 8;
const SECTION_LABEL_H = 28;
const CHIP_H = 40;
const CHIP_GAP = 8;
const LANG_H = 40;
const LOADING_H = 28;
/** Сколько чипов пресетов в строке (6 пресетов → 2 строки по 3). */
const PRESETS_PER_ROW = 3;

export const PRESET_ORDER: ReadonlyArray<VoicePresetId> = [
  // «Перевод» первым: это самый частый выбор оператора, которому нужен не
  // характер, а просто другой язык. Стилизующие пресеты — дальше.
  "translate",
  "technical",
  "street",
  "caveman",
  "business",
  "philosopher",
  "lenin"
];

/**
 * Fallback-список пресетов (имена из `voice_presets.yaml`). Сервер НЕ шлёт
 * событие `voice_presets` — список статичен и известен клиенту; если сервер
 * когда-нибудь пришлёт свой — `setPresets` его заменит.
 */
export const FALLBACK_PRESETS: ReadonlyArray<VoicePresetInfo> = [
  { id: "translate", name: "Перевод" },
  { id: "technical", name: "Технический" },
  { id: "street", name: "По понятиям" },
  { id: "caveman", name: "Пещерный" },
  { id: "business", name: "Деловой" },
  { id: "philosopher", name: "Философ" },
  { id: "lenin", name: "Ленин" }
];

/** Сколько кнопок языка в строке (6 языков → 2 строки по 3). */
const LANGS_PER_ROW = 3;

export function computePipelineLayout(canvasW: number): PipelineLayout {
  const usableW = canvasW - 2 * PAD_X;
  let y = PAD_Y;

  const header: Rect = { x: PAD_X, y, w: usableW, h: HEADER_H };
  y += HEADER_H + TOGGLE_GAP;

  const progress: Rect = { x: PAD_X, y, w: usableW, h: PROGRESS_H };
  y += PROGRESS_H + TOGGLE_GAP;

  const stt: Rect = { x: PAD_X, y, w: usableW, h: TOGGLE_H };
  y += TOGGLE_H + TOGGLE_GAP;
  const llm: Rect = { x: PAD_X, y, w: usableW, h: TOGGLE_H };
  y += TOGGLE_H + TOGGLE_GAP;
  const tts: Rect = { x: PAD_X, y, w: usableW, h: TOGGLE_H };
  y += TOGGLE_H + TOGGLE_GAP;

  const sectionLabel: Rect = { x: PAD_X, y, w: usableW, h: SECTION_LABEL_H };
  y += SECTION_LABEL_H + TOGGLE_GAP;

  // «Без стиля» — во всю ширину над чипами: это не один из стилей, а
  // выход из режима переписывания, и визуально он не должен читаться
  // как седьмой равноправный чип.
  const noStyle: ButtonRect = {
    id: STYLE_OFF_TARGET_ID,
    buttonId: "style_off",
    rect: { x: PAD_X, y, w: usableW, h: CHIP_H }
  };
  y += CHIP_H + CHIP_GAP;

  // Чипы стилей — сеткой по 3 в ряд, в порядке PRESET_ORDER.
  const chipW = (usableW - (PRESETS_PER_ROW - 1) * CHIP_GAP) / PRESETS_PER_ROW;
  const presetChips: ButtonRect[] = [];
  PRESET_ORDER.forEach((preset, i) => {
    const row = Math.floor(i / PRESETS_PER_ROW);
    const col = i % PRESETS_PER_ROW;
    const rect: Rect = {
      x: PAD_X + col * (chipW + CHIP_GAP),
      y: y + row * (CHIP_H + CHIP_GAP),
      w: chipW,
      h: CHIP_H
    };
    presetChips.push({ id: presetTargetId(preset), buttonId: `preset:${preset}`, rect });
  });
  const presetRows = Math.ceil(PRESET_ORDER.length / PRESETS_PER_ROW);
  y += presetRows * CHIP_H + (presetRows - 1) * CHIP_GAP + TOGGLE_GAP;

  // Язык вывода — та же сетка по 3 в ряд, в порядке LANG_ORDER.
  const langW = (usableW - (LANGS_PER_ROW - 1) * CHIP_GAP) / LANGS_PER_ROW;
  const langButtons: ButtonRect[] = LANG_ORDER.map((lang, i) => {
    const row = Math.floor(i / LANGS_PER_ROW);
    const col = i % LANGS_PER_ROW;
    return {
      id: langTargetId(lang),
      buttonId: `lang:${lang}`,
      rect: {
        x: PAD_X + col * (langW + CHIP_GAP),
        y: y + row * (LANG_H + CHIP_GAP),
        w: langW,
        h: LANG_H
      }
    };
  });
  const langRows = Math.ceil(LANG_ORDER.length / LANGS_PER_ROW);
  y += langRows * LANG_H + (langRows - 1) * CHIP_GAP + TOGGLE_GAP;

  const loading: Rect = { x: PAD_X, y, w: usableW, h: LOADING_H };

  return {
    header,
    progress,
    stt,
    llm,
    tts,
    sectionLabel,
    noStyle,
    presetChips,
    langButtons,
    loading
  };
}

/**
 * Чистая hit-функция (тестируется без Three.js). Нормированные координаты
 * [0..1] от канваса. Возвращает id цели (`vpl:stt` и т.д.) или `null`.
 */
export function hitTest(
  xNorm: number,
  yNorm: number,
  layout: PipelineLayout,
  canvasWidthPx: number,
  canvasHeightPx: number
): string | null {
  if (
    typeof xNorm !== "number" ||
    typeof yNorm !== "number" ||
    !Number.isFinite(xNorm) ||
    !Number.isFinite(yNorm) ||
    canvasWidthPx <= 0 ||
    canvasHeightPx <= 0
  ) {
    return null;
  }
  const x = xNorm * canvasWidthPx;
  const y = yNorm * canvasHeightPx;
  const inRect = (r: Rect): boolean => x >= r.x && x < r.x + r.w && y >= r.y && y < r.y + r.h;

  if (inRect(layout.stt)) return STT_TARGET_ID;
  if (inRect(layout.llm)) return LLM_TARGET_ID;
  if (inRect(layout.tts)) return TTS_TARGET_ID;
  if (inRect(layout.noStyle.rect)) return layout.noStyle.id;
  for (const b of layout.presetChips) if (inRect(b.rect)) return b.id;
  for (const b of layout.langButtons) if (inRect(b.rect)) return b.id;
  return null;
}

/**
 * Высота канваса под раскладку: нижний край последнего элемента + отступ.
 * Считается, а не задаётся числом, — иначе добавление седьмого пресета или
 * четвёртого языка молча уводило бы кнопки за край текстуры (а вместе с
 * ними и hit-test, который живёт в тех же координатах).
 */
export function pipelineCanvasHeight(layout: PipelineLayout): number {
  return Math.ceil(layout.loading.y + layout.loading.h + PAD_Y);
}

export const CANVAS_H = pipelineCanvasHeight(computePipelineLayout(CANVAS_W));

/**
 * Высота панели держится в аспекте канваса — иначе текст и кнопки
 * растягиваются, а hit-test (нормированные UV → пиксели канваса) начинает
 * промахиваться мимо того, что оператор видит.
 */
export const VOICE_PIPELINE_H_M = (VOICE_PIPELINE_W_M * CANVAS_H) / CANVAS_W;

// ───────────────────────── состояние для отрисовки ─────────────────────────

export interface VoicePipelineView {
  presets: VoicePresetInfo[];
  languages: VoiceLanguage[];
  currentPreset: VoicePreset | null;
  currentLanguage: VoiceLanguage | null;
  loading: boolean;
  /** STT-ступень вкл/выкл; null — сервер ещё не подтвердил. */
  sttOn: boolean | null;
  /** LLM-ступень вкл/выкл; null — сервер ещё не подтвердил. */
  llmOn: boolean | null;
  /** Активный голос TTS (voice_id из voice_list / voice_set_ack). */
  currentVoice: string | null;
  /**
   * Где сейчас реплика оператора (`state/utterance_progress.ts`).
   * null — прогресс ещё не считался ни разу (панель показывает покой).
   */
  progress: UtteranceProgressView | null;
}

export const DEFAULT_VIEW: VoicePipelineView = {
  presets: [...FALLBACK_PRESETS],
  languages: [...LANG_ORDER],
  currentPreset: "technical",
  currentLanguage: "ru",
  loading: false,
  sttOn: null,
  llmOn: null,
  currentVoice: null,
  progress: null
};

export interface VoicePipelinePanelHandle {
  readonly object: THREE.Group;
  /** Цели для PointerSystem (кнопки панели). */
  targets(): Array<{ id: string; object: THREE.Object3D }>;
  setPresets(presets: VoicePresetInfo[]): void;
  setLanguages(languages: VoiceLanguage[]): void;
  setCurrentPreset(preset: VoicePreset | null): void;
  setCurrentLanguage(language: VoiceLanguage | null): void;
  setLoading(loading: boolean): void;
  setSttOn(on: boolean | null): void;
  setLlmOn(on: boolean | null): void;
  setCurrentVoice(voiceId: string | null): void;
  /** Обновить прогресс реплики (вызывается на каждое изменение стадии). */
  setProgress(progress: UtteranceProgressView | null): void;
  setVisible(visible: boolean): void;
  isVisible(): boolean;
  /** Переместить панель и развернуть лицом к оператору (0, y, 0). */
  setPosition(x: number, y: number, z: number): void;
  getPosition(): { x: number; y: number; z: number };
  dispose(): void;
}

// ───────────────────────── three.js рендер ─────────────────────────

const COLORS = {
  bg: "rgba(10, 13, 17, 0.92)",
  accent: "#2ec27e",
  warn: "#f5c211",
  bad: "#e01b24",
  mute: "#8b98a5",
  text: "#d6dde5",
  panelBtn: "rgba(28, 33, 39, 0.92)",
  panelBtnBlocked: "#1c2127",
  /** Фон прогресс-строки: чуть светлее панели, чтобы читалась как «табло». */
  progressBg: "rgba(20, 25, 32, 0.95)",
  /** Ступень, которую эта реплика уже прошла. */
  stepDone: "#1f6f4a",
  /** Ступень, которая идёт прямо сейчас. */
  stepActive: "#2ec27e",
  stepIdle: "#242b33"
};

/** Подписи колонок прогресс-строки. */
const STEP_LABELS: Readonly<Record<ProgressStep, string>> = {
  voice: "ГОЛОС",
  stt: "STT",
  llm: "LLM",
  tts: "TTS"
};

function stepFill(status: StepStatus): string {
  if (status === "active") return COLORS.stepActive;
  if (status === "done") return COLORS.stepDone;
  if (status === "bad") return COLORS.bad;
  return COLORS.stepIdle;
}

function levelColor(level: UtteranceProgressView["level"]): string {
  if (level === "bad") return COLORS.bad;
  if (level === "warn") return COLORS.warn;
  if (level === "idle") return COLORS.mute;
  return COLORS.accent;
}

function toggleStateLabel(on: boolean | null): string {
  if (on === null) return "…";
  return on ? "ВКЛ" : "ВЫКЛ";
}

export function createVoicePipelinePanel(): VoicePipelinePanelHandle {
  const canvas = document.createElement("canvas");
  canvas.width = CANVAS_W;
  canvas.height = CANVAS_H;
  const ctx2d = canvas.getContext("2d");
  if (!ctx2d) throw new Error("voice_pipeline_panel: failed to acquire 2D context");
  const ctx: CanvasRenderingContext2D = ctx2d;

  const texture = new THREE.CanvasTexture(canvas);
  texture.minFilter = THREE.LinearFilter;
  texture.magFilter = THREE.LinearFilter;

  const group = new THREE.Group();
  group.renderOrder = 15; // как supervisor-панель
  const geom = panelGeometry(VOICE_PIPELINE_ANGLE_DEG, VOICE_PIPELINE_RADIUS_M, VOICE_PIPELINE_Y_M);
  group.position.set(geom.position.x, geom.position.y, geom.position.z);
  group.rotation.y = Math.atan2(geom.facing.x, geom.facing.z);

  const mesh = new THREE.Mesh(
    new THREE.PlaneGeometry(VOICE_PIPELINE_W_M, VOICE_PIPELINE_H_M),
    new THREE.MeshBasicMaterial({ map: texture, transparent: true, depthTest: false })
  );
  mesh.renderOrder = 15;
  group.add(mesh);

  // Каждая кнопка — отдельный прозрачный меш для hit-test в PointerSystem.
  const buttonMeshes = new Map<string, THREE.Mesh>();
  const buttonMat = new THREE.MeshBasicMaterial({
    color: 0xffffff,
    transparent: true,
    opacity: 0.0,
    depthTest: false
  });

  function rebuildButtonMeshes(layout: PipelineLayout): void {
    for (const [, m] of buttonMeshes) {
      group.remove(m);
      m.geometry.dispose();
    }
    buttonMeshes.clear();
    const wScale = VOICE_PIPELINE_W_M / CANVAS_W;
    const hScale = VOICE_PIPELINE_H_M / CANVAS_H;
    const add = (id: string, r: Rect) => {
      const g = new THREE.PlaneGeometry(r.w * wScale, r.h * hScale);
      const m = new THREE.Mesh(g, buttonMat);
      m.position.set(
        -VOICE_PIPELINE_W_M / 2 + (r.x + r.w / 2) * wScale,
        VOICE_PIPELINE_H_M / 2 - (r.y + r.h / 2) * hScale,
        0.005
      );
      m.renderOrder = 16;
      group.add(m);
      buttonMeshes.set(id, m);
    };
    add(STT_TARGET_ID, layout.stt);
    add(LLM_TARGET_ID, layout.llm);
    add(TTS_TARGET_ID, layout.tts);
    add(layout.noStyle.id, layout.noStyle.rect);
    for (const b of layout.presetChips) add(b.id, b.rect);
    for (const b of layout.langButtons) add(b.id, b.rect);
  }

  let view: VoicePipelineView = { ...DEFAULT_VIEW, presets: [...DEFAULT_VIEW.presets] };
  const layout = computePipelineLayout(CANVAS_W);
  rebuildButtonMeshes(layout);

  function drawText(
    text: string,
    x: number,
    y: number,
    color: string,
    sizePx: number,
    align: CanvasTextAlign = "left",
    maxWidth?: number
  ): void {
    ctx.fillStyle = color;
    ctx.font = `bold ${sizePx}px monospace`;
    ctx.textAlign = align;
    ctx.textBaseline = "middle";
    ctx.fillText(text, x, y, maxWidth);
  }

  function drawButtonRect(r: Rect, label: string, hint: string, visual: "active" | "warn" | "idle" | "blocked"): void {
    const fill =
      visual === "active"
        ? COLORS.accent
        : visual === "warn"
          ? COLORS.warn
          : visual === "blocked"
            ? COLORS.panelBtnBlocked
            : COLORS.panelBtn;
    ctx.fillStyle = fill;
    ctx.fillRect(r.x, r.y, r.w, r.h);
    const labelColor = visual === "active" ? "#0a0d11" : visual === "blocked" ? COLORS.mute : COLORS.text;
    drawText(label, r.x + 12, r.y + r.h / 2 - 8, labelColor, 16);
    if (hint) {
      drawText(hint, r.x + 12, r.y + r.h / 2 + 12, visual === "active" ? "rgba(10,13,17,0.7)" : COLORS.mute, 12);
    }
  }

  /**
   * Прогресс-строка: крупный статус + счётчик ожидания + 4 ступени пути.
   *
   * Читается за один взгляд и отвечает ровно на один вопрос: «я сказал —
   * где сейчас моя реплика». Пока прогресс не считался (progress === null)
   * показываем покой, а не выдуманную стадию.
   */
  function drawProgress(rect: Rect): void {
    const p = view.progress;
    ctx.fillStyle = COLORS.progressBg;
    ctx.fillRect(rect.x, rect.y, rect.w, rect.h);

    const color = p ? levelColor(p.level) : COLORS.mute;
    // Цветная полоса слева — тот же приём, что в status HUD: уровень
    // видно боковым зрением, не вчитываясь в текст.
    ctx.fillStyle = color;
    ctx.fillRect(rect.x, rect.y, 6, rect.h);

    const label = p?.label ?? "ГОТОВ";
    const hint = p?.hint ?? "зажми грипп и говори";
    drawText(label, rect.x + 18, rect.y + 26, color, 26);
    if (p?.elapsedS !== null && p?.elapsedS !== undefined) {
      drawText(`${p.elapsedS}s`, rect.x + rect.w - 12, rect.y + 26, color, 22, "right");
    }
    drawText(hint, rect.x + 18, rect.y + 52, COLORS.mute, 14, "left", rect.w - 30);

    // Ступени: 4 равных сегмента со стрелками между ними.
    const steps = p?.steps;
    const barY = rect.y + 72;
    const barH = 34;
    const gap = 6;
    const segW = (rect.w - 12 - gap * (PROGRESS_STEPS.length - 1)) / PROGRESS_STEPS.length;
    PROGRESS_STEPS.forEach((step, i) => {
      const status: StepStatus = steps?.[step] ?? "idle";
      const x = rect.x + 6 + i * (segW + gap);
      ctx.fillStyle = stepFill(status);
      ctx.fillRect(x, barY, segW, barH);
      drawText(
        STEP_LABELS[step],
        x + segW / 2,
        barY + barH / 2,
        status === "active" || status === "bad" ? "#0a0d11" : COLORS.text,
        14,
        "center",
        segW - 6
      );
    });
  }

  function draw(): void {
    ctx.clearRect(0, 0, CANVAS_W, CANVAS_H);
    ctx.fillStyle = COLORS.bg;
    ctx.fillRect(0, 0, CANVAS_W, CANVAS_H);

    // Header: ГОЛОС + HUD ST:...
    drawText("ГОЛОС", layout.header.x + 8, layout.header.y + 20, COLORS.accent, 22);
    const hud = renderHud(view.currentPreset, view.currentLanguage);
    drawText(hud, layout.header.x + layout.header.w - 8, layout.header.y + 20, COLORS.text, 18, "right");

    // «Где моя реплика» — раньше настроек: во время разговора оператор
    // смотрит именно сюда.
    drawProgress(layout.progress);

    // Тумблеры STT / LLM.
    drawButtonRect(
      layout.stt,
      `STT: ${toggleStateLabel(view.sttOn)}`,
      view.sttOn === false ? "выкл = рация (сырой голос)" : "распознавание речи",
      view.sttOn === null ? "blocked" : view.sttOn ? "active" : "warn"
    );
    drawButtonRect(
      layout.llm,
      `LLM: ${toggleStateLabel(view.llmOn)}`,
      view.llmOn === false ? "выкл = STT→TTS дословно" : "стиль + язык (ниже)",
      view.llmOn === null ? "blocked" : view.llmOn ? "active" : "warn"
    );
    drawButtonRect(
      layout.tts,
      `ГОЛОС: ${view.currentVoice ?? "—"}`,
      "выбрать голос (TTS picker)",
      view.currentVoice ? "active" : "idle"
    );

    // Секция стиля речи.
    drawText("СТИЛЬ РЕЧИ", layout.sectionLabel.x + 8, layout.sectionLabel.y + layout.sectionLabel.h / 2, COLORS.mute, 14);

    // «Без стиля» — активна, когда LLM-ступень выключена: тогда робот
    // произносит сказанное дословно и ни один стиль не применяется.
    const styleOff = view.llmOn === false;
    drawButtonRect(
      layout.noStyle.rect,
      "Без стиля",
      styleOff ? "робот говорит дословно" : "выключить переписывание",
      view.loading ? "blocked" : styleOff ? "active" : "idle"
    );

    // Чипы пресетов. При выключенном LLM ни один стиль не применяется —
    // показываем их приглушёнными, чтобы подсветка не врала.
    for (const chip of layout.presetChips) {
      const preset = chip.buttonId.slice("preset:".length) as VoicePresetId;
      const info = view.presets.find((p) => p.id === preset);
      const label = info?.name ?? preset;
      const active = !styleOff && view.currentPreset === preset;
      drawButtonRect(
        chip.rect,
        label,
        "",
        view.loading || styleOff ? "blocked" : active ? "active" : "idle"
      );
    }

    // Язык.
    for (const b of layout.langButtons) {
      const lang = b.buttonId.slice("lang:".length) as VoiceLanguage;
      const active = view.currentLanguage === lang;
      drawButtonRect(b.rect, LANG_LABELS[lang] ?? lang.toUpperCase(), "", view.loading ? "blocked" : active ? "active" : "idle");
    }

    // Loading.
    if (view.loading) {
      drawText("Загрузка пресетов…", layout.loading.x + 8, layout.loading.y + layout.loading.h / 2, COLORS.mute, 14);
    }

    texture.needsUpdate = true;
  }

  function commit(): void {
    draw();
  }

  return {
    object: group,
    targets() {
      return [...buttonMeshes.entries()].map(([id, object]) => ({ id, object }));
    },
    setPresets(presets: VoicePresetInfo[]): void {
      view = { ...view, presets: [...presets] };
      if (view.currentPreset && !presets.some((p) => p.id === view.currentPreset)) {
        view = { ...view, currentPreset: null };
      }
      commit();
    },
    setLanguages(languages: VoiceLanguage[]): void {
      view = { ...view, languages: [...languages] };
      if (view.currentLanguage && !languages.includes(view.currentLanguage)) {
        view = { ...view, currentLanguage: null };
      }
      commit();
    },
    setCurrentPreset(preset: VoicePreset | null): void {
      if (view.currentPreset === preset) return;
      view = { ...view, currentPreset: preset };
      commit();
    },
    setCurrentLanguage(language: VoiceLanguage | null): void {
      if (view.currentLanguage === language) return;
      view = { ...view, currentLanguage: language };
      commit();
    },
    setLoading(loading: boolean): void {
      if (view.loading === loading) return;
      view = { ...view, loading };
      commit();
    },
    setSttOn(on: boolean | null): void {
      if (view.sttOn === on) return;
      view = { ...view, sttOn: on };
      commit();
    },
    setLlmOn(on: boolean | null): void {
      if (view.llmOn === on) return;
      view = { ...view, llmOn: on };
      commit();
    },
    setCurrentVoice(voiceId: string | null): void {
      if (view.currentVoice === voiceId) return;
      view = { ...view, currentVoice: voiceId };
      commit();
    },
    setProgress(progress: UtteranceProgressView | null): void {
      view = { ...view, progress };
      commit();
    },
    setVisible(visible: boolean): void {
      group.visible = visible;
    },
    isVisible(): boolean {
      return group.visible;
    },
    setPosition(x: number, y: number, z: number): void {
      group.position.set(x, y, z);
      // Нормаль к центру (оператор в 0,0): direction = normalize(-x, -z).
      const len = Math.hypot(x, z) || 1;
      group.rotation.y = Math.atan2(-x / len, -z / len);
    },
    getPosition(): { x: number; y: number; z: number } {
      return { x: group.position.x, y: group.position.y, z: group.position.z };
    },
    dispose(): void {
      for (const [, m] of buttonMeshes) m.geometry.dispose();
      buttonMat.dispose();
      texture.dispose();
      (mesh.material as THREE.Material).dispose();
      mesh.geometry.dispose();
    }
  };
}
