// 3D-панель управления режимами аватара (R14, ADR-0027 §2 R14, ADR-0028 §4).
//
// Назначение: оператор из очков меняет режим аватара (off / avatar_present /
// teleop_only / voice_only / mixed) и берёт/отдаёт floor голоса/телеопа, не
// произнося голосовых команд. Это наименьший шаг с наибольшей отдачей для
// голосовой ветки (аудит 30.08 §4-bis): одна и та же панель нужна и
// «заткнуть робота» (off), и «включить оператора» (avatar_present).
//
// Разделение как в остальной сцене: формат/разбор и геометрия — чистая логика
// (`parseSupervisorState`, `hitTest`, `panelGeometry`), рисование — canvas +
// CanvasTexture на Plane-меше.
//
// Источник правды — `SupervisorState`, который супервизор публикует в
// `/avatar/state` (msgpack: `mode`, `teleop_floor`, `voice_floor`,
// `last_event`, `last_event_ts_ms`, `held_by_<floor>`). До первого `STATE_UPDATE`
// панель показывает «неизвестно» и БЛОКИРУЕТ кнопки взятия floor (защита
// от того, как оператор шлёт вслепую и клиент тут же репортит ack).
//
// Геометрия (по ADR-0027 R14 + дизайн §6 фаза P5):
//   угол −105° от оператора — симметрично боковой видео-панели (−75°) со
//   стороны camera_oak_depth (но дальше влево, чтобы не перекрывать её);
//   радиус 2.4 м, высота 1.45 (чуть ниже глаз), facing в центр.

import * as THREE from "three";
import { decodeMsgpackMap } from "../wire/msgpack";

// ───────────────────────── источник правды ─────────────────────────

/** Режим аватара согласно ADR-0028 §4.1. */
export type AvatarMode =
  | "off"
  | "telegram_active"
  | "avatar_present"
  | "teleop_only"
  | "voice_only"
  | "mixed";

/** Детальная информация о владельце floor (кто и когда взял). */
export interface FloorHolder {
  client_id: string;
  /** ms since epoch — момент захвата. */
  since_ms: number;
}

/** Один floor (teleop или voice). */
export interface Floor {
  /** null если никто не держит. */
  held_by: FloorHolder | null;
  /** null если ack от супервизора ещё не приходил. */
  last_event: "ACQUIRED" | "RELEASED" | "TAKEOVER" | "DENIED" | null;
}

/** Полный state супервизора, каким его видит клиент. */
export interface SupervisorState {
  mode: AvatarMode;
  teleop_floor: Floor;
  voice_floor: Floor;
  /** Что в последний раз сказал супервизор (для тостов). */
  last_event: string;
  last_event_ts_ms: number;
}

/** Сырое состояние ДО первого `STATE_UPDATE` — показывает «неизвестно». */
export const UNKNOWN_STATE: SupervisorState = Object.freeze({
  mode: "off",
  teleop_floor: { held_by: null, last_event: null },
  voice_floor: { held_by: null, last_event: null },
  last_event: "UNKNOWN",
  last_event_ts_ms: 0
}) as SupervisorState;

/** Состояние «ещё ничего не получали» — для блокировки floor-кнопок. */
export function isUnknownState(s: SupervisorState): boolean {
  return s.last_event === "UNKNOWN" && s.last_event_ts_ms === 0;
}

// ───────────────────────── msgpack разбор ─────────────────────────

/** Numeric floor flags клиента могущего: bit0 = teleop, bit1 = voice. */
export function floorHeldBy(s: SupervisorState, kind: "teleop" | "voice"): FloorHolder | null {
  return s[kind === "teleop" ? "teleop_floor" : "voice_floor"].held_by;
}

export function floorEvent(s: SupervisorState, kind: "teleop" | "voice"): Floor["last_event"] {
  return s[kind === "teleop" ? "teleop_floor" : "voice_floor"].last_event;
}

const FLOOR_EVENTS: ReadonlyArray<Floor["last_event"]> = [
  "ACQUIRED",
  "RELEASED",
  "TAKEOVER",
  "DENIED",
  null
];

/**
 * Разбирает msgpack-payload супервизора. Зеркало `set_avatar_state` на
 * Python-стороне (см. docs/architecture/meta-quest-api.md §6 future).
 * Структура:
 *   {
 *     "mode": "avatar_present",
 *     "teleop": {"held_by": {"client_id": "x", "since_ms": 1700...},
 *                 "last_event": "ACQUIRED"},
 *     "voice":  {"held_by": null, "last_event": "RELEASED"},
 *     "last_event": "SET_MODE",
 *     "last_event_ts_ms": 1700000000
 *   }
 * `null` — payload битый или не map; UI остаётся в UNKNOWN_STATE.
 */
export function parseSupervisorState(payload: Uint8Array): SupervisorState | null {
  let map: Record<string, unknown> | null;
  try {
    map = decodeMsgpackMap(payload);
  } catch {
    return null;
  }
  if (!map) return null;

  const modeRaw = typeof map.mode === "string" ? map.mode : "off";
  const teleop = parseFloor(map.teleop);
  const voice = parseFloor(map.voice);
  const lastEvent = typeof map.last_event === "string" ? map.last_event : "STATE_UPDATE";
  const lastTs = typeof map.last_event_ts_ms === "number" ? map.last_event_ts_ms : Date.now();

  return {
    mode: isAvatarMode(modeRaw) ? modeRaw : "off",
    teleop_floor: teleop,
    voice_floor: voice,
    last_event: lastEvent,
    last_event_ts_ms: lastTs
  };
}

const AVATAR_MODES: ReadonlyArray<AvatarMode> = [
  "off",
  "telegram_active",
  "avatar_present",
  "teleop_only",
  "voice_only",
  "mixed"
];

function isAvatarMode(v: string): v is AvatarMode {
  return AVATAR_MODES.includes(v as AvatarMode);
}

function parseFloor(v: unknown): Floor {
  const obj = v && typeof v === "object" ? (v as Record<string, unknown>) : null;
  if (!obj) return { held_by: null, last_event: null };

  const heldRaw = obj.held_by;
  let held: FloorHolder | null = null;
  if (heldRaw && typeof heldRaw === "object") {
    const h = heldRaw as Record<string, unknown>;
    const id = typeof h.client_id === "string" ? h.client_id : "unknown";
    const since = typeof h.since_ms === "number" && Number.isFinite(h.since_ms) ? h.since_ms : Date.now();
    held = { client_id: id, since_ms: since };
  }

  const ev = obj.last_event;
  const event: Floor["last_event"] = FLOOR_EVENTS.find((c) => c === ev) ?? null;

  return { held_by: held, last_event: event };
}

// ───────────────────────── локальная FSM (UX-уровень) ─────────────────────────

/** Переходы режима согласно ADR-0028 §4.2. Запрещённые — `allowed=false`. */
const MODE_TRANSITIONS: Record<AvatarMode, ReadonlyArray<AvatarMode>> = {
  off: ["off", "avatar_present", "telegram_active"],
  telegram_active: ["telegram_active", "off", "avatar_present"],
  avatar_present: [
    "avatar_present",
    "off",
    "telegram_active",
    "teleop_only",
    "voice_only",
    "mixed"
  ],
  teleop_only: ["teleop_only", "avatar_present", "mixed", "off"],
  voice_only: ["voice_only", "avatar_present", "mixed", "off"],
  mixed: ["mixed", "avatar_present", "teleop_only", "voice_only", "off"]
};

export interface ModeTransitionResult {
  applied: boolean;
  /** Если applied=false, причина отказа FSM (например для UI-тоста). */
  reason?: string;
}

/**
 * Проверяет, можно ли перейти `from → to`. Не отправляет ничего на сервер —
 * чистая функция для UX (UI блокирует переход, заранее зная, что
 * супервизор ответит MODE_CONFLICT).
 */
export function canTransitionMode(from: AvatarMode, to: AvatarMode): ModeTransitionResult {
  if (from === to) return { applied: true };
  const allowed = MODE_TRANSITIONS[from];
  if (allowed.includes(to)) return { applied: true };
  return {
    applied: false,
    reason: `переход ${from} → ${to} запрещён FSM супервизора (ADR-0028 §4.2)`
  };
}

// ───────────────────────── геометрия ─────────────────────────

/** Положение панели режимов в радианах от оператора (−105°). */
export const SUPERVISOR_PANEL_ANGLE_DEG = -105;
export const SUPERVISOR_PANEL_RADIUS_M = 2.4;
export const SUPERVISOR_PANEL_Y_M = 1.45;
/** Размер панели в метрах: под размер всех кнопок внутри. */
export const SUPERVISOR_PANEL_W_M = 0.95;
export const SUPERVISOR_PANEL_H_M = 1.15;

export interface PanelGeometry {
  position: { x: number; y: number; z: number };
  /** Нормаль к центру (единичная) — `facing` в сцену. */
  facing: { x: number; z: number };
}

/**
 * Чистая геометрия (тестируется без Three.js). Угол отрицательный → слева;
 * −105° ставит панель ЗА `camera_oak_depth` (−75°) на левом фланге, не
 * перекрывая её и не залезая во фронт-сектор экрана-стены.
 */
export function panelGeometry(
  angleDeg = SUPERVISOR_PANEL_ANGLE_DEG,
  radius = SUPERVISOR_PANEL_RADIUS_M,
  y = SUPERVISOR_PANEL_Y_M
): PanelGeometry {
  const a = (angleDeg * Math.PI) / 180;
  const x = radius * Math.sin(a);
  const z = -radius * Math.cos(a);
  const fx = -Math.sin(a);
  const fz = Math.cos(a);
  return {
    position: { x, y, z },
    facing: { x: fx, z: fz }
  };
}

// ───────────────────────── кнопки и hitTest ─────────────────────────

/** Префикс id для PointerSystem — отличаем кнопки панели от видео-панелей. */
export const PANEL_TARGET_PREFIX = "sup:";

export interface ButtonRect {
  /** Идентификатор кнопки в PointerSystem: `sup:<buttonId>`. */
  id: string;
  /** Логическое имя для UI/тестов (например `mode:mixed`). */
  buttonId: string;
  /** Нормализованный прямоугольник на канвасе (0..1, 0..1, верх-лево). */
  rect: { x: number; y: number; w: number; h: number };
}

export type ButtonKind = "mode" | "floor" | "dialogue";

export interface PanelButton {
  kind: ButtonKind;
  /** Ключ режима для `kind=mode` или `teleop`/`voice` для kind=floor. */
  key: string;
  /** Подпись (имя для теста, используется во всём коде). */
  label: string;
  /** Длинная подпись для подсказки под кнопкой. */
  hint: string;
}

/** Раскладка кнопок панели (сверху-вниз). */
export const PANEL_BUTTONS: ReadonlyArray<PanelButton> = Object.freeze([
  // Режимы
  { kind: "mode", key: "avatar_present", label: "Я оператор", hint: "Полный доступ" },
  { kind: "mode", key: "teleop_only", label: "Только рулю", hint: "Без диалога" },
  { kind: "mode", key: "voice_only", label: "Только голос", hint: "Без телеопа" },
  { kind: "mode", key: "mixed", label: "Смешанный", hint: "Руль + голос" },
  { kind: "mode", key: "off", label: "Выключить", hint: "Аватар молчит" },
  // Floor
  { kind: "floor", key: "teleop", label: "Взять руль", hint: "Забрать телеоп" },
  { kind: "floor", key: "voice", label: "Взять голос", hint: "Забрать TTS" }
]);

/**
 * Вычисляет экранный rect для каждой кнопки на канвасе размером
 * `${canvasW}×${canvasH}`. Чистая функция — детерминированно, без Three.js.
 *
 * Раскладка:
 *   ┌───────────────────────────────┐
 *   │  STATE LINE  ← статус аватара │
 *   ├───────────────────────────────┤
 *   │  [Я оператор]                 │  ← 5 mode-кнопок
 *   │  [Только рулю]                │
 *   │  [Только голос]               │
 *   │  [Смешанный]                  │
 *   │  [Выключить]                  │
 *   ├───────────────────────────────┤
 *   │  [Взять руль]  [Отдать руль]  │  ← 2 floor-кнопки (teleop + voice)
 *   │  [Взять голос] [Отдать голос] │
 *   ├───────────────────────────────┤
 *   │  [Диалог робота: вкл] ◀ toggle │
 *   ├───────────────────────────────┤
 *   │  «Стиль: скоро» — AV-28       │
 *   └───────────────────────────────┘
 *
 * Кнопка «Отдать» показывается только если floor удерживается этим
 * клиентом; «Взять» — если не удерживается. Этот выбор делает
 * `computePanelButtons(state, voiceOff)`.
 */
export function computeButtonLayout(canvasW: number, canvasH: number): {
  stateLine: { x: number; y: number; w: number; h: number };
  floors: Record<"teleop" | "voice", { acquire: ButtonRect; release: ButtonRect }>;
  modeButtons: ButtonRect[];
  dialogueToggle: ButtonRect;
  styleStub: { x: number; y: number; w: number; h: number };
} {
  const modeCount = PANEL_BUTTONS.filter((b) => b.kind === "mode").length;
  // верх — state line + padding
  const PAD_X = 24;
  const PAD_Y = 20;
  const STATE_LINE_H = 64;
  const MODE_GAP = 8;
  const FLOOR_GAP = 8;
  const DIALOGUE_H = 56;
  const STYLE_H = 36;

  // modeRows: одинаковой высоты, делят доступное пространство.
  const modeTotalH = canvasH - 2 * PAD_Y - STATE_LINE_H - DIALOGUE_H - STYLE_H - 2 * FLOOR_GAP;
  const modeRowH = (modeTotalH - (modeCount - 1) * MODE_GAP) / modeCount;

  const stateLine = { x: PAD_X, y: PAD_Y, w: canvasW - 2 * PAD_X, h: STATE_LINE_H };

  const modeButtons: ButtonRect[] = [];
  for (let i = 0; i < modeCount; i += 1) {
    const y = PAD_Y + STATE_LINE_H + i * (modeRowH + MODE_GAP);
    modeButtons.push({
      id: `${PANEL_TARGET_PREFIX}mode:${PANEL_BUTTONS[i].key}`,
      buttonId: `mode:${PANEL_BUTTONS[i].key}`,
      rect: { x: PAD_X, y, w: canvasW - 2 * PAD_X, h: modeRowH }
    });
  }

  // Floors — две строки (teleop + voice), по 2 кнопки (Взять/Отдать).
  const floorStartY = PAD_Y + STATE_LINE_H + modeCount * (modeRowH + MODE_GAP) + FLOOR_GAP;
  const floorRowH = (modeTotalH - FLOOR_GAP) / 4; // две floor-строки делят остаток
  const floors: Record<"teleop" | "voice", { acquire: ButtonRect; release: ButtonRect }> = {
    teleop: {
      acquire: makeFloorRect("teleop", "acquire", PAD_X, floorStartY, floorRowH, canvasW),
      release: makeFloorRect("teleop", "release", PAD_X, floorStartY, floorRowH, canvasW)
    },
    voice: {
      acquire: makeFloorRect("voice", "acquire", PAD_X, floorStartY + floorRowH + FLOOR_GAP, floorRowH, canvasW),
      release: makeFloorRect("voice", "release", PAD_X, floorStartY + floorRowH + FLOOR_GAP, floorRowH, canvasW)
    }
  };

  const dialogueY = canvasH - PAD_Y - DIALOGUE_H - STYLE_H - FLOOR_GAP;
  const dialogueToggle: ButtonRect = {
    id: `${PANEL_TARGET_PREFIX}dialogue:toggle`,
    buttonId: "dialogue:toggle",
    rect: { x: PAD_X, y: dialogueY, w: canvasW - 2 * PAD_X, h: DIALOGUE_H }
  };

  const styleY = canvasH - PAD_Y - STYLE_H;
  const styleStub = { x: PAD_X, y: styleY, w: canvasW - 2 * PAD_X, h: STYLE_H };

  return { stateLine, floors, modeButtons, dialogueToggle, styleStub };
}

function makeFloorRect(
  floor: "teleop" | "voice",
  op: "acquire" | "release",
  padX: number,
  y: number,
  h: number,
  canvasW: number
): ButtonRect {
  const usableW = canvasW - 2 * padX;
  const gap = 8;
  const halfW = (usableW - gap) / 2;
  const x = padX + (op === "release" ? halfW + gap : 0);
  return {
    id: `${PANEL_TARGET_PREFIX}floor:${floor}:${op}`,
    buttonId: `floor:${floor}:${op}`,
    rect: { x, y, w: halfW, h }
  };
}

/**
 * Чистая hit-функция (тестируется без Three.js).
 *
 * Принимает нормализованные координаты [0..1] и buttonLayout (см.
 * `computeButtonLayout`); возвращает `buttonId` первой кнопки, чей rect
 * содержит точку, либо `null`. Граничные правила: левый/верхний край —
 * внутри, правый/нижний — строго меньше (без двоичной ловушки при тач-
 * вводе на стыке).
 *
 * `canvasWidthPx` / `canvasHeightPx` — размеры канваса в пикселях, которые
 * использовались в `computeButtonLayout` (тот же канвас, что рисует
 * supervisorPanel). Нужны для перевода нормированных координат в абсолютные.
 */
export function hitTest(
  xNorm: number,
  yNorm: number,
  layout: ReturnType<typeof computeButtonLayout>,
  canvasWidthPx: number,
  canvasHeightPx: number
): string | null {
  if (
    typeof xNorm !== "number" ||
    typeof yNorm !== "number" ||
    !Number.isFinite(xNorm) ||
    !Number.isFinite(yNorm)
  ) {
    return null;
  }
  if (canvasWidthPx <= 0 || canvasHeightPx <= 0) return null;
  const x = xNorm * canvasWidthPx;
  const y = yNorm * canvasHeightPx;
  const inRect = (r: { x: number; y: number; w: number; h: number }): boolean =>
    x >= r.x && x < r.x + r.w && y >= r.y && y < r.y + r.h;

  // Проверяем в порядке: диалог → floor → mode → отбой.
  if (inRect(layout.dialogueToggle.rect)) return layout.dialogueToggle.buttonId;

  for (const kind of ["teleop", "voice"] as const) {
    const pair = layout.floors[kind];
    if (inRect(pair.acquire.rect)) return pair.acquire.buttonId;
    if (inRect(pair.release.rect)) return pair.release.buttonId;
  }
  for (const b of layout.modeButtons) {
    if (inRect(b.rect)) return b.buttonId;
  }
  return null;
}

// ───────────────────────── тексты / форматирование ─────────────────────────

/**
 * Подпись «кто держит» для индикатора floor. Использует короткую форму
 * (хвост после `:`) для длинных id, чтобы не растягивать UI. Для id < 8
 * символов оставляем оригинал — tg:42 узнаваемее, чем "42".
 */
export function shortHolderName(clientId: string | null | undefined): string {
  if (!clientId) return "—";
  return clientId.length > 8 ? clientId.slice(0, 6) + "…" : clientId;
}

/**
 * Подпись под кнопкой «Взять руль» / «Взять голос» — что произойдёт по факту.
 * Используется и в тосте отказа, и в подсказке панели.
 */
export function floorOccupantLabel(
  floor: Floor,
  selfClientId: string
): string {
  if (floor.held_by === null) return "свободно";
  if (floor.held_by.client_id === selfClientId) return "у вас";
  return `у ${shortHolderName(floor.held_by.client_id)}`;
}

/**
 * Строка статуса аватара в верхней части панели. До STATE_UPDATE —
 * «неизвестно», чтобы кнопки floor были заблокированы.
 */
export function formatStateLine(s: SupervisorState, selfClientId: string): string {
  if (isUnknownState(s)) return "статус: неизвестно";
  const teleop = floorOccupantLabel(s.teleop_floor, selfClientId);
  const voice = floorOccupantLabel(s.voice_floor, selfClientId);
  return `${humanMode(s.mode)} | руль: ${teleop} | голос: ${voice}`;
}

/** Русское имя режима аватара для UI. */
export function humanMode(m: AvatarMode): string {
  switch (m) {
    case "off":
      return "аватар выкл";
    case "telegram_active":
      return "telegram";
    case "avatar_present":
      return "вы — оператор";
    case "teleop_only":
      return "только рулю";
    case "voice_only":
      return "только голос";
    case "mixed":
      return "смешанный";
    default:
      return m;
  }
}

/**
 * Честная подпись тумблера «Диалог робота». По задаче: если `voice_input_mode
 * = off` на `dialogue_node` глушит ТОЛЬКО ReSpeaker (люди рядом), а голосовой
 * канал из Quest продолжает работать (см. ADR-0028 §2/S5 + meta-quest-api.md
 * voice_mode). Подпись должна это отражать, а не говорить «диалог выключен» —
 * иначе оператор думает, что всё молчит, а потом удивляется, что grip
 * контроллера по-прежнему слышен.
 */
export function formatDialogueToggle(state: SupervisorState, voiceOff: boolean): {
  label: string;
  hint: string;
} {
  if (isUnknownState(state)) {
    return { label: "Диалог робота: …", hint: "Статус неизвестен" };
  }
  if (voiceOff) {
    return {
      label: "Диалог робота: ВЫКЛ",
      hint: "Глушит людей рядом (ReSpeaker). Микрофон очков продолжает работать."
    };
  }
  return {
    label: "Диалог робота: ВКЛ",
    hint: "Голос рядом слышим, микрофон очков тоже."
  };
}

// ───────────────────────── three.js рендер ─────────────────────────

export interface SupervisorPanelOptions {
  /** Дефолтный canvas size (если первый draw вызван до setSize). */
  canvasWidth?: number;
  canvasHeight?: number;
  /** ID этого клиента — чтобы понять «у вас» в floor. */
  selfClientId?: string;
}

export interface SupervisorPanelHandle {
  readonly object: THREE.Group;
  /**
   * Зарегистрировать кнопки панели в PointerSystem. Возвращает массив
   * `{id, object}` — каждая кнопка это отдельный маленький меш.
   */
  targets(): Array<{ id: string; object: THREE.Object3D }>;
  /** Обновить состояние (пришёл `STATE_UPDATE`). */
  setState(state: SupervisorState): void;
  /** Обновить флажок `voice_off` (по событию voice_mode_ack). */
  setVoiceOff(off: boolean): void;
  /** Показать/скрыть. */
  setVisible(visible: boolean): void;
  isVisible(): boolean;
  /** Тоггл видимости — для горячей клавиши M. */
  toggleVisible(): void;
  /** Подсказка про «почему не нажали» — на случай MODE_CONFLICT. */
  showToast(message: string, level?: "warn" | "bad"): void;
  /** Адаптировать размер канваса под фактический меш (если пользователь drag-ом отрегулировал масштаб). */
  setCanvasSize(w: number, h: number): void;
  dispose(): void;
}

const CANVAS_W = 512;
const CANVAS_H = 640;
const COLORS = {
  bg: "#0a0d11",
  accent: "#2ec27e",
  warn: "#f5c211",
  bad: "#e01b24",
  mute: "#8b98a5",
  panel: "rgba(10, 13, 17, 0.92)"
};

export function createSupervisorPanel(
  opts: SupervisorPanelOptions = {}
): SupervisorPanelHandle {
  const canvas = document.createElement("canvas");
  let canvasW = Math.max(1, opts.canvasWidth ?? CANVAS_W);
  let canvasH = Math.max(1, opts.canvasHeight ?? CANVAS_H);
  canvas.width = canvasW;
  canvas.height = canvasH;
  const ctx2d = canvas.getContext("2d");
  if (!ctx2d) throw new Error("supervisor_panel: failed to acquire 2D context");
  const ctx: CanvasRenderingContext2D = ctx2d;
  const texture = new THREE.CanvasTexture(canvas);
  texture.minFilter = THREE.LinearFilter;
  texture.magFilter = THREE.LinearFilter;

  const group = new THREE.Group();
  group.renderOrder = 15; // ниже stream_menu (20), чтобы меню видео-панелей всплывало поверх
  const geom = panelGeometry();
  group.position.set(geom.position.x, geom.position.y, geom.position.z);
  group.rotation.y = Math.atan2(geom.facing.x, geom.facing.z);
  group.visible = false;

  // Plane-меш для canvas-текстуры.
  const mesh = new THREE.Mesh(
    new THREE.PlaneGeometry(SUPERVISOR_PANEL_W_M, SUPERVISOR_PANEL_H_M),
    new THREE.MeshBasicMaterial({ map: texture, transparent: true, depthTest: false })
  );
  mesh.renderOrder = 15;
  group.add(mesh);

  // Каждая кнопка — отдельный маленький меш для hit-test в PointerSystem
  // (так же, как строки меню в stream_menu.ts).
  const buttonMeshes = new Map<string, THREE.Mesh>();
  const buttonMat = new THREE.MeshBasicMaterial({
    color: 0xffffff,
    transparent: true,
    opacity: 0.0,
    depthTest: false
  });
  function rebuildButtonMeshes(): void {
    // Снять старые
    for (const [, m] of buttonMeshes) {
      group.remove(m);
      m.geometry.dispose();
    }
    buttonMeshes.clear();
    const layout = computeButtonLayout(canvasW, canvasH);
    const wScale = SUPERVISOR_PANEL_W_M / canvasW;
    const hScale = SUPERVISOR_PANEL_H_M / canvasH;
    const add = (id: string, r: { x: number; y: number; w: number; h: number }) => {
      const g = new THREE.PlaneGeometry(r.w * wScale, r.h * hScale);
      const m = new THREE.Mesh(g, buttonMat);
      m.position.set(
        -SUPERVISOR_PANEL_W_M / 2 + (r.x + r.w / 2) * wScale,
        SUPERVISOR_PANEL_H_M / 2 - (r.y + r.h / 2) * hScale,
        0.005
      );
      m.renderOrder = 16;
      group.add(m);
      buttonMeshes.set(id, m);
    };
    for (const b of layout.modeButtons) add(b.id, b.rect);
    add(layout.dialogueToggle.id, layout.dialogueToggle.rect);
    for (const kind of ["teleop", "voice"] as const) {
      add(layout.floors[kind].acquire.id, layout.floors[kind].acquire.rect);
      add(layout.floors[kind].release.id, layout.floors[kind].release.rect);
    }
  }
  rebuildButtonMeshes();

  let state: SupervisorState = UNKNOWN_STATE;
  let voiceOff = false;
  let toast: { message: string; level: "warn" | "bad"; untilMs: number } | null = null;
  const selfClientId = opts.selfClientId ?? "quest";

  function draw(): void {
    ctx.clearRect(0, 0, canvasW, canvasH);
    ctx.fillStyle = COLORS.panel;
    ctx.fillRect(0, 0, canvasW, canvasH);

    const layout = computeButtonLayout(canvasW, canvasH);

    // State line
    drawText(ctx, formatStateLine(state, selfClientId), layout.stateLine.x + 8, layout.stateLine.y + 22, COLORS.accent, 22, "left");
    if (!isUnknownState(state)) {
      const since = state.last_event_ts_ms > 0 ? `${state.last_event_ts_ms}` : "";
      drawText(ctx, `${state.last_event}${since ? " · ts " + since : ""}`,
        layout.stateLine.x + 8, layout.stateLine.y + 44, COLORS.mute, 14, "left");
    }

    // Mode buttons
    for (let i = 0; i < PANEL_BUTTONS.length; i += 1) {
      const meta = PANEL_BUTTONS[i];
      const r = layout.modeButtons[i]?.rect;
      if (meta.kind !== "mode" || !r) continue;
      const active = !isUnknownState(state) && state.mode === meta.key;
      drawButton(ctx, r, meta.label, meta.hint, active ? "active" : "idle");
    }

    // Floor buttons
    const blocked = isUnknownState(state);
    for (const kind of ["teleop", "voice"] as const) {
      const pair = layout.floors[kind];
      const floor = kind === "teleop" ? state.teleop_floor : state.voice_floor;
      const heldBySelf =
        floor.held_by !== null && floor.held_by.client_id === selfClientId;
      const held = floor.held_by !== null;
      const acquire: "active" | "blocked" | "idle" = blocked
        ? "blocked"
        : held && !heldBySelf
          ? "blocked"
          : "idle";
      const release: "active" | "blocked" | "idle" = blocked
        ? "blocked"
        : heldBySelf
          ? "active"
          : "idle";
      drawButton(
        ctx,
        pair.acquire.rect,
        `Взять ${kind === "teleop" ? "руль" : "голос"}`,
        floorOccupantLabel(floor, selfClientId),
        acquire
      );
      drawButton(
        ctx,
        pair.release.rect,
        `Отдать ${kind === "teleop" ? "руль" : "голос"}`,
        floorOccupantLabel(floor, selfClientId),
        release
      );
    }

    // Dialogue toggle
    const dt = formatDialogueToggle(state, voiceOff);
    drawButton(ctx, layout.dialogueToggle.rect, dt.label, dt.hint, voiceOff ? "warn" : "idle");

    // Style stub (AV-28)
    drawText(ctx, "Стиль: скоро", layout.styleStub.x + 8, layout.styleStub.y + 22, COLORS.mute, 16, "left");

    // Toast (красный/жёлтый баннер сверху)
    if (toast && toast.untilMs > Date.now()) {
      ctx.fillStyle = toast.level === "bad" ? COLORS.bad : COLORS.warn;
      ctx.fillRect(0, 0, canvasW, 24);
      drawText(ctx, toast.message, 8, 12, "#0a0d11", 16, "left");
    } else if (toast) {
      toast = null;
    }

    texture.needsUpdate = true;
  }

  function showToast(message: string, level: "warn" | "bad" = "warn"): void {
    toast = { message, level, untilMs: Date.now() + 2500 };
    draw();
  }

  function setState(next: SupervisorState): void {
    state = next;
    if (toast) {
      // Проглатываем старый toast при новом state — пользователь увидел последнее.
      toast = null;
    }
    draw();
  }

  function setVoiceOff(off: boolean): void {
    voiceOff = off;
    draw();
  }

  function setVisible(v: boolean): void {
    group.visible = v;
  }

  function isVisibleFn(): boolean {
    return group.visible;
  }

  function toggleVisibleFn(): void {
    setVisible(!group.visible);
  }

  function setCanvasSize(w: number, h: number): void {
    if (w === canvasW && h === canvasH) return;
    canvasW = Math.max(1, w);
    canvasH = Math.max(1, h);
    canvas.width = canvasW;
    canvas.height = canvasH;
    rebuildButtonMeshes();
    draw();
  }

  function dispose(): void {
    for (const [, m] of buttonMeshes) m.geometry.dispose();
    buttonMat.dispose();
    texture.dispose();
    (mesh.material as THREE.Material).dispose();
    mesh.geometry.dispose();
  }

  // Первая отрисовка — сразу показываем UNKNOWN.
  draw();

  return {
    object: group,
    targets() {
      return [...buttonMeshes.entries()].map(([id, object]) => ({ id, object }));
    },
    setState,
    setVoiceOff,
    setVisible,
    isVisible: isVisibleFn,
    toggleVisible: toggleVisibleFn,
    showToast,
    setCanvasSize,
    dispose
  };
}

// ───────────────────────── helpers ─────────────────────────

interface ButtonVisualProps {
  rect: { x: number; y: number; w: number; h: number };
  label: string;
  hint: string;
  /** активная = горит зелёным; warn = жёлтый; bad = красный; blocked = серый + курсор невидим; idle = тёмный. */
  visual: "active" | "warn" | "bad" | "idle" | "blocked";
}

function drawButton(ctx: CanvasRenderingContext2D, r: ButtonVisualProps["rect"], label: string, hint: string, visual: ButtonVisualProps["visual"]): void {
  const fill =
    visual === "active"
      ? COLORS.accent
      : visual === "warn"
        ? COLORS.warn
        : visual === "bad"
          ? COLORS.bad
          : visual === "blocked"
            ? "#1c2127"
            : "rgba(28, 33, 39, 0.92)";
  ctx.fillStyle = fill;
  ctx.fillRect(r.x, r.y, r.w, r.h);

  // Active outline.
  if (visual === "active") {
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 2;
    ctx.strokeRect(r.x + 1, r.y + 1, r.w - 2, r.h - 2);
  }

  ctx.fillStyle =
    visual === "active" ? "#0a0d11" : visual === "blocked" ? COLORS.mute : "#d6dde5";
  ctx.font = `bold ${Math.max(14, Math.min(22, Math.floor(r.h * 0.42)))}px monospace`;
  ctx.textBaseline = "top";
  ctx.fillText(label, r.x + 12, r.y + 8);

  ctx.font = `12px monospace`;
  ctx.fillStyle = visual === "active" ? "rgba(10,13,17,0.7)" : COLORS.mute;
  ctx.fillText(hint, r.x + 12, r.y + r.h - 18);
}

function drawText(
  ctx: CanvasRenderingContext2D,
  text: string,
  x: number,
  y: number,
  color: string,
  sizePx: number,
  align: "left" | "center" | "right"
): void {
  ctx.fillStyle = color;
  ctx.font = `bold ${sizePx}px monospace`;
  ctx.textAlign = align;
  ctx.textBaseline = "top";
  ctx.fillText(text, x, y);
  ctx.textAlign = "left";
}
