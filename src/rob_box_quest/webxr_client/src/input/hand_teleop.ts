// src/input/hand_teleop.ts
//
// Phase 2.2 — Hand-tracking teleop binding (Quest 3 / Quest Pro, без контроллеров).
//
// Контракт:
//   - читает позы 25-joint XRHand через XRFrame.getJointPose(),
//   - вычисляет «pinch» (distance thumb-tip ↔ index-tip) и «grip»
//     (среднее расстояние 4 tips ↔ wrist),
//   - edge-trigger → отправляет WSS `hand_pinch` / `hand_grip` JSON_CMD,
//   - дополнительно выставляет pressed-уровень для FSM-агрегации
//     (pinch ↔ deadman=false для emergency stop, grip ↔ deadman=true
//     для drive).
//
// Что НЕ делает этот модуль:
//   - рендер скелета — это hand_visual.ts (XRHandModelFactory),
//   - маппинг жестов на twist/safety — это FSM/Connection,
//     здесь мы только шлём raw event.
//   - выбор позы для визуализации — это render loop captain_bridge.ts.
//
// Почему отдельный файл (а не в xr_teleop.ts):
//   - hand-tracking и controller-tracking — разные XR-фичи (WebXR
//     Hand Input Module vs xr-standard). Код разделения чище.
//   - unit-тесты hand_teleop не требуют мока gamepad и наоборот.

import type { HandPinchCmd, HandGripCmd, JsonCmd } from "../wire/messages";

// ------------------------------------------------------------------
// Геометрия жестов — пороги расстояний.
// ------------------------------------------------------------------

/** Pinch = кончик большого + кончик указательного сблизились.
 *  2 см — комфортный pinch в Three.js samples и Quest системе.
 *  См. https://immersive-web.github.io/webxr-hand-input/ §6.1. */
export const PINCH_THRESHOLD_M = 0.02;

/** Grip = кулак. Считаем что все 4 finger-tips ближе этого порога
 *  относительно wrist (проксимальная точка ладони). */
export const GRIP_THRESHOLD_M = 0.05;

// ------------------------------------------------------------------
// Чистая геометрия: расстояние в 3D + edge-detection.
// ------------------------------------------------------------------

/** 3D расстояние между двумя точками (в xr space units, обычно метры). */
export function distance3D(
  a: { x: number; y: number; z: number },
  b: { x: number; y: number; z: number }
): number {
  const dx = a.x - b.x;
  const dy = a.y - b.y;
  const dz = a.z - b.z;
  return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

/** Pinch detection: thumb-tip ↔ index-tip. */
export function isPinching(
  thumbTip: { x: number; y: number; z: number } | null,
  indexTip: { x: number; y: number; z: number } | null,
  threshold = PINCH_THRESHOLD_M
): boolean {
  if (!thumbTip || !indexTip) return false;
  return distance3D(thumbTip, indexTip) < threshold;
}

/** Grip detection: кулак = все 4 finger-tips близко к wrist.
 *  Возвращает true если среднее расстояние < threshold. */
export function isGripping(
  wrist: { x: number; y: number; z: number } | null,
  tips: ReadonlyArray<{ x: number; y: number; z: number } | null>,
  threshold = GRIP_THRESHOLD_M
): boolean {
  if (!wrist) return false;
  let sum = 0;
  let n = 0;
  for (const tip of tips) {
    if (!tip) continue;
    sum += distance3D(wrist, tip);
    n += 1;
  }
  if (n < 4) return false; // если не все 4 пальца трекнулись — не считаем grip
  return sum / n < threshold;
}

/** Edge-detector (level → edge). Возвращает true только на переходе. */
export function detectEdge(prev: boolean, curr: boolean): "rising" | "falling" | null {
  if (curr && !prev) return "rising";
  if (!curr && prev) return "falling";
  return null;
}

// ------------------------------------------------------------------
// Состояние per-hand (живёт между XR-кадрами).
// ------------------------------------------------------------------

export interface HandTeleopState {
  /** Последний «pressed» для pinch (для edge-detect). */
  pinchPressed: boolean;
  /** Последний «pressed» для grip. */
  gripPressed: boolean;
}

export function createHandTeleopState(): HandTeleopState {
  return { pinchPressed: false, gripPressed: false };
}

// ------------------------------------------------------------------
// Результат одного XR-кадра.
// ------------------------------------------------------------------

export interface HandFrameResult {
  /** Pinch начался/закончился в этом кадре. null = без изменений. */
  pinchEdge: "rising" | "falling" | null;
  /** Grip начался/закончился в этом кадре. */
  gripEdge: "rising" | "falling" | null;
  /** Текущие «уровни» (level) — нужны для FSM-агрегации (deadman и пр.). */
  pinching: boolean;
  gripping: boolean;
}

// ------------------------------------------------------------------
// Joint-extractor: достаём (thumb_tip, index_tip, wrist, tips[]) из
// XRFrame+XRHand. Разделён, чтобы тесты могли подсунуть фейковый pose.
// ------------------------------------------------------------------

/** Минимальная проекция XRJointPose, которую мы используем. */
export interface PosePosition {
  x: number;
  y: number;
  z: number;
}

/** Достать 6 ключевых поз из XRHand (или null если joint не трекнулся). */
export interface HandKeyJoints {
  thumbTip: PosePosition | null;
  indexTip: PosePosition | null;
  wrist: PosePosition | null;
  /** index/middle/ring/little tips. */
  fingerTips: [PosePosition | null, PosePosition | null, PosePosition | null, PosePosition | null];
}

/** Тип-контракт XRFrame, который мы используем (узкая проекция для тестов). */
export interface JointFrame {
  getJointPose?: (joint: unknown, baseSpace: unknown) => JointPoseLike | undefined;
}
export interface JointPoseLike {
  transform: { position: PosePosition };
}

/** Достать joint space из XRHand для указанного jointName (или null). */
export function getJointPosition(
  frame: JointFrame,
  hand: { get(key: string): unknown } | null | undefined,
  jointName: string,
  baseSpace: unknown
): PosePosition | null {
  if (!hand || typeof frame.getJointPose !== "function") return null;
  const joint = hand.get(jointName) as unknown;
  if (!joint) return null;
  const pose = frame.getJointPose(joint, baseSpace);
  if (!pose) return null;
  return pose.transform.position;
}

/** Утилита: достать все 6 ключевых joints. */
export function extractKeyJoints(
  frame: JointFrame,
  hand: { get(key: string): unknown } | null | undefined,
  baseSpace: unknown
): HandKeyJoints {
  return {
    thumbTip: getJointPosition(frame, hand, "thumb-tip", baseSpace),
    indexTip: getJointPosition(frame, hand, "index-finger-tip", baseSpace),
    wrist: getJointPosition(frame, hand, "wrist", baseSpace),
    fingerTips: [
      getJointPosition(frame, hand, "index-finger-tip", baseSpace),
      getJointPosition(frame, hand, "middle-finger-tip", baseSpace),
      getJointPosition(frame, hand, "ring-finger-tip", baseSpace),
      getJointPosition(frame, hand, "pinky-finger-tip", baseSpace)
    ]
  };
}

// ------------------------------------------------------------------
// Главный tick: чистая функция — получает joints, отдаёт edges + levels.
// ------------------------------------------------------------------

/** Обновить state на основе поз и вернуть edge-events. Чистая, без I/O. */
export function processHandFrame(
  state: HandTeleopState,
  joints: HandKeyJoints,
  options: { pinchThreshold?: number; gripThreshold?: number } = {}
): HandFrameResult {
  const pinchThr = options.pinchThreshold ?? PINCH_THRESHOLD_M;
  const gripThr = options.gripThreshold ?? GRIP_THRESHOLD_M;

  const pinching = isPinching(joints.thumbTip, joints.indexTip, pinchThr);
  const gripping = isGripping(joints.wrist, joints.fingerTips, gripThr);

  const pinchEdge = detectEdge(state.pinchPressed, pinching);
  const gripEdge = detectEdge(state.gripPressed, gripping);

  state.pinchPressed = pinching;
  state.gripPressed = gripping;

  return { pinchEdge, gripEdge, pinching, gripping };
}

// ------------------------------------------------------------------
// Build WSS commands из HandFrameResult. Edge → шлём cmd; rising=true
// нажат, rising=false/falling=true отпущен.
// ------------------------------------------------------------------

/** Сконвертировать hand frame в JSON_CMD[] для WSS. */
export function handFrameToCommands(
  result: HandFrameResult,
  handedness: "left" | "right",
  tsMs: number
): JsonCmd[] {
  const cmds: JsonCmd[] = [];
  if (result.pinchEdge) {
    const pressed = result.pinchEdge === "rising";
    cmds.push({
      cmd: "hand_pinch",
      ts_ms: tsMs,
      pressed,
      handedness,
      source: "select"
    } as HandPinchCmd);
  }
  if (result.gripEdge) {
    const pressed = result.gripEdge === "rising";
    cmds.push({
      cmd: "hand_grip",
      ts_ms: tsMs,
      pressed,
      handedness,
      source: "squeeze"
    } as HandGripCmd);
  }
  return cmds;
}

// ------------------------------------------------------------------
// High-level handle: жизненный цикл per XR-сессии.
// ------------------------------------------------------------------

export interface HandTeleopHandle {
  /** Сбросить edge-state (например, при disconnect). */
  reset(): void;
  /** true, если в этом кадре хоть одна рука активна. */
  isActive(): boolean;
}

/** Создать per-session handle (без I/O). Caller в XR-кадровом цикле
 *  вызывает process() с фреймом и рукой — handle лишь хранит state. */
export function createHandTeleop(): {
  state: HandTeleopState;
  process(
    frame: JointFrame,
    hand: { get(key: string): unknown } | null | undefined,
    handedness: "left" | "right",
    baseSpace: unknown,
    tsMs: number
  ): { joints: HandKeyJoints; result: HandFrameResult; cmds: JsonCmd[] };
  reset(): void;
  isActive(): boolean;
} {
  const state = createHandTeleopState();
  return {
    state,
    process(frame, hand, handedness, baseSpace, tsMs) {
      const joints = extractKeyJoints(frame, hand, baseSpace);
      const result = processHandFrame(state, joints);
      const cmds = handFrameToCommands(result, handedness, tsMs);
      return { joints, result, cmds };
    },
    reset() {
      state.pinchPressed = false;
      state.gripPressed = false;
    },
    isActive() {
      return state.pinchPressed || state.gripPressed;
    }
  };
}
