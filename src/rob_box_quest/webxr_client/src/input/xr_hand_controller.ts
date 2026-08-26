// XR Hand controller (Phase 2 §3.5):
//
//   - Pinch (index + thumb tip distance < PINCH_DISTANCE_M) → UI click
//     (raycast от index tip).
//   - Среднее сгибание всех пальцев → "grip" → panel drag.
//
// Работает через XRHand API из three/examples/jsm/webxr/XRHand.js.
// Типизирован минимально (чтобы не зависеть от импорта three/examples
// в build-графе — three уже подключён в captain_bridge).
//
// Fallback на контроллеры — ответственность caller: если XRInputSource.handedness
// не hand, вызывающий код полльёт обычный xr_teleop. Здесь мы предоставляем
// только логику жестов для тех XRInputSource, у которых есть XRHand.

export const PINCH_DISTANCE_M = 0.02; // 2 см — дизайн §3.5
export const GRIP_FLEX_THRESHOLD = 0.7; // 0..1, "среднее сгибание"
export const HAND_JOINT_INDEX_TIP = "index-finger-tip";
export const HAND_JOINT_THUMB_TIP = "thumb-tip";

/** Минимальный интерфейс XRHand, который нам нужен (без импорта three/examples). */
export interface XrHandLike {
  handedness: "left" | "right";
  /** Map джойнт → XRJointSpace (или совместимый). */
  joints: Record<string, unknown>;
}

export interface XrHandControllerOptions {
  /** True если pinch сейчас активен (последний кадр). */
  onPinchStart?: (hand: "left" | "right") => void;
  onPinchEnd?: (hand: "left" | "right") => void;
  /** True если "grip" (все пальцы согнуты). */
  onGripStart?: (hand: "left" | "right") => void;
  onGripEnd?: (hand: "left" | "right") => void;
  /** Сообщить текущее состояние (для UI / pinch ray). */
  onUpdate?: (state: HandState) => void;
}

export interface HandState {
  handedness: "left" | "right";
  pinching: boolean;
  gripping: boolean;
  /** Позиция index tip в world-space (если доступна). null если joint не валиден. */
  indexTipPos: { x: number; y: number; z: number } | null;
}

export interface XrHandControllerHandle {
  /** Тик: вызывать на каждом XR frame (renderer.xr.getFrame). */
  update(hand: XrHandLike, frame: XRFrame): void;
  /** Текущее состояние по handedness. */
  getState(handedness: "left" | "right"): HandState | null;
  /** Прекратить работу. */
  destroy(): void;
}

/** Получить позицию джойнта в world-space через XRJointPose. */
function readJointPos(
  joint: unknown,
  frame: XRFrame
): { x: number; y: number; z: number } | null {
  if (!joint || typeof joint !== "object") return null;
  // XRJointSpace extends XRSpace — getPose возвращает XRPose с .transform.
  const space = joint as { getPose?: (ref: XRReferenceSpace, f: XRFrame) => XRPose | undefined };
  if (typeof space.getPose !== "function") return null;
  const ref = (frame.session as unknown as { referenceSpace?: XRReferenceSpace }).referenceSpace
    ?? (typeof globalThis !== "undefined" && "referenceSpace" in frame.session
      ? (frame.session as unknown as { referenceSpace: XRReferenceSpace }).referenceSpace
      : null);
  if (!ref) return null;
  const pose = space.getPose(ref, frame);
  if (!pose) return null;
  const p = pose.transform.position;
  return { x: p.x, y: p.y, z: p.z };
}

/** Грубая оценка "grip": сумма расстояний кончиков пальцев от ладони. */
function estimateGrip(hand: XrHandLike, frame: XRFrame): boolean {
  const wrist = hand.joints["wrist"];
  if (!wrist) return false;
  const wristPos = readJointPos(wrist, frame);
  if (!wristPos) return false;
  const tips = ["thumb-tip", "index-finger-tip", "middle-finger-tip", "ring-finger-tip", "pinky-finger-tip"];
  let sum = 0;
  let n = 0;
  for (const t of tips) {
    const p = readJointPos(hand.joints[t], frame);
    if (!p) continue;
    const d = Math.hypot(p.x - wristPos.x, p.y - wristPos.y, p.z - wristPos.z);
    sum += d;
    n += 1;
  }
  if (n === 0) return false;
  const avgDist = sum / n;
  // При сжатой ладони ~0.05м, при распрямлённой ~0.18м.
  return avgDist < 0.07;
}

/** Позиция джойнта; обёртка над readJointPos. */
function readJoint(
  hand: XrHandLike,
  jointName: string,
  frame: XRFrame
): { x: number; y: number; z: number } | null {
  return readJointPos(hand.joints[jointName], frame);
}

export function createXrHandController(opts: XrHandControllerOptions = {}): XrHandControllerHandle {
  const states = new Map<"left" | "right", HandState>();
  const prev = new Map<"left" | "right", { pinching: boolean; gripping: boolean }>();

  function update(hand: XrHandLike, frame: XRFrame): void {
    const indexPos = readJoint(hand, HAND_JOINT_INDEX_TIP, frame);
    const thumbPos = readJoint(hand, HAND_JOINT_THUMB_TIP, frame);
    let pinching = false;
    if (indexPos && thumbPos) {
      const d = Math.hypot(
        indexPos.x - thumbPos.x,
        indexPos.y - thumbPos.y,
        indexPos.z - thumbPos.z
      );
      pinching = d < PINCH_DISTANCE_M;
    }
    const gripping = estimateGrip(hand, frame);

    const handedness = hand.handedness;
    const next: HandState = {
      handedness,
      pinching,
      gripping,
      indexTipPos: indexPos
    };
    states.set(handedness, next);

    const was = prev.get(handedness) ?? { pinching: false, gripping: false };
    if (pinching && !was.pinching) opts.onPinchStart?.(handedness);
    if (!pinching && was.pinching) opts.onPinchEnd?.(handedness);
    if (gripping && !was.gripping) opts.onGripStart?.(handedness);
    if (!gripping && was.gripping) opts.onGripEnd?.(handedness);
    prev.set(handedness, { pinching, gripping });
    opts.onUpdate?.(next);
  }

  return {
    update,
    getState(handedness): HandState | null {
      return states.get(handedness) ?? null;
    },
    destroy(): void {
      states.clear();
      prev.clear();
    }
  };
}

/**
 * Запросить hand tracking feature у XRSession. Безопасно вызвать до start().
 * Если браузер не поддерживает — тихо вернёт false.
 */
export async function requestHandTrackingFeature(session: XRSession): Promise<boolean> {
  // XR session.requestFeature — нативный API Quest browser.
  // В jsdom / других рантаймах метода нет — fallback gracefully.
  const s = session as unknown as { requestFeature?: (f: string) => Promise<void> };
  if (typeof s.requestFeature !== "function") return false;
  try {
    await s.requestFeature("hand-tracking");
    return true;
  } catch {
    return false;
  }
}