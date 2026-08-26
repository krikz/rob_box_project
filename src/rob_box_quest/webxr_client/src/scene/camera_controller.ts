// Camera controller for Captain Bridge (Phase 2 §3.6):
//
//   - Right stick X → yaw (поворот вокруг Y), ±180° (clamp при желании).
//   - Right stick Y → pitch (наклон), ±60° clamp.
//   - A-button rising-edge → reset camera (snap к default 0,1.6,0 facing -Z).
//   - Damping: rotation применяется через lerp (factor 0.15 per frame).
//
// Чистая логика — работает с абстрактным "camera" объектом с методами
// getYaw/getPitch/setYaw/setPitch/setPosition/setLookAt. Это позволяет
// использовать контроллер и в XR (Three.js perspective camera) и в
// desktop fallback (виртуальная камера). Логика без зависимостей от
// Three.js — тестируется в изоляции.

export const DEFAULT_CAMERA_POSITION = { x: 0, y: 1.6, z: 0 } as const;
export const DEFAULT_CAMERA_YAW = 0;
export const DEFAULT_CAMERA_PITCH = 0;
export const PITCH_LIMIT_DEG = 60;
export const YAW_LIMIT_DEG = 180;
export const LERP_FACTOR = 0.15;
export const YAW_RATE_RAD_PER_SEC = 1.5; // when stick at full deflection
export const PITCH_RATE_RAD_PER_SEC = 1.0;

export interface CameraState {
  yaw: number;
  pitch: number;
  position: { x: number; y: number; z: number };
}

export interface CameraControllerOptions {
  /** Initial state. Default: yaw=0, pitch=0, position=DEFAULT_CAMERA_POSITION. */
  initial?: Partial<CameraState>;
  /** Lerp factor 0..1. Default 0.15. */
  lerpFactor?: number;
  /** Optional clock — если не задан, используется performance.now(). */
  now?: () => number;
  /** Limits в радианах. Default ±60° pitch, ±180° yaw. */
  pitchLimitRad?: number;
  yawLimitRad?: number;
  /** Stick → rotation rate (rad/sec at full deflection). */
  yawRate?: number;
  pitchRate?: number;
}

export interface CameraControllerHandle {
  /** Прочитать текущее состояние. */
  getState(): CameraState;
  /** Применить stick axes [-1..1] за dtMs миллисекунд. */
  applyStickAxes(yawStick: number, pitchStick: number, dtMs: number): void;
  /** Snap к default. */
  reset(): void;
  /** Damping tick — сглаживает target→current через lerp(0.15). */
  tickDamping(dtMs: number): void;
  /** Текущие значения target vs current (для тестов). */
  getTarget(): CameraState;
  /** Прямо установить (для тестов / snap). */
  setState(s: Partial<CameraState>): void;
}

function clamp(v: number, lo: number, hi: number): number {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

function lerp(a: number, b: number, t: number): number {
  return a + (b - a) * t;
}

function lerpState(a: CameraState, b: CameraState, t: number): CameraState {
  return {
    yaw: lerp(a.yaw, b.yaw, t),
    pitch: lerp(a.pitch, b.pitch, t),
    position: {
      x: lerp(a.position.x, b.position.x, t),
      y: lerp(a.position.y, b.position.y, t),
      z: lerp(a.position.z, b.position.z, t)
    }
  };
}

export function createCameraController(opts: CameraControllerOptions = {}): CameraControllerHandle {
  const initialYaw = opts.initial?.yaw ?? DEFAULT_CAMERA_YAW;
  const initialPitch = opts.initial?.pitch ?? DEFAULT_CAMERA_PITCH;
  const initialPos = opts.initial?.position ?? { ...DEFAULT_CAMERA_POSITION };

  const lerpFactor = opts.lerpFactor ?? LERP_FACTOR;
  const pitchLimitRad = opts.pitchLimitRad ?? (PITCH_LIMIT_DEG * Math.PI) / 180;
  const yawLimitRad = opts.yawLimitRad ?? (YAW_LIMIT_DEG * Math.PI) / 180;
  const yawRate = opts.yawRate ?? YAW_RATE_RAD_PER_SEC;
  const pitchRate = opts.pitchRate ?? PITCH_RATE_RAD_PER_SEC;

  let current: CameraState = {
    yaw: initialYaw,
    pitch: initialPitch,
    position: { ...initialPos }
  };
  let target: CameraState = {
    yaw: initialYaw,
    pitch: initialPitch,
    position: { ...initialPos }
  };

  function applyStickAxes(yawStick: number, pitchStick: number, dtMs: number): void {
    const dt = dtMs / 1000;
    const dyaw = clamp(yawStick, -1, 1) * yawRate * dt;
    // Stick up (Y=+1) → camera looks up (positive pitch).
    const dpitch = clamp(pitchStick, -1, 1) * pitchRate * dt;
    target = {
      yaw: clamp(target.yaw + dyaw, -yawLimitRad, yawLimitRad),
      pitch: clamp(target.pitch + dpitch, -pitchLimitRad, pitchLimitRad),
      position: current.position // не меняем через стик — это для reset
    };
  }

  function reset(): void {
    target = {
      yaw: DEFAULT_CAMERA_YAW,
      pitch: DEFAULT_CAMERA_PITCH,
      position: { ...DEFAULT_CAMERA_POSITION }
    };
  }

  function tickDamping(dtMs: number): void {
    // dtMs не используется напрямую — lerp factor фиксированный за тик
    // (предполагается 60fps). Если dtMs сильно отличается, можно скорректировать.
    const t = dtMs > 0 ? 1 - Math.exp(-lerpFactor * (dtMs / (1000 / 60))) : lerpFactor;
    current = lerpState(current, target, t);
  }

  function getState(): CameraState {
    return current;
  }

  function getTarget(): CameraState {
    return target;
  }

  function setState(s: Partial<CameraState>): void {
    if (s.yaw !== undefined) target.yaw = clamp(s.yaw, -yawLimitRad, yawLimitRad);
    if (s.pitch !== undefined) target.pitch = clamp(s.pitch, -pitchLimitRad, pitchLimitRad);
    if (s.position) target.position = { ...s.position };
  }

  return {
    getState,
    getTarget,
    applyStickAxes,
    reset,
    tickDamping,
    setState
  };
}

/** Гомогенная матрица lookAt для дефолтной ориентации (facing -Z). */
export function defaultLookAt(): { yaw: number; pitch: number; position: { x: number; y: number; z: number } } {
  return {
    yaw: DEFAULT_CAMERA_YAW,
    pitch: DEFAULT_CAMERA_PITCH,
    position: { ...DEFAULT_CAMERA_POSITION }
  };
}