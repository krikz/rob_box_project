// src/scene/audio_hud.ts
//
// Audio HUD для Captain Bridge: показывает в VR два индикатора голоса:
//
//   1. Иконка микрофона (зелёная idle / красная talk) — слева.
//   2. Audio level meter (waveform-like полоска) — справа, ширина
//      пропорциональна RMS амплитуды последнего PCM-чанка.
//
// Чистая функция `drawAudioHud(ctx, state)` рисует на 2D-canvas; Three.js
// обвязка (sprite + texture) — отдельная фабрика `createAudioHud()`.
// split нужен для unit-тестирования drawFn без webxr/three фикстур.
//
// Состояние:
//   { micState: "idle" | "talk" | "muted"; level: number ∈ [0, 1] }
//   level — нормализованный RMS (0..1). clamp в [0, 1] внутри drawFn.

import * as THREE from "three";

export type MicState = "idle" | "talk" | "muted";

export interface AudioHudState {
  micState: MicState;
  level: number;
}

export interface AudioHudGeometry {
  width: number;
  height: number;
}

export const DEFAULT_AUDIO_HUD_GEOMETRY: AudioHudGeometry = {
  width: 512,
  height: 96
};

const COLORS = {
  bg: "rgba(10, 13, 17, 0.72)",
  micIdle: "#8b98a5",
  micTalk: "#e64545",
  micMuted: "#3a4150",
  meterTrack: "#1f242c",
  meterFill: "#2ec27e",
  meterPeak: "#e8c547",
  text: "#cbd2da",
  textMuted: "#5a6371"
};

/**
 * Нарисовать audio HUD на 2D-контексте. Чистая функция для тестов.
 * `level` клампится в [0, 1]; отрицательные/NaN трактуются как 0.
 */
export function drawAudioHud(
  ctx: CanvasRenderingContext2D,
  state: AudioHudState,
  geom: AudioHudGeometry = DEFAULT_AUDIO_HUD_GEOMETRY
): void {
  const { width: W, height: H } = geom;
  const level = clamp01(state.level);

  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = COLORS.bg;
  ctx.fillRect(0, 0, W, H);

  // ---------- иконка микрофона ----------
  drawMicIcon(ctx, state.micState, H);

  // ---------- level meter ----------
  // Полоса справа от иконки. icon + padding = 96px слева.
  const meterX = 96;
  const meterW = W - meterX - 16;
  const meterH = 16;
  const meterY = (H - meterH) / 2;

  // Track.
  ctx.fillStyle = COLORS.meterTrack;
  ctx.fillRect(meterX, meterY, meterW, meterH);

  // Fill.
  const fillColor =
    state.micState === "muted" ? COLORS.micMuted : COLORS.meterFill;
  ctx.fillStyle = fillColor;
  ctx.fillRect(meterX, meterY, meterW * level, meterH);

  // Peak marker (тонкая вертикальная риска при level > 0.85).
  if (level > 0.85 && state.micState !== "muted") {
    ctx.fillStyle = COLORS.meterPeak;
    ctx.fillRect(meterX + meterW * level - 2, meterY - 4, 4, meterH + 8);
  }

  // ---------- текст справа ----------
  ctx.font = "bold 22px monospace";
  ctx.textBaseline = "middle";
  ctx.fillStyle =
    state.micState === "muted" ? COLORS.textMuted : COLORS.text;
  const label =
    state.micState === "muted"
      ? "MUTED"
      : state.micState === "talk"
        ? "TALK"
        : "IDLE";
  ctx.fillText(label, meterX, H / 2 + 22);
}

function drawMicIcon(
  ctx: CanvasRenderingContext2D,
  state: MicState,
  H: number
): void {
  const cx = 32;
  const cy = H / 2;
  const color =
    state === "talk"
      ? COLORS.micTalk
      : state === "muted"
        ? COLORS.micMuted
        : COLORS.micIdle;

  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = 3;

  // Корпус микрофона (прямоугольник со скруглёнными углами).
  const bodyX = 24;
  const bodyY = cy - 18;
  const bodyW = 16;
  const bodyH = 28;
  const r = 6;
  ctx.beginPath();
  ctx.moveTo(bodyX + r, bodyY);
  ctx.lineTo(bodyX + bodyW - r, bodyY);
  ctx.quadraticCurveTo(bodyX + bodyW, bodyY, bodyX + bodyW, bodyY + r);
  ctx.lineTo(bodyX + bodyW, bodyY + bodyH - r);
  ctx.quadraticCurveTo(
    bodyX + bodyW,
    bodyY + bodyH,
    bodyX + bodyW - r,
    bodyY + bodyH
  );
  ctx.lineTo(bodyX + r, bodyY + bodyH);
  ctx.quadraticCurveTo(bodyX, bodyY + bodyH, bodyX, bodyY + bodyH - r);
  ctx.lineTo(bodyX, bodyY + r);
  ctx.quadraticCurveTo(bodyX, bodyY, bodyX + r, bodyY);
  ctx.closePath();
  ctx.fill();

  // Дуга-держатель под микрофоном.
  ctx.beginPath();
  ctx.arc(cx, cy + 4, 14, Math.PI * 0.2, Math.PI - Math.PI * 0.2);
  ctx.stroke();

  // Ножка.
  ctx.beginPath();
  ctx.moveTo(cx, cy + 18);
  ctx.lineTo(cx, cy + 26);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(cx - 8, cy + 26);
  ctx.lineTo(cx + 8, cy + 26);
  ctx.stroke();

  // Косая черта при muted.
  if (state === "muted") {
    ctx.strokeStyle = COLORS.micTalk;
    ctx.lineWidth = 4;
    ctx.beginPath();
    ctx.moveTo(8, 8);
    ctx.lineTo(56, H - 8);
    ctx.stroke();
  }
}

function clamp01(v: number): number {
  if (Number.isNaN(v) || !Number.isFinite(v)) return 0;
  if (v < 0) return 0;
  if (v > 1) return 1;
  return v;
}

/** RMS-уровень из PCM-Int16 чанка → [0, 1]. */
export function rmsLevel(pcm: Int16Array): number {
  if (pcm.length === 0) return 0;
  let sum = 0;
  for (let i = 0; i < pcm.length; i += 1) {
    const s = pcm[i] / 32768;
    sum += s * s;
  }
  return Math.sqrt(sum / pcm.length);
}

/**
 * Сглаживание уровня: лёгкий attack/release чтобы meter не дёргался.
 * Возвращает новое значение smoothed на основе prev.
 */
export function smoothLevel(
  prev: number,
  target: number,
  attack = 0.6,
  release = 0.15
): number {
  const t = clamp01(target);
  const k = t > prev ? attack : release;
  return prev + (t - prev) * k;
}

// ---------- Three.js обвязка ----------

export interface AudioHudHandle {
  /** Обновить состояние (вызывать из rAF / onChunk). */
  setState(next: AudioHudState): void;
  /** Обновить из PCM-чанка (int16) — рассчитает RMS, обновит level + texture. */
  setLevel(pcm: Int16Array): void;
  setMicState(state: MicState): void;
  /** Three.js sprite для добавления в сцену. */
  sprite: THREE.Sprite;
  dispose(): void;
}

export interface AudioHudOptions {
  /** Позиция в VR-мире (метры). По умолчанию — слева от main screen. */
  position?: { x: number; y: number; z: number };
  /** Масштаб sprite (метры). */
  scale?: { x: number; y: number };
  /** Если true — не использовать CanvasTexture (для unit-тестов). */
  noThree?: boolean;
}

export function createAudioHud(opts: AudioHudOptions = {}): AudioHudHandle {
  const geom = DEFAULT_AUDIO_HUD_GEOMETRY;
  let canvas: HTMLCanvasElement | null = null;
  let ctx: CanvasRenderingContext2D | null = null;
  let texture: THREE.CanvasTexture | null = null;
  let sprite: THREE.Sprite | null = null;

  if (!opts.noThree) {
    canvas = document.createElement("canvas");
    canvas.width = geom.width;
    canvas.height = geom.height;
    ctx = canvas.getContext("2d");
    if (!ctx) throw new Error("audio_hud: failed to acquire 2D context");
    texture = new THREE.CanvasTexture(canvas);
    texture.minFilter = THREE.LinearFilter;
    texture.magFilter = THREE.LinearFilter;
    sprite = new THREE.Sprite(
      new THREE.SpriteMaterial({
        map: texture,
        depthTest: false,
        transparent: true
      })
    );
    sprite.position.set(
      opts.position?.x ?? -2.35,
      opts.position?.y ?? 2.95,
      opts.position?.z ?? -3.85
    );
    sprite.scale.set(opts.scale?.x ?? 1.1, opts.scale?.y ?? 0.2, 1);
  }

  let state: AudioHudState = { micState: "idle", level: 0 };
  let smoothed = 0;

  function redraw(): void {
    if (!ctx || !texture) return;
    drawAudioHud(ctx, state, geom);
    texture.needsUpdate = true;
  }

  redraw();

  return {
    sprite: sprite as unknown as THREE.Sprite,
    setState(next: AudioHudState): void {
      state = { micState: next.micState, level: clamp01(next.level) };
      redraw();
    },
    setLevel(pcm: Int16Array): void {
      const target = rmsLevel(pcm);
      smoothed = smoothLevel(smoothed, target);
      state = { ...state, level: smoothed };
      redraw();
    },
    setMicState(micState: MicState): void {
      state = { ...state, micState };
      redraw();
    },
    dispose(): void {
      texture?.dispose();
      sprite = null;
      ctx = null;
      canvas = null;
      texture = null;
    }
  };
}