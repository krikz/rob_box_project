// «Камеры» симулятора (issue #3149): canvas-рендер вида из робота +
// JPEG через canvas. Только браузер: в jsdom canvas-контекста нет, и
// `createCanvasCameraRenderer()` честно вернёт null — мок-робот тогда
// просто не шлёт видеокадры (панели остаются на заглушке).
//
// Вид вперёд — псевдо-3D рейкаст по 2D-плану (как Wolfenstein): для
// каждой колонки кадра собираем ВСЕ пересечения луча с отрезками и рисуем
// их от дальнего к ближнему, поэтому низкий стол не прячет стену за ним.
// Depth — тот же рейкаст в цветовой шкале дальности. Потолочная — вид
// снизу на плитку потолка, повёрнутый по курсу робота.

import { raySegment, type SimRobotState, type SimWorld } from "./sim_world";

export type SimCameraKind = "color" | "depth" | "ceiling";

export interface CameraFrameContext {
  world: SimWorld;
  robot: SimRobotState;
  topic: string;
  kind: SimCameraKind;
  nowMs: number;
  /** Строка режима для HUD (idle / teleop / emergency_stop). */
  mode: string;
}

/** Рендерит кадр и отдаёт JPEG-байты (`null` — не вышло). */
export type CameraRenderer = (ctx: CameraFrameContext) => Promise<Uint8Array | null>;

/** Какой «камерой» симулировать топик. */
export function cameraKindForTopic(topic: string): SimCameraKind | null {
  if (!topic.startsWith("camera_")) return null;
  if (topic === "camera_ceiling") return "ceiling";
  if (topic === "camera_oak_depth") return "depth";
  return "color";
}

const CAM_HEIGHT_M = 0.46; // OAK-D на роботе (rob_box.xacro, z = 0.4595)
const HFOV_COLOR = (69 * Math.PI) / 180;
const HFOV_DEPTH = (72 * Math.PI) / 180;
const DEPTH_MAX_M = 8;

type Ctx2D = CanvasRenderingContext2D | OffscreenCanvasRenderingContext2D;
type AnyCanvas = HTMLCanvasElement | OffscreenCanvas;

function makeCanvas(w: number, h: number): { canvas: AnyCanvas; ctx: Ctx2D } | null {
  try {
    if (typeof OffscreenCanvas !== "undefined") {
      const c = new OffscreenCanvas(w, h);
      const ctx = c.getContext("2d");
      if (ctx) return { canvas: c, ctx };
    }
    if (typeof document !== "undefined") {
      const c = document.createElement("canvas");
      c.width = w;
      c.height = h;
      const ctx = c.getContext("2d");
      if (ctx) return { canvas: c, ctx };
    }
  } catch {
    // нет canvas (jsdom) — ниже null
  }
  return null;
}

async function toJpeg(canvas: AnyCanvas, quality: number): Promise<Uint8Array | null> {
  let blob: Blob | null = null;
  if ("convertToBlob" in canvas) {
    blob = await canvas.convertToBlob({ type: "image/jpeg", quality });
  } else {
    blob = await new Promise<Blob | null>((resolve) =>
      (canvas as HTMLCanvasElement).toBlob(resolve, "image/jpeg", quality)
    );
  }
  if (!blob) return null;
  return new Uint8Array(await blob.arrayBuffer());
}

function rgb(color: number, k: number): string {
  const r = Math.min(255, Math.round(((color >> 16) & 0xff) * k));
  const g = Math.min(255, Math.round(((color >> 8) & 0xff) * k));
  const b = Math.min(255, Math.round((color & 0xff) * k));
  return `rgb(${r},${g},${b})`;
}

/** Шкала «turbo»-подобная для глубины: близко — тёплое, далеко — холодное. */
function depthColor(d: number): string {
  const t = Math.max(0, Math.min(1, d / DEPTH_MAX_M));
  const hue = 10 + t * 230;
  const light = 55 - t * 25;
  return `hsl(${hue.toFixed(0)},90%,${light.toFixed(0)}%)`;
}

interface Hit {
  t: number;
  u: number;
  segIndex: number;
}

function drawForward(c: Ctx2D, w: number, h: number, f: CameraFrameContext): void {
  const { world, robot } = f;
  const depth = f.kind === "depth";
  const hfov = depth ? HFOV_DEPTH : HFOV_COLOR;
  const focal = w / 2 / Math.tan(hfov / 2);
  const horizon = h / 2;

  // Пол и потолок.
  if (depth) {
    c.fillStyle = "#05070a";
    c.fillRect(0, 0, w, h);
    const g = c.createLinearGradient(0, horizon, 0, h);
    g.addColorStop(0, depthColor(DEPTH_MAX_M));
    g.addColorStop(1, depthColor(0.4));
    c.fillStyle = g;
    c.fillRect(0, horizon, w, h - horizon);
  } else {
    const sky = c.createLinearGradient(0, 0, 0, horizon);
    sky.addColorStop(0, "#2a3140");
    sky.addColorStop(1, "#171b23");
    c.fillStyle = sky;
    c.fillRect(0, 0, w, horizon);
    const floor = c.createLinearGradient(0, horizon, 0, h);
    floor.addColorStop(0, "#1a1d22");
    floor.addColorStop(1, "#4a4640");
    c.fillStyle = floor;
    c.fillRect(0, horizon, w, h - horizon);
  }

  const colStep = 2;
  const hits: Hit[] = [];
  for (let x = 0; x < w; x += colStep) {
    const off = Math.atan((x + colStep / 2 - w / 2) / focal);
    const a = robot.yaw - off; // экран вправо = поворот по часовой
    const dx = Math.cos(a);
    const dy = Math.sin(a);
    hits.length = 0;
    world.segments.forEach((s, segIndex) => {
      const r = raySegment(robot.x, robot.y, dx, dy, s);
      if (r && r.t > 0.05) hits.push({ t: r.t, u: r.u, segIndex });
    });
    hits.sort((p, q) => q.t - p.t); // дальние первыми
    for (const hit of hits) {
      const s = world.segments[hit.segIndex];
      const d = hit.t * Math.cos(off); // без «рыбьего глаза»
      const top = horizon - ((s.height - CAM_HEIGHT_M) / d) * focal;
      const bottom = horizon + (CAM_HEIGHT_M / d) * focal;
      if (depth) {
        c.fillStyle = depthColor(d);
      } else {
        // Освещённость грани: направление стены к «окну» на северо-востоке.
        const ex = s.bx - s.ax;
        const ey = s.by - s.ay;
        const len = Math.hypot(ex, ey) || 1;
        const lambert = 0.65 + 0.35 * Math.abs((-ey / len) * 0.6 + (ex / len) * 0.8);
        const fog = Math.max(0.25, 1 - d / 14);
        // Шов панелей каждый метр — даёт параллакс при движении.
        const along = hit.u * len;
        const seam = along % 1 < 0.04 ? 0.6 : 1;
        c.fillStyle = rgb(s.color, lambert * fog * seam);
      }
      c.fillRect(x, top, colStep, bottom - top);
    }
  }
}

function drawCeiling(c: Ctx2D, w: number, h: number, f: CameraFrameContext): void {
  const { world, robot } = f;
  const dist = Math.max(0.5, world.ceilingHeight - CAM_HEIGHT_M);
  const hfov = (70 * Math.PI) / 180;
  const pxPerM = w / 2 / (Math.tan(hfov / 2) * dist);
  c.fillStyle = "#07090c";
  c.fillRect(0, 0, w, h);
  c.save();
  // Камера смотрит вверх, верх кадра — вперёд робота. Снизу мир зеркален:
  // левая сторона робота — слева в кадре, поэтому x карты не отражаем,
  // а ось «вперёд» ведём вверх кадра.
  c.translate(w / 2, h / 2);
  c.scale(pxPerM, pxPerM);
  c.rotate(-(Math.PI / 2 - robot.yaw));
  c.scale(1, -1);
  c.translate(-robot.x, -robot.y);
  // Потолок внутри контура.
  c.beginPath();
  world.outline.forEach(([x, y], i) => (i === 0 ? c.moveTo(x, y) : c.lineTo(x, y)));
  c.closePath();
  c.fillStyle = "#c9ccd1";
  c.fill();
  c.clip();
  // Плитка 0.6 м.
  const { minX, minY, maxX, maxY } = world.bounds;
  c.strokeStyle = "#9aa0a8";
  c.lineWidth = 0.02;
  c.beginPath();
  for (let x = Math.floor(minX / 0.6) * 0.6; x <= maxX; x += 0.6) {
    c.moveTo(x, minY);
    c.lineTo(x, maxY);
  }
  for (let y = Math.floor(minY / 0.6) * 0.6; y <= maxY; y += 0.6) {
    c.moveTo(minX, y);
    c.lineTo(maxX, y);
  }
  c.stroke();
  // Светильники каждые 2.4 м.
  c.fillStyle = "#fff8e0";
  for (let x = Math.floor(minX / 2.4) * 2.4 + 1.2; x <= maxX; x += 2.4) {
    for (let y = Math.floor(minY / 2.4) * 2.4 + 1.2; y <= maxY; y += 2.4) {
      c.fillRect(x - 0.3, y - 0.15, 0.6, 0.3);
    }
  }
  c.restore();
}

function drawHud(c: Ctx2D, w: number, h: number, f: CameraFrameContext): void {
  const { robot } = f;
  // SIM-водяной знак.
  c.save();
  c.globalAlpha = 0.16;
  c.fillStyle = "#ffffff";
  c.font = `bold ${Math.round(h * 0.34)}px sans-serif`;
  c.textAlign = "center";
  c.textBaseline = "middle";
  c.fillText("SIM", w / 2, h / 2);
  c.restore();

  const fs = Math.max(10, Math.round(h / 20));
  c.font = `${fs}px monospace`;
  c.textBaseline = "top";
  c.textAlign = "left";
  c.fillStyle = "rgba(0,0,0,0.55)";
  c.fillRect(0, 0, w, fs * 1.6);
  c.fillRect(0, h - fs * 1.6, w, fs * 1.6);
  c.fillStyle = "#7fffd4";
  const time = new Date(f.nowMs).toISOString().slice(11, 23);
  c.fillText(`● SIM  ${f.topic}`, fs * 0.5, fs * 0.3);
  c.textAlign = "right";
  c.fillText(time, w - fs * 0.5, fs * 0.3);
  c.textAlign = "left";
  const yawDeg = ((robot.yaw * 180) / Math.PI).toFixed(0);
  c.fillStyle = f.mode === "emergency_stop" ? "#ff6b6b" : robot.bumped ? "#ffd24a" : "#cfd8e3";
  const status = f.mode === "emergency_stop" ? "E-STOP" : robot.bumped ? "BUMP" : f.mode;
  c.fillText(
    `x ${robot.x.toFixed(2)}  y ${robot.y.toFixed(2)}  θ ${yawDeg}°  v ${robot.v.toFixed(2)}  ω ${robot.w.toFixed(2)}  ${status}`,
    fs * 0.5,
    h - fs * 1.3
  );
  if (f.kind === "color") {
    // Прицел по центру — где «перед робота».
    c.strokeStyle = "rgba(127,255,212,0.6)";
    c.lineWidth = 1;
    c.beginPath();
    c.moveTo(w / 2 - 10, h / 2);
    c.lineTo(w / 2 + 10, h / 2);
    c.moveTo(w / 2, h / 2 - 10);
    c.lineTo(w / 2, h / 2 + 10);
    c.stroke();
  }
}

const SIZES: Record<SimCameraKind, [number, number]> = {
  color: [640, 360],
  depth: [400, 250],
  ceiling: [400, 300]
};

/**
 * Canvas-рендерер камер. `null`, если canvas недоступен (jsdom/воркер без
 * OffscreenCanvas) — вызывающий обязан это пережить.
 */
export function createCanvasCameraRenderer(): CameraRenderer | null {
  const canvases = new Map<SimCameraKind, { canvas: AnyCanvas; ctx: Ctx2D }>();
  for (const kind of Object.keys(SIZES) as SimCameraKind[]) {
    const [w, h] = SIZES[kind];
    const made = makeCanvas(w, h);
    if (!made) return null;
    canvases.set(kind, made);
  }
  return async (f) => {
    const entry = canvases.get(f.kind);
    if (!entry) return null;
    const [w, h] = SIZES[f.kind];
    const { canvas, ctx } = entry;
    if (f.kind === "ceiling") drawCeiling(ctx, w, h, f);
    else drawForward(ctx, w, h, f);
    drawHud(ctx, w, h, f);
    return toJpeg(canvas, 0.72);
  };
}
