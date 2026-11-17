// Оболочка мостика: боковые и задняя стены, потолок, шапка над экраном-стеной.
//
// Зачем. Из GLB-окружения (bridge_walls.optimized.glb) на сцене одна-
// единственная передняя стена — по бокам и сзади оператор видел сетку пола,
// уходящую в черноту. Мостик должен читаться как кабина: стены с рёбрами,
// cyan-подсветка по плинтусу и под потолком, иллюминаторы сзади.
//
// Как. Всё процедурно, без текстур и новых GLB: коробки с цветом на
// вершину, слитые в ДВА меша — «металл» (MeshStandardMaterial, свет сцены)
// и «свечение» (MeshBasicMaterial, полосы и иллюминаторы). Два draw call на
// всю оболочку — Quest этого не заметит.
//
// Семантика слоёв не трогается: карта, лидар и путь Nav2 рисуются без
// depth-теста (floor_overlay.ts / lidar_overlay.ts) и продолжают читаться
// поверх стен. Оболочка стоит строго за пределами рабочего объёма
// оператора: стены — на границе комнаты (|x| = 5.8, z = +4.56), всё, что
// ближе, — выше 3 м (шапка, потолок). Это проверяет tests/bridge_shell.test.ts.
//
// Размер комнаты — из bridge_scene_meta.json (11.6 × 9.12, стена-GLB 3 м).
// Потолок оболочки выше GLB-стены (3.8 м): над рамкой экрана-стены (верх
// ≈ 3.04 м) нужна «шапка» под HUD-полосу — раньше HUD торчал выше потолка.

import * as THREE from "three";
import { mergeGeometries } from "three/examples/jsm/utils/BufferGeometryUtils.js";

/** Полуширина комнаты (x), м — ROOM_W 11.6 / 2. */
export const SHELL_HALF_W = 5.8;
/** Полуглубина комнаты (z), м — ROOM_D 9.12 / 2. */
export const SHELL_HALF_D = 4.56;
/** Высота потолка оболочки, м. */
export const SHELL_HEIGHT = 3.8;
/** Высота передней стены из GLB (над ней оболочка ставит шапку). */
export const FRONT_WALL_GLB_HEIGHT = 3.0;

/** Палитра мостика (docs/plans/2026-09-26-captain-bridge-tripo3d-assets.md). */
export const SHELL_COLORS = {
  wallBase: 0x181c25,
  panel: 0x232a34,
  panelLow: 0x1d232c,
  rib: 0x2c333e,
  ceiling: 0x0f1319,
  bezel: 0x1a2029,
  cyan: 0x2ec27e,
  holo: 0x44ddff,
  viewportTop: 0x040a10,
  viewportBottom: 0x0c2433
} as const;

export type ShellLayer = "solid" | "glow";

/** Коробка оболочки. Чистые данные — тестируются без WebGL. */
export interface ShellBox {
  center: { x: number; y: number; z: number };
  size: { x: number; y: number; z: number };
  /** Поворот вокруг +Y (локальный X коробки = вдоль стены). */
  rotY: number;
  /** Цвет низа и верха (одинаковые — сплошной цвет). */
  color: number;
  colorTop?: number;
  layer: ShellLayer;
}

/** Прямоугольник под HUD-полосой (подложка-«шапка» над экраном-стеной). */
export interface HudBezelRect {
  center: { x: number; y: number; z: number };
  width: number;
  height: number;
}

export interface BridgeShellOptions {
  hudBezel?: HudBezelRect | null;
}

function scaleColor(hex: number, k: number): number {
  const c = new THREE.Color(hex).multiplyScalar(k);
  return c.getHex();
}

interface WallSpec {
  /** Начало и конец внутренней грани стены на полу (x, z). */
  from: { x: number; z: number };
  to: { x: number; z: number };
  /** Иллюминаторы в верхних секциях вместо панелей. */
  viewports: boolean;
}

/**
 * Стена по отрезку. Локальные координаты: u — вдоль стены от `from`,
 * d — внутрь комнаты (к оператору), y — вверх.
 */
function wallBoxes(spec: WallSpec, height: number): ShellBox[] {
  const dx = spec.to.x - spec.from.x;
  const dz = spec.to.z - spec.from.z;
  const len = Math.hypot(dx, dz);
  const dir = { x: dx / len, z: dz / len };
  // Внутренняя нормаль — та из двух перпендикуляров, что смотрит к (0, 0).
  let n = { x: -dir.z, z: dir.x };
  const mid = { x: (spec.from.x + spec.to.x) / 2, z: (spec.from.z + spec.to.z) / 2 };
  if (n.x * -mid.x + n.z * -mid.z < 0) n = { x: -n.x, z: -n.z };
  // rotY: локальный +X коробки → dir.
  const rotY = Math.atan2(-dir.z, dir.x);
  const at = (u: number, d: number, y: number) => ({
    x: spec.from.x + dir.x * u + n.x * d,
    y,
    z: spec.from.z + dir.z * u + n.z * d
  });
  const out: ShellBox[] = [];
  const box = (u: number, d: number, y: number, sx: number, sy: number, sz: number, color: number, layer: ShellLayer, colorTop?: number) =>
    out.push({ center: at(u, d, y), size: { x: sx, y: sy, z: sz }, rotY, color, colorTop, layer });

  // Плита стены — наружу от линии, чтобы внутренняя грань легла ровно на неё.
  box(len / 2, -0.06, height / 2, len, height, 0.12, SHELL_COLORS.wallBase, "solid");

  const bays = Math.max(1, Math.round(len / 1.8));
  const bayW = len / bays;
  for (let i = 0; i <= bays; i += 1) {
    const u = i * bayW;
    // Ребро на всю высоту.
    box(u, 0.09, height / 2, 0.16, height, 0.18, SHELL_COLORS.rib, "solid");
    // Holo-штрих на ребре (крайние рёбра — в углах, штрих там не нужен).
    if (i > 0 && i < bays) {
      box(u, 0.185, 1.9, 0.03, 1.6, 0.01, scaleColor(SHELL_COLORS.holo, 0.45), "glow");
    }
  }
  for (let b = 0; b < bays; b += 1) {
    const u = (b + 0.5) * bayW;
    const w = bayW - 0.3;
    box(u, 0.02, 0.65, w, 0.7, 0.04, SHELL_COLORS.panelLow, "solid");
    if (spec.viewports) {
      // Иллюминатор: тёмное «стекло» с градиентом к горизонту + рамка-панели.
      box(u, 0.01, 2.15, w - 0.2, 1.7, 0.02, SHELL_COLORS.viewportBottom, "glow", SHELL_COLORS.viewportTop);
      box(u, 0.03, 1.22, w, 0.14, 0.06, SHELL_COLORS.panel, "solid");
      box(u, 0.03, 3.08, w, 0.14, 0.06, SHELL_COLORS.panel, "solid");
      box(u, 0.045, 1.3, w - 0.2, 0.012, 0.01, scaleColor(SHELL_COLORS.holo, 0.7), "glow");
    } else {
      box(u, 0.02, 2.2, w, 2.1, 0.04, SHELL_COLORS.panel, "solid");
      // Горизонтальная риска на уровне глаз — даёт масштаб стене.
      box(u, 0.045, 1.6, w * 0.6, 0.012, 0.01, scaleColor(SHELL_COLORS.holo, 0.35), "glow");
    }
  }
  // Плинтус (cyan) и карниз (holo) во всю длину — перед рёбрами.
  box(len / 2, 0.19, 0.12, len, 0.035, 0.02, scaleColor(SHELL_COLORS.cyan, 0.85), "glow");
  box(len / 2, 0.19, height - 0.35, len, 0.03, 0.02, scaleColor(SHELL_COLORS.holo, 0.6), "glow");
  return out;
}

/**
 * Все коробки оболочки. Чистая функция: геометрия и цвета без WebGL —
 * тест проверяет, что ничто не залезает в рабочий объём оператора.
 */
export function bridgeShellBoxes(opts: BridgeShellOptions = {}): ShellBox[] {
  const W = SHELL_HALF_W;
  const D = SHELL_HALF_D;
  const H = SHELL_HEIGHT;
  const out: ShellBox[] = [];

  // Стены: левая, правая (от передней стены до задней) и задняя с
  // иллюминаторами. Передняя — GLB, её не дублируем.
  out.push(...wallBoxes({ from: { x: -W, z: -D }, to: { x: -W, z: D }, viewports: false }, H));
  out.push(...wallBoxes({ from: { x: W, z: D }, to: { x: W, z: -D }, viewports: false }, H));
  out.push(...wallBoxes({ from: { x: W, z: D }, to: { x: -W, z: D }, viewports: true }, H));

  // Шапка над GLB-стеной (3.0 → 3.8 м) + cyan-кант по её низу.
  const headerH = H - FRONT_WALL_GLB_HEIGHT;
  out.push({
    center: { x: 0, y: FRONT_WALL_GLB_HEIGHT + headerH / 2, z: -D - 0.02 },
    size: { x: 2 * W, y: headerH, z: 0.12 },
    rotY: 0,
    color: SHELL_COLORS.wallBase,
    layer: "solid"
  });
  out.push({
    center: { x: 0, y: FRONT_WALL_GLB_HEIGHT + 0.02, z: -D + 0.05 },
    size: { x: 2 * W, y: 0.03, z: 0.02 },
    rotY: 0,
    color: scaleColor(SHELL_COLORS.cyan, 0.85),
    layer: "glow"
  });

  // Потолок: плита + поперечные и продольные балки с holo-полосой снизу.
  out.push({
    center: { x: 0, y: H + 0.05, z: 0 },
    size: { x: 2 * W, y: 0.1, z: 2 * D },
    rotY: 0,
    color: SHELL_COLORS.ceiling,
    layer: "solid"
  });
  for (const z of [-3.3, -1.75, 1.75, 3.3]) {
    out.push({ center: { x: 0, y: H - 0.08, z }, size: { x: 2 * W, y: 0.16, z: 0.22 }, rotY: 0, color: SHELL_COLORS.rib, layer: "solid" });
    out.push({ center: { x: 0, y: H - 0.165, z }, size: { x: 2 * W, y: 0.01, z: 0.035 }, rotY: 0, color: scaleColor(SHELL_COLORS.holo, 0.5), layer: "glow" });
  }
  for (const x of [-2.1, 2.1]) {
    out.push({ center: { x, y: H - 0.1, z: 0 }, size: { x: 0.22, y: 0.12, z: 2 * D }, rotY: 0, color: SHELL_COLORS.rib, layer: "solid" });
  }

  // Подложка HUD-полосы: «шапка» пульта над рамкой экрана-стены.
  const bz = opts.hudBezel;
  if (bz) {
    // Поля 5 см: низ подложки (≈ 3.05) не заходит на верх рамки экрана (≈ 3.04).
    const pad = 0.05;
    const w = bz.width + pad * 2;
    const h = bz.height + pad * 2;
    // Чуть позади спрайтов полосы: спрайты без depth-теста, подложка — нет.
    const z = bz.center.z - 0.12;
    out.push({ center: { x: bz.center.x, y: bz.center.y, z }, size: { x: w, y: h, z: 0.04 }, rotY: 0, color: SHELL_COLORS.bezel, layer: "solid" });
    for (const dy of [-h / 2, h / 2]) {
      out.push({
        center: { x: bz.center.x, y: bz.center.y + dy, z: z + 0.025 },
        size: { x: w, y: 0.014, z: 0.01 },
        rotY: 0,
        color: SHELL_COLORS.cyan,
        layer: "glow"
      });
    }
  }
  return out;
}

export interface BridgeShellHandle {
  object: THREE.Group;
  dispose(): void;
}

function layerGeometry(boxes: ShellBox[]): THREE.BufferGeometry | null {
  if (boxes.length === 0) return null;
  const parts: THREE.BufferGeometry[] = [];
  const m = new THREE.Matrix4();
  const q = new THREE.Quaternion();
  const up = new THREE.Vector3(0, 1, 0);
  const cBottom = new THREE.Color();
  const cTop = new THREE.Color();
  for (const b of boxes) {
    const g = new THREE.BoxGeometry(b.size.x, b.size.y, b.size.z);
    // Цвет на вершину: низ → color, верх → colorTop (градиент иллюминатора).
    cBottom.setHex(b.color);
    cTop.setHex(b.colorTop ?? b.color);
    const pos = g.getAttribute("position");
    const colors = new Float32Array(pos.count * 3);
    for (let i = 0; i < pos.count; i += 1) {
      const c = pos.getY(i) > 0 ? cTop : cBottom;
      colors[i * 3] = c.r;
      colors[i * 3 + 1] = c.g;
      colors[i * 3 + 2] = c.b;
    }
    g.setAttribute("color", new THREE.BufferAttribute(colors, 3));
    g.deleteAttribute("uv");
    q.setFromAxisAngle(up, b.rotY);
    m.compose(new THREE.Vector3(b.center.x, b.center.y, b.center.z), q, new THREE.Vector3(1, 1, 1));
    g.applyMatrix4(m);
    parts.push(g);
  }
  const merged = mergeGeometries(parts, false);
  for (const p of parts) p.dispose();
  return merged;
}

/** Оболочка мостика: два меша (металл + свечение). */
export function createBridgeShell(opts: BridgeShellOptions = {}): BridgeShellHandle {
  const boxes = bridgeShellBoxes(opts);
  const object = new THREE.Group();
  object.name = "bridge_shell";
  const solidMat = new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.8, metalness: 0.25 });
  const glowMat = new THREE.MeshBasicMaterial({ vertexColors: true, toneMapped: false });
  const solidGeom = layerGeometry(boxes.filter((b) => b.layer === "solid"));
  const glowGeom = layerGeometry(boxes.filter((b) => b.layer === "glow"));
  if (solidGeom) {
    const mesh = new THREE.Mesh(solidGeom, solidMat);
    mesh.name = "bridge_shell_solid";
    object.add(mesh);
  }
  if (glowGeom) {
    const mesh = new THREE.Mesh(glowGeom, glowMat);
    mesh.name = "bridge_shell_glow";
    object.add(mesh);
  }
  return {
    object,
    dispose(): void {
      solidGeom?.dispose();
      glowGeom?.dispose();
      solidMat.dispose();
      glowMat.dispose();
    }
  };
}
