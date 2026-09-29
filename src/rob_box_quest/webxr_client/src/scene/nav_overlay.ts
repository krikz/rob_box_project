// Навигационный слой на полу мостика (issue #3151): путь Nav2, след
// робота, голо-пин цели, прицел и кнопка отмены. Только рисование —
// координаты уже в сцене (= base_link), пересчёт map → сцена делает
// nav/nav_layer.ts через nav/nav_frames.ts.
//
// Высоты (снизу вверх, см. шапку floor_overlay.ts):
//   0.05  карта, 0.07 логотип,
//   0.075 след робота, 0.08 путь Nav2, 0.085 кольцо/стрелка цели,
//   0.4765 луч лидара.
// Как карта и лидар — `depthTest: false` + renderOrder: путь длиннее
// комнаты мостика, стены не должны его обрезать.
//
// Палитра. Карта — голубой holo (0x44ddff), лидар — красный→зелёный.
// Путь и цель — пурпурный (0xff4fd8): третий цвет, не спорящий ни с
// картой, ни со сканом. След — бледно-сиреневый и гаснет к хвосту: он
// вторичен, путь «куда едем» важнее, чем «где были».

import * as THREE from "three";
import type { SceneXz } from "../nav/nav_frames";
import { buildRibbon } from "../nav/ribbon";

export const TRAIL_Y = 0.075;
export const PATH_Y = 0.08;
export const GOAL_Y = 0.085;

const PATH_COLOR = 0xff4fd8;
const TRAIL_COLOR = new THREE.Color(0xc6b8ff);
const PREVIEW_COLOR = 0xffffff;

/** id цели указателя для кнопки отмены (prefix `nav:` — маршрутизация в captain_bridge). */
export const NAV_CANCEL_TARGET_ID = "nav:cancel";

/**
 * Где висит кнопка «ОТМЕНА НАВ»: по центру, сразу за кромкой подиума
 * (радиус 1 м), низко — под нижним краем экрана-стены из глаз оператора
 * (≈ 35° вниз против ≈ 20° у низа видео) и между голо-проекторами
 * (x = ±1, z = −1.8 — ≈ 29° по азимуту, кнопка — ±13°). Раньше стояла на
 * (−0.75, 1.0, −1.25) и налезала на левый проектор.
 */
export const NAV_CANCEL_POS = { x: 0, y: 0.75, z: -1.2 } as const;
export const NAV_CANCEL_SIZE = { width: 0.56, height: 0.14 } as const;
/** Высота глаз оператора — кнопка развёрнута нормалью к ним. */
const NAV_CANCEL_EYE_Y = 1.6;

export interface NavPin {
  point: SceneXz;
  /** Поворот вокруг +Y сцены (nav_frames.mapYawToSceneRotY). */
  rotY: number;
}

export interface TrailVertex extends SceneXz {
  freshness: number;
}

export interface NavOverlayHandle {
  object: THREE.Group;
  /** Кнопка «ОТМЕНА НАВ» — регистрируется в PointerSystem, пока видна. */
  cancelButton: THREE.Mesh;
  setPath(pts: readonly SceneXz[]): void;
  setTrail(pts: readonly TrailVertex[]): void;
  setGoal(pin: NavPin | null): void;
  setPreview(pin: NavPin | null): void;
  setReticle(p: SceneXz | null): void;
  setCancelVisible(visible: boolean): void;
  isCancelVisible(): boolean;
  dispose(): void;
}

function overlayMaterial(color: number, opacity: number, additive = true): THREE.MeshBasicMaterial {
  return new THREE.MeshBasicMaterial({
    color,
    transparent: true,
    opacity,
    depthTest: false,
    depthWrite: false,
    fog: false,
    toneMapped: false,
    side: THREE.DoubleSide,
    blending: additive ? THREE.AdditiveBlending : THREE.NormalBlending
  });
}

function ribbonMesh(material: THREE.Material, renderOrder: number): THREE.Mesh {
  const mesh = new THREE.Mesh(new THREE.BufferGeometry(), material);
  mesh.renderOrder = renderOrder;
  mesh.frustumCulled = false; // путь уходит за пределы комнаты
  mesh.visible = false;
  return mesh;
}

function applyRibbon(mesh: THREE.Mesh, pts: readonly SceneXz[], halfWidth: number, y: number): void {
  const r = buildRibbon(pts, halfWidth, y);
  const g = mesh.geometry as THREE.BufferGeometry;
  g.setAttribute("position", new THREE.BufferAttribute(r.positions, 3));
  g.setIndex(new THREE.BufferAttribute(r.indices, 1));
  g.computeBoundingSphere();
  mesh.visible = r.indices.length > 0;
}

/** Голо-пин: кольцо на полу + стрелка курса + вертикальный луч. */
function createPin(color: number, opacity: number): THREE.Group {
  const pin = new THREE.Group();
  const ring = new THREE.Mesh(new THREE.RingGeometry(0.22, 0.3, 40), overlayMaterial(color, opacity));
  ring.rotation.x = -Math.PI / 2;
  ring.position.y = GOAL_Y;
  ring.renderOrder = 9;
  pin.add(ring);
  // Стрелка «носом» в −Z (вперёд робота) — поворот пина задаёт курс.
  const arrowShape = new THREE.Shape();
  arrowShape.moveTo(0, 0.55);
  arrowShape.lineTo(0.14, 0.3);
  arrowShape.lineTo(-0.14, 0.3);
  arrowShape.closePath();
  const arrow = new THREE.Mesh(new THREE.ShapeGeometry(arrowShape), overlayMaterial(color, opacity));
  arrow.rotation.x = -Math.PI / 2; // локальный +Y фигуры → −Z сцены
  arrow.position.y = GOAL_Y;
  arrow.renderOrder = 9;
  pin.add(arrow);
  const beam = new THREE.Mesh(
    new THREE.CylinderGeometry(0.035, 0.035, 1.4, 12, 1, true),
    overlayMaterial(color, opacity * 0.45)
  );
  beam.position.y = 0.7 + GOAL_Y;
  beam.renderOrder = 9;
  pin.add(beam);
  pin.visible = false;
  return pin;
}

function placePin(pin: THREE.Group, p: NavPin | null): void {
  if (!p) {
    pin.visible = false;
    return;
  }
  pin.position.set(p.point.x, 0, p.point.z);
  pin.rotation.y = p.rotY;
  pin.visible = true;
}

function createCancelButton(): { mesh: THREE.Mesh; texture: THREE.Texture | null } {
  const material = new THREE.MeshBasicMaterial({
    color: 0xffffff,
    transparent: true,
    depthTest: false,
    depthWrite: false,
    fog: false,
    toneMapped: false
  });
  let texture: THREE.Texture | null = null;
  const canvas = typeof document !== "undefined" ? document.createElement("canvas") : null;
  const ctx = canvas?.getContext("2d") ?? null;
  if (canvas && ctx) {
    canvas.width = 512;
    canvas.height = 128;
    ctx.fillStyle = "rgba(90, 10, 60, 0.85)";
    ctx.fillRect(0, 0, 512, 128);
    ctx.strokeStyle = "#ff4fd8";
    ctx.lineWidth = 6;
    ctx.strokeRect(3, 3, 506, 122);
    ctx.fillStyle = "#ffffff";
    ctx.font = "bold 52px monospace";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText("✕ ОТМЕНА НАВ", 256, 66);
    texture = new THREE.CanvasTexture(canvas);
    material.map = texture;
  } else {
    material.color.set(PATH_COLOR);
  }
  const mesh = new THREE.Mesh(new THREE.PlaneGeometry(NAV_CANCEL_SIZE.width, NAV_CANCEL_SIZE.height), material);
  // Как пульт: наклонена к глазам (нормаль → (0, EYE, 0)), читается при
  // взгляде вниз-вперёд на пол, где лежит путь и цель.
  const p = NAV_CANCEL_POS;
  mesh.position.set(p.x, p.y, p.z);
  mesh.rotation.x = -Math.atan2(NAV_CANCEL_EYE_Y - p.y, -p.z);
  mesh.renderOrder = 12;
  mesh.visible = false;
  return { mesh, texture };
}

export function createNavOverlay(): NavOverlayHandle {
  const object = new THREE.Group();
  object.name = "nav_overlay";

  const pathGlow = ribbonMesh(overlayMaterial(PATH_COLOR, 0.28), 7);
  const pathCore = ribbonMesh(overlayMaterial(PATH_COLOR, 0.95), 8);
  const trailMaterial = new THREE.MeshBasicMaterial({
    vertexColors: true,
    transparent: true,
    depthTest: false,
    depthWrite: false,
    fog: false,
    toneMapped: false,
    side: THREE.DoubleSide
  });
  const trail = ribbonMesh(trailMaterial, 6);
  object.add(trail, pathGlow, pathCore);

  const goalPin = createPin(PATH_COLOR, 0.9);
  const previewPin = createPin(PREVIEW_COLOR, 0.6);
  object.add(goalPin, previewPin);

  const reticle = new THREE.Mesh(new THREE.RingGeometry(0.1, 0.14, 32), overlayMaterial(PREVIEW_COLOR, 0.8));
  reticle.rotation.x = -Math.PI / 2;
  reticle.renderOrder = 10;
  reticle.visible = false;
  object.add(reticle);

  const cancel = createCancelButton();
  object.add(cancel.mesh);

  function setTrail(pts: readonly TrailVertex[]): void {
    applyRibbon(trail, pts, 0.03, TRAIL_Y);
    if (!trail.visible) return;
    // RGBA на вершину: альфа = свежесть, две вершины на точку.
    const colors = new Float32Array(pts.length * 2 * 4);
    pts.forEach((p, i) => {
      const a = 0.85 * p.freshness;
      for (let k = 0; k < 2; k += 1) {
        const o = (i * 2 + k) * 4;
        colors[o] = TRAIL_COLOR.r;
        colors[o + 1] = TRAIL_COLOR.g;
        colors[o + 2] = TRAIL_COLOR.b;
        colors[o + 3] = a;
      }
    });
    (trail.geometry as THREE.BufferGeometry).setAttribute("color", new THREE.BufferAttribute(colors, 4));
  }

  function dispose(): void {
    object.traverse((obj) => {
      const mesh = obj as THREE.Mesh;
      if (!mesh.isMesh) return;
      mesh.geometry.dispose();
      (mesh.material as THREE.Material).dispose();
    });
    cancel.texture?.dispose();
  }

  return {
    object,
    cancelButton: cancel.mesh,
    setPath(pts) {
      applyRibbon(pathCore, pts, 0.025, PATH_Y);
      applyRibbon(pathGlow, pts, 0.1, PATH_Y - 0.001);
    },
    setTrail,
    setGoal: (pin) => placePin(goalPin, pin),
    setPreview: (pin) => placePin(previewPin, pin),
    setReticle(p) {
      reticle.visible = p !== null;
      if (p) reticle.position.set(p.x, GOAL_Y, p.z);
    },
    setCancelVisible(visible) {
      cancel.mesh.visible = visible;
    },
    isCancelVisible: () => cancel.mesh.visible,
    dispose
  };
}
