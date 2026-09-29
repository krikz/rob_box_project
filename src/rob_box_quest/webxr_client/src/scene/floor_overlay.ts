// Пол мостика: SLAM-карта под ногами + логотип поверх неё в центре.
//
// Что где лежит по высоте (снизу вверх):
//   y ≤ 0.025  bridge_floor.glb — декоративный пол мостика. Это НЕ ноль:
//              у плит есть толщина, мировой bbox пола — y ∈ [−0.025, 0.025];
//   y = 0.05   карта (map_2d, 0x1103) — реальная решётка занятости;
//   y = 0.07   логотип — эмблема на палубе, всегда поверх карты;
//   y = 0.4765 плоскость луча лидара (lidar_overlay.ts).
//
// Зазор до пола выбран по верхней грани GLB, а не «чуть больше нуля»:
// с включённым depth-тестом карта на 0.02 была не видна вообще — плиты
// палубы проходили НАД ней. Сейчас depth-тест выключен (см. ниже) и
// зазор уже не решает, но пусть остаётся честным: обратно включить
// depth-тест не должно значить «и снова всё пропало».
//
// Почему карта рисуется поверх геометрии комнаты (`depthTest: false`) —
// ровно как лидар и по той же причине (см. шапку lidar_overlay.ts).
// Решётка rtabmap — 47×37 м, комната мостика — 7×8. С включённым
// depth-тестом стены обрезают карту по границе комнаты, а лидар
// продолжает рисоваться поверх них: получается, что точки скана висят
// над стеной, а карта, которой они соответствуют, там же обрывается.
// Два слоя об одном и том же не должны спорить друг с другом, поэтому
// карта живёт в том же слое, что и скан, только под ним по renderOrder.
//
// Цена решения та же, что у лидара: смотришь на стену — видишь на её
// нижней части кусок пола, лежащего за ней. Поэтому карта смешивается
// АДДИТИВНО: она только подсвечивает то, что под ней (unknown = alpha 0 —
// ничего, свободно — синеватый отсвет, препятствия — яркий holo), и никогда
// не затемняет. С обычным смешиванием «свободные» клетки (тёмно-синие,
// 105/255) притемняли низ экрана-стены и палубу — карта читалась как грязь,
// а не как голограмма.
//
// Логотип — декорация и живёт иначе: с depth-тестом (подиум и голо-
// проекторы его закрывают), полупрозрачный и позади подиума. Раньше он
// лежал под ногами поверх всего, 3.2 м шириной, и при взгляде вниз
// перекрывал подиум и забивал карту.
//
// Ориентация. Робот стоит в начале координат сцены и смотрит в −Z (туда же
// смотрит фронтальная камера на экране-стене). Карта доворачивается под
// курс робота — см. `mapPlaneTransform` в map_payload.ts.

import * as THREE from "three";
import { parseMapFrame, mapPlaneTransform, type MapFrame } from "./map_payload";

/** Высота плоскости карты: выше верхней грани плит палубы (0.025). */
export const MAP_Y = 0.05;
/** Высота логотипа — над плитами палубы (0.025). */
export const LOGO_Y = 0.07;

/** Ширина логотипа на полу в метрах (высота считается из пропорций SVG). */
export const LOGO_WIDTH_M = 2.6;
/**
 * Где лежит логотип: позади подиума (подиум — диаметр 2 м вокруг (0, 0)),
 * на полпути к задней стене. Повёрнут так, что читается, когда оператор
 * разворачивается к корме. Спереди пол занят картой и путём Nav2.
 */
export const LOGO_CENTER_Z = 2.6;
/** Непрозрачность логотипа: эмблема, а не слой данных. */
export const LOGO_OPACITY = 0.35;
/** Растр SVG по ширине. 2048 px на 3.2 м ≈ 640 px/м — читаемо вблизи. */
const LOGO_RASTER_PX = 2048;

export interface FloorOverlayHandle {
  /** Корень оверлея — добавляется в сцену. */
  object: THREE.Group;
  /**
   * Кадр map_2d (0x1103). Возвращает `false`, если payload битый или в нём
   * нет позы робота (карту тогда некуда класть — она остаётся скрытой).
   */
  ingestMapPayload(payload: Uint8Array): boolean;
  /** Последний разобранный кадр карты (для тестов и HUD). */
  lastMapFrame(): MapFrame | null;
  /** Виден ли сейчас слой карты. */
  isMapVisible(): boolean;
  /**
   * Подгрузить логотип на пол. Ошибка загрузки не фатальна — мостик
   * работает и без эмблемы, поэтому промис резолвится в `false`.
   */
  loadLogo(url?: string): Promise<boolean>;
  dispose(): void;
}

export function createFloorOverlay(): FloorOverlayHandle {
  const object = new THREE.Group();

  // ── Карта ───────────────────────────────────────────────────────────
  // Группа держит поворот под курс робота, плоскость внутри неё — сдвиг
  // центра решётки относительно робота. Разделение не косметическое:
  // поворот и сдвиг заданы в разных кадрах (см. mapPlaneTransform).
  const mapGroup = new THREE.Group();
  const mapMaterial = new THREE.MeshBasicMaterial({
    transparent: true,
    depthTest: false, // поверх геометрии комнаты — см. шапку модуля
    depthWrite: false,
    // Только подсветка, без затемнения — см. шапку модуля.
    blending: THREE.AdditiveBlending,
    fog: false, // туман сцены 6→16 м, карта достаёт дальше
    toneMapped: false,
    side: THREE.DoubleSide,
    visible: false
  });
  const mapMesh = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), mapMaterial);
  mapMesh.rotation.x = -Math.PI / 2;
  mapMesh.renderOrder = 5;
  mapMesh.frustumCulled = false; // плоскость больше комнаты, центр уезжает
  mapGroup.add(mapMesh);
  object.add(mapGroup);

  let mapTexture: THREE.Texture | null = null;
  let lastFrame: MapFrame | null = null;
  let pendingImageUrl: string | null = null;
  let mapGeomSize = { x: 0, z: 0 };

  function applyTransform(frame: MapFrame): boolean {
    const t = mapPlaneTransform(frame);
    if (!t) return false;
    mapGroup.rotation.y = t.groupYaw;
    mapMesh.position.set(t.planeX, MAP_Y, t.planeZ);
    if (t.sizeX !== mapGeomSize.x || t.sizeZ !== mapGeomSize.z) {
      mapMesh.geometry.dispose();
      mapMesh.geometry = new THREE.PlaneGeometry(t.sizeX, t.sizeZ);
      mapGeomSize = { x: t.sizeX, z: t.sizeZ };
    }
    return true;
  }

  function applyPng(png: Uint8Array): void {
    // Drop-oldest: пока предыдущий PNG декодируется, новый не начинаем.
    // Карта приходит раз в несколько секунд, очередь тут не нужна.
    if (pendingImageUrl !== null) return;
    const blob = new Blob([png as BlobPart], { type: "image/png" });
    const url = URL.createObjectURL(blob);
    pendingImageUrl = url;
    const img = new Image();
    img.onload = () => {
      const tex = new THREE.Texture(img);
      tex.colorSpace = THREE.SRGBColorSpace;
      // Решётка — данные, а не фото: NearestFilter показывает клетки как
      // клетки, а не как размытое пятно на масштабе 5 см/клетку.
      tex.magFilter = THREE.NearestFilter;
      tex.minFilter = THREE.LinearMipmapLinearFilter;
      tex.generateMipmaps = true;
      tex.needsUpdate = true;
      mapTexture?.dispose();
      mapTexture = tex;
      mapMaterial.map = tex;
      mapMaterial.visible = true;
      mapMaterial.needsUpdate = true;
      URL.revokeObjectURL(url);
      pendingImageUrl = null;
    };
    img.onerror = () => {
      URL.revokeObjectURL(url);
      pendingImageUrl = null;
    };
    img.src = url;
  }

  function ingestMapPayload(payload: Uint8Array): boolean {
    const frame = parseMapFrame(payload);
    if (!frame) return false;
    lastFrame = frame;
    if (!applyTransform(frame)) {
      // Позы нет — карту показывать некуда.
      mapMaterial.visible = false;
      return false;
    }
    if (frame.png) applyPng(frame.png);
    return true;
  }

  // ── Логотип ─────────────────────────────────────────────────────────
  const logoMaterial = new THREE.MeshBasicMaterial({
    transparent: true,
    opacity: LOGO_OPACITY,
    // С depth-тестом: подиум и голо-проекторы закрывают эмблему, она не
    // просвечивает сквозь них (над плитами палубы она всё равно выше).
    depthTest: true,
    depthWrite: false,
    fog: false,
    toneMapped: false,
    side: THREE.DoubleSide,
    visible: false
  });
  const logoMesh = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), logoMaterial);
  // Лицом вверх и «низом» к корме: читается, когда оператор развернулся назад.
  logoMesh.rotation.set(-Math.PI / 2, 0, Math.PI);
  logoMesh.position.set(0, LOGO_Y, LOGO_CENTER_Z);
  // После карты (renderOrder 5): карта аддитивная и depthWrite не пишет,
  // так что эмблема ложится на палубу, а не под подсветку карты.
  logoMesh.renderOrder = 6;
  object.add(logoMesh);

  let logoTexture: THREE.CanvasTexture | null = null;

  async function loadLogo(url = "logo.svg"): Promise<boolean> {
    try {
      // SVG нельзя отдать в текстуру напрямую — растеризуем через canvas.
      // У logo.svg заданы width/height, поэтому <img> получает его
      // собственные размеры, и drawImage работает без хаков.
      const img = await loadImage(url);
      const aspect = img.height > 0 ? img.width / img.height : 1;
      const canvas = document.createElement("canvas");
      canvas.width = LOGO_RASTER_PX;
      canvas.height = Math.max(1, Math.round(LOGO_RASTER_PX / aspect));
      const ctx = canvas.getContext("2d");
      if (!ctx) return false;
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
      const tex = new THREE.CanvasTexture(canvas);
      tex.colorSpace = THREE.SRGBColorSpace;
      tex.minFilter = THREE.LinearMipmapLinearFilter;
      tex.magFilter = THREE.LinearFilter;
      tex.needsUpdate = true;
      logoTexture?.dispose();
      logoTexture = tex;
      logoMesh.geometry.dispose();
      logoMesh.geometry = new THREE.PlaneGeometry(LOGO_WIDTH_M, LOGO_WIDTH_M / aspect);
      logoMaterial.map = tex;
      logoMaterial.visible = true;
      logoMaterial.needsUpdate = true;
      return true;
    } catch {
      // Логотип — декорация: мостик обязан подняться и без него.
      // eslint-disable-next-line no-console
      console.warn(`[floor_overlay] logo load failed: ${url}`);
      return false;
    }
  }

  function dispose(): void {
    mapMesh.geometry.dispose();
    mapMaterial.dispose();
    mapTexture?.dispose();
    logoMesh.geometry.dispose();
    logoMaterial.dispose();
    logoTexture?.dispose();
    if (pendingImageUrl) URL.revokeObjectURL(pendingImageUrl);
  }

  return {
    object,
    ingestMapPayload,
    lastMapFrame: () => lastFrame,
    isMapVisible: () => mapMaterial.visible,
    loadLogo,
    dispose
  };
}

function loadImage(url: string): Promise<HTMLImageElement> {
  return new Promise((resolve, reject) => {
    const img = new Image();
    img.onload = () => resolve(img);
    img.onerror = () => reject(new Error(`image load failed: ${url}`));
    img.src = url;
  });
}
