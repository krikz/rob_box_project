// bridge_assets.ts — runtime loader for the Captain Bridge CC0 environment.
//
// Loads 5 GLB assets (floor, walls, props, nav, occluders) and one HDR
// (for IBL on metallic parts) from public/models/environment/, integrates
// them into a Three.js scene, and exposes the parsed nav-points /
// occluders / safe-walk-area metadata for the rest of the app.
//
// Why a separate module (not inlined in captain_bridge.ts):
//   - captain_bridge.ts owns the runtime loop, XR bootstrap, and panel
//     manager; environment loading is a one-shot async setup step that
//     should be testable in isolation.
//   - The loader needs `GLTFLoader + DRACOLoader + MeshoptDecoder` which
//     use Web Workers (`new Worker(...)`); jsdom can't drive that. The
//     module exposes a `loadBridgeMeta()` async that just fetches +
//     JSON-parses the scene-meta file and returns nav-points /
//     occluders — this is unit-testable in jsdom without a Web Worker.
//
// Files referenced (all committed under public/models/environment/):
//   - bridge_floor.optimized.glb        (22 KB)
//   - bridge_walls.optimized.glb        (12 KB)
//   - bridge_props.optimized.glb        (24 KB)
//   - bridge_nav.optimized.glb          ( 8 KB)
//   - bridge_occluders.optimized.glb    ( 4 KB)
//   - bridge_scene_meta.json            ( 5 KB, runtime metadata)
//   - hdr/bridge_env_1k.hdr             (1.6 MB, IBL for metals)
//
// Total GLB payload: 70 KB (vs 2 MB `environment/` budget per ADR-0032
// §3.2; HDR is outside the GLB budget).

import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import { DRACOLoader } from "three/examples/jsm/loaders/DRACOLoader.js";
import { MeshoptDecoder } from "three/examples/jsm/libs/meshopt_decoder.module.js";
import { RGBELoader } from "three/examples/jsm/loaders/RGBELoader.js";

// ---------- public types ----------

export interface BridgeNavPoint {
  id: string;
  position: [number, number, number];
  size: [number, number, number];
  kind: "origin" | "console" | "terminal" | "entry";
  label: string;
}

export interface BridgeOccluder {
  id: string;
  position: [number, number, number];
  size: [number, number, number];
  normal: [number, number, number];
  label: string;
}

export interface BridgeSafeWalkArea {
  min: [number, number, number];
  max: [number, number, number];
  note: string;
}

export interface BridgeDesign {
  style: string;
  palette: Record<string, string>;
  lighting_plan: {
    ambient: { color: string; intensity: number };
    directional: { color: string; intensity: number; position: [number, number, number] };
    ibl: { source: string; format: string; target_ktx2_after_phase_2_0?: string };
  };
  dimensions_m: { width: number; depth: number; height: number };
  ceiling: string;
}

export interface BridgeSceneMeta {
  design: BridgeDesign;
  safe_walk_area: BridgeSafeWalkArea;
  nav_points: BridgeNavPoint[];
  occluders: BridgeOccluder[];
  budgets_raw_bytes: Record<string, number>;
  files: Array<{ name: string; size: number; budget: number; ok: boolean }>;
}

export interface BridgeAssetHandle {
  /** Floor/walls/props/occluders groups, added to the scene. */
  readonly groups: {
    floor?: THREE.Group;
    walls?: THREE.Group;
    props?: THREE.Group;
    occluders?: THREE.Group;
    heroPlatform?: THREE.Group;
    heroScreen?: THREE.Group;
  };
  /** Nav-point AABB meshes (visible in dev, hidden in production). */
  readonly nav: THREE.Group;
  /** Decoded nav-points (parsed from `bridge_scene_meta.json`). */
  readonly navPoints: ReadonlyArray<BridgeNavPoint>;
  /** Decoded occluders (parsed from `bridge_scene_meta.json`). */
  readonly occluders: ReadonlyArray<BridgeOccluder>;
  /** Safe-walk-area AABB. */
  readonly safeWalkArea: BridgeSafeWalkArea;
  /** PMREM-generated environment map (for `scene.environment`). */
  readonly envMap?: THREE.Texture;
  /** Total GLB payload in bytes (sum of optimized sizes). */
  readonly payloadBytes: number;
  /** Set all nav markers visible or hidden (debug toggle). */
  setNavMarkersVisible(visible: boolean): void;
  /** Set all occluder planes visible or hidden (debug toggle). */
  setOccludersVisible(visible: boolean): void;
  /** Dispose geometries / materials / textures. */
  dispose(): void;
}

export interface LoadBridgeAssetsOptions {
  /** Base URL for the models directory, defaults to `/models/environment/`. */
  baseUrl?: string;
  /** Whether to load HDR for IBL (default true; pass false in unit tests). */
  loadHdr?: boolean;
  /** Custom fetch impl (used by tests to return fixtures from disk). */
  fetchImpl?: typeof fetch;
}

const DEFAULT_BASE_URL = "/models/environment/";
const GLB_NAMES = [
  "bridge_floor.optimized.glb",
  "bridge_walls.optimized.glb",
  "bridge_props.optimized.glb",
  "bridge_nav.optimized.glb",
  "bridge_occluders.optimized.glb",
] as const;

// Tripo3D hero props (2026-09-26) — loaded separately from the procedural
// environment; positioned by captain_bridge.ts (unknown baked scale).
const HERO_GLB_NAMES = [
  "bridge_platform.optimized.glb",
  "bridge_screen.optimized.glb",
] as const;

// ---------- meta loader (testable in jsdom) ----------

/**
 * Fetch and parse `bridge_scene_meta.json`. Returns the decoded metadata
 * or throws on network/parse error. Pure function over (fetch, url),
 * so it can be unit-tested with a fake fetch.
 */
export async function loadBridgeMeta(
  baseUrl: string = DEFAULT_BASE_URL,
  fetchImpl: typeof fetch = (typeof fetch !== "undefined" ? fetch : (() => { throw new Error("fetch unavailable"); }) as typeof fetch),
): Promise<BridgeSceneMeta> {
  const url = `${baseUrl}bridge_scene_meta.json`.replace(/\/+/g, "/");
  const res = await fetchImpl(url);
  if (!res.ok) {
    throw new Error(`bridge_scene_meta.json: HTTP ${res.status} ${res.statusText}`);
  }
  const json = await res.json();
  return validateBridgeMeta(json);
}

/**
 * Validate the shape of `bridge_scene_meta.json`. Throws on missing keys
 * or wrong types. Cheap structural check; we trust per-file sizes from
 * `files` rather than re-stat'ing the GLB.
 */
export function validateBridgeMeta(raw: unknown): BridgeSceneMeta {
  if (!raw || typeof raw !== "object") {
    throw new Error("bridge_scene_meta: not an object");
  }
  const r = raw as Record<string, unknown>;
  if (!r.design || typeof r.design !== "object") throw new Error("bridge_scene_meta: missing 'design'");
  if (!r.safe_walk_area || typeof r.safe_walk_area !== "object") {
    throw new Error("bridge_scene_meta: missing 'safe_walk_area'");
  }
  const swa = r.safe_walk_area as Record<string, unknown>;
  if (!Array.isArray(swa.min) || swa.min.length !== 3) throw new Error("bridge_scene_meta: safe_walk_area.min must be [x,y,z]");
  if (!Array.isArray(swa.max) || swa.max.length !== 3) throw new Error("bridge_scene_meta: safe_walk_area.max must be [x,y,z]");
  if (!Array.isArray(r.nav_points)) throw new Error("bridge_scene_meta: 'nav_points' must be an array");
  if (!Array.isArray(r.occluders)) throw new Error("bridge_scene_meta: 'occluders' must be an array");

  for (const np of r.nav_points as unknown[]) {
    const n = np as Record<string, unknown>;
    if (typeof n.id !== "string") throw new Error("bridge_scene_meta: nav_point.id must be string");
    if (!Array.isArray(n.position) || n.position.length !== 3) {
      throw new Error(`bridge_scene_meta: nav_point.${n.id as string}.position must be [x,y,z]`);
    }
    if (!Array.isArray(n.size) || n.size.length !== 3) {
      throw new Error(`bridge_scene_meta: nav_point.${n.id as string}.size must be [x,y,z]`);
    }
    if (typeof n.kind !== "string") {
      throw new Error(`bridge_scene_meta: nav_point.${n.id as string}.kind must be string`);
    }
  }
  for (const oc of r.occluders as unknown[]) {
    const o = oc as Record<string, unknown>;
    if (typeof o.id !== "string") throw new Error("bridge_scene_meta: occluder.id must be string");
    if (!Array.isArray(o.position) || o.position.length !== 3) {
      throw new Error(`bridge_scene_meta: occluder.${o.id as string}.position must be [x,y,z]`);
    }
  }
  return r as unknown as BridgeSceneMeta;
}

// ---------- hero-prop placement (pure, jsdom-testable) ----------

/** Target size (metres) to fit a group's bounding box into, per axis. */
export interface FitTarget {
  width?: number; // X
  height?: number; // Y
  depth?: number; // Z
}

/**
 * Uniform scale factor that fits `box`'s size to the given `target` axis
 * (or axes; the last one with a positive target size wins — see #3044).
 * Falls back to `1` (no scaling) for an empty/degenerate box instead of
 * producing `NaN`/`Infinity` (#3047) — an empty GLB stays at its authored
 * scale rather than silently vanishing or exploding.
 */
export function computeFitScale(box: THREE.Box3, target: FitTarget): number {
  if (box.isEmpty()) return 1;
  const size = box.getSize(new THREE.Vector3());
  let scale = 1;
  if (target.width && size.x > 0) scale = target.width / size.x;
  if (target.height && size.y > 0) scale = target.height / size.y;
  if (target.depth && size.z > 0) scale = target.depth / size.z;
  return scale;
}

/**
 * Places `group` on the floor (`y = 0`) at `(x, z)`, uniformly scaled to
 * fit `target`. Shared by all floor-standing hero props (platform) — previously each call site recomputed the bbox/scale
 * by hand (#3044).
 *
 * Guards against an empty/degenerate bounding box: `box.min.y` is
 * `+Infinity` for `Box3.makeEmpty()`, which would otherwise place the
 * group at `y = -Infinity` (#3047). Such a group is left at `y = 0`.
 */
export function placeOnFloor(group: THREE.Object3D, x: number, z: number, target: FitTarget = {}): void {
  const box = new THREE.Box3().setFromObject(group);
  const scale = computeFitScale(box, target);
  group.scale.setScalar(scale);
  const floorY = box.isEmpty() || !Number.isFinite(box.min.y) ? 0 : -box.min.y * scale;
  group.position.set(x, floorY, z);
}

// ---------- screen-frame fit (bridge_screen.optimized.glb) ----------

/**
 * Где у модели рамки экрана «экранное полотно» — в долях её bounding box.
 *
 * Замерено по геометрии `bridge_screen.optimized.glb` (Tripo3D, 2026-09-26;
 * gltf-transform + meshopt, треугольники с нормалью +Z, сгруппированные по
 * глубине). Модель — плоская «плита» 0.470 × 0.618 × 0.131 (ПОРТРЕТ, origin
 * внизу по центру, лицом +Z):
 *   - полотно (display face): z ≈ 0.130, x ∈ [−0.218, 0.218], y ∈ [0.040, 0.578];
 *   - безель-кант: z ≈ 0.135 (на 5 мм впереди полотна), x ∈ ±0.233, y ∈ [0.006, 0.613];
 *   - трапециевидные «ушки» сверху/снизу по центру — утоплены (z ≈ 0.115).
 * bbox: x ∈ [−0.2351, 0.2352], y ∈ [0, 0.6184], z ∈ [0.0050, 0.1364].
 *
 * Раньше рамку масштабировали равномерно «по ширине 4.8 м» — ×10.2: плита
 * вырастала в 6.3 м высотой и 1.3 м толщиной, вставала origin'ом (низом) на
 * y = 1.5 и выпирала на 1.3 м К ОПЕРАТОРУ, закрывая верхнюю половину видео
 * нижней перекладиной. Теперь масштаб по осям раздельный: полотно = прямо-
 * угольник видео, толщина — отдельно (см. `fitScreenFrame`).
 */
export interface ScreenFaceFractions {
  /** Полотно по X: [u0, u1] в долях ширины bbox (0 = min.x). */
  u0: number;
  u1: number;
  /** Полотно по Y: [v0, v1] в долях высоты bbox (0 = min.y). */
  v0: number;
  v1: number;
}

export const BRIDGE_SCREEN_FACE: Readonly<ScreenFaceFractions> = Object.freeze({
  u0: (0.2351 - 0.218) / 0.4703,
  u1: (0.2351 + 0.218) / 0.4703,
  v0: 0.04 / 0.6184,
  v1: 0.578 / 0.6184,
});

export interface ScreenFrameTarget {
  /** Центр видео-прямоугольника (мир), плоскость смотрит на +Z. */
  center: { x: number; y: number; z: number };
  width: number;
  height: number;
  /** Толщина рамки целиком, м. */
  depth: number;
  /** Зазор между передней точкой рамки и плоскостью видео, м (> 0). */
  gap: number;
  /**
   * Полотно рамки меньше видео на эту долю (0.01 = 1 %): край видео
   * заходит на кант, и под углом не видно щели между видео и безелем.
   */
  overlap?: number;
}

export interface ScreenFrameFit {
  scale: { x: number; y: number; z: number };
  position: { x: number; y: number; z: number };
}

/**
 * Посадка рамки экрана «вокруг» видео-панели, лицом к оператору (+Z):
 * полотно рамки ровно под видео-прямоугольником (центры совпадают), вся
 * геометрия рамки — ЗА плоскостью видео (между оператором и видео ничего).
 *
 * `box` — bbox группы в её собственных координатах при scale = 1,
 * position = 0 (как её отдаёт GLTFLoader). Масштаб по осям раздельный:
 * пропорции модели (портрет 0.81) не совпадают с 16:9, а кант тонкий —
 * неравномерное растяжение даёт безель ~0.17 м по бокам и ~0.2 м сверху/снизу.
 * Пустой/вырожденный bbox → `null` (рамку не трогаем, как computeFitScale → 1).
 */
export function fitScreenFrame(
  box: THREE.Box3,
  face: ScreenFaceFractions,
  target: ScreenFrameTarget,
): ScreenFrameFit | null {
  if (box.isEmpty()) return null;
  const size = box.getSize(new THREE.Vector3());
  const faceW = (face.u1 - face.u0) * size.x;
  const faceH = (face.v1 - face.v0) * size.y;
  if (!(faceW > 0) || !(faceH > 0) || !(size.z > 0)) return null;
  const k = 1 - (target.overlap ?? 0.01);
  const sx = (target.width * k) / faceW;
  const sy = (target.height * k) / faceH;
  const sz = target.depth / size.z;
  const faceCx = box.min.x + ((face.u0 + face.u1) / 2) * size.x;
  const faceCy = box.min.y + ((face.v0 + face.v1) / 2) * size.y;
  return {
    scale: { x: sx, y: sy, z: sz },
    position: {
      x: target.center.x - sx * faceCx,
      y: target.center.y - sy * faceCy,
      // Передняя точка рамки (box.max.z) — на `gap` позади видео.
      z: target.center.z - target.gap - sz * box.max.z,
    },
  };
}

// ---------- GLB / HDR loader (browser-only; needs Web Worker for Draco) ----------

/**
 * Load all 5 GLB + (optionally) HDR into the given scene. Returns a
 * `BridgeAssetHandle` exposing nav-points / occluders / safe-walk-area.
 *
 * NOTE: requires a real browser (or jsdom + a Worker polyfill) because
 * DRACOLoader spawns a Web Worker internally.
 */
export async function loadBridgeAssets(
  scene: THREE.Scene,
  renderer: THREE.WebGLRenderer,
  opts: LoadBridgeAssetsOptions = {},
): Promise<BridgeAssetHandle> {
  const baseUrl = opts.baseUrl ?? DEFAULT_BASE_URL;
  const loadHdr = opts.loadHdr ?? true;

  // GLTFLoader with Draco + Meshopt.
  const draco = new DRACOLoader();
  // Local decoder served from public/draco/ (copied from
  // node_modules/three/examples/jsm/libs/draco/ — no CDN, per vite.config.ts).
  draco.setDecoderPath("/draco/");
  const gltfLoader = new GLTFLoader();
  gltfLoader.setDRACOLoader(draco);
  gltfLoader.setMeshoptDecoder(MeshoptDecoder);

  const meta = await loadBridgeMeta(baseUrl, opts.fetchImpl);
  const groups: BridgeAssetHandle["groups"] = {};

  let payloadBytes = 0;

  async function loadGlb(name: string): Promise<THREE.Group> {
    const url = `${baseUrl}${name}`.replace(/\/+/g, "/");
    const gltf = await new Promise<{ scene: THREE.Group }>((resolve, reject) => {
      gltfLoader.load(
        url,
        (g) => resolve(g),
        undefined,
        (err) => reject(err instanceof Error ? err : new Error(String(err))),
      );
    });
    payloadBytes += gltf.scene.userData?.payloadBytes ?? 0; // not populated, but reserved
    return gltf.scene;
  }

  // Floor + walls + props are visible environment meshes.
  for (const [file, slot] of [
    [GLB_NAMES[0], "floor"],
    [GLB_NAMES[1], "walls"],
    [GLB_NAMES[2], "props"],
  ] as const) {
    const g = await loadGlb(file);
    scene.add(g);
    groups[slot as "floor" | "walls" | "props"] = g;
  }

  // Nav markers — visible in dev, hidden in production via opacity toggle.
  const navGroup = await loadGlb(GLB_NAMES[3]);
  navGroup.visible = false; // default hidden; toggle via setNavMarkersVisible(true) in dev
  scene.add(navGroup);

  // Occluders — semi-transparent planes coincident with walls. In
  // production their material.colorWrite should be set to false so they
  // only write depth and don't tint the color buffer. We default to
  // visible-faint so dev can see them.
  const occludersGroup = await loadGlb(GLB_NAMES[4]);
  scene.add(occludersGroup);
  groups.occluders = occludersGroup;

  // Hero props (Tripo3D) — added to the scene, positioned later by
  // captain_bridge.ts (their baked scale/origin are unknown at load time).
  const heroPlatform = await loadGlb(HERO_GLB_NAMES[0]);
  const heroScreen = await loadGlb(HERO_GLB_NAMES[1]);
  scene.add(heroPlatform);
  scene.add(heroScreen);
  groups.heroPlatform = heroPlatform;
  groups.heroScreen = heroScreen;

  // HDR → PMREM environment map. Only if renderer supports PMREM and the
  // caller asked for it (tests pass loadHdr=false to skip the fetch).
  let envMap: THREE.Texture | undefined;
  if (loadHdr) {
    const rgbe = new RGBELoader();
    const hdrUrl = `${baseUrl}hdr/bridge_env_1k.hdr`.replace(/\/+/g, "/");
    const hdrData: THREE.DataTexture = await new Promise((resolve, reject) => {
      rgbe.load(
        hdrUrl,
        (t) => resolve(t),
        undefined,
        (err) => reject(err instanceof Error ? err : new Error(String(err))),
      );
    });
    const pmrem = new THREE.PMREMGenerator(renderer);
    pmrem.compileEquirectangularShader();
    const envRT = pmrem.fromEquirectangular(hdrData);
    envMap = envRT.texture;
    hdrData.dispose();
    pmrem.dispose();
    // IBL отключён: яркий HDR на металлических стенах/консоли
    // (metalness 0.4–0.7) засвечивает их в «белую стену» — стены
    // отражают яркий горизонт HDR. Оставляем только ambient +
    // directional свет. envMap возвращается в handle для будущего
    // использования (напр. с environmentIntensity ≪ 1).
    // scene.background is intentionally NOT set: bridge walls/viewports
    // give the dark interior look; HDR is for reflection only.
  }

  // Restore nav-marker geometry `transparent` opacity default (in case
  // the GLTF export kept it at 0.6) so toggling visibility flips it
  // cleanly. We mutate each mesh's material.opacity through the group.
  function applyOpacity(group: THREE.Group | undefined, opacity: number): void {
    if (!group) return;
    group.traverse((obj) => {
      const mesh = obj as THREE.Mesh;
      const mat = mesh.material as THREE.Material | undefined;
      if (mat && "transparent" in mat && "opacity" in mat) {
        const m = mat as THREE.MeshStandardMaterial;
        m.transparent = true;
        m.opacity = opacity;
        m.depthWrite = opacity >= 0.99;
        m.needsUpdate = true;
      }
    });
  }
  applyOpacity(navGroup, 0); // start hidden
  applyOpacity(occludersGroup, 0); // start fully transparent (depthWrite=true keeps wall culling)

  return {
    groups,
    nav: navGroup,
    navPoints: meta.nav_points,
    occluders: meta.occluders,
    safeWalkArea: meta.safe_walk_area,
    envMap,
    payloadBytes,
    setNavMarkersVisible(visible: boolean): void {
      applyOpacity(navGroup, visible ? 0.6 : 0);
    },
    setOccludersVisible(visible: boolean): void {
      applyOpacity(occludersGroup, visible ? 0.15 : 0);
    },
    dispose(): void {
      for (const g of Object.values(groups)) {
        if (!g) continue;
        g.traverse((obj) => {
          const mesh = obj as THREE.Mesh;
          mesh.geometry?.dispose?.();
          const mat = mesh.material as THREE.Material | undefined;
          if (mat && "dispose" in mat && typeof mat.dispose === "function") mat.dispose();
        });
      }
      navGroup.traverse((obj) => {
        const mesh = obj as THREE.Mesh;
        mesh.geometry?.dispose?.();
        const mat = mesh.material as THREE.Material | undefined;
        if (mat && "dispose" in mat && typeof mat.dispose === "function") mat.dispose();
      });
      envMap?.dispose?.();
      draco.dispose();
    },
  };
}