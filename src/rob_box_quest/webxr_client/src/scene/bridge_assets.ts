// bridge_assets.ts — runtime loader для Captain Bridge GLB-ассетов и HDR.
//
// Файл загружает 3 .glb + .hdr из public/assets/bridge/ через GLTFLoader +
// RGBELoader. По дизайну §1.1:
//   - bridge_floor.glb   (≤ 120 KB) — hex-grid пол 6×6 метров
//   - bridge_walls.glb   (≤ 180 KB) — back/front/side walls + viewports + console strips
//   - bridge_props.glb   (≤ 250 KB) — captain chair + curved main console +
//                                    4 side terminals + 2 holo-projectors
//   - bridge_env_1k.hdr  (1.63 MB)  — Poly Haven CC0, IBL через PMREMGenerator
//
// Honesty trade-off (см. License.txt): Quaternius/Khronos-паки не автоматизируются
// headless-tools, поэтому ассеты синтезированы из three.js примитивов (CC0, свой код).
// Если Шифу нужны именно Quaternius-модели — заменить .glb после ручного скачивания.
//
// Graceful fallback: если хотя бы один из .glb не загрузился (404 / parse error / CORS),
// возвращаем state="fallback" + объект { fallbackWalls: Mesh[] } — синтетические
// стены из BoxGeometry с grid-текстурой. Сцена не ломается.
//
// Loader — DI-friendly: в тестах можно подсунуть mock-лоадер через opts.gltfLoader /
// opts.hdrLoader / opts.fetcher. По умолчанию — real three.js loaders + fetch().

import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import { DRACOLoader } from "three/examples/jsm/loaders/DRACOLoader.js";
import { RGBELoader } from "three/examples/jsm/loaders/RGBELoader.js";
import { RoomEnvironment } from "three/examples/jsm/environments/RoomEnvironment.js";

export type AssetState = "idle" | "loading" | "ready" | "fallback" | "error";

export interface BridgeAssetManifest {
  floorUrl: string;
  wallsUrl: string;
  propsUrl: string;
  hdrUrl: string;
}

export const DEFAULT_BRIDGE_MANIFEST: BridgeAssetManifest = {
  floorUrl: "/assets/bridge/bridge_floor.glb",
  wallsUrl: "/assets/bridge/bridge_walls.glb",
  propsUrl: "/assets/bridge/bridge_props.glb",
  hdrUrl: "/assets/bridge/bridge_env_1k.hdr",
};

export interface ProgressEvent {
  loaded: number;
  total: number;
  /** имя текущего файла ("bridge_floor.glb" / "bridge_env_1k.hdr" / ...) */
  name: string;
}

export interface BridgeAssetsHandle {
  readonly state: AssetState;
  readonly error: Error | null;
  readonly manifest: BridgeAssetManifest;
  readonly envMap: THREE.Texture | null;
  /** Сумма всех загруженных GLB root-ов, отцентрированных и добавленных к сцене. */
  readonly rootGroup: THREE.Group;
  /** true если loader переключился на fallback (BoxGeometry walls). */
  readonly usingFallback: boolean;
  /** Обновить прогресс UI (lil-gui status bar). */
  onProgress(cb: (e: ProgressEvent) => void): () => void;
  /** Освободить GPU-ресурсы и забыть callback'и. */
  dispose(): void;
}

export interface BridgeAssetsOptions {
  manifest?: Partial<BridgeAssetManifest>;
  /** Injectable GLTFLoader (для тестов). Default: new GLTFLoader() + DRACOLoader. */
  gltfLoader?: GLTFLoader;
  /** Injectable RGBELoader (для тестов). Default: new RGBELoader(). */
  hdrLoader?: RGBELoader;
  /** Injectable fetch-like (для тестов head-count). Default: global fetch(). */
  fetcher?: typeof fetch;
  /** Injectable RoomEnvironment factory (для тестов). */
  roomEnvironmentFactory?: () => RoomEnvironment;
  /** Источник envmap для PMREMGenerator. По умолчанию RoomEnvironment,
   *  если HDR не загрузился — fallback. */
  preferHdrEnv?: boolean;
  renderer?: THREE.WebGLRenderer;
  onError?: (e: Error) => void;
}

export async function initBridgeAssets(
  opts: BridgeAssetsOptions = {}
): Promise<BridgeAssetsHandle> {
  const manifest: BridgeAssetManifest = {
    ...DEFAULT_BRIDGE_MANIFEST,
    ...opts.manifest
  };

  const rootGroup = new THREE.Group();
  rootGroup.name = "bridge-assets";

  let state: AssetState = "loading";
  let error: Error | null = null;
  let envMap: THREE.Texture | null = null;
  let usingFallback = false;

  const progressListeners = new Set<(e: ProgressEvent) => void>();

  function emitProgress(loaded: number, total: number, name: string): void {
    const evt: ProgressEvent = { loaded, total, name };
    for (const cb of progressListeners) cb(evt);
  }

  // ---------- Loaders setup ----------
  const gltfLoader = opts.gltfLoader ?? (() => {
    const l = new GLTFLoader();
    const draco = new DRACOLoader();
    // DRACOLoader decoder path: стандартный three.js CDN. Для offline-Quest
    // build может понадобиться локальный путь — Phase 2.x backlog.
    draco.setDecoderPath("https://www.gstatic.com/draco/versioned/decoders/1.5.6/");
    l.setDRACOLoader(draco);
    return l;
  })();

  const hdrLoader = opts.hdrLoader ?? new RGBELoader();
  const fetcher: typeof fetch = opts.fetcher ?? globalThis.fetch.bind(globalThis);

  // ---------- Head-check: проверить что .glb + .hdr доступны ----------
  // Если хоть один вернёт 404 / network error — переключаемся на fallback
  // для ВСЕХ GLB, не мучаемся с частичной сценой.
  async function headOrFallback(): Promise<"ok" | "fallback"> {
    const urls = [manifest.floorUrl, manifest.wallsUrl, manifest.propsUrl, manifest.hdrUrl];
    for (const url of urls) {
      try {
        const r = await fetcher(url, { method: "HEAD" });
        if (!r.ok) return "fallback";
      } catch {
        return "fallback";
      }
    }
    return "ok";
  }

  const mode = await headOrFallback();

  if (mode === "fallback") {
    // Не выбрасываемся — собираем procedural стены и возвращаемся.
    state = "fallback";
    usingFallback = true;
    const fallback = buildFallbackWalls();
    rootGroup.add(fallback);

    // Envmap из RoomEnvironment (дешёвая fallback-IBL).
    // Только если renderer реальный (WebGL доступен). В headless-тестах
    // (jsdom) WebGLRenderer создать нельзя — пропускаем envmap, оставляем
    // state="fallback" с procedural geometry + ambient освещением.
    if (opts.renderer) {
      try {
        const env = (opts.roomEnvironmentFactory ?? (() => new RoomEnvironment()))();
        const pmrem = new THREE.PMREMGenerator(opts.renderer);
        envMap = pmrem.fromScene(env, 0.04).texture;
        pmrem.dispose();
      } catch {
        envMap = null;
      }
    }

    return makeHandle();
  }

  // ---------- Load 3 GLBs ----------
  let loadedCount = 0;
  const totalCount = 4; // 3 GLB + 1 HDR

  async function loadOne(url: string, name: string): Promise<THREE.Group | null> {
    emitProgress(loadedCount, totalCount, name);
    try {
      const gltf = await gltfLoader.loadAsync(url);
      loadedCount += 1;
      emitProgress(loadedCount, totalCount, name);
      return gltf.scene;
    } catch (e) {
      // Один .glb упал → переходим на fallback для всех.
      return null;
    }
  }

  const [floorScene, wallsScene, propsScene] = await Promise.all([
    loadOne(manifest.floorUrl, "bridge_floor.glb"),
    loadOne(manifest.wallsUrl, "bridge_walls.glb"),
    loadOne(manifest.propsUrl, "bridge_props.glb")
  ]);

  // ---------- Load HDR ----------
  let envTexture: THREE.DataTexture | null = null;
  try {
    envTexture = await hdrLoader.loadAsync(manifest.hdrUrl);
    loadedCount += 1;
    emitProgress(loadedCount, totalCount, "bridge_env_1k.hdr");
  } catch {
    envTexture = null;
  }

  // Если любой из .glb упал → fallback полностью (per design: "не ломать сцену").
  if (!floorScene || !wallsScene || !propsScene) {
    state = "fallback";
    usingFallback = true;
    rootGroup.clear();
    const fallback = buildFallbackWalls();
    rootGroup.add(fallback);
  } else {
    // Все три .glb загружены — собираем в rootGroup с лёгким offset.
    // Оригинал центрирован вокруг (0,0,0), ничего не сдвигаем; наоборот,
    // floor.glb по дизайну начинается с угла — компенсируем через Y-rotation,
    // чтобы совместить с layout полукруга panel_manager'а (пользователь = центр).
    floorScene.rotation.y = Math.PI; // glb смотрит "назад", развернём к пользователю
    rootGroup.add(floorScene, wallsScene, propsScene);
  }

  // ---------- Envmap setup ----------
  if (opts.renderer && envTexture && opts.preferHdrEnv !== false) {
    try {
      const pmrem = new THREE.PMREMGenerator(opts.renderer);
      envMap = pmrem.fromEquirectangular(envTexture).texture;
      pmrem.dispose();
      envTexture.dispose();
    } catch {
      envMap = null;
    }
  } else if (opts.renderer) {
    try {
      const env = (opts.roomEnvironmentFactory ?? (() => new RoomEnvironment()))();
      const pmrem = new THREE.PMREMGenerator(opts.renderer);
      envMap = pmrem.fromScene(env, 0.04).texture;
      pmrem.dispose();
    } catch {
      envMap = null;
    }
  }
  // Если renderer отсутствует (headless) — envMap = null, state="ready",
  // GLB-walls/props всё равно загружены и в сцене.
  // (если state уже "fallback" — оставляем, чтобы UI показал degraded mode)
  if (state !== "fallback") {
    state = "ready";
  }

  function makeHandle(): BridgeAssetsHandle {
    return {
      get state(): AssetState { return state; },
      get error(): Error | null { return error; },
      get manifest(): BridgeAssetManifest { return manifest; },
      get envMap(): THREE.Texture | null { return envMap; },
      get rootGroup(): THREE.Group { return rootGroup; },
      get usingFallback(): boolean { return usingFallback; },
      onProgress(cb: (e: ProgressEvent) => void): () => void {
        progressListeners.add(cb);
        return () => progressListeners.delete(cb);
      },
      dispose(): void {
        progressListeners.clear();
        if (envMap) envMap.dispose();
        rootGroup.traverse((obj) => {
          if (obj instanceof THREE.Mesh) {
            obj.geometry?.dispose?.();
            const mat = obj.material;
            if (Array.isArray(mat)) mat.forEach((m) => m.dispose?.());
            else mat?.dispose?.();
          }
        });
      }
    };
  }

  return makeHandle();
}

/** Procedural fallback: 4 стены (BoxGeometry) + grid-texture floor.
 *  Когда .glb недоступны, не должны показывать пустую сцену — собираем
 *  минимальное sci-fi представление из примитивов. */
export function buildFallbackWalls(): THREE.Group {
  const g = new THREE.Group();
  g.name = "bridge-fallback";

  const wallMat = new THREE.MeshStandardMaterial({
    color: 0x1a2230,
    roughness: 0.85,
    metalness: 0.15,
    emissive: 0x0a0d11,
    emissiveIntensity: 0.4
  });

  const emissiveMat = new THREE.MeshStandardMaterial({
    color: 0x2ec27e,
    emissive: 0x2ec27e,
    emissiveIntensity: 1.0,
    roughness: 0.4,
    metalness: 0.0
  });

  const W = 6; // half-width
  const H = 2.5;
  const D = 5; // half-depth
  const T = 0.1;

  // back wall (-Z) + front viewport (+Z transparent) + left/right side
  const back = new THREE.Mesh(new THREE.BoxGeometry(W * 2, H, T), wallMat);
  back.position.set(0, H / 2, -D);
  g.add(back);

  const front = new THREE.Mesh(new THREE.BoxGeometry(W * 2, H, T), wallMat);
  front.position.set(0, H / 2, D);
  g.add(front);

  const left = new THREE.Mesh(new THREE.BoxGeometry(T, H, D * 2), wallMat);
  left.position.set(-W, H / 2, 0);
  g.add(left);

  const right = new THREE.Mesh(new THREE.BoxGeometry(T, H, D * 2), wallMat);
  right.position.set(W, H / 2, 0);
  g.add(right);

  // console strip с эмиссивными вставками
  const strip = new THREE.Mesh(
    new THREE.BoxGeometry(2.5, 0.3, 0.4),
    emissiveMat
  );
  strip.position.set(0, 1.0, -D + 0.3);
  g.add(strip);

  return g;
}

/** Вспомогательный экспорт: получить envmap из RoomEnvironment без HDR.
 *  Если renderer не передан (headless-тесты / jsdom) — функция возвращает
 *  пустую 1×1 Texture заглушку; production-код всегда передаёт реальный
 *  WebGLRenderer из captain_bridge.ts. */
export function buildRoomEnvMap(renderer?: THREE.WebGLRenderer): THREE.Texture {
  if (!renderer) {
    // 1×1 placeholder — Three.js Texture API требует non-null.
    const data = new Uint8Array([255, 255, 255, 255]);
    return new THREE.DataTexture(data, 1, 1, THREE.RGBAFormat);
  }
  const env = new RoomEnvironment();
  const pmrem = new THREE.PMREMGenerator(renderer);
  const tex = pmrem.fromScene(env, 0.04).texture;
  pmrem.dispose();
  return tex;
}