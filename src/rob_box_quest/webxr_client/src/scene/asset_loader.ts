// asset_loader.ts — Phase 2.2 Three.js WebXR client asset pipeline.
//
// A single place that wires the three Three.js example loaders we need to
// load production-grade glTF 2.0 assets:
//   - GLTFLoader               (Three.js examples/jsm/loaders)
//   - DRACOLoader              (decodes KHR_draco_mesh_compression buffers)
//   - KTX2Loader               (transcodes KTX2 / Basis textures on GPU upload)
// Plus MeshoptDecoder for EXT_meshopt_compression (animation + buffer
// views) and THREE.Cache so the same URL is only fetched once per session.
//
// Decoder paths are self-hosted under public/ — no CDN dependency, no
// runtime network hop to a third-party origin (matches the vite.config.ts
// "no CDN" policy already used for the environment module):
//   - Draco decoder → /draco/        (public/draco/draco_decoder.{js,wasm})
//   - Basis transcoder → /basis/     (public/basis/basis_transcoder.{js,wasm})
//
// The runtime path is browser-only: GLTFLoader + DRACOLoader use Web Workers
// internally and KTX2Loader transcode uses WASM. jsdom doesn't ship
// Worker/WASM, so the unit tests assert on the configuration (correct
// paths, MeshoptDecoder attached, THREE.Cache hit/miss) rather than
// actually parsing a Draco-compressed GLB. End-to-end "did it decode?"
// assertions live in tests/gltf_pipeline.test.ts via gltf-transform's
// NodeIO, which uses the same decoders synchronously.
//
// Why a separate module from bridge_assets.ts:
//   bridge_assets.ts is the Captain Bridge environment (5 GLBs + 1 HDR)
//   wired into captain_bridge.ts. asset_loader.ts is the *generic* loader
//   that avatar_loader.ts (Phase 2.2) and any future model category
//   (props, hand-mesh, holo-projection) reuse. Keeping it generic means
//   we don't entangle avatar-loading with environment-specific logic.

import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import { DRACOLoader } from "three/examples/jsm/loaders/DRACOLoader.js";
import { KTX2Loader } from "three/examples/jsm/loaders/KTX2Loader.js";
import { MeshoptDecoder } from "three/examples/jsm/libs/meshopt_decoder.module.js";

// ---------- public types ----------

/** Result of a successful glTF load. */
export interface LoadedAsset {
  /** The parsed scene graph (a THREE.Group; root mesh is the asset itself). */
  readonly scene: THREE.Group;
  /** Animations declared by the asset (TRACKS / Action). Empty array for static meshes. */
  readonly animations: ReadonlyArray<THREE.AnimationClip>;
  /** Decimal asset size in bytes (after THREE.Cache check). */
  readonly payloadBytes: number;
  /** URL the asset was loaded from. */
  readonly url: string;
}

/** DI / test hooks. All optional. */
export interface LoadGlbOptions {
  /**
   * Override the fetch implementation (used by unit tests to inject a
   * mock). When provided, the fetch is invoked *before* GLTFLoader.load
   * to populate THREE.Cache with the bytes — that way the loader sees a
   * cache hit and skips the network round-trip.
   */
  fetchImpl?: typeof fetch;
  /**
   * Override the decoder paths (e.g. `/draco/`, `/basis/`). The defaults
   * match the public/ directory layout for vite-served assets. Tests can
   * pass a different prefix to assert the loader config is variable.
   */
  dracoPath?: string;
  basisPath?: string;
  /**
   * Network-failure retry budget (default 3). Exponential back-off:
   * attempt N waits 100ms · 2^(N-1) before retrying. Set to 0 to disable
   * retry (fail on the first network error).
   */
  retries?: number;
  /**
   * Abort signal for cancellation (unused yet — exposed so hand-tracking
   * and telemetry child tasks can cancel an in-flight avatar load if the
   * session ends mid-download).
   */
  signal?: AbortSignal;
}

// ---------- constants ----------

/** Where the Draco decoder is served from (public/draco/). */
export const DEFAULT_DRACO_PATH = "/draco/";

/** Where the Basis Universal transcoder is served from (public/basis/). */
export const DEFAULT_BASIS_PATH = "/basis/";

const DEFAULT_RETRIES = 3;
const RETRY_BASE_DELAY_MS = 100;

/**
 * Ensure THREE.Cache is enabled. THREE.Cache ships disabled by default
 * (it's a no-op until you opt in). We want the avatar / environment
 * loader to reuse decoded meshes across navigations and XR session
 * re-entries — without this opt-in, `THREE.Cache.add(url, bytes)` is a
 * silent no-op and `THREE.Cache.get(url)` always returns undefined,
 * making every reload trigger a fresh fetch.
 *
 * Idempotent: calling it multiple times is safe.
 */
export function ensureCacheEnabled(): void {
  if (!THREE.Cache.enabled) {
    THREE.Cache.enabled = true;
  }
}

// ---------- factory: build a pre-configured loader set ----------

/**
 * Construct a fresh `(GLTFLoader, DRACOLoader, KTX2Loader)` triple wired
 * together. The DRACOLoader + KTX2Loader are registered as dependencies
 * on the GLTFLoader so any glTF file can use any combination of the
 * three compression extensions without per-call configuration.
 *
 * The returned loaders are *owned* by the caller — call `.dispose()`
 * on the DRACOLoader and KTX2Loader when done. The GLTFLoader has no
 * resources to release.
 */
export function createAssetLoader(opts: { dracoPath?: string; basisPath?: string } = {}): {
  gltf: GLTFLoader;
  draco: DRACOLoader;
  ktx2: KTX2Loader;
} {
  const draco = new DRACOLoader();
  draco.setDecoderPath(opts.dracoPath ?? DEFAULT_DRACO_PATH);
  // Worker pool defaults are fine for our scene (5–10 mesh primitives max);
  // we don't override setWorkerLimit unless we hit decode contention.

  const ktx2 = new KTX2Loader();
  ktx2.setTranscoderPath(opts.basisPath ?? DEFAULT_BASIS_PATH);

  const gltf = new GLTFLoader();
  // Dependency wiring: glTF parser pulls these lazily based on the file's
  // extensionsUsed. Both decoders are stateless w.r.t. the parsed glTF;
  // they only need their static config (decoder/transcoder path).
  gltf.setDRACOLoader(draco);
  gltf.setKTX2Loader(ktx2);
  // MeshoptDecoder is synchronous (no WASM init). attachDecoder tells the
  // parser to call it inline when it sees EXT_meshopt_compression.
  gltf.setMeshoptDecoder(MeshoptDecoder);

  return { gltf, draco, ktx2 };
}

// ---------- core: load one glTF, with retry + cache + progress ----------

/**
 * Load a glTF (.glb) URL into a parsed THREE.Group, with:
 *   - THREE.Cache dedup (second load of the same URL is free)
 *   - automatic retry on transient fetch failures (offline Quest headset,
 *     cold cache, etc.) with exponential back-off
 *   - DRACO + KTX2 + Meshopt decoders pre-wired via createAssetLoader()
 *
 * Returns the parsed group + declared animations. The caller is expected
 * to add `result.scene` to a THREE.Scene themselves (so this function
 * stays composable — the same loader is used by avatar and environment).
 */
export async function loadGlb(
  url: string,
  opts: LoadGlbOptions = {},
  loaderSet: ReturnType<typeof createAssetLoader> = createAssetLoader({ dracoPath: opts.dracoPath, basisPath: opts.basisPath }),
): Promise<LoadedAsset> {
  const fetchImpl = opts.fetchImpl ?? (typeof fetch !== "undefined" ? fetch : undefined);
  const retries = opts.retries ?? DEFAULT_RETRIES;
  const signal = opts.signal;

  // ----- 1. Cache check -----
  // THREE.Cache is keyed by URL; we treat a hit as authoritative. We
  // bypass GLTFLoader.load() (which always fetches via FileLoader,
  // ignoring THREE.Cache) and call GLTFLoader.parse() directly with
  // the cached bytes — that's the only way to get a true zero-fetch
  // path.
  ensureCacheEnabled();
  const cached = THREE.Cache.get(url);
  if (cached !== undefined) {
    return parseFromCache(url, loaderSet.gltf, cached, signal);
  }

  // ----- 2. Fetch (with retry) -----
  let bytes: ArrayBuffer | undefined;
  let lastErr: unknown;
  for (let attempt = 0; attempt <= retries; attempt++) {
    if (signal?.aborted) {
      throw new DOMException("loadGlb: aborted", "AbortError");
    }
    try {
      if (!fetchImpl) {
        throw new Error("loadGlb: no fetch implementation (browser or fetchImpl required)");
      }
      const res = await fetchImpl(url, { signal });
      if (!res.ok) {
        throw new Error(`HTTP ${res.status} ${res.statusText} for ${url}`);
      }
      bytes = await res.arrayBuffer();
      // Populate THREE.Cache so subsequent loads are free. GLTFLoader's
      // own load() path checks this and skips the network round-trip.
      THREE.Cache.add(url, bytes);
      break;
    } catch (err) {
      lastErr = err;
      if (attempt >= retries) break;
      const delayMs = RETRY_BASE_DELAY_MS * 2 ** attempt;
      await sleep(delayMs, signal);
    }
  }
  if (!bytes) {
    const reason = lastErr instanceof Error ? lastErr.message : String(lastErr);
    throw new Error(`loadGlb: failed after ${retries + 1} attempt(s) for ${url}: ${reason}`);
  }

  // ----- 3. Parse via GLTFLoader.parse (already-loaded bytes) -----
  // We deliberately use `parse()` instead of `load()` so the bytes we
  // already fetched (and put in the cache) are reused. This also lets
  // the retry loop own the fetch path, which is what we want to test.
  return new Promise<LoadedAsset>((resolve, reject) => {
    try {
      loaderSet.gltf.parse(
        bytes,
        url,
        (gltf) => {
          const scene = gltf.scene ?? gltf.scenes?.[0];
          if (!scene) {
            reject(new Error(`loadGlb: parser returned no scene for ${url}`));
            return;
          }
          // Stash payload size for tests + telemetry (no longer needed by
          // GLTFLoader, but the existing bridge_assets.ts reserves the
          // slot on userData so we mirror the contract here).
          scene.userData = { ...scene.userData, payloadBytes: bytes!.byteLength, url };
          resolve({
            scene,
            animations: gltf.animations ?? [],
            payloadBytes: bytes!.byteLength,
            url,
          });
        },
        (err) => reject(err instanceof Error ? err : new Error(String(err))),
      );
    } catch (err) {
      reject(err instanceof Error ? err : new Error(String(err)));
    }
  });
}

/**
 * Variant that consumes bytes from THREE.Cache and parses them through
 * GLTFLoader.parse without a network round-trip. Used by the cache-hit
 * branch of `loadGlb()`: when the URL is already in `THREE.Cache`, we
 * don't need to refetch — the cached bytes are the ground truth.
 *
 * (Note: GLTFLoader.load(url) goes through FileLoader and does NOT
 * check THREE.Cache; it always fetches. So we can't reuse `load()` for
 * the cache hit path — we have to call `parse()` directly with the
 * cached ArrayBuffer.)
 */
async function parseFromCache(
  url: string,
  gltf: GLTFLoader,
  bytes: ArrayBuffer,
  signal?: AbortSignal,
): Promise<LoadedAsset> {
  if (signal?.aborted) {
    throw new DOMException("parseFromCache: aborted", "AbortError");
  }
  return await new Promise<LoadedAsset>((resolve, reject) => {
    try {
      gltf.parse(
        bytes,
        url,
        (g) => {
          const scene = g.scene ?? g.scenes?.[0];
          if (!scene) {
            reject(new Error(`parseFromCache: parser returned no scene for ${url}`));
            return;
          }
          scene.userData = { ...scene.userData, payloadBytes: bytes.byteLength, url };
          resolve({
            scene,
            animations: g.animations ?? [],
            payloadBytes: bytes.byteLength,
            url,
          });
        },
        (err) => reject(err instanceof Error ? err : new Error(String(err))),
      );
    } catch (err) {
      reject(err instanceof Error ? err : new Error(String(err)));
    }
  });
}

/** Release decoder worker pools / WASM modules held by the loader triple. */
export function disposeAssetLoader(loaderSet: ReturnType<typeof createAssetLoader>): void {
  loaderSet.draco.dispose();
  loaderSet.ktx2.dispose();
  // GLTFLoader has no resources to release (it's a pure parser façade).
}

// ---------- helpers ----------

function sleep(ms: number, signal?: AbortSignal): Promise<void> {
  return new Promise((resolve, reject) => {
    if (signal?.aborted) {
      reject(new DOMException("loadGlb: aborted", "AbortError"));
      return;
    }
    const t = setTimeout(() => {
      cleanup();
      resolve();
    }, ms);
    const onAbort = () => {
      cleanup();
      reject(new DOMException("loadGlb: aborted", "AbortError"));
    };
    const cleanup = () => {
      clearTimeout(t);
      signal?.removeEventListener("abort", onAbort);
    };
    signal?.addEventListener("abort", onAbort, { once: true });
  });
}
