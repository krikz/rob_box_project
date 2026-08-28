// avatar_loader.ts — Phase 2.2 avatar loader for the Captain Bridge.
//
// Loads the Phase 2.1 optimized avatar (avatar.glb, Draco + Meshopt
// compressed, CC0) into a THREE.Scene via the generic asset_loader
// (GLTFLoader + DRACOLoader + KTX2Loader + MeshoptDecoder). Returns a
// handle with the parsed scene + declared animations for downstream
// hand-tracking and telemetry to drive (idle pose, etc.).
//
// Why a separate module from bridge_assets.ts and asset_loader.ts:
//   bridge_assets.ts owns the *environment* (5 GLBs + HDR, ~70 KB + 1.6
//   MB HDR) wired into captain_bridge.ts. asset_loader.ts is the *generic*
//   loader factory (DRACO + KTX2 + Meshopt wiring). avatar_loader.ts is
//   the *avatar-specific* orchestration — it knows the avatar URL,
//   positioning convention (1.6 m above floor, facing -Z), and exposes a
//   typed handle with the animation clips.
//
// Phase 1.5 (issue #1639) shipped the bare Three.js scene with a procedural
// floor and no avatar. Phase 2.0 (ADR-0032) shipped the asset pipeline.
// Phase 2.1 (t_1fa6e505 / t_0bd54b80) shipped the avatar + environment
// assets. This module is the bridge that wires the assets into the
// runtime so the avatar is visible to the operator in the Quest headset.

import * as THREE from "three";
import {
  createAssetLoader,
  loadGlb,
  disposeAssetLoader,
  DEFAULT_DRACO_PATH,
  DEFAULT_BASIS_PATH,
  type LoadedAsset,
} from "./asset_loader";

// ---------- public types ----------

/** Per spec: avatar stands at y = 1.6 m (eye height) on the captain bridge. */
export const AVATAR_DEFAULT_HEIGHT_M = 1.6;

/** Per spec: avatar faces -Z (toward the main screen at z = -3.9). */
export const AVATAR_DEFAULT_FACING_RAD_Y = Math.PI; // π rad = facing -Z

/**
 * Phase 2.1 avatar is at this URL relative to the avatar/ directory.
 * We point at the *.optimized.glb artifact (Phase 2.0 pipeline output)
 * because that's the only variant committed to the repo — raw avatar.glb
 * is .gitignore'd per webxr_client/.gitignore (which only allows
 * *.optimized.glb in public/models/). For the avatar asset the raw and
 * optimized files are byte-identical (Draco + Meshopt are already
 * applied at the source GLB level), so the URL is unambiguous.
 */
export const AVATAR_DEFAULT_URL = "/models/avatar/avatar.optimized.glb";

/** Loader hooks for tests + downstream callers (e.g. dev-only avatars). */
export interface LoadAvatarOptions {
  /**
   * Base URL for the avatar directory, defaults to `/models/avatar/`.
   * Pass a different value for test fixtures or CDN deployments.
   */
  baseUrl?: string;
  /** Override the avatar file name (default `avatar.glb`). */
  filename?: string;
  /** Position override — defaults to (0, 1.6, 0). Set y to 0 for "spawn at floor". */
  position?: THREE.Vector3 | [number, number, number];
  /** Yaw rotation override (radians around Y, default π = facing -Z). */
  facingYRad?: number;
  /** Network retry budget (forwarded to asset_loader). Default 3. */
  retries?: number;
  /**
   * Skip adding the avatar to the supplied scene. Useful when the caller
   * wants to inspect the parsed Group before parenting it.
   */
  skipAttach?: boolean;
}

export interface AvatarAssetHandle {
  /** Parsed Group for the avatar (already added to scene unless skipAttach). */
  readonly group: THREE.Group;
  /** Animation clips (idle, walking, gesture…); Phase 2.1 ships 5. */
  readonly animations: ReadonlyArray<THREE.AnimationClip>;
  /** Final world position of the avatar root. */
  readonly position: THREE.Vector3;
  /** Source URL of the loaded glTF (for telemetry / cache keying). */
  readonly url: string;
  /** Asset payload size in bytes (Draco-decoded + Meshopt-decoded). */
  readonly payloadBytes: number;
  /** Dispose decoder resources. Call when the scene tears down. */
  dispose(): void;
}

// ---------- core ----------

const DEFAULT_BASE_URL = "/models/avatar/";

/**
 * Load the Phase 2.1 avatar into `scene`. Returns a handle with the
 * parsed Group + animation clips. The avatar is positioned at
 * (0, 1.6, 0) facing -Z by default; both can be overridden via opts.
 *
 * On error: throws. Callers that want fail-soft behaviour (e.g. for
 * offline Quest headsets during dev) should wrap this in try/catch and
 * fall back to a placeholder mesh. captain_bridge.ts does this for the
 * environment already; the avatar load is non-optional for production.
 */
export async function loadAvatar(
  scene: THREE.Scene,
  opts: LoadAvatarOptions = {},
): Promise<AvatarAssetHandle> {
  const baseUrl = opts.baseUrl ?? DEFAULT_BASE_URL;
  const filename = opts.filename ?? AVATAR_DEFAULT_URL.split("/").pop() ?? "avatar.glb";
  const url = `${baseUrl}${filename}`.replace(/\/+/g, "/");

  // Build a fresh loader triple (DRACO + KTX2 + Meshopt). We don't share
  // the environment's loaders because:
  //   - Their DRACO worker pools have different priorities (env decodes
  //     ~30 MB of geometry, avatar decodes ~19 KB)
  //   - Cleaner disposal semantics — avatar_loader.dispose() releases
  //     the avatar's decoder pool independently of the env's
  const loaderSet = createAssetLoader({
    dracoPath: DEFAULT_DRACO_PATH,
    basisPath: DEFAULT_BASIS_PATH,
  });

  try {
    const loaded: LoadedAsset = await loadGlb(url, { retries: opts.retries }, loaderSet);

    // Apply positioning convention. We do this AFTER the parse so the
    // avatar's internal animation tracks (which encode local-space
    // transforms) are unaffected.
    const pos = opts.position;
    if (Array.isArray(pos)) {
      loaded.scene.position.set(pos[0], pos[1], pos[2]);
    } else if (pos) {
      loaded.scene.position.copy(pos);
    } else {
      loaded.scene.position.set(0, AVATAR_DEFAULT_HEIGHT_M, 0);
    }
    const facing = opts.facingYRad ?? AVATAR_DEFAULT_FACING_RAD_Y;
    loaded.scene.rotation.y = facing;

    if (!opts.skipAttach) {
      scene.add(loaded.scene);
    }

    return {
      group: loaded.scene,
      animations: loaded.animations,
      position: loaded.scene.position.clone(),
      url: loaded.url,
      payloadBytes: loaded.payloadBytes,
      dispose(): void {
        // Dispose meshes / materials before releasing decoders so the
        // GPU upload buffers are freed first (DRACOLoader.dispose() only
        // tears down worker threads).
        loaded.scene.traverse((obj) => {
          const mesh = obj as THREE.Mesh;
          mesh.geometry?.dispose?.();
          const mat = mesh.material as THREE.Material | THREE.Material[] | undefined;
          if (Array.isArray(mat)) {
            for (const m of mat) m?.dispose?.();
          } else {
            mat?.dispose?.();
          }
        });
        // Detach from parent if we attached it.
        loaded.scene.parent?.remove(loaded.scene);
        disposeAssetLoader(loaderSet);
      },
    };
  } catch (err) {
    // Make sure the decoder workers don't leak if parse failed.
    disposeAssetLoader(loaderSet);
    throw err;
  }
}
