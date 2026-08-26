// Captain Bridge scene: пол, освещение, сетка, рендерер, анимация loop.

import * as THREE from "three";
import { LidarOverlay } from "./lidar_overlay";
import { VideoPanel } from "./video_panel";
import { PanelManager } from "./panel_manager";
import {
  loadBridgeAssets,
  type BridgeAssetHandle,
} from "./bridge_assets";

export interface CaptainBridgeOptions {
  canvas: HTMLCanvasElement;
  enableXr?: boolean;
  /**
   * Optional override for the environment base URL. Defaults to
   * `/models/environment/`. Pass `null` to disable environment loading
   * (e.g. unit tests that only exercise panels/LiDAR).
   */
  environmentBaseUrl?: string | null;
}

export interface CaptainBridgeHandle {
  scene: THREE.Scene;
  renderer: THREE.WebGLRenderer;
  camera: THREE.PerspectiveCamera;
  lidar: LidarOverlay;
  panels: PanelManager;
  videoPanels: Map<string, VideoPanel>;
  /**
   * Loaded environment handle once `loadEnvironment()` resolves. `null`
   * until then, or if `environmentBaseUrl === null` was passed.
   */
  environment: BridgeAssetHandle | null;
  /** Async-load the Phase 2.1 Captain Bridge environment (GLB + HDR). */
  loadEnvironment(): Promise<BridgeAssetHandle | null>;
  initLayout(): void;
  attachXrSession(session: XRSession): Promise<void>;
  /**
   * Установить callback, который вызывается в XR animation loop с текущим
   * XRFrame. Callback должен быть идемпотентным и быстрым (никаких тяжёлых
   * raycast через Object3D.traverse на каждом кадре). Используется для
   * §3.7 panel hover/click + §3.5 hand-tracking.
   */
  setOnXrFrame(cb: ((frame: XRFrame, session: XRSession) => void) | null): void;
  /**
   * Список объектов панелей для raycast (panelId → Object3D mesh).
   * Можно дёргать из main.ts / xr_panel_raycast.
   */
  getPanelRaycastTargets(): { panelId: string; mesh: THREE.Object3D }[];
  start(): () => void;
  resize(): void;
  dispose(): void;
}

export function createCaptainBridge(opts: CaptainBridgeOptions): CaptainBridgeHandle {
  const renderer = new THREE.WebGLRenderer({
    canvas: opts.canvas,
    antialias: true,
    alpha: false,
    powerPreference: "high-performance"
  });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.setClearColor(0x0a0d11, 1);

  const scene = new THREE.Scene();
  scene.fog = new THREE.Fog(0x0a0d11, 6, 16);

  const camera = new THREE.PerspectiveCamera(
    70,
    window.innerWidth / window.innerHeight,
    0.05,
    50
  );
  camera.position.set(0, 1.6, 0); // высота глаз ~1.6м

  // Освещение (видео-панели MeshBasicMaterial — освещение не нужно,
  // но LiDAR/ground лучше читаются с лёгким светом).
  const ambient = new THREE.AmbientLight(0xffffff, 0.6);
  scene.add(ambient);
  const dir = new THREE.DirectionalLight(0xffffff, 0.4);
  dir.position.set(2, 4, 1);
  scene.add(dir);

  // Пол: grid + solid plane.
  const grid = new THREE.GridHelper(20, 20, 0x444a52, 0x2a2f36);
  grid.position.y = 0;
  scene.add(grid);
  const floorGeom = new THREE.PlaneGeometry(20, 20);
  const floorMat = new THREE.MeshStandardMaterial({
    color: 0x14181f,
    roughness: 0.95,
    metalness: 0.0
  });
  const floor = new THREE.Mesh(floorGeom, floorMat);
  floor.rotation.x = -Math.PI / 2;
  scene.add(floor);

  // Маркер позиции пользователя.
  const origin = new THREE.Mesh(
    new THREE.CylinderGeometry(0.1, 0.1, 0.01, 24),
    new THREE.MeshBasicMaterial({ color: 0x2ec27e })
  );
  origin.position.set(0, 0.005, 0);
  scene.add(origin);

  // LiDAR overlay.
  const lidar = new LidarOverlay();
  scene.add(lidar.object);

  // Panel manager + video panels.
  const panelMgr = new PanelManager();
  const videoPanels = new Map<string, VideoPanel>();

  // Phase 2.1 environment (loaded lazily via loadEnvironment()).
  let environment: BridgeAssetHandle | null = null;
  const environmentBaseUrl = opts.environmentBaseUrl === null ? null : (opts.environmentBaseUrl ?? "/models/environment/");
  async function loadEnvironment(): Promise<BridgeAssetHandle | null> {
    if (environment) return environment;
    if (environmentBaseUrl === null) return null;
    try {
      environment = await loadBridgeAssets(scene, renderer, {
        baseUrl: environmentBaseUrl,
        loadHdr: true,
      });
    } catch (err) {
      // Fail soft: keep the procedural fallback floor + grid so the scene
      // remains usable in environments where the GLB cannot be served
      // (offline dev, missing static server, CDN failure). Log once.
      // eslint-disable-next-line no-console
      console.warn("[captain_bridge] bridge environment failed to load, falling back to procedural scene:", err);
      environment = null;
    }
    return environment;
  }

  function syncPanels(): void {
    const states = panelMgr.list();
    const seen = new Set<string>();
    for (const s of states) {
      seen.add(s.id);
      let vp = videoPanels.get(s.id);
      if (!vp) {
        vp = new VideoPanel(s);
        scene.add(vp.mesh);
        videoPanels.set(s.id, vp);
      } else {
        vp.setState(s);
      }
      vp.setLabel(s.topic);
    }
    for (const [id, vp] of videoPanels.entries()) {
      if (!seen.has(id)) {
        scene.remove(vp.mesh);
        vp.dispose();
        videoPanels.delete(id);
      }
    }
  }

  function initLayout(): void {
    panelMgr.resetLayout();
    syncPanels();
  }

  // ---------- render loop ----------

  let running = false;
  let raf = 0;
  function loop(): void {
    if (!running) return;
    raf = requestAnimationFrame(loop);
    renderer.render(scene, camera);
  }

  function start(): () => void {
    if (running) return () => undefined;
    running = true;
    loop();
    return () => {
      running = false;
      cancelAnimationFrame(raf);
    };
  }

  function resize(): void {
    const w = window.innerWidth;
    const h = window.innerHeight;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
  }
  window.addEventListener("resize", resize);

  // ---------- XR (опционально) ----------

  let onXrFrameCb: ((frame: XRFrame, session: XRSession) => void) | null = null;

  async function attachXrSession(session: XRSession): Promise<void> {
    if (opts.enableXr === false) return;
    // Включаем XR-режим рендерера ДО setSession: без этого three.js
    // не подменяет камеру на XR-камеру (голова не отслеживается, взор
    // зафиксирован) и не биндит XR framebuffer.
    renderer.xr.enabled = true;
    await renderer.xr.setSession(session);
    renderer.setAnimationLoop((_time, frame) => {
      renderer.render(scene, camera);
      if (frame && onXrFrameCb) {
        try {
          onXrFrameCb(frame, session);
        } catch (err) {
          // eslint-disable-next-line no-console
          console.warn("[captain_bridge] onXrFrame threw:", err);
        }
      }
    });
  }

  function setOnXrFrame(cb: ((frame: XRFrame, session: XRSession) => void) | null): void {
    onXrFrameCb = cb;
  }

  function getPanelRaycastTargets(): { panelId: string; mesh: THREE.Object3D }[] {
    const out: { panelId: string; mesh: THREE.Object3D }[] = [];
    for (const [id, vp] of videoPanels.entries()) {
      out.push({ panelId: id, mesh: vp.mesh });
    }
    return out;
  }

  function dispose(): void {
    window.removeEventListener("resize", resize);
    for (const vp of videoPanels.values()) vp.dispose();
    lidar.dispose();
    environment?.dispose();
    renderer.dispose();
  }

  return {
    scene,
    renderer,
    camera,
    lidar,
    panels: panelMgr,
    videoPanels,
    environment,
    loadEnvironment,
    initLayout,
    attachXrSession,
    setOnXrFrame,
    getPanelRaycastTargets,
    start,
    resize,
    dispose
  };
}