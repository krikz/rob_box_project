// Captain Bridge scene: пол, освещение, сетка, рендерер, анимация loop.

import * as THREE from "three";
import { LidarOverlay } from "./lidar_overlay";
import { VideoPanel } from "./video_panel";
import { PanelManager } from "./panel_manager";
import {
  loadBridgeAssets,
  type BridgeAssetHandle,
} from "./bridge_assets";
import {
  loadAvatar as loadAvatarAsset,
  type AvatarAssetHandle,
} from "./avatar_loader";

// Фронтальная камера робота — выводится на большой экран-стену перед
// оператором. Это OAK-D color (0x1001), которая в protocol/topics.py
// исторически названа "camera_rear", хотя это и есть передняя камера
// (та же, что в Telegram: /camera/camera/color/image_raw).
export const MAIN_SCREEN_TOPIC = "camera_rear";

export interface CaptainBridgeOptions {
  canvas: HTMLCanvasElement;
  enableXr?: boolean;
  /**
   * Optional override for the environment base URL. Defaults to
   * `/models/environment/`. Pass `null` to disable environment loading
   * (e.g. unit tests that only exercise panels/LiDAR).
   */
  environmentBaseUrl?: string | null;
  /**
   * Optional override for the avatar base URL. Defaults to
   * `/models/avatar/`. Pass `null` to skip avatar loading (e.g. unit
   * tests that exercise the environment only, or an operator profile
   * that wants to navigate the bridge without the avatar visible).
   */
  avatarBaseUrl?: string | null;
}

export interface CaptainBridgeHandle {
  scene: THREE.Scene;
  renderer: THREE.WebGLRenderer;
  camera: THREE.PerspectiveCamera;
  lidar: LidarOverlay;
  panels: PanelManager;
  videoPanels: Map<string, VideoPanel>;
  /** Большой экран-стена с фронтальной камерой (MAIN_SCREEN_TOPIC). */
  mainScreen: VideoPanel;
  /**
   * Loaded environment handle once `loadEnvironment()` resolves. `null`
   * until then, or if `environmentBaseUrl === null` was passed.
   */
  environment: BridgeAssetHandle | null;
  /**
   * Loaded avatar handle once `loadAvatar()` resolves. `null` until
   * then, or if `avatarBaseUrl === null` was passed.
   */
  avatar: AvatarAssetHandle | null;
  /** Async-load the Phase 2.1 Captain Bridge environment (GLB + HDR). */
  loadEnvironment(): Promise<BridgeAssetHandle | null>;
  /** Async-load the Phase 2.2 avatar (Draco + Meshopt compressed GLB). */
  loadAvatar(): Promise<AvatarAssetHandle | null>;
  initLayout(): void;
  attachXrSession(session: XRSession): Promise<void>;
  /** Visual feedback: подсветить grip контроллеров (deadman зажат). */
  setControllerActive(active: boolean): void;
  /** Arm-state HUD на стене (справа вверху): true=ARM, false=DISARM. */
  setArmState(armed: boolean): void;
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

  // Panel manager + video panels. Дефолтных floating-панелей больше нет —
  // вместо них один большой экран-стена (см. mainScreen ниже).
  const panelMgr = new PanelManager({ defaultTopics: [] });
  const videoPanels = new Map<string, VideoPanel>();

  // Большой экран-стена перед оператором: на него выводим фронтальную
  // камеру. Стена мостика стоит на z = -4 (ROOM_D/2); экран висит чуть
  // ближе (z = -3.9), лицом к пользователю (facing +Z).
  const mainScreen = new VideoPanel(
    {
      id: "main_screen",
      topic: MAIN_SCREEN_TOPIC,
      position: { x: 0, y: 1.5, z: -3.9 },
      facing: { x: 0, z: 1 },
      size: { width: 4.8, height: 2.7 },
      selected: false
    },
    { showLabel: false, canvasWidth: 1280, canvasHeight: 720 }
  );
  scene.add(mainScreen.mesh);

  // Arm-state HUD: справа вверху на стене, рядом с экраном камеры.
  // Sprite всегда повёрнут к камере — читается из любой позы оператора.
  const armCanvas = document.createElement("canvas");
  armCanvas.width = 512;
  armCanvas.height = 128;
  const armCtx = armCanvas.getContext("2d");
  if (!armCtx) {
    throw new Error("captain_bridge: failed to acquire arm HUD 2D context");
  }
  const armTexture = new THREE.CanvasTexture(armCanvas);
  armTexture.minFilter = THREE.LinearFilter;
  armTexture.magFilter = THREE.LinearFilter;
  const armSprite = new THREE.Sprite(
    new THREE.SpriteMaterial({ map: armTexture, depthTest: false, transparent: true })
  );
  // Правый верхний угол стены-экрана (mainScreen 4.8×2.7, центр y=1.5, z=-3.9).
  armSprite.position.set(2.35, 2.95, -3.85);
  armSprite.scale.set(1.1, 0.275, 1);
  scene.add(armSprite);

  function drawArmHud(armed: boolean): void {
    const ctx = armCtx!;
    ctx.clearRect(0, 0, armCanvas.width, armCanvas.height);
    // Тёмная подложка.
    ctx.fillStyle = "rgba(10, 13, 17, 0.72)";
    ctx.fillRect(0, 0, armCanvas.width, armCanvas.height);
    // Цветной индикатор слева.
    ctx.fillStyle = armed ? "#2ec27e" : "#8b98a5";
    ctx.fillRect(0, 0, 16, armCanvas.height);
    // Текст.
    ctx.fillStyle = armed ? "#2ec27e" : "#8b98a5";
    ctx.font = "bold 56px monospace";
    ctx.textBaseline = "middle";
    ctx.fillText(armed ? "ARM" : "DISARM", 44, armCanvas.height / 2);
    armTexture.needsUpdate = true;
  }
  drawArmHud(false);

  function setArmState(armed: boolean): void {
    drawArmHud(armed);
  }

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

  // Phase 2.2 avatar (loaded lazily via loadAvatar()). Same fail-soft
  // contract as the environment: if the avatar GLB can't be fetched we
  // log once and continue with the rest of the scene (the procedural
  // origin marker stays in place). Operator-visible only — does not
  // gate any panel / XR / connection logic.
  let avatar: AvatarAssetHandle | null = null;
  const avatarBaseUrl = opts.avatarBaseUrl === null ? null : (opts.avatarBaseUrl ?? "/models/avatar/");
  async function loadAvatarFn(): Promise<AvatarAssetHandle | null> {
    if (avatar) return avatar;
    if (avatarBaseUrl === null) return null;
    try {
      avatar = await loadAvatarAsset(scene, { baseUrl: avatarBaseUrl });
    } catch (err) {
      // eslint-disable-next-line no-console
      console.warn("[captain_bridge] avatar failed to load:", err);
      avatar = null;
    }
    return avatar;
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

  // Визуализация XR-контроллеров: ray из targetRaySpace + маркер grip.
  // Позиции/ориентацию подставляет three.js из XR-кадров автоматически.
  const controllerGrips: THREE.Mesh[] = [];
  const CONTROLLER_RAY_LENGTH = 1.5;
  const GRIP_IDLE_COLOR = 0x556677;
  const GRIP_ACTIVE_COLOR = 0x2ec27e;

  function setControllerActive(active: boolean): void {
    for (const grip of controllerGrips) {
      (grip.material as THREE.MeshBasicMaterial).color.set(active ? GRIP_ACTIVE_COLOR : GRIP_IDLE_COLOR);
    }
  }

  async function attachXrSession(session: XRSession): Promise<void> {
    if (opts.enableXr === false) return;
    // Включаем XR-режим рендерера ДО setSession: без этого three.js
    // не подменяет камеру на XR-камеру (голова не отслеживается, взор
    // зафиксирован) и не биндит XR framebuffer.
    renderer.xr.enabled = true;
    await renderer.xr.setSession(session);

    // Контроллеры: добавляем по одному разу (повторный вход в VR не дублирует).
    for (let i = 0; i < 2; i++) {
      if (controllerGrips[i]) continue;
      const root = renderer.xr.getController(i);
      const ray = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints([
          new THREE.Vector3(0, 0, 0),
          new THREE.Vector3(0, 0, -CONTROLLER_RAY_LENGTH)
        ]),
        new THREE.LineBasicMaterial({ color: 0x2ec27e, transparent: true, opacity: 0.75 })
      );
      root.add(ray);
      const grip = new THREE.Mesh(
        new THREE.CylinderGeometry(0.02, 0.02, 0.14, 12),
        new THREE.MeshBasicMaterial({ color: GRIP_IDLE_COLOR })
      );
      grip.rotation.x = Math.PI / 2; // цилиндр вдоль -Z (направление ray)
      grip.position.set(0, 0, -0.07);
      root.add(grip);
      scene.add(root);
      controllerGrips[i] = grip;
    }

    renderer.setAnimationLoop(() => {
      renderer.render(scene, camera);
    });
  }

  function dispose(): void {
    window.removeEventListener("resize", resize);
    mainScreen.dispose();
    for (const vp of videoPanels.values()) vp.dispose();
    lidar.dispose();
    environment?.dispose();
    avatar?.dispose();
    armTexture.dispose();
    renderer.dispose();
  }

  return {
    scene,
    renderer,
    camera,
    lidar,
    panels: panelMgr,
    videoPanels,
    mainScreen,
    environment,
    avatar,
    loadEnvironment,
    loadAvatar: loadAvatarFn,
    initLayout,
    attachXrSession,
    setControllerActive,
    setArmState,
    start,
    resize,
    dispose
  };
}