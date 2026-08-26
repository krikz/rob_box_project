// Captain Bridge scene: пол, освещение, сетка, рендерер, анимация loop.
//
// Phase 2.1 (t_0b4d85be, ADR-0032 §3.2): surface переходит с procedural grid
// на GLB-ассеты + HDR-IBL. Layered init:
//   1) initBridgeAssets() async — загружает 3 .glb + .hdr с graceful fallback
//      на procedural walls (BoxGeometry), если ассеты недоступны
//   2) PMREMGenerator + RoomEnvironment/HDR → scene.environment для PBR reflections
//   3) HemisphereLight + PointLights возле video-panels для ambient variation
//      и эмиссивной подсветки console strips
//   4) Grid расширен 20×20 → 40×40 (большой bridge), fog (6,16) → (8,24)
//      (дальние стены не обрезаются резко)

import * as THREE from "three";
import { LidarOverlay } from "./lidar_overlay";
import { VideoPanel } from "./video_panel";
import { PanelManager } from "./panel_manager";
import { initBridgeAssets, buildRoomEnvMap, type BridgeAssetsHandle } from "./bridge_assets";

export interface CaptainBridgeOptions {
  canvas: HTMLCanvasElement;
  enableXr?: boolean;
  /** Путь к bridge-ассетам. Default: /assets/bridge/ (Vite public/). */
  assetBaseUrl?: string;
  /** Кастомный fetcher для тестов. */
  fetcher?: typeof fetch;
}

export interface CaptainBridgeHandle {
  scene: THREE.Scene;
  renderer: THREE.WebGLRenderer;
  camera: THREE.PerspectiveCamera;
  lidar: LidarOverlay;
  panels: PanelManager;
  videoPanels: Map<string, VideoPanel>;
  /** Async init: загружает GLB + HDR, обновляет scene.environment, выставляет
   *  PointLights возле panels. Должен быть вызван ДО start() / attachXrSession(). */
  initEnvironment(): Promise<BridgeAssetsHandle>;
  initLayout(): void;
  attachXrSession(session: XRSession): Promise<void>;
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
  // Background — глубокий sci-fi black-blue (§1.3 дизайна). Альтернативно
  // используется renderer.setClearColor(0x0a0d11, 1), но явный scene.background
  // гарантирует видимый dark interior для vr/3D и предсказуемый fallback,
  // если envMap/envMap выключен в VR.
  scene.background = new THREE.Color(0x0a0d11);
  // Fog (8, 24) — расширили с (6, 16), чтобы дальние стены (≈6-8м) не
  // обрезались резко. Цвет — глубокий sci-fi black-blue, как и clearColor.
  scene.fog = new THREE.Fog(0x0a0d11, 8, 24);

  const camera = new THREE.PerspectiveCamera(
    70,
    window.innerWidth / window.innerHeight,
    0.05,
    50
  );
  camera.position.set(0, 1.6, 0); // высота глаз ~1.6м

  // ---------- Базовое освещение ----------
  // Видео-панели используют MeshBasicMaterial (toneMapped=false), освещение
  // для них не нужно. Но LiDAR/ground/props читаются с лёгким светом.
  const ambient = new THREE.AmbientLight(0xffffff, 0.6);
  scene.add(ambient);
  const dir = new THREE.DirectionalLight(0xffffff, 0.4);
  dir.position.set(2, 4, 1);
  scene.add(dir);
  // Hemisphere для ambient variation: холодный sky + тёплый ground
  // (по дизайну §1.2, не "белый" — добавляет depth в shader PBR materials).
  const hemi = new THREE.HemisphereLight(0x88aaff, 0x221a0a, 0.2);
  scene.add(hemi);

  // ---------- Пол: расширенный grid + solid plane ----------
  // 40×40 — большой мост, чтобы при ходьбе в VR не упёрся в край сетки.
  const grid = new THREE.GridHelper(40, 40, 0x444a52, 0x2a2f36);
  grid.position.y = 0;
  scene.add(grid);
  const floorGeom = new THREE.PlaneGeometry(40, 40);
  const floorMat = new THREE.MeshStandardMaterial({
    color: 0x14181f,
    roughness: 0.95,
    metalness: 0.0
  });
  const floor = new THREE.Mesh(floorGeom, floorMat);
  floor.rotation.x = -Math.PI / 2;
  scene.add(floor);

  // Маркер позиции пользователя (центр bridge).
  const origin = new THREE.Mesh(
    new THREE.CylinderGeometry(0.1, 0.1, 0.01, 24),
    new THREE.MeshBasicMaterial({ color: 0x2ec27e })
  );
  origin.position.set(0, 0.005, 0);
  scene.add(origin);

  // ---------- LiDAR overlay ----------
  const lidar = new LidarOverlay();
  scene.add(lidar.object);

  // ---------- Panel manager + video panels ----------
  const panelMgr = new PanelManager();
  const videoPanels = new Map<string, VideoPanel>();
  // Список PointLights возле каждой panel (создаются в syncPanels).
  const panelLights = new Map<string, THREE.PointLight>();

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
        // Эмиссивная подсветка от console strips: голубой PointLight
        // (по дизайну §1.2), distance=3м чтобы не светил сквозь дальние стены.
        const pl = new THREE.PointLight(0x3a8fff, 0.3, 3, 1.5);
        pl.position.set(s.position.x, s.position.y + 0.2, s.position.z);
        scene.add(pl);
        panelLights.set(s.id, pl);
      } else {
        vp.setState(s);
        // PointLight следует за panel position при move().
        const pl = panelLights.get(s.id);
        if (pl) pl.position.set(s.position.x, s.position.y + 0.2, s.position.z);
      }
      vp.setLabel(s.topic);
    }
    for (const [id, vp] of videoPanels.entries()) {
      if (!seen.has(id)) {
        scene.remove(vp.mesh);
        vp.dispose();
        videoPanels.delete(id);
        const pl = panelLights.get(id);
        if (pl) {
          scene.remove(pl);
          pl.dispose();
          panelLights.delete(id);
        }
      }
    }
  }

  function initLayout(): void {
    panelMgr.resetLayout();
    syncPanels();
  }

  // ---------- Environment (Phase 2.1) ----------
  let envAssets: BridgeAssetsHandle | null = null;

  async function initEnvironment(): Promise<BridgeAssetsHandle> {
    const manifest: Record<string, string> = {};
    if (opts.assetBaseUrl) {
      const base = opts.assetBaseUrl.replace(/\/$/, "");
      manifest.floorUrl = `${base}/bridge_floor.glb`;
      manifest.wallsUrl = `${base}/bridge_walls.glb`;
      manifest.propsUrl = `${base}/bridge_props.glb`;
      manifest.hdrUrl = `${base}/bridge_env_1k.hdr`;
    }

    envAssets = await initBridgeAssets({
      manifest,
      fetcher: opts.fetcher,
      renderer
    });
    scene.add(envAssets.rootGroup);

    if (envAssets.envMap) {
      // scene.environment для PBR reflections (envMap), НЕ scene.background —
      // bridge закрытый, background остаётся чистым dark-color.
      scene.environment = envAssets.envMap;
    } else {
      // Fallback: RoomEnvironment через PMREMGenerator, чтобы PBR-материалы
      // (console / walls) не были flat-black.
      try {
        scene.environment = buildRoomEnvMap(renderer);
      } catch {
        // jsdom без WebGL — оставляем scene без envmap, ambient достаточен.
      }
    }

    return envAssets;
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

  async function attachXrSession(session: XRSession): Promise<void> {
    if (opts.enableXr === false) return;
    await renderer.xr.setSession(session);
    renderer.setAnimationLoop(() => {
      renderer.render(scene, camera);
    });
  }

  function dispose(): void {
    window.removeEventListener("resize", resize);
    for (const vp of videoPanels.values()) vp.dispose();
    for (const pl of panelLights.values()) {
      scene.remove(pl);
      pl.dispose();
    }
    panelLights.clear();
    envAssets?.dispose();
    envAssets = null;
    lidar.dispose();
    renderer.dispose();
  }

  return {
    scene,
    renderer,
    camera,
    lidar,
    panels: panelMgr,
    videoPanels,
    initEnvironment,
    initLayout,
    attachXrSession,
    start,
    resize,
    dispose
  };
}