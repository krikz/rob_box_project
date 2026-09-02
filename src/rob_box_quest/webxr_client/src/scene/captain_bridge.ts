// Captain Bridge scene: пол, освещение, сетка, рендерер, анимация loop.

import * as THREE from "three";
import { LidarOverlay } from "./lidar_overlay";
import { VideoPanel } from "./video_panel";
import { PanelManager } from "./panel_manager";
import { createStatusHud, type RobotStatus, type StatusHud } from "./status_hud";
import { PointerSystem, type PointerRay } from "../interaction/pointer";
import { createStreamMenu, topicFromTargetId, type StreamMenuHandle, type StreamMenuRow } from "./stream_menu";
import { createSupervisorPanel, type SupervisorPanelHandle, PANEL_TARGET_PREFIX } from "./supervisor_panel";
import {
  loadBridgeAssets,
  type BridgeAssetHandle,
} from "./bridge_assets";

// Фронтальная камера робота — выводится на большой экран-стену перед
// оператором. Это OAK-D color (0x1001), которая в protocol/topics.py
// исторически названа "camera_rear", хотя это и есть передняя камера
// (та же, что в Telegram: /camera/camera/color/image_raw).
export const MAIN_SCREEN_TOPIC = "camera_rear";

// Боковые панели (Wave 3.A). Экран-стена занимает фронт, поэтому на
// панели уходят стримы, которых на нём нет: OAK-D depth и потолочная
// камера. `camera_oak_color` сюда не берём — это тот же сенсор, что и
// на экране-стене (registry: 0x1001 через ROS vs 0x1003 через depthai).
export const SIDE_PANEL_TOPICS = ["camera_oak_depth", "camera_ceiling"] as const;

// Углы боковых панелей: шире дефолтного полукруга (дизайн §3), чтобы
// не перекрывать экран-стену во фронтальном секторе обзора.
export const SIDE_PANEL_ANGLES_DEG = [-75, 75];

export interface CaptainBridgeOptions {
  canvas: HTMLCanvasElement;
  enableXr?: boolean;
  /**
   * Панель сменила топик через меню выбора стрима (R10). Клиент сам
   * решает, что делать с подписками: сцена про WSS ничего не знает.
   */
  onPanelTopicChange?(panelId: string, oldTopic: string, newTopic: string): void;
  /**
   * Клик по кнопке панели супервизора (R14). Действие — одна из строк:
   *   `mode:<avatar_mode>` — отправить SET_MODE (`avatar_set_mode` cmd);
   *   `floor:<teleop|voice>:<acquire|release>` — `ACQUIRE/RELEASE_FLOOR`;
   *   `dialogue:toggle` — переключить локальный `voice_mode`.
   * Сцена сама не знает транспорт — это ответственность bootstrap.
   */
  onSupervisorAction?(
    action: string,
    panel: SupervisorPanelHandle
  ): void;
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
  /** Большой экран-стена с фронтальной камерой (MAIN_SCREEN_TOPIC). */
  mainScreen: VideoPanel;
  /**
   * Loaded environment handle once `loadEnvironment()` resolves. `null`
   * until then, or if `environmentBaseUrl === null` was passed.
   */
  environment: BridgeAssetHandle | null;
  /** Async-load the Phase 2.1 Captain Bridge environment (GLB + HDR). */
  loadEnvironment(): Promise<BridgeAssetHandle | null>;
  initLayout(): void;
  attachXrSession(session: XRSession): Promise<void>;
  /** Visual feedback: подсветить grip контроллеров (deadman зажат). */
  setControllerActive(active: boolean): void;
  /** Arm-state HUD на стене (справа вверху): true=ARM, false=DISARM. */
  setArmState(armed: boolean): void;
  /** Status HUD на стене (слева вверху): battery / Wi-Fi / speed / RTT. */
  statusHud: StatusHud;
  /**
   * 3D-панель управления режимами аватара (R14, ADR-0027 R14 + ADR-0028 §4).
   * Создаётся сразу, видима после `supervisorPanel.setVisible(true)` (или
   * переключается клавишей `M` на десктопе).
   */
  supervisorPanel: SupervisorPanelHandle;
  /**
   * Топики, которые сцена умеет показывать — на них клиент подписывается
   * после WELCOME (main screen + боковые панели).
   */
  videoTopics(): string[];
  /**
   * Отдать JPEG-кадр панели с этим topic. `false` — панели с таким
   * топиком в сцене нет (или кадр дропнут, GPU занят).
   */
  ingestPanelFrame(topic: string, jpeg: Uint8Array): boolean;
  /** robot_status (0x1201) → HUD. */
  setRobotStatus(status: RobotStatus | null): void;
  /**
   * Кадр указателя (мышь на десктопе, луч контроллера в VR). `null` —
   * указателя нет: наведение снимается, начатый драг корректно закрывается.
   */
  updatePointer(ray: PointerRay | null): void;
  /** Слой указателя — сюда регистрируются будущие кликабельные объекты. */
  pointer: PointerSystem;
  /**
   * Каталог доступных стримов (из `stream_list`) — наполняет меню выбора
   * стрима, которое всплывает по клику на панель.
   */
  setAvailableStreams(rows: StreamMenuRow[]): void;
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

  // LiDAR строится от центра робота = начало координат сцены (пол под
  // оператором), на реальной высоте плоскости луча. Подробности и причины
  // настроек видимости — в lidar_overlay.ts.
  const lidar = new LidarOverlay({ center: { x: 0, y: 0, z: 0 } });
  scene.add(lidar.object);

  // Panel manager + video panels: экран-стена спереди (mainScreen ниже)
  // + боковые панели с остальными камерами (Wave 3.A).
  const panelMgr = new PanelManager({
    defaultTopics: [...SIDE_PANEL_TOPICS],
    angles: [...SIDE_PANEL_ANGLES_DEG]
  });
  const videoPanels = new Map<string, VideoPanel>();

  // Указатель: наведение / клик / перетаскивание панелей лучом.
  // Центр сферы драга — голова оператора (панели катаются вокруг него,
  // расстояние не меняется, facing всегда в центр). Сюда же потом
  // регистрируются кнопки панели режимов супервизора и карта.
  const pointer = new PointerSystem({
    center: { x: 0, y: 1.6, z: 0 },
    handlers: {
      onHover: () => refreshHighlights(),
      onSelect: (id) => {
        // Клик по строке меню — смена стрима выбранной панели.
        const menuTopic = topicFromTargetId(id);
        if (menuTopic !== null) {
          applyMenuChoice(menuTopic);
          return;
        }
        // Клик по кнопке панели супервизора (R14): маршрутизируем в
        // callback, который установлен через `onSupervisorAction`.
        if (id.startsWith(PANEL_TARGET_PREFIX)) {
          const action = id.slice(PANEL_TARGET_PREFIX.length);
          opts.onSupervisorAction?.(action, supervisorPanel);
          return;
        }
        // Клик по панели — выбор + меню стримов (повторный клик закрывает).
        const already = panelMgr.get(id)?.selected ?? false;
        panelMgr.select(already ? null : id);
        if (already) closeStreamMenu();
        else openStreamMenu(id);
        refreshHighlights();
      },
      onDrag: (id, position) => {
        panelMgr.move(id, position.x, position.z, position.y);
        const s = panelMgr.get(id);
        if (s) videoPanels.get(id)?.setState(s);
      },
      onDragEnd: () => refreshHighlights()
    }
  });

  // 3D-панель управления режимами аватара (R14). Каждая кнопка — отдельный
  // меш, raycaster из PointerSystem ловит её по prefix `sup:`. Сама панель
  // не перекрывается stream_menu (renderOrder=15 < 20) и не участвует в
  // PanelManager-геометрии (это не видео-панель с потоком). По умолчанию
  // скрыта — оператор открывает её клавишей M на десктопе, в VR лучом контроллера.
  const supervisorPanel = createSupervisorPanel();
  scene.add(supervisorPanel.object);
  for (const t of supervisorPanel.targets()) {
    pointer.addTarget({ id: t.id, object: t.object, draggable: false });
  }

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

  // Status HUD (Wave 3.A / R8): battery, Wi-Fi, скорость, RTT, режим.
  // Зеркально ARM-индикатору — левый верх стены-экрана.
  const statusHud = createStatusHud();
  scene.add(statusHud.sprite);

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
        // Панель становится целью указателя: наводится, кликается, тянется.
        pointer.addTarget({ id: s.id, object: vp.mesh, draggable: true });
      } else {
        vp.setState(s);
      }
      vp.setLabel(s.topic);
      vp.setHighlight(highlightFor(s.id, s.selected));
    }
    for (const [id, vp] of videoPanels.entries()) {
      if (!seen.has(id)) {
        scene.remove(vp.mesh);
        pointer.removeTarget(id);
        vp.dispose();
        videoPanels.delete(id);
      }
    }
  }

  function highlightFor(id: string, selected: boolean): "none" | "hover" | "selected" {
    if (selected) return "selected";
    return pointer.getHovered() === id ? "hover" : "none";
  }

  function refreshHighlights(): void {
    for (const s of panelMgr.list()) {
      videoPanels.get(s.id)?.setHighlight(highlightFor(s.id, s.selected));
    }
  }

  function initLayout(): void {
    panelMgr.resetLayout();
    syncPanels();
  }

  // ---------- меню выбора стрима (R10) ----------

  let streamMenu: StreamMenuHandle | null = null;
  let menuPanelId: string | null = null;

  function setAvailableStreams(rows: StreamMenuRow[]): void {
    closeStreamMenu();
    streamMenu?.dispose();
    if (streamMenu) scene.remove(streamMenu.object);
    // В меню только видео: лидар и robot_status на панель не положишь.
    const videoRows = rows.filter((r) => r.topic.startsWith("camera_"));
    streamMenu = videoRows.length > 0 ? createStreamMenu(videoRows) : null;
    if (streamMenu) scene.add(streamMenu.object);
  }

  function openStreamMenu(panelId: string): void {
    const state = panelMgr.get(panelId);
    if (!streamMenu || !state) return;
    closeStreamMenu();
    menuPanelId = panelId;
    streamMenu.show(
      new THREE.Vector3(state.position.x, state.position.y, state.position.z),
      Math.atan2(state.facing.x, state.facing.z),
      state.topic
    );
    // Цели регистрируем только на время показа: скрытый меш всё равно
    // ловил бы луч, и оператор кликал бы в невидимое меню.
    for (const t of streamMenu.targets()) {
      pointer.addTarget({ id: t.id, object: t.object, draggable: false });
    }
  }

  function closeStreamMenu(): void {
    if (!streamMenu) return;
    for (const t of streamMenu.targets()) pointer.removeTarget(t.id);
    streamMenu.hide();
    menuPanelId = null;
  }

  function applyMenuChoice(topic: string): void {
    const panelId = menuPanelId;
    if (!panelId) return;
    const state = panelMgr.get(panelId);
    closeStreamMenu();
    if (!state || state.topic === topic) return;
    const oldTopic = state.topic;
    panelMgr.switchStream(panelId, topic);
    const next = panelMgr.get(panelId);
    const vp = videoPanels.get(panelId);
    if (next && vp) {
      vp.setState(next);
      vp.setLabel(next.topic);
    }
    opts.onPanelTopicChange?.(panelId, oldTopic, topic);
  }

  function updatePointer(ray: PointerRay | null): void {
    pointer.update(ray);
  }

  function videoTopics(): string[] {
    const topics = new Set<string>([MAIN_SCREEN_TOPIC]);
    for (const s of panelMgr.list()) topics.add(s.topic);
    return [...topics];
  }

  function ingestPanelFrame(topic: string, jpeg: Uint8Array): boolean {
    if (topic === MAIN_SCREEN_TOPIC) return mainScreen.ingestJpeg(jpeg);
    for (const vp of videoPanels.values()) {
      if (vp.topic === topic) return vp.ingestJpeg(jpeg);
    }
    return false;
  }

  function setRobotStatus(status: RobotStatus | null): void {
    statusHud.setStatus(status);
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
    streamMenu?.dispose();
    environment?.dispose();
    armTexture.dispose();
    statusHud.dispose();
    supervisorPanel.dispose();
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
    loadEnvironment,
    initLayout,
    updatePointer,
    pointer,
    supervisorPanel,
    setAvailableStreams,
    attachXrSession,
    setControllerActive,
    setArmState,
    statusHud,
    videoTopics,
    ingestPanelFrame,
    setRobotStatus,
    start,
    resize,
    dispose
  };
}