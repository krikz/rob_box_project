// Captain Bridge scene: пол, освещение, сетка, рендерер, анимация loop.

import * as THREE from "three";
import { LidarOverlay } from "./lidar_overlay";
import { createFloorOverlay, type FloorOverlayHandle } from "./floor_overlay";
import { VideoPanel } from "./video_panel";
import { PanelManager } from "./panel_manager";
import {
  applyLayout,
  createLayoutSaver,
  parseLayout,
  serializeLayout,
  PANEL_LAYOUT_STORAGE_KEY,
  type LayoutStorage
} from "./panel_layout_store";
import { FpsMeter } from "./fps_meter";
import { createStatusHud, type RobotStatus, type StatusHud } from "./status_hud";
import { PointerSystem, type PointerRay } from "../interaction/pointer";
import { createPointerBeam, type PointerBeamHandle } from "../interaction/pointer_beam";
import { resizeSize } from "../interaction/pointer_math";
import { createStreamMenu, topicFromTargetId, type StreamMenuHandle, type StreamMenuRow } from "./stream_menu";
import { createTtsPickerMenu, type TtsPickerMenuHandle } from "./tts_picker_menu";
import { parseTtsTargetId, type TtsPickerState, type TtsPickerTarget } from "../state/tts_picker_state";
import { createSupervisorPanel, type SupervisorPanelHandle, PANEL_TARGET_PREFIX } from "./supervisor_panel";
import {
  createVoicePipelinePanel,
  parsePipelineTargetId,
  PIPELINE_DRAG_TARGET_ID,
  type VoicePipelineAction,
  type VoicePipelinePanelHandle
} from "./voice_pipeline_panel";
import {
  loadBridgeAssets,
  type BridgeAssetHandle,
} from "./bridge_assets";
import {
  createVoiceStateIndicator,
  parseVoiceState,
  type VoiceStateFrame,
  type VoiceStateIndicator
} from "../ui/voice_state_indicator";
import {
  createTars1TextPanel,
  type Tars1TextPanelHandle
} from "./tars1_text_panel";
import {
  createTars2MetricsPanel,
  type Tars2MetricsPanelHandle
} from "./tars2_metrics_panel";

// Фронтальная камера робота — выводится на большой экран-стену перед
// оператором. Это OAK-D color (0x1001), которая в protocol/topics.py
// исторически названа "camera_rear", хотя это и есть передняя камера
// (та же, что в Telegram: /camera/camera/color/image_raw).
export const MAIN_SCREEN_TOPIC = "camera_rear";

// Потолочная камера робота (USB /dev/video0, registry: camera_ceiling).
// Смотрит вверх — и в сцене её экран висит над оператором (см. ниже,
// CEILING_SCREEN_*).
export const CEILING_SCREEN_TOPIC = "camera_ceiling";

// Боковые панели (Wave 3.A). Экран-стена занимает фронт, потолочная
// камера — верх, поэтому на свободную панель остаётся OAK-D depth.
// `camera_oak_color` сюда не берём — это тот же сенсор, что и на
// экране-стене (registry: 0x1001 через ROS vs 0x1003 через depthai).
export const SIDE_PANEL_TOPICS = ["camera_oak_depth"] as const;

// Углы боковых панелей: шире дефолтного полукруга (дизайн §3), чтобы
// не перекрывать экран-стену во фронтальном секторе обзора.
export const SIDE_PANEL_ANGLES_DEG = [-75];

// ── Потолочный экран ────────────────────────────────────────────────────
//
// Раскладка экранов повторяет геометрию камер на роботе. В URDF
// (rob_box.xacro) обе камеры стоят практически в одной точке на осевой
// линии: OAK-D на xyz=(-0.1158, -0.0002, 0.4595), потолочная на
// xyz=(-0.1707, 0.065, 0.4615) — на 55 мм позади и 65 мм левее, на той
// же высоте. Различаются они только направлением взгляда: OAK-D вперёд,
// потолочная вверх.
//
// Значит в сцене они обязаны делить азимут и различаться только
// наклоном: смотришь прямо — видишь, что перед роботом; поднимаешь
// голову — видишь, что над ним. Держать потолочный вид сбоку, на панели
// рядом с depth, значит ломать эту связь: оператор не может «посмотреть
// вверх», он должен вспомнить, на какой панели верх.
//
// Экран стоит РОВНО над головой (x = z = 0), а не сдвинут вперёд: сдвиг
// превращал его в ещё одну наклонную панель над экраном-стеной, и чтобы
// её увидеть, надо было смотреть вперёд-вверх, а не вверх. Потолок
// комнаты мостика на 3 м (bridge_scene_meta.json), экран висит на 2.85 —
// под ним, но заметно выше вытянутой руки: на 2.7 он висел почти на
// голове.
//
// Размер 4:3 — потолочная камера отдаёт 640×480 (usb_cam), 16:9 растянул
// бы кадр. 3.2×2.4 вместо 2.4×1.8: подъём с 2.7 до 2.85 уводит экран от
// глаз (1.10 м → 1.25 м, −12% видимого размера), и без увеличения он бы
// стал МЕНЬШЕ, а не больше. Множитель 4/3 перекрывает этот минус и
// добавляет сверху.
export const CEILING_SCREEN_POS = { x: 0, y: 2.85, z: 0 };
export const CEILING_SCREEN_SIZE = { width: 3.2, height: 2.4 };
/** Высота глаз оператора — экран доворачивается нормалью именно в неё. */
export const EYE_HEIGHT_M = 1.6;

/**
 * Наклон потолочного экрана: нормаль плоскости смотрит из центра экрана
 * в глаза оператора. Чистая функция — считается из позиции, а не задана
 * числом, чтобы сдвиг экрана не оставил его смотрящим мимо.
 *
 * Плоскость по умолчанию смотрит в +Z; поворот вокруг X на угол φ уводит
 * её нормаль в (0, −sin φ, cos φ). Нужна нормаль вдоль вектора
 * «экран → глаза», отсюда φ = atan2(y − eyeY, z_eye − z).
 */
export function ceilingScreenPitchRad(
  pos: { y: number; z: number } = CEILING_SCREEN_POS,
  eyeY: number = EYE_HEIGHT_M
): number {
  return Math.atan2(pos.y - eyeY, -pos.z);
}

/**
 * Разворот кадра потолочной камеры вокруг нормали экрана (радианы).
 *
 * При нулевом значении «верх кадра» ложится на +Z сцены — то есть за
 * спину оператора. Это верно, если камера смотрит вверх и её кадр
 * развёрнут верхом назад по ходу робота. Реального крепления мы не
 * знаем: в URDF (`rob_box.xacro:343`) потолочная камера объявлена
 * обычным макросом `usb_camera` с `rpy="0 0 0"`, то есть её optical
 * frame смотрит ВПЕРЁД, а не вверх — модель тут расходится с железом и
 * ориентацию по ней не восстановить.
 *
 * Поэтому разворот вынесен сюда отдельной константой: увидев на потолке
 * перевёрнутый или боком лежащий кадр, ставим π (перевернуть) или ±π/2
 * (довернуть на четверть) — одно число, без правки геометрии.
 */
export const CEILING_SCREEN_ROLL_RAD = 0;

export interface CaptainBridgeOptions {
  canvas: HTMLCanvasElement;
  enableXr?: boolean;
  /**
   * Панель сменила топик через меню выбора стрима (R10). Клиент сам
   * решает, что делать с подписками: сцена про WSS ничего не знает.
   */
  onPanelTopicChange?(panelId: string, oldTopic: string, newTopic: string): void;
  /**
   * AV-27: оператор ткнул лучом в TTS picker (строку/PREVIEW/APPLY/STOP/
   * CLOSE/вкладку VOICE). Сцена не знает ни про WSS, ни про состояние
   * стора — она только сообщает, куда попал луч.
   */
  onTtsPickerAction?(action: TtsPickerTarget): void;
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
   * Клик по кнопке панели голосового пайплайна (W6-2 / спека §3.6):
   *   `stt` / `llm` — тумблеры ступеней (клиент решает, какой `voice_mode`
   *   слать); `tts` — открыть TTS picker; `preset:<id>` / `lang:<lang>` —
   *   сменить стиль/язык. Сцена сама не знает транспорт — это ответственность
   *   bootstrap (тот же контракт, что у `onSupervisorAction`).
   */
  onPipelineAction?(action: VoicePipelineAction): void;
  /**
   * Optional override for the environment base URL. Defaults to
   * `/models/environment/`. Pass `null` to disable environment loading
   * (e.g. unit tests that only exercise panels/LiDAR).
   */
  environmentBaseUrl?: string | null;
  /**
   * AV-25: layout-store. Если не передавать — используется window.localStorage.
   * Для unit-тестов инжектится in-memory store. `null` отключает persist
   * (например, для тестов, которые не должны трогать реальный storage).
   */
  layoutStorage?: LayoutStorage | null;
  /**
   * AV-25: дебаунс записи layout (мс). Дефолт 500. `0` — синхронно.
   */
  layoutSaveDebounceMs?: number;
}

export interface CaptainBridgeHandle {
  scene: THREE.Scene;
  renderer: THREE.WebGLRenderer;
  camera: THREE.PerspectiveCamera;
  lidar: LidarOverlay;
  /**
   * Пол мостика: SLAM-карта под ногами + логотип поверх неё. Карта
   * кормится кадрами map_2d (0x1103) через `ingestMapFrame`.
   */
  floor: FloorOverlayHandle;
  panels: PanelManager;
  videoPanels: Map<string, VideoPanel>;
  /**
   * Потолочный экран над оператором (CEILING_SCREEN_TOPIC): смотришь
   * вверх — видишь, что над роботом.
   */
  ceilingScreen: VideoPanel;
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
  /** AV-25: сброс раскладки панелей к default + стирание localStorage. */
  resetPanelLayout(): void;
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
   * 3D-панель голосового пайплайна оператора (W6-2 / спека §3.6):
   * `голос → STT → LLM → TTS → динамик` + быстрые настройки ступеней.
   * Всегда видима (это не HUD-оверлей, а панель на мостике).
   */
  voicePipeline: VoicePipelinePanelHandle;
  /**
   * TARS 1 — текстовое полотно (issue #2113, quest #2112). Слева от
   * FRONT CAM, лицом к оператору. Показывает текст, который TARS
   * произносит (mirror `/avatar/tts/request`). API: append / clear /
   * setStreaming (см. tars1_text_panel.ts).
   */
  tars1Panel: Tars1TextPanelHandle;
  /**
   * TARS 2 — дашборд метрик (issue #2113, quest #2112). Справа от
   * FRONT CAM, симметрично TARS 1. URL Grafana-панели (Prometheus /
   * Loki) приходит через `/avatar/tars/panel_url` и рисуется ТЕКСТОМ на
   * canvas-preview (host/path) — реального рендера Grafana-контента в
   * immersive-WebXR нет и в этой карточке не появился (DOM/iframe не
   * проецируется в VR-сцену; см. tars2_metrics_panel.ts, шапка файла, и
   * PR #2114 "TARS2 — честное инженерное решение"). API: setPanelUrl /
   * clear / setState (см. tars2_metrics_panel.ts).
   */
  tars2Panel: Tars2MetricsPanelHandle;
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
  /**
   * map_2d (0x1103) → карта на полу. `false` — кадр битый или в нём нет
   * позы робота (карту тогда некуда класть).
   */
  ingestMapFrame(payload: Uint8Array): boolean;
  /** robot_status (0x1201) → HUD. */
  setRobotStatus(status: RobotStatus | null): void;
  /**
   * voice_state (0x1202) → центральный HUD-индикатор. Парсит msgpack-payload
   * и обновляет визуальное состояние + a11y live-region. Если payload
   * битый — кадр пропускается (молча, без падения).
   */
  /**
   * Возвращает распарсенный кадр (или `null`, если payload битый/пустой) —
   * чтобы `main.ts` кормил им трекер стадий реплики, не парся msgpack
   * второй раз.
   */
  setVoiceState(payload: Uint8Array | null): VoiceStateFrame | null;
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
  /**
   * AV-27: 3D-меню TTS picker'а. Сцена только рисует и ловит клики —
   * состояние и WS-команды живут в `main.ts` (тот же контракт, что у
   * `onPanelTopicChange`: сцена про WSS ничего не знает).
   */
  renderTtsPicker(state: TtsPickerState): void;
  /** Открыть меню TTS picker'а (рядом с экраном-стеной). */
  openTtsPicker(): void;
  /** Открыть меню TTS picker'а рядом с панелью голосового пайплайна. */
  openTtsPickerNearPipeline(): void;
  closeTtsPicker(): void;
  isTtsPickerOpen(): boolean;
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

  // Пол: карта SLAM под ногами + логотип поверх неё. Тот же центр, что у
  // лидара (начало координат = робот), поэтому карта и точки скана
  // совмещены один-в-один.
  const floorOverlay = createFloorOverlay();
  scene.add(floorOverlay.object);

  // Panel manager + video panels: экран-стена спереди (mainScreen ниже)
  // + боковые панели с остальными камерами (Wave 3.A).
  const panelMgr = new PanelManager({
    defaultTopics: [...SIDE_PANEL_TOPICS],
    angles: [...SIDE_PANEL_ANGLES_DEG]
  });
  const videoPanels = new Map<string, VideoPanel>();

  // AV-25: layout-store (localStorage) + дебаунс-запись. Если opts
  // выставил layoutStorage === null, persist отключён (для тестов).
  const layoutStorage: LayoutStorage | null =
    opts.layoutStorage === null
      ? null
      : opts.layoutStorage ?? (typeof window !== "undefined" && window.localStorage
        ? (window.localStorage as LayoutStorage)
        : null);
  const layoutSaver = layoutStorage
    ? createLayoutSaver(layoutStorage, opts.layoutSaveDebounceMs ?? 500)
    : null;
  // Реестр известных топиков: наполняется из stream_list (см.
  // setAvailableStreams ниже). До первого ответа сервера — содержит
  // дефолтные топики панелей и mainScreen, чтобы parseLayout мог
  // принять сохранённую раскладку сразу после старта.
  const knownTopics = new Set<string>([
    MAIN_SCREEN_TOPIC,
    CEILING_SCREEN_TOPIC,
    ...SIDE_PANEL_TOPICS
  ]);

  /**
   * Восстановить раскладку из store. Возвращает true, если что-то
   * применили; false — store пуст / битый / отключён (тогда мостик
   * остаётся на дефолтной раскладке, как до AV-25).
   *
   * applyLayout сам создаёт недостающие панели через
   * `state.createPanel(id, topic, ...)` с id из saved state. В
   * production PanelManager.createPanelWithId реализует именно эту
   * сигнатуру; обычный `createPanel(topic, ...)` (без id) используется
   * для пользовательских панелей, не приходящих из store.
   */
  function applyStoredLayout(): boolean {
    if (!layoutStorage) return false;
    const raw = layoutStorage.getItem(PANEL_LAYOUT_STORAGE_KEY);
    const parsed = parseLayout(raw, knownTopics);
    if (!parsed) return false;
    // Потолочная камера переехала со боковой панели на собственный экран
    // над оператором. У тех, кто уже летал на мостике, в localStorage
    // лежит старая раскладка с панелью camera_ceiling — восстановив её,
    // мы показали бы один и тот же поток дважды. Панель отбрасываем,
    // остальную раскладку оператора сохраняем.
    const panels = parsed.panels.filter((p) => p.topic !== CEILING_SCREEN_TOPIC);
    if (panels.length !== parsed.panels.length) {
      // eslint-disable-next-line no-console
      console.info(
        "[captain_bridge] dropped stored camera_ceiling panel — поток теперь на потолочном экране"
      );
    }
    if (panels.length === 0) return false;
    applyLayout({ ...parsed, panels }, panelMgr);
    return true;
  }

  // Указатель: наведение / клик / перетаскивание панелей лучом.
  // Центр сферы драга — голова оператора (панели катаются вокруг него,
  // расстояние не меняется, facing всегда в центр). Сюда же потом
  // регистрируются кнопки панели режимов супервизора и карта.
  const pointer = new PointerSystem({
    center: { x: 0, y: 1.6, z: 0 },
    handlers: {
      onHover: () => refreshHighlights(),
      onSelect: (id) => {
        // W6-2: клик по панели голосового пайплайна (vpl:*) — раньше
        // всего остального: его цели на том же слое указателя.
        const pipelineTarget = parsePipelineTargetId(id);
        if (pipelineTarget !== null) {
          opts.onPipelineAction?.(pipelineTarget);
          return;
        }
        // AV-27: клик по TTS picker'у (вкладка VOICE, строка, PREVIEW,
        // APPLY, STOP, CLOSE) — раньше всего остального: его цели живут
        // на том же слое указателя, что панели и stream_menu.
        const ttsTarget = parseTtsTargetId(id);
        if (ttsTarget !== null) {
          handleTtsTarget(ttsTarget);
          return;
        }
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
        // Фон панели голосового пайплайна — ручка перетаскивания.
        if (id === PIPELINE_DRAG_TARGET_ID) {
          voicePipeline.setPosition(position.x, position.y, position.z);
          return;
        }
        panelMgr.move(id, position.x, position.z, position.y);
        const s = panelMgr.get(id);
        if (s) videoPanels.get(id)?.setState(s);
        scheduleLayoutSave();
      },
      onDragEnd: () => {
        refreshHighlights();
        flushLayoutSave();
        savePipelinePos();
      },
      onResize: (id, corner, position) => {
        const s = panelMgr.get(id);
        if (!s) return;
        const next = resizeSize(s.position, s.size, position, s.facing, corner);
        const changed = panelMgr.resize(id, next.width, next.height);
        if (changed) {
          const updated = panelMgr.get(id);
          if (updated) videoPanels.get(id)?.setState(updated);
          scheduleLayoutSave();
        }
      },
      onResizeEnd: () => flushLayoutSave()
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

  // 3D-панель голосового пайплайна (W6-2 / спека §3.6): оператор видит и
  // настраивает путь «голос → STT → LLM → TTS → динамик». Всегда видима —
  // это панель на мостике, а не всплывающее меню. Кнопки — отдельные меши
  // на слое указателя (prefix `vpl:`), фон панели — ручка перетаскивания.
  const voicePipeline = createVoicePipelinePanel();
  scene.add(voicePipeline.object);
  for (const t of voicePipeline.targets()) {
    pointer.addTarget({ id: t.id, object: t.object, draggable: false });
  }
  // Фон панели тащит всю панель по сфере вокруг оператора. Кнопки ловят
  // луч первыми (они ближе к камере), поэтому перетаскивание не мешает клику.
  pointer.addTarget({ id: PIPELINE_DRAG_TARGET_ID, object: voicePipeline.object, draggable: true });

  // Большой экран-стена перед оператором: на него выводим фронтальную
  // камеру. Стена мостика стоит на z = -4.56 (ROOM_D/2, ADR-0076 R1,
  // ROOM_D = 9.12); экран висит чуть ближе (z = -3.9), лицом к
  // пользователю (facing +Z).
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

  // Потолочный экран: тот же азимут, что у экрана-стены, но над головой —
  // «смотрю прямо» / «смотрю вверх» повторяет пару камер на роботе.
  // Это фиксированный экран, а не панель PanelManager: панели живут на
  // горизонтальной окружности вокруг оператора (facing только в XZ) и
  // наклон по определению не умеют.
  const ceilingScreen = new VideoPanel(
    {
      id: "ceiling_screen",
      topic: CEILING_SCREEN_TOPIC,
      position: { ...CEILING_SCREEN_POS },
      facing: { x: 0, z: 1 },
      size: { ...CEILING_SCREEN_SIZE },
      selected: false
    },
    { showLabel: false, canvasWidth: 960, canvasHeight: 540 }
  );
  // VideoPanel.applyTransform умеет только rotation.y (панели стоят
  // вертикально). Наклон ставим сами — setState по этому экрану не
  // вызывается, так что перезатирания не будет.
  ceilingScreen.mesh.rotation.x = ceilingScreenPitchRad();
  ceilingScreen.mesh.rotation.z = CEILING_SCREEN_ROLL_RAD;
  scene.add(ceilingScreen.mesh);

  // TARS 1 + TARS 2 (issue #2113, quest #2112 / follow-up #2142):
  // Captain Bridge — два боковых экрана по сторонам от FRONT CAM, лицом к
  // оператору. Размер и ориентация — по ADR-0074 §4.0 (вариант E, выбран
  // Шифу 2026-09-08): yaw = 36.9°, дистанция 4.50 м, back-tilt ≈ 1.3°.
  // Позиции и углы НЕ меняются; меняется только ширина W (ADR-0076 R1).
  //
  // ADR-0076 (PR #2156) фиксирует расширение ROOM_D до 9.12 м (R1).
  // Ширина W = 3.0 м (безопасный компромисс: z_inner = -4.50, требуемый
  // ROOM_D/2 = 4.55, R1 = 4.56 хватает с запасом 1 см). Если Шифу выберет
  // W = 3.2 м — переключить на R2 (ROOM_D = 10.0) одной правкой ниже.
  // aspect 16:9 (как у основного экрана 4.8 × 2.7).
  //
  // ИЗВЕСТНЫЙ ОТКРЫТЫЙ ДЕФЕКТ (issue #2142-B, раскопано nightly-review-fix
  // 2026-09-08, доказано скриптом на THREE.Box3 + точным пересечением
  // кромки панели с плоскостью экрана-стены z=-3.9): при ТЕКУЩИХ
  // TARS_PANEL_X/Y/Z и yaw-формуле (см. ниже) панели TARS1/TARS2
  // физически пересекают прямоугольник главного экрана ДАЖЕ на honestly
  // исправленной геометрии (mesh.scale = финальный размер, без двойного
  // масштабирования) и даже на нижней границе диапазона ADR-0074
  // (W=2.4 м). Расчёт (при неизменных X=2.7/Y=1.5/Z=-3.6/yaw=36.87°):
  // безопасная ширина без пересечения — не больше ~0.98 м. То есть само
  // положение/угол из ADR-0074 §4.0 «вариант E» несовместимо с любой
  // шириной панели из согласованного диапазона 2.4–3.2 м — предыдущая
  // геометрическая проверка (ADR-0074/0076) сверяла только клиренс до
  // задней стены (ROOM_D), но не пересечение с главным экраном.
  // Пофиксить долю бага (двойное масштабирование, mesh был 4.8×1.52 м
  // вместо заявленных 3.0×1.69 м) — сделано ниже и в tars1_text_panel.ts /
  // tars2_metrics_panel.ts. Пересечение с главным экраном ЭТИМ не снято:
  // нужна новая архитектурная карточка (сдвинуть X/Z, увеличить дистанцию
  // или уменьшить диапазон W) — владелец должен решить, что двигать, как
  // это уже было с ROOM_D в ADR-0076. НЕ меняю позицию/угол сам.
  const TARS_PANEL_WIDTH = 3.0; // TODO(ADR-0076 R2): 3.2 если Шифу захочет максимум
  const TARS_PANEL_SIZE = {
    width: TARS_PANEL_WIDTH,
    height: (TARS_PANEL_WIDTH * 9) / 16, // 16:9, как у основного экрана
  };
  const TARS_PANEL_Y = 1.5;
  const TARS_PANEL_Z = -3.6;
  const TARS_PANEL_X = 2.7;
  const tars1Panel = createTars1TextPanel();
  tars1Panel.mesh.position.set(-TARS_PANEL_X, TARS_PANEL_Y, TARS_PANEL_Z);
  tars1Panel.mesh.scale.set(TARS_PANEL_SIZE.width, TARS_PANEL_SIZE.height, 1);
  // Back-tilt: нормаль направлена из центра экрана в оператора
  // (0, 1.6, 0). Разница по y: 1.6 - 1.5 = 0.1, по z: -3.6 - 0 = -3.6.
  // Плоскость по умолчанию смотрит в +Z, rotateY на atan2(x, z) даёт
  // нормаль в плоскости XZ. Здесь нужно ещё немного наклонить по X —
  // поднимаем низ экрана к оператору, верх — от него.
  {
    const dx = -tars1Panel.mesh.position.x; // 2.7 (положительный X)
    const dz = -tars1Panel.mesh.position.z; // 3.6 (положительный Z)
    tars1Panel.mesh.rotation.y = Math.atan2(dx, dz);
    // Наклон вверх (верх экрана чуть к стене): небольшой, чтобы текст
    // читался без запрокидывания головы.
    const dy = EYE_HEIGHT_M - TARS_PANEL_Y;
    const horizDist = Math.hypot(dx, dz);
    tars1Panel.mesh.rotation.x = -Math.atan2(dy, horizDist);
  }
  scene.add(tars1Panel.mesh);

  const tars2Panel = createTars2MetricsPanel();
  tars2Panel.mesh.position.set(TARS_PANEL_X, TARS_PANEL_Y, TARS_PANEL_Z);
  tars2Panel.mesh.scale.set(TARS_PANEL_SIZE.width, TARS_PANEL_SIZE.height, 1);
  // Симметричный back-tilt: оператор слева от FRONT CAM не появляется,
    // правый экран смотрит на него так же.
  {
    const dx = -tars2Panel.mesh.position.x; // -2.7
    const dz = -tars2Panel.mesh.position.z; // 3.6
    tars2Panel.mesh.rotation.y = Math.atan2(dx, dz);
    const dy = EYE_HEIGHT_M - TARS_PANEL_Y;
    const horizDist = Math.hypot(dx, dz);
    tars2Panel.mesh.rotation.x = -Math.atan2(dy, horizDist);
  }
  scene.add(tars2Panel.mesh);

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

  // Voice state indicator (AV-20): центр стены над экраном, между
  // status_hud и arm-sprite. Позиция (0, 2.85, -3.85) — выше main screen
  // (центр y=1.5) и не перекрывает ни ARM-sprite (x=2.35), ни status_hud
  // (x=-2.35). Размер 1.1 × 0.5 — компактнее, чем статус/ARM: это не
  // «главный HUD», а индикатор активности микрофона при работе с PTT на
  // гриппах (аудит §4-bis).
  const voiceIndicator: VoiceStateIndicator = createVoiceStateIndicator({
    position: { x: 0, y: 2.85, z: -3.85 },
    scale: { x: 1.1, y: 0.5 }
  });
  scene.add(voiceIndicator.sprite);

  // Phase 2.1 environment (loaded lazily via loadEnvironment()).
  let environment: BridgeAssetHandle | null = null;
  const environmentBaseUrl = opts.environmentBaseUrl === null ? null : (opts.environmentBaseUrl ?? "/models/environment/");
  async function loadEnvironment(): Promise<BridgeAssetHandle | null> {
    if (environment) return environment;
    if (environmentBaseUrl === null) return null;
    // Логотип на палубу. Грузится параллельно окружению и намеренно НЕ
    // ожидается: это декорация, из-за неё мостик не должен ждать.
    void floorOverlay.loadLogo();
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

  // Рамка наведения для КНОПОК. Видео-панели умеют подсвечиваться сами
  // (VideoPanel.setHighlight), а кнопки меню и панелей — это прозрачные
  // меши (opacity 0) поверх canvas-текстуры: попал в них луч или прошёл в
  // сантиметре мимо, картинка была одна и та же. Отсюда и «ложные
  // срабатывания»: промах оператор замечал только по результату клика.
  //
  // Рамка — один меш на всю сцену: он переезжает на наведённую цель,
  // повторяя её позу и размер. Дешевле, чем держать по подсветке на
  // каждую из десятков кнопок.
  const hoverFrame = new THREE.Mesh(
    new THREE.PlaneGeometry(1, 1),
    new THREE.MeshBasicMaterial({
      color: 0x8fd4ff,
      transparent: true,
      opacity: 0.22,
      depthTest: false,
      depthWrite: false,
      side: THREE.DoubleSide
    })
  );
  hoverFrame.visible = false;
  hoverFrame.renderOrder = 29;
  scene.add(hoverFrame);

  const hoverBox = new THREE.Box3();
  const hoverSize = new THREE.Vector3();

  /** Поставить рамку на цель под лучом (или спрятать, если цели нет). */
  function updateHoverFrame(): void {
    const id = pointer.getHovered();
    // Панели подсвечиваются своим тоном — рамка им не нужна и только
    // перекрывала бы кадр видео.
    if (id === null || videoPanels.has(id) || id === PIPELINE_DRAG_TARGET_ID) {
      hoverFrame.visible = false;
      return;
    }
    const target = pointer.getTarget(id);
    if (!target) {
      hoverFrame.visible = false;
      return;
    }
    const obj = target.object;
    obj.updateWorldMatrix(true, false);
    // Локальный bbox + мировая матрица цели: рамка совпадает с кнопкой,
    // как бы ни была повёрнута панель.
    hoverBox.setFromObject(obj, true);
    hoverBox.getSize(hoverSize);
    if (hoverSize.x <= 0 || hoverSize.y <= 0) {
      hoverFrame.visible = false;
      return;
    }
    obj.getWorldPosition(hoverFrame.position);
    obj.getWorldQuaternion(hoverFrame.quaternion);
    // Чуть больше кнопки — рамка читается как обводка, а не как заливка.
    hoverFrame.scale.set(hoverSize.x * 1.04 + 0.012, hoverSize.y * 1.04 + 0.012, 1);
    // Выносим на волос к оператору, чтобы не тонуть в текстуре панели.
    hoverFrame.translateZ(0.004);
    hoverFrame.visible = true;
  }

  function refreshHighlights(): void {
    for (const s of panelMgr.list()) {
      videoPanels.get(s.id)?.setHighlight(highlightFor(s.id, s.selected));
    }
    updateHoverFrame();
  }

  function initLayout(): void {
    // AV-25: пробуем восстановить сохранённую раскладку; если в
    // localStorage пусто/битый JSON/чужой version — берём дефолт.
    if (!applyStoredLayout()) {
      panelMgr.resetLayout();
    }
    syncPanels();
    restorePipelinePos();
  }

  // AV-25: дебаунс-сохранение раскладки (500мс после последнего
  // изменения). На каждый кадр драга писать в localStorage — слишком
  // дорого; на отпускание делаем flush, чтобы схваченный layout не
  // потерялся при экстренном закрытии страницы.
  function scheduleLayoutSave(): void {
    layoutSaver?.schedule(() => serializeLayout(panelMgr.list()));
  }
  function flushLayoutSave(): void {
    layoutSaver?.flush(() => serializeLayout(panelMgr.list()));
  }

  // AV-25-расширение: позиция панели голосового пайплайна тоже переживает
  // перезапуск клиента. Отдельный ключ — панель не видео-поток и живёт вне
  // PanelManager. Битый JSON/чужой version → дефолт (молча не молчим: warn).
  const PIPELINE_POS_STORAGE_KEY = "rob_box_quest.voice_pipeline_pos.v1";

  // Диагностика 2026-09-08 (nightly-review-fix, issue "voice button stuck
  // on main screen"): в отличие от panel_layout_store (там позиция всегда
  // пересчитывается из angleDeg + ФИКСИРОВАННОГО радиуса 2.0, см.
  // panel_layout_store.ts:positionFromAngleAndHeight — устойчиво к любым
  // изменениям геометрии), этот ключ хранит СЫРЫЕ мировые координаты и до
  // сих пор восстанавливал их без всякой проверки. Раскопки git log
  // показали: 2026-09-03 (b0c338a9) панель стала перетаскиваемой и её
  // позиция начала сохраняться; 2026-09-03..09-08 в PointerSystem.radiusOf
  // жил баг (issue #2143 / ADR-0072), тянувший панель к лицу оператора с
  // каждым повторным захватом ("после 3-4 захватов — 0.3 м от лица").
  // Баг в pointer.ts пофиксили (setCenter + честный 3D radiusOf), но САМ
  // ключ в localStorage — нет: если у оператора уже была захвачена
  // "убежавшая" позиция, она восстанавливается по сей день, и фикс #2150
  // на неё не влияет никак. Отсюда и "чиним - а на шлеме всё как было".
  //
  // Минимальная защита без версионирования (версия геометрии панели не
  // менялась после b0c338a9, так что version-bump тут не поможет сам по
  // себе): отбрасываем сохранённую позицию, если её 3D-расстояние от
  // дефолтного центра оператора (0, EYE_HEIGHT_M, 0) выходит за разумные
  // границы — либо "прилипло к лицу" (создуп-баг), либо улетело за пределы
  // мостика. Дефолтный радиус панели — VOICE_PIPELINE_RADIUS_M (2.4 м);
  // границы дают запас на осознанный драг оператора, но отсекают явный
  // мусор.
  const PIPELINE_POS_MIN_DIST_M = 0.8;
  const PIPELINE_POS_MAX_DIST_M = 4.0;

  function savePipelinePos(): void {
    if (!layoutStorage) return;
    const p = voicePipeline.getPosition();
    layoutStorage.setItem(
      PIPELINE_POS_STORAGE_KEY,
      JSON.stringify({ version: 1, x: p.x, y: p.y, z: p.z })
    );
  }

  function restorePipelinePos(): void {
    if (!layoutStorage) return;
    const raw = layoutStorage.getItem(PIPELINE_POS_STORAGE_KEY);
    if (!raw) return;
    try {
      const d = JSON.parse(raw) as { version?: number; x?: number; y?: number; z?: number };
      if (
        d &&
        d.version === 1 &&
        typeof d.x === "number" &&
        typeof d.y === "number" &&
        typeof d.z === "number"
      ) {
        const dist = Math.hypot(d.x - 0, d.y - EYE_HEIGHT_M, d.z - 0);
        if (dist < PIPELINE_POS_MIN_DIST_M || dist > PIPELINE_POS_MAX_DIST_M) {
          // eslint-disable-next-line no-console
          console.warn(
            `[captain_bridge] restorePipelinePos: сохранённая позиция на расстоянии ${dist.toFixed(2)} м ` +
              `от оператора вне допустимых границ [${PIPELINE_POS_MIN_DIST_M}, ${PIPELINE_POS_MAX_DIST_M}] м ` +
              "(похоже на наследие бага #2143 drag-creep) — игнорируем, стираем ключ, панель остаётся на дефолтной позиции"
          );
          layoutStorage.removeItem(PIPELINE_POS_STORAGE_KEY);
          return;
        }
        voicePipeline.setPosition(d.x, d.y, d.z);
      }
    } catch (err) {
      // eslint-disable-next-line no-console
      console.warn("[captain_bridge] restorePipelinePos: invalid JSON, using default", err);
    }
  }

  // AV-25: сброс к default по клавише R (desktop) или из help-overlay.
  // Стираем сохранённое и пересоздаём панели.
  function resetPanelLayout(): void {
    layoutStorage?.removeItem(PANEL_LAYOUT_STORAGE_KEY);
    layoutSaver?.cancel();
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
    // AV-25: пополняем реестр известных топиков — теперь parseLayout
    // примет сохранённую раскладку, даже если в ней ещё незнакомый
    // серверу топик (например, добавленная камера между стартом и
    // приходом stream_list).
    for (const r of rows) knownTopics.add(r.topic);
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

  // Видимый луч + курсор в точке попадания. Держим на уровне сцены, а не
  // как child контроллера: PointerSystem считает попадание в мировых
  // координатах, и рисовать надо ровно то, что он посчитал.
  const pointerBeam: PointerBeamHandle = createPointerBeam();
  scene.add(pointerBeam.object);

  function updatePointer(ray: PointerRay | null): void {
    // Сфера драга живёт вокруг головы оператора. Камера в VR может
    // уехать из (0, 1.6, 0) — local-floor mode позволяет отойти от
    // стартовой точки и наклониться. Без `setCenter` каждый кадр
    // панели катались бы вокруг начала координат, а не вокруг
    // оператора: чем дальше он ушёл, тем сильнее панель «прилипала
    // не туда» (issue #2143 §1.1). Центр обновляем ДО update(ray), и
    // в частности ДО первого `radiusOf(id)` в момент justPressed —
    // иначе при захвате панели в стороне от старта координат
    // `dragRadius` посчитается от старого центра.
    pointer.setCenter({
      x: camera.position.x,
      y: camera.position.y,
      z: camera.position.z
    });
    pointer.update(ray);
    pointerBeam.update(ray, pointer.getHit());
  }

  // ---------- AV-27: TTS picker (3D-меню выбора голоса) ----------
  //
  // Живёт рядом с экраном-стеной: оператор смотрит на видео, меню всплывает
  // левее, на том же радиусе. Вкладка VOICE висит постоянно — в VR клавиш
  // нет, точка входа обязана быть кликабельным объектом.

  const ttsPicker: TtsPickerMenuHandle = createTtsPickerMenu();
  scene.add(ttsPicker.object);
  scene.add(ttsPicker.launchObject);
  // Вкладка — левее и ниже экрана-стены, той же ориентации (facing +Z).
  ttsPicker.launchObject.position.set(-1.35, 0.95, -3.85);

  // Вкладка кликабельна всегда: цель регистрируется один раз.
  {
    const lt = ttsPicker.launchTarget();
    pointer.addTarget({ id: lt.id, object: lt.object, draggable: false });
  }

  /** Пере-регистрация целей меню: только пока оно открыто. */
  let ttsTargetIds: string[] = [];

  function syncTtsTargets(): void {
    for (const id of ttsTargetIds) pointer.removeTarget(id);
    ttsTargetIds = [];
    if (!ttsPicker.isVisible()) return;
    for (const t of ttsPicker.targets()) {
      pointer.addTarget({ id: t.id, object: t.object, draggable: false });
      ttsTargetIds.push(t.id);
    }
  }

  function renderTtsPicker(state: TtsPickerState): void {
    ttsPicker.render(state);
    // Набор активных целей зависит от состояния (APPLY/STOP гаснут,
    // строки появляются) — держим PointerSystem в синхроне.
    syncTtsTargets();
  }

  function openTtsPicker(): void {
    if (ttsPicker.isVisible()) return;
    // Ставим меню на позицию вкладки, чтобы оно оказалось на том же
    // радиусе и повороте, что панели (глубина слоя как у stream_menu).
    const p = ttsPicker.launchObject.position;
    ttsPicker.show(new THREE.Vector3(p.x, p.y, p.z), 0);
    syncTtsTargets();
  }

  function openTtsPickerNearPipeline(): void {
    if (ttsPicker.isVisible()) return;
    // Меню всплывает над панелью пайплайна и развёрнуто к оператору так же,
    // как панель — иначе оператор, смотрящий на панель, не увидит меню
    // (вкладка VOICE висит далеко слева у экрана-стены).
    const p = voicePipeline.getPosition();
    ttsPicker.show(new THREE.Vector3(p.x, p.y, p.z), voicePipeline.object.rotation.y);
    syncTtsTargets();
  }

  function closeTtsPicker(): void {
    if (!ttsPicker.isVisible()) return;
    ttsPicker.hide();
    syncTtsTargets();
  }

  function handleTtsTarget(action: TtsPickerTarget): void {
    if (action.kind === "launch") {
      if (ttsPicker.isVisible()) closeTtsPicker();
      else openTtsPicker();
    } else if (action.kind === "close") {
      closeTtsPicker();
    }
    // Остальные действия (select / preview / apply / stop) — забота
    // main.ts: только он знает про сокет и стор. Открытие/закрытие
    // обрабатываем здесь, потому что это чистая геометрия сцены.
    opts.onTtsPickerAction?.(action);
  }

  function videoTopics(): string[] {
    const topics = new Set<string>([MAIN_SCREEN_TOPIC, CEILING_SCREEN_TOPIC]);
    for (const s of panelMgr.list()) topics.add(s.topic);
    return [...topics];
  }

  function ingestMapFrame(payload: Uint8Array): boolean {
    return floorOverlay.ingestMapPayload(payload);
  }

  function ingestPanelFrame(topic: string, jpeg: Uint8Array): boolean {
    if (topic === MAIN_SCREEN_TOPIC) return mainScreen.ingestJpeg(jpeg);
    if (topic === CEILING_SCREEN_TOPIC) return ceilingScreen.ingestJpeg(jpeg);
    for (const vp of videoPanels.values()) {
      if (vp.topic === topic) return vp.ingestJpeg(jpeg);
    }
    return false;
  }

  function setRobotStatus(status: RobotStatus | null): void {
    statusHud.setStatus(status);
  }

  /**
   * Voice state (0x1202) → индикатор. Парсинг в чистой функции
   * (parseVoiceState) — битый payload не падает, мы просто его пропускаем.
   * До первого кадра показываем «—» (state="unknown").
   */
  function setVoiceState(payload: Uint8Array | null): VoiceStateFrame | null {
    if (!payload) {
      voiceIndicator.setState(null);
      return null;
    }
    const frame: VoiceStateFrame | null = parseVoiceState(payload);
    if (frame) voiceIndicator.setState(frame);
    return frame;
  }

  // ---------- render loop ----------

  let running = false;
  let raf = 0;
  // AV-25 / B4: FPS-счётчик. Скользящее среднее по 60 кадрам,
  // значение в HUD обновляется раз в 500мс (а не на каждый кадр —
  // иначе цифра дрожит, а текстура перерисовывается 90 раз/сек).
  const fpsMeter = new FpsMeter({ windowSize: 60 });

  function tickFps(now: number): void {
    fpsMeter.push(now);
    if (!fpsMeter.shouldUpdate(500, now)) return;
    const v = fpsMeter.value();
    statusHud.setFps(v > 0 ? v : null);
    fpsMeter.markUpdated(now);
  }

  function loop(): void {
    if (!running) return;
    raf = requestAnimationFrame(loop);
    tickFps(performance.now());
    renderer.render(scene, camera);
  }

  function startDesktopLoop(): void {
    if (running) return;
    running = true;
    loop();
  }

  function stopDesktopLoop(): void {
    if (!running) return;
    running = false;
    cancelAnimationFrame(raf);
  }

  function start(): () => void {
    if (running) return () => undefined;
    startDesktopLoop();
    return stopDesktopLoop;
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
      // Луч НЕ рисуем здесь: раньше на каждый контроллер вешалась Line
      // фиксированной длины 1.5 м, которая не доставала до панелей (они
      // на 2.4–4 м) и была того же цвета, что подсветка кнопок. Теперь
      // луч один, общий, и рисует его pointerBeam — по реальному
      // попаданию активной руки (см. interaction/pointer_beam.ts).
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

    // Десктопный цикл ОБЯЗАН остановиться: с этого момента кадры рисует
    // XR-цикл рендерера, и второй render() в тот же canvas — это лишний
    // проход по уже привязанному XR-фреймбуферу. Три.js в xr.enabled
    // рисует через ArrayCamera и оставляет viewport последнего глаза, так
    // что чужой кадр попадает в ОДИН глаз — оператор видит периодическую
    // рябь слева (или справа). window.requestAnimationFrame в immersive-vr
    // браузер обычно тормозит, но «обычно» — не гарантия: на Quest он
    // просыпается (смена фокуса, системный оверлей, выход в 2D), и рябь
    // приходит ровно этими всплесками.
    stopDesktopLoop();
    renderer.setAnimationLoop(() => {
      // FPS считаем и в VR: HUD должен показывать частоту XR-кадров, а не
      // замороженную цифру от последнего десктопного кадра.
      tickFps(performance.now());
      renderer.render(scene, camera);
    });

    // Вышли из VR — десктопный цикл поднимаем обратно, иначе на странице
    // остаётся мёртвая картинка.
    session.addEventListener("end", () => {
      renderer.setAnimationLoop(null);
      renderer.xr.enabled = false;
      startDesktopLoop();
    });
  }

  function dispose(): void {
    window.removeEventListener("resize", resize);
    stopDesktopLoop();
    renderer.setAnimationLoop(null);
    layoutSaver?.cancel();
    mainScreen.dispose();
    ceilingScreen.dispose();
    for (const vp of videoPanels.values()) vp.dispose();
    lidar.dispose();
    floorOverlay.dispose();
    streamMenu?.dispose();
    ttsPicker.dispose();
    pointerBeam.dispose();
    hoverFrame.geometry.dispose();
    (hoverFrame.material as THREE.Material).dispose();
    environment?.dispose();
    armTexture.dispose();
    statusHud.dispose();
    supervisorPanel.dispose();
    voicePipeline.dispose();
    voiceIndicator.dispose();
    tars1Panel.dispose();
    tars2Panel.dispose();
    renderer.dispose();
  }

  return {
    scene,
    renderer,
    camera,
    lidar,
    floor: floorOverlay,
    panels: panelMgr,
    videoPanels,
    mainScreen,
    ceilingScreen,
    environment,
    loadEnvironment,
    initLayout,
    resetPanelLayout,
    updatePointer,
    pointer,
    supervisorPanel,
    voicePipeline,
    tars1Panel,
    tars2Panel,
    setAvailableStreams,
    renderTtsPicker,
    openTtsPicker,
    openTtsPickerNearPipeline,
    closeTtsPicker,
    isTtsPickerOpen: () => ttsPicker.isVisible(),
    attachXrSession,
    setControllerActive,
    setArmState,
    statusHud,
    videoTopics,
    ingestPanelFrame,
    ingestMapFrame,
    setRobotStatus,
    setVoiceState,
    start,
    resize,
    dispose
  };
}