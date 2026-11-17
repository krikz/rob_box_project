// Навигационный слой мостика (issue #3151, Captain Bridge волна 2) — клей
// между данными (поза из map_2d, путь nav_path, события nav_*), жестом
// цели и отрисовкой (scene/nav_overlay.ts). Транспорта не знает: команды
// отдаёт через `send`, как остальные панели мостика (сцена про WSS не знает).
//
// Всё, что хранится, хранится в `map`; в сцену перекладывается по
// последней позе на каждом её обновлении (~5 Гц) — эгоцентричный мостик:
// оператор стоит в base_link, мир едет вокруг.

import type { NavCancelCmd, NavGoalCmd } from "../wire/protocol_generated";
import type { StatusLine } from "../scene/status_hud";
import {
  NAV_CANCEL_TARGET_ID,
  createNavOverlay,
  type NavOverlayHandle,
  type NavPin
} from "../scene/nav_overlay";
import type { PointerSystem } from "../interaction/pointer";
import {
  mapToScene,
  mapYawToSceneRotY,
  sceneToMap,
  wrapAngle,
  type Pose2D,
  type SceneXz,
  type Xy
} from "./nav_frames";
import { NavGoalGesture, type GestureRay } from "./nav_goal_gesture";
import { parseNavPath } from "./nav_path_payload";
import {
  INITIAL_NAV_STATE,
  isGoalLive,
  navStatusLine,
  reasonText,
  reduceNav,
  type NavAction,
  type NavState
} from "./nav_state";
import { OdomTrail } from "./odom_trail";

/** Поза старше — цель из неё не считаем (робот мог уехать). */
export const POSE_MAX_AGE_MS = 1500;
/** Сколько держать на HUD итог цели («ПРИЕХАЛ», «ОТКАЗ…»), потом строка гаснет. */
export const TERMINAL_LINE_MS = 10_000;

export interface NavLayerOptions {
  /** Отправить команду на сервер. `false` — связи нет, команда не ушла. */
  send(cmd: NavGoalCmd | NavCancelCmd): boolean;
  /** Строка NAV для status-HUD (`null` — убрать). */
  onStatusLine?(line: StatusLine | null): void;
  /** Короткое уведомление оператору (тост). */
  notify?(text: string, level: "info" | "warn"): void;
  /** Слой указателя — для кнопки отмены. */
  pointer?: PointerSystem;
  /** Часы (тесты). */
  now?(): number;
}

export interface NavLayerHandle {
  readonly overlay: NavOverlayHandle;
  /** Новая поза робота из map_2d (`null` — позы нет). */
  setPose(pose: Pose2D | null): void;
  /** nav_path (0x1104). `false` — кадр битый. */
  ingestPathPayload(payload: Uint8Array): boolean;
  /** JSON_EVENT nav_*: `true` — событие наше и обработано. */
  handleEvent(event: unknown): boolean;
  /** Кадр указателя. `blocked` — луч сейчас на панели/кнопке. */
  updatePointer(ray: GestureRay | null, blocked: boolean): void;
  /** Клик по цели указателя с prefix `nav:`. */
  handleSelect(id: string): void;
  toggleAim(): void;
  isAiming(): boolean;
  /** Отменить текущую цель (кнопка / клавиша). */
  cancel(): void;
  /** Связь потеряна: статус цели больше не факт. */
  onDisconnected(): void;
  state(): NavState;
  dispose(): void;
}

export function createNavLayer(opts: NavLayerOptions): NavLayerHandle {
  const now = opts.now ?? (() => Date.now());
  const overlay = createNavOverlay();
  const gesture = new NavGoalGesture();
  const trail = new OdomTrail();

  let pose: Pose2D | null = null;
  let poseAtMs = 0;
  let pathMap: Xy[] = [];
  let state: NavState = INITIAL_NAV_STATE;
  let terminalAtMs: number | null = null;
  let aiming = false;
  let seq = 0;
  let cancelRegistered = false;
  let lastLineKey = "";

  function dispatch(action: NavAction): void {
    const prev = state;
    state = reduceNav(state, action);
    if (state === prev) return;
    terminalAtMs = isGoalLive(state) || state.phase === "unknown" ? null : now();
    if (!isGoalLive(state) && state.phase !== "unknown") pathMap = [];
    refresh();
  }

  function pin(p: Xy, yawMap: number): NavPin | null {
    if (!pose) return null;
    return { point: mapToScene(pose, p), rotY: mapYawToSceneRotY(pose, yawMap) };
  }

  function syncCancelTarget(visible: boolean): void {
    overlay.setCancelVisible(visible);
    if (!opts.pointer || visible === cancelRegistered) return;
    if (visible) {
      opts.pointer.addTarget({ id: NAV_CANCEL_TARGET_ID, object: overlay.cancelButton, draggable: false });
    } else {
      opts.pointer.removeTarget(NAV_CANCEL_TARGET_ID);
    }
    cancelRegistered = visible;
  }

  function pushStatusLine(): void {
    const t = now();
    const stale = terminalAtMs !== null && t - terminalAtMs > TERMINAL_LINE_MS;
    const line = stale && !aiming ? null : navStatusLine(state, aiming);
    const key = line ? `${line.value}|${line.level}` : "";
    if (key === lastLineKey) return;
    lastLineKey = key;
    opts.onStatusLine?.(line);
  }

  /** Переложить всё из `map` в сцену по текущей позе. */
  function refresh(): void {
    const live = isGoalLive(state) || state.phase === "unknown";
    if (!pose) {
      overlay.setPath([]);
      overlay.setTrail([]);
      overlay.setGoal(null);
    } else {
      const p = pose;
      overlay.setPath(live ? pathMap.map((q) => mapToScene(p, q)) : []);
      overlay.setTrail(
        trail.samples(now()).map((s) => ({ ...mapToScene(p, s), freshness: s.freshness }))
      );
      overlay.setGoal(live && state.goal ? pin(state.goal, state.goal.yaw) : null);
    }
    syncCancelTarget(live && state.goal !== null);
    pushStatusLine();
  }

  function sendGoal(point: SceneXz, yawBase: number): void {
    if (!pose || now() - poseAtMs > POSE_MAX_AGE_MS) {
      opts.notify?.("Nav-цель не отправлена: нет свежей позы робота", "warn");
      return;
    }
    const m = sceneToMap(pose, point);
    seq += 1;
    const goal = { seq, x: m.x, y: m.y, yaw: wrapAngle(pose.yaw + yawBase) };
    const cmd: NavGoalCmd = { cmd: "nav_goal", ts_ms: Date.now(), frame: "map", ...goal };
    if (!opts.send(cmd)) {
      opts.notify?.("Nav-цель не отправлена: нет связи", "warn");
      return;
    }
    dispatch({ kind: "sent", goal });
  }

  function cancel(): void {
    const cmd: NavCancelCmd = { cmd: "nav_cancel", ts_ms: Date.now() };
    if (!opts.send(cmd)) opts.notify?.("Отмена не отправлена: нет связи", "warn");
  }

  function handleEvent(event: unknown): boolean {
    const ev = event as Record<string, unknown> | null;
    const type = typeof ev?.type === "string" ? ev.type : "";
    if (!ev || !type.startsWith("nav_")) return false;
    const num = (v: unknown): number | null => (typeof v === "number" && Number.isFinite(v) ? v : null);
    if (type === "nav_goal_ack") {
      const s = num(ev.seq);
      if (s !== null) dispatch({ kind: "ack", seq: s });
    } else if (type === "nav_goal_nack") {
      const reason = typeof ev.reason === "string" ? ev.reason : "unknown";
      dispatch({ kind: "nack", seq: num(ev.seq), reason });
      opts.notify?.(`Nav-цель не принята: ${reasonText(reason)}`, "warn");
    } else if (type === "nav_status") {
      const s = num(ev.seq);
      const x = num(ev.x);
      const y = num(ev.y);
      const yaw = num(ev.yaw);
      if (s === null || x === null || y === null || yaw === null || typeof ev.state !== "string") return true;
      dispatch({
        kind: "status",
        state: ev.state,
        seq: s,
        x,
        y,
        yaw,
        distanceM: num(ev.distance_remaining),
        reason: typeof ev.reason === "string" ? ev.reason : null
      });
    } else if (type === "nav_cancel_ack") {
      if (ev.had_goal === false) opts.notify?.("Отменять нечего: активной цели нет", "info");
    } else {
      return false;
    }
    return true;
  }

  return {
    overlay,
    setPose(next) {
      pose = next;
      if (next) {
        poseAtMs = now();
        trail.push(next, poseAtMs);
      }
      refresh();
    },
    ingestPathPayload(payload) {
      const frame = parseNavPath(payload);
      if (!frame) return false;
      pathMap = frame.points;
      refresh();
      return true;
    },
    handleEvent,
    updatePointer(ray, blocked) {
      const out = gesture.update({ armed: aiming, ray, blocked });
      overlay.setReticle(out.reticle);
      const preview = out.preview ?? out.commit;
      overlay.setPreview(preview && pose ? { point: preview.point, rotY: preview.yaw } : null);
      if (out.commit) {
        aiming = false; // одна взводка — одна цель
        overlay.setPreview(null);
        sendGoal(out.commit.point, out.commit.yaw);
      }
      // Итог цели гаснет по таймеру — проверяем в кадре указателя.
      if (terminalAtMs !== null) pushStatusLine();
    },
    handleSelect(id) {
      if (id === NAV_CANCEL_TARGET_ID) cancel();
    },
    toggleAim() {
      aiming = !aiming;
      if (!aiming) gesture.reset();
      overlay.setReticle(null);
      overlay.setPreview(null);
      pushStatusLine();
    },
    isAiming: () => aiming,
    cancel,
    onDisconnected() {
      aiming = false;
      gesture.reset();
      dispatch({ kind: "disconnected" });
      pushStatusLine();
    },
    state: () => state,
    dispose() {
      if (cancelRegistered) opts.pointer?.removeTarget(NAV_CANCEL_TARGET_ID);
      overlay.dispose();
    }
  };
}
