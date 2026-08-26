// Bootstrap: подключение к WSS, инициализация сцены, teleop, UI.
//
// Phase 1.5: WebXR-вход (immersive-vr). Если navigator.xr есть и устройство
// поддерживает режим — в stream_select появляется блок "XR Mode" с кнопками
// Enter/Exit VR. По клику (user-activation обязателен!) → requestSession →
// attachXrSession → Three.js XR-рендер + XR-контроллеры читаются в frame loop
// и кормят TeleopFSM.
//
// Phase 2: ModeManager + HUD + boost-gated desktop teleop + deadman warning
// wiring + XR right-stick camera + ramp handoff при переключении режима.

import { Connection } from "./wire/connection";
import { createCaptainBridge } from "./scene/captain_bridge";
import { TeleopFSM } from "./input/teleop_fsm";
import { createDesktopTeleop } from "./input/desktop_teleop";
import { createXrTeleop, pollXrInput, tickXrTeleop } from "./input/xr_teleop";
import { DeadmanTimer, DEADMAN_RELEASE_MS } from "./input/deadman_timer";
import { TeleopCmdRamp } from "./input/teleop_cmd_ramp";
import { createCameraController } from "./scene/camera_controller";
import { createXrBootstrap, type XrBootstrap } from "./xr_bootstrap";
import { createStreamSelect } from "./ui/stream_select";
import { ModeManager, CAPTAIN_MODES } from "./modes/mode_manager";
import { createModeHud } from "./modes/mode_hud";
import { createPanelHover } from "./scene/panel_hover";
import {
  getControllerWorldPose,
  pickPanelByRay,
  isControllerInputSource,
  type PanelRaycastTarget
} from "./input/xr_panel_raycast";
import { createXrHandController, type XrHandLike } from "./input/xr_hand_controller";
import * as THREE from "three";
import GUI from "lil-gui";
import type { StreamMeta } from "./wire/messages";

const CLIENT_VERSION = "0.1.0";
const SUBPROTOCOL = "robbox-quest-v1";

interface BootstrapOptions {
  url?: string;
  pin: string;
  canvas: HTMLCanvasElement;
  pinOverlay: HTMLElement;
  pinInput: HTMLInputElement;
  pinForm: HTMLFormElement;
  pinError: HTMLElement;
  statusEl: HTMLElement;
}

export function bootstrap(opts: BootstrapOptions): { dispose(): void } {
  const url = opts.url ?? deriveWsUrl();
  const bridge = createCaptainBridge({ canvas: opts.canvas, enableXr: true });
  bridge.initLayout();
  const stopRender = bridge.start();

  const fsm = new TeleopFSM();
  // Phase 2: ModeManager + HUD + boost-gated desktop teleop.
  const modeManager = new ModeManager();
  const modeHud = createModeHud();
  modeHud.attachModeManager(modeManager);
  // 50ms ramp handoff — плавный переход teleop cmd при смене режима.
  const cmdRamp = new TeleopCmdRamp();
  // Камера (yaw/pitch + damping + reset).
  const camera = createCameraController();
  // Deadman таймер для XR-источника — warning 300ms, release 500ms.
  const xrDeadman = new DeadmanTimer({ releaseMs: DEADMAN_RELEASE_MS });
  // Phase 2 §3.7: panel hover FSM + Raycaster (используется в XR frame loop).
  const panelHover = createPanelHover();
  const raycaster = new THREE.Raycaster();
  // При переключении режима в mixed/teleop — ramp к нулю (старый cmd затухает),
  // потом новый cmd ramp'ится от нуля к target.
  let lastRampedMode: string = modeManager.getMode();
  modeManager.subscribe(() => {
    const cur = modeManager.getMode();
    // Если уходим из teleop/mixed в explore/voice — ramp к нулю.
    const wasDriving = lastRampedMode === "teleop" || lastRampedMode === "mixed";
    const isDriving = cur === "teleop" || cur === "mixed";
    if (wasDriving && !isDriving) {
      cmdRamp.setTarget({ linear: 0, angular: 0 });
    } else if (!wasDriving && isDriving) {
      // Въезжаем в driving mode — пусть ramp подхватит текущий teleop.
      // (Caller запишет target через setLinear/setAngular в tick.)
    }
    // Reset deadman warning при смене режима (чтобы не висел stale warning).
    if (cur !== lastRampedMode) {
      modeHud.setWarning(null);
      xrDeadman.reset();
    }
    lastRampedMode = cur;
  });
  const desktopTeleop = createDesktopTeleop({
    fsm,
    modeManager,
    boostEnabled: () => modeManager.getMode() === "teleop" || modeManager.getMode() === "mixed"
  });
  const xr: XrBootstrap = createXrBootstrap();

  // WASD intent → auto-upgrade voice → mixed (только в voice-режиме).
  window.addEventListener("keydown", (ev) => {
    if (
      ["KeyW", "KeyA", "KeyS", "KeyD", "ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight"].includes(ev.code) &&
      modeManager.getMode() === "voice"
    ) {
      modeManager.reportTeleopIntent();
    }
  });

  let conn: Connection | null = null;
  let streamSelect: ReturnType<typeof createStreamSelect> | null = null;
  let disconnected = true;
  let xrTeleopHandle: ReturnType<typeof createXrTeleop> | null = null;
  let xrTeleopTick: ReturnType<typeof tickXrTeleop> | null = null;
  // Последний кэшированный inputSources; обновляется через inputsourceschange.
  let xrInputSources: XRInputSource[] = [];

  function setStatus(text: string, cls: "connected" | "connecting" | "lost"): void {
    opts.statusEl.textContent = text;
    opts.statusEl.className = `status status--${cls}`;
    streamSelect?.setConnectionStatus(text, cls);
  }

  function openConnection(): void {
    conn = new Connection(
      {
        url,
        subprotocol: SUBPROTOCOL,
        clientVersion: CLIENT_VERSION,
        capabilities: ["webxr"],
        pin: opts.pin
      },
      {
        onStateChange: (state) => {
          if (state === "connected") {
            disconnected = false;
            setStatus("CONNECTED", "connected");
            // Подписаться на дефолтные видео-панели + lidar.
            for (const topic of ["camera_rear", "camera_oak_color", "camera_oak_depth", "camera_ceiling", "lidar_2d"]) {
              conn!.subscribe(topic);
            }
            conn!.requestStreamList();
            // Открыть PIN-overlay hide.
            opts.pinOverlay.classList.add("pin-overlay--hidden");
          } else if (state === "auth_failed") {
            setStatus("WRONG PIN", "lost");
            opts.pinOverlay.classList.remove("pin-overlay--hidden");
            opts.pinError.hidden = false;
          } else if (state === "reconnecting") {
            setStatus("RECONNECTING…", "connecting");
          } else if (state === "connecting") {
            setStatus("CONNECTING…", "connecting");
          } else if (state === "closed") {
            setStatus("CLOSED", "lost");
            disconnected = true;
          }
        },
        onBinaryFrame: (streamId, payload) => {
          const topic = conn!.getTopicForStream(streamId);
          if (!topic) return;
          if (topic === "lidar_2d") {
            bridge.lidar.ingestPayload(payload);
            return;
          }
          if (
            topic === "camera_rear" ||
            topic === "camera_oak_color" ||
            topic === "camera_oak_depth" ||
            topic === "camera_ceiling" ||
            topic === "camera_front"
          ) {
            for (const [panelId, vp] of bridge.videoPanels.entries()) {
              const panelState = bridge.panels.get(panelId);
              if (panelState && panelState.topic === topic) {
                vp.ingestJpeg(payload);
              }
            }
            return;
          }
          // Other binary topics → ignore в Phase 1.5.
        },
        onStreamList: (items: StreamMeta[]) => {
          streamSelect?.setAvailableStreams(items);
        },
        onError: (code, message) => {
          // Только фатальные ошибки показываем в HUD; reconnect-попытки тихо в консоль.
          if (code === "AUTH_FAIL") return;
          if (code === "RECONNECT") {
            // eslint-disable-next-line no-console
            console.warn("[quest] reconnect:", message);
            return;
          }
          // eslint-disable-next-line no-console
          console.warn(`[quest] server error ${code}: ${message}`);
        }
      }
    );
    conn.connect();
  }

  // stream_select UI создаём один раз, но callbacks ссылаются на текущий conn.
  streamSelect = createStreamSelect({
    panels: bridge.panels,
    onSubscribe: (topic) => conn?.subscribe(topic),
    onUnsubscribe: (topic) => conn?.unsubscribe(topic),
    onResetLayout: () => {
      // Сохраняем текущие подписки и пересоздаём с теми же темами.
      const topics = bridge.panels.list().map((p) => p.topic);
      for (const t of topics) conn?.unsubscribe(t);
      bridge.initLayout();
      for (const t of topics) conn?.subscribe(t);
      streamSelect!.refresh();
    },
    getActiveTopics: () => bridge.panels.list().map((p) => p.topic),
    xr,
    onEnterVr: () => enterVr(),
    onExitVr: () => exitVr()
  });

  // Phase 2: отдельный lil-gui dropdown для Captain Mode.
  // Располагаем рядом со stream_select через shared parent — оба создают
  // свой GUI; пользователь увидит два stack-окна. Это OK для Phase 2; в
  // Phase 3 можно объединить.
  const modeGui = new GUI({ title: "rob_box_quest / captain_mode", width: 280 });
  const modeProxy = { mode: modeManager.getMode() as string };
  const modeCtrl = modeGui
    .add(modeProxy, "mode", [...CAPTAIN_MODES])
    .name("Mode")
    .onChange((v: string) => {
      if (typeof v === "string") {
        modeManager.requestMode(v as Parameters<typeof modeManager.requestMode>[0], "ui_select");
        modeProxy.mode = modeManager.getMode();
        modeCtrl.updateDisplay();
      }
    });
  modeGui.add({ cycle: () => modeManager.cycleNext("hotkey") }, "cycle").name("Cycle (M / A)");
  modeManager.subscribe(() => {
    modeProxy.mode = modeManager.getMode();
    modeCtrl.updateDisplay();
  });

  setStatus("CONNECTING…", "connecting");

  // PIN form.
  opts.pinForm.addEventListener("submit", (ev) => {
    ev.preventDefault();
    const v = opts.pinInput.value.trim();
    if (!/^\d{6}$/.test(v)) {
      opts.pinError.textContent = "PIN must be 6 digits";
      opts.pinError.hidden = false;
      return;
    }
    opts.pinError.hidden = true;
    opts.pin = v;
    openConnection();
  });

  // Teleop loop.
  let lastTickTs = 0;
  function teleopLoop(): void {
    const now = performance.now();
    if (now - lastTickTs >= 33) {
      lastTickTs = now;
      // Получить intent (linear/angular + deadman) — приоритет: XR > desktop.
      const desktopHandle = (window as unknown as {
        __questDesktopTeleop?: {
          getIntent(): { linear: number; angular: number };
          isDeadmanHeld(): boolean;
        };
      }).__questDesktopTeleop;
      const xrActive = xr.isActive() && xrInputSources.length > 0;
      let intent: { linear: number; angular: number; deadman: boolean } | null = null;
      let xrDummyState: {
        linear: number;
        angular: number;
        cameraYaw: number;
        cameraPitch: number;
        deadman: boolean;
        emergencyEdge: boolean;
        modeCycleEdge: boolean;
        resetCameraEdge: boolean;
      } | null = null;
      if (xrActive) {
        xrDummyState = {
          linear: 0,
          angular: 0,
          cameraYaw: 0,
          cameraPitch: 0,
          deadman: false,
          emergencyEdge: false,
          modeCycleEdge: false,
          resetCameraEdge: false
        };
        intent = { linear: 0, angular: 0, deadman: false };
        for (const src of xrInputSources) {
          if (src && (src as { gamepad?: { axes?: number[] } }).gamepad) {
            pollXrInput(xrDummyState, src, fsm, intent);
          }
        }
      } else if (desktopHandle) {
        const i = desktopHandle.getIntent();
        intent = { linear: i.linear, angular: i.angular, deadman: desktopHandle.isDeadmanHeld() };
      }
      const isDriving = modeManager.getMode() === "teleop" || modeManager.getMode() === "mixed";
      if (isDriving && intent) {
        // §3.3: ramp cmd через TeleopCmdRamp (50ms handoff).
        cmdRamp.setTarget(intent);
      } else if (!isDriving) {
        cmdRamp.setTarget({ linear: 0, angular: 0 });
      }
      const ramped = cmdRamp.tick();
      if (isDriving) {
        fsm.setLinear(ramped.linear);
        fsm.setAngular(ramped.angular);
        fsm.setDeadman(intent?.deadman ?? false);
      } else {
        fsm.setLinear(0);
        fsm.setAngular(0);
        fsm.setDeadman(false);
      }
      if (conn && !disconnected) {
        const teleopHandle = (window as unknown as { __questDesktopTeleop?: { consumeEmergency(): { source: string } | null } }).__questDesktopTeleop;
        if (teleopHandle?.consumeEmergency()) {
          conn.send(fsm.triggerEmergency("ui_button"));
        }
        const out = fsm.tick(Date.now());
        if (out) conn.send(out.cmd);
      }
      // XR-side edge actions (mode cycle, camera, deadman).
      if (xrActive && xrDummyState) {
        xrTeleopTick?.consumeModeCycleEdge();
        if (xrTeleopTick?.consumeResetCameraEdge()) {
          camera.reset();
        }
        const cam = xrTeleopTick?.getCameraAxes();
        if (cam) {
          const dtMs = 33;
          camera.applyStickAxes(cam.yaw, cam.pitch, dtMs);
          camera.tickDamping(dtMs);
        }
        // Deadman: проверим grip (используем состояние, проставленное pollXrInput).
        if (xrDummyState.deadman) {
          xrDeadman.gripPressed();
        } else {
          const ev = xrDeadman.check();
          if (ev && ev.kind === "triggered") {
            modeHud.setWarning(null);
            if (conn && !disconnected) conn.send(fsm.triggerEmergency("ui_button"));
            // §3.3: deadman release + no voice → downgrade mixed → teleop.
            modeManager.reportDeadmanReleased();
          } else if (ev && ev.kind === "warning") {
            modeHud.setWarning(`Release B in ${Math.round(ev.remainingMs)}ms`);
          }
        }
      }
    }
    requestAnimationFrame(teleopLoop);
  }
  requestAnimationFrame(teleopLoop);

  // ---------- WebXR entry / exit ----------

  async function enterVr(): Promise<void> {
    try {
      const session = await xr.requestSession("immersive-vr");
      xr.bindSession(session);
      // refetch input sources каждый раз при добавлении/удалении контроллеров.
      session.addEventListener("inputsourceschange", () => {
        xrInputSources = Array.from(session.inputSources);
      });
      xrInputSources = Array.from(session.inputSources);
      await bridge.attachXrSession(session);
      // Phase 2 §3.7: подсветка панели при наведении (raycast от controller).
      // Подписчик применяет hover через VideoPanel.setHover().
      panelHover.subscribe({
        onHoverEnter(panelId) {
          bridge.videoPanels.get(panelId)?.setHover(true);
        },
        onHoverExit(panelId) {
          bridge.videoPanels.get(panelId)?.setHover(false);
        }
      });
      // Phase 2 §3.5: hand controller — pinch/grip ловим на каждом XR frame.
      const handCtl = createXrHandController({
        onPinchStart(hand) {
          // eslint-disable-next-line no-console
          console.info(`[quest] hand ${hand} pinch start`);
        },
        onPinchEnd(hand) {
          // eslint-disable-next-line no-console
          console.info(`[quest] hand ${hand} pinch end`);
        }
      });
      // Phase 2 §3.7/§3.5: единый XR-frame callback.
      bridge.setOnXrFrame((frame, s) => {
        const sources = Array.from(s.inputSources);
        // 1) Hand tracking.
        for (const src of sources) {
          const hand = (src as unknown as { hand?: XrHandLike }).hand;
          if (hand) {
            try {
              handCtl.update(hand, frame);
            } catch (err) {
              // eslint-disable-next-line no-console
              console.warn("[quest] hand update threw:", err);
            }
          }
        }
        // 2) Controller raycast → panel hover (ближайшая попавшая панель = hovered).
        // Несколько контроллеров: используем тот, что ближе к панели.
        const targets: PanelRaycastTarget[] = bridge.getPanelRaycastTargets();
        let bestHover: { panelId: string; distance: number } | null = null;
        for (const src of sources) {
          if (!isControllerInputSource(src)) continue;
          const pose = getControllerWorldPose(
            src as unknown as Parameters<typeof getControllerWorldPose>[0],
            frame as unknown as Parameters<typeof getControllerWorldPose>[1]
          );
          if (!pose) continue;
          const hit = pickPanelByRay(raycaster, pose.position, pose.direction, targets, 6);
          if (hit && (bestHover == null || hit.distance < bestHover.distance)) {
            bestHover = hit;
          }
        }
        const nextHoverId = bestHover ? bestHover.panelId : null;
        if (nextHoverId !== panelHover.getHovered()) {
          panelHover.setHovered(nextHoverId);
        }
      });
      const xrOpts: Parameters<typeof createXrTeleop>[0] = {
        fsm,
        session,
        modeManager,
        onEmergency: (source) => {
          // Emergency от B-button ИЛИ deadman triggered → отправляем stop_emergency.
          if (conn && !disconnected) conn.send(fsm.triggerEmergency(source));
          modeHud.setWarning(null);
        },
        onDeadmanWarning: (remainingMs) => {
          modeHud.setWarning(`Release B in ${Math.round(remainingMs)}ms`);
        },
        onDeadmanTriggered: (_elapsedMs) => {
          modeHud.setWarning(null);
        }
      };
      xrTeleopHandle = createXrTeleop(xrOpts);
      // tickXrTeleop читает __xrDeadmanTimer/__xrTeleopState из opts (createXrTeleop их сохранил).
      xrTeleopTick = tickXrTeleop(xrOpts);
      streamSelect?.setXrSessionState("in-vr");
    } catch (err) {
      // eslint-disable-next-line no-console
      console.warn("[quest] requestSession failed:", err);
      streamSelect?.setXrSessionState(`failed: ${(err as Error).message}`);
    }
  }

  async function exitVr(): Promise<void> {
    try {
      await xr.endSession();
    } finally {
      xrTeleopHandle?.destroy();
      xrTeleopHandle = null;
      xrTeleopTick = null;
      xrInputSources = [];
      xrDeadman.reset();
      modeHud.setWarning(null);
      panelHover.reset();
      bridge.setOnXrFrame(null);
      streamSelect?.setXrSessionState("not-in-vr");
    }
  }

  return {
    dispose(): void {
      stopRender();
      desktopTeleop.destroy();
      xrTeleopHandle?.destroy();
      streamSelect?.destroy();
      modeGui.destroy();
      modeHud.destroy();
      conn?.close();
      bridge.dispose();
    }
  };
}

function deriveWsUrl(): string {
  const proto = location.protocol === "https:" ? "wss:" : "ws:";
  return `${proto}//${location.host}/quest`;
}