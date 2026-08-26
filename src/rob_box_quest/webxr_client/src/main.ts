// Bootstrap: подключение к WSS, инициализация сцены, teleop, UI.
//
// Phase 1.5: WebXR-вход (immersive-vr). Если navigator.xr есть и устройство
// поддерживает режим — в stream_select появляется блок "XR Mode" с кнопками
// Enter/Exit VR. По клику (user-activation обязателен!) → requestSession →
// attachXrSession → Three.js XR-рендер + XR-контроллеры читаются в frame loop
// и кормят TeleopFSM.
//
// Phase 2 добавляет:
//   - Voice picker (lil-gui справа-сверху, на отдельной панели)
//   - Voice state (список голосов, set_voice, preview_voice, indicator)
//   - Panel layout modes (single/split/PIP/2x2) с цикл-кнопкой
//   - Drag-from-gui → drop-on-panel переназначение топика
//   - HUD voice indicator (top-right)
//   - Voice preview audio (WebAudio)
//   - ModeManager + HUD + boost-gated desktop teleop + deadman warning
//   wiring + XR right-stick camera + ramp handoff при переключении режима.

import { Connection } from "./wire/connection";
import { createCaptainBridge } from "./scene/captain_bridge";
import { TeleopFSM } from "./input/teleop_fsm";
import { createDesktopTeleop } from "./input/desktop_teleop";
import { createXrTeleop, pollXrInput, tickXrTeleop } from "./input/xr_teleop";
import { DeadmanTimer, DEADMAN_RELEASE_MS } from "./input/deadman_timer";
import { TeleopCmdRamp } from "./input/teleop_cmd_ramp";
import { createCameraController } from "./scene/camera_controller";
import { createXrBootstrap, type XrBootstrap } from "./xr_bootstrap";
import { createStreamSelect, readDropTopic } from "./ui/stream_select";
import { createVoicePicker } from "./ui/voice_picker_panel";
import { PreviewPlayer } from "./audio/preview_player";
import { cycleLayout, type LayoutMode } from "./scene/panel_layout_modes";
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
import type { JsonCmd, StreamMeta, VoiceInfo, VoicePreset } from "./wire/messages";
import * as THREE from "three";
import GUI from "lil-gui";

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
  /** HUD voice indicator (опционально — для обратной совместимости). */
  voiceIndicatorEl?: HTMLElement;
}

interface PanelLayoutBinding {
  mode: LayoutMode;
  topics: string[];
}

export function bootstrap(opts: BootstrapOptions): { dispose(): void } {
  const url = opts.url ?? deriveWsUrl();
  const bridge = createCaptainBridge({ canvas: opts.canvas, enableXr: true });
  bridge.initLayout();
  const stopRender = bridge.start();
  // Phase 2.1: load Captain Bridge CC0 environment (5 GLB + HDR).
  // Fail-soft — see captain_bridge.ts → loadEnvironment(). The procedural
  // fallback floor + grid stays in place if the GLB fetch fails.
  bridge.loadEnvironment().catch((err) => {
    // eslint-disable-next-line no-console
    console.warn("[bootstrap] bridge environment load rejected:", err);
  });

  // Phase 2: voice state.
  const previewPlayer = new PreviewPlayer();
  let knownVoices: VoiceInfo[] = [];
  let currentVoice: { voice_id: string; preset: VoicePreset } | null = null;
  let pendingPreview: { request_id: string; chunks: Uint8Array[]; format: string } | null = null;

  // Phase 2: panel layout bindings.
  const panelLayouts = new Map<string, PanelLayoutBinding>();
  // По умолчанию каждый panel — single mode с одной темой (его текущей).
  for (const p of bridge.panels.list()) {
    panelLayouts.set(p.id, { mode: "single", topics: [p.topic] });
  }

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
  let voicePicker: ReturnType<typeof createVoicePicker> | null = null;
  let disconnected = true;
  let xrTeleopHandle: ReturnType<typeof createXrTeleop> | null = null;
  let xrTeleopTick: ReturnType<typeof tickXrTeleop> | null = null;
  // Последний кэшированный inputSources; обновляется через inputsourceschange.
  let xrInputSources: XRInputSource[] = [];

  // ---- HUD helpers ----------------------------------------------------------

  function setStatus(text: string, cls: "connected" | "connecting" | "lost"): void {
    opts.statusEl.textContent = text;
    opts.statusEl.className = `status status--${cls}`;
    streamSelect?.setConnectionStatus(text, cls);
    voicePicker?.setConnectionOnline(cls === "connected");
  }

  function setVoiceIndicatorText(text: string): void {
    if (opts.voiceIndicatorEl) opts.voiceIndicatorEl.textContent = text;
  }

  // ---- Server interactions --------------------------------------------------

  function sendVoiceList(): void {
    if (!conn) return;
    const cmd: JsonCmd = { cmd: "list_voices", ts_ms: Date.now() };
    conn.send(cmd);
  }

  function sendSetVoice(voice_id: string, preset: VoicePreset): void {
    if (!conn) return;
    const cmd: JsonCmd = {
      cmd: "set_voice",
      ts_ms: Date.now(),
      voice_id,
      preset
    };
    conn.send(cmd);
  }

  function sendPreviewVoice(voice_id: string, text: string, request_id: string): void {
    if (!conn) return;
    const cmd: JsonCmd = {
      cmd: "preview_voice",
      ts_ms: Date.now(),
      voice_id,
      text,
      request_id
    };
    conn.send(cmd);
    pendingPreview = { request_id, chunks: [], format: "audio/mpeg" };
    voicePicker?.setPreviewState("playing");
  }

  // ---- JSON_EVENT handlers --------------------------------------------------

  function handleVoiceList(voices: VoiceInfo[]): void {
    knownVoices = voices;
    voicePicker?.setVoices(voices);
  }

  function handleVoiceSetAck(voice_id: string, preset: VoicePreset): void {
    currentVoice = { voice_id, preset };
    voicePicker?.setCurrentVoice(voice_id, preset);
    setVoiceIndicatorText(`Voice: ${voice_id} (${preset})`);
  }

  function handleVoiceSetNack(reason: string): void {
    // eslint-disable-next-line no-console
    console.warn("[quest] set_voice nack:", reason);
    voicePicker?.setPreviewState("error");
  }

  function handlePreviewAudio(event: {
    request_id: string;
    format: string;
    content_type: string;
    seq: number;
    total: number;
  }): void {
    if (!pendingPreview || pendingPreview.request_id !== event.request_id) return;
    pendingPreview.format = event.content_type || "audio/mpeg";
    // Audio bytes придут в BINARY_FRAME с тем же stream_id mapping —
    // для простоты Phase 2: первый BINARY_FRAME после preview_voice_audio
    // (для stream_id 0x0001..0x0FFF, выделенного под preview) считаем
    // частью текущего preview. Сервер шлёт chunk-by-chunk в JSON_EVENT
    // + BINARY_FRAME; мы собираем.
    void event.seq;
    void event.total;
  }

  function handlePreviewBinary(payload: Uint8Array): void {
    if (!pendingPreview) return;
    pendingPreview.chunks.push(payload);
  }

  function handlePreviewDone(): void {
    if (!pendingPreview) {
      voicePicker?.setPreviewState("idle");
      return;
    }
    const format = pendingPreview.format;
    const total = mergeChunks(pendingPreview.chunks);
    pendingPreview = null;
    previewPlayer
      .play(total, format)
      .then(() => {
        // previewPlayer сам управляет состоянием через onended.
      })
      .catch((err: Error) => {
        // eslint-disable-next-line no-console
        console.warn("[quest] preview play failed:", err);
        voicePicker?.setPreviewState("error");
      });
  }

  function handlePreviewError(reason: string): void {
    pendingPreview = null;
    previewPlayer.stop();
    voicePicker?.setPreviewState("error");
    // eslint-disable-next-line no-console
    console.warn("[quest] preview error:", reason);
  }

  // ---- Connection lifecycle -------------------------------------------------

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
            for (const topic of [
              "camera_rear",
              "camera_oak_color",
              "camera_oak_depth",
              "camera_ceiling",
              "lidar_2d"
            ]) {
              conn!.subscribe(topic);
            }
            conn!.requestStreamList();
            sendVoiceList();
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
            voicePicker?.setConnectionOnline(false);
          }
        },
        onBinaryFrame: (streamId, payload) => {
          // Phase 2: BINARY_FRAME во время preview → append к pending.
          if (pendingPreview) {
            handlePreviewBinary(payload);
            return;
          }
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
        },
        onStreamList: (items: StreamMeta[]) => {
          streamSelect?.setAvailableStreams(items);
        },
        onJsonEvent: (event) => {
          const type = (event as { type?: string }).type;
          switch (type) {
            case "voice_list":
              handleVoiceList((event as { voices: VoiceInfo[] }).voices ?? []);
              break;
            case "voice_set_ack":
              handleVoiceSetAck(
                (event as { voice_id: string }).voice_id,
                (event as { preset: VoicePreset }).preset
              );
              break;
            case "voice_set_nack":
              handleVoiceSetNack((event as { reason: string }).reason);
              break;
            case "preview_voice_audio":
              handlePreviewAudio(
                event as {
                  request_id: string;
                  format: string;
                  content_type: string;
                  seq: number;
                  total: number;
                }
              );
              break;
            case "preview_voice_done":
              handlePreviewDone();
              break;
            case "preview_voice_error":
              handlePreviewError((event as { reason: string }).reason);
              break;
            default:
              break;
          }
        },
        onError: (code, message) => {
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

  // previewPlayer → onended → setPreviewState("idle")
  previewPlayer.setListeners({
    onPlayingChange: (state) => {
      if (state === "idle") voicePicker?.setPreviewState("idle");
      else if (state === "error") voicePicker?.setPreviewState("error");
    },
    onError: (reason) => {
      voicePicker?.setPreviewState("error");
      // eslint-disable-next-line no-console
      console.warn("[quest] preview:", reason);
    }
  });

  // ---- stream_select UI -----------------------------------------------------

  streamSelect = createStreamSelect({
    panels: bridge.panels,
    onSubscribe: (topic) => conn?.subscribe(topic),
    onUnsubscribe: (topic) => conn?.unsubscribe(topic),
    onResetLayout: () => {
      const topics = bridge.panels.list().map((p) => p.topic);
      for (const t of topics) conn?.unsubscribe(t);
      bridge.initLayout();
      for (const t of topics) conn?.subscribe(t);
      // Сбрасываем layouts → single.
      for (const [id] of panelLayouts) {
        const p = bridge.panels.get(id);
        panelLayouts.set(id, { mode: "single", topics: p ? [p.topic] : [] });
      }
      streamSelect!.refresh();
    },
    onDropTopic: (panelId, topic) => {
      const panel = bridge.panels.get(panelId);
      if (!panel) return;
      const oldTopic = panel.topic;
      bridge.panels.switchStream(panelId, topic);
      // Layout topics тоже обновляем — slot[0] теперь новая тема.
      const layout = panelLayouts.get(panelId);
      if (layout) {
        const newTopics = [...layout.topics];
        const idx = newTopics.indexOf(oldTopic);
        if (idx >= 0) newTopics[idx] = topic;
        else newTopics[0] = topic;
        panelLayouts.set(panelId, { mode: layout.mode, topics: newTopics });
      }
      conn?.unsubscribe(oldTopic);
      conn?.subscribe(topic);
      // Серверу — set_panel_topic (для телеметрии / логирования на стороне сервера).
      const cmd: JsonCmd = {
        cmd: "set_panel_topic",
        ts_ms: Date.now(),
        panel_id: panelId,
        topic
      };
      conn?.send(cmd);
      streamSelect!.refresh();
    },
    onCyclePanelLayout: (panelId, currentMode) => {
      const next = cycleLayout(currentMode);
      const layout = panelLayouts.get(panelId) ?? { mode: "single", topics: [] };
      // Если текущий layout 1-slot, а новый требует >1 — расширяем topics до текущих panel-тем.
      // Минимально: просто сохраняем список (captain_bridge сам решит что показывать).
      panelLayouts.set(panelId, { mode: next, topics: layout.topics });
      streamSelect!.setPanelLayoutMode(panelId, next);
      // Серверу шлём для логирования / синхронизации (опционально).
      // TODO: можно добавить отдельную команду если сервер захочет поддержать layouts.
    },
    getActiveTopics: () => bridge.panels.list().map((p) => p.topic),
    getPanelLayoutModes: () => {
      const m = new Map<string, LayoutMode>();
      for (const [id, b] of panelLayouts) m.set(id, b.mode);
      return m;
    },
    xr,
    onEnterVr: (): void => {
      // fire-and-forget; ошибки обработаны внутри enterVr.
      void enterVr();
    },
    onExitVr: (): void => {
      void exitVr();
    }
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

  // ---- voice_picker UI ------------------------------------------------------

  if (opts.voiceIndicatorEl) {
    voicePicker = createVoicePicker({
      getVoices: () => knownVoices,
      getCurrentVoice: () => currentVoice,
      isOnline: () => !disconnected,
      voiceIndicatorEl: opts.voiceIndicatorEl,
      onSetVoice: ({ voice_id, preset }) => sendSetVoice(voice_id, preset),
      onPreviewVoice: ({ voice_id, text, request_id }) =>
        sendPreviewVoice(voice_id, text, request_id),
      onRequestVoices: () => sendVoiceList()
    });
  }

  // ---- PIN form -------------------------------------------------------------

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

  // ---- Drop-on-panel raycast (3D) -------------------------------------------
  // Phase 2 §6.2: HTML5 drag-and-drop на canvas → THREE.Raycaster →
  // определяем panel id → applyDropTopic.
  // Работает в desktop (mouse coords) и в VR (controller ray — отдельная карточка).

  function attachPanelDropHandlers(): void {
    const canvas = opts.canvas;
    canvas.addEventListener("dragover", (ev) => {
      ev.preventDefault();
      if (ev.dataTransfer) ev.dataTransfer.dropEffect = "move";
    });
    canvas.addEventListener("drop", (ev) => {
      ev.preventDefault();
      const topic = readDropTopic(ev);
      if (!topic) return;
      // Raycast-заглушка. Реальная логика — отдельная карточка.
      const panelId = pickPanelFromPointer(ev.clientX, ev.clientY);
      if (!panelId) return;
      applyDropTopic(panelId, topic);
    });
  }

  function applyDropTopic(panelId: string, topic: string): void {
    const panel = bridge.panels.get(panelId);
    if (!panel) return;
    const oldTopic = panel.topic;
    if (oldTopic === topic) return;
    bridge.panels.switchStream(panelId, topic);
    const layout = panelLayouts.get(panelId);
    if (layout) {
      const newTopics = [...layout.topics];
      const idx = newTopics.indexOf(oldTopic);
      if (idx >= 0) newTopics[idx] = topic;
      else newTopics[0] = topic;
      panelLayouts.set(panelId, { mode: layout.mode, topics: newTopics });
    }
    conn?.unsubscribe(oldTopic);
    conn?.subscribe(topic);
    const cmd: JsonCmd = {
      cmd: "set_panel_topic",
      ts_ms: Date.now(),
      panel_id: panelId,
      topic
    };
    conn?.send(cmd);
    streamSelect?.refresh();
  }

  function pickPanelFromPointer(clientX: number, clientY: number): string | null {
    // Raycast через THREE.Raycaster по meshes из bridge.videoPanels.
    const rect = opts.canvas.getBoundingClientRect();
    const ndcX = ((clientX - rect.left) / rect.width) * 2 - 1;
    const ndcY = -((clientY - rect.top) / rect.height) * 2 + 1;
    const raycaster = new THREE.Raycaster();
    raycaster.setFromCamera({ x: ndcX, y: ndcY } as THREE.Vector2, bridge.camera);
    // Соберём список mesh'ей с id-маркером.
    const meshes: Array<{ mesh: THREE.Object3D; panelId: string }> = [];
    for (const [panelId, vp] of bridge.videoPanels) {
      vp.mesh.userData["panelId"] = panelId;
      meshes.push({ mesh: vp.mesh, panelId });
    }
    if (meshes.length === 0) return null;
    const intersects = raycaster.intersectObjects(
      meshes.map((m) => m.mesh),
      false
    );
    if (intersects.length === 0) return null;
    const hit = intersects[0];
    const panelId = (hit.object.userData["panelId"] as string | undefined) ?? null;
    return panelId;
  }

  attachPanelDropHandlers();

  // ---- Teleop loop -----------------------------------------------------------

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
      voicePicker?.destroy();
      previewPlayer.dispose();
      modeGui.destroy();
      modeHud.destroy();
      conn?.close();
      bridge.dispose();
    }
  };
}

function mergeChunks(chunks: Uint8Array[]): Uint8Array {
  let total = 0;
  for (const c of chunks) total += c.byteLength;
  const out = new Uint8Array(total);
  let off = 0;
  for (const c of chunks) {
    out.set(c, off);
    off += c.byteLength;
  }
  return out;
}

function deriveWsUrl(): string {
  const proto = location.protocol === "https:" ? "wss:" : "ws:";
  return `${proto}//${location.host}/quest`;
}
