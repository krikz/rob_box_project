// Bootstrap: подключение к WSS, инициализация сцены, teleop, WebXR auto-entry, overlays.
//
// Вход на мостик — только PIN-форма. После ввода PIN:
//   - если браузер поддерживает WebXR (immersive-vr) — сразу входим в VR
//     (requestSession вызывается в user-activation submit handler);
//   - иначе остаёмся в desktop-режиме (WASD fallback + 2D-рендер).
//
// Phase 2 интеграция:
//   - loading_screen показывает пока грузятся CC0 GLB/HDR ассеты;
//   - error_overlay сообщает о disconnect > 5s и ошибках сервера;
//   - help_overlay (H key) — список горячих клавиш;
//   - mode_manager — клиентский стор UI-состояния (voice mode / armed / current voice).
// Debug-панелей (lil-gui) больше нет — вход только через PIN-форму.

import { Connection } from "./wire/connection";
import { createCaptainBridge } from "./scene/captain_bridge";
import { parseRobotStatus } from "./scene/status_hud";
import { TeleopFSM } from "./input/teleop_fsm";
import { createDesktopTeleop } from "./input/desktop_teleop";
import { createXrTeleop, pollXrInput } from "./input/xr_teleop";
import { createVoiceCapture } from "./input/voice_capture";
import { createXrBootstrap, type XrBootstrap } from "./xr_bootstrap";
import { createDesktopPointer } from "./interaction/desktop_pointer";
import { xrPointerRay } from "./interaction/xr_pointer";
import { createLoadingScreen } from "./ui/loading_screen";
import {
  createErrorOverlay,
  createDisconnectWatchdog,
  type ErrorOverlay,
  type DisconnectWatchdog
} from "./ui/error_overlay";
import { createHelpOverlay, type HelpOverlay } from "./ui/help_overlay";
import { createModeManager, type ClientModeManager } from "./ui/mode_manager";

const CLIENT_VERSION = "0.1.0";
const SUBPROTOCOL = "robbox-quest-v1";

// Не-видео стримы. Список видео-топиков берём у сцены (`videoTopics()`),
// чтобы подписка не разъезжалась с тем, что она реально умеет показать.
const NON_VIDEO_TOPICS = ["lidar_2d", "robot_status", "voice_state"];

interface BootstrapOptions {
  url?: string;
  pin: string;
  canvas: HTMLCanvasElement;
  pinOverlay: HTMLElement;
  pinInput: HTMLInputElement;
  pinForm: HTMLFormElement;
  pinError: HTMLElement;
  statusEl: HTMLElement;
  /** Контейнер для overlay'ов (loading/error/help). Обычно document.body. */
  body: HTMLElement;
  /** Опциональная кнопка "?" в HUD; клик тогглит help overlay. */
  helpToggle?: HTMLElement | null;
}

export function bootstrap(opts: BootstrapOptions): { dispose(): void } {
  const url = opts.url ?? deriveWsUrl();

  // Phase 2.3 overlays — создаём ДО старта асинхронных pipeline'ов,
  // чтобы loading-screen сразу перекрыл экран пока грузятся CC0 GLB.
  const loading = createLoadingScreen(opts.body, "Loading environment…");
  const errorOverlay: ErrorOverlay = createErrorOverlay(opts.body);
  const help: HelpOverlay = createHelpOverlay(opts.body);
  const modeManager: ClientModeManager = createModeManager();
  const watchdog: DisconnectWatchdog = createDisconnectWatchdog(errorOverlay);

  // HUD: подключаем help-toggle кнопку (если есть) к help overlay.
  if (opts.helpToggle) {
    opts.helpToggle.addEventListener("click", () => help.toggle());
  }
  // Mode-manager → voice/teleop состояние. Сейчас в HUD напрямую не
  // показываем (Phase 2.3 не делал voice picker UI — голос управляется
  // гриппами XR-контроллеров), но mode_manager используется ниже
  // для синхронизации с реальным состоянием и для будущих подписчиков.
  modeManager.on(() => {
    // no-op placeholder: реальный HUD-индикатор добавим в Phase 3,
    // здесь только проверяем, что стор жив и listener работает.
  });

  const bridge = createCaptainBridge({
    canvas: opts.canvas,
    enableXr: true,
    // Панель сменила стрим через меню (R10): подписываемся на новый топик
    // и отписываемся от старого, если его больше никто не показывает.
    onPanelTopicChange: (_panelId, oldTopic, newTopic) => {
      if (!conn || disconnected) return;
      conn.subscribe(newTopic);
      const stillUsed = bridge.videoTopics().includes(oldTopic);
      if (!stillUsed) conn.unsubscribe(oldTopic);
    }
  });
  bridge.initLayout();
  const stopRender = bridge.start();
  // Phase 2.1: load Captain Bridge CC0 environment (5 GLB + HDR).
  // Fail-soft — see captain_bridge.ts → loadEnvironment(). The procedural
  // fallback floor + grid stays in place if the GLB fetch fails.
  // loading.watch прячет overlay при успехе или показывает ошибку при сбое.
  void loading.watch(bridge.loadEnvironment(), "Loading environment…");

  // Указатель: на десктопе — мышь через камеру, в VR — луч контроллера
  // (см. XR-цикл ниже). Панели наводятся, выбираются и перетаскиваются.
  const desktopPointer = createDesktopPointer({ canvas: opts.canvas, camera: bridge.camera });

  const fsm = new TeleopFSM();
  const desktopTeleop = createDesktopTeleop({ fsm });
  const xr: XrBootstrap = createXrBootstrap();

  let conn: Connection | null = null;
  let disconnected = true;
  let xrTeleopHandle: ReturnType<typeof createXrTeleop> | null = null;
  // Последний кэшированный inputSources; обновляется через inputsourceschange.
  let xrInputSources: XRInputSource[] = [];
  // XR-кадровый цикл teleop (session.requestAnimationFrame) — window.rAF
  // в immersive-vr заморожен. xrRafSession нужен для cancelAnimationFrame.
  let xrRafSession: XRSession | null = null;
  let xrRafId = 0;
  // Edge-trigger для B/Y: шлём emergency один раз на нажатие.
  let xrEmergencyWasPressed = false;
  // Guard: авто-вход в VR — не более одной сессии на submit PIN.
  let vrRequested = false;
  // Голос: рация (правый grip) и робот-голос (левый grip → STT → LLM → TTS)
  // делят один mic-захват (int16 PCM 16 kHz) → VOICE_AUDIO. Режим кодируется
  // в voice_ptt_start/stop как `mode` ("radio" | "robot_voice").
  const voiceCapture = createVoiceCapture({
    onChunk: (pcm) => {
      if (!conn || disconnected) return;
      const bytes = new Uint8Array(pcm.buffer, pcm.byteOffset, pcm.byteLength);
      conn.sendVoiceAudio(bytes);
    }
  });
  // Edge-состояние PTT. Робот-голос приоритетнее рации, если зажаты оба.
  let voicePttMode: "none" | "radio" | "robot_voice" = "none";

  function applyVoicePtt(radio: boolean, robot: boolean): void {
    const next: "none" | "radio" | "robot_voice" = robot ? "robot_voice" : radio ? "radio" : "none";
    if (next === voicePttMode) return;
    const c = conn;
    const send = c !== null && !disconnected;

    // Закрыть предыдущий PTT-режим.
    if (voicePttMode !== "none" && send) {
      c!.send({ cmd: "voice_ptt_stop", mode: voicePttMode, ts_ms: Date.now() });
    }
    // Режим голоса ПЕРСИСТЕНТНЫЙ: при входе в робот-голос ставим ttts_proxy
    // (применяет супервизор, ADR-0028 S5) и НЕ сбрасываем на отпускание —
    // иначе STT, дораспознающий уже после release, приходит при
    // voice_input_mode=respeaker и dialogue_node его игнорирует (гонка).
    voicePttMode = next;
    if (next === "none") {
      voiceCapture.stop();
      // Mode-manager: клиентский UI-state — "off".
      modeManager.setVoiceMode("off");
      return;
    }
    if (next === "robot_voice" && send) {
      c!.send({ cmd: "voice_mode", mode: "ttts_proxy", ts_ms: Date.now() });
    }
    if (send) {
      c!.send({ cmd: "voice_ptt_start", mode: next, ts_ms: Date.now() });
    }
    // Mode-manager: клиентский UI-state — текущий voice mode.
    modeManager.setVoiceMode(next);
    void voiceCapture.start();
  }

  // ---- HUD helpers ----------------------------------------------------------

  function setStatus(text: string, cls: "connected" | "connecting" | "lost"): void {
    opts.statusEl.textContent = text;
    opts.statusEl.className = `status status--${cls}`;
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
            // Phase 2.3: ошибка прячется при восстановлении коннекта.
            watchdog.markConnected();
            for (const topic of [...bridge.videoTopics(), ...NON_VIDEO_TOPICS]) {
              conn!.subscribe(topic);
            }
            // Каталог стримов → меню выбора на панелях (R10).
            conn!.requestStreamList();
            opts.pinOverlay.classList.add("pin-overlay--hidden");
          } else if (state === "auth_failed") {
            setStatus("WRONG PIN", "lost");
            opts.pinOverlay.classList.remove("pin-overlay--hidden");
            opts.pinError.hidden = false;
            // Если уже вошли в VR с неверным PIN — выходим обратно к форме.
            void exitVr();
          } else if (state === "reconnecting") {
            setStatus("RECONNECTING…", "connecting");
            // Старый RTT после разрыва — враньё: обнуляем до первого pong.
            bridge.statusHud.setRtt(null);
            // Disconnect-watchdog начинает отсчёт; если > 5s без успеха —
            // покажем error overlay (см. createDisconnectWatchdog).
            watchdog.markDisconnected();
          } else if (state === "connecting") {
            setStatus("CONNECTING…", "connecting");
          } else if (state === "closed") {
            setStatus("CLOSED", "lost");
            bridge.statusHud.setRtt(null);
            disconnected = true;
            // "closed" — окончательно (не reconnect). Прямо сейчас
            // показываем overlay без 5-секундного порога.
            errorOverlay.show("Disconnected", "Connection closed by server");
          }
        },
        onBinaryFrame: (streamId, payload) => {
          const topic = conn!.getTopicForStream(streamId);
          if (!topic) return;
          if (topic === "lidar_2d") {
            bridge.lidar.ingestPayload(payload);
            return;
          }
          if (topic === "robot_status") {
            bridge.setRobotStatus(parseRobotStatus(payload));
            return;
          }
          if (topic === "voice_state") {
            // AV-20: voice_state (0x1202) → центральный HUD-индикатор.
            // Парсинг внутри bridge.setVoiceState — битый payload не падает.
            bridge.setVoiceState(payload);
            return;
          }
          // Видео: экран-стена и боковые панели (Wave 3.A).
          bridge.ingestPanelFrame(topic, payload);
        },
        onRtt: (rttMs) => {
          bridge.statusHud.setRtt(rttMs);
        },
        onStreamList: (items) => {
          bridge.setAvailableStreams(
            items.map((it) => ({ topic: it.topic, description: it.description }))
          );
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
    // WebXR requestSession требует user-activation — вызываем сразу из submit,
    // пока активация ещё действительна.
    void autoEnterVr();
  });

  // ---- AV-25: глобальные хоткеи мостика ----
  //
  // R — сброс раскладки панелей к default (стирает localStorage и
  // пересоздаёт панели). Слушаем на document, чтобы работало и в VR
  // (XR-сессия не глушит document keydown), и в desktop-режиме.
  document.addEventListener("keydown", (ev) => {
    if (ev.repeat) return;
    const target = ev.target as HTMLElement | null;
    const tag = target?.tagName?.toLowerCase();
    if (tag === "input" || tag === "textarea" || target?.isContentEditable) return;
    if (ev.key === "r" || ev.key === "R") {
      ev.preventDefault();
      bridge.resetPanelLayout();
    }
  });

  // ---- Teleop loop -----------------------------------------------------------

  let lastTickTs = 0;

  // Полинг XR-контроллеров: агрегируем по всем inputSources — arm-клик =
  // любой правый стик, стик-оси = контроллер с наибольшим отклонением,
  // B/Y = любой нажат. Arm/disarm: клик правого стика тогглит активацию
  // телеопа. DISARM по умолчанию.
  let armed = false;
  let xrArmWasPressed = false;

  function pollXrControllers(): void {
    if (!xr.isActive() || xrInputSources.length === 0) {
      xrEmergencyWasPressed = false;
      xrArmWasPressed = false;
      applyVoicePtt(false, false);
      bridge.setControllerActive(false);
      return;
    }
    let armPress = false;
    let emergency = false;
    let ptt = false;
    let robotPtt = false;
    let linear = 0;
    let angular = 0;
    let bestMag = -1;
    for (const src of xrInputSources) {
      const r = pollXrInput(src);
      armPress = armPress || r.armPress;
      emergency = emergency || r.emergency;
      ptt = ptt || r.ptt;
      robotPtt = robotPtt || r.robotPtt;
      const mag = r.linear * r.linear + r.angular * r.angular;
      if (mag > bestMag) {
        bestMag = mag;
        linear = r.linear;
        angular = r.angular;
      }
    }
    // Edge-triggered toggle: нажал стик → ARM, нажал ещё раз → DISARM.
    if (armPress && !xrArmWasPressed) {
      armed = !armed;
      bridge.setArmState(armed);
      // Phase 2.3: mode-manager синхронизируется с реальным arm-стейтом.
      modeManager.setTeleopState(armed ? "armed" : "disarmed");
    }
    xrArmWasPressed = armPress;
    fsm.setDeadman(armed);
    fsm.setLinear(linear);
    fsm.setAngular(angular);
    bridge.setControllerActive(armed);
    if (emergency && !xrEmergencyWasPressed && conn && !disconnected) {
      conn.send(fsm.triggerEmergency("controller_b"));
      // B/Y — жёсткий стоп: локально дизармимся, HUD отражает реальность.
      armed = false;
      bridge.setArmState(false);
      modeManager.setTeleopState("disarmed");
    }
    xrEmergencyWasPressed = emergency;
    applyVoicePtt(ptt, robotPtt);
  }

  // Один тик teleop: desktop-emergency + FSM-tick + XR-контроллеры.
  // Вызывается из двух циклов: window.rAF (desktop) и session.rAF (VR).
  function tickTeleop(): void {
    if (conn && !disconnected) {
      const teleopHandle = (
        window as unknown as { __questDesktopTeleop?: { consumeEmergency(): boolean } }
      ).__questDesktopTeleop;
      if (teleopHandle?.consumeEmergency()) {
        conn.send(fsm.triggerEmergency("ui_button"));
      }
      const out = fsm.tick(Date.now());
      if (out) conn.send(out.cmd);
    }
    pollXrControllers();
  }

  function teleopLoop(): void {
    const now = performance.now();
    if (now - lastTickTs >= 33) {
      lastTickTs = now;
      tickTeleop();
    }
    // В immersive-vr этот цикл заморожен, указатель там тикает из
    // XR-кадра (ниже) — здесь только desktop-мышь.
    if (!xr.isActive()) bridge.updatePointer(desktopPointer.poll());
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
      xrTeleopHandle = createXrTeleop({ fsm, session });
      // window.requestAnimationFrame в immersive-vr заморожен браузером,
      // поэтому teleop тикаем в XR-кадровом цикле (session.requestAnimationFrame).
      xrRafSession = session;
      const refSpace = await session.requestReferenceSpace("local-floor").catch(() =>
        session.requestReferenceSpace("local")
      );
      const xrFrame = (_time: DOMHighResTimeStamp, frame: XRFrame): void => {
        if (!xr.isActive()) return;
        tickTeleop();
        // Луч указателя — из targetRaySpace активного контроллера.
        bridge.updatePointer(refSpace ? xrPointerRay(frame, refSpace, xrInputSources) : null);
        xrRafId = session.requestAnimationFrame(xrFrame);
      };
      xrRafId = session.requestAnimationFrame(xrFrame);
    } catch (err) {
      // eslint-disable-next-line no-console
      console.warn("[quest] requestSession failed:", err);
    }
  }

  async function exitVr(): Promise<void> {
    try {
      await xr.endSession();
    } finally {
      if (xrRafSession && xrRafId) {
        xrRafSession.cancelAnimationFrame(xrRafId);
      }
      xrRafSession = null;
      xrRafId = 0;
      xrEmergencyWasPressed = false;
      armed = false;
      xrArmWasPressed = false;
      bridge.setArmState(false);
      // Phase 2.3: синхронизируем UI-state.
      modeManager.setTeleopState("disarmed");
      applyVoicePtt(false, false);
      xrTeleopHandle?.destroy();
      xrTeleopHandle = null;
      xrInputSources = [];
      vrRequested = false;
    }
  }

  async function autoEnterVr(): Promise<void> {
    if (vrRequested || xr.isActive()) return;
    vrRequested = true;
    try {
      if (!(await xr.isSupported("immersive-vr"))) {
        vrRequested = false;
        return;
      }
    } catch {
      // WebXR недоступен (desktop) — остаёмся в 2D-режиме.
      vrRequested = false;
      return;
    }
    await enterVr();
  }

  return {
    dispose(): void {
      stopRender();
      desktopTeleop.destroy();
      if (xrRafSession && xrRafId) {
        xrRafSession.cancelAnimationFrame(xrRafId);
      }
      xrTeleopHandle?.destroy();
      desktopPointer.destroy();
      voiceCapture.stop();
      conn?.close();
      bridge.dispose();
      // Phase 2.3 overlays cleanup.
      loading.dispose();
      errorOverlay.dispose();
      help.dispose();
      watchdog.dispose();
    }
  };
}

function deriveWsUrl(): string {
  const proto = location.protocol === "https:" ? "wss:" : "ws:";
  return `${proto}//${location.host}/quest`;
}
