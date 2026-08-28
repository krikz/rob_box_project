// Bootstrap: подключение к WSS, инициализация сцены, teleop, WebXR auto-entry.
//
// Вход на мостик — только PIN-форма. После ввода PIN:
//   - если браузер поддерживает WebXR (immersive-vr) — сразу входим в VR
//     (requestSession вызывается в user-activation submit handler);
//   - иначе остаёмся в desktop-режиме (WASD fallback + 2D-рендер).
//
// Debug-панелей (lil-gui) больше нет — вход только через PIN-форму.

import { Connection } from "./wire/connection";
import { TelemetryReporter } from "./wire/telemetry";
import { createCaptainBridge, MAIN_SCREEN_TOPIC } from "./scene/captain_bridge";
import { TeleopFSM } from "./input/teleop_fsm";
import { createDesktopTeleop } from "./input/desktop_teleop";
import { createXrTeleop, pollXrInput } from "./input/xr_teleop";
import { createVoiceCapture } from "./input/voice_capture";
import { createXrBootstrap, type XrBootstrap } from "./xr_bootstrap";

const CLIENT_VERSION = "0.1.0";
const SUBPROTOCOL = "robbox-quest-v1";

const DEFAULT_SUBSCRIBED_TOPICS = [
  MAIN_SCREEN_TOPIC,
  "lidar_2d"
];

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
  // Phase 2.1: load Captain Bridge CC0 environment (5 GLB + HDR).
  // Fail-soft — see captain_bridge.ts → loadEnvironment(). The procedural
  // fallback floor + grid stays in place if the GLB fetch fails.
  bridge.loadEnvironment().catch((err) => {
    // eslint-disable-next-line no-console
    console.warn("[bootstrap] bridge environment load rejected:", err);
  });
  // Phase 2.2: load avatar (Draco + Meshopt compressed GLB via
  // GLTFLoader + DRACOLoader + KTX2Loader + MeshoptDecoder, see
  // src/scene/asset_loader.ts). Fail-soft like the environment — the
  // procedural origin marker stays in place if the GLB fetch fails.
  bridge.loadAvatar().catch((err) => {
    // eslint-disable-next-line no-console
    console.warn("[bootstrap] avatar load rejected:", err);
  });

  // Phase 2.2 telemetry reporter (ADR-0032 §3.5): 1 Hz tick → WSS
  // 0x40 TELEMETRY_PERF → server republishes в ROS2 /quest/perf.
  // Battery-friendly: один setInterval + ~30 строк в нём, без busy-loop.
  // GPU probe опционален (EXT_disjoint_timer_query_webgl2 нет в Quest
  // Browser reliably) — если недоступен, gpu_ms просто null в payload.
  const telemetry = new TelemetryReporter({ intervalMs: 1000, fpsWindow: 60 });
  // WebGL2 context получаем из renderer.domElement. Captain Bridge всегда
  // создаёт WebGLRenderer; Three.js использует WebGL2 если доступно, иначе
  // фолбэчит на WebGL1. Для EXT_disjoint_timer_query_webgl2 probe нам
  // нужен WebGL2 — если renderer дал WebGL1, GPU-probe просто не активируется
  // (runtime check через isWebGL2 / cast).
  const renderer = bridge.renderer;
  const gl = renderer.getContext();
  if (typeof WebGL2RenderingContext !== "undefined" && gl instanceof WebGL2RenderingContext) {
    telemetry.attachRenderer(gl);
  } else {
    // eslint-disable-next-line no-console
    console.warn("[bootstrap] WebGL2 unavailable — GPU-time probe disabled");
  }
  telemetry.onTick((payload) => {
    if (conn && !disconnected) conn.sendTelemetryPerf(payload);
  });
  telemetry.start();

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
      return;
    }
    if (next === "robot_voice" && send) {
      c!.send({ cmd: "voice_mode", mode: "ttts_proxy", ts_ms: Date.now() });
    }
    if (send) {
      c!.send({ cmd: "voice_ptt_start", mode: next, ts_ms: Date.now() });
    }
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
            for (const topic of DEFAULT_SUBSCRIBED_TOPICS) {
              conn!.subscribe(topic);
            }
            opts.pinOverlay.classList.add("pin-overlay--hidden");
          } else if (state === "auth_failed") {
            setStatus("WRONG PIN", "lost");
            opts.pinOverlay.classList.remove("pin-overlay--hidden");
            opts.pinError.hidden = false;
            // Если уже вошли в VR с неверным PIN — выходим обратно к форме.
            void exitVr();
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
          if (topic === MAIN_SCREEN_TOPIC) {
            bridge.mainScreen.ingestJpeg(payload);
            return;
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

  // ---- Teleop loop -----------------------------------------------------------

  let lastTickTs = performance.now();
  // Phase 2.2: отдельный ts для telemetry sample, чтобы большой first-tick
  // delta не загрязнял FPS window (performance.now() - 0 ≈ минуты).
  let lastSampleTs = performance.now();

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
    // Phase 2.2: sample frame deltas для telemetry (FPS rolling avg).
    // Один раз в rAF — на VR ~72/90 Hz, на desktop ~60 Hz. XR rAF
    // отдельный цикл (выше в enterVr), но он тикает teleop, не нас;
    // sampleFrame внутри XR-кадрового цикла не вызываем — telemetry
    // привязан к window.rAF (он в VR всё равно «заморожен», но
    // если VR не активен — это и есть основной цикл).
    telemetry.sampleFrame(now - lastSampleTs);
    lastSampleTs = now;
    if (now - lastTickTs >= 33) {
      lastTickTs = now;
      tickTeleop();
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
      xrTeleopHandle = createXrTeleop({ fsm, session });
      // window.requestAnimationFrame в immersive-vr заморожен браузером,
      // поэтому teleop тикаем в XR-кадровом цикле (session.requestAnimationFrame).
      xrRafSession = session;
      const xrFrame = (_time: DOMHighResTimeStamp, _frame: XRFrame): void => {
        if (!xr.isActive()) return;
        tickTeleop();
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
      telemetry.stop();
      stopRender();
      desktopTeleop.destroy();
      if (xrRafSession && xrRafId) {
        xrRafSession.cancelAnimationFrame(xrRafId);
      }
      xrTeleopHandle?.destroy();
      voiceCapture.stop();
      conn?.close();
      bridge.dispose();
    }
  };
}

function deriveWsUrl(): string {
  const proto = location.protocol === "https:" ? "wss:" : "ws:";
  return `${proto}//${location.host}/quest`;
}
