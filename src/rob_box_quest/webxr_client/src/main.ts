// Bootstrap: подключение к WSS, инициализация сцены, teleop, UI.
//
// Phase 1.5: WebXR-вход (immersive-vr). Если navigator.xr есть и устройство
// поддерживает режим — в stream_select появляется блок "XR Mode" с кнопками
// Enter/Exit VR. По клику (user-activation обязателен!) → requestSession →
// attachXrSession → Three.js XR-рендер + XR-контроллеры читаются в frame loop
// и кормят TeleopFSM.

import { Connection } from "./wire/connection";
import { createCaptainBridge } from "./scene/captain_bridge";
import { TeleopFSM } from "./input/teleop_fsm";
import { createDesktopTeleop } from "./input/desktop_teleop";
import { createXrTeleop, pollXrInput } from "./input/xr_teleop";
import { createXrBootstrap, type XrBootstrap } from "./xr_bootstrap";
import { createStreamSelect } from "./ui/stream_select";
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
  // Phase 2.1: запускаем async loader bridge-ассетов параллельно со
  // start() — render loop крутится пока грузится, и когда initEnvironment
  // resolve'нется, scene получит GLB-walls/props + scene.environment.
  // На failure loader переходит в state="fallback" (BoxGeometry walls),
  // сцена не ломается — дизайн §1.1 graceful degradation.
  const envReady = bridge.initEnvironment().catch((e: unknown) => {
    const err = e instanceof Error ? e : new Error(String(e));
    console.warn("[bridge] initEnvironment failed, continuing with fallback:", err.message);
  });
  const stopRender = bridge.start();

  const fsm = new TeleopFSM();
  const desktopTeleop = createDesktopTeleop({ fsm });
  const xr: XrBootstrap = createXrBootstrap();

  let conn: Connection | null = null;
  let streamSelect: ReturnType<typeof createStreamSelect> | null = null;
  let disconnected = true;
  let xrTeleopHandle: ReturnType<typeof createXrTeleop> | null = null;
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
      if (conn && !disconnected) {
        // emergency: проверим desktop E-клавишу.
        const teleopHandle = (window as unknown as { __questDesktopTeleop?: { consumeEmergency(): boolean } }).__questDesktopTeleop;
        if (teleopHandle?.consumeEmergency()) {
          conn.send(fsm.triggerEmergency("ui_button"));
        }
        const out = fsm.tick(Date.now());
        if (out) conn.send(out.cmd);
      }
      // XR-контроллеры (Quest): если есть активная XR-сессия и inputSources,
      // каждый кадр кормим FSM. pollXrInput идемпотентен (только если есть grip).
      if (xr.isActive() && xrInputSources.length > 0) {
        const dummyState = { linear: 0, angular: 0, deadman: false, emergencyEdge: false };
        for (const src of xrInputSources) {
          if (src && (src as { gamepad?: { axes?: number[] } }).gamepad) {
            pollXrInput(dummyState, src, fsm);
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
      xrTeleopHandle = createXrTeleop({ fsm, session });
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
      xrInputSources = [];
      streamSelect?.setXrSessionState("not-in-vr");
    }
  }

  return {
    dispose(): void {
      stopRender();
      // envReady может быть in-flight; не ждём — dispose bridge ниже
      // вызовет envAssets?.dispose(), что покроет и resolved-state.
      void envReady;
      desktopTeleop.destroy();
      xrTeleopHandle?.destroy();
      streamSelect?.destroy();
      conn?.close();
      bridge.dispose();
    }
  };
}

function deriveWsUrl(): string {
  const proto = location.protocol === "https:" ? "wss:" : "ws:";
  return `${proto}//${location.host}/quest`;
}