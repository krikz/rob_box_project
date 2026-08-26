// Bootstrap: подключение к WSS, инициализация сцены, teleop, UI.

import { Connection } from "./wire/connection";
import { createCaptainBridge } from "./scene/captain_bridge";
import { TeleopFSM } from "./input/teleop_fsm";
import { createDesktopTeleop } from "./input/desktop_teleop";
import { createStreamSelect } from "./ui/stream_select";
import { ModeManager, CAPTAIN_MODES } from "./modes/mode_manager";
import { createModeHud } from "./modes/mode_hud";
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
  const bridge = createCaptainBridge({ canvas: opts.canvas });
  bridge.initLayout();
  const stopRender = bridge.start();

  const fsm = new TeleopFSM();
  const modeManager = new ModeManager();
  const modeHud = createModeHud();
  modeHud.attachModeManager(modeManager);

  // Boost multiplier разрешён только когда teleop/mixed режим (не explore/voice).
  const desktopTeleop = createDesktopTeleop({
    fsm,
    modeManager,
    boostEnabled: () => modeManager.getMode() === "teleop" || modeManager.getMode() === "mixed"
  });
  // При teleop-вводе → авто-upgrade voice→mixed.
  modeManager.subscribe(() => {
    // no-op; ModeManager уже обновил HUD через attachModeManager.
  });

  // Telop keyboard/WASD → сигнал в ModeManager (auto-upgrade).
  // Это отдельный wiring — desktop_teleop тоже зовёт fsm, но мы хотим
  // знать, был ли вообще teleop-intent.
  const onTeleopIntent = (): void => {
    modeManager.reportTeleopIntent();
  };
  window.addEventListener("keydown", (ev) => {
    if (["KeyW", "KeyA", "KeyS", "KeyD", "ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight"].includes(ev.code)) {
      onTeleopIntent();
    }
  });

  let conn: Connection | null = null;
  let streamSelect: ReturnType<typeof createStreamSelect> | null = null;
  let disconnected = true;

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
    getActiveTopics: () => bridge.panels.list().map((p) => p.topic)
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
      if (conn && !disconnected) {
        // emergency: проверим desktop E-клавишу.
        const teleopHandle = (window as unknown as { __questDesktopTeleop?: { consumeEmergency(): boolean } }).__questDesktopTeleop;
        if (teleopHandle?.consumeEmergency()) {
          conn.send(fsm.triggerEmergency("ui_button"));
        }
        const out = fsm.tick(Date.now());
        if (out) conn.send(out.cmd);
      }
    }
    requestAnimationFrame(teleopLoop);
  }
  requestAnimationFrame(teleopLoop);

  return {
    dispose(): void {
      stopRender();
      desktopTeleop.destroy();
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