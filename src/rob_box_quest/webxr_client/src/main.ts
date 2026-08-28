// Bootstrap: подключение к WSS, инициализация сцены, teleop, WebXR auto-entry.
//
// Вход на мостик — только PIN-форма. После ввода PIN:
//   - если браузер поддерживает WebXR (immersive-vr) — сразу входим в VR
//     (requestSession вызывается в user-activation submit handler);
//   - иначе остаёмся в desktop-режиме (WASD fallback + 2D-рендер).
//
// Debug-панелей (lil-gui) больше нет — вход только через PIN-форму.

import { Connection } from "./wire/connection";
import { createCaptainBridge, MAIN_SCREEN_TOPIC } from "./scene/captain_bridge";
import { TeleopFSM } from "./input/teleop_fsm";
import { createDesktopTeleop } from "./input/desktop_teleop";
import { createXrTeleop, pollXrInput } from "./input/xr_teleop";
import { createVoiceCapture } from "./input/voice_capture";
import { createXrBootstrap, type XrBootstrap } from "./xr_bootstrap";
import { ModeSwitcher, modeFromKey, type BridgeMode } from "./ui/mode_switcher";
import { MuteController } from "./ui/mute_controller";
import {
  createAudioHud,
  type AudioHudHandle,
  rmsLevel as computeRmsLevel,
  smoothLevel as smoothAudioLevel
} from "./scene/audio_hud";
import {
  defaultStorage,
  loadLayout,
  saveLayout,
  clearLayout as clearPanelLayout
} from "./scene/panel_persistence";

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
  /** Контейнер для mode-chip'ов (HUD переключатель режима). */
  modeHud?: HTMLElement;
  /** Кнопка сброса layout panels в localStorage (опционально). */
  resetLayoutBtn?: HTMLElement;
}

export function bootstrap(opts: BootstrapOptions): { dispose(): void } {
  const url = opts.url ?? deriveWsUrl();
  const bridge = createCaptainBridge({ canvas: opts.canvas, enableXr: true });
  // Layout: пробуем восстановить из localStorage; иначе — дефолтный
  // resetLayout(). Если storage недоступен (SSR/private mode) — дефолт.
  const storage = defaultStorage();
  const restoredLayout = loadLayout(storage);
  if (restoredLayout && restoredLayout.length > 0) {
    // Применяем каждую запись напрямую через PanelManager (не вызываем
    // initLayout — он бы стёр наш loop через resetLayout()). После
    // добавления всех panels — syncPanels() через initLayout НЕ нужен,
    // потому что syncPanels вызывается явно ниже. Но в публичном API
    // captain_bridge нет прямого syncPanels — вызов initLayout
    // пересоздаст panels. Решение: используем внутренний addPanel
    // подход — создаём через createPanel() и потом отдельно триггерим
    // пересоздание через resetLayout() + ручное восстановление.
    // Проще: сначала resetLayout (создаёт дефолт), потом мутации.
    bridge.initLayout();
    for (const p of restoredLayout) {
      const all = bridge.panels.list();
      if (all.length === 0) break;
      const last = all[all.length - 1];
      if (!last) continue;
      // Перезаписываем topic/position/facing у последнего panel.
      // PanelManager.switchStream меняет только topic; для position/facing
      // используем move() (он пересчитывает facing к началу координат).
      bridge.panels.switchStream(last.id, p.topic);
      bridge.panels.move(last.id, p.position.x, p.position.z);
    }
    if (opts.resetLayoutBtn) opts.resetLayoutBtn.hidden = false;
  } else {
    bridge.initLayout();
  }
  const stopRender = bridge.start();
  // Phase 2.1: load Captain Bridge CC0 environment (5 GLB + HDR).
  // Fail-soft — see captain_bridge.ts → loadEnvironment(). The procedural
  // fallback floor + grid stays in place if the GLB fetch fails.
  bridge.loadEnvironment().catch((err) => {
    // eslint-disable-next-line no-console
    console.warn("[bootstrap] bridge environment load rejected:", err);
  });

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
  // ---------- Phase 2: mode switcher, mute, audio HUD ----------
  // Эти объекты нужны ДО voiceCapture.onChunk (mute блокирует исходящий
  // поток, audio HUD читает RMS из тех же чанков).
  const modeSwitcher = new ModeSwitcher();
  const muteCtl = new MuteController({ pressDurationMs: 350 });
  const audioHud: AudioHudHandle | null = (() => {
    try {
      const h = createAudioHud({
        position: { x: -2.35, y: 2.95, z: -3.85 },
        scale: { x: 1.1, y: 0.2 }
      });
      bridge.scene.add(h.sprite);
      return h;
    } catch (err) {
      // eslint-disable-next-line no-console
      console.warn("[quest] audio HUD init failed:", err);
      return null;
    }
  })();
  let audioHudSmoothed = 0;

  // Голос: рация (правый grip) и робот-голос (левый grip → STT → LLM → TTS)
  // делят один mic-захват (int16 PCM 16 kHz) → VOICE_AUDIO. Режим кодируется
  // в voice_ptt_start/stop как `mode` ("radio" | "robot_voice").
  const voiceCapture = createVoiceCapture({
    onChunk: (pcm) => {
      // 1) Audio HUD: рисуем уровень ДО любых early-return.
      //    Mute меняет только иконку, не уровень (пользователь видит,
      //    что mic живой, но заблокирован).
      if (audioHud) {
        const target = computeRmsLevel(pcm);
        audioHudSmoothed = smoothAudioLevel(audioHudSmoothed, target);
        audioHud.setState({
          micState: muteCtl.isMuted() ? "muted" : voicePttMode === "none" ? "idle" : "talk",
          level: audioHudSmoothed
        });
      }
      // 2) Mute → блокируем исходящий поток.
      if (!conn || disconnected) return;
      if (muteCtl.isMuted()) return;
      // 3) Mode-gate: voice passthrough разрешён в explore/teleop/voice/mixed.
      //    ModeSwitcher.shouldEmitVoice() == true во всех режимах (это рация).
      //    Если в будущем сделаем per-mode фильтр — здесь точка расширения.
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

  // ---- Phase 2: mode switcher (keyboard + HUD chips) -------------------------

  function paintModeChips(active: BridgeMode): void {
    if (!opts.modeHud) return;
    for (const chip of Array.from(opts.modeHud.querySelectorAll("[<data-mode]"))) {
      const el = chip as HTMLElement;
      const m = el.dataset["mode"] as BridgeMode | undefined;
      if (!m) continue;
      el.classList.toggle("mode-hud__chip--active", m === active);
    }
  }
  paintModeChips(modeSwitcher.current());
  modeSwitcher.subscribe((next) => {
    paintModeChips(next);
    // Если переключились в explore/voice — disarm (safety: «не ехать без
    // явного намерения»). teleop/mixed оставляют arm-state как есть.
    if (next === "explore" || next === "voice") {
      armed = false;
      bridge.setArmState(false);
      fsm.setDeadman(false);
      fsm.setLinear(0);
      fsm.setAngular(0);
      bridge.setControllerActive(false);
    }
  });

  if (opts.modeHud) {
    for (const chip of Array.from(opts.modeHud.querySelectorAll("[<data-mode]"))) {
      const el = chip as HTMLElement;
      el.addEventListener("click", () => {
        const m = el.dataset["mode"] as BridgeMode | undefined;
        if (m) modeSwitcher.setMode(m);
      });
    }
  }

  window.addEventListener("keydown", (ev) => {
    // Игнорируем если фокус на input/textarea — иначе PIN-форма сломается.
    const t = ev.target as HTMLElement | null;
    if (t && (t.tagName === "INPUT" || t.tagName === "TEXTAREA")) return;
    const m = modeFromKey(ev.key);
    if (m) {
      modeSwitcher.setMode(m);
      ev.preventDefault();
      return;
    }
    // M → toggle mute. Это keyboard-аналог long-press A на XR-контроллере.
    // Long-press A будет добавлен в отдельной карточке (через расширение
    // teleop_config + pollXrInput).
    if (ev.key === "m" || ev.key === "M" || ev.key === "ь" || ev.key === "Ь") {
      muteCtl.toggle();
      ev.preventDefault();
    }
  });

  // ---- Reset to Default (panel layout) ---------------------------------------

  if (opts.resetLayoutBtn) {
    opts.resetLayoutBtn.addEventListener("click", () => {
      bridge.panels.resetLayout();
      clearPanelLayout(storage);
      opts.resetLayoutBtn!.hidden = true;
    });
  }

  // Сохраняем layout в localStorage при любом изменении (debounce не нужен —
  // события редкие). Подписываемся на фиксацию через микротаск после tick.
  let layoutDirty = false;
  // Помечаем dirty в mutation hooks через monkey-patch move/close/switchStream.
  // Чтобы не патчить приватный API — следим через dirty-flag, который
  // поднимается в pollXrControllers (drag и т.д. пока не реализованы).
  function maybeSaveLayout(): void {
    if (!layoutDirty) return;
    saveLayout(storage, bridge.panels.list());
    layoutDirty = false;
  }

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
    // Phase 2: mode-gate. В explore/voice — НЕ шлём teleop-команды, даже
    // если arm=true. Это safety-фича: переключение в voice должно явно
    // остановить движение робота.
    if (modeSwitcher.shouldEmitTeleop() && conn && !disconnected) {
      const teleopHandle = (
        window as unknown as { __questDesktopTeleop?: { consumeEmergency(): boolean } }
      ).__questDesktopTeleop;
      if (teleopHandle?.consumeEmergency()) {
        conn.send(fsm.triggerEmergency("ui_button"));
      }
      const out = fsm.tick(Date.now());
      if (out) conn.send(out.cmd);
    } else if (!modeSwitcher.shouldEmitTeleop()) {
      // mode gate закрыт — глушим импульсы если они были активны.
      fsm.setLinear(0);
      fsm.setAngular(0);
    }
    pollXrControllers();
    maybeSaveLayout();
  }

  function teleopLoop(): void {
    const now = performance.now();
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
      stopRender();
      desktopTeleop.destroy();
      if (xrRafSession && xrRafId) {
        xrRafSession.cancelAnimationFrame(xrRafId);
      }
      xrTeleopHandle?.destroy();
      voiceCapture.stop();
      audioHud?.dispose();
      conn?.close();
      bridge.dispose();
    }
  };
}

function deriveWsUrl(): string {
  const proto = location.protocol === "https:" ? "wss:" : "ws:";
  return `${proto}//${location.host}/quest`;
}
