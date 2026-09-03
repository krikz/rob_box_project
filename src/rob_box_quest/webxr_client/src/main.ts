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
import {
  isUnknownState,
  type AvatarMode,
  type SupervisorState as PanelSupervisorState,
  UNKNOWN_STATE
} from "./scene/supervisor_panel";
import { createAlertToast, alertText } from "./scene/alert_toast";
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
import {
  createModeManager,
  type ClientModeManager,
  type ClientModeDefaults
} from "./ui/mode_manager";
import { createVoicePresetsPanel, type VoicePresetsPanel } from "./ui/voice_presets_panel";
import type {
  VoiceLanguage,
  VoicePresetId,
  VoicePresetInfo
} from "./wire/messages";
import { createToast, type Toast } from "./ui/toast";
import { supervisorEffect, type FloorLabel, type SupervisorState } from "./state/supervisor_state";
import type { JsonCmd } from "./wire/messages";

const CLIENT_VERSION = "0.1.0";
// AV-17: subprotocol v2 по умолчанию. Если сервер на v1 — supervisor
// не используется (Connection.canSendSupervisor → false), и HUD
// показывает строку «SUPERVISOR: v1 (no coordination)». См.
// docs/architecture/meta-quest-api.md §11 + docs/adr/0028 §4.4.
const SUBPROTOCOL = "robbox-quest-v2";

// AV-28 §P7: дефолты UI до ответа сервера. Должны совпадать с
// default_preset / default_language в voice_presets.yaml.
const DEFAULT_VOICE_PRESET: VoicePresetId = "technical";
const DEFAULT_VOICE_LANGUAGE: VoiceLanguage = "ru";
// Fallback-список пресетов до ответа сервера (см. voice_presets.yaml).
// Если сервер пришлёт свой voice_presets event — этот список заменится.
const FALLBACK_PRESETS: VoicePresetInfo[] = [
  { id: "technical", name: "Технический" },
  { id: "street", name: "По понятиям" },
  { id: "caveman", name: "Пещерный" },
  { id: "business", name: "Деловой" },
  { id: "philosopher", name: "Философ" },
  { id: "lenin", name: "Ленин" }
];

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
  // Состояние ПАНЕЛИ режимов (R14) — проекция того же STATE_UPDATE,
  // который разбирает AV-17. Отдельная переменная нужна лишь потому, что
  // у панели своя форма (teleop_floor/voice_floor c last_event), а не
  // потому, что это второй источник истины: пишется она только из
  // applySupervisorState и из avatar_state_ack.
  // `UNKNOWN_STATE` — «до первого STATE_UPDATE»: панель блокирует
  // floor-кнопки (см. `draw()` в supervisor_panel.ts).
  let panelState: PanelSupervisorState = UNKNOWN_STATE;
  // Кэш последнего подтверждённого voice_input_mode==off (диалог выкл).
  // Используется supervisorActionToCmd, чтобы корректно выставить следующий
  // режим без лишнего round-trip.
  let voiceOffCached = false;

  /** Маппит действие с панели супервизора (R14) в JSON_CMD. */
  function supervisorActionToCmd(
    action: string,
    panel: { showToast(msg: string, level?: "warn" | "bad"): void }
  ): JsonCmd | null {
    const ts = Date.now();

    if (action.startsWith("mode:")) {
      const target = action.slice("mode:".length) as AvatarMode;
      // Допустимость перехода решает супервизор, а не клиент. Раньше
      // здесь стояла локальная копия таблицы переходов FSM — она успела
      // разъехаться с сервером (разрешала telegram_active →
      // avatar_present, который сервер отклоняет, и запрещала
      // telegram_active → mixed, который сервер как раз разрешает).
      // Отказ прилетит как ERROR{MODE_CONFLICT} и покажется тостом
      // (см. onSupervisorError).
      return {
        cmd: "avatar_set_mode",
        ts_ms: ts,
        mode: target,
        reason: "ui_panel"
      };
    }

    if (action.startsWith("floor:")) {
      // `floor:teleop:acquire` / `floor:voice:release`
      const [, kind, op] = action.split(":");
      if (kind !== "teleop" && kind !== "voice") return null;
      if (op === "acquire") {
        // До STATE_UPDATE кнопка всё равно заблокирована в UI (см. supervisor_panel
        // draw → blocked). Доп. сторож здесь на случай пропуска события.
        if (isUnknownState(panelState)) {
          panel.showToast("статус неизвестен, повторите позже", "warn");
          return null;
        }
        return { cmd: "avatar_acquire_floor", ts_ms: ts, kind };
      }
      if (op === "release") {
        return { cmd: "avatar_release_floor", ts_ms: ts, kind };
      }
      return null;
    }

    if (action === "dialogue:toggle") {
      // Существующий путь: voice_mode → супервизор → dialogue_node.
      // off — глушит ТОЛЬКО ReSpeaker (люди рядом, ADR-0028 S5), не Quest-микрофон.
      const next = voiceOffCached ? "passthrough" : "off";
      voiceOffCached = !voiceOffCached;
      return { cmd: "voice_mode", ts_ms: ts, mode: next };
    }

    return null;
  }

  /**
   * Распаковывает `state` из avatar_state_ack. Сервер сейчас шлёт state
   * JSON-объектом внутри JSON_EVENT; msgpack-вариант (0x1203) —
   * forward-compatible, парсер уже есть в supervisor_panel.ts.
   */
  function applySupervisorAck(
    raw: Record<string, unknown>,
    panel: { setState(s: PanelSupervisorState): void }
  ): void {
    const mode = typeof raw.mode === "string" ? (raw.mode as AvatarMode) : "off";
    const teleopRaw = (raw.teleop as Record<string, unknown> | undefined) ?? {};
    const voiceRaw = (raw.voice as Record<string, unknown> | undefined) ?? {};
    const teleop = parseFloorShallow(teleopRaw);
    const voice = parseFloorShallow(voiceRaw);
    const lastEvent =
      typeof raw.last_event === "string" ? raw.last_event : "STATE_UPDATE";
    const lastTs =
      typeof raw.last_event_ts_ms === "number" ? raw.last_event_ts_ms : Date.now();
    panelState = {
      mode,
      teleop_floor: teleop,
      voice_floor: voice,
      last_event: lastEvent,
      last_event_ts_ms: lastTs
    };
    panel.setState(panelState);
  }

  function parseFloorShallow(v: Record<string, unknown>): {
    held_by: { client_id: string; since_ms: number } | null;
    last_event: "ACQUIRED" | "RELEASED" | "TAKEOVER" | "DENIED" | null;
  } {
    const heldRaw = v.held_by;
    let heldBy: { client_id: string; since_ms: number } | null = null;
    if (heldRaw && typeof heldRaw === "object") {
      const h = heldRaw as Record<string, unknown>;
      heldBy = {
        client_id: typeof h.client_id === "string" ? h.client_id : "unknown",
        since_ms: typeof h.since_ms === "number" ? h.since_ms : Date.now()
      };
    }
    const ev = v.last_event;
    const lastEvent =
      ev === "ACQUIRED" ||
      ev === "RELEASED" ||
      ev === "TAKEOVER" ||
      ev === "DENIED"
        ? ev
        : null;
    return { held_by: heldBy, last_event: lastEvent };
  }

  // Phase 2.3 overlays — создаём ДО старта асинхронных pipeline'ов,
  // чтобы loading-screen сразу перекрыл экран пока грузятся CC0 GLB.
  const loading = createLoadingScreen(opts.body, "Loading environment…");
  const errorOverlay: ErrorOverlay = createErrorOverlay(opts.body);
  const help: HelpOverlay = createHelpOverlay(opts.body);
  // AV-28 §P7: defaults UI = voice_presets.yaml default_preset / default_language.
  const voiceDefaults: ClientModeDefaults = {
    preset: DEFAULT_VOICE_PRESET,
    language: DEFAULT_VOICE_LANGUAGE
  };
  const modeManager: ClientModeManager = createModeManager(undefined, voiceDefaults);
  const watchdog: DisconnectWatchdog = createDisconnectWatchdog(errorOverlay);
  // AV-17: короткие уведомления о supervisor-событиях (потерял руль,
  // floor занят другим клиентом). Создаём сразу, чтобы не пропустить
  // первый STATE_UPDATE в гонке с инициализацией UI.
  const toast: Toast = createToast(opts.body);
  // AV-26: robot_alert toast + дублирование в status HUD как постоянная
  // метка пока алёрт активен. Error-алёрты ещё и в errorOverlay.
  const alertToast = createAlertToast({
    parent: opts.body,
    errorOverlay
  });

  // Панель выбора пресета/языка (AV-28 §P7). До ответа сервера показывает
  // fallback-список и дефолтную подсветку; setLoading(true) отключает
  // кнопки. TODO: когда AV-18-якорь в 3D-сцене будет готов (R12 из
  // task-graph), переместить эту панель в 3D-HUD; сейчас — DOM-overlay
  // в нижнем-левом углу, проецируется в VR рядом с левым грипом.
  const voicePanel: VoicePresetsPanel = createVoicePresetsPanel(opts.body, {
    presets: FALLBACK_PRESETS,
    languages: [DEFAULT_VOICE_LANGUAGE, "en"],
    currentPreset: DEFAULT_VOICE_PRESET,
    currentLanguage: DEFAULT_VOICE_LANGUAGE,
    loading: true,
    onPresetChange: (preset) => {
      const c = conn;
      if (!c || disconnected) return;
      // Запоминаем желание → mode_manager уже синхронизирован panel'ом;
      // шлём на сервер. ACK подтвердит, NACK откатит.
      try {
        c.send({
          cmd: "set_voice",
          ts_ms: Date.now(),
          voice_id: modeManager.snapshot().currentVoice ?? "",
          preset,
          language: modeManager.snapshot().currentLanguage ?? DEFAULT_VOICE_LANGUAGE
        });
      } catch (err) {
        // eslint-disable-next-line no-console
        console.warn("[quest] set_voice send failed:", err);
        // Откатываем UI к последнему известному состоянию.
        voicePanel.setCurrentPreset(modeManager.snapshot().currentPreset);
      }
    },
    onLanguageChange: (language) => {
      const c = conn;
      if (!c || disconnected) return;
      try {
        c.send({
          cmd: "set_voice",
          ts_ms: Date.now(),
          voice_id: modeManager.snapshot().currentVoice ?? "",
          preset: modeManager.snapshot().currentPreset ?? DEFAULT_VOICE_PRESET,
          language
        });
      } catch (err) {
        // eslint-disable-next-line no-console
        console.warn("[quest] set_voice send failed:", err);
        voicePanel.setCurrentLanguage(modeManager.snapshot().currentLanguage);
      }
    }
  });

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
    },
    // Клик по кнопке панели супервизора (R14): маппим action → JSON_CMD.
    onSupervisorAction: (action) => {
      if (!conn || disconnected) return;
      const cmd = supervisorActionToCmd(action, bridge.supervisorPanel);
      if (cmd === null) return;
      conn.send(cmd);
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
  // AV-19 (issue #1911, ADR-0028 §4.4): session_id нашей сессии.
  // Сравнивается с WELCOME.teleop_floor_held_by — если равны, мы держим
  // teleop_floor. Запоминается при первом WELCOME и сбрасывается на disconnect.
  // В текущей реализации FSM уже синхронизируется через onWelcome
  // (teleopFloorHeldBy === sessionId). Переменная оставлена как hook
  // для Phase 2 (e.g. конкретный «Acquired by YOU» indicator в HUD).
  let mySessionId: string | null = null;
  // AV-19: текст тоста «возьми руль», показывается когда ARM заблокирован
  // из-за чужого floor. Снимается когда floor снова наш.
  let floorBlockToast: { show(): void; hide(): void } | null = null;
  // AV-17: avatar_supervisor state. `null` = STATE_UPDATE ещё не пришёл
  // (или сервер на v1, тогда degraded=true). myClientId придёт в WELCOME,
  // когда AV-16 добавит поле; пока WELCOME его не содержит — оставляем
  // null и UI честно показывает «?».
  let supervisorState: SupervisorState | null = null;
  let supervisorMyClientId: string | null = null;
  let supervisorDegraded = false;
  // Кэш предыдущего floorLabel для teleop: чтобы зафиксировать переход
  // «my → other» и снять ARM (только на этом переходе).
  let prevTeleopLabel: FloorLabel = "unknown";
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

  /**
   * AV-19: показать/скрыть тост «возьми руль», когда ARM заблокирован.
   * Создаём отдельный ErrorOverlay как info-toast (не конфликтует с
   * disconnect-overlay); общий parent — body.
   */
  function setFloorBlocked(blocked: boolean): void {
    if (blocked) {
      if (floorBlockToast === null) {
        const toast = createErrorOverlay(opts.body, { level: "info" });
        floorBlockToast = {
          show() {
            toast.show("Возьми руль", "Управление у другого оператора");
          },
          hide() {
            toast.dismiss();
          }
        };
      }
      floorBlockToast.show();
    } else {
      floorBlockToast?.hide();
    }
  }

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

  /**
   * Применить supervisor-state: HUD + дисарм на потере teleop-floor + тост.
   * Вызывается из onSupervisorState и из onStateChange (reset при reconnect).
   */
  function applySupervisorState(next: SupervisorState | null): void {
    supervisorState = next;
    // Вся логика перехода — в чистом редьюсере (state/supervisor_state.ts),
    // здесь только применение эффектов к железу UI.
    const eff = supervisorEffect(prevTeleopLabel, next, supervisorMyClientId);

    if (eff.disarm) {
      armed = false;
      xrArmWasPressed = false;
      fsm.setDeadman(false);
      fsm.reset();
      bridge.setArmState(false);
      bridge.setControllerActive(false);
      modeManager.setTeleopState("disarmed");
      if (eff.toast) toast.show(eff.toast, { level: "warn", autoHideMs: 5000 });
      // eslint-disable-next-line no-console
      console.warn("[quest] supervisor: teleop-floor revoked → DISARM", {
        newHolder: next?.teleopFloor.clientId ?? null
      });
    }
    prevTeleopLabel = eff.teleopLabel;

    bridge.statusHud.setSupervisor(next, supervisorMyClientId, { degraded: supervisorDegraded });

    // AV-18: та же истина — в панель режимов. Панель не подписывается на
    // сервер сама: STATE_UPDATE разбирается один раз здесь, дальше это
    // просто вторая проекция того же состояния.
    panelState =
      next === null
        ? UNKNOWN_STATE
        : {
            mode: next.mode as AvatarMode,
            teleop_floor: {
              held_by:
                next.teleopFloor.clientId === null
                  ? null
                  : {
                      client_id: next.teleopFloor.clientId,
                      since_ms: next.teleopFloor.sinceMs
                    },
              last_event: null
            },
            voice_floor: {
              held_by:
                next.voiceFloor.clientId === null
                  ? null
                  : {
                      client_id: next.voiceFloor.clientId,
                      since_ms: next.voiceFloor.sinceMs
                    },
              last_event: null
            },
            last_event: "STATE_UPDATE",
            last_event_ts_ms: next.updatedMs
          };
    bridge.supervisorPanel.setState(panelState);
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
            // Phase 2.3: ошибка прячется при восстановлении коннекта.
            watchdog.markConnected();
            for (const topic of [...bridge.videoTopics(), ...NON_VIDEO_TOPICS]) {
              conn!.subscribe(topic);
            }
            // Каталог стримов → меню выбора на панелях (R10).
            conn!.requestStreamList();
            // AV-28 §P7: запросить список голосов для панели пресетов.
            // Сервер ответит JSON_EVENT{type:"voice_list"} (voice_id+display_name)
            // и/или {type:"voice_presets"} (список пресетов+языков+дефолты).
            conn!.send({ cmd: "list_voices", ts_ms: Date.now() });
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
            // AV-19: на reconnect FSM предполагает «оптимистично» hasFloor=true;
            // настоящее состояние придёт из WELCOME.teleop_floor_held_by
            // и/или первого FLOOR_HELD/floor_lost.
            mySessionId = null;
            fsm.setHasFloor(true);
            setFloorBlocked(false);
            // Аналогично supervisor-state: после разрыва мы не знаем, кто
            // держит floor-ы → «неизвестно», не «свободно» (ADR-0018).
            supervisorMyClientId = null;
            supervisorDegraded = false;
            prevTeleopLabel = "unknown";
            applySupervisorState(null);
            // Disconnect-watchdog начинает отсчёт; если > 5s без успеха —
            // покажем error overlay (см. createDisconnectWatchdog).
            watchdog.markDisconnected();
            // Сервер ещё может быть жив (например, Wi-Fi дёрнулся) → держим
            // существующие alert'ы; они вернутся в onJsonEvent после reconnect.
          } else if (state === "connecting") {
            setStatus("CONNECTING…", "connecting");
          } else if (state === "closed") {
            setStatus("CLOSED", "lost");
            bridge.statusHud.setRtt(null);
            disconnected = true;
            // AV-19: сброс FSM-state и тостов.
            mySessionId = null;
            fsm.setHasFloor(true);
            setFloorBlocked(false);
            // "closed" — окончательно (не reconnect). Прямо сейчас
            // показываем overlay без 5-секундного порога.
            errorOverlay.show("Disconnected", "Connection closed by server");
            // Очищаем alert'ы — соединения нет, оператору не показываем
            // устаревшие «Батарея 12%».
            alertToast.clear();
            bridge.statusHud.setAlert(null);
          }
        },
        onWelcome: (sessionId, _serverTimeMs, teleopFloorHeldBy) => {
          // AV-19: запоминаем наш session_id для сравнения с WELCOME.teleop_floor_held_by.
          mySessionId = sessionId;
          // Сервер в WELCOME уже сообщил, держит ли floor наша сессия.
          // Если да — hasFloor=true; если поле отсутствует (Phase 1 бэкенд)
          // — оставляем оптимистичный default; первый FLOOR_HELD/floor_lost
          // скорректирует.
          if (teleopFloorHeldBy !== null) {
            const weHold = teleopFloorHeldBy === sessionId;
            fsm.setHasFloor(weHold);
            setFloorBlocked(!weHold);
          }
          // AV-17: тот же session_id — наш client_id для supervisor-HUD.
          // Отдельного поля client_id в WELCOME нет; сервер формирует
          // session_id сам, клиент ничего не выдумывает.
          supervisorMyClientId = sessionId || null;
          // v1-сервер → supervisor-координации нет, HUD это показывает.
          supervisorDegraded = conn?.getNegotiatedVersion() === "v1";
          applySupervisorState(supervisorState);
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
        onSupervisorState: (state) => {
          applySupervisorState(state);
        },
        onSupervisorError: (code, message, meta) => {
          const holder = typeof meta?.held_by === "string" ? meta.held_by : null;
          const text =
            code === "FLOOR_HELD"
              ? holder
                ? `Занято другим оператором (${holder.slice(0, 8)}…)`
                : "Руль/голос сейчас у другого оператора"
              : `Режим не разрешён сейчас: ${message}`;
          toast.show(text, { level: "warn", autoHideMs: 5000 });
        },
        onJsonEvent: (event) => {
          // AV-18: ack/nack панели режимов — у них свои типы событий,
          // ниже по функции их уже не ждут.
          const t = (event as { type?: string }).type;
          if (t === "avatar_state_ack") {
            const raw = (event as { state?: Record<string, unknown> }).state;
            if (raw) applySupervisorAck(raw, bridge.supervisorPanel);
            return;
          }
          if (t === "avatar_state_nack") {
            const reason = (event as { reason?: string }).reason ?? "супервизор отклонил";
            bridge.supervisorPanel.showToast(reason, "bad");
            return;
          }
          if (t === "voice_mode_ack") {
            const mode = (event as { mode?: string }).mode;
            voiceOffCached = mode === "off";
            bridge.supervisorPanel.setVoiceOff(voiceOffCached);
            return;
          }

          // AV-28 §P7: voice_presets / voice_set_ack / voice_set_nack.
          // Свои типы событий, дальше по функции их уже не ждут.
          const type = (event as { type?: string }).type;
          if (type === "voice_presets" || type === "voice_set_ack" || type === "voice_set_nack") {
          const type = (event as { type?: string }).type;
          if (type === "voice_presets") {
            // Сервер прислал канонический список пресетов + дефолты
            // (AV-28 §P7). Заменяем fallback-список на серверный.
            const p = event as {
              presets?: VoicePresetInfo[];
              languages?: VoiceLanguage[];
              default_preset?: VoicePresetId;
              default_language?: VoiceLanguage;
            };
            if (Array.isArray(p.presets) && p.presets.length > 0) {
              voicePanel.setPresets(p.presets);
            }
            if (Array.isArray(p.languages) && p.languages.length > 0) {
              voicePanel.setLanguages(p.languages);
            }
            // Дефолт сервера имеет приоритет над локальным fallback.
            const srvPreset =
              p.default_preset ?? modeManager.snapshot().currentPreset;
            const srvLang =
              p.default_language ?? modeManager.snapshot().currentLanguage;
            if (srvPreset) {
              voicePanel.setCurrentPreset(srvPreset);
              modeManager.setCurrentPreset(srvPreset);
            }
            if (srvLang) {
              voicePanel.setCurrentLanguage(srvLang);
              modeManager.setCurrentLanguage(srvLang);
            }
            voicePanel.setLoading(false);
          } else if (type === "voice_set_ack") {
            const ack = event as {
              voice_id?: string;
              preset?: VoicePresetId;
              language?: VoiceLanguage;
            };
            if (ack.voice_id) modeManager.setCurrentVoice(ack.voice_id);
            if (ack.preset) modeManager.setCurrentPreset(ack.preset);
            if (ack.language) modeManager.setCurrentLanguage(ack.language);
          } else if (type === "voice_set_nack") {
            // Сервер отказал — откатываем UI и mode_manager к последнему
            // известному «хорошему» значению из snapshot.
            const nack = event as { reason?: string };
            // eslint-disable-next-line no-console
            console.warn("[quest] voice_set_nack:", nack.reason ?? "(no reason)");
            const snap = modeManager.snapshot();
            voicePanel.setCurrentPreset(snap.currentPreset);
            voicePanel.setCurrentLanguage(snap.currentLanguage);
            errorOverlay.show(
              "Не удалось сменить голос",
              nack.reason ?? "Сервер отклонил запрос"
            );
            }
            return;
          }
          // AV-19: JSON_EVENT{type:"floor_lost"} → мгновенный DISARM.
          if ((event as { type?: string }).type === "floor_lost") {
            fsm.setHasFloor(false);
            setFloorBlocked(true);
            // eslint-disable-next-line no-console
            console.warn(
              "[quest] floor_lost (reason=%s) — DISARM, toast shown. my_session_id=%s",
              (event as { reason?: string }).reason ?? "n/a",
              mySessionId ?? "unknown"
            );
            return;
          }
          // AV-26 / R7: robot_alert от сервера → toast + HUD-метка.
          // Формат: { type:"robot_alert", code, level, active?, args, ts_ms }.
          // active:true → поднятие; active:false или отсутствует +
          // level:"info" → снятие (см. meta-quest-api.md §6 + наш серверный
          // _send_alert_event, level="info" при снятии).
          const ev = event as { type?: string; code?: string; level?: string; active?: boolean; args?: Record<string, unknown>; ts_ms?: number };
          if (ev.type !== "robot_alert" || typeof ev.code !== "string") return;
          const level: "warn" | "error" = ev.level === "error" ? "error" : "warn";
          // Сервер шлёт active явно; старые клиенты могут не знать — считаем
          // active отсутствующим с level=warn как поднятие (обратная совмест).
          const isCleared = ev.active === false || ev.level === "info";
          if (isCleared) {
            alertToast.ingest({
              code: ev.code,
              active: false,
              level: "info",
              args: ev.args,
              ts_ms: typeof ev.ts_ms === "number" ? ev.ts_ms : Date.now()
            });
          } else {
            alertToast.ingest({
              code: ev.code,
              active: true,
              level,
              args: ev.args,
              ts_ms: typeof ev.ts_ms === "number" ? ev.ts_ms : Date.now()
            });
          }
          // HUD-метка: worst активного алёрта.
          const worst = alertToast.worstActive;
          if (worst) {
            bridge.statusHud.setAlert({
              text: alertText(worst),
              level: worst.level === "error" ? "error" : "warn"
            });
          } else {
            bridge.statusHud.setAlert(null);
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
      // AV-19: ARM требует наш teleop_floor. Если нет — НЕ ставим
      // armed=true (FSM сама не пошлёт twist благодаря setHasFloor(false),
      // но XR-стейк мог быть включён, и нам нужно показать «возьми руль»
      // + оставить deadman=false чтобы не накапливать «активность».
      if (fsm.hasFloor()) {
        armed = !armed;
        bridge.setArmState(armed);
        // Phase 2.3: mode-manager синхронизируется с реальным arm-стейтом.
        modeManager.setTeleopState(armed ? "armed" : "disarmed");
        setFloorBlocked(false);
      } else {
        // Floor чужой — показываем тост, ARM не активируем.
        setFloorBlocked(true);
        // НЕ ставим armed=true; оставляем deadman=false чтобы FSM не
        // пыталась слать twist из stopping-стейта (мы уже в idle).
      }
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
      setFloorBlocked(false);
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
      // AV-19: relay teleop_heartbeat 10 Гц пока FSM в armed (== ARM+floor).
      // Источник живости — клиент; сервер не генерирует heartbeat-ы сам
      // (иначе dead-man теряет смысл). Шлём через общий tick, чтобы
      // пользоваться одним event-loop и не плодить setInterval.
      const hb = fsm.heartbeatCmd(Date.now());
      if (hb) conn.send(hb);
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

  // ---- Hot keys (M: panel, H: help) ---------------------------------------

  function onHotKey(ev: KeyboardEvent): void {
    // Не реагируем, если фокус на input/textarea (PIN, future chat).
    const target = ev.target as HTMLElement | null;
    const tag = target?.tagName?.toLowerCase();
    if (tag === "input" || tag === "textarea" || target?.isContentEditable) {
      return;
    }
    // PIN-overlay ещё на экране → не открывать панель M-клавишей.
    if (!opts.pinOverlay.classList.contains("pin-overlay--hidden")) return;

    if (ev.key === "m" || ev.key === "M" || ev.key === "ь" || ev.key === "Ь") {
      ev.preventDefault();
      bridge.supervisorPanel.toggleVisible();
    }
  }
  document.addEventListener("keydown", onHotKey);

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
      document.removeEventListener("keydown", onHotKey);
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
      voicePanel.dispose();
      alertToast.dispose();
    }
  };
}

function deriveWsUrl(): string {
  const proto = location.protocol === "https:" ? "wss:" : "ws:";
  return `${proto}//${location.host}/quest`;
}
