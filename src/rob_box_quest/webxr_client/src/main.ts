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
import { PRESET_ORDER } from "./scene/voice_pipeline_panel";
import type {
  VoiceLanguage,
  VoicePresetId,
  VoicePresetInfo
} from "./wire/messages";
import { createToast, type Toast } from "./ui/toast";
import { supervisorEffect, type FloorLabel, type SupervisorState } from "./state/supervisor_state";
import { createPreviewAudioSink, type PreviewAudioSink } from "./ui/preview_audio_sink";
import {
  INITIAL_TTS_PICKER_STATE,
  PREVIEW_TEXT,
  newPreviewRequestId,
  ttsPickerReducer,
  type TtsPickerAction,
  type TtsPickerState,
  type TtsPickerTarget
} from "./state/tts_picker_state";
import type { JsonEvent, VoiceInfo, VoicePreset } from "./wire/messages";
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

  // W6-2 / спека §3.6: панель голосового пайплайна теперь 3D на мостике
  // (bridge.voicePipeline). Здесь — только клиентское состояние тумблеров
  // STT/LLM (намерение оператора) и маппинг в wire-команды. Дефолт —
  // ttts_proxy (STT вкл, LLM выкл) — ровно то, что шлёт applyVoicePtt при
  // входе в робот-голос; voice_mode_ack синхронизирует состояние.
  let pipelineSttOn: boolean | null = null;
  let pipelineLlmOn: boolean | null = null;
  // AV-28 §P7: пока не пришёл список пресетов — кнопки заблокированы.
  let voicePresetsLoading = true;

  /** Смена стиля речи / языка вывода → `set_voice` (AV-28 §P7). */
  function sendStyleChange(preset?: VoicePresetId, language?: VoiceLanguage): void {
    const c = conn;
    if (!c || disconnected) return;
    const snap = modeManager.snapshot();
    c.send({
      cmd: "set_voice",
      ts_ms: Date.now(),
      voice_id: snap.currentVoice ?? "",
      preset: preset ?? snap.currentPreset ?? DEFAULT_VOICE_PRESET,
      language: language ?? snap.currentLanguage ?? DEFAULT_VOICE_LANGUAGE
    });
  }

  /**
   * WIRE_TO_VOICE_INPUT_MODE (quest_node.py) → тумблеры панели пайплайна:
   *   passthrough → STT выкл (рация);
   *   ttts_proxy → STT вкл, LLM выкл (STT→TTS дословно);
   *   stt_llm / llm_formalize → STT вкл, LLM вкл (полный пайплайн);
   *   off / прочее → неизвестно (показываем «…»).
   */
  function applyVoiceModeToPipeline(mode: string | undefined): void {
    switch (mode) {
      case "passthrough":
        pipelineSttOn = false;
        pipelineLlmOn = false;
        break;
      case "ttts_proxy":
        pipelineSttOn = true;
        pipelineLlmOn = false;
        break;
      case "stt_llm":
      case "llm_formalize":
        pipelineSttOn = true;
        pipelineLlmOn = true;
        break;
      default:
        pipelineSttOn = null;
        pipelineLlmOn = null;
        break;
    }
    bridge.voicePipeline.setSttOn(pipelineSttOn);
    bridge.voicePipeline.setLlmOn(pipelineLlmOn);
  }

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
    // AV-27: клик по TTS picker'у. Сцена уже открыла/закрыла меню сама,
    // здесь остаётся то, что требует сокета и стора.
    onTtsPickerAction: (action) => handleTtsPickerAction(action),
    // Клик по кнопке панели супервизора (R14): маппим action → JSON_CMD.
    onSupervisorAction: (action) => {
      if (!conn || disconnected) return;
      const cmd = supervisorActionToCmd(action, bridge.supervisorPanel);
      if (cmd === null) return;
      conn.send(cmd);
    },
    // Клик по панели голосового пайплайна (W6-2 / спека §3.6).
    onPipelineAction: (action) => {
      if (!conn || disconnected) {
        // Нет соединения: оптимистичную подсветку откатываем к стора.
        if (action.kind === "preset") {
          bridge.voicePipeline.setCurrentPreset(modeManager.snapshot().currentPreset);
        } else if (action.kind === "lang") {
          bridge.voicePipeline.setCurrentLanguage(modeManager.snapshot().currentLanguage);
        }
        return;
      }
      switch (action.kind) {
        case "stt": {
          const nextStt = !pipelineSttOn;
          pipelineSttOn = nextStt;
          const mode = !nextStt ? "passthrough" : pipelineLlmOn ? "llm_formalize" : "ttts_proxy";
          bridge.voicePipeline.setSttOn(nextStt);
          conn.send({ cmd: "voice_mode", ts_ms: Date.now(), mode });
          return;
        }
        case "llm": {
          const nextLlm = !pipelineLlmOn;
          pipelineLlmOn = nextLlm;
          const mode = nextLlm ? "llm_formalize" : "ttts_proxy";
          bridge.voicePipeline.setLlmOn(nextLlm);
          conn.send({ cmd: "voice_mode", ts_ms: Date.now(), mode });
          return;
        }
        case "tts": {
          toggleTtsPicker();
          return;
        }
        case "preset": {
          if (voicePresetsLoading) return;
          bridge.voicePipeline.setCurrentPreset(action.preset);
          modeManager.setCurrentPreset(action.preset);
          try {
            sendStyleChange(action.preset);
          } catch (err) {
            // eslint-disable-next-line no-console
            console.warn("[quest] set_voice preset send failed:", err);
            bridge.voicePipeline.setCurrentPreset(modeManager.snapshot().currentPreset);
          }
          return;
        }
        case "lang": {
          if (voicePresetsLoading) return;
          bridge.voicePipeline.setCurrentLanguage(action.language);
          modeManager.setCurrentLanguage(action.language);
          try {
            sendStyleChange(undefined, action.language);
          } catch (err) {
            // eslint-disable-next-line no-console
            console.warn("[quest] set_voice lang send failed:", err);
            bridge.voicePipeline.setCurrentLanguage(modeManager.snapshot().currentLanguage);
          }
          return;
        }
      }
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

  // ---- AV-27: TTS picker (list_voices / set_voice / preview_voice) ----------
  //
  // Стор — чистый редьюсер (`state/tts_picker_state.ts`), сцена — только
  // отрисовка (`scene/tts_picker_menu.ts`). Здесь связка с сокетом:
  //   open        → JSON_CMD{list_voices}
  //   select      → локально
  //   preview     → JSON_CMD{preview_voice, request_id}
  //   apply       → JSON_CMD{set_voice}
  // и обратно: voice_list / voice_set_ack / voice_set_nack /
  // preview_voice_audio+BINARY_FRAME / preview_voice_done / _error.

  let ttsState: TtsPickerState = INITIAL_TTS_PICKER_STATE;
  const previewSink: PreviewAudioSink = createPreviewAudioSink();
  // Первая отрисовка: меню скрыто, но текстуры готовы — при открытии не
  // будет кадра с пустыми плашками.
  bridge.renderTtsPicker(ttsState);

  function dispatchTts(action: TtsPickerAction): void {
    const next = ttsPickerReducer(ttsState, action);
    if (next === ttsState) return;
    ttsState = next;
    bridge.renderTtsPicker(ttsState);
  }

  /** Отправить команду, если сокет жив. `false` — не отправили. */
  function sendCmd(cmd: Parameters<Connection["send"]>[0]): boolean {
    if (!conn || disconnected) return false;
    try {
      conn.send(cmd);
      return true;
    } catch (err) {
      // eslint-disable-next-line no-console
      console.warn("[quest] send failed:", (err as Error).message, cmd);
      return false;
    }
  }

  function requestVoiceList(): void {
    if (!sendCmd({ cmd: "list_voices", ts_ms: Date.now() })) {
      // Честно: не смогли спросить — не показываем «пусто», показываем
      // ошибку (пустой список означал бы «провайдер без голосов»).
      dispatchTts({ kind: "preview_error", requestId: "", reason: "not connected" });
    }
  }

  /**
   * Открыть/закрыть picker из кода (клавиша V на десктопе). Видимость
   * держит сцена, поэтому синхронизируем её и стор в одном месте.
   */
  function toggleTtsPicker(): void {
    if (bridge.isTtsPickerOpen()) {
      bridge.closeTtsPicker();
      previewSink.stop();
      dispatchTts({ kind: "close" });
      return;
    }
    bridge.openTtsPicker();
    dispatchTts({ kind: "open" });
    requestVoiceList();
  }

  function handleTtsPickerAction(action: TtsPickerTarget): void {
    switch (action.kind) {
      case "launch":
        // Сцена уже переключила видимость по клику: синхронизируем стор и,
        // если открылись, запрашиваем свежий список.
        if (bridge.isTtsPickerOpen()) {
          dispatchTts({ kind: "open" });
          requestVoiceList();
        } else {
          previewSink.stop();
          dispatchTts({ kind: "close" });
        }
        return;
      case "close":
        previewSink.stop();
        dispatchTts({ kind: "close" });
        return;
      case "select":
        dispatchTts({ kind: "select", voiceId: action.voiceId });
        return;
      case "preview": {
        const requestId = newPreviewRequestId();
        const ok = sendCmd({
          cmd: "preview_voice",
          ts_ms: Date.now(),
          voice_id: action.voiceId,
          text: PREVIEW_TEXT,
          request_id: requestId
        });
        if (!ok) return;
        previewSink.stop();
        dispatchTts({ kind: "preview_sent", requestId, voiceId: action.voiceId });
        return;
      }
      case "stop":
        previewSink.stop();
        dispatchTts({ kind: "preview_stopped" });
        return;
      case "apply": {
        const voiceId = ttsState.selectedVoiceId;
        if (!voiceId) return;
        const preset: VoicePreset | undefined = modeManager.snapshot().currentPreset ?? undefined;
        const ok = sendCmd({
          cmd: "set_voice",
          ts_ms: Date.now(),
          voice_id: voiceId,
          ...(preset ? { preset } : {})
        });
        if (!ok) return;
        // Лочим UI до ack/nack — второй set_voice до ответа не отправить.
        dispatchTts({ kind: "apply_sent", voiceId });
        return;
      }
      default:
        return;
    }
  }

  /** Голосовые JSON_EVENT'ы TTS picker'а. `true` — событие обработано здесь. */
  function handleVoiceEvent(ev: JsonEvent): boolean {
    const type = (ev as { type?: string }).type;
    switch (type) {
      case "voice_list": {
        const e = ev as { voices?: VoiceInfo[]; active_voice?: string; active_provider?: string };
        dispatchTts({
          kind: "voice_list",
          voices: Array.isArray(e.voices) ? e.voices : [],
          activeVoice: e.active_voice ?? null,
          activeProvider: e.active_provider ?? null
        });
        // Активный голос сервера — единственный источник истины для
        // mode_manager и панели пайплайна (клиент своё значение не выдумывает).
        if (typeof e.active_voice === "string") {
          modeManager.setCurrentVoice(e.active_voice);
          bridge.voicePipeline.setCurrentVoice(e.active_voice);
        }
        return true;
      }
      case "voice_set_ack": {
        const e = ev as { voice_id: string; preset?: VoicePreset };
        dispatchTts({ kind: "voice_set_ack", voiceId: e.voice_id, preset: e.preset });
        modeManager.setCurrentVoice(e.voice_id);
        if (e.preset) modeManager.setCurrentPreset(e.preset);
        return true;
      }
      case "voice_set_nack": {
        const e = ev as { voice_id?: string; reason: string; available?: string[] };
        dispatchTts({
          kind: "voice_set_nack",
          voiceId: e.voice_id ?? null,
          reason: e.reason,
          available: e.available
        });
        toast.show(`Голос не применён: ${e.reason}`, { level: "warn", autoHideMs: 5000 });
        return true;
      }
      case "preview_voice_audio": {
        const e = ev as { request_id: string; content_type?: string; seq: number; total: number };
        previewSink.onMeta(e.request_id, e.content_type ?? "audio/mpeg", e.seq, e.total);
        dispatchTts({
          kind: "preview_audio",
          requestId: e.request_id,
          seq: e.seq,
          total: e.total
        });
        return true;
      }
      case "preview_voice_done": {
        const e = ev as { request_id: string };
        dispatchTts({ kind: "preview_done", requestId: e.request_id });
        void previewSink.play(e.request_id);
        return true;
      }
      case "preview_voice_error": {
        const e = ev as { request_id: string; reason: string };
        previewSink.stop();
        dispatchTts({ kind: "preview_error", requestId: e.request_id, reason: e.reason });
        return true;
      }
      default:
        return false;
    }
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
            // AV-27: если picker открыт (например, разрыв случился при
            // открытом меню) — перезапрашиваем список: провайдер мог
            // смениться, а старый список после реконнекта не факт.
            if (bridge.isTtsPickerOpen()) requestVoiceList();
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
            // AV-27: активный голос/список после разрыва — не факт
            // (ADR-0018). Picker уходит в loading, preview обрывается.
            previewSink.stop();
            dispatchTts({ kind: "disconnected" });
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
          // AV-27: preview-аудио приходит с stream_id = 0 (control), у него
          // нет topic'а в subscribe_ack — маршрутизируем в preview-sink по
          // последней пришедшей мете preview_voice_audio.
          if (streamId === 0) {
            previewSink.onChunk(payload);
            return;
          }
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
          // AV-27: voice_list / voice_set_ack / voice_set_nack /
          // preview_voice_audio / _done / _error → TTS picker.
          // Диспетчер сам игнорирует чужие типы, поэтому зовём его
          // первым и не мешаем обработчикам ниже: voice_set_ack нужен
          // обоим — picker'у и панели пресетов AV-28.
          handleVoiceEvent(event as JsonEvent);
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
            applyVoiceModeToPipeline(mode);
            return;
          }

          // AV-28 §P7: voice_presets / voice_set_ack / voice_set_nack.
          // Свои типы событий, дальше по функции их уже не ждут.
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
              bridge.voicePipeline.setPresets(p.presets);
            }
            if (Array.isArray(p.languages) && p.languages.length > 0) {
              bridge.voicePipeline.setLanguages(p.languages);
            }
            // Дефолт сервера имеет приоритет над локальным fallback.
            const srvPreset =
              p.default_preset ?? modeManager.snapshot().currentPreset;
            const srvLang =
              p.default_language ?? modeManager.snapshot().currentLanguage;
            if (srvPreset) {
              bridge.voicePipeline.setCurrentPreset(srvPreset);
              modeManager.setCurrentPreset(srvPreset);
            }
            if (srvLang) {
              bridge.voicePipeline.setCurrentLanguage(srvLang);
              modeManager.setCurrentLanguage(srvLang);
            }
            voicePresetsLoading = false;
            bridge.voicePipeline.setLoading(false);
            return;
          }
          if (type === "voice_set_ack") {
            const ack = event as {
              voice_id?: string;
              preset?: string;
              language?: string;
            };
            // AV-27 (TTS picker): ack всегда с voice_id; preset там — пресет
            // провайдера (standard/friendly/...), а НЕ стиль речи AV-28.
            if (typeof ack.voice_id === "string" && ack.voice_id) {
              modeManager.setCurrentVoice(ack.voice_id);
              bridge.voicePipeline.setCurrentVoice(ack.voice_id);
            }
            // AV-28 (§P7): стиль речи только из whitelist'а PRESET_ORDER.
            const stylePreset =
              typeof ack.preset === "string" && (PRESET_ORDER as readonly string[]).includes(ack.preset)
                ? (ack.preset as VoicePresetId)
                : null;
            if (stylePreset) {
              modeManager.setCurrentPreset(stylePreset);
              bridge.voicePipeline.setCurrentPreset(stylePreset);
            }
            if (ack.language === "ru" || ack.language === "en") {
              modeManager.setCurrentLanguage(ack.language);
              bridge.voicePipeline.setCurrentLanguage(ack.language);
            }
            return;
          }
          if (type === "voice_set_nack") {
            // Сервер отказал — откатываем UI и mode_manager к последнему
            // известному «хорошему» значению из snapshot.
            const nack = event as { reason?: string };
            // eslint-disable-next-line no-console
            console.warn("[quest] voice_set_nack:", nack.reason ?? "(no reason)");
            const snap = modeManager.snapshot();
            bridge.voicePipeline.setCurrentPreset(snap.currentPreset);
            bridge.voicePipeline.setCurrentLanguage(snap.currentLanguage);
            errorOverlay.show(
              "Не удалось сменить голос",
              nack.reason ?? "Сервер отклонил запрос"
            );
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
    // AV-27: V — открыть/закрыть TTS picker на десктопе. В VR та же
    // операция делается кликом по вкладке VOICE (клавиатуры там нет).
    if (ev.key === "v" || ev.key === "V") {
      ev.preventDefault();
      toggleTtsPicker();
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
      previewSink.dispose();
      conn?.close();
      bridge.dispose();
      // Phase 2.3 overlays cleanup.
      loading.dispose();
      errorOverlay.dispose();
      help.dispose();
      watchdog.dispose();
      alertToast.dispose();
    }
  };
}

function deriveWsUrl(): string {
  const proto = location.protocol === "https:" ? "wss:" : "ws:";
  return `${proto}//${location.host}/quest`;
}
