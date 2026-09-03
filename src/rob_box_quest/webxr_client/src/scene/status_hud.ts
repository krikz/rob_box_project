// Status HUD (Wave 3.A / ADR-0027 R8): battery, Wi-Fi, скорость, режим, RTT.
// + AV-17: MODE (avatar_supervisor), FLOOR T / FLOOR V.
//
// Живёт слева вверху на стене мостика — зеркально ARM-индикатору справа.
// Sprite всегда повёрнут к оператору, поэтому читается из любой позы.
//
// Разделение как в остальном клиенте: формат строк — чистая логика
// (`formatStatusLines`, тестируется без Three.js/DOM), рисование —
// canvas + CanvasTexture.

import * as THREE from "three";
import { decodeMsgpackMap } from "../wire/msgpack";
import { floorLabel, type FloorLabel, type SupervisorState } from "../state/supervisor_state";

/** robot_status (0x1201), meta-quest-api.md §4 + поле battery_v (Wave 3.A). */
export interface RobotStatus {
  battery_pct: number;
  battery_v: number | null;
  wifi_rssi: number;
  mode: string;
  vel_linear: number;
  vel_angular: number;
  ts_ms: number;
}

export interface StatusLine {
  label: string;
  value: string;
  /** "ok" | "warn" | "bad" | "unknown" — цвет значения в HUD. */
  level: "ok" | "warn" | "bad" | "unknown";
}

/** Порог «низкий заряд» (robot_alert BATTERY_LOW, meta-quest-api.md §6). */
export const BATTERY_LOW_PCT = 20;
/** Порог «слабый Wi-Fi» (robot_alert WIFI_WEAK). */
export const WIFI_WEAK_DBM = -75;
/** Round-trip выше — телеоп уже некомфортный (ADR-0027 §2 latency budget). */
export const RTT_WARN_MS = 200;
export const RTT_BAD_MS = 400;

// AV-26 / R7: robot_alert метка в HUD. Когда алёрт активен, в нижней
// части спрайта появляется красная строка с текстом. Показывается до тех
// пор, пока сервер не пришлёт active:false (с явным code).
// Текст приходит с сервера уже локализованный (alertText() в alert_toast.ts
// использует ту же таблицу), но в HUD рисуем именно то, что сказал сервер
// (server-side i18n согласован с клиентским).

const ALERT_BG = "rgba(225, 27, 36, 0.92)";
const ALERT_BG_WARN = "rgba(245, 194, 17, 0.92)";
const ALERT_TEXT_COLOR = "#0a0d11";

/** Размеры алёрт-строки (в px канваса 512×320). */
const ALERT_LINE_HEIGHT = 56;
const ALERT_PADDING_X = 16;
const ALERT_PADDING_Y = 8;
const ALERT_FONT = "bold 28px monospace";

/**
 * Разобрать msgpack-payload robot_status. `null` — кадр битый или не map;
 * отсутствующие поля заполняются sentinel'ами сервера (-1 / 0), чтобы UI
 * ниже мог отличить «нет источника» от реального значения.
 */
export function parseRobotStatus(payload: Uint8Array): RobotStatus | null {
  const map = decodeMsgpackMap(payload);
  if (!map) return null;
  const num = (v: unknown, fallback: number): number =>
    typeof v === "number" && Number.isFinite(v) ? v : fallback;
  return {
    battery_pct: num(map.battery_pct, -1),
    battery_v: typeof map.battery_v === "number" ? map.battery_v : null,
    wifi_rssi: num(map.wifi_rssi, 0),
    mode: typeof map.mode === "string" ? map.mode : "unknown",
    vel_linear: num(map.vel_linear, 0),
    vel_angular: num(map.vel_angular, 0),
    ts_ms: num(map.ts_ms, 0)
  };
}

/**
 * Строки HUD. Отсутствующий источник показывается прочерком, а не нулём —
 * «0%» и «нет данных о заряде» для оператора это разные вещи.
 *
 * `fps` — текущее значение FPS (или `null`, если данных ещё нет). По
 * дизайну (AV-25 / B4) строка FPS идёт сразу после RTT: оператор
 * читает «как идут кадры» рядом с «как идёт сеть».
 */
export function formatStatusLines(
  status: RobotStatus | null,
  rttMs: number | null,
  fps: number | null = null
): StatusLine[] {
  const lines: StatusLine[] = [];

  // BAT: проценты, если источник есть; иначе вольты; иначе прочерк.
  if (status && status.battery_pct >= 0) {
    lines.push({
      label: "BAT",
      value: `${Math.round(status.battery_pct)}%`,
      level: status.battery_pct <= BATTERY_LOW_PCT ? "bad" : "ok"
    });
  } else if (status && status.battery_v !== null) {
    lines.push({ label: "BAT", value: `${status.battery_v.toFixed(1)} V`, level: "ok" });
  } else {
    lines.push({ label: "BAT", value: "—", level: "unknown" });
  }

  // WIFI: sentinel 0 = источника нет (на Vision Pi не читается /proc/net/wireless).
  if (status && status.wifi_rssi !== 0) {
    lines.push({
      label: "WIFI",
      value: `${status.wifi_rssi} dBm`,
      level: status.wifi_rssi <= WIFI_WEAK_DBM ? "warn" : "ok"
    });
  } else {
    lines.push({ label: "WIFI", value: "—", level: "unknown" });
  }

  lines.push({
    label: "SPD",
    value: status ? `${status.vel_linear.toFixed(2)} m/s` : "—",
    level: status ? "ok" : "unknown"
  });

  if (rttMs === null) {
    lines.push({ label: "RTT", value: "—", level: "unknown" });
  } else {
    lines.push({
      label: "RTT",
      value: `${Math.round(rttMs)} ms`,
      level: rttMs >= RTT_BAD_MS ? "bad" : rttMs >= RTT_WARN_MS ? "warn" : "ok"
    });
  }

  // FPS (AV-25): рядом с RTT, обновляется реже (раз в 500мс), но строка
  // живёт в том же формате. < 30 fps = жёлтый, < 15 = красный, иначе ok.
  if (fps === null || !Number.isFinite(fps) || fps <= 0) {
    lines.push({ label: "FPS", value: "—", level: "unknown" });
  } else {
    lines.push({
      label: "FPS",
      value: `${Math.round(fps)}`,
      level: fps < 15 ? "bad" : fps < 30 ? "warn" : "ok"
    });
  }

  // MODE/Teleop (robot_status.mode — старая семантика, см. ADR-0027 R8):
  // отражает teleop-состояние ЭТОГО клиента (idle/teleop_active/emergency),
  // а не FSM аватара. До AV-17 HUD показывал именно его как «MODE»; после
  // AV-17 эта строка переименована в «TELEOP», чтобы не путать с
  // avatar_supervisor.mode (formatSupervisorLines ниже).
  lines.push({
    label: "TELEOP",
    value: status ? status.mode : "—",
    level: status && status.mode === "emergency" ? "bad" : status ? "ok" : "unknown"
  });

  return lines;
}

/**
 * Строки HUD по avatar_supervisor (AV-17): MODE (режим аватара), FLOOR T
 * (teleop-floor), FLOOR V (voice-floor). Если `state === null` — STATE_UPDATE
 * ещё не пришёл: показываем `?` (ADR-0018 «неизвестно ≠ свободно»).
 *
 * `floorLabel` — результат `floorLabel()`: `"my"` / `"other"` / `"free"` /
 * `"unknown"`. В цвете: «my» = ok, «free» = ok, «other» = warn, «unknown»
 * = unknown. Дополнительно: если `state` показывает `"avatar_present"` и
 * другой клиент держит teleop-floor — это тоже «bad» для нас.
 */
export function formatSupervisorLines(
  state: SupervisorState | null,
  _myClientId: string | null,
  teleopLabel: FloorLabel,
  voiceLabel: FloorLabel
): StatusLine[] {
  const lines: StatusLine[] = [];
  // MODE (avatar_supervisor)
  if (state === null) {
    lines.push({ label: "MODE", value: "?", level: "unknown" });
  } else {
    const level: StatusLine["level"] =
      state.mode === "off" ? "warn" : state.mode === "avatar_present" ? "ok" : "ok";
    lines.push({ label: "MODE", value: state.mode, level });
  }

  // FLOOR T (teleop)
  lines.push({
    label: "FLOOR T",
    value: state === null ? "?" : floorLabelText(teleopLabel),
    level:
      state === null
        ? "unknown"
        : teleopLabel === "my"
        ? "ok"
        : teleopLabel === "free"
        ? "ok"
        : teleopLabel === "other"
        ? "warn"
        : "unknown"
  });

  // FLOOR V (voice)
  lines.push({
    label: "FLOOR V",
    value: state === null ? "?" : floorLabelText(voiceLabel),
    level:
      state === null
        ? "unknown"
        : voiceLabel === "my"
        ? "ok"
        : voiceLabel === "free"
        ? "ok"
        : voiceLabel === "other"
        ? "warn"
        : "unknown"
  });

  return lines;
}

function floorLabelText(label: FloorLabel): string {
  switch (label) {
    case "my":
      return "my";
    case "other":
      return "other";
    case "free":
      return "free";
    case "unknown":
    default:
      return "?";
  }
}

/**
 * Деградация: сервер на v1 subprotocol, supervisor-API недоступно.
 * Одна строка-плашка: «SUPERVISOR: v1 (no coordination)». UI должен
 * показывать её явно, чтобы оператор знал, что floor-ов сейчас нет.
 */
export const SUPERVISOR_DEGRADED_NOTE = "SUPERVISOR: v1 (no coordination)";

const LEVEL_COLORS: Record<StatusLine["level"], string> = {
  ok: "#2ec27e",
  warn: "#f5c211",
  bad: "#e01b24",
  unknown: "#8b98a5"
};

export interface StatusHud {
  readonly sprite: THREE.Sprite;
  /** Новый robot_status с сервера (или `null` — данных ещё нет). */
  setStatus(status: RobotStatus | null): void;
  /** RTT из ping/pong (`null` — pong ещё не приходил). */
  setRtt(rttMs: number | null): void;
  /** FPS из scene loop (`null` — данных ещё нет). AV-25. */
  setFps(fps: number | null): void;
  /**
   * Supervisor-state (AV-17). `null` = STATE_UPDATE ещё не пришёл
   * (или сервер на v1 — тогда `degraded=true`).
   */
  setSupervisor(
    state: SupervisorState | null,
    myClientId: string | null,
    options?: { degraded?: boolean }
  ): void;
  /** AV-26: вывести плашку с активным robot_alert. `null` — скрыть. */
  setAlert(alert: { text: string; level: "warn" | "error" } | null): void;
  dispose(): void;
}

export interface StatusHudOptions {
  /** Позиция спрайта в сцене (по умолчанию — левый верх стены-экрана). */
  position?: { x: number; y: number; z: number };
  /** Размер спрайта в метрах (default 1.1 × 0.66). */
  scale?: { x: number; y: number };
}

export function createStatusHud(opts: StatusHudOptions = {}): StatusHud {
  const canvas = document.createElement("canvas");
  canvas.width = 512;
  canvas.height = 320;
  const ctx2d = canvas.getContext("2d");
  if (!ctx2d) {
    throw new Error("status_hud: failed to acquire 2D context");
  }
  // Явный const после guard: TS не переносит narrowing внутрь замыкания draw().
  const ctx: CanvasRenderingContext2D = ctx2d;
  const texture = new THREE.CanvasTexture(canvas);
  texture.minFilter = THREE.LinearFilter;
  texture.magFilter = THREE.LinearFilter;

  const sprite = new THREE.Sprite(
    new THREE.SpriteMaterial({ map: texture, depthTest: false, transparent: true })
  );
  const pos = opts.position ?? { x: -2.35, y: 2.85, z: -3.85 };
  const scale = opts.scale ?? { x: 1.1, y: 0.69 };
  sprite.position.set(pos.x, pos.y, pos.z);
  sprite.scale.set(scale.x, scale.y, 1);

  let status: RobotStatus | null = null;
  let rttMs: number | null = null;
  let fps: number | null = null;
  // AV-17: supervisor-state. `null` = неизвестно (STATE_UPDATE ещё не пришёл).
  let supervisor: SupervisorState | null = null;
  let supervisorMyClientId: string | null = null;
  let supervisorDegraded = false;
  // Кэш последних floorLabel'ов (зависят от myClientId). Пересчитываем
  // только при изменении state или myClientId, не на каждый draw().
  let teleopLabel: FloorLabel = "unknown";
  let voiceLabel: FloorLabel = "unknown";

  function recomputeFloorLabels(): void {
    if (supervisor === null) {
      teleopLabel = "unknown";
      voiceLabel = "unknown";
      return;
    }
    teleopLabel = floorLabel(supervisor, "teleop", supervisorMyClientId);
    voiceLabel = floorLabel(supervisor, "voice", supervisorMyClientId);
  }
  let alert: { text: string; level: "warn" | "error" } | null = null;

  function draw(): void {
    // AV-25 (FPS): передаём fps в formatStatusLines.
    // AV-26: если есть активный алёрт — перекрашиваем фон HUD плашкой,
    // обычные строки рисуем поверх (они не гаснут, оператор всё ещё
    // видит заряд/связь/RTT/FPS).
    const lines = formatStatusLines(status, rttMs, fps);
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    if (alert !== null) {
      ctx.fillStyle = alert.level === "error" ? ALERT_BG : ALERT_BG_WARN;
      ctx.fillRect(0, 0, canvas.width, ALERT_LINE_HEIGHT + ALERT_PADDING_Y * 2);
      ctx.fillStyle = ALERT_TEXT_COLOR;
      ctx.font = ALERT_FONT;
      ctx.textBaseline = "middle";
      // Простейший wrap по длине строки — без измерений ширины глифов
      // (canvas.measureText дорого в каждом кадре). Текст на русском,
      // ~30 символов обычно влезает; дальше оператор увидит toast-стек.
      const text = alert.text;
      ctx.fillText(text, ALERT_PADDING_X, ALERT_LINE_HEIGHT / 2 + ALERT_PADDING_Y);
    } else {
      ctx.fillStyle = "rgba(10, 13, 17, 0.72)";
      ctx.fillRect(0, 0, canvas.width, canvas.height);
    }
    ctx.textBaseline = "middle";

    // AV-17: supervisor-строки идут ПЕРЕД robot_status — оператор хочет
    // видеть «кто сейчас рулит» сверху (самый важный индикатор).
    if (supervisorDegraded) {
      lines.unshift({ label: "SUP", value: SUPERVISOR_DEGRADED_NOTE, level: "warn" });
    } else {
      const sup = formatSupervisorLines(supervisor, supervisorMyClientId, teleopLabel, voiceLabel);
      for (let i = sup.length - 1; i >= 0; i -= 1) lines.unshift(sup[i]);
    }

    // Если алёрт активен — строки сдвигаем вниз, чтобы не перекрывать.
    const topOffset = alert !== null ? ALERT_LINE_HEIGHT + ALERT_PADDING_Y * 2 : 0;
    const rowH = (canvas.height - topOffset) / lines.length;
    lines.forEach((line, i) => {
      const y = topOffset + rowH * i + rowH / 2;
      ctx.fillStyle = "#8b98a5";
      ctx.font = "bold 28px monospace";
      ctx.fillText(line.label, 20, y);
      ctx.fillStyle = LEVEL_COLORS[line.level];
      ctx.font = "bold 28px monospace";
      // Длинный текст (SUPERVISOR: v1 …) чуть сжимаем, чтобы влез.
      const fontPx = ctx.measureText(line.value).width;
      if (fontPx > canvas.width - 130) {
        ctx.font = "bold 22px monospace";
      }
      ctx.fillText(line.value, 130, y);
    });
    texture.needsUpdate = true;
  }

  draw();

  return {
    sprite,
    setStatus(next: RobotStatus | null): void {
      status = next;
      draw();
    },
    setRtt(next: number | null): void {
      rttMs = next;
      draw();
    },
    setFps(next: number | null): void {
      fps = next;
      draw();
    },
    setSupervisor(
      next: SupervisorState | null,
      myClientId: string | null,
      options?: { degraded?: boolean }
    ): void {
      supervisor = next;
      supervisorMyClientId = myClientId;
      supervisorDegraded = options?.degraded ?? false;
      recomputeFloorLabels();
      draw();
    },
    setAlert(next: { text: string; level: "warn" | "error" } | null): void {
      alert = next;
      draw();
    },
    dispose(): void {
      texture.dispose();
      (sprite.material as THREE.SpriteMaterial).dispose();
    }
  };
}
