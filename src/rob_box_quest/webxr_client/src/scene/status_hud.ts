// Status HUD (Wave 3.A / ADR-0027 R8): battery, Wi-Fi, скорость, режим, RTT.
//
// Живёт слева вверху на стене мостика — зеркально ARM-индикатору справа.
// Sprite всегда повёрнут к оператору, поэтому читается из любой позы.
//
// Разделение как в остальном клиенте: формат строк — чистая логика
// (`formatStatusLines`, тестируется без Three.js/DOM), рисование —
// canvas + CanvasTexture.

import * as THREE from "three";
import { decodeMsgpackMap } from "../wire/msgpack";

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

  lines.push({
    label: "MODE",
    value: status ? status.mode : "—",
    level: status && status.mode === "emergency" ? "bad" : status ? "ok" : "unknown"
  });

  return lines;
}

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
    // Если алёрт активен — строки сдвигаем вниз, чтобы не перекрывать.
    const topOffset = alert !== null ? ALERT_LINE_HEIGHT + ALERT_PADDING_Y * 2 : 0;
    const rowH = (canvas.height - topOffset) / lines.length;
    lines.forEach((line, i) => {
      const y = topOffset + rowH * i + rowH / 2;
      ctx.fillStyle = "#8b98a5";
      ctx.font = "bold 32px monospace";
      ctx.fillText(line.label, 20, y);
      ctx.fillStyle = LEVEL_COLORS[line.level];
      ctx.font = "bold 36px monospace";
      ctx.fillText(line.value, 180, y);
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
