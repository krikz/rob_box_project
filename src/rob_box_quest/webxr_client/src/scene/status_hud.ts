// Status HUD (Wave 3.A / ADR-0027 R8): battery, Wi-Fi, скорость, режим, RTT.
// + AV-17: MODE (avatar_supervisor), FLOOR T / FLOOR V.
//
// Живёт в «шапке» над экраном-стеной: широкая полоса-таблица (4 колонки ×
// N строк) над рамкой видео, левая часть HUD-полосы мостика (справа от неё
// — голос и ARM, см. HUD_STRIP в captain_bridge.ts). Раньше это был узкий
// столбец 1.1 × 0.69 м, который наезжал на верх видео и торчал выше потолка.
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

/** Размеры алёрт-строки (в px канваса STATUS_HUD_CANVAS_*). */
const ALERT_LINE_HEIGHT = 56;
const ALERT_PADDING_X = 20;
const ALERT_PADDING_Y = 6;
const ALERT_FONT = "bold 36px monospace";

/** Канвас полосы: 6:1, как спрайт 3.0 × 0.5 м (STATUS_HUD_SIZE_M). */
export const STATUS_HUD_CANVAS_W = 1536;
export const STATUS_HUD_CANVAS_H = 256;
/** Размер спрайта по умолчанию, м. Пропорции = пропорции канваса. */
export const STATUS_HUD_SIZE_M = { x: 3.0, y: 0.5 } as const;
/** Колонок в таблице статуса: 11 строк (MODE…NAV) → 3 ряда. */
export const STATUS_HUD_COLUMNS = 4;

export interface GridCell {
  x: number;
  y: number;
  w: number;
  h: number;
}

/**
 * Раскладка ячеек таблицы статуса в px канваса. Чистая функция.
 * Строки идут по рядам слева направо (порядок = важность: MODE первым).
 * `topOffset` — место под алёрт-плашку сверху.
 */
export function statusGridLayout(
  count: number,
  width = STATUS_HUD_CANVAS_W,
  height = STATUS_HUD_CANVAS_H,
  topOffset = 0,
  columns = STATUS_HUD_COLUMNS
): GridCell[] {
  if (count <= 0) return [];
  const rows = Math.ceil(count / columns);
  const pad = 8;
  const cellW = (width - pad * (columns + 1)) / columns;
  const cellH = (height - topOffset - pad * (rows + 1)) / rows;
  const cells: GridCell[] = [];
  for (let i = 0; i < count; i += 1) {
    const c = i % columns;
    const r = Math.floor(i / columns);
    cells.push({
      x: pad + c * (cellW + pad),
      y: topOffset + pad + r * (cellH + pad),
      w: cellW,
      h: cellH
    });
  }
  return cells;
}

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
  fps: number | null = null,
  netKbps: number | null = null
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

  // NET (issue #3150): суммарный входящий трафик подписок. Строки нет,
  // пока счётчик не подключён (null) — не рисуем «0» вместо «не меряли».
  if (netKbps !== null && Number.isFinite(netKbps)) {
    lines.push({
      label: "NET",
      value: netKbps < 1000 ? `${Math.round(netKbps)} kbit/s` : `${(netKbps / 1000).toFixed(1)} Mbit/s`,
      level: "ok"
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
  /** Суммарный входящий трафик подписок, кбит/с (issue #3150). */
  setBandwidth(kbps: number | null): void;
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
  /**
   * #3151: строка NAV (статус nav-цели, см. nav/nav_state.ts:navStatusLine).
   * `null` — навигации нет, строку не рисуем.
   */
  setNav(line: StatusLine | null): void;
  dispose(): void;
}

export interface StatusHudOptions {
  /** Позиция спрайта в сцене (по умолчанию — левый верх стены-экрана). */
  position?: { x: number; y: number; z: number };
  /** Размер спрайта в метрах (default STATUS_HUD_SIZE_M). */
  scale?: { x: number; y: number };
}

export function createStatusHud(opts: StatusHudOptions = {}): StatusHud {
  const canvas = document.createElement("canvas");
  canvas.width = STATUS_HUD_CANVAS_W;
  canvas.height = STATUS_HUD_CANVAS_H;
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
  // Поверх карты/лидара (renderOrder 5–10, тоже без depth-теста): иначе
  // точки скана, лежащие «за» полосой, прорисовывались бы по тексту.
  sprite.renderOrder = 14;
  const pos = opts.position ?? { x: -0.95, y: 3.35, z: -3.85 };
  const scale = opts.scale ?? STATUS_HUD_SIZE_M;
  sprite.position.set(pos.x, pos.y, pos.z);
  sprite.scale.set(scale.x, scale.y, 1);

  let status: RobotStatus | null = null;
  let rttMs: number | null = null;
  let fps: number | null = null;
  let netKbps: number | null = null;
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
  // #3151: строка NAV — последней, под статусом робота.
  let navLine: StatusLine | null = null;

  function draw(): void {
    // AV-25 (FPS): передаём fps в formatStatusLines.
    // AV-26: если есть активный алёрт — сверху полосы плашка во всю ширину,
    // таблица сжимается под неё (строки не гаснут: оператор всё ещё видит
    // заряд/связь/RTT/FPS).
    const lines = formatStatusLines(status, rttMs, fps, netKbps);
    const W = canvas.width;
    const H = canvas.height;
    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = "rgba(10, 13, 17, 0.82)";
    ctx.fillRect(0, 0, W, H);
    // Кант полосы — тот же holo, что у рамок мостика.
    ctx.strokeStyle = "rgba(68, 221, 255, 0.55)";
    ctx.lineWidth = 3;
    ctx.strokeRect(1.5, 1.5, W - 3, H - 3);
    ctx.textBaseline = "middle";

    const alertH = ALERT_LINE_HEIGHT + ALERT_PADDING_Y * 2;
    if (alert !== null) {
      ctx.fillStyle = alert.level === "error" ? ALERT_BG : ALERT_BG_WARN;
      ctx.fillRect(0, 0, W, alertH);
      ctx.fillStyle = ALERT_TEXT_COLOR;
      ctx.font = ALERT_FONT;
      ctx.fillText(alert.text, ALERT_PADDING_X, alertH / 2);
    }

    // AV-17: supervisor-строки идут ПЕРЕД robot_status — оператор хочет
    // видеть «кто сейчас рулит» первым (самый важный индикатор).
    if (supervisorDegraded) {
      lines.unshift({ label: "SUP", value: SUPERVISOR_DEGRADED_NOTE, level: "warn" });
    } else {
      const sup = formatSupervisorLines(supervisor, supervisorMyClientId, teleopLabel, voiceLabel);
      for (let i = sup.length - 1; i >= 0; i -= 1) lines.unshift(sup[i]);
    }
    if (navLine !== null) lines.push(navLine);

    const cells = statusGridLayout(lines.length, W, H, alert !== null ? alertH : 0);
    lines.forEach((line, i) => {
      const c = cells[i];
      const cy = c.y + c.h / 2;
      ctx.fillStyle = "rgba(28, 33, 39, 0.9)";
      ctx.fillRect(c.x, c.y, c.w, c.h);
      // Цветная метка уровня слева — видно и без чтения значения.
      ctx.fillStyle = LEVEL_COLORS[line.level];
      ctx.fillRect(c.x, c.y, 6, c.h);
      const labelPx = Math.max(16, Math.min(26, Math.floor(c.h * 0.36)));
      ctx.fillStyle = "#8b98a5";
      ctx.font = `bold ${labelPx}px monospace`;
      ctx.fillText(line.label, c.x + 16, cy);
      const valueX = c.x + 16 + labelPx * 0.62 * 7 + 8;
      const room = c.x + c.w - 10 - valueX;
      // Значение — крупно, но длинный текст (SUPERVISOR: v1 …, NAV)
      // ужимаем, чтобы он не вылезал в соседнюю ячейку.
      let valuePx = Math.max(16, Math.min(38, Math.floor(c.h * 0.5)));
      ctx.font = `bold ${valuePx}px monospace`;
      const wText = ctx.measureText(line.value).width;
      if (wText > room && wText > 0) {
        valuePx = Math.max(14, Math.floor((valuePx * room) / wText));
        ctx.font = `bold ${valuePx}px monospace`;
      }
      ctx.fillStyle = LEVEL_COLORS[line.level];
      ctx.fillText(line.value, valueX, cy);
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
    setBandwidth(next: number | null): void {
      netKbps = next;
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
    setNav(next: StatusLine | null): void {
      navLine = next;
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
