// 3D-панель «ПОТОКИ» (issue #3150): что приходит с робота и сколько это стоит.
//
// Оператор по интернету выбирает профиль подписок (LAN / Интернет / Минимум /
// Свой), включает/выключает поток и крутит его частоту (max_hz), а рядом
// видит живые кбит/с и fps по каждому потоку и сумму.
//
// Техника как у supervisor_panel.ts: canvas → CanvasTexture на Plane-меше,
// каждая кнопка — отдельный прозрачный меш для PointerSystem (prefix `str:`).
// Состояние и транспорт живут в `SubscriptionManager` (state/), сцена только
// рисует `SubscriptionsView` и сообщает клики через `onStreamsAction`.
//
// Геометрия: +105° от оператора, радиус 2.4 м — зеркально панели режимов
// (−105°). Справа от панели голосового пайплайна (+60°, ширина 0.95 м
// ≈ ±11°), за правым крылом TARS2 по азимуту не пересекается: крыло лежит
// в секторе ≈ 32°..88° на дальности 4..5.5 м, панель — 94°..116° на 2.4 м.

import * as THREE from "three";
import { panelGeometry } from "./supervisor_panel";
import {
  PROFILE_IDS,
  PROFILE_LABELS,
  type ProfileId,
  type SubscriptionsView
} from "../state/subscription_manager";

export const STREAMS_PANEL_ANGLE_DEG = 105;
export const STREAMS_PANEL_RADIUS_M = 2.4;
export const STREAMS_PANEL_Y_M = 1.45;
export const STREAMS_PANEL_W_M = 0.95;
const CANVAS_W = 512;
const CANVAS_H = 640;
export const STREAMS_PANEL_H_M = (STREAMS_PANEL_W_M * CANVAS_H) / CANVAS_W;

/** Префикс id кнопок панели в PointerSystem. */
export const STREAMS_TARGET_PREFIX = "str:";

export type StreamsAction =
  | { kind: "profile"; profile: ProfileId }
  | { kind: "toggle"; topic: string }
  | { kind: "rate"; topic: string };

export function streamsTargetId(action: StreamsAction): string {
  if (action.kind === "profile") return `${STREAMS_TARGET_PREFIX}profile:${action.profile}`;
  return `${STREAMS_TARGET_PREFIX}${action.kind}:${action.topic}`;
}

/** Разбор id цели; `null` — это не кнопка панели потоков. */
export function parseStreamsTargetId(id: string): StreamsAction | null {
  if (!id.startsWith(STREAMS_TARGET_PREFIX)) return null;
  const rest = id.slice(STREAMS_TARGET_PREFIX.length);
  const sep = rest.indexOf(":");
  if (sep <= 0) return null;
  const kind = rest.slice(0, sep);
  const arg = rest.slice(sep + 1);
  if (!arg) return null;
  if (kind === "profile") {
    const profile = PROFILE_IDS.find((p) => p === arg);
    return profile ? { kind: "profile", profile } : null;
  }
  if (kind === "toggle" || kind === "rate") return { kind, topic: arg };
  return null;
}

export interface Rect {
  x: number;
  y: number;
  w: number;
  h: number;
}

export interface StreamsLayout {
  header: Rect;
  profiles: Array<{ profile: ProfileId; rect: Rect }>;
  rows: Array<{ topic: string; toggle: Rect; rate: Rect; stats: Rect }>;
}

/** Раскладка в пикселях канваса. Чистая функция — тестируется без Three.js. */
export function computeStreamsLayout(topics: readonly string[], w = CANVAS_W, h = CANVAS_H): StreamsLayout {
  const pad = 8;
  const header = { x: pad, y: pad, w: w - 2 * pad, h: 56 };
  const profY = header.y + header.h + pad;
  const profH = 56;
  const profW = (w - pad * (PROFILE_IDS.length + 1)) / PROFILE_IDS.length;
  const profiles = PROFILE_IDS.map((profile, i) => ({
    profile,
    rect: { x: pad + i * (profW + pad), y: profY, w: profW, h: profH }
  }));
  const rowsTop = profY + profH + pad * 2;
  const rowH = topics.length > 0 ? Math.min(64, (h - rowsTop - pad) / topics.length) : 0;
  const toggleW = Math.round(w * 0.52);
  const rateW = Math.round(w * 0.18);
  const rows = topics.map((topic, i) => {
    const y = rowsTop + i * rowH;
    const inner = rowH - pad / 2;
    return {
      topic,
      toggle: { x: pad, y, w: toggleW, h: inner },
      rate: { x: pad * 2 + toggleW, y, w: rateW, h: inner },
      stats: { x: pad * 3 + toggleW + rateW, y, w: w - (pad * 4 + toggleW + rateW), h: inner }
    };
  });
  return { header, profiles, rows };
}

export function formatKbps(kbps: number): string {
  if (!Number.isFinite(kbps) || kbps < 0.05) return "0 кбит/с";
  if (kbps < 10) return `${kbps.toFixed(1)} кбит/с`;
  if (kbps < 1000) return `${Math.round(kbps)} кбит/с`;
  return `${(kbps / 1000).toFixed(1)} Мбит/с`;
}

export function formatRate(maxHz: number | null): string {
  return maxHz === null ? "макс" : `${maxHz} Гц`;
}

export interface StreamsPanelTarget {
  id: string;
  object: THREE.Object3D;
}

export interface StreamsPanelHandle {
  object: THREE.Group;
  targets(): StreamsPanelTarget[];
  /**
   * Перерисовать. Если набор потоков изменился, хит-меши пересобираются и
   * возвращается `true` — владелец сцены должен перерегистрировать цели.
   */
  render(view: SubscriptionsView): boolean;
  dispose(): void;
}

const COLORS = {
  panel: "rgba(10, 13, 17, 0.92)",
  accent: "#2ec27e",
  mute: "#8b98a5",
  text: "#d6dde5",
  idle: "rgba(28, 33, 39, 0.92)",
  off: "#1c2127"
};

export function createStreamsPanel(): StreamsPanelHandle {
  const canvas = document.createElement("canvas");
  canvas.width = CANVAS_W;
  canvas.height = CANVAS_H;
  const ctx2d = canvas.getContext("2d");
  if (!ctx2d) throw new Error("streams_panel: failed to acquire 2D context");
  const ctx: CanvasRenderingContext2D = ctx2d;
  const texture = new THREE.CanvasTexture(canvas);
  texture.minFilter = THREE.LinearFilter;
  texture.magFilter = THREE.LinearFilter;

  const group = new THREE.Group();
  group.renderOrder = 15; // ниже stream_menu (20), как у панели режимов
  const geom = panelGeometry(STREAMS_PANEL_ANGLE_DEG, STREAMS_PANEL_RADIUS_M, STREAMS_PANEL_Y_M);
  group.position.set(geom.position.x, geom.position.y, geom.position.z);
  group.rotation.y = Math.atan2(geom.facing.x, geom.facing.z);

  const mesh = new THREE.Mesh(
    new THREE.PlaneGeometry(STREAMS_PANEL_W_M, STREAMS_PANEL_H_M),
    new THREE.MeshBasicMaterial({ map: texture, transparent: true, depthTest: false })
  );
  mesh.renderOrder = 15;
  group.add(mesh);

  const hitMat = new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0, depthTest: false });
  const hitMeshes = new Map<string, THREE.Mesh>();
  let topicsKey = "";

  function addHit(id: string, r: Rect): void {
    const sx = STREAMS_PANEL_W_M / CANVAS_W;
    const sy = STREAMS_PANEL_H_M / CANVAS_H;
    const m = new THREE.Mesh(new THREE.PlaneGeometry(r.w * sx, r.h * sy), hitMat);
    m.position.set(-STREAMS_PANEL_W_M / 2 + (r.x + r.w / 2) * sx, STREAMS_PANEL_H_M / 2 - (r.y + r.h / 2) * sy, 0.005);
    m.renderOrder = 16;
    group.add(m);
    hitMeshes.set(id, m);
  }

  function rebuildHits(layout: StreamsLayout): void {
    for (const m of hitMeshes.values()) {
      group.remove(m);
      m.geometry.dispose();
    }
    hitMeshes.clear();
    for (const p of layout.profiles) addHit(streamsTargetId({ kind: "profile", profile: p.profile }), p.rect);
    for (const row of layout.rows) {
      addHit(streamsTargetId({ kind: "toggle", topic: row.topic }), row.toggle);
      addHit(streamsTargetId({ kind: "rate", topic: row.topic }), row.rate);
    }
  }

  function drawButton(r: Rect, label: string, active: boolean, dim = false): void {
    ctx.fillStyle = active ? COLORS.accent : dim ? COLORS.off : COLORS.idle;
    ctx.fillRect(r.x, r.y, r.w, r.h);
    ctx.fillStyle = active ? "#0a0d11" : dim ? COLORS.mute : COLORS.text;
    ctx.font = `bold ${Math.max(12, Math.min(20, Math.floor(r.h * 0.4)))}px monospace`;
    ctx.textBaseline = "middle";
    ctx.fillText(label, r.x + 8, r.y + r.h / 2);
  }

  function draw(view: SubscriptionsView, layout: StreamsLayout): void {
    ctx.clearRect(0, 0, CANVAS_W, CANVAS_H);
    ctx.fillStyle = COLORS.panel;
    ctx.fillRect(0, 0, CANVAS_W, CANVAS_H);
    ctx.textBaseline = "middle";
    ctx.fillStyle = COLORS.accent;
    ctx.font = "bold 26px monospace";
    ctx.fillText("ПОТОКИ", layout.header.x + 4, layout.header.y + layout.header.h / 2);
    ctx.fillStyle = COLORS.text;
    ctx.font = "bold 22px monospace";
    ctx.fillText(`Σ ${formatKbps(view.totalKbps)}`, layout.header.x + 180, layout.header.y + layout.header.h / 2);
    for (const p of layout.profiles) drawButton(p.rect, PROFILE_LABELS[p.profile], view.profile === p.profile);
    for (const row of layout.rows) {
      const s = view.streams.find((x) => x.topic === row.topic);
      if (!s) continue;
      drawButton(row.toggle, `${s.enabled ? "●" : "○"} ${s.topic}`, false, !s.enabled);
      drawButton(row.rate, formatRate(s.maxHz), false, !s.enabled);
      ctx.fillStyle = s.enabled ? COLORS.text : COLORS.mute;
      ctx.font = "14px monospace";
      ctx.fillText(formatKbps(s.kbps), row.stats.x, row.stats.y + row.stats.h * 0.3);
      ctx.fillText(`${s.fps.toFixed(1)} fps`, row.stats.x, row.stats.y + row.stats.h * 0.72);
    }
    texture.needsUpdate = true;
  }

  function render(view: SubscriptionsView): boolean {
    const topics = view.streams.map((s) => s.topic);
    const layout = computeStreamsLayout(topics);
    const key = topics.join("|");
    const changed = key !== topicsKey;
    if (changed) {
      topicsKey = key;
      rebuildHits(layout);
    }
    draw(view, layout);
    return changed;
  }

  return {
    object: group,
    targets() {
      return [...hitMeshes.entries()].map(([id, object]) => ({ id, object }));
    },
    render,
    dispose() {
      for (const m of hitMeshes.values()) m.geometry.dispose();
      hitMat.dispose();
      texture.dispose();
      (mesh.material as THREE.Material).dispose();
      mesh.geometry.dispose();
    }
  };
}
