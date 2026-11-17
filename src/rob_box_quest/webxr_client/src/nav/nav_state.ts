// Состояние навигации на клиенте (issue #3151) — чистый редьюсер.
//
// Источники: наши команды (nav_goal / nav_cancel) и события сервера
// (nav_goal_ack / nav_goal_nack / nav_status / nav_cancel_ack). Из него
// рисуются пин цели, кнопка отмены и строка NAV на status-HUD.
//
// Честность строки (ADR-0018): `nav_goal_ack` — это «цель ушла в Nav2», а
// не «Nav2 принял». Пока нет nav_status{accepted} — пишем «ОТПРАВЛЕНО», не
// «ЕДЕТ». Разрыв связи — «нет связи», а не последнее известное «едет».

import type { StatusLine } from "../scene/status_hud";

export type NavPhase =
  | "idle"
  | "sending" // nav_goal отправлен, ждём ack/nack
  | "sent" // ack: цель ушла в Nav2, Nav2 ещё не ответил
  | "accepted"
  | "active"
  | "succeeded"
  | "aborted"
  | "canceled"
  | "rejected" // Nav2 отказал
  | "nacked" // мост отказал до Nav2
  | "unknown"; // связь потеряна посреди цели

export interface NavGoal {
  seq: number;
  /** Цель в `map`. */
  x: number;
  y: number;
  yaw: number;
}

export interface NavState {
  phase: NavPhase;
  goal: NavGoal | null;
  distanceM: number | null;
  reason: string | null;
}

export const INITIAL_NAV_STATE: NavState = { phase: "idle", goal: null, distanceM: null, reason: null };

export type NavAction =
  | { kind: "sent"; goal: NavGoal }
  | { kind: "ack"; seq: number }
  | { kind: "nack"; seq: number | null; reason: string }
  | {
      kind: "status";
      state: string;
      seq: number;
      x: number;
      y: number;
      yaw: number;
      distanceM: number | null;
      reason: string | null;
    }
  | { kind: "disconnected" };

const TERMINAL = new Set<NavPhase>(["succeeded", "aborted", "canceled", "rejected", "nacked"]);
const STATUS_PHASES = new Set<string>(["accepted", "rejected", "active", "succeeded", "aborted", "canceled"]);

export function isGoalLive(state: NavState): boolean {
  return state.goal !== null && !TERMINAL.has(state.phase) && state.phase !== "idle";
}

export function reduceNav(state: NavState, action: NavAction): NavState {
  switch (action.kind) {
    case "sent":
      return { phase: "sending", goal: action.goal, distanceM: null, reason: null };
    case "ack":
      if (state.phase !== "sending" || state.goal?.seq !== action.seq) return state;
      return { ...state, phase: "sent" };
    case "nack":
      if (state.phase !== "sending") return state;
      if (action.seq !== null && state.goal?.seq !== action.seq) return state;
      return { ...state, phase: "nacked", reason: action.reason };
    case "status":
      return reduceStatus(state, action);
    case "disconnected":
      return isGoalLive(state) ? { ...state, phase: "unknown", distanceM: null } : state;
  }
}

function reduceStatus(state: NavState, a: Extract<NavAction, { kind: "status" }>): NavState {
  if (!STATUS_PHASES.has(a.state)) return state;
  const phase = a.state as NavPhase;
  const sameGoal = state.goal?.seq === a.seq;
  // Терминальный статус чужой цели не гасит нашу живую (сервер не шлёт
  // статусы вытесненных целей, но и не доверяем этому вслепую).
  if (!sameGoal && TERMINAL.has(phase) && isGoalLive(state)) return state;
  return {
    phase,
    goal: { seq: a.seq, x: a.x, y: a.y, yaw: a.yaw },
    // Feedback без distance_remaining не затирает известное расстояние.
    distanceM: a.distanceM ?? (sameGoal && phase === "active" ? state.distanceM : null),
    reason: a.reason
  };
}

const REASON_TEXT: Record<string, string> = {
  floor_held: "руль у другого",
  emergency_active: "аварийный стоп",
  nav2_unavailable: "Nav2 недоступен",
  bad_frame: "не тот кадр",
  bad_payload: "битая цель",
  nav2_rejected: "Nav2 отказал"
};

export function reasonText(reason: string | null): string {
  if (!reason) return "";
  return REASON_TEXT[reason] ?? reason;
}

/** Строка NAV для status-HUD. `null` — навигации нет, строку не показываем. */
export function navStatusLine(state: NavState, aiming: boolean): StatusLine | null {
  if (aiming) return { label: "NAV", value: "ПРИЦЕЛ: пол", level: "warn" };
  switch (state.phase) {
    case "idle":
      return null;
    case "sending":
      return { label: "NAV", value: "отправка…", level: "unknown" };
    case "sent":
      return { label: "NAV", value: "ОТПРАВЛЕНО", level: "unknown" };
    case "accepted":
      return { label: "NAV", value: "ПРИНЯТО", level: "ok" };
    case "active":
      return {
        label: "NAV",
        value: state.distanceM !== null ? `ЕДЕТ ${state.distanceM.toFixed(1)} м` : "ЕДЕТ",
        level: "ok"
      };
    case "succeeded":
      return { label: "NAV", value: "ПРИЕХАЛ", level: "ok" };
    case "canceled":
      return { label: "NAV", value: "ОТМЕНЕНО", level: "unknown" };
    case "aborted":
      return { label: "NAV", value: withReason("СДАЛСЯ", state.reason), level: "bad" };
    case "rejected":
      return { label: "NAV", value: withReason("ОТКАЗ", state.reason), level: "bad" };
    case "nacked":
      return { label: "NAV", value: withReason("НЕ ПРИНЯТО", state.reason), level: "bad" };
    case "unknown":
      return { label: "NAV", value: "нет связи", level: "warn" };
  }
}

function withReason(head: string, reason: string | null): string {
  const r = reasonText(reason);
  return r ? `${head}: ${r}` : head;
}
