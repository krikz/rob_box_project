// Чистая модель состояния avatar_supervisor (AV-17 / docs/adr/0028-avatar-supervisor.md §4.4).
//
// Клиент получает `STATE_UPDATE` (frame 0x33) с msgpack-payload вида:
//   {
//     mode: "off" | "telegram_active" | "avatar_present" | "mixed"
//         | "teleop_only" | "voice_only",
//     floors: {
//       teleop: { client_id: "uuid" | null, since_ms: number } | null,
//       voice:  { client_id: "uuid" | null, since_ms: number } | null
//     },
//     updated_ms: number
//   }
// На этой карте:
//   - `null` в client_id  → floor свободен;
//   - `client_id === myClientId` → floor наш;
//   - иначе → floor держит другой оператор (Telegram-бот, второй Quest).
//
// Никаких зависимостей от three.js / DOM — чистая логика, тесты в
// tests/supervisor_state.test.ts (валидный map, map без floors, битый
// payload → null).

import type { MsgpackValue } from "../wire/msgpack";

/** Режим аватара, как его видит supervisor (ADR-0028 §4.1). */
export type AvatarMode =
  | "off"
  | "telegram_active"
  | "avatar_present"
  | "mixed"
  | "teleop_only"
  | "voice_only"
  | string; // forward-compat: неизвестный серверный режим не должен ломать UI

export type FloorName = "teleop" | "voice";

export interface FloorHolder {
  clientId: string | null;
  sinceMs: number;
}

export interface SupervisorState {
  mode: AvatarMode;
  teleopFloor: FloorHolder;
  voiceFloor: FloorHolder;
  /** Когда сервер прислал это состояние (мс с epoch). */
  updatedMs: number;
}

/** Каркас для случая «STATE_UPDATE ещё не пришёл» — отличается от «свободен». */
export const UNKNOWN_SUPERVISOR_STATE: SupervisorState = Object.freeze({
  mode: "?",
  teleopFloor: Object.freeze({ clientId: null, sinceMs: 0 }) as FloorHolder,
  voiceFloor: Object.freeze({ clientId: null, sinceMs: 0 }) as FloorHolder,
  updatedMs: 0
});

function asFloorHolder(raw: unknown): FloorHolder | null {
  if (raw === null || raw === undefined) return null;
  if (typeof raw !== "object" || Array.isArray(raw)) return null;
  const m = raw as Record<string, MsgpackValue>;
  const cid = m.client_id;
  const since = m.since_ms;
  return {
    clientId: typeof cid === "string" ? cid : cid === null ? null : null,
    sinceMs: typeof since === "number" && Number.isFinite(since) ? since : 0
  };
}

/**
 * Разобрать msgpack-map в `SupervisorState`. Возвращает `null`, если
 * payload не map или обязательные поля битые (mode/floors отсутствуют
 * или неправильного типа) — UI должен показать `?`, а не выдумывать
 * «off / свободен» (ADR-0018 честность в UI).
 */
export function parseSupervisorState(map: unknown): SupervisorState | null {
  if (map === null || typeof map !== "object" || Array.isArray(map)) return null;
  const m = map as Record<string, MsgpackValue>;
  const mode = m.mode;
  if (typeof mode !== "string") return null;

  // Основной формат — плоский, как его пакует сервер
  // (`rob_box_supervisor/core/state.py::pack`):
  //   {mode, teleop_floor, voice_floor, last_event, since_ms, version}
  // где *_floor = null | {client_id, since_ms, last_heartbeat_ms}.
  // Дополнительно принимаем вложенный вариант `floors: {teleop, voice}`
  // (черновик meta-quest-api §3) — forward-compat, если сервер сменит форму.
  let teleopRaw: MsgpackValue | undefined = m.teleop_floor;
  let voiceRaw: MsgpackValue | undefined = m.voice_floor;
  const floors = m.floors;
  const hasFlat = "teleop_floor" in m || "voice_floor" in m;
  const hasNested = floors !== null && typeof floors === "object" && !Array.isArray(floors);
  if (!hasFlat && !hasNested) return null;
  if (!hasFlat && hasNested) {
    const f = floors as Record<string, MsgpackValue>;
    teleopRaw = f.teleop;
    voiceRaw = f.voice;
  }
  const teleop = asFloorHolder(teleopRaw ?? null);
  const voice = asFloorHolder(voiceRaw ?? null);
  const updatedMsRaw = m.updated_ms ?? m.since_ms;
  const updatedMs =
    typeof updatedMsRaw === "number" && Number.isFinite(updatedMsRaw) ? updatedMsRaw : 0;
  return {
    mode,
    teleopFloor: teleop ?? { clientId: null, sinceMs: 0 },
    voiceFloor: voice ?? { clientId: null, sinceMs: 0 },
    updatedMs
  };
}

/** Floor свободен у всех (наш myClientId тут не важен). */
export function floorIsFree(state: SupervisorState, floor: FloorName): boolean {
  const holder = floor === "teleop" ? state.teleopFloor : state.voiceFloor;
  return holder.clientId === null;
}

/** Floor держит наш клиент. */
export function floorIsMine(state: SupervisorState, floor: FloorName, myClientId: string | null): boolean {
  if (myClientId === null) return false;
  const holder = floor === "teleop" ? state.teleopFloor : state.voiceFloor;
  return holder.clientId === myClientId;
}

/**
 * Подпись владельца floor-а для HUD. `myClientId=null` (например, наш
 * сервер v1 не выдал client_id) трактуется как «мы не знаем, кто мы» —
 * UI покажет `?`, а не «свободен» / «чужой» (ADR-0018).
 */
export type FloorLabel = "my" | "other" | "free" | "unknown";

export function floorLabel(
  state: SupervisorState,
  floor: FloorName,
  myClientId: string | null
): FloorLabel {
  const holder = floor === "teleop" ? state.teleopFloor : state.voiceFloor;
  if (holder.clientId === null) return "free";
  if (myClientId === null) return "unknown";
  return holder.clientId === myClientId ? "my" : "other";
}
