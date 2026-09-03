// FSM для teleop-ввода. Чистая логика (без Three.js / XR / клавиатуры),
// тестируется в изоляции. API:
//
//   const fsm = new TeleopFSM();
//   fsm.setLinear(0.5);   // -1..1
//   fsm.setAngular(-0.3); // -1..1
//   fsm.setDeadman(true);
//   fsm.setHasFloor(true); // AV-19: gate по teleop_floor
//   const out = fsm.tick(now_ms);  // возвращает null или TeleopTwistCmd
//   fsm.heartbeatCmd(now_ms);       // AV-19: вернуть teleop_heartbeat cmd
//
// Правила (дизайн §6 + api.md §5):
// - tick() вызывается каждый animation frame.
// - send() возвращает cmd только если throttle прошёл (30 Hz).
// - Если deadman=false И нет активного hold'а, шлём "stop" (twist=0) сразу,
//   даже если линейная/угловая скорости ≠ 0 (страховка от застрявшего пакета).
// - При первом deadman=false после hold → один "stop" фрейм, затем тишина.
// - На отпускании grip (XR) или Space (desktop) следующий фрейм
//   deadman=false, через 100 мс — stop (twist=0).
//
// AV-19 (issue #1911, ADR-0028 §4.4): добавлен gate teleop_floor.
// - Если !hasFloor → state="armed_no_floor"; tick() возвращает null
//   (twist не шлётся); heartbeat НЕ шлётся (источник живости — клиент).
// - На setHasFloor(true) → переход в "armed" если deadman=true.
// - На setHasFloor(false) из "armed" → принудительный DISARM.
// - heartbeat cmd возвращается, только если state in {"armed"}.
//
// MAX_LINEAR = 0.5 м/с, MAX_ANGULAR = 1.0 рад/с (дизайн §6).

import type { TeleopTwistCmd, TeleopHeartbeatCmd, StopEmergencyCmd } from "../wire/messages";

export const MAX_LINEAR = 0.5;
export const MAX_ANGULAR = 1.0;
export const THROTTLE_HZ = 30;
export const THROTTLE_INTERVAL_MS = 1000 / THROTTLE_HZ; // 33.33...
export const STOP_DELAY_MS = 100;
// AV-19: heartbeat rate (ADR-0028 §4.4 S10) — 10 Hz пока ARM+floor.
export const HEARTBEAT_HZ = 10;
export const HEARTBEAT_INTERVAL_MS = 1000 / HEARTBEAT_HZ; // 100 ms

export type TeleopState =
  | "idle" // deadman выключен
  | "armed" // deadman включён + floor наш — шлём twist + heartbeat
  | "armed_no_floor" // deadman включён, НО floor чужой — twist не шлём (gate)
  | "stopping"; // только что отпустили deadman, шлём один stop

export interface TeleopInput {
  linear: number; // -1..1
  angular: number; // -1..1
  deadman: boolean;
}

export interface TeleopTickResult {
  cmd: TeleopTwistCmd | StopEmergencyCmd;
  type: "twist" | "stop";
}

export class TeleopFSM {
  private state: TeleopState = "idle";
  private linear = 0;
  private angular = 0;
  private deadman = false;
  private _hasFloor = true; // AV-19: optimistic default — пока не получили
  // floor_lost JSON_EVENT от сервера, считаем, что floor наш. Сервер
  // ответит FLOOR_HELD / floor_lost очень быстро (< 1 с), и FSM
  // переключится в armed_no_floor. На HELLO сервер передаёт
  // teleop_floor_held_by (см. meta-quest-api.md §3), и caller должен
  // вызвать setHasFloor(<== == session.session_id>) сразу после WELCOME.
  private seq = 0;
  private heartbeatSeq = 0;
  private lastSendTsMs = 0;
  private lastHeartbeatTsMs = 0;
  private stopAtMs = 0; // когда переключились из active в stopping

  setLinear(v: number): void {
    this.linear = clamp(v, -1, 1);
  }

  setAngular(v: number): void {
    this.angular = clamp(v, -1, 1);
  }

  /**
   * AV-19: уведомить FSM о текущем состоянии teleop_floor. Вызывается
   * из main.ts на WELCOME (teleop_floor_held_by == session.session_id)
   * и на JSON_EVENT{type:"floor_lost"} (hasFloor=false).
   */
  setHasFloor(hasFloor: boolean): void {
    if (hasFloor === this._hasFloor) return;
    this._hasFloor = hasFloor;
    if (!hasFloor) {
      // Потеряли floor — мгновенный DISARM (без waiting следующего кадра).
      this.forceDisarm("floor_lost");
    }
    // Получили floor — если deadman=true, переходим в armed.
    if (hasFloor && this.deadman) {
      this.state = "armed";
    }
  }

  hasFloor(): boolean {
    return this._hasFloor;
  }

  /**
   * AV-19: жёсткий DISARM без отправки stop-фрейма. Используется при
   * потере floor (setHasFloor) и при reset() — ниже, чем setDeadman(false).
   */
  forceDisarm(reason: string): void {
    if (this.state === "idle") return;
    this.state = "idle";
    this.deadman = false;
    this.lastSendTsMs = 0;
    this.lastHeartbeatTsMs = 0;
    this.stopAtMs = 0;
    // Caller (main.ts) логирует reason через console — здесь намеренно
    // не делаем console.log, чтобы FSM оставался чисто-логическим слоем.
    void reason;
  }

  setDeadman(v: boolean): void {
    if (v === this.deadman) return;
    const prev = this.deadman;
    this.deadman = v;
    if (prev && !v) {
      // Только что отпустили — отправим stop через STOP_DELAY_MS.
      // Важно: даже если мы в armed_no_floor, при отпускании deadman
      // переходим в stopping→idle (не «застреваем» в armed_no_floor).
      this.state = "stopping";
      this.stopAtMs = 0; // отложим до tick
    } else if (!prev && v) {
      // Только что нажали — переходим в armed, но только если есть floor.
      this.state = this._hasFloor ? "armed" : "armed_no_floor";
    }
  }

  /** Немедленный emergency-stop (B-кнопка или UI). */
  triggerEmergency(source: StopEmergencyCmd["source"]): StopEmergencyCmd {
    // AV-19: emergency работает ВСЕГДА, в обход гейта (ADR-0028 §4.4).
    // FSM-смена состояния ниже — чтобы tick() не слал лишнее.
    this.state = "idle";
    this.deadman = false;
    return {
      cmd: "stop_emergency",
      ts_ms: Date.now(),
      source
    };
  }

  /** Текущее состояние для UI / тестов. */
  getState(): TeleopState {
    return this.state;
  }

  getSeq(): number {
    return this.seq;
  }

  /**
   * Вернуть teleop_heartbeat cmd для отправки на сервер (AV-19).
   * Throttle — HEARTBEAT_INTERVAL_MS (10 Гц). Возвращает null если
   * throttle не прошёл или FSM не в active-состоянии (idle/armed_no_floor
   * / stopping).
   */
  heartbeatCmd(nowMs: number): TeleopHeartbeatCmd | null {
    if (this.state !== "armed") return null;
    if (nowMs - this.lastHeartbeatTsMs < HEARTBEAT_INTERVAL_MS) return null;
    this.heartbeatSeq += 1;
    this.lastHeartbeatTsMs = nowMs;
    return {
      cmd: "teleop_heartbeat",
      ts_ms: nowMs,
      seq: this.heartbeatSeq
    };
  }

  /**
   * Возвращает команду для отправки или null (throttle не прошёл, нечего слать).
   * @param nowMs текущее время в мс (Date.now() или performance.now).
   * @param forceSend опционально пропустить throttle (используется в тестах).
   */
  tick(nowMs: number, forceSend = false): TeleopTickResult | null {
    // AV-19: если нет floor — twist НЕ шлём (gate). Heartbeat тоже не
    // нужен — клиент не владеет floor (см. heartbeatCmd).
    if (this.state === "idle" || this.state === "armed_no_floor") return null;
    if (!forceSend && nowMs - this.lastSendTsMs < THROTTLE_INTERVAL_MS) return null;

    if (this.state === "stopping") {
      // Первый тик после release — пошлём twist=0 с deadman=false.
      // Если прошло достаточно времени после release (≥ STOP_DELAY_MS),
      // считаем что stop отправлен и переходим в idle.
      if (this.stopAtMs === 0) {
        this.stopAtMs = nowMs + STOP_DELAY_MS;
        return this.emitStop(nowMs);
      }
      if (nowMs >= this.stopAtMs) {
        this.state = "idle";
        return null;
      }
      // Пока ждём — не шлём больше.
      return null;
    }

    // armed
    return this.emitTwist(nowMs);
  }

  private emitTwist(nowMs: number): TeleopTickResult {
    this.seq += 1;
    this.lastSendTsMs = nowMs;
    const cmd: TeleopTwistCmd = {
      cmd: "teleop_twist",
      ts_ms: nowMs,
      seq: this.seq,
      linear: { x: this.linear * MAX_LINEAR, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: this.angular * MAX_ANGULAR },
      deadman: this.deadman
    };
    return { cmd, type: "twist" };
  }

  private emitStop(nowMs: number): TeleopTickResult {
    this.seq += 1;
    this.lastSendTsMs = nowMs;
    const cmd: TeleopTwistCmd = {
      cmd: "teleop_twist",
      ts_ms: nowMs,
      seq: this.seq,
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
      deadman: false
    };
    return { cmd, type: "stop" };
  }

  /** Сбросить всё (при потере соединения, например). */
  reset(): void {
    this.state = "idle";
    this.linear = 0;
    this.angular = 0;
    this.deadman = false;
    this._hasFloor = true; // оптимистично — reconnect → WELCOME обновит.
    this.stopAtMs = 0;
    this.lastSendTsMs = 0;
    this.lastHeartbeatTsMs = 0;
  }
}

function clamp(v: number, lo: number, hi: number): number {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}