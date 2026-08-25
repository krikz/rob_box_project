// FSM для teleop-ввода. Чистая логика (без Three.js / XR / клавиатуры),
// тестируется в изоляции. API:
//
//   const fsm = new TeleopFSM();
//   fsm.setLinear(0.5);   // -1..1
//   fsm.setAngular(-0.3); // -1..1
//   fsm.setDeadman(true);
//   const out = fsm.tick(now_ms);  // возвращает null или TeleopTwistCmd
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
// MAX_LINEAR = 0.5 м/с, MAX_ANGULAR = 1.0 рад/с (дизайн §6).

import type { TeleopTwistCmd, StopEmergencyCmd } from "../wire/messages";

export const MAX_LINEAR = 0.5;
export const MAX_ANGULAR = 1.0;
export const THROTTLE_HZ = 30;
export const THROTTLE_INTERVAL_MS = 1000 / THROTTLE_HZ; // 33.33...
export const STOP_DELAY_MS = 100;

export type TeleopState =
  | "idle" // deadman выключен
  | "active" // deadman включён, шлём twist
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
  private seq = 0;
  private lastSendTsMs = 0;
  private stopAtMs = 0; // когда переключились из active в stopping

  setLinear(v: number): void {
    this.linear = clamp(v, -1, 1);
  }

  setAngular(v: number): void {
    this.angular = clamp(v, -1, 1);
  }

  setDeadman(v: boolean): void {
    if (v === this.deadman) return;
    const prev = this.deadman;
    this.deadman = v;
    if (prev && !v) {
      // Только что отпустили — отправим stop через STOP_DELAY_MS.
      this.state = "stopping";
      this.stopAtMs = 0; // отложим до tick
    } else if (!prev && v) {
      // Только что нажали — переходим в active, но tick сам пошлёт.
      this.state = "active";
    }
  }

  /** Немедленный emergency-stop (B-кнопка или UI). */
  triggerEmergency(source: StopEmergencyCmd["source"]): StopEmergencyCmd {
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
   * Возвращает команду для отправки или null (throttle не прошёл, нечего слать).
   * @param nowMs текущее время в мс (Date.now() или performance.now).
   * @param forceSend опционально пропустить throttle (используется в тестах).
   */
  tick(nowMs: number, forceSend = false): TeleopTickResult | null {
    // Не шлём ничего, пока connected не подтверждён (это решает caller).
    if (this.state === "idle") return null;
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

    // active
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
    this.stopAtMs = 0;
  }
}

function clamp(v: number, lo: number, hi: number): number {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}