// Desktop-телеоп (дизайн §6): стрелки + Space (deadman) + E (emergency).
// Активируется автоматически, если XR недоступен.
//
// WASD раньше тоже вёл робота, но на десктопе WASD теперь ходит оператором
// по мостику (input/desktop_walk.ts, #3149). Робота ведут стрелки И ТОЛЬКО
// при зажатом Space: без deadman FSM twist не шлёт, так что случайная
// стрелка робота не сдвинет.

import type { TeleopFSM } from "./teleop_fsm";

export interface DesktopTeleopHandle {
  destroy(): void;
  isActive(): boolean;
}

export interface DesktopTeleopOptions {
  fsm: TeleopFSM;
  element?: HTMLElement | Window;
}

const KEY_BINDINGS: Record<string, "linear+" | "linear-" | "angular+" | "angular-" | "deadman" | "emergency"> = {
  ArrowUp: "linear+",
  ArrowDown: "linear-",
  ArrowLeft: "angular+",
  ArrowRight: "angular-",
  Space: "deadman",
  KeyE: "emergency"
};

/** Клавиши, которые перехватывает телеоп (экспорт — для конфликт-теста с ходьбой). */
export const DESKTOP_TELEOP_KEYS: ReadonlyArray<string> = Object.freeze(Object.keys(KEY_BINDINGS));

export function createDesktopTeleop(opts: DesktopTeleopOptions): DesktopTeleopHandle {
  const fsm = opts.fsm;
  const target: HTMLElement | Window = opts.element ?? window;
  const pressed = new Set<string>();

  function onKeyDown(ev: KeyboardEvent): void {
    // Не перехватываем ввод в PIN-форме и текстовых полях.
    const tgt = ev.target as HTMLElement | null;
    if (
      tgt &&
      (tgt.tagName === "INPUT" ||
        tgt.tagName === "TEXTAREA" ||
        tgt.tagName === "SELECT" ||
        (tgt as HTMLElement).isContentEditable)
    ) {
      return;
    }
    const binding = KEY_BINDINGS[ev.code];
    if (!binding) return;
    ev.preventDefault();
    if (binding === "emergency") {
      // emergency — edge-triggered: шлём сразу при нажатии.
      // Реальную отправку делает caller (TeleopController), здесь мы
      // просто помечаем — он увидит, что в `pressed` есть 'KeyE'.
      pressed.add(ev.code);
      return;
    }
    pressed.add(ev.code);
    refresh();
  }

  function onKeyUp(ev: KeyboardEvent): void {
    const binding = KEY_BINDINGS[ev.code];
    if (!binding) return;
    pressed.delete(ev.code);
    refresh();
  }

  function refresh(): void {
    const linear = (pressed.has("ArrowUp") ? 1 : 0) + (pressed.has("ArrowDown") ? -1 : 0);
    const angular = (pressed.has("ArrowLeft") ? 1 : 0) + (pressed.has("ArrowRight") ? -1 : 0);
    const deadman = pressed.has("Space");
    fsm.setLinear(linear);
    fsm.setAngular(angular);
    fsm.setDeadman(deadman);
  }

  function consumeEmergency(): boolean {
    if (pressed.has("KeyE")) {
      pressed.delete("KeyE");
      return true;
    }
    return false;
  }

  target.addEventListener("keydown", onKeyDown as EventListener);
  target.addEventListener("keyup", onKeyUp as EventListener);
  // expose для интеграции
  (target as unknown as { __questDesktopTeleop?: unknown }).__questDesktopTeleop = { consumeEmergency };

  return {
    destroy(): void {
      target.removeEventListener("keydown", onKeyDown as EventListener);
      target.removeEventListener("keyup", onKeyUp as EventListener);
      pressed.clear();
    },
    isActive(): boolean {
      return pressed.size > 0;
    }
  };
}