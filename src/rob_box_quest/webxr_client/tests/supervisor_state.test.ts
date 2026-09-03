// AV-17: чистая логика supervisor-state (src/state/supervisor_state.ts).
//
// Wire-формат берём из сервера (`rob_box_supervisor/core/state.py::pack`):
//   {mode, teleop_floor, voice_floor, last_event, since_ms, version}
// где *_floor = null | {client_id, since_ms, last_heartbeat_ms}.

import { describe, it, expect } from "vitest";
import {
  parseSupervisorState,
  floorLabel,
  floorIsFree,
  floorIsMine,
  supervisorEffect
} from "../src/state/supervisor_state";

const flat = (over: Record<string, unknown> = {}) => ({
  mode: "avatar_present",
  teleop_floor: { client_id: "quest-1", since_ms: 1000, last_heartbeat_ms: 1500 },
  voice_floor: null,
  last_event: null,
  since_ms: 900,
  version: 1,
  ...over
});

describe("parseSupervisorState", () => {
  it("парсит плоский серверный формат (teleop_floor / voice_floor)", () => {
    const st = parseSupervisorState(flat());
    expect(st).not.toBeNull();
    expect(st!.mode).toBe("avatar_present");
    expect(st!.teleopFloor).toEqual({ clientId: "quest-1", sinceMs: 1000 });
    expect(st!.voiceFloor).toEqual({ clientId: null, sinceMs: 0 });
    expect(st!.updatedMs).toBe(900);
  });

  it("принимает вложенный вариант floors:{teleop,voice} (forward-compat)", () => {
    const st = parseSupervisorState({
      mode: "mixed",
      floors: {
        teleop: { client_id: "quest-1", since_ms: 5 },
        voice: { client_id: "tg-42", since_ms: 7 }
      },
      updated_ms: 11
    });
    expect(st).not.toBeNull();
    expect(st!.teleopFloor.clientId).toBe("quest-1");
    expect(st!.voiceFloor.clientId).toBe("tg-42");
    expect(st!.updatedMs).toBe(11);
  });

  it("map без floor-полей вообще → null (не выдумываем «свободно»)", () => {
    expect(parseSupervisorState({ mode: "off", since_ms: 1 })).toBeNull();
  });

  it("mode не строка → null", () => {
    expect(parseSupervisorState({ mode: 7, teleop_floor: null, voice_floor: null })).toBeNull();
  });

  it("не-map payload → null", () => {
    expect(parseSupervisorState(null)).toBeNull();
    expect(parseSupervisorState(42)).toBeNull();
    expect(parseSupervisorState([1, 2, 3])).toBeNull();
    expect(parseSupervisorState("mode=off")).toBeNull();
  });

  it("битый floor (строка вместо map) → floor трактуется как свободный, state валиден", () => {
    const st = parseSupervisorState(flat({ teleop_floor: "quest-1" }));
    expect(st).not.toBeNull();
    expect(st!.teleopFloor).toEqual({ clientId: null, sinceMs: 0 });
  });

  it("отсутствующий since_ms/updated_ms → 0, а не NaN", () => {
    const st = parseSupervisorState({ mode: "off", teleop_floor: null, voice_floor: null });
    expect(st!.updatedMs).toBe(0);
  });

  it("неизвестный серверный режим не ломает парсинг (forward-compat)", () => {
    const st = parseSupervisorState(flat({ mode: "future_mode_x" }));
    expect(st!.mode).toBe("future_mode_x");
  });
});

describe("floorLabel / floorIsMine / floorIsFree", () => {
  const st = parseSupervisorState(
    flat({
      teleop_floor: { client_id: "quest-1", since_ms: 1 },
      voice_floor: { client_id: "tg-42", since_ms: 2 }
    })
  )!;

  it("наш floor → my", () => {
    expect(floorLabel(st, "teleop", "quest-1")).toBe("my");
    expect(floorIsMine(st, "teleop", "quest-1")).toBe(true);
  });

  it("чужой floor → other", () => {
    expect(floorLabel(st, "voice", "quest-1")).toBe("other");
    expect(floorIsMine(st, "voice", "quest-1")).toBe(false);
  });

  it("свободный floor → free", () => {
    const free = parseSupervisorState(flat({ teleop_floor: null }))!;
    expect(floorLabel(free, "teleop", "quest-1")).toBe("free");
    expect(floorIsFree(free, "teleop")).toBe(true);
  });

  it("myClientId неизвестен, floor занят → unknown, НЕ free и НЕ other", () => {
    expect(floorLabel(st, "teleop", null)).toBe("unknown");
    expect(floorIsMine(st, "teleop", null)).toBe(false);
  });
});

describe("supervisorEffect — реакция UI на потерю teleop-floor", () => {
  // Сборка типового «наш → забрали»-перехода.
  const ourFloor = (): ReturnType<typeof parseSupervisorState> =>
    parseSupervisorState(
      flat({ teleop_floor: { client_id: "quest-1", since_ms: 1 } })
    );
  const otherClientTakes = (): ReturnType<typeof parseSupervisorState> =>
    parseSupervisorState(
      flat({ teleop_floor: { client_id: "telegram-42", since_ms: 2 } })
    );
  const freeFloor = (): ReturnType<typeof parseSupervisorState> =>
    parseSupervisorState(flat({ teleop_floor: null }));

  it("my → other (забрал Telegram-бот) → DISARM + тост с именем держателя", () => {
    const prev = ourFloor()!;
    expect(floorLabel(prev, "teleop", "quest-1")).toBe("my");
    const eff = supervisorEffect("my", otherClientTakes(), "quest-1");
    expect(eff.disarm).toBe(true);
    expect(eff.teleopLabel).toBe("other");
    expect(eff.toast).toMatch(/^Руль забрал другой клиент \(telegram…\)$/);
  });

  it("my → free (свободен, оператор отпустил) → DISARM + generic тост", () => {
    const eff = supervisorEffect("my", freeFloor(), "quest-1");
    expect(eff.disarm).toBe(true);
    expect(eff.teleopLabel).toBe("free");
    expect(eff.toast).toBe("Руль снят супервизором");
  });

  it("my → null (reconnect, STATE_UPDATE ещё не пришёл) → DISARM, метка unknown", () => {
    const eff = supervisorEffect("my", null, "quest-1");
    expect(eff.disarm).toBe(true);
    expect(eff.teleopLabel).toBe("unknown");
    // Без нового держателя (next=null) — generic «снят», не «забрал».
    expect(eff.toast).toBe("Руль снят супервизором");
  });

  it("other → my (перехватили назад) → НЕ дисарм", () => {
    const eff = supervisorEffect("other", ourFloor(), "quest-1");
    expect(eff.disarm).toBe(false);
    expect(eff.teleopLabel).toBe("my");
    expect(eff.toast).toBeNull();
  });

  it("free → other (мы и не были armed) → НЕ дисарм", () => {
    const eff = supervisorEffect("free", otherClientTakes(), "quest-1");
    expect(eff.disarm).toBe(false);
    expect(eff.teleopLabel).toBe("other");
    expect(eff.toast).toBeNull();
  });

  it("my → my (стабильно наш) → НЕ дисарм, тоста нет", () => {
    const eff = supervisorEffect("my", ourFloor(), "quest-1");
    expect(eff.disarm).toBe(false);
    expect(eff.teleopLabel).toBe("my");
    expect(eff.toast).toBeNull();
  });

  it("двойной DISARM не срабатывает (после первого перехода prev становится other)", () => {
    // Сценарий: бот забрал → отпустил → супервизор ещё не прислал ничего →
    // мы опять на my. Второй эффект на «my → my» уже ничего не делает.
    const afterTakeover = supervisorEffect("my", otherClientTakes(), "quest-1");
    expect(afterTakeover.disarm).toBe(true);
    expect(afterTakeover.teleopLabel).toBe("other");
    // Аналогично для возврата: «other → my» — не дисарм (не «потеря»).
    const regained = supervisorEffect("other", ourFloor(), "quest-1");
    expect(regained.disarm).toBe(false);
    // И повторный «my → other» — да, опять дисарм (правильно, руль только что забрали).
    const takenAgain = supervisorEffect("my", otherClientTakes(), "quest-1");
    expect(takenAgain.disarm).toBe(true);
  });
});
