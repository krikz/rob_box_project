// supervisor_panel (R14, ADR-0027 R14 + ADR-0028 §4): pure-логика и
// формат-парсер. Только чистые функции: parseSupervisorState (msgpack),
// hitTest (геометрия без Three.js),
// formatStateLine / formatDialogueToggle (UX-тексты).
//
// Render/Three.js не тестируем — jsdom не даёт WebGL, в остальных тестах
// пакета так же (см. bridge_environment.test.ts, status_hud.test.ts).
// msgpack-encoder свой — без зависимостей; структура совпадает с
// `decodeMsgpackMap` в src/wire/msgpack.ts.

import { describe, it, expect } from "vitest";
import {
  type SupervisorState,
  type AvatarMode,
  type Floor,
  UNKNOWN_STATE,
  computeButtonLayout,
  formatStateLine,
  formatDialogueToggle,
  hitTest,
  humanMode,
  isUnknownState,
  panelGeometry,
  parseSupervisorState,
  shortHolderName,
  floorOccupantLabel,
  SUPERVISOR_PANEL_ANGLE_DEG,
  SUPERVISOR_PANEL_RADIUS_M
} from "../src/scene/supervisor_panel";

/**
 * Минимальный msgpack-encoder для тестов: строки, числа, null, bool, map.
 * Соответствует тому, что умеет `decodeMsgpackMap` в src/wire/msgpack.ts.
 */
function pack(value: unknown): Uint8Array {
  const out: number[] = [];
  const encode = (v: unknown): void => {
    if (v === null) {
      out.push(0xc0);
      return;
    }
    if (typeof v === "boolean") {
      out.push(v ? 0xc3 : 0xc2);
      return;
    }
    if (typeof v === "number") {
      if (Number.isInteger(v) && v >= 0 && v <= 0x7f) {
        out.push(v & 0x7f);
        return;
      }
      if (Number.isInteger(v) && v >= -32 && v < 0) {
        out.push(0xe0 | (v & 0x1f));
        return;
      }
      if (Number.isInteger(v) && v >= 0 && v <= 0xffff) {
        out.push(0xcd, (v >>> 8) & 0xff, v & 0xff);
        return;
      }
      if (Number.isInteger(v) && v >= 0 && v <= 0xffffffff) {
        out.push(0xce, (v >>> 24) & 0xff, (v >>> 16) & 0xff, (v >>> 8) & 0xff, v & 0xff);
        return;
      }
      if (Number.isInteger(v) && v >= 0) {
        // Большие числа в тестах нам не нужны.
        throw new Error(`test pack: uint too large (${v})`);
      }
      throw new Error(`test pack: negative integer (${v})`);
    }
    if (typeof v === "string") {
      const bytes = new TextEncoder().encode(v);
      if (bytes.length <= 31) out.push(0xa0 | bytes.length);
      else if (bytes.length <= 0xffff) {
        out.push(0xda, (bytes.length >>> 8) & 0xff, bytes.length & 0xff);
      } else {
        throw new Error("test pack: string too long");
      }
      for (const b of bytes) out.push(b);
      return;
    }
    if (Array.isArray(v)) {
      if (v.length <= 15) out.push(0x90 | v.length);
      else out.push(0xdc, (v.length >>> 8) & 0xff, v.length & 0xff);
      for (const e of v) encode(e);
      return;
    }
    if (typeof v === "object") {
      const entries = Object.entries(v as Record<string, unknown>);
      if (entries.length <= 15) out.push(0x80 | entries.length);
      else throw new Error("test pack: map too large");
      for (const [k, val] of entries) {
        const kb = new TextEncoder().encode(k);
        if (kb.length <= 31) out.push(0xa0 | kb.length);
        else throw new Error("test pack: key too long");
        for (const b of kb) out.push(b);
        encode(val);
      }
      return;
    }
    throw new Error(`test pack: unsupported value ${typeof v}`);
  };
  encode(value);
  return new Uint8Array(out);
}

interface FloorSpec {
  held_by: { client_id: string; since_ms: number } | null;
  last_event: Floor["last_event"];
}

function buildFloor(spec: Partial<FloorSpec> = {}): Floor {
  return {
    held_by: spec.held_by ?? null,
    last_event: spec.last_event ?? null
  };
}

function buildState(over: {
  mode?: AvatarMode;
  teleop?: FloorSpec;
  voice?: FloorSpec;
  last_event?: string;
  last_event_ts_ms?: number;
}): SupervisorState {
  return {
    mode: over.mode ?? "off",
    teleop_floor: buildFloor(over.teleop),
    voice_floor: buildFloor(over.voice),
    last_event: over.last_event ?? "STATE_UPDATE",
    last_event_ts_ms: over.last_event_ts_ms ?? 1_700_000_000_000
  };
}

// ───────────────────────────── parseSupervisorState ─────────────────────────────

describe("parseSupervisorState — well-formed msgpack", () => {
  // Реалистичный диапазон timestamp: msgpack webxr_client поддерживает
  // int32 (0xd2 через 0xce→uint32) — выберем значение < 2^32.
  const TS = 1_700_000_000;

  it("decodes a complete state payload with held teleop", () => {
    const payload = pack({
      mode: "avatar_present",
      teleop: { held_by: { client_id: "quest:abc123", since_ms: TS }, last_event: "ACQUIRED" },
      voice: { held_by: null, last_event: "RELEASED" },
      last_event: "ACQUIRED",
      last_event_ts_ms: TS
    });
    const s = parseSupervisorState(payload);
    expect(s).not.toBeNull();
    expect(s!.mode).toBe("avatar_present");
    expect(s!.teleop_floor.held_by?.client_id).toBe("quest:abc123");
    expect(s!.teleop_floor.last_event).toBe("ACQUIRED");
    expect(s!.voice_floor.held_by).toBeNull();
    expect(s!.voice_floor.last_event).toBe("RELEASED");
    expect(s!.last_event).toBe("ACQUIRED");
    expect(s!.last_event_ts_ms).toBe(TS);
  });

  it("returns null on a broken payload", () => {
    // 0xcd 0x30 — стартовый байт, после которого нечего парсить.
    const s = parseSupervisorState(new Uint8Array([0xcd, 0x30, 0x00, 0x00]));
    expect(s).toBeNull();
  });

  it("substitutes sentinels for missing fields", () => {
    // mixed — реальный режим core.fsm.Mode. Раньше здесь стоял
    // voice_only: такого режима у автомата нет, и теперь он honestly
    // деградирует в off (см. тест ниже про неизвестный режим).
    const payload = pack({ mode: "mixed" });
    const s = parseSupervisorState(payload);
    expect(s).not.toBeNull();
    expect(s!.mode).toBe("mixed");
    expect(s!.teleop_floor.held_by).toBeNull();
    expect(s!.voice_floor.last_event).toBeNull();
    expect(s!.last_event).toBe("STATE_UPDATE");
    expect(typeof s!.last_event_ts_ms).toBe("number");
  });

  it("falls back to `off` for an unknown mode", () => {
    const payload = pack({ mode: "polymorphism_deprecated" });
    const s = parseSupervisorState(payload);
    expect(s!.mode).toBe("off");
  });

  it("recovers from a payload where mode key is absent", () => {
    const payload = pack({
      teleop: { held_by: null, last_event: null },
      voice: { held_by: null, last_event: null }
    });
    const s = parseSupervisorState(payload);
    expect(s!.mode).toBe("off");
  });
});

// ───────────────────────────── panelGeometry ─────────────────────────────

describe("panelGeometry", () => {
  it("places the panel at the design-specified angle by default", () => {
    const g = panelGeometry();
    const a = (SUPERVISOR_PANEL_ANGLE_DEG * Math.PI) / 180;
    expect(g.position.x).toBeCloseTo(SUPERVISOR_PANEL_RADIUS_M * Math.sin(a), 5);
    expect(g.position.z).toBeCloseTo(-SUPERVISOR_PANEL_RADIUS_M * Math.cos(a), 5);
  });

  it("places the panel on the left flank (negative x)", () => {
    const g = panelGeometry();
    expect(g.position.x).toBeLessThan(-2);
  });

  it("does not collide with camera_oak_depth at -75°", () => {
    // Конструкция: при −105° панель оказывается симметричной x=-2.32 и
    // z=+0.62 относительно camera_oak_depth (−75°, x=-1.93, z=-0.52).
    // Симметрия sin(−105°) = sin(−75°) даёт совпадение x — это by design,
    // авторы карточки выбрали −105° именно для того, чтобы она ушла
    // назад по z и не «светила» в зону прямого обзора оператора, где
    // уже стоит экран-стена и боковая видео-панель. Проверяем, что
    // позиции различаются по z (|Δz| ≥ 1 м) и не сливаются.
    const g = panelGeometry();
    const oakAngle = -75;
    const oak = panelGeometry(oakAngle, SUPERVISOR_PANEL_RADIUS_M, g.position.y);
    // sanity: углы действительно разные (Δz ≥ 1 м)
    expect(Math.abs(g.position.z - oak.position.z)).toBeGreaterThanOrEqual(1.0);
    // sanity: горизонтальный радиус одинаковый — формула одна
    expect(Math.hypot(g.position.x, g.position.z)).toBeCloseTo(SUPERVISOR_PANEL_RADIUS_M, 5);
    expect(Math.hypot(oak.position.x, oak.position.z)).toBeCloseTo(SUPERVISOR_PANEL_RADIUS_M, 5);
  });

  it("does not face into a wall (normal направлен к оператору)", () => {
    const g = panelGeometry();
    const dot = g.facing.x * g.position.x + g.facing.z * g.position.z;
    expect(dot).toBeLessThan(-0.5);
  });
});

// ───────────────────────────── hitTest ─────────────────────────────

describe("hitTest (pure hit-rect detector)", () => {
  const W = 512;
  const H = 640;
  const layout = computeButtonLayout(W, H);

  it("hits every mode button at its centre", () => {
    const modes = ["mode:avatar_present", "mode:teleop_only", "mode:voice_only", "mode:mixed", "mode:off"];
    for (const m of modes) {
      const buttonId = m.replace("mode:", "");
      const idx = ["avatar_present", "teleop_only", "voice_only", "mixed", "off"].indexOf(buttonId);
      const r = layout.modeButtons[idx].rect;
      const cx = (r.x + r.w / 2) / W;
      const cy = (r.y + r.h / 2) / H;
      expect(hitTest(cx, cy, layout, W, H)).toBe(m);
    }
  });

  it("hits the teleop acquire button at its centre", () => {
    const r = layout.floors.teleop.acquire.rect;
    expect(hitTest((r.x + r.w / 2) / W, (r.y + r.h / 2) / H, layout, W, H)).toBe(
      "floor:teleop:acquire"
    );
  });

  it("hits the voice release button at its centre", () => {
    const r = layout.floors.voice.release.rect;
    expect(hitTest((r.x + r.w / 2) / W, (r.y + r.h / 2) / H, layout, W, H)).toBe(
      "floor:voice:release"
    );
  });

  it("hits the dialogue toggle at its centre", () => {
    const r = layout.dialogueToggle.rect;
    expect(hitTest((r.x + r.w / 2) / W, (r.y + r.h / 2) / H, layout, W, H)).toBe(
      "dialogue:toggle"
    );
  });

  it("returns null just above the top-left corner (outside everything)", () => {
    expect(hitTest(0.001, 0.001, layout, W, H)).toBeNull();
  });

  it("treats right/bottom edges as outside (no NaN-trap on seams)", () => {
    const last = layout.modeButtons[layout.modeButtons.length - 1].rect;
    const xRight = (last.x + last.w) / W;
    const yMid = (last.y + last.h / 2) / H;
    expect(hitTest(xRight, yMid, layout, W, H)).toBeNull();
  });

  it("ignores non-finite coordinates", () => {
    expect(hitTest(NaN, 0.5, layout, W, H)).toBeNull();
    expect(hitTest(0.5, Infinity, layout, W, H)).toBeNull();
    expect(hitTest(-0.1, 0.5, layout, W, H)).toBeNull();
  });

  it("ignores a zero-sized canvas", () => {
    expect(hitTest(0.5, 0.5, layout, 0, 0)).toBeNull();
  });
});

// ───────────────────────────── format texts ─────────────────────────────

describe("formatStateLine", () => {
  it("shows `неизвестно` before the first STATE_UPDATE", () => {
    expect(formatStateLine(UNKNOWN_STATE, "quest:abc")).toContain("неизвестно");
  });

  it("lists mode + teleop + voice occupancy after STATE_UPDATE", () => {
    const s = buildState({
      mode: "avatar_present",
      teleop: { held_by: { client_id: "quest:abc", since_ms: 1 }, last_event: "ACQUIRED" },
      voice: { held_by: { client_id: "tg:42", since_ms: 2 }, last_event: "ACQUIRED" }
    });
    const line = formatStateLine(s, "quest:abc");
    expect(line).toContain("вы — оператор");
    expect(line).toContain("у вас");
    expect(line).toContain("tg:42");
  });
});

describe("formatDialogueToggle — honesty about voice_off semantics", () => {
  it("shows a generic placeholder before the first STATE_UPDATE", () => {
    const t = formatDialogueToggle(UNKNOWN_STATE, false);
    expect(t.label).toContain("…");
    expect(t.hint.toLowerCase()).toContain("неизвест");
  });

  it("ON-label is honest when voice is on", () => {
    const t = formatDialogueToggle(buildState({ mode: "avatar_present" }), false);
    expect(t.label).toContain("ВКЛ");
    expect(t.hint).toMatch(/микрофон/);
  });

  it("OFF-label is honest and explains which source it actually mutes", () => {
    // Сценарий: оператор жмёт «Диалог робота: ВЫКЛ» на панели — внутри
    // dialogue_node ставится voice_input_mode=off, и это глушит ТОЛЬКО
    // ReSpeaker (люди рядом), а не Quest-микрофон. Без честной подписи
    // оператор будет думать, что всё молчит.
    const t = formatDialogueToggle(buildState({ mode: "avatar_present" }), true);
    expect(t.label).toContain("ВЫКЛ");
    expect(t.hint).toMatch(/ReSpeaker|рядом/);
    expect(t.hint).toMatch(/очков|Quest|микрофон/);
  });
});

describe("humanMode / shortHolderName / floorOccupantLabel", () => {
  it("translates known modes", () => {
    expect(humanMode("off")).toBe("аватар выкл");
    expect(humanMode("avatar_present")).toBe("вы — оператор");
    expect(humanMode("mixed")).toBe("смешанный");
  });

  it("returns the original id when it fits in 8 chars", () => {
    expect(shortHolderName("tg:42")).toBe("tg:42");
    expect(shortHolderName("a")).toBe("a");
  });

  it("truncates ids longer than 8 chars to 6 chars + ellipsis", () => {
    expect(shortHolderName("quest:supersession")).toBe("quest:…");
    expect(shortHolderName("abcdefghijk")).toBe("abcdef…");
  });

  it("labels `свободно` for an empty floor", () => {
    expect(floorOccupantLabel({ held_by: null, last_event: null }, "quest:me")).toBe(
      "свободно"
    );
  });

  it("labels `у вас` for own floor, `у <id>` for someone else", () => {
    const mine = {
      held_by: { client_id: "quest:me", since_ms: 1 },
      last_event: "ACQUIRED" as const
    };
    const theirs = {
      held_by: { client_id: "tg:42", since_ms: 1 },
      last_event: "ACQUIRED" as const
    };
    expect(floorOccupantLabel(mine, "quest:me")).toBe("у вас");
    expect(floorOccupantLabel(theirs, "quest:me")).toBe("у tg:42");
  });
});

describe("isUnknownState", () => {
  it("identifies the initial sentinel state", () => {
    expect(isUnknownState(UNKNOWN_STATE)).toBe(true);
  });
  it("rejects any real state", () => {
    expect(isUnknownState(buildState({ mode: "off" }))).toBe(false);
  });
});
