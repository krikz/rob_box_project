// Voice State Indicator (AV-20).
//
// Тесты чистой логики — без Three.js и canvas:
//   - parseVoiceState: разбор msgpack-payload (реальный серверный формат,
//     битый payload, отсутствующие поля, неизвестные значения).
//   - formatVoiceStatePresentation: маппинг (state, detail) →
//     {label, color, ariaText, level}. Ключевые инварианты:
//       * detail="denied" перебивает state в красный "MIC DENIED";
//       * detail="silenced" перебивает state в серый "MUTED";
//       * неизвестный state → серый "—";
//       * уровень "bad" только для denied, остальное "ok" / "warn" / "unknown".

import { describe, it, expect } from "vitest";
import {
  parseVoiceState,
  formatVoiceStatePresentation,
  type VoiceState,
  type VoiceStateFrame
} from "../src/ui/voice_state_indicator";

/**
 * Собрать msgpack-payload voice_state для тестов.
 * Минимум полей: state + ts_ms. detail — опционально.
 *
 * msgpack-кодирование вручную (минимальное):
 *   fixmap (≤15):   0x80 | N
 *   fixstr (≤31):   0xa0 | N
 *   fixint 0..127:  0x00..0x7f
 *   str8 (0xd9):    0xd9 + u8 length + bytes
 *   map16 (0xde):   0xde + u16 length + N*entries
 */
function packMap(entries: Array<[string, unknown]>): Uint8Array {
  const parts: number[] = [];
  const N = entries.length;
  if (N <= 15) {
    parts.push(0x80 | N);
  } else {
    parts.push(0xde, (N >> 8) & 0xff, N & 0xff);
  }
  // big-endian для u64/u32.
  const u64 = (v: number) => {
    // bigint для больших int64 (>2^32).
    const big = BigInt(v);
    for (let i = 7; i >= 0; i--) {
      parts.push(Number((big >> BigInt(i * 8)) & 0xffn));
    }
  };
  for (const [k, v] of entries) {
    // key
    const kBytes = new TextEncoder().encode(k);
    if (kBytes.length <= 31) {
      parts.push(0xa0 | kBytes.length);
    } else if (kBytes.length <= 0xff) {
      parts.push(0xd9, kBytes.length);
    } else {
      throw new Error("test helper: keys >255 bytes not supported");
    }
    for (const b of kBytes) parts.push(b);
    // value
    if (typeof v === "string") {
      const vBytes = new TextEncoder().encode(v);
      if (vBytes.length <= 31) {
        parts.push(0xa0 | vBytes.length);
      } else if (vBytes.length <= 0xff) {
        parts.push(0xd9, vBytes.length);
      } else {
        throw new Error("test helper: string values >255 bytes not supported");
      }
      for (const b of vBytes) parts.push(b);
    } else if (typeof v === "number" && Number.isInteger(v) && v >= 0 && v <= 0x7f) {
      parts.push(v);
    } else if (typeof v === "number" && Number.isInteger(v) && v >= 0 && v <= 0xff) {
      parts.push(0xcc, v);
    } else if (typeof v === "number" && Number.isInteger(v) && v >= 0 && v <= 0xffff) {
      parts.push(0xcd, (v >> 8) & 0xff, v & 0xff);
    } else if (typeof v === "number" && Number.isInteger(v) && v >= 0 && v <= 0xffffffff) {
      parts.push(0xce, (v >>> 24) & 0xff, (v >>> 16) & 0xff, (v >>> 8) & 0xff, v & 0xff);
    } else if (typeof v === "number" && Number.isInteger(v) && v >= 0) {
      // u64 (msgpack 0xcf)
      parts.push(0xcf);
      u64(v);
    } else if (typeof v === "number" && Number.isInteger(v) && v < 0 && v >= -32) {
      parts.push(0xe0 | (v & 0x1f));
    } else if (v === null) {
      parts.push(0xc0);
    } else {
      throw new Error(`test helper: unsupported value ${JSON.stringify(v)}`);
    }
  }
  return new Uint8Array(parts);
}

describe("parseVoiceState", () => {
  it("decodes a real server payload (idle)", () => {
    // Полный payload как шлёт backend: {state:"idle", ts_ms:..., detail:"silenced"}.
    const payload = packMap([
      ["state", "idle"],
      ["ts_ms", 1_700_000_000_123],
      ["detail", "silenced"]
    ]);
    const f = parseVoiceState(payload)!;
    expect(f.state).toBe<VoiceState>("idle");
    expect(f.detail).toBe("silenced");
    expect(f.tsMs).toBe(1_700_000_000_123);
  });

  it("decodes all 4 bridge states", () => {
    for (const state of ["idle", "listening", "thinking", "speaking"] as const) {
      const f = parseVoiceState(packMap([["state", state], ["ts_ms", 0]]))!;
      expect(f.state).toBe(state);
      expect(f.detail).toBe("none");
    }
  });

  it("decodes detail='denied' even with idle state", () => {
    const f = parseVoiceState(
      packMap([["state", "idle"], ["ts_ms", 0], ["detail", "denied"]])
    )!;
    expect(f.state).toBe("idle");
    expect(f.detail).toBe("denied");
  });

  it("uppercase / unknown state → unknown (no crash)", () => {
    const f = parseVoiceState(packMap([["state", "LISTENING"], ["ts_ms", 0]]))!;
    expect(f.state).toBe("unknown");
    expect(f.detail).toBe("none");
  });

  it("non-string state → unknown", () => {
    const f = parseVoiceState(packMap([["state", 42], ["ts_ms", 0]]))!;
    expect(f.state).toBe("unknown");
  });

  it("unknown detail → 'none' (not 'unknown' — мы не хотим красный UI от шума)", () => {
    const f = parseVoiceState(
      packMap([["state", "speaking"], ["ts_ms", 0], ["detail", "banana"]])
    )!;
    expect(f.state).toBe("speaking");
    expect(f.detail).toBe("none");
  });

  it("missing ts_ms → 0 sentinel", () => {
    const f = parseVoiceState(packMap([["state", "idle"]]))!;
    expect(f.tsMs).toBe(0);
  });

  it("non-finite ts_ms → 0 sentinel", () => {
    // Безопасный путь: payload сломан — UI не должен падать.
    // encodeMsgpackMap пробрасывает NaN как есть (наш helper откажется),
    // поэтому просто шлём payload с state и отсутствующим ts_ms —
    // это эквивалент "поле есть, но не число" в нашем парсере.
    const f = parseVoiceState(packMap([["state", "idle"], ["ts_ms", "now"]]))!;
    expect(f.tsMs).toBe(0);
  });

  it("returns null on a broken payload", () => {
    // cd 30 — incomplete uint16 (msgpack: unsupported type after that).
    expect(parseVoiceState(new Uint8Array([0xcd, 0x30]))).toBeNull();
  });

  it("returns null on a non-map payload", () => {
    // fixarray of 2 elements.
    expect(parseVoiceState(new Uint8Array([0x92, 0x01, 0x02]))).toBeNull();
  });
});

describe("formatVoiceStatePresentation", () => {
  const frame = (over: Partial<VoiceStateFrame> = {}): VoiceStateFrame => ({
    state: "idle",
    detail: "none",
    tsMs: 0,
    ...over
  });

  it("idle state → grey IDLE", () => {
    const p = formatVoiceStatePresentation(frame({ state: "idle" }));
    expect(p.label).toBe("IDLE");
    expect(p.color).toBe("#8b98a5");
    expect(p.ariaText).toContain("idle");
    expect(p.level).toBe("ok");
  });

  it("listening state → blue LISTENING", () => {
    const p = formatVoiceStatePresentation(frame({ state: "listening" }));
    expect(p.label).toBe("LISTENING");
    expect(p.color).toBe("#3b8eea");
    expect(p.ariaText).toContain("listening");
  });

  it("thinking state → orange THINKING", () => {
    const p = formatVoiceStatePresentation(frame({ state: "thinking" }));
    expect(p.label).toBe("THINKING");
    expect(p.color).toBe("#f5a623");
    expect(p.ariaText).toContain("thinking");
  });

  it("speaking state → green SPEAKING", () => {
    const p = formatVoiceStatePresentation(frame({ state: "speaking" }));
    expect(p.label).toBe("SPEAKING");
    expect(p.color).toBe("#2ec27e");
    expect(p.ariaText).toContain("speaking");
  });

  it("unknown state → grey dash, level=unknown", () => {
    const p = formatVoiceStatePresentation(frame({ state: "unknown" }));
    expect(p.label).toBe("—");
    expect(p.color).toBe("#8b98a5");
    expect(p.level).toBe("unknown");
  });

  it("detail='denied' OVERRIDES state → red MIC DENIED, level=bad", () => {
    const p = formatVoiceStatePresentation(
      frame({ state: "listening", detail: "denied" })
    );
    expect(p.label).toBe("MIC DENIED");
    expect(p.color).toBe("#e01b24");
    expect(p.ariaText).toContain("denied");
    expect(p.level).toBe("bad");
  });

  it("detail='silenced' OVERRIDES state → grey MUTED, level=warn", () => {
    const p = formatVoiceStatePresentation(
      frame({ state: "speaking", detail: "silenced" })
    );
    expect(p.label).toBe("MUTED");
    expect(p.color).toBe("#8b98a5");
    expect(p.ariaText).toContain("muted");
    expect(p.level).toBe("warn");
  });

  it("detail='denied' wins over 'unknown' state too (operator must see it)", () => {
    const p = formatVoiceStatePresentation(
      frame({ state: "unknown", detail: "denied" })
    );
    expect(p.label).toBe("MIC DENIED");
    expect(p.level).toBe("bad");
  });
});
