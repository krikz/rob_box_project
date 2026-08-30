// Тесты минимального msgpack-декодера (wire/msgpack.ts).
//
// Байтовые векторы получены из msgpack.packb(..., use_bin_type=True) —
// того же вызова, что делает сервер в protocol/topics.py.

import { describe, it, expect } from "vitest";
import { decodeMsgpack, decodeMsgpackMap } from "../src/wire/msgpack";

const bytes = (...b: number[]) => new Uint8Array(b);

describe("decodeMsgpack scalars", () => {
  it("decodes positive fixint", () => {
    expect(decodeMsgpack(bytes(0x00))).toBe(0);
    expect(decodeMsgpack(bytes(0x7f))).toBe(127);
  });

  it("decodes negative fixint", () => {
    expect(decodeMsgpack(bytes(0xff))).toBe(-1);
    expect(decodeMsgpack(bytes(0xe0))).toBe(-32);
  });

  it("decodes nil / booleans", () => {
    expect(decodeMsgpack(bytes(0xc0))).toBeNull();
    expect(decodeMsgpack(bytes(0xc2))).toBe(false);
    expect(decodeMsgpack(bytes(0xc3))).toBe(true);
  });

  it("decodes uint8/16/32", () => {
    expect(decodeMsgpack(bytes(0xcc, 0xc8))).toBe(200);
    expect(decodeMsgpack(bytes(0xcd, 0x30, 0x39))).toBe(12345);
    expect(decodeMsgpack(bytes(0xce, 0x00, 0x01, 0x00, 0x00))).toBe(65536);
  });

  it("decodes int8/16/32", () => {
    expect(decodeMsgpack(bytes(0xd0, 0xd8))).toBe(-40);
    expect(decodeMsgpack(bytes(0xd1, 0xff, 0x38))).toBe(-200);
    expect(decodeMsgpack(bytes(0xd2, 0xff, 0xff, 0x00, 0x00))).toBe(-65536);
  });

  it("decodes uint64 (ts_ms) without precision loss", () => {
    // msgpack.packb(1_700_000_000_123) → cf 0000018bcfe5687b
    const ts = decodeMsgpack(bytes(0xcf, 0x00, 0x00, 0x01, 0x8b, 0xcf, 0xe5, 0x68, 0x7b));
    expect(ts).toBe(1_700_000_000_123);
  });

  it("decodes float64", () => {
    // 0.5 → 3FE0000000000000
    expect(decodeMsgpack(bytes(0xcb, 0x3f, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00))).toBe(0.5);
  });

  it("decodes fixstr and str8", () => {
    expect(decodeMsgpack(bytes(0xa3, 0x61, 0x62, 0x63))).toBe("abc");
    expect(decodeMsgpack(bytes(0xd9, 0x03, 0x78, 0x79, 0x7a))).toBe("xyz");
  });

  it("decodes bin8", () => {
    const v = decodeMsgpack(bytes(0xc4, 0x02, 0x01, 0x02));
    expect(v).toBeInstanceOf(Uint8Array);
    expect(Array.from(v as Uint8Array)).toEqual([1, 2]);
  });
});

describe("decodeMsgpack containers", () => {
  it("decodes fixarray", () => {
    expect(decodeMsgpack(bytes(0x93, 0x01, 0x02, 0x03))).toEqual([1, 2, 3]);
  });

  it("decodes fixmap", () => {
    // {"a": 1, "b": 2}
    const v = decodeMsgpack(bytes(0x82, 0xa1, 0x61, 0x01, 0xa1, 0x62, 0x02));
    expect(v).toEqual({ a: 1, b: 2 });
  });

  it("decodes nested map inside array", () => {
    // [{"a": true}]
    const v = decodeMsgpack(bytes(0x91, 0x81, 0xa1, 0x61, 0xc3));
    expect(v).toEqual([{ a: true }]);
  });
});

describe("decodeMsgpack errors", () => {
  it("throws on truncated payload", () => {
    expect(() => decodeMsgpack(bytes(0xcd, 0x30))).toThrow(/truncated/);
  });

  it("throws on unsupported ext type", () => {
    expect(() => decodeMsgpack(bytes(0xd4, 0x00, 0x00))).toThrow(/unsupported/);
  });
});

describe("decodeMsgpackMap", () => {
  it("returns the map", () => {
    expect(decodeMsgpackMap(bytes(0x81, 0xa1, 0x61, 0x01))).toEqual({ a: 1 });
  });

  it("returns null for non-map values", () => {
    expect(decodeMsgpackMap(bytes(0x93, 0x01, 0x02, 0x03))).toBeNull();
    expect(decodeMsgpackMap(bytes(0xc0))).toBeNull();
  });

  it("returns null instead of throwing on garbage", () => {
    expect(decodeMsgpackMap(bytes(0xcd, 0x30))).toBeNull();
  });
});
