import { describe, it, expect } from "vitest";
import {
  encodeFrame,
  decodeFrame,
  encodeLeb128,
  decodeLeb128,
  encodeJsonFrame,
  FrameType
} from "../src/wire/protocol";

describe("LEB128 varint", () => {
  it("encodes 0 as single byte", () => {
    expect(Array.from(encodeLeb128(0))).toEqual([0x00]);
  });

  it("encodes 127 as single byte", () => {
    expect(Array.from(encodeLeb128(127))).toEqual([0x7f]);
  });

  it("encodes 128 with continuation bit", () => {
    // 128 = 0x80 (low 7 bits) | 0x80 (continuation) → 0x80 0x01
    expect(Array.from(encodeLeb128(128))).toEqual([0x80, 0x01]);
  });

  it("round-trips a range of values", () => {
    for (const v of [0, 1, 64, 127, 128, 300, 16383, 16384, 1 << 20, 0xffffffff]) {
      const bytes = encodeLeb128(v);
      const [out, off] = decodeLeb128(bytes);
      expect(out).toBe(v >>> 0);
      expect(off).toBe(bytes.length);
    }
  });

  it("rejects negative values", () => {
    expect(() => encodeLeb128(-1)).toThrow(RangeError);
  });

  it("rejects truncated varint", () => {
    expect(() => decodeLeb128(new Uint8Array([0x80]), 0)).toThrow();
  });

  it("rejects varint > 10 bytes", () => {
    // 11 bytes with continuation bit each = > 70 bits shift.
    const long = new Uint8Array(11).fill(0x80);
    expect(() => decodeLeb128(long, 0)).toThrow(/too long/);
  });
});

describe("frame codec round-trip", () => {
  it("encodes HELLO header without payload", () => {
    const bytes = encodeFrame(FrameType.HELLO, 0, new Uint8Array(0));
    expect(bytes[0]).toBe(FrameType.HELLO);
    // stream_id = 0 little-endian uint32
    expect(Array.from(bytes.subarray(1, 5))).toEqual([0, 0, 0, 0]);
    // payload_len = 0 (LEB128)
    expect(bytes[5]).toBe(0);
    expect(bytes.length).toBe(6);
  });

  it("round-trips JSON_CMD with payload", () => {
    const payload = new TextEncoder().encode(JSON.stringify({ cmd: "teleop_twist", seq: 1 }));
    const bytes = encodeFrame(FrameType.JSON_CMD, 0x0fff, payload);
    const decoded = decodeFrame(bytes);
    expect(decoded.type).toBe(FrameType.JSON_CMD);
    expect(decoded.streamId).toBe(0x0fff);
    expect(new TextDecoder().decode(decoded.payload)).toContain("teleop_twist");
  });

  it("round-trips BINARY_FRAME with JPEG-ish payload", () => {
    const jpeg = new Uint8Array([0xff, 0xd8, 0xff, 0xe0, 0x00, 0x10]);
    const bytes = encodeFrame(FrameType.BINARY_FRAME, 0x1001, jpeg);
    const decoded = decodeFrame(bytes);
    expect(decoded.type).toBe(FrameType.BINARY_FRAME);
    expect(decoded.streamId).toBe(0x1001);
    expect(Array.from(decoded.payload)).toEqual(Array.from(jpeg));
  });

  it("rejects truncated payload", () => {
    const payload = new Uint8Array(10);
    const bytes = encodeFrame(FrameType.JSON_EVENT, 0, payload);
    const truncated = bytes.subarray(0, bytes.length - 3);
    expect(() => decodeFrame(truncated)).toThrow(/truncated/);
  });

  it("rejects incomplete header", () => {
    expect(() => decodeFrame(new Uint8Array(3))).toThrow(/header/);
  });
});

describe("JSON helper", () => {
  it("encodeJsonFrame produces a frame decodable to JSON", () => {
    const bytes = encodeJsonFrame(FrameType.JSON_EVENT, 0, {
      type: "heartbeat",
      ts_ms: 12345
    });
    const decoded = decodeFrame(bytes);
    const obj = JSON.parse(new TextDecoder().decode(decoded.payload));
    expect(obj).toEqual({ type: "heartbeat", ts_ms: 12345 });
  });
});

describe("VOICE_AUDIO frame", () => {
  it("defines VOICE_AUDIO = 0x13 (client→server)", () => {
    expect(FrameType.VOICE_AUDIO).toBe(0x13);
  });

  it("round-trips VOICE_AUDIO with raw int16 PCM payload", () => {
    // int16 LE samples: 0, 32767, -32768
    const pcm = new Uint8Array([0x00, 0x00, 0xff, 0x7f, 0x00, 0x80]);
    const bytes = encodeFrame(FrameType.VOICE_AUDIO, 0, pcm);
    const decoded = decodeFrame(bytes);
    expect(decoded.type).toBe(FrameType.VOICE_AUDIO);
    expect(decoded.streamId).toBe(0);
    expect(Array.from(decoded.payload)).toEqual(Array.from(pcm));
  });
});