// Frame codec для WSS (binary) — см. docs/architecture/meta-quest-api.md §2.
//
// Layout (little-endian):
//   [1 byte : type]
//   [4 bytes: stream_id]
//   [varint : payload_len (LEB128)]
//   [payload_len bytes : payload]
//
// Phase 2.2 (telemetry): используется JSON_EVENT (type=0x12) с payload
// `{type: "telemetry_perf", ...}` — это оборачивается в binary frame через
// encodeJsonFrame. Type TELEMETRY_PERF_BIN = 0x40 (raw binary, более
// компактный) опционален для Phase 3+; в Phase 2.2 используем JSON_EVENT.

export enum FrameType {
  HELLO = 0x01,
  WELCOME = 0x02,
  SUBSCRIBE = 0x03,
  UNSUBSCRIBE = 0x04,
  BINARY_FRAME = 0x10,
  JSON_CMD = 0x11,
  JSON_EVENT = 0x12,
  GOODBYE = 0x20,
  // Supervisor (Phase 2 v2 subprotocol):
  SET_MODE = 0x30,
  ACQUIRE_FLOOR = 0x31,
  RELEASE_FLOOR = 0x32,
  STATE_UPDATE = 0x33,
  // Phase 2.2 — telemetry (binary). Payload — msgpack или JSON.
  TELEMETRY_PERF_BIN = 0x40,
  ERROR = 0xff,
}

export interface DecodedFrame {
  type: FrameType;
  streamId: number;
  payload: Uint8Array;
}

const textEncoder = new TextEncoder();
const textDecoder = new TextDecoder();

// LEB128 unsigned varint encode/decode (small payloads, payload_len < 256 байт).
export function encodeLeb128(n: number): Uint8Array {
  if (n < 0 || !Number.isFinite(n) || !Number.isInteger(n)) {
    throw new RangeError("leb128: non-negative integer required");
  }
  const out: number[] = [];
  do {
    let byte = n & 0x7f;
    n >>>= 7;
    if (n !== 0) byte |= 0x80;
    out.push(byte);
  } while (n !== 0);
  return new Uint8Array(out);
}

export function decodeLeb128(buf: Uint8Array, offset: number): { value: number; bytesRead: number } {
  let result = 0;
  let shift = 0;
  let i = offset;
  while (i < buf.length) {
    const byte = buf[i];
    result |= (byte & 0x7f) << shift;
    i++;
    if ((byte & 0x80) === 0) {
      return { value: result >>> 0, bytesRead: i - offset };
    }
    shift += 7;
    if (shift > 35) {
      throw new Error("leb128: varint too long");
    }
  }
  throw new Error("leb128: unexpected EOF");
}

export function encodeFrame(type: FrameType, streamId: number, payload: Uint8Array): Uint8Array {
  if (streamId < 0 || streamId > 0xffffffff || !Number.isInteger(streamId)) {
    throw new RangeError("streamId must be uint32");
  }
  const lenBytes = encodeLeb128(payload.length);
  const total = 1 + 4 + lenBytes.length + payload.length;
  const out = new Uint8Array(total);
  const dv = new DataView(out.buffer);
  let p = 0;
  out[p++] = type;
  dv.setUint32(p, streamId >>> 0, true);
  p += 4;
  out.set(lenBytes, p);
  p += lenBytes.length;
  out.set(payload, p);
  return out;
}

export function encodeJsonFrame(type: FrameType, streamId: number, obj: unknown): Uint8Array {
  const json = textEncoder.encode(JSON.stringify(obj));
  return encodeFrame(type, streamId, json);
}

export function decodeFrame(buf: Uint8Array): DecodedFrame {
  if (buf.length < 5) {
    throw new Error("frame too short");
  }
  const type = buf[0] as FrameType;
  const dv = new DataView(buf.buffer, buf.byteOffset);
  const streamId = dv.getUint32(1, true);
  const { value: payloadLen, bytesRead: lebLen } = decodeLeb128(buf, 5);
  const payloadStart = 5 + lebLen;
  if (payloadStart + payloadLen > buf.length) {
    throw new Error("frame truncated");
  }
  const payload = buf.subarray(payloadStart, payloadStart + payloadLen);
  return { type, streamId, payload };
}

export function decodeJsonFrame<T = unknown>(buf: Uint8Array): T {
  return JSON.parse(textDecoder.decode(buf)) as T;
}