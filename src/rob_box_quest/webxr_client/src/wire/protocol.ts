// Wire-protocol codec для rob_box_quest.
//
// Зеркало src/rob_box_quest/rob_box_quest/protocol/frame.py.
// Формат (docs/architecture/meta-quest-api.md §2):
//   [1 byte: type][4 bytes: stream_id LE uint32][LEB128: payload_len][payload]
//
// ВАЖНО (расхождение с api.md §4, зафиксировано 25.08.2026 в канбан-карточке #1639):
// api.md обещает 4-байтовый topic_id в начале payload BINARY_FRAME.
// Реальный сервер (commit b720c8af, src/rob_box_quest/rob_box_quest/server/ws_server.py:160)
// шлёт payload as-is через Bridge.publish_frame → CameraProvider.encode → JPEG bytes
// БЕЗ префикса topic_id. Клиент роутит входящие BINARY_FRAME по `stream_id`
// из заголовка + маппингу stream_id→topic из `subscribe_ack`.
// TODO(future-proof): если api.md когда-нибудь вернёт 4-byte topic_id префикс —
// добавить условный парсинг здесь, см. meta-quest-api.md §4.

export enum FrameType {
  HELLO = 0x01,
  WELCOME = 0x02,
  SUBSCRIBE = 0x03,
  UNSUBSCRIBE = 0x04,
  BINARY_FRAME = 0x10,
  JSON_CMD = 0x11,
  JSON_EVENT = 0x12,
  GOODBYE = 0x20,
  ERROR = 0xff
}

// ------------------------------------------------------------------
// LEB128 unsigned varint (mirror encode_leb128 / decode_leb128).
// ------------------------------------------------------------------

export function encodeLeb128(value: number): Uint8Array {
  if (!Number.isInteger(value) || value < 0) {
    throw new RangeError(`LEB128 supports unsigned integer >= 0, got ${value}`);
  }
  const out: number[] = [];
  let v = value;
  while (true) {
    const byte = v & 0x7f;
    v >>>= 7;
    if (v !== 0) {
      out.push(byte | 0x80);
    } else {
      out.push(byte);
      break;
    }
  }
  return new Uint8Array(out);
}

export function decodeLeb128(data: Uint8Array, offset = 0): [number, number] {
  let result = 0;
  let shift = 0;
  let off = offset;
  // Cap at 70 bits like Python version to avoid runaway.
  while (true) {
    if (off >= data.length) {
      throw new RangeError("truncated LEB128");
    }
    const byte = data[off];
    off += 1;
    result |= (byte & 0x7f) << shift;
    if ((byte & 0x80) === 0) {
      return [result >>> 0, off];
    }
    shift += 7;
    if (shift > 70) {
      throw new RangeError("LEB128 too long");
    }
  }
}

// ------------------------------------------------------------------
// Frame codec.
// ------------------------------------------------------------------

export interface DecodedFrame {
  type: FrameType;
  streamId: number;
  payload: Uint8Array;
}

const HEADER_SIZE = 5; // 1 byte type + 4 bytes stream_id

export function encodeFrame(
  type: FrameType,
  streamId: number,
  payload: Uint8Array
): Uint8Array {
  if (!Number.isInteger(streamId) || streamId < 0 || streamId > 0xffffffff) {
    throw new RangeError(`streamId must be uint32, got ${streamId}`);
  }
  const header = new Uint8Array(HEADER_SIZE);
  const view = new DataView(header.buffer);
  view.setUint8(0, type & 0xff);
  view.setUint32(1, streamId >>> 0, true);
  const leb = encodeLeb128(payload.length);
  const out = new Uint8Array(HEADER_SIZE + leb.length + payload.length);
  out.set(header, 0);
  out.set(leb, HEADER_SIZE);
  out.set(payload, HEADER_SIZE + leb.length);
  return out;
}

export function decodeFrame(raw: Uint8Array): DecodedFrame {
  if (raw.length < HEADER_SIZE) {
    throw new RangeError("incomplete header");
  }
  const view = new DataView(raw.buffer, raw.byteOffset, raw.byteLength);
  const type = view.getUint8(0) as FrameType;
  const streamId = view.getUint32(1, true);
  const [payloadLen, off] = decodeLeb128(raw, HEADER_SIZE);
  if (off + payloadLen > raw.length) {
    throw new RangeError("truncated payload");
  }
  const payload = raw.subarray(off, off + payloadLen);
  return { type, streamId, payload };
}

// Удобный хелпер: парсит payload JSON для JSON_CMD / JSON_EVENT.
export function decodeJsonPayload<T = unknown>(frame: DecodedFrame): T {
  const text = new TextDecoder("utf-8").decode(frame.payload);
  return JSON.parse(text) as T;
}

// Удобный хелпер: сериализует JSON и возвращает готовый BINARY_FRAME.
export function encodeJsonFrame(
  type: FrameType,
  streamId: number,
  obj: unknown
): Uint8Array {
  const text = JSON.stringify(obj);
  const bytes = new TextEncoder().encode(text);
  return encodeFrame(type, streamId, bytes);
}