"""Wire-протокол rob_box_quest: frame encode/decode.

Формат (docs/architecture/meta-quest-api.md §2):
  [1 byte: type][4 bytes: stream_id LE][LEB128: payload_len][payload]

Этот модуль — чистая логика, без зависимостей от aiohttp / ROS / Zenoh.
Тестируется без rclpy.
"""

from __future__ import annotations

import struct
from enum import IntEnum


class FrameType(IntEnum):
    """Frame type byte (см. meta-quest-api.md §3)."""

    HELLO = 0x01
    WELCOME = 0x02
    SUBSCRIBE = 0x03
    UNSUBSCRIBE = 0x04
    BINARY_FRAME = 0x10
    JSON_CMD = 0x11
    JSON_EVENT = 0x12
    GOODBYE = 0x20
    ERROR = 0xFF


# Header: 1 byte type + 4 bytes stream_id (little-endian uint32).
HEADER_STRUCT = struct.Struct("<BI")  # total 5 bytes


def encode_leb128(value: int) -> bytes:
    """Unsigned LEB128 (Little Endian Base 128) — varint encoding.

    Raises ValueError для отрицательных значений (LEB128 в нашем протоколе
    только unsigned; signed — отдельная история через zig-zag, но мы её
    пока не используем).
    """
    if value < 0:
        raise ValueError("LEB128 supports unsigned only")
    out = bytearray()
    while True:
        byte = value & 0x7F
        value >>= 7
        if value:
            out.append(byte | 0x80)
        else:
            out.append(byte)
            break
    return bytes(out)


def decode_leb128(data: bytes, offset: int = 0) -> tuple[int, int]:
    """Decode unsigned LEB128. Возвращает (value, new_offset)."""
    result = 0
    shift = 0
    while True:
        if offset >= len(data):
            raise ValueError("truncated LEB128")
        byte = data[offset]
        offset += 1
        result |= (byte & 0x7F) << shift
        if not byte & 0x80:
            return result, offset
        shift += 7
        if shift > 70:
            raise ValueError("LEB128 too long")


def encode_frame(ftype: FrameType, stream_id: int, payload: bytes) -> bytes:
    """Encode фрейм целиком: header + leb128(len) + payload."""
    header = HEADER_STRUCT.pack(int(ftype), stream_id)
    return header + encode_leb128(len(payload)) + payload


def decode_frame(raw: bytes) -> tuple[FrameType, int, bytes]:
    """Decode фрейм. Raises ValueError на truncated/incomplete данных."""
    if len(raw) < HEADER_STRUCT.size:
        raise ValueError("incomplete header")
    ftype, stream_id = HEADER_STRUCT.unpack_from(raw)
    plen, off = decode_leb128(raw, HEADER_STRUCT.size)
    payload = raw[off : off + plen]
    if len(payload) != plen:
        raise ValueError("truncated payload")
    return FrameType(ftype), stream_id, payload
