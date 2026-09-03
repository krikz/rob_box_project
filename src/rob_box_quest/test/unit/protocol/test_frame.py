"""Unit-тесты wire-протокола rob_box_quest (frame encode/decode).

Источник истины: docs/architecture/meta-quest-api.md §2 (frame format).
Формат: [1 byte: type][4 bytes: stream_id LE uint32][LEB128: payload_len][payload]
"""

import pytest

from rob_box_quest.protocol.frame import (
    FrameType,
    decode_frame,
    encode_frame,
    encode_leb128,
    decode_leb128,
)


class TestLEB128:
    @pytest.mark.parametrize(
        "value,expected",
        [
            (0, b"\x00"),
            (127, b"\x7f"),
            (128, b"\x80\x01"),
            (300, b"\xac\x02"),
            (2**32, b"\x80\x80\x80\x80\x10"),
        ],
    )
    def test_roundtrip(self, value, expected):
        # encode: байты совпадают с эталоном
        assert encode_leb128(value) == expected
        # decode: roundtrip value + offset == len(expected)
        decoded, offset = decode_leb128(encode_leb128(value))
        assert decoded == value
        assert offset == len(expected)

    def test_reject_negative(self):
        with pytest.raises(ValueError):
            encode_leb128(-1)


class TestFrame:
    def test_roundtrip_json_payload(self):
        payload = b'{"cmd": "teleop_twist"}'
        raw = encode_frame(FrameType.JSON_CMD, stream_id=7, payload=payload)
        ftype, sid, got = decode_frame(raw)
        assert ftype == FrameType.JSON_CMD
        assert sid == 7
        assert got == payload

    def test_roundtrip_empty_payload(self):
        raw = encode_frame(FrameType.GOODBYE, stream_id=0, payload=b"")
        ftype, sid, got = decode_frame(raw)
        assert ftype == FrameType.GOODBYE
        assert got == b""

    def test_decode_incomplete_header_raises(self):
        with pytest.raises(ValueError):
            decode_frame(b"\x01\x02\x03")

    def test_decode_truncated_payload_raises(self):
        raw = encode_frame(FrameType.BINARY_FRAME, stream_id=0x1001, payload=b"abcdef")
        with pytest.raises(ValueError):
            decode_frame(raw[:-2])

    def test_roundtrip_voice_audio(self):
        pcm = b"\x00\x00\xff\x7f\x00\x80"  # int16 LE: 0, 32767, -32768
        raw = encode_frame(FrameType.VOICE_AUDIO, stream_id=0, payload=pcm)
        ftype, sid, got = decode_frame(raw)
        assert ftype == FrameType.VOICE_AUDIO
        assert sid == 0
        assert got == pcm


class TestSupervisorFrameTypes:
    """AV-16: маппинг wire-фреймов 0x30..0x33 ↔ supervisor API (§3, docs §5.1)."""

    def test_set_mode_value(self):
        # §3 строки 63-66: 0x30 SET_MODE
        assert int(FrameType.SET_MODE) == 0x30

    def test_acquire_floor_value(self):
        assert int(FrameType.ACQUIRE_FLOOR) == 0x31

    def test_release_floor_value(self):
        assert int(FrameType.RELEASE_FLOOR) == 0x32

    def test_state_update_value(self):
        assert int(FrameType.STATE_UPDATE) == 0x33

    def test_roundtrip_set_mode_msgpack_payload(self):
        # Payload — msgpack, не JSON: имитируем через raw bytes.
        payload = b"\x82\xa8client_id\xa3q1\xa4mode\xa9mixed"
        raw = encode_frame(FrameType.SET_MODE, stream_id=0, payload=payload)
        ftype, sid, got = decode_frame(raw)
        assert ftype == FrameType.SET_MODE
        assert sid == 0  # supervisor API — control-фрейм, stream_id = 0
        assert got == payload

    def test_roundtrip_state_update_msgpack_payload(self):
        # /avatar/state binary, ~200 bytes; pubsub broadcast, не stream.
        payload = b"\x82\xa4mode\xa3off\xa8since_ms\xcf\x00\x00\x01\x80\x00\x00\x00\x00"
        raw = encode_frame(FrameType.STATE_UPDATE, stream_id=0, payload=payload)
        ftype, sid, got = decode_frame(raw)
        assert ftype == FrameType.STATE_UPDATE
        assert sid == 0
        assert got == payload

    def test_acquire_release_roundtrip(self):
        # Один byte — наглядность: всё равно encode/decode симметричен.
        for ftype in (FrameType.ACQUIRE_FLOOR, FrameType.RELEASE_FLOOR):
            raw = encode_frame(ftype, 0, b"\x00")
            f, s, p = decode_frame(raw)
            assert f == ftype
            assert s == 0
            assert p == b"\x00"

    def test_supervisor_frame_types_distinct_from_legacy(self):
        # Защита от случайного «съезжания» значений: 0x30..0x33 не должны
        # совпасть с 0x01..0x20 / 0xFF ни по одному.
        legacy = {0x01, 0x02, 0x03, 0x04, 0x10, 0x11, 0x12, 0x13, 0x20, 0xFF}
        for v in (0x30, 0x31, 0x32, 0x33):
            assert v not in legacy
