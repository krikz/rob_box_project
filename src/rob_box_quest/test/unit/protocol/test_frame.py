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
