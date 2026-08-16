#!/usr/bin/env python3
"""Unit tests for rob_box_teleop.sbus (issue #1345 desync fix).

Pure Python — no rclpy, no serial hardware. Verifies:
  * 11-bit channel decode round-trips the standard SBUS packing;
  * validate_packet() rejects broken frames (length/header/footer/range);
  * read_frame() recovers from a mid-frame start where the first 0x0F is a
    channel data byte, not a header (the desync that made throttle pulse
    0.999/0.0).
"""

import pytest

from rob_box_teleop.sbus import (
    SBUS_CHANNEL_CENTER,
    SBUS_CHANNEL_MAX,
    SBUS_CHANNEL_MIN,
    SBUS_FRAME_SIZE,
    SBUS_HEADER,
    SBUS_FOOTER,
    decode_channels,
    read_frame,
    validate_packet,
)


def pack_channels(channels):
    """Pack 16 x 11-bit channel values into the 22 SBUS data bytes."""
    data = bytearray(22)
    for i, ch in enumerate(channels):
        value = ch & 0x07FF
        bitpos = i * 11
        for bit in range(11):
            if (value >> bit) & 1:
                byte_idx = (bitpos + bit) // 8
                bit_idx = (bitpos + bit) % 8
                data[byte_idx] |= 1 << bit_idx
    return bytes(data)


def make_frame(channels, flags=0x00, footer=SBUS_FOOTER, header=SBUS_HEADER):
    """Build a 25-byte SBUS frame from 16 channel values."""
    assert len(channels) == 16
    return bytes([header]) + pack_channels(channels) + bytes([flags, footer])


def gas_frame():
    """Realistic frame with gas held: Ch2 (index 1) = 1811, rest neutral."""
    channels = [SBUS_CHANNEL_CENTER] * 16
    channels[1] = 1811
    return make_frame(channels)


class FakeSerial:
    """Minimal pyserial-like reader for read_frame() tests."""

    def __init__(self, data, max_chunk=25):
        self._data = bytearray(data)
        self.max_chunk = max_chunk

    def read(self, n):
        if not self._data:
            return b""
        take = min(n, self.max_chunk, len(self._data))
        chunk = bytes(self._data[:take])
        del self._data[:take]
        return chunk


def test_decode_roundtrip():
    channels = [172, 1811, 992, 1500, 300, 700, 1100, 1300,
                500, 900, 1200, 1600, 400, 800, 1000, 1400]
    frame = make_frame(channels)
    assert decode_channels(frame) == channels


def test_decode_rejects_bad_length():
    with pytest.raises(ValueError):
        decode_channels(b"\x0f" + bytes(23))


def test_validate_accepts_valid_frame():
    assert validate_packet(make_frame([SBUS_CHANNEL_CENTER] * 16)) is True
    assert validate_packet(make_frame([SBUS_CHANNEL_MIN, SBUS_CHANNEL_MAX] + [992] * 14)) is True


def test_validate_rejects_bad_length():
    frame = make_frame([SBUS_CHANNEL_CENTER] * 16)
    assert validate_packet(frame[:-1]) is False
    assert validate_packet(frame + b"\x00") is False
    assert validate_packet(b"") is False


def test_validate_rejects_bad_header():
    assert validate_packet(make_frame([992] * 16, header=0x00)) is False
    assert validate_packet(make_frame([992] * 16, header=0x10)) is False


def test_validate_rejects_bad_footer():
    assert validate_packet(make_frame([992] * 16, footer=0xFF)) is False


def test_validate_rejects_out_of_range_channels():
    channels = [SBUS_CHANNEL_CENTER] * 16
    channels[3] = 0
    assert validate_packet(make_frame(channels)) is False
    channels[3] = 2047
    assert validate_packet(make_frame(channels)) is False


def test_validate_accepts_flag_bytes():
    # frame-lost/failsafe flags are structurally valid; the caller decides
    # how to react (joystick_control_node publishes neutral on them).
    assert validate_packet(make_frame([992] * 16, flags=0b0100)) is True
    assert validate_packet(make_frame([992] * 16, flags=0b1000)) is True


def test_read_frame_clean_stream():
    frame = gas_frame()
    ser = FakeSerial(frame + frame)
    assert read_frame(ser.read) == frame
    assert read_frame(ser.read) == frame
    assert read_frame(ser.read) is None


def test_read_frame_fragmented_stream():
    # Serial reads often return partial chunks; the buffer must assemble them.
    frame = gas_frame()
    ser = FakeSerial(frame + frame, max_chunk=7)
    assert read_frame(ser.read) == frame
    assert read_frame(ser.read) == frame
    assert read_frame(ser.read) is None


def test_read_frame_empty_stream():
    ser = FakeSerial(b"")
    assert read_frame(ser.read) is None


def test_read_frame_desync_recovery():
    """Regression: 0x0F inside channel data must not permanently desync.

    Scenario (issue #1345): the reader starts mid-frame, right at a channel
    data byte that happens to be 0x0F. A naive reader locks onto it, reads
    24 bytes of garbage and (when the false frame passes the weak footer
    check) decodes junk channels — throttle pulsing 0.999/0.0. read_frame()
    must drop the false header and find the real header of the next frame.
    """
    # Channel 0 = 1807 -> data byte 0 = 0x0F: a legal data byte that looks
    # like a frame header. This is the tail of a previous (partial) frame.
    prev_tail = pack_channels([1807] + [SBUS_CHANNEL_CENTER] * 15)
    prev_flags_footer = bytes([0x00, 0x00])

    gas = gas_frame()

    # Reader sees: 0x0F (data byte, false header) + rest of prev frame
    # + two real frames. It must skip the false header and return `gas`.
    stream = prev_tail + prev_flags_footer + gas + gas
    ser = FakeSerial(stream)
    got = read_frame(ser.read)
    assert got is not None
    assert got == gas
    assert decode_channels(got)[1] == 1811  # gas held, not neutral 992


def test_read_frame_noise_between_frames():
    # A few stray bytes between valid frames must not break sync.
    frame = gas_frame()
    noise = bytes([0x55, 0xAA, 0x0F, 0x13])  # includes a false 0x0F
    ser = FakeSerial(frame + noise + frame)
    assert read_frame(ser.read) == frame
    assert read_frame(ser.read) == frame
    assert read_frame(ser.read) is None
