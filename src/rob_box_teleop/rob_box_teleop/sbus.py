"""SBUS frame protocol helpers for rob_box_teleop.

SBUS (Futaba serial bus) frames are fixed 25 bytes, sent continuously at
100000 baud 8E2 (8 data bits, even parity, 2 stop bits):

    byte 0     : header, always 0x0F
    bytes 1-22 : 16 channels x 11 bits, little-endian packed (176 bits)
    byte 23    : flags (bit 0: ch17, bit 1: ch18, bit 2: frame lost,
                 bit 3: failsafe)
    byte 24    : footer, always 0x00

Channel data bytes may legally contain 0x0F, so a naive "scan for 0x0F then
read 24 bytes" sync can lock onto a data byte and decode garbage channels —
observed as throttle pulsing 0.999/0.0 (issue #1345). The helpers here
decode, validate and resync SBUS frames defensively:

    * :func:`decode_channels` — 16 x 11-bit channel extraction.
    * :func:`validate_packet` — rejects structurally broken frames (bad
      header/footer, out-of-range channel values). Frames flagged
      frame-lost/failsafe are structurally valid but carry unreliable data;
      the caller decides how to react (the joystick node publishes neutral).
    * :func:`read_frame` — sliding-window resync: after a failed validation
      only the false header byte is dropped and the search continues in the
      already-read bytes, so the reader never skips past the real header
      and cannot stay desynced forever.
"""

from typing import Callable, List, Optional

SBUS_FRAME_SIZE = 25
SBUS_HEADER = 0x0F
SBUS_FOOTER = 0x00
SBUS_CHANNEL_MIN = 172
SBUS_CHANNEL_MAX = 1811
SBUS_CHANNEL_CENTER = 992
SBUS_FLAG_FRAME_LOST = 0b0100
SBUS_FLAG_FAILSAFE = 0b1000
# Flags that mean "channel data is not reliable for control".
SBUS_FLAG_REJECT = SBUS_FLAG_FRAME_LOST | SBUS_FLAG_FAILSAFE
# If no valid (non-failsafe) frame arrives within this many seconds the link
# is considered dead and the caller should publish neutral instead of
# holding the last commanded value.
SBUS_STALE_TIMEOUT = 0.2


def decode_channels(packet: bytes) -> List[int]:
    """Decode 16 x 11-bit channel values from a 25-byte SBUS frame."""
    if len(packet) != SBUS_FRAME_SIZE:
        raise ValueError(
            f"SBUS frame must be {SBUS_FRAME_SIZE} bytes, got {len(packet)}"
        )
    p = packet
    return [
        (p[1] | p[2] << 8) & 0x07FF,
        (p[2] >> 3 | p[3] << 5) & 0x07FF,
        (p[3] >> 6 | p[4] << 2 | p[5] << 10) & 0x07FF,
        (p[5] >> 1 | p[6] << 7) & 0x07FF,
        (p[6] >> 4 | p[7] << 4) & 0x07FF,
        (p[7] >> 7 | p[8] << 1 | p[9] << 9) & 0x07FF,
        (p[9] >> 2 | p[10] << 6) & 0x07FF,
        (p[10] >> 5 | p[11] << 3) & 0x07FF,
        (p[12] | p[13] << 8) & 0x07FF,
        (p[13] >> 3 | p[14] << 5) & 0x07FF,
        (p[14] >> 6 | p[15] << 2 | p[16] << 10) & 0x07FF,
        (p[16] >> 1 | p[17] << 7) & 0x07FF,
        (p[17] >> 4 | p[18] << 4) & 0x07FF,
        (p[18] >> 7 | p[19] << 1 | p[20] << 9) & 0x07FF,
        (p[20] >> 2 | p[21] << 6) & 0x07FF,
        (p[21] >> 5 | p[22] << 3) & 0x07FF,
    ]


def validate_packet(packet: bytes) -> bool:
    """Return True if ``packet`` is a structurally usable SBUS frame.

    Checks, in order:
      * length is exactly :data:`SBUS_FRAME_SIZE` bytes;
      * header byte is 0x0F and footer byte is 0x00;
      * every decoded channel lies within the SBUS range
        [172, 1811] (values outside it mean the frame is mis-synced
        garbage, not a valid receiver output).

    frame-lost/failsafe flags are intentionally NOT rejected here: such
    frames are structurally valid, and dropping them entirely would hide
    link-loss from the caller. The node handles them by publishing neutral.
    """
    if len(packet) != SBUS_FRAME_SIZE:
        return False
    if packet[0] != SBUS_HEADER or packet[SBUS_FRAME_SIZE - 1] != SBUS_FOOTER:
        return False
    return all(
        SBUS_CHANNEL_MIN <= ch <= SBUS_CHANNEL_MAX
        for ch in decode_channels(packet)
    )


def read_frame(read: Callable[[int], bytes]) -> Optional[bytes]:
    """Read one validated SBUS frame from a byte stream.

    ``read(n)`` behaves like pyserial ``Serial.read(n)``: returns up to ``n``
    bytes, or ``b''`` when the stream is idle/EOF.

    Why a sliding window: channel data bytes may be 0x0F, so the first 0x0F
    in the stream is not necessarily a frame header. After a failed
    validation only the first byte is dropped and the search resumes inside
    the bytes already read — if the whole frame were discarded instead, the
    reader would skip past the real header and stay permanently desynced
    (the observed throttle pulsation).
    """
    buf = bytearray()

    while True:
        # Find a candidate header inside bytes we already hold.
        idx = buf.find(bytes([SBUS_HEADER]))
        if idx < 0:
            chunk = read(SBUS_FRAME_SIZE)
            if not chunk:
                return None
            buf.extend(chunk)
            idx = buf.find(bytes([SBUS_HEADER]))
            if idx < 0:
                # A full frame's worth of bytes with no header — stale line
                # noise. Drop and keep listening.
                buf.clear()
                continue

        # Discard bytes before the candidate header.
        del buf[:idx]

        # Make sure we hold a complete frame before validating.
        while len(buf) < SBUS_FRAME_SIZE:
            chunk = read(SBUS_FRAME_SIZE - len(buf))
            if not chunk:
                return None
            buf.extend(chunk)

        frame = bytes(buf[:SBUS_FRAME_SIZE])
        if validate_packet(frame):
            del buf[:SBUS_FRAME_SIZE]
            return frame

        # False header (0x0F inside channel data): keep the remaining bytes
        # and look for the next candidate — the real header is likely there.
        del buf[:1]
