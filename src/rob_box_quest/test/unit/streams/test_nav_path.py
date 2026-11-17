"""nav_path (0x1104, issue #3151): прорежение, кодек, дроссель."""

from __future__ import annotations

import struct
from types import SimpleNamespace

import msgpack
import pytest

from rob_box_quest.streams.nav_path import (
    NAV_PATH_MAX_POINTS,
    NAV_PATH_MIN_PERIOD_S,
    NavPathThrottle,
    decimate_path,
    encode_nav_path,
    path_msg_points,
)


def test_short_path_kept_as_is():
    pts = [(0.0, 0.0), (1.0, 0.5), (2.0, 1.0)]
    assert decimate_path(pts) == pts


def test_long_path_decimated_to_limit_and_keeps_ends():
    pts = [(float(i), float(-i)) for i in range(1234)]
    out = decimate_path(pts)
    assert len(out) == NAV_PATH_MAX_POINTS
    assert out[0] == pts[0]
    assert out[-1] == pts[-1]
    xs = [p[0] for p in out]
    assert xs == sorted(xs), "порядок точек сохраняется"


def test_decimate_rejects_degenerate_limit():
    with pytest.raises(ValueError):
        decimate_path([(0.0, 0.0)] * 5, max_points=1)


def test_encode_roundtrip_float32_pairs():
    raw = encode_nav_path(frame="map", points=[(1.5, -2.0), (3.25, 4.0)], ts_ms=42)
    body = msgpack.unpackb(raw, raw=False)
    assert body["frame"] == "map"
    assert body["n"] == 2
    assert body["ts_ms"] == 42
    assert struct.unpack("<4f", body["xy"]) == (1.5, -2.0, 3.25, 4.0)


def test_encode_empty_path_is_explicit_clear():
    body = msgpack.unpackb(encode_nav_path(frame="map", points=[], ts_ms=1), raw=False)
    assert body["n"] == 0
    assert body["xy"] == b""


def test_path_msg_points_reads_header_and_poses():
    def stamped(x, y):
        return SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y, z=0.0)))

    msg = SimpleNamespace(
        header=SimpleNamespace(frame_id="map"),
        poses=[stamped(0, 0), stamped(1, 2)],
    )
    assert path_msg_points(msg) == ("map", [(0.0, 0.0), (1.0, 2.0)])


def test_throttle_limits_rate_but_force_passes():
    t = NavPathThrottle()
    assert t.admit(10.0)
    assert not t.admit(10.0 + NAV_PATH_MIN_PERIOD_S / 2)
    assert t.admit(10.0 + NAV_PATH_MIN_PERIOD_S / 2, force=True)
    assert t.admit(10.0 + NAV_PATH_MIN_PERIOD_S * 2)
