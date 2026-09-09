#!/usr/bin/env python3
"""test_tts_finished_duration.py — #949 AC1: /voice/tts/finished carries duration_sec.

The finished event published by ``tts_node._publish_tts_finished`` must
include the actual synthesis duration (``duration_sec``) so downstream
consumers (dialogue_node / music arrangement) can time the beat to the rap.

These tests exercise the real ``_publish_tts_finished`` method on a stub
node (no ROS spin-up): we create an uninitialized ``TTSNode`` via
``object.__new__``, attach capturing publishers, and assert on the JSON
payload that hits the wire.

Heavy ROS/ML deps (rclpy, grpc, torch, sounddevice, audio_common_msgs) are
stubbed in ``sys.modules`` before the import — the same pattern the
``test_tools/`` suite uses — so the file runs both locally and in CI.

Run with::

    python3 -m pytest src/rob_box_voice/test/unit/tts/test_tts_finished_duration.py
"""

from __future__ import annotations

import json
import sys
import types
from unittest.mock import Mock

# Stub heavy third-party deps before importing tts_node (pattern from
# rob_box_mcp_tools/test/test_tools/). numpy is importable locally, so
# setdefault keeps the real module; the rest become Mock.
for _mod in (
    "grpc",
    "rclpy",
    "rclpy.node",
    "rclpy.qos",
    "sounddevice",
    "torch",
    "audio_common_msgs",
    "audio_common_msgs.msg",
):
    sys.modules.setdefault(_mod, Mock())

# Provide a real minimal ``String`` so the published message has a ``data``
# attribute we can json.loads().
class _FakeString:
    def __init__(self) -> None:
        self.data = ""


_fake_std_msgs = types.ModuleType("std_msgs")
_fake_std_msgs_msg = types.ModuleType("std_msgs.msg")
setattr(_fake_std_msgs_msg, "String", _FakeString)
setattr(_fake_std_msgs, "msg", _fake_std_msgs_msg)
sys.modules.setdefault("std_msgs", _fake_std_msgs)
sys.modules.setdefault("std_msgs.msg", _fake_std_msgs_msg)

from rob_box_voice.tts_node import TTSNode  # noqa: E402


class _CapturingPublisher:
    """Records published messages (std_msgs-like)."""

    def __init__(self) -> None:
        self.messages: list = []

    def publish(self, msg) -> None:
        self.messages.append(msg)


def _make_node():
    """Uninitialized TTSNode with capturing publishers (no ROS)."""
    node = object.__new__(TTSNode)
    node.finished_pub = _CapturingPublisher()
    node.batch_complete_pub = _CapturingPublisher()
    node._logger = Mock()
    node.get_logger = lambda: node._logger
    return node


def test_finished_event_includes_duration_sec() -> None:
    """AC1: success finished event carries the real synthesis duration."""
    node = _make_node()
    node._publish_tts_finished("sid-1", success=True, duration_sec=12.34)

    assert len(node.finished_pub.messages) == 1
    payload = json.loads(node.finished_pub.messages[0].data)
    assert payload["speech_id"] == "sid-1"
    assert payload["success"] is True
    assert payload["duration_sec"] == 12.34


def test_finished_event_omits_duration_sec_when_none() -> None:
    """Back-compat: without duration the payload must look like before."""
    node = _make_node()
    node._publish_tts_finished("sid-1", success=True)

    payload = json.loads(node.finished_pub.messages[0].data)
    assert payload["speech_id"] == "sid-1"
    assert payload["success"] is True
    assert "duration_sec" not in payload


def test_finished_event_duration_sec_with_batch_metadata() -> None:
    """duration_sec survives the #980 batch-metadata path (last chunk)."""
    node = _make_node()
    node._publish_tts_finished(
        "sid-1",
        success=True,
        duration_sec=7.5,
        batch_id="b-1",
        batch_index=2,
        batch_total=2,
    )

    # Last chunk → batch_complete side-channel fires + finished republished
    # with the batch_complete marker, keeping duration_sec intact.
    assert len(node.batch_complete_pub.messages) == 1
    bc_payload = json.loads(node.batch_complete_pub.messages[0].data)
    assert bc_payload["batch_id"] == "b-1"
    assert bc_payload["chunks_total"] == 2

    finished_payloads = [
        json.loads(m.data) for m in node.finished_pub.messages
    ]
    # Every published finished payload carries duration_sec.
    assert finished_payloads
    for payload in finished_payloads:
        assert payload["duration_sec"] == 7.5
    # The last republish carries the closure marker.
    assert finished_payloads[-1]["batch_complete"] is True


def test_finished_event_error_branch_can_carry_duration() -> None:
    """Error branch keeps the optional field when provided (defensive)."""
    node = _make_node()
    node._publish_tts_finished(
        "sid-1", success=False, error="stopped", duration_sec=3.1
    )

    payload = json.loads(node.finished_pub.messages[0].data)
    assert payload["success"] is False
    assert payload["error"] == "stopped"
    assert payload["duration_sec"] == 3.1
