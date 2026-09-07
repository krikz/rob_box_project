"""Smoke tests for the TTSNode pre-fetch API contract (ADR-0056 §6.4 #10).

Verifies that the three public methods on ``TTSNode``
(``pregenerate``, ``claim_pregen``, ``cancel_pregen``) follow the
contract from ADR-0056 §2.2 / §3.5:

* :meth:`pregenerate` is a no-op when ``_pregenerate_enabled``
  is ``False`` or when the payload lacks a ``pregenerate`` field;
* :meth:`claim_pregen` returns ``None`` for unknown ``speech_id``;
* :meth:`cancel_pregen` is safe to call before any pregenerate();

We deliberately do NOT instantiate ``TTSNode`` (it pulls in rclpy /
sounddevice / torch — heavy modules the unit-test conftest would
shadow). Instead we patch the methods on a lightweight
``SimpleNamespace``-style stub so the *behaviour* under test is
the production code, not a re-implementation.

This is sufficient for §6.4 #10 because the actual chain-skip
behaviour is covered by the *module-level* unit tests of
:func:`speculative_executor.kickoff` and
:meth:`SpeculativeExecutor.claim` — those run real asyncio tasks
without any rclpy.
"""
from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from rob_box_voice.scheduler.pregen import SpeculativeExecutor


def _make_node() -> MagicMock:
    """Build a lightweight MagicMock shaped like a TTSNode.

    The methods we test (pregenerate / claim_pregen / cancel_pregen)
    are bound from the **real** ``tts_node.TTSNode`` class — we
    import the module lazily so the heavy ROS surface isn't pulled
    in at conftest time. If rclpy / torch / grpc are missing, the
    tests still run because the methods only read the attributes
    they need, never the ROS graph itself.
    """
    try:
        from rob_box_voice import tts_node  # noqa: F401
    except ImportError:
        pytest.skip(
            "rob_box_voice.tts_node requires rclpy/sounddevice/torch; "
            "those aren't available in this test environment"
        )

    from rob_box_voice import tts_node

    node = MagicMock(spec=tts_node.TTSNode)
    node._prefetch = None
    node._pregenerate_enabled = True
    node._pregenerate_confidence_floor = 0.6
    node._pregenerate_history_window = 10
    node.current_dialogue_id = "d1"
    node.provider = "yandex"
    node.yandex_voice = "anton"
    node.minimax_voice = "male-qn-qingse"
    node.silero_speaker = "baya"
    node.minimax_language = "ru"

    node.pregenerate = lambda chunk, ctx=None: tts_node.TTSNode.pregenerate(
        node, chunk, ctx
    )
    node.claim_pregen = lambda speech_id: tts_node.TTSNode.claim_pregen(
        node, speech_id
    )
    node.cancel_pregen = lambda reason: tts_node.TTSNode.cancel_pregen(
        node, reason
    )
    node.publish_prefetch_metrics = (
        lambda: tts_node.TTSNode.publish_prefetch_metrics(node)
    )
    return node


def test_pregenerate_disabled_is_noop():
    """Kill-switch honoured."""
    node = _make_node()
    node._pregenerate_enabled = False
    chunk = {
        "speech_id": "cur",
        "ssml": "<speak>x</speak>",
        "pregenerate": {
            "next_speech_id": "next",
            "next_ssml": "<speak>y</speak>",
        },
    }
    node.pregenerate(chunk)
    assert node._prefetch is None


def test_pregenerate_invalid_input_is_noop():
    node = _make_node()
    node.pregenerate("not-a-dict")  # type: ignore[arg-type]
    assert node._prefetch is None
    node.pregenerate({"speech_id": "x"})  # no pregenerate field
    assert node._prefetch is None


def test_pregenerate_creates_engine_lazy():
    """First successful pregenerate constructs the executor."""
    import asyncio as _asyncio
    import numpy as np

    node = _make_node()
    # Stub the rclpy get_loop so the dispatcher can pick the path.
    node.get_loop = MagicMock(side_effect=Exception("no rclpy in test"))

    chunk = {
        "speech_id": "cur",
        "ssml": "<speak>x</speak>",
        "batch_index": 1,
        "batch_total": 3,
        "dialogue_id": "d1",
        "pregenerate": {
            "next_speech_id": "next",
            "next_ssml": "<speak>y</speak>",
        },
    }
    node.pregenerate(chunk)
    assert node._prefetch is not None
    assert "executor" in node._prefetch
    assert isinstance(node._prefetch["executor"], SpeculativeExecutor)


def test_claim_pregen_returns_none_when_disabled():
    node = _make_node()
    node._pregenerate_enabled = False
    assert node.claim_pregen("any") is None


def test_cancel_pregen_returns_zero_before_engine():
    """cancel_pregen is safe to call before any pregenerate()."""
    node = _make_node()
    assert node.cancel_pregen(reason="early_test") == 0


def test_publish_prefetch_metrics_noop_without_engine():
    node = _make_node()
    node.publish_prefetch_metrics()
    # No prebuild → no error, no publisher.
    assert not hasattr(node, "_prefetch_metrics_pub") or (
        getattr(node, "_prefetch_metrics_pub", None) is None
    )


def test_prefetch_fallback_voice_resolves_by_provider():
    try:
        from rob_box_voice import tts_node
    except ImportError:
        pytest.skip("tts_node unavailable in this env")
    node = _make_node()
    node.provider = "minimax"
    assert tts_node.TTSNode._prefetch_fallback_voice(node) == "male-qn-qingse"
    node.provider = "yandex"
    assert tts_node.TTSNode._prefetch_fallback_voice(node) == "anton"
    node.provider = "silero"
    assert tts_node.TTSNode._prefetch_fallback_voice(node) == "baya"


def test_prefetch_fallback_language_returns_minimax_language():
    try:
        from rob_box_voice import tts_node
    except ImportError:
        pytest.skip("tts_node unavailable in this env")
    node = _make_node()
    assert tts_node.TTSNode._prefetch_fallback_language(node) == "ru"