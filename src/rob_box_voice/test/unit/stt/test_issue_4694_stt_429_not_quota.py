"""#4694 — HTTP 429 MiniMax STT не «кончились деньги».

Живой Sound 10.1.1.22, 2026-11-18: 429 rate-limit при живом TTS-балансе
озвучивался как ``provider.exhausted.quota``. Квота (2056 / QUOTA_HINTS)
по-прежнему permanent + фраза про деньги; 429 — transient, цепочка дальше.
"""

from __future__ import annotations

import importlib
import sys
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from rob_box_dialog import phrases
from rob_box_dialog.degrade import ProviderState, phrase_on_change
from rob_box_voice.stt_fallback import (
    ProviderDeadCache,
    ProviderPolicy,
    STTAuthError,
    STTQuotaError,
    failure_class,
    is_permanent_failure,
    select_recognition,
)
from rob_box_voice.stt_providers.minimax_provider import (
    MiniMaxSTTAuthError,
    MiniMaxSTTQuotaError,
    MiniMaxSTTRateLimitError,
)


class _Adapter:
    def __init__(self, name: str, fn):
        self.name = name
        self._fn = fn

    def recognize(self, audio_bytes: bytes):
        return self._fn(audio_bytes)


@pytest.fixture
def stt_node_module(monkeypatch):
    """stt_node без ROS: те же заглушки, что #4135."""
    import rob_box_voice

    stubs = {
        "rclpy": SimpleNamespace(init=None, shutdown=None, spin=None, ok=lambda: False),
        "rclpy.node": SimpleNamespace(Node=type("Node", (), {})),
        "rclpy.qos": MagicMock(),
        "std_msgs": MagicMock(),
        "std_msgs.msg": MagicMock(),
        "audio_common_msgs": MagicMock(),
        "audio_common_msgs.msg": MagicMock(),
        "vosk": MagicMock(),
        "grpc": MagicMock(),
    }
    for name, module in stubs.items():
        monkeypatch.setitem(sys.modules, name, module)
    sys.modules.pop("rob_box_voice.stt_node", None)
    if hasattr(rob_box_voice, "stt_node"):
        delattr(rob_box_voice, "stt_node")
    yield importlib.import_module("rob_box_voice.stt_node")
    sys.modules.pop("rob_box_voice.stt_node", None)
    if hasattr(rob_box_voice, "stt_node"):
        delattr(rob_box_voice, "stt_node")


def _call_recognize_minimax(module, side_effect):
    node = SimpleNamespace(
        _grip_minimax_language=None,
        _ensure_minimax_provider=lambda: MagicMock(transcribe=MagicMock(side_effect=side_effect)),
    )
    return module.STTNode._recognize_minimax(node, b"\x00" * 100)


def test_rate_limit_429_is_not_stt_quota_error(stt_node_module):
    with pytest.raises(MiniMaxSTTRateLimitError) as caught:
        _call_recognize_minimax(
            stt_node_module,
            MiniMaxSTTRateLimitError("minimax STT: rate-limited (HTTP 429)"),
        )
    assert not isinstance(caught.value, STTQuotaError)
    assert is_permanent_failure(caught.value) is False
    assert failure_class(None, caught.value) != "quota"


def test_quota_2056_still_stt_quota_error(stt_node_module):
    with pytest.raises(STTQuotaError) as caught:
        _call_recognize_minimax(
            stt_node_module,
            MiniMaxSTTQuotaError("minimax STT: Token Plan usage limit reached (2056)"),
        )
    assert is_permanent_failure(caught.value) is True
    assert failure_class(None, caught.value) == "quota"


def test_auth_still_stt_auth_error(stt_node_module):
    with pytest.raises(STTAuthError):
        _call_recognize_minimax(stt_node_module, MiniMaxSTTAuthError("HTTP 401"))


def test_429_chain_falls_through_short_ttl_no_money_phrase():
    cache = ProviderDeadCache(ttl_s=300.0, transient_ttl_s=30.0)
    policies = {
        "minimax": ProviderPolicy(timeout_s=5.0, max_retries=0, retry_backoff_s=0.0),
        "vosk": ProviderPolicy(timeout_s=5.0, max_retries=0, retry_backoff_s=0.0),
    }

    def minimax(_audio):
        raise MiniMaxSTTRateLimitError("minimax STT: rate-limited (HTTP 429)")

    text, attempts = select_recognition(
        [_Adapter("minimax", minimax), _Adapter("vosk", lambda _: "робот привет")],
        b"\x00" * 200,
        policies=policies,
        dead_cache=cache,
        min_text_chars=3,
        retry_backoff_s=0.0,
    )

    assert text == "робот привет"
    assert [a.provider for a in attempts] == ["minimax", "vosk"]
    assert attempts[0].reason == "error"
    assert 0 < cache.remaining_s("minimax") <= 30
    assert cache.health_class("minimax") != "quota"
    assert phrase_on_change(None, ProviderState("minimax", "stt", "quota")) == ("provider.exhausted.quota")
    assert "кончились деньги" in phrases.text(
        "provider.exhausted.quota",
        subject="minimax (распознавание речи)",
        next="x",
    )
    assert phrase_on_change(None, ProviderState("minimax", "stt", "down")) != ("provider.exhausted.quota")


def test_quota_2056_chain_long_ttl_and_money_phrase_key():
    cache = ProviderDeadCache(ttl_s=300.0, transient_ttl_s=30.0)
    policies = {
        "minimax": ProviderPolicy(timeout_s=5.0, max_retries=1, retry_backoff_s=0.0),
        "vosk": ProviderPolicy(timeout_s=5.0, max_retries=0, retry_backoff_s=0.0),
    }
    calls = {"n": 0}

    def minimax(_audio):
        calls["n"] += 1
        raise STTQuotaError("minimax STT: Token Plan usage limit reached (2056)")

    text, _attempts = select_recognition(
        [_Adapter("minimax", minimax), _Adapter("vosk", lambda _: "робот привет")],
        b"\x00" * 200,
        policies=policies,
        dead_cache=cache,
        min_text_chars=3,
        retry_backoff_s=0.0,
    )

    assert text == "робот привет"
    assert calls["n"] == 1
    assert cache.remaining_s("minimax") > 100
    assert cache.health_class("minimax") == "quota"
    assert phrase_on_change(None, ProviderState("minimax", "stt", "quota")) == ("provider.exhausted.quota")
