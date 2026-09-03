"""Unit tests for the new :mod:`rob_box_voice.tts_chain` module.

Covers the kanban card ``t_b33bfee1`` (issue #1976):

* Default chain ordering is Yandex → MiniMax → Silero (per Шифу).
* ``TTSProviderChain`` walks the chain in priority order, returning the
  first live provider's result.
* Transient failures (timeout / rate limit / quota / service unavailable)
  trigger failover to the next provider.
* Non-transient failures propagate unchanged (no silent swallowing,
  ADR-0018 honesty culture).
* All-providers-failed raises :class:`TTSUnavailable` carrying the
  per-provider exception list.
* Dead-cache integration: providers in the dead cache are skipped
  without invoking their ``synthesize``.
* Mark-dead integration: transient failures call ``mark_dead(name, exc)``
  exactly once per slot.
* ``chain_from_yaml_config`` deserialises the YAML shape from
  ``tts_node.yaml`` and rejects unknown / malformed entries.
* Legacy message-level transient classification
  (``"YANDEX_AUTHERROR"`` / ``"grpc unavailable"`` / ``"2056 Token Plan
  usage limit reached"``) — issue #1976 acceptance.

Tests are pure-Python (no rclpy / no grpc / no torch). They import
``tts_chain`` directly, which has no ROS2 dependency.
"""
from __future__ import annotations

from unittest.mock import MagicMock

import pytest

# No rclpy / grpc / torch required — tts_chain is pure Python.
from rob_box_voice.tts_chain import (
    DEFAULT_CHAIN,
    QuotaExceeded,
    RateLimit,
    ServiceUnavailable,
    Timeout,
    TransientTTSFailure,
    TTSProviderChain,
    TTSProviderSlot,
    TTSUnavailable,
    _is_transient_failure,
    chain_from_yaml_config,
)


# ── Defaults ─────────────────────────────────────────────────────────────────


def test_default_chain_is_yandex_first() -> None:
    """Acceptance (issue #1976): Yandex → MiniMax → Silero."""
    assert DEFAULT_CHAIN == ("yandex", "minimax", "silero")


# ── Happy path ──────────────────────────────────────────────────────────────


def _slot(name: str, priority: int, fn=None) -> TTSProviderSlot:
    return TTSProviderSlot(
        name=name,
        synthesize=fn or (lambda t, s, v: f"{name}-audio".encode()),
        priority=priority,
        voice=None,
    )


def test_happy_path_first_provider_used() -> None:
    chain = TTSProviderChain(
        [
            _slot("yandex", 1, fn=MagicMock(return_value=b"yandex-audio")),
            _slot("minimax", 2, fn=MagicMock(return_value=b"minimax-audio")),
            _slot("silero", 3, fn=MagicMock(return_value=b"silero-audio")),
        ]
    )
    result = chain.synthesize("hi", None, None)
    assert result == b"yandex-audio"


def test_priority_order_respected() -> None:
    """Insertion in non-priority order → still sorted by priority."""
    chain = TTSProviderChain(
        [
            _slot("silero", 3),
            _slot("yandex", 1),
            _slot("minimax", 2),
        ]
    )
    assert chain.names == ("yandex", "minimax", "silero")


def test_chain_with_one_provider_works() -> None:
    chain = TTSProviderChain([_slot("silero", 1)])
    assert chain.synthesize("hi", None, None) == b"silero-audio"


# ── Transient failover ──────────────────────────────────────────────────────


def test_yandex_down_falls_to_minimax() -> None:
    """Acceptance: Yandex fail → MiniMax is used."""
    yandex_fn = MagicMock(side_effect=Timeout("grpc unavailable"))
    minimax_fn = MagicMock(return_value=b"minimax-audio")
    chain = TTSProviderChain(
        [
            _slot("yandex", 1, fn=yandex_fn),
            _slot("minimax", 2, fn=minimax_fn),
            _slot("silero", 3),
        ]
    )
    assert chain.synthesize("hi", None, None) == b"minimax-audio"
    yandex_fn.assert_called_once()
    minimax_fn.assert_called_once()


def test_yandex_and_minimax_down_falls_to_silero() -> None:
    """Acceptance: Yandex+MiniMax fail → Silero answers."""
    chain = TTSProviderChain(
        [
            _slot("yandex", 1, fn=MagicMock(side_effect=QuotaExceeded("quota"))),
            _slot("minimax", 2, fn=MagicMock(side_effect=RateLimit("429"))),
            _slot("silero", 3, fn=MagicMock(return_value=b"silero-audio")),
        ]
    )
    assert chain.synthesize("hi", None, None) == b"silero-audio"


def test_all_providers_fail_raises_tts_unavailable() -> None:
    """Acceptance: all three fail → TTSUnavailable with 3 errors."""
    chain = TTSProviderChain(
        [
            _slot("yandex", 1, fn=MagicMock(side_effect=Timeout("net"))),
            _slot("minimax", 2, fn=MagicMock(side_effect=Timeout("net"))),
            _slot("silero", 3, fn=MagicMock(side_effect=Timeout("net"))),
        ]
    )
    with pytest.raises(TTSUnavailable) as ei:
        chain.synthesize("hi", None, None)
    assert len(ei.value.errors) == 3


def test_non_transient_error_propagates() -> None:
    """ValueError from provider 1 propagates immediately (no failover)."""
    chain = TTSProviderChain(
        [
            _slot("yandex", 1, fn=MagicMock(side_effect=ValueError("bad text"))),
            _slot("minimax", 2, fn=MagicMock(return_value=b"minimax-audio")),
        ]
    )
    with pytest.raises(ValueError, match="bad text"):
        chain.synthesize("hi", None, None)


# ── Transient classification (legacy message-level) ─────────────────────────


@pytest.mark.parametrize(
    "msg",
    [
        "YANDEX_AUTHERROR",
        "grpc unavailable",
        "2056 Token Plan usage limit reached",
        "HTTP/1.1 429 Too Many Requests",
        "HTTP/1.1 503 Service Unavailable",
        "MiniMax auth error, NO retry: 2056 Token Plan usage limit reached",
    ],
)
def test_is_transient_failure_matches_legacy_messages(msg: str) -> None:
    """Acceptance: legacy error strings trigger failover."""
    assert _is_transient_failure(RuntimeError(msg))


@pytest.mark.parametrize(
    "msg",
    [
        "bad input",
        "missing key",
        "AttributeError: foo",
    ],
)
def test_is_transient_failure_rejects_unrelated(msg: str) -> None:
    """Unrelated ``Exception`` messages are NOT transient — they propagate."""
    assert not _is_transient_failure(RuntimeError(msg))


@pytest.mark.parametrize(
    "exc_cls",
    [Timeout, RateLimit, QuotaExceeded, ServiceUnavailable, TransientTTSFailure],
)
def test_typed_transient_exceptions_always_failover(exc_cls) -> None:
    """Any subclass of :class:`TransientTTSFailure` triggers failover."""
    assert _is_transient_failure(exc_cls("x"))


# ── Dead-cache integration ──────────────────────────────────────────────────


def test_dead_cache_skips_provider() -> None:
    """Provider in dead cache → skipped without invoking synthesize."""
    dead: dict[str, bool] = {"yandex": True}
    yandex_fn = MagicMock(return_value=b"yandex-audio")
    minimax_fn = MagicMock(return_value=b"minimax-audio")
    chain = TTSProviderChain(
        [
            _slot("yandex", 1, fn=yandex_fn),
            _slot("minimax", 2, fn=minimax_fn),
        ],
        dead_cache=lambda name: dead.get(name, False),
    )
    assert chain.synthesize("hi", None, None) == b"minimax-audio"
    yandex_fn.assert_not_called()
    minimax_fn.assert_called_once()


def test_mark_dead_called_on_transient_failure() -> None:
    """Transient failures call mark_dead(name, exc) once per slot."""
    seen: list[tuple[str, BaseException]] = []

    chain = TTSProviderChain(
        [
            _slot("yandex", 1, fn=MagicMock(side_effect=Timeout("net"))),
            _slot("minimax", 2, fn=MagicMock(side_effect=Timeout("net"))),
            _slot("silero", 3, fn=MagicMock(side_effect=Timeout("net"))),
        ],
        mark_dead=lambda name, exc: seen.append((name, exc)),
    )
    with pytest.raises(TTSUnavailable):
        chain.synthesize("hi", None, None)
    assert [n for n, _ in seen] == ["yandex", "minimax", "silero"]
    # Each mark_dead call gets the matching exception instance
    for name, exc in seen:
        assert isinstance(exc, Timeout)


def test_dead_cache_all_skipped_raises_tts_unavailable() -> None:
    """If every provider is in dead cache → TTSUnavailable."""
    chain = TTSProviderChain(
        [
            _slot("yandex", 1, fn=MagicMock(return_value=b"x")),
            _slot("minimax", 2, fn=MagicMock(return_value=b"x")),
        ],
        dead_cache=lambda _name: True,
    )
    with pytest.raises(TTSUnavailable):
        chain.synthesize("hi", None, None)


# ── Empty / malformed chain ────────────────────────────────────────────────


def test_empty_chain_rejected() -> None:
    with pytest.raises(ValueError, match="at least one provider"):
        TTSProviderChain([])


def test_slot_validates_name_and_priority() -> None:
    with pytest.raises(ValueError, match="name must be non-empty"):
        TTSProviderSlot(name="", synthesize=lambda t, s, v: b"x", priority=1)
    with pytest.raises(ValueError, match="priority must be >= 1"):
        TTSProviderSlot(name="yandex", synthesize=lambda t, s, v: b"x", priority=0)


# ── YAML deserialisation ────────────────────────────────────────────────────


def test_chain_from_yaml_basic() -> None:
    cfg = [
        {"provider": "yandex", "voice": "anton", "priority": 1},
        {"provider": "minimax", "voice": "male-qn-qingse", "priority": 2},
        {"provider": "silero", "voice": "aidar", "priority": 3},
    ]

    def lookup(name: str):
        return lambda t, s, v: f"{name}-audio".encode()

    chain = chain_from_yaml_config(cfg, lookup)
    assert chain.names == ("yandex", "minimax", "silero")
    assert [s.voice for s in chain.providers] == ["anton", "male-qn-qingse", "aidar"]


def test_chain_from_yaml_accepts_no_voice() -> None:
    cfg = [
        {"provider": "yandex", "priority": 1},
        {"provider": "minimax", "priority": 2},
    ]

    def lookup(name: str):
        return lambda t, s, v: f"{name}".encode()

    chain = chain_from_yaml_config(cfg, lookup)
    assert chain.names == ("yandex", "minimax")


def test_chain_from_yaml_rejects_unknown_provider() -> None:
    cfg = [{"provider": "elevenlabs", "priority": 1}]
    with pytest.raises(ValueError, match="unknown provider 'elevenlabs'"):
        chain_from_yaml_config(cfg, lambda n: lambda t, s, v: None)


def test_chain_from_yaml_rejects_missing_priority() -> None:
    cfg = [{"provider": "yandex"}]
    with pytest.raises(ValueError, match="'priority' must be int"):
        chain_from_yaml_config(cfg, lambda n: lambda t, s, v: None)


def test_chain_from_yaml_rejects_duplicate_priority() -> None:
    cfg = [
        {"provider": "yandex", "priority": 1},
        {"provider": "minimax", "priority": 1},
    ]
    with pytest.raises(ValueError, match="duplicate priority 1"):
        chain_from_yaml_config(cfg, lambda n: lambda t, s, v: None)


def test_chain_from_yaml_rejects_missing_provider() -> None:
    cfg = [{"voice": "anton", "priority": 1}]
    with pytest.raises(ValueError, match="missing 'provider'"):
        chain_from_yaml_config(cfg, lambda n: lambda t, s, v: None)


# ── Voice propagation ──────────────────────────────────────────────────────


def test_caller_voice_overrides_slot_voice() -> None:
    """Caller voice > slot.voice (per-side override)."""
    captured: list[tuple[str, str | None]] = []

    def fn(name):
        def inner(t, s, v):
            captured.append((name, v))
            return b"x"
        return inner

    chain = TTSProviderChain(
        [
            TTSProviderSlot("yandex", fn("yandex"), priority=1, voice="slot-anton"),
        ]
    )
    chain.synthesize("hi", None, voice="caller-ermil")
    assert captured == [("yandex", "caller-ermil")]


def test_slot_voice_used_when_caller_passes_none() -> None:
    captured: list[str | None] = []

    def fn(name):
        def inner(t, s, v):
            captured.append(v)
            return b"x"
        return inner

    chain = TTSProviderChain(
        [
            TTSProviderSlot("yandex", fn("yandex"), priority=1, voice="slot-anton"),
        ]
    )
    chain.synthesize("hi", None, None)
    assert captured == ["slot-anton"]