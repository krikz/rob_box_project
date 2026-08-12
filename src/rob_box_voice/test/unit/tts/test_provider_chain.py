"""Unit tests for TTS provider fallback chain (issue #1083).

Covers the kanban task ``t_424e8172``:

* ``minimax → yandex → silero`` priority chain (config-driven via
  ``provider_chain`` ROS parameter, derived from ``provider`` otherwise);
* failover on provider error — MiniMax 2056 quota goes to Yandex (NOT
  straight to Silero), Yandex gRPC failure goes to Silero;
* dead-provider cache with TTL (long for quota/auth, short for transient),
  so a dead MiniMax isn't hammered on every turn;
* ``_synthesize_and_play`` end-to-end chain walk with fake provider
  failures (acceptance: fake failure of each provider → correct next
  in chain).

Tests use the ``conftest.py`` rclpy/grpc/torch stubs so the module can be
imported without the heavy ROS2 stack; node stubs are bare classes with
just the attributes the hot path reads (same contract as
``test_minimax_integration._make_fake_node``).
"""
from __future__ import annotations

import sys
import threading
import time
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
import pytest

_PACKAGE_ROOT = Path(__file__).resolve().parents[3]  # rob_box_voice/
sys.path.insert(0, str(_PACKAGE_ROOT))

from test.unit.tts.conftest import _install_all_mocks  # noqa: E402

_install_all_mocks()

from rob_box_voice import tts_node  # noqa: E402
from rob_box_voice.tts_node import TTSNode  # noqa: E402

# Real MiniMax error classes (imported by tts_node when rob_box_llm is
# available). Used to verify long-TTL classification for quota errors.
MiniMaxTTSAuthError = tts_node.MiniMaxTTSAuthError


# ── Pure chain helpers (no node needed) ──────────────────────────────────────


def test_default_provider_chain_is_minimax_yandex_silero() -> None:
    assert TTSNode._default_provider_chain() == ["minimax", "yandex", "silero"]


def test_chain_from_provider_minimax() -> None:
    # The core fix for #1083: provider=minimax must fall back to Yandex
    # (not Silero) when MiniMax dies.
    assert TTSNode._chain_from_provider("minimax") == ["minimax", "yandex", "silero"]


def test_chain_from_provider_yandex_backcompat() -> None:
    # Back-compat: provider=yandex stays yandex → silero (no MiniMax).
    assert TTSNode._chain_from_provider("yandex") == ["yandex", "silero"]


def test_chain_from_provider_silero_only() -> None:
    assert TTSNode._chain_from_provider("silero") == ["silero"]


@pytest.mark.parametrize(
    "chain,expected",
    [
        (["minimax", "yandex", "silero"], ["minimax", "yandex", "silero"]),
        # Silero mid-chain is moved to the end (invariant: always last).
        (["silero", "minimax", "yandex"], ["minimax", "yandex", "silero"]),
        # Unknown providers are dropped.
        (["bogus", "minimax", "nope"], ["minimax", "silero"]),
        # Duplicates are removed.
        (["minimax", "minimax", "yandex"], ["minimax", "yandex", "silero"]),
        # Missing silero → appended at the end.
        (["yandex"], ["yandex", "silero"]),
        # Empty chain → default full chain.
        ([], ["minimax", "yandex", "silero"]),
        # Explicit silero-only is legit (provider=silero mode).
        (["silero"], ["silero"]),
    ],
)
def test_normalize_provider_chain(chain: list[str], expected: list[str]) -> None:
    assert TTSNode._normalize_provider_chain(chain) == expected


def test_effective_provider_chain_uses_attribute_first() -> None:
    node = _bare_node()
    node.provider_chain = ["yandex", "silero"]
    node.provider = "minimax"  # must be ignored — attribute wins
    assert TTSNode._effective_provider_chain(node) == ["yandex", "silero"]


def test_effective_provider_chain_falls_back_to_provider() -> None:
    node = _bare_node()
    node.provider = "yandex"
    assert TTSNode._effective_provider_chain(node) == ["yandex", "silero"]


# ── Dead-provider cache with TTL ─────────────────────────────────────────────


class _Bare:
    """Minimal node stub: attributes only, no methods (contract for stubs)."""


def _bare_node() -> _Bare:
    return _Bare()


def _bind_dead_cache(node: _Bare) -> _Bare:
    """Attach the real dead-cache methods to a bare stub."""
    node.provider_dead_ttl_s = 300.0
    node.provider_dead_ttl_transient_s = 30.0
    node._provider_dead_until = {}
    node._provider_dead_reason = {}
    node.get_logger = MagicMock()
    node._provider_is_dead = TTSNode._provider_is_dead.__get__(node, type(node))  # type: ignore[attr-defined]
    node._mark_provider_dead = TTSNode._mark_provider_dead.__get__(node, type(node))  # type: ignore[attr-defined]
    node._provider_dead_until_s = TTSNode._provider_dead_until_s.__get__(node, type(node))  # type: ignore[attr-defined]
    return node


def test_dead_cache_quota_error_uses_long_ttl() -> None:
    node = _bind_dead_cache(_bare_node())
    # 2056 Token Plan limit — реальный класс ошибки квоты (TTSAuthError).
    TTSNode._mark_provider_dead(
        node,
        "minimax",
        MiniMaxTTSAuthError(
            "MiniMax auth error, NO retry: 2056 Token Plan usage limit reached",
            provider="minimax",
        ),
    )
    # Long TTL (default 300 s) for a quota error.
    assert node._provider_dead_until["minimax"] > time.monotonic() + 250
    assert TTSNode._provider_is_dead(node, "minimax") is True


def test_dead_cache_expires_after_ttl() -> None:
    node = _bind_dead_cache(_bare_node())
    TTSNode._mark_provider_dead(node, "minimax", RuntimeError("boom"), ttl_s=0.01)
    assert TTSNode._provider_is_dead(node, "minimax") is True
    time.sleep(0.02)
    assert TTSNode._provider_is_dead(node, "minimax") is False


def test_dead_cache_skip_logs_remaining_seconds() -> None:
    node = _bind_dead_cache(_bare_node())
    TTSNode._mark_provider_dead(node, "minimax", RuntimeError("boom"), ttl_s=60.0)
    remaining = TTSNode._provider_dead_until_s(node, "minimax")
    assert 0.0 < remaining <= 60.0


# ── _synthesize_and_play chain walk (fake provider failures) ─────────────────


def _playback_node() -> _Bare:
    """Node stub with everything _synthesize_and_play touches after synth."""
    node = _bare_node()
    node.provider = "minimax"
    node.normalize_text = False
    node.stop_requested = False
    node.processing_dialogue_id = None
    node.current_dialogue_id = None
    node.minimax_streaming = False
    node.minimax_model = "speech-02-hd"
    node.minimax_voice = "male-qn-qingse"
    node._synthesize_minimax = MagicMock(
        return_value={"audio_np": np.zeros(800, dtype=np.float32), "sample_rate": 32000}
    )
    node.publish_state = MagicMock()
    node._publish_audio = MagicMock()
    # Shared logger: ``self.get_logger()`` must return the SAME mock every
    # call so tests can inspect warn/info/error via ``node.logger.*``.
    node.logger = MagicMock()
    node.get_logger = lambda: node.logger
    node.chipmunk_mode = False
    node.pitch_shift = 1.0
    node.volume_gain = 1.0
    node.device_index = None
    node.current_stream = None
    node.playback_manager = MagicMock()
    node.playback_manager.play_audio.return_value = True
    node.cleanup_playback_noise = MagicMock()
    node.finished_pub = MagicMock()
    node.audio_output_sample_rate = 16000
    node._prepare_audio_for_topic = TTSNode._prepare_audio_for_topic.__get__(node, type(node))  # type: ignore[attr-defined]

    # Silero fallback prerequisites: warm-load event already set, a fake
    # model, and a stub _synthesize_silero returning real float32 audio.
    node._silero_loaded = threading.Event()
    node._silero_loaded.set()
    node.silero_model = MagicMock(name="FakeSileroModel")
    node.silero_sample_rate = 48000
    node.silero_put_stress_homo = True
    node._synthesize_silero = MagicMock(
        return_value=np.zeros(4800, dtype=np.float32)
    )
    return node


def _run_and_play(node) -> None:
    TTSNode._synthesize_and_play(
        node,
        "<speak>hello</speak>",
        "hello",
        None,
        {},
        None,
    )


def test_chain_minimax_alive_first_priority() -> None:
    """All three alive → MiniMax is used (first priority)."""
    node = _playback_node()
    node.yandex_stub = object()
    node._synthesize_yandex = MagicMock(return_value=np.zeros(2205, dtype=np.float32))
    node.minimax_streaming = False
    node._synthesize_minimax = MagicMock(
        return_value={"audio_np": np.zeros(800, dtype=np.float32), "sample_rate": 32000}
    )

    _run_and_play(node)

    node._synthesize_minimax.assert_called_once()
    node._synthesize_yandex.assert_not_called()
    node._synthesize_silero.assert_not_called()
    node._publish_audio.assert_called_once()


def test_chain_minimax_quota_2056_goes_to_yandex_not_silero() -> None:
    """Acceptance: MiniMax 2056 quota → Yandex (NOT Silero)."""
    node = _playback_node()
    node.yandex_stub = object()
    node._synthesize_yandex = MagicMock(return_value=np.zeros(2205, dtype=np.float32))
    node.minimax_streaming = False
    node._synthesize_minimax = MagicMock(
        side_effect=RuntimeError("MiniMax auth error, NO retry: 2056 Token Plan usage limit reached")
    )

    _run_and_play(node)

    node._synthesize_minimax.assert_called_once()
    node._synthesize_yandex.assert_called_once()
    node._synthesize_silero.assert_not_called()
    node._publish_audio.assert_called_once()
    # Honest log: the switch to Yandex is visible in docker logs.
    warn_msgs = " ".join(str(c) for c in node.logger.warn.call_args_list)
    assert "переключаюсь на Yandex" in warn_msgs


def test_chain_minimax_and_yandex_dead_goes_to_silero() -> None:
    """Acceptance: MiniMax+Yandex dead → Silero, robot still answers."""
    node = _playback_node()
    node.yandex_stub = object()
    node._synthesize_yandex = MagicMock(
        side_effect=RuntimeError("Yandex gRPC отвалился: simulated")
    )
    node.minimax_streaming = False
    node._synthesize_minimax = MagicMock(
        side_effect=RuntimeError("MiniMax auth error, NO retry: 2056 Token Plan usage limit reached")
    )

    _run_and_play(node)

    node._synthesize_minimax.assert_called_once()
    node._synthesize_yandex.assert_called_once()
    node._synthesize_silero.assert_called_once()
    node._publish_audio.assert_called_once()


def test_chain_dead_cache_skips_minimax() -> None:
    """Dead cache: MiniMax marked dead → skipped on the next turn."""
    node = _playback_node()
    _bind_dead_cache(node)
    node.yandex_stub = object()
    node._synthesize_yandex = MagicMock(return_value=np.zeros(2205, dtype=np.float32))
    node.minimax_streaming = False
    node._synthesize_minimax = MagicMock(
        side_effect=RuntimeError("MiniMax auth error, NO retry: 2056 Token Plan usage limit reached")
    )

    # First turn: MiniMax fails → marked dead → Yandex answers.
    _run_and_play(node)
    assert TTSNode._provider_is_dead(node, "minimax") is True

    # Second turn: MiniMax is in the dead cache → skipped, Yandex again,
    # MiniMax's synthesize is NOT called again (no hammering).
    calls_after_first = node._synthesize_minimax.call_count
    _run_and_play(node)
    assert node._synthesize_minimax.call_count == calls_after_first, (
        "dead-cached MiniMax must not be called again until TTL expiry"
    )
    assert node._synthesize_yandex.call_count == 2


def test_chain_provider_yandex_backcompat_skips_minimax() -> None:
    """provider=yandex → chain [yandex, silero]; MiniMax never called."""
    node = _playback_node()
    node.provider = "yandex"
    node.yandex_stub = object()
    node._synthesize_yandex = MagicMock(return_value=np.zeros(2205, dtype=np.float32))
    node.minimax_streaming = False
    node._synthesize_minimax = MagicMock(
        return_value={"audio_np": np.zeros(800, dtype=np.float32), "sample_rate": 32000}
    )

    _run_and_play(node)

    node._synthesize_minimax.assert_not_called()
    node._synthesize_yandex.assert_called_once()
    node._publish_audio.assert_called_once()


def test_chain_ttl_expiry_retries_minimax_first() -> None:
    """Edge: MiniMax recovers (TTL expired) → tried first again."""
    node = _playback_node()
    _bind_dead_cache(node)
    node.yandex_stub = object()
    node._synthesize_yandex = MagicMock(return_value=np.zeros(2205, dtype=np.float32))
    node.minimax_streaming = False
    node._synthesize_minimax = MagicMock(
        return_value={"audio_np": np.zeros(800, dtype=np.float32), "sample_rate": 32000}
    )

    # Mark MiniMax dead with a very short TTL, then let it expire.
    TTSNode._mark_provider_dead(node, "minimax", RuntimeError("quota"), ttl_s=0.01)
    assert TTSNode._provider_is_dead(node, "minimax") is True
    time.sleep(0.02)

    _run_and_play(node)

    # MiniMax is tried first again (and succeeds).
    node._synthesize_minimax.assert_called_once()
    node._synthesize_yandex.assert_not_called()
