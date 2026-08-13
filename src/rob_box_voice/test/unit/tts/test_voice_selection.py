"""Unit tests for tts_node voice selection (issue #1219).

Covers the LLM-voice plumbing in ``tts_node``:

* ``dialogue_callback`` reads ``voice`` from the request payload and
  forwards it through ``_submit_synthesis`` → worker → ``_synthesize_and_play``
  (as keyword, preserving the canonical positional arity);
* provider branches resolve the requested voice against the ACTUAL provider's
  voice list (fallback chain) and fall back to the provider default (Q6/Q11);
* unknown voice → provider default + warn + voice_used in logs.

Uses the ``conftest.py`` stubs (no rclpy/grpc/torch).
"""

from __future__ import annotations

import sys
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


class _Bare:
    """Minimal node stub: attributes only (contract for stubs)."""


def _playback_node() -> _Bare:
    """Node stub with everything _synthesize_and_play touches after synth."""
    node = _Bare()
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
    # Silero fallback prerequisites
    node._silero_loaded = __import__("threading").Event()
    node._silero_loaded.set()
    node.silero_model = MagicMock(name="FakeSileroModel")
    node.silero_sample_rate = 48000
    node.silero_put_stress_homo = True
    node.silero_speaker = "aidar"
    node._synthesize_silero = MagicMock(return_value=np.zeros(4800, dtype=np.float32))
    node.yandex_stub = object()
    node.yandex_voice = "anton"
    node.chunk_max_chars_yandex = 250
    node.chunk_max_chars_silero = 800
    node.chunk_min_chars = 50
    node.chunk_max_retries = 3
    return node


def _run_with_voice(node, voice=None) -> None:
    TTSNode._synthesize_and_play(
        node,
        "<speak>hello</speak>",
        "hello",
        None,
        {},
        None,
        None,
        None,
        None,
        voice=voice,
    )


# ── Voice resolution per actual provider (Q6/Q11) ─────────────────────────────


def test_minimax_known_voice_passed_through() -> None:
    """speak_text(voice=female-shaonv) → MiniMax синтезирует этим голосом."""
    node = _playback_node()
    _run_with_voice(node, voice="female-shaonv")
    node._synthesize_minimax.assert_called_once()
    _, kwargs = node._synthesize_minimax.call_args
    assert kwargs.get("voice") == "female-shaonv"


def test_minimax_default_when_no_voice() -> None:
    """Без voice → дефолт MiniMax (male-qn-qingse) уходит в провайдер."""
    node = _playback_node()
    _run_with_voice(node, voice=None)
    _, kwargs = node._synthesize_minimax.call_args
    assert kwargs.get("voice") == "male-qn-qingse"


def test_minimax_unknown_voice_falls_back_to_default() -> None:
    """Голос Yandex (alena) недоступен у MiniMax → дефолт MiniMax + warn."""
    node = _playback_node()
    _run_with_voice(node, voice="alena")
    _, kwargs = node._synthesize_minimax.call_args
    assert kwargs.get("voice") == "male-qn-qingse"
    warn_msgs = " ".join(str(c) for c in node.logger.warn.call_args_list)
    assert "недоступен у MiniMax" in warn_msgs


def test_yandex_known_voice_passed_through() -> None:
    """provider=yandex + speak_text(voice=zahar) → yandex синтезирует zahar."""
    node = _playback_node()
    node.provider = "yandex"
    node.provider_chain = ["yandex", "silero"]
    node._synthesize_yandex = MagicMock(return_value=np.zeros(2205, dtype=np.float32))
    node._synthesize_minimax = MagicMock()  # not called for yandex chain
    _run_with_voice(node, voice="zahar")
    node._synthesize_yandex.assert_called_once()
    _, kwargs = node._synthesize_yandex.call_args
    assert kwargs.get("voice") == "zahar"


def test_yandex_unknown_voice_falls_back_to_anton() -> None:
    node = _playback_node()
    node.provider = "yandex"
    node.provider_chain = ["yandex", "silero"]
    node._synthesize_yandex = MagicMock(return_value=np.zeros(2205, dtype=np.float32))
    node._synthesize_minimax = MagicMock()
    _run_with_voice(node, voice="terminator")
    _, kwargs = node._synthesize_yandex.call_args
    assert kwargs.get("voice") == "anton"
    warn_msgs = " ".join(str(c) for c in node.logger.warn.call_args_list)
    assert "недоступен у Yandex" in warn_msgs


def test_silero_known_voice_passed_through() -> None:
    """Фолбек на Silero: голос baya уходит как speaker в _synthesize_silero."""
    node = _playback_node()
    node.provider = "minimax"
    node.provider_chain = ["minimax", "yandex", "silero"]
    node._synthesize_minimax = MagicMock(
        side_effect=RuntimeError("MiniMax dead")
    )
    node._synthesize_yandex = MagicMock(side_effect=RuntimeError("Yandex dead"))
    _run_with_voice(node, voice="baya")
    node._synthesize_silero.assert_called_once()
    _, kwargs = node._synthesize_silero.call_args
    assert kwargs.get("voice") == "baya"


def test_silero_unknown_voice_falls_back_to_default() -> None:
    node = _playback_node()
    node.provider = "minimax"
    node.provider_chain = ["minimax", "yandex", "silero"]
    node._synthesize_minimax = MagicMock(side_effect=RuntimeError("MiniMax dead"))
    node._synthesize_yandex = MagicMock(side_effect=RuntimeError("Yandex dead"))
    _run_with_voice(node, voice="alena")  # yandex voice — not a silero speaker
    _, kwargs = node._synthesize_silero.call_args
    assert kwargs.get("voice") == "aidar"  # default silero


# ── dialogue_callback → submit chain (voice as keyword) ───────────────────────


def test_voice_forwarded_via_kwargs_not_positional() -> None:
    """``voice`` должен идти через kwargs, а не позиционно (test_speech_id_arg_chain)."""
    node = _playback_node()
    # Replay: dialogue_callback → _submit_synthesis → worker → _synthesize_and_play.
    # Используем AST-контракт: _submit_synthesis принимает **kwargs и
    # прокидывает их в executor.submit. Здесь проверяем, что worker
    # достаёт voice из kwargs и передаёт в _synthesize_and_play.
    worker_kwargs = {"play_seq": 1, "voice": "zahar"}
    node._synthesize_and_play = MagicMock()
    TTSNode._run_synthesis_worker.__get__(node, type(node))(
        "<speak>x</speak>", "x", None, {}, "sid1", "bid1", 1, 1, **worker_kwargs
    )
    node._synthesize_and_play.assert_called_once()
    _, kwargs = node._synthesize_and_play.call_args
    assert kwargs.get("voice") == "zahar"
