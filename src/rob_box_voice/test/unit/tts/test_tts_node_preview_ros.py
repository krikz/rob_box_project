"""ADR-0077 / issue #2138.A.3 — tests for preview-channel in tts_node.

Тесты НЕ покрывают сам синтез (он в test_tts_node_preview.py) — здесь
только ROS-обвязка:

* ``_on_avatar_tts_request(sink="preview")`` НЕ идёт в ThreadPoolExecutor
  и НЕ публикует в /avatar/tts/audio или /voice/audio/speech.
* ``_on_avatar_tts_request(sink="preview")`` ПУБЛИКУЕТ в
  /avatar/preview_voice/audio (JSON+base64) и /avatar/preview_voice/result.
* При ошибке провайдера — /avatar/preview_voice/error с reason.
* ``sink="headset"`` НЕ затронут (старое поведение).
* ``sink="bogus"`` → /avatar/tts/error с error="invalid_sink".
"""

from __future__ import annotations

import base64
import json
import sys
import threading
from pathlib import Path
from unittest.mock import MagicMock

import pytest

_PACKAGE_ROOT = Path(__file__).resolve().parents[3]  # rob_box_voice/
sys.path.insert(0, str(_PACKAGE_ROOT))

from test.unit.tts.conftest import _install_all_mocks  # noqa: E402

_install_all_mocks()

from rob_box_voice import tts_node as _tts_node_mod  # noqa: E402
from rob_box_voice.tts_node import (  # noqa: E402
    PreviewSynthesisError,
    TTSNode,
)
from rob_box_llm import TTSAudio, TTSFormat  # noqa: E402

del _tts_node_mod


class _CapturingPublisher:
    """Минимальный mock publisher — record.publish() и .published список."""

    def __init__(self):
        self.published: list = []

    def publish(self, msg) -> None:
        self.published.append(msg)


def _bare_node_with_preview(mocks: dict) -> TTSNode:
    """Собрать TTSNode через object.__new__ + минимальные publishers/attrs."""
    n = object.__new__(TTSNode)
    n.get_logger = lambda: MagicMock()
    # Publishers из объявления в __init__ (мы их не зовём, а создаём сами
    # чтобы не инициализировать весь rclpy).
    n._preview_audio_pub = mocks.get("preview_audio_pub", _CapturingPublisher())
    n._preview_result_pub = mocks.get("preview_result_pub", _CapturingPublisher())
    n._preview_error_pub = mocks.get("preview_error_pub", _CapturingPublisher())
    n._avatar_audio_pub = mocks.get("avatar_audio_pub", _CapturingPublisher())
    n.audio_pub = mocks.get("audio_pub", _CapturingPublisher())
    n._avatar_tts_error_pub = mocks.get("avatar_tts_error_pub", _CapturingPublisher())
    # Параметры preview.
    n.minimax_voice = "default-active-voice"
    n.minimax_model = "speech-02-hd"
    n.minimax_language = "ru"
    n.preview_format = TTSFormat.MP3
    # Preview НЕ идёт в FIFO/ALSA/metrics — ставим no-op.
    n._play_order_cond = threading.Condition()
    n.play_audio = MagicMock()
    return n


def _fake_provider_factory(audio_bytes: bytes, fmt=TTSFormat.MP3, sr=24000):
    """Поддельный TTSProvider для synthesize_preview."""

    async def _synthesize(text, *, settings=None):
        return TTSAudio(samples=audio_bytes, sample_rate=sr, format=fmt)

    p = MagicMock()
    p.synthesize = _synthesize
    p.name = "fake"
    return p


# ── test 1: sink="preview" ветка не трогает headset-каналы ──────────────


def test_sink_preview_publishes_audio_and_result_but_not_headset_or_speaker():
    """sink="preview" → публикует только в preview_voice_*, НЕ трогает
    /avatar/tts/audio (headset) и /voice/audio/speech (speaker)."""
    mocks = {
        "preview_audio_pub": _CapturingPublisher(),
        "preview_result_pub": _CapturingPublisher(),
        "preview_error_pub": _CapturingPublisher(),
        "avatar_audio_pub": _CapturingPublisher(),
        "audio_pub": _CapturingPublisher(),
    }
    node = _bare_node_with_preview(mocks)
    # Подменяем synthesize_preview на фейк через mock на уровне класса.
    fake_provider = _fake_provider_factory(b"\x00" * 32)
    original_ensure = node._ensure_minimax_provider
    node._ensure_minimax_provider = lambda: fake_provider  # type: ignore[assignment]

    msg = MagicMock()
    msg.data = json.dumps(
        {
            "request_id": "abc12345",
            "sink": "preview",
            "ssml": "<speak>привет</speak>",
            "voice": "male-qn-qingse",
        },
        ensure_ascii=False,
    )
    node._on_avatar_tts_request(msg)

    audio_msgs = mocks["preview_audio_pub"].published
    result_msgs = mocks["preview_result_pub"].published
    error_msgs = mocks["preview_error_pub"].published
    avatar_audio_msgs = mocks["avatar_audio_pub"].published
    speaker_msgs = mocks["audio_pub"].published

    assert len(audio_msgs) == 1, f"audio: {audio_msgs}"
    assert len(result_msgs) == 1, f"result: {result_msgs}"
    assert not error_msgs, f"error (не должно быть): {error_msgs}"
    assert not avatar_audio_msgs, "headset /avatar/tts/audio НЕ должен был публиковаться"
    assert not speaker_msgs, "speaker /voice/audio/speech НЕ должен был публиковаться"

    # Декодируем payload audio.
    audio_payload = json.loads(audio_msgs[0].data)
    assert audio_payload["request_id"] == "abc12345"
    assert audio_payload["format"] == "mp3"
    assert audio_payload["content_type"] == "audio/mpeg"
    decoded = base64.b64decode(audio_payload["audio_b64"])
    assert decoded == b"\x00" * 32
    # Result тоже с request_id.
    result_payload = json.loads(result_msgs[0].data)
    assert result_payload["request_id"] == "abc12345"

    # Восстановим ensure для последующих тестов (на всякий случай).
    del original_ensure


# ── test 2: ошибка синтеза → preview_error с reason ────────────────────


def test_sink_preview_provider_error_publishes_preview_error_with_reason():
    """PreviewSynthesisError → /avatar/preview_voice/error с reason."""
    mocks = {
        "preview_audio_pub": _CapturingPublisher(),
        "preview_result_pub": _CapturingPublisher(),
        "preview_error_pub": _CapturingPublisher(),
    }
    node = _bare_node_with_preview(mocks)

    # synthesize_preview бросает с нашим reason.
    from rob_box_voice.tts_node import PreviewSynthesisError

    def _raise(*a, **kw):
        raise PreviewSynthesisError(
            "fake provider boom", reason="minimax_rate_limited"
        )

    node.synthesize_preview = _raise  # type: ignore[assignment]

    msg = MagicMock()
    msg.data = json.dumps(
        {
            "request_id": "fail-test",
            "sink": "preview",
            "ssml": "<speak>привет</speak>",
            "voice": "male-qn-qingse",
        },
        ensure_ascii=False,
    )
    node._on_avatar_tts_request(msg)

    err = mocks["preview_error_pub"].published
    assert len(err) == 1, f"error: {err}"
    payload = json.loads(err[0].data)
    assert payload["request_id"] == "fail-test"
    assert payload["reason"] == "minimax_rate_limited"
    assert not mocks["preview_audio_pub"].published
    assert not mocks["preview_result_pub"].published


# ── test 3: пустой text → empty_text preview_error, без синтеза ────────


def test_sink_preview_empty_text_publishes_empty_text_error():
    """sink="preview" + пустой ssml/text → preview_error empty_text."""
    mocks = {
        "preview_audio_pub": _CapturingPublisher(),
        "preview_result_pub": _CapturingPublisher(),
        "preview_error_pub": _CapturingPublisher(),
    }
    node = _bare_node_with_preview(mocks)
    # synthesize_preview НЕ должен вызываться.
    node.synthesize_preview = MagicMock()  # type: ignore[assignment]

    msg = MagicMock()
    msg.data = json.dumps(
        {"request_id": "empty-test", "sink": "preview", "ssml": "<speak>   </speak>"},
        ensure_ascii=False,
    )
    node._on_avatar_tts_request(msg)

    err = mocks["preview_error_pub"].published
    assert len(err) == 1
    payload = json.loads(err[0].data)
    assert payload["request_id"] == "empty-test"
    assert payload["reason"] == "empty_text"
    node.synthesize_preview.assert_not_called()


# ── test 4: sink="headset" НЕ затронут (старая ветка) ──────────────────


def test_sink_headset_does_not_touch_preview_publishers():
    """sink="headset" идёт в старый путь, preview_* publishers НЕ трогает."""
    mocks = {
        "preview_audio_pub": _CapturingPublisher(),
        "preview_result_pub": _CapturingPublisher(),
        "preview_error_pub": _CapturingPublisher(),
        "avatar_audio_pub": _CapturingPublisher(),
    }
    node = _bare_node_with_preview(mocks)
    # Помечаем headset-путь: synthesize_preview не должен вызываться.
    node.synthesize_preview = MagicMock()  # type: ignore[assignment]
    # Чтобы старый путь headset не падал на _submit_synthesis и т.п.,
    # мокаем все внешние зависимости no-op.
    node._submit_synthesis = MagicMock()  # type: ignore[assignment]
    node._parse_ssml_attributes = lambda ssml: {}  # type: ignore[assignment]
    node._extract_text_from_ssml = lambda ssml: "привет"  # type: ignore[assignment]
    node._publish_tars1_text = MagicMock()  # type: ignore[assignment]

    msg = MagicMock()
    msg.data = json.dumps(
        {
            "request_id": "headset-test",
            "sink": "headset",
            "ssml": "<speak>привет</speak>",
            "voice": "default",
        },
        ensure_ascii=False,
    )
    node._on_avatar_tts_request(msg)

    # Preview НЕ должен ничего публиковать.
    assert not mocks["preview_audio_pub"].published
    assert not mocks["preview_result_pub"].published
    assert not mocks["preview_error_pub"].published
    # synthesize_preview не вызывался.
    node.synthesize_preview.assert_not_called()


# ── test 5: невалидный sink → /avatar/tts/error с error="invalid_sink" ─


def test_invalid_sink_publishes_avatar_tts_error_invalid_sink():
    """sink="bogus" → /avatar/tts/error (НЕ preview_error)."""
    mocks = {
        "preview_audio_pub": _CapturingPublisher(),
        "preview_result_pub": _CapturingPublisher(),
        "preview_error_pub": _CapturingPublisher(),
        "avatar_tts_error_pub": _CapturingPublisher(),
    }
    node = _bare_node_with_preview(mocks)
    node.synthesize_preview = MagicMock()  # type: ignore[assignment]

    msg = MagicMock()
    msg.data = json.dumps(
        {"request_id": "bogus-test", "sink": "bogus", "ssml": "<speak>x</speak>"},
        ensure_ascii=False,
    )
    node._on_avatar_tts_request(msg)

    err = mocks["avatar_tts_error_pub"].published
    assert len(err) == 1
    payload = json.loads(err[0].data)
    assert payload["request_id"] == "bogus-test"
    assert payload["error"] == "invalid_sink"
    # Preview publishers НЕ тронуты.
    assert not mocks["preview_audio_pub"].published
    assert not mocks["preview_result_pub"].published
    assert not mocks["preview_error_pub"].published