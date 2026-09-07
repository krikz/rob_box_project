"""Unit-тесты обратного канала ТАРС в шлем (ADR-0055, issue #1993).

Покрывает commit C2 из impl-плана 0055:

* ``_on_avatar_tts_request`` с ``sink != "headset"`` → ``/avatar/tts/error``
  с ``error="invalid_sink"`` + DROP, ``_submit_synthesis`` НЕ вызван.
* ``_on_avatar_tts_request`` с ``sink == "headset"`` → ``_submit_synthesis``
  с ``sink="headset"`` (worker берёт headset-путь).
* ``_publish_headset_audio`` шлёт AudioData в ``headset_audio_topic``,
  формат — int16 LE PCM (тот же, что ``/voice/audio/speech``).
* Управляющий топик ``/avatar/tts/control`` идёт через существующий
  ``control_callback`` (тот же формат команды) — регресс: STOP сбрасывает
  ``_avatar_tts_request_id``.
* Параметры ноды: ``headset_audio_topic`` / ``avatar_request_topic`` /
  ``avatar_error_topic`` / ``avatar_control_topic`` объявлены в ``__init__``.

Запуск:
    PYTHONPATH=src/rob_box_voice:src/rob_box_core:src/rob_box_harness:src/rob_box_llm \\
        pytest src/rob_box_voice/test/unit/tts/test_tts_node_avatar.py -v
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
import pytest

_PACKAGE_ROOT = Path(__file__).resolve().parents[3]  # rob_box_voice/
sys.path.insert(0, str(_PACKAGE_ROOT))

from test.unit.tts.conftest import _install_all_mocks  # noqa: E402

_install_all_mocks()

from rob_box_voice import tts_node as _tts_node_mod  # noqa: E402
from rob_box_voice.tts_node import TTSNode  # noqa: E402


# ── helpers ────────────────────────────────────────────────────────────


class _CapturingPublisher:
    """Минимальный ловец String/AudioData паблишей."""

    def __init__(self):
        self.messages: list = []

    def publish(self, msg) -> None:
        self.messages.append(msg)


class _CapturingAudioPub:
    """AudioData-паблишер (msg.data == list[int])."""

    def __init__(self):
        self.messages: list = []

    def publish(self, msg) -> None:
        # AudioData.data = list[int]; сохраняем сырой msg для проверки полей.
        self.messages.append(msg)


def _make_request_node():
    """Минимальный node-stub для ``_on_avatar_tts_request``.

    Только то, что нужно callback'у: pub'ы (finished/error), state-machine
    для stale-dialogue guard, метод ``_publish_avatar_tts_error`` (через
    метод реального класса) и ``_submit_synthesis`` (MagicMock).
    """
    n = object.__new__(TTSNode)
    logger = MagicMock()
    n.get_logger = lambda: logger
    n.finished_pub = _CapturingPublisher()
    n.batch_complete_pub = _CapturingPublisher()
    n._avatar_tts_error_pub = _CapturingPublisher()
    n._avatar_tts_request_id = None
    n.current_speech_id = None
    n.current_dialogue_id = None
    n.processing_dialogue_id = None
    n._submit_synthesis = MagicMock()
    # dialogue_id-check / interrupt_playback зависимости.
    n._interrupt_playback = MagicMock()
    return n


def _msg(payload: dict):
    m = MagicMock()
    m.data = json.dumps(payload, ensure_ascii=False)
    return m


def _finished_payloads(node) -> list:
    return [json.loads(m.data) for m in node.finished_pub.messages]


def _error_payloads(node) -> list:
    return [json.loads(m.data) for m in node._avatar_tts_error_pub.messages]


# ── tests ──────────────────────────────────────────────────────────────


def test_avatar_tts_request_invalid_sink_published_error():
    """sink != "headset" → /avatar/tts/error + DROP, без _submit_synthesis."""
    node = _make_request_node()
    node._on_avatar_tts_request(
        _msg({
            "request_id": "req-bad-sink",
            "ssml": "<speak>привет</speak>",
            "text": "привет",
            "sink": "speaker",  # недопустимый
        })
    )
    # Синтез не стартовал.
    node._submit_synthesis.assert_not_called()
    # Ошибка ушла в /avatar/tts/error.
    errs = _error_payloads(node)
    assert errs, "avatar_tts_error_pub получил 0 сообщений"
    assert errs[0]["request_id"] == "req-bad-sink"
    assert errs[0]["error"] == "invalid_sink"


def test_avatar_tts_request_empty_ssml_dropped_with_empty_text_error():
    """Issue #2096 — пустой SSML → DROP + empty_text error (без _submit_synthesis).

    До фикса ``_on_avatar_tts_request`` не имел защиты для пустого SSML/text:
    payload ``<speak></speak>`` уходил в ``_submit_synthesis`` →
    ``_synthesize_and_play`` → ``_synthesize_minimax_with_retry`` →
    MiniMax райзил ``TTSBadRequestError("text is empty")`` → CRITICAL
    в deploy-логе (run #34144712828).

    Теперь: DROP + ``/avatar/tts/error{error="empty_text"}`` +
    ``/voice/tts/finished{success=False, error="empty_text"}``.
    Caller (operator-agent / grip-pipeline) получает сигнал, что синтеза
    не будет, и не зависает в ожидании speech_id.
    """
    node = _make_request_node()
    node._on_avatar_tts_request(
        _msg({
            "request_id": "req-empty-1",
            "speech_id": "sid-empty-1",
            "ssml": "<speak></speak>",  # пустой SSML
            "text": "",
            "sink": "headset",
        })
    )
    # Синтез НЕ стартовал (раньше уходил в MiniMax bad-request).
    node._submit_synthesis.assert_not_called()
    # Avatar-error уведомил caller'а.
    errs = _error_payloads(node)
    assert errs, "avatar_tts_error_pub получил 0 сообщений"
    assert errs[0]["request_id"] == "req-empty-1"
    assert errs[0]["error"] == "empty_text"
    # /voice/tts/finished с success=False + empty_text (чтобы speak_text не висел).
    finished = _finished_payloads(node)
    assert finished, "finished_pub получил 0 сообщений"
    assert finished[0]["success"] is False
    assert finished[0]["error"] == "empty_text"
    assert finished[0]["speech_id"] == "sid-empty-1"


def test_avatar_tts_request_whitespace_only_ssml_dropped():
    """Issue #2096 — SSML из одних пробелов/переносов → DROP.

    ``_extract_text_from_ssml`` нормализует текст через ``strip_markdown().strip()``,
    поэтому ``"   \n  "`` после извлечения даёт пустую строку → тот же
    guard. Это покрывает кейс, когда LLM или grip-pipeline мог прислать
    «пустой по сути» SSML с whitespace.
    """
    node = _make_request_node()
    node._on_avatar_tts_request(
        _msg({
            "request_id": "req-ws-1",
            "speech_id": "sid-ws-1",
            "ssml": "<speak>   \n\t  </speak>",
            "text": "   \n\t  ",
            "sink": "headset",
        })
    )
    node._submit_synthesis.assert_not_called()
    errs = _error_payloads(node)
    assert errs[0]["error"] == "empty_text"


def test_avatar_tts_request_no_sink_defaults_to_invalid():
    """sink отсутствует → тоже DROP (только "headset" допустим)."""
    node = _make_request_node()
    node._on_avatar_tts_request(
        _msg({
            "request_id": "req-no-sink",
            "ssml": "<speak>x</speak>",
            "text": "x",
            # sink не указан
        })
    )
    node._submit_synthesis.assert_not_called()
    errs = _error_payloads(node)
    assert errs[0]["error"] == "invalid_sink"


def test_avatar_tts_request_headset_sink_submits_synthesis():
    """sink == "headset" → _submit_synthesis с sink="headset" kwarg."""
    node = _make_request_node()
    node._on_avatar_tts_request(
        _msg({
            "request_id": "req-headset-1",
            "ssml": "<speak>Готово</speak>",
            "text": "Готово",
            "sink": "headset",
            "voice": "male-qn-qingse",
            "language": "ru",
        })
    )
    # Синтез стартовал.
    assert node._submit_synthesis.call_count == 1
    call_kwargs = node._submit_synthesis.call_args.kwargs
    assert call_kwargs.get("sink") == "headset", (
        f"ожидался sink='headset', got {call_kwargs.get('sink')!r}"
    )
    assert call_kwargs.get("voice") == "male-qn-qingse"
    assert call_kwargs.get("language") == "ru"
    # request_id запомнен в node state.
    assert node._avatar_tts_request_id == "req-headset-1"


def test_avatar_tts_request_emits_finished_with_speech_id_on_unsupported_script():
    """Unicode-script guard (issue 1709) → /avatar/tts/error + finished(Fail)."""
    node = _make_request_node()
    node._on_avatar_tts_request(
        _msg({
            "request_id": "req-cjk",
            "speech_id": "sid-cjk-1",
            "ssml": "<speak>你好世界，加油加油</speak>",
            "text": "你好世界",
            "sink": "headset",
        })
    )
    node._submit_synthesis.assert_not_called()
    errs = _error_payloads(node)
    assert errs[0]["error"] == "unsupported_script"
    finished = _finished_payloads(node)
    assert finished[0]["success"] is False
    assert finished[0]["error"] == "unsupported_script"
    assert finished[0]["speech_id"] == "sid-cjk-1"


def test_avatar_tts_request_stale_dialogue_dropped():
    """Защита от barge-in: чужой dialogue_id → DROP без синтеза."""
    node = _make_request_node()
    node.current_dialogue_id = "dlg-current"
    node._on_avatar_tts_request(
        _msg({
            "request_id": "req-stale",
            "speech_id": "sid-stale",
            "ssml": "<speak>устарело</speak>",
            "text": "устарело",
            "sink": "headset",
            "dialogue_id": "dlg-old",
        })
    )
    node._submit_synthesis.assert_not_called()
    finished = _finished_payloads(node)
    assert finished[0]["success"] is False
    assert finished[0]["error"] == "stale_dialogue"


def test_avatar_tts_request_bad_json_does_not_crash():
    """Битый JSON → warning + DROP, без падения."""
    node = _make_request_node()
    bad = MagicMock()
    bad.data = "not-valid-json{"
    node._on_avatar_tts_request(bad)
    node._submit_synthesis.assert_not_called()


# ── _publish_headset_audio ─────────────────────────────────────────────


def test_publish_headset_audio_sends_int16_le_pcm():
    """``_publish_headset_audio(np.ndarray)`` → AudioData с int16 LE PCM.

    Проверяем формат (длина = 2×samples, int16-диапазон), а не точные
    значения (зависят от rounding np.clip). Контракт важнее конкретных
    байтов — тест фиксирует, что НЕ base64 и НЕ float32.
    """
    n = object.__new__(TTSNode)
    pub = _CapturingAudioPub()
    n._avatar_audio_pub = pub
    n._avatar_audio_pub.publish = pub.publish

    # 1000 семплов @ 16 kHz = 0.0625 с моно.
    audio_np = np.linspace(-0.5, 0.5, 1000, dtype=np.float32)
    TTSNode._publish_headset_audio(n, audio_np)
    assert len(pub.messages) == 1
    msg = pub.messages[0]
    # AudioData.data = list[int] (octets, int16 LE).
    assert isinstance(msg.data, list), f"ожидался list[int], got {type(msg.data)}"
    assert len(msg.data) == len(audio_np) * 2, (
        f"int16 = 2 байта/семпл: ожидалось {len(audio_np) * 2}, got {len(msg.data)}"
    )
    # Каждая пара — int16 LE, абсолютное значение <= 32767.
    import struct as _struct
    for i in range(0, len(msg.data), 2):
        lo, hi = msg.data[i], msg.data[i + 1]
        value = _struct.unpack("<h", bytes([lo, hi]))[0]
        assert -32768 <= value <= 32767, f"out of int16 range: {value}"


def test_publish_headset_audio_int16_clipping():
    """Пики за пределами [-1.0, 1.0] → clip в int16 (защита от overflow)."""
    n = object.__new__(TTSNode)
    pub = _CapturingAudioPub()
    n._avatar_audio_pub = pub

    audio_np = np.array([2.0, -2.0, 1.5, -1.5], dtype=np.float32)
    TTSNode._publish_headset_audio(n, audio_np)
    msg = pub.messages[0]
    # clip(2.0, -1, 1) = 1.0 → 32767 = 0x7FFF
    assert msg.data[0] == 0xFF and msg.data[1] == 0x7F
    # clip(-2.0, -1, 1) = -1.0 → -32767 = 0x8001
    assert msg.data[2] == 0x01 and msg.data[3] == 0x80


# ── _publish_avatar_tts_error ──────────────────────────────────────────


def test_publish_avatar_tts_error_writes_json_with_request_id():
    """``_publish_avatar_tts_error(request_id, error)`` → JSON в error-pub."""
    n = object.__new__(TTSNode)
    n._avatar_tts_error_pub = _CapturingPublisher()
    n.get_logger = lambda: MagicMock()

    TTSNode._publish_avatar_tts_error(n, "req-x", "synthesis_failed")
    msgs = n._avatar_tts_error_pub.messages
    assert len(msgs) == 1
    body = json.loads(msgs[0].data)
    assert body == {"request_id": "req-x", "error": "synthesis_failed"}


# ── параметры ноды (forward-compat тест) ───────────────────────────────


def test_tts_node_declares_avatar_topics_as_parameters():
    """Параметры headset_audio_topic/avatar_request_topic/etc. объявлены в __init__."""
    import inspect

    src = inspect.getsource(TTSNode.__init__)
    assert 'declare_parameter("headset_audio_topic"' in src
    assert 'declare_parameter("avatar_request_topic"' in src
    assert 'declare_parameter("avatar_error_topic"' in src
    assert 'declare_parameter("avatar_control_topic"' in src
    # Дефолты — ровно те, что в ADR-0055 §tts_node.
    assert '"/avatar/tts/audio"' in src
    assert '"/avatar/tts/request"' in src
    assert '"/avatar/tts/error"' in src
    assert '"/avatar/tts/control"' in src


def test_sink_kwarg_in_synthesize_and_play_signature():
    """``_synthesize_and_play(sink=...)`` — kwarg-only в позиционном хвосте.

    Канонический positional arity (test_speech_id_arg_chain) НЕ сломан:
    sink идёт после language, оба — после последнего позиционного
    ``batch_total`` (т.е. только как keyword).
    """
    import inspect

    sig = inspect.signature(TTSNode._synthesize_and_play)
    params = [p for p in sig.parameters.values() if p.name != "self"]
    sink_param = sig.parameters.get("sink")
    assert sink_param is not None
    assert sink_param.default == "speaker"
    # sink должен идти ПОСЛЕ всех 8 канонических позиционных параметров.
    canonical = [
        "ssml", "text", "dialogue_id", "ssml_attributes", "speech_id",
        "batch_id", "batch_index", "batch_total",
    ]
    names = [p.name for p in params]
    sink_idx = names.index("sink")
    for cname in canonical:
        assert cname in names, f"канонический {cname} исчез из сигнатуры"
        cidx = names.index(cname)
        assert cidx < sink_idx, (
            f"{cname} (idx {cidx}) должен идти ДО sink (idx {sink_idx})"
        )


def test_sink_kwarg_in_run_synthesis_worker_signature():
    """``_run_synthesis_worker(sink=...)`` тоже keyword-only через **kwargs."""
    import inspect

    sig = inspect.signature(TTSNode._run_synthesis_worker)
    # sink НЕ positional — пробрасывается через **kwargs (как voice/language).
    assert "sink" not in sig.parameters
    assert "**kwargs" in str(sig)
