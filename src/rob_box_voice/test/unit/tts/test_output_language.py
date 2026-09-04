"""AV-28 — язык произношения на конкретную реплику (per-utterance override).

`ros2-audio-contract-spec.md` §2.2 объявляет `language` варьирующимся
параметром («ROS-param minimax_language ИЛИ override»), но override не был
реализован: dialogue_node переписывал реплику оператора на французский, а
tts_node синтезировал её со статическим `minimax_language="ru"`. Из-за этого
«работали только русский и английский» — на остальных языках робот выдавал
французский текст русской фонетикой.

Здесь закреплены три звена:
  1. поле `language` в payload `/voice/dialogue/response`;
  2. проброс payload → dialogue_callback → worker → _synthesize_and_play;
  3. поведение провайдеров: minimax получает language_boost, а yandex и
     silero (в нашей раскладке оба ru-only) — честную фразу-отказ вместо
     чужого текста.

Стабы — те же, что в test_voice_selection.py (без rclpy/grpc/torch).
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

from rob_box_voice.core.speak_helpers import (  # noqa: E402
    build_ssml_payload,
    unsupported_language_notice,
)
from rob_box_voice.tts_node import TTSNode  # noqa: E402
from test.unit.tts.test_voice_selection import _playback_node  # noqa: E402


def _run(node, *, voice=None, language=None, text="hello") -> None:
    TTSNode._synthesize_and_play(
        node,
        f"<speak>{text}</speak>",
        text,
        None,
        {},
        None,
        None,
        None,
        None,
        voice=voice,
        language=language,
    )


# ── Звено 1: поле в payload ─────────────────────────────────────────────────


def test_payload_carries_language_when_set() -> None:
    payload = json.loads(build_ssml_payload("bonjour", language="fr"))
    assert payload["language"] == "fr"


def test_payload_omits_language_when_absent() -> None:
    """Обычная реплика робота язык не указывает — поведение прежнее."""
    payload = json.loads(build_ssml_payload("привет"))
    assert "language" not in payload


# ── Звено 3: что умеет каждый провайдер ─────────────────────────────────────


@pytest.mark.parametrize("language", ["ru", "en", "fr", "de", "zh", "hi"])
def test_minimax_accepts_every_ui_language(language: str) -> None:
    """MiniMax задаёт язык через language_boost — ограничений нет."""
    assert unsupported_language_notice("minimax", language) is None


@pytest.mark.parametrize("provider", ["yandex", "silero"])
@pytest.mark.parametrize("language", ["fr", "de", "zh", "hi", "en"])
def test_ru_only_providers_refuse_foreign_language(
    provider: str, language: str
) -> None:
    notice = unsupported_language_notice(provider, language)
    assert notice is not None
    assert provider in notice


@pytest.mark.parametrize("provider", ["minimax", "yandex", "silero"])
def test_no_language_requested_never_refuses(provider: str) -> None:
    """`language=None` — обычный русский диалог, ограничений быть не должно."""
    assert unsupported_language_notice(provider, None) is None
    assert unsupported_language_notice(provider, "ru") is None


# ── Звено 2+3: проброс до провайдера ────────────────────────────────────────


def test_minimax_receives_requested_language() -> None:
    node = _playback_node()
    _run(node, language="fr")
    _, kwargs = node._synthesize_minimax.call_args
    assert kwargs.get("language") == "fr"


def test_minimax_language_none_keeps_ros_param_behaviour() -> None:
    """Без запроса язык не навязываем — сработает minimax_language."""
    node = _playback_node()
    _run(node)
    _, kwargs = node._synthesize_minimax.call_args
    assert kwargs.get("language") is None


def test_silero_speaks_notice_instead_of_foreign_text() -> None:
    """Silero (модель v5_ru) не должен читать французский русской фонетикой.

    Ровно этот сценарий и случился на роботе: minimax и yandex умерли по
    деньгам, синтез свалился на Silero, а оператор продолжал выбирать язык.
    """
    node = _playback_node()
    node._synthesize_minimax = MagicMock(side_effect=RuntimeError("MiniMax dead"))
    node._synthesize_yandex = MagicMock(side_effect=RuntimeError("Yandex dead"))
    node.provider_chain = ["minimax", "yandex", "silero"]
    _run(node, language="fr", text="Je vais vers le portail")
    node._synthesize_silero.assert_called_once()
    spoken = node._synthesize_silero.call_args[0][0]
    assert "Je vais vers le portail" not in spoken
    assert "французском" in spoken


def test_silero_speaks_russian_text_unchanged() -> None:
    """Русский язык Silero умеет — текст обязан дойти нетронутым."""
    node = _playback_node()
    node._synthesize_minimax = MagicMock(side_effect=RuntimeError("MiniMax dead"))
    node._synthesize_yandex = MagicMock(side_effect=RuntimeError("Yandex dead"))
    node.provider_chain = ["minimax", "yandex", "silero"]
    _run(node, language="ru", text="еду к воротам")
    assert node._synthesize_silero.call_args[0][0] == "еду к воротам"


def test_silero_without_language_is_untouched() -> None:
    """Обычная реплика робота (language=None) — прежнее поведение."""
    node = _playback_node()
    node._synthesize_minimax = MagicMock(side_effect=RuntimeError("MiniMax dead"))
    node._synthesize_yandex = MagicMock(side_effect=RuntimeError("Yandex dead"))
    node.provider_chain = ["minimax", "yandex", "silero"]
    _run(node, text="еду к воротам")
    assert node._synthesize_silero.call_args[0][0] == "еду к воротам"


def test_yandex_refuses_foreign_language() -> None:
    """Весь наш каталог Yandex-голосов ru-RU (tts_voice_registry)."""
    node = _playback_node()
    node._synthesize_minimax = MagicMock(side_effect=RuntimeError("MiniMax dead"))
    node._synthesize_yandex = MagicMock(
        return_value=np.zeros(4800, dtype=np.float32)
    )
    node.provider_chain = ["minimax", "yandex", "silero"]
    _run(node, language="de", text="Ich fahre zum Tor")
    node._synthesize_yandex.assert_called_once()
    spoken = node._synthesize_yandex.call_args[0][0]
    assert "Ich fahre zum Tor" not in spoken
    assert "немецком" in spoken


# ── Звено 2: worker достаёт language из kwargs ──────────────────────────────


def test_worker_forwards_language_via_kwargs() -> None:
    """`language`, как и `voice`, идёт через kwargs.

    Позиционная арность `_run_synthesis_worker` / `_synthesize_and_play`
    закреплена контрактом test_speech_id_arg_chain: новый параметр обязан
    ехать keyword-only, иначе поедут speech_id/batch_* и сломается
    корреляция /voice/tts/finished (issue #980).
    """
    node = _playback_node()
    node._synthesize_and_play = MagicMock()
    TTSNode._run_synthesis_worker.__get__(node, type(node))(
        "<speak>x</speak>", "x", None, {}, "sid1", "bid1", 1, 1,
        play_seq=1, voice="zahar", language="fr",
    )
    node._synthesize_and_play.assert_called_once()
    _, kwargs = node._synthesize_and_play.call_args
    assert kwargs.get("language") == "fr"
    assert kwargs.get("voice") == "zahar"


def test_worker_without_language_passes_none() -> None:
    node = _playback_node()
    node._synthesize_and_play = MagicMock()
    TTSNode._run_synthesis_worker.__get__(node, type(node))(
        "<speak>x</speak>", "x", None, {}, "sid1", "bid1", 1, 1, play_seq=1,
    )
    _, kwargs = node._synthesize_and_play.call_args
    assert kwargs.get("language") is None
