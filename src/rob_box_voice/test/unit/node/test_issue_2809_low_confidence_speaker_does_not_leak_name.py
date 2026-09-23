"""test_issue_2809_low_confidence_speaker_does_not_leak_name.py

Issue #2809 -- незнакомцу робот назвал имя другого диктора («Борис»).

Диагноз (PR #2815, docs/plans/2026-09-23-issue-2809-persona-cluster-
diagnosis.md): ``must_not_say`` работает как detection (режет фразу с
именем в ``robot_speech()``), но не как prevention -- имя всё равно
доезжает до LLM, потому что ``speaker_id_node`` публикует ``name`` уже
при ``confidence`` выше ``identify_threshold`` (0.72), а между 0.72 и
реальной уверенностью (0.763 на живом run 35788126541, шаг n210) match
регулярно указывает на ДРУГОГО реального диктора.

Фикс (вариант C из диагноза):

1. ``speaker_id_node._publish_result`` подавляет ``name`` (публикует
   ``None``) в JSON, когда ``match.confidence < confident_identify_
   threshold`` (default 0.85) -- ``is_known``/``speaker_id``/``epithet``
   остаются для observability и роста галереи/эпитета.
2. ``dialogue_node._apply_speaker_identity`` на ``is_known=True,
   name=None`` не подставляет ``[Spkr:...]`` в user_input (и -- за счёт
   того же пустого ``sp["name"]`` -- ``_build_dynamic_system_context``
   сам уходит в существующую ветку ``unknown`` с ``privacy_note``,
   issue #2779), а помечает реплику явным ``[Speaker:tentative]``.

Тест не поднимает ROS2 (тот же приём, что test_epithet_wiring.py и
test_issue_1195_tg_source.py -- узлы собираются через ``object.__new__``).
"""

from __future__ import annotations

import asyncio
import sys
import threading
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest

# ``audio_common_msgs`` нужен только speaker_id_node -- заглушка, как в
# test_epithet_wiring.py.
_audio_common = types.ModuleType("audio_common_msgs")
_audio_common_msg = types.ModuleType("audio_common_msgs.msg")
_audio_common_msg.AudioData = MagicMock
sys.modules.setdefault("audio_common_msgs", _audio_common)
sys.modules.setdefault("audio_common_msgs.msg", _audio_common_msg)

for _hw in ("pyaudio", "usb", "usb.core", "usb.util", "sounddevice"):
    sys.modules.setdefault(_hw, MagicMock())

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from rob_box_voice import speaker_id_node as sid_node  # noqa: E402
from rob_box_voice.dialogue_node import DialogueNode  # noqa: E402
from rob_box_voice.utils.speaker_embeddings import SpeakerMatch  # noqa: E402


# ─────────────────────────────────────────────────────────────────────────────
#  speaker_id_node._publish_result -- гейт по confident_identify_threshold
# ─────────────────────────────────────────────────────────────────────────────


@pytest.fixture()
def sid():
    instance = object.__new__(sid_node.SpeakerIdNode)
    instance._confident_identify_threshold = 0.85
    instance._result_pub = MagicMock()
    instance.get_logger = MagicMock(return_value=MagicMock())
    return instance


def _published_payload(pub_mock) -> dict:
    import json

    assert pub_mock.publish.call_count == 1
    msg = pub_mock.publish.call_args.args[0]
    return json.loads(msg.data)


class TestPublishResultConfidenceGate:
    def test_low_confidence_match_suppresses_name(self, sid):
        """Живой n210: confidence=0.763 < 0.85 -- имя НЕ уходит в топик,
        is_known/speaker_id/confidence/epithet остаются."""
        match = SpeakerMatch(
            speaker_id="0ddc1ab9-9af0-4c00-8000-000000000000",
            name="Борис",
            confidence=0.763,
            epithet="Собеседник",
        )
        sid._publish_result(match)

        payload = _published_payload(sid._result_pub)
        assert payload["is_known"] is True
        assert payload["name"] is None
        assert payload["speaker_id"] == match.speaker_id
        assert payload["confidence"] == pytest.approx(0.763)
        assert payload["epithet"] == "Собеседник"

    def test_high_confidence_match_publishes_name(self, sid):
        """Выше порога -- обычное поведение, имя публикуется как раньше."""
        match = SpeakerMatch(
            speaker_id="known-1", name="Борис", confidence=0.91, epithet=None
        )
        sid._publish_result(match)

        payload = _published_payload(sid._result_pub)
        assert payload["name"] == "Борис"

    def test_register_source_bypasses_confidence_gate(self, sid):
        """Явная регистрация голоса (``source="register"``) -- имя названо
        человеком секунду назад, не догадка по cosine; порог не применяем
        даже если confidence ниже него."""
        match = SpeakerMatch(speaker_id="new-1", name="Гриша", confidence=0.5)
        sid._publish_result(match, source="register")

        payload = _published_payload(sid._result_pub)
        assert payload["name"] == "Гриша"
        assert payload["source"] == "register"

    def test_unknown_speaker_untouched(self, sid):
        sid._publish_result(None)
        payload = _published_payload(sid._result_pub)
        assert payload == {"is_known": False}


# ─────────────────────────────────────────────────────────────────────────────
#  dialogue_node._apply_speaker_identity -- low-confidence не доезжает до LLM
# ─────────────────────────────────────────────────────────────────────────────


@pytest.fixture()
def node():
    n = object.__new__(DialogueNode)
    n.get_logger = MagicMock(return_value=MagicMock())
    n._speaker_lock = threading.Lock()
    n._publish_speaker_observation = MagicMock()
    return n


def _run(coro):
    return asyncio.run(coro)


class TestApplySpeakerIdentityConfidenceGate:
    def test_name_none_low_confidence_yields_tentative_tag_not_name(self, node):
        """speaker_id_node уже подавил имя (name=None, is_known=True) --
        воспроизводит n210 (был опознан Борис, потом low-confidence
        match): в user_input не должно быть ни 'Борис', ни '[Spkr:'."""
        node._current_speaker = {
            "is_known": True,
            "speaker_id": "0ddc1ab9-9af0-4c00-8000-000000000000",
            "name": None,
            "confidence": 0.763,
            "epithet": "Собеседник",
        }

        result = _run(
            node._apply_speaker_identity(
                "я мимо шел. Имя свое я тебе называть не буду.",
                speaker_context=None,
            )
        )

        assert "Борис" not in result
        assert "[Spkr:" not in result
        assert "[Speaker:tentative]" in result
        node._publish_speaker_observation.assert_called_once()

    def test_confident_known_speaker_still_gets_name_tag(self, node):
        """Регресс-контроль: высокая уверенность -- поведение не меняем."""
        node._current_speaker = {
            "is_known": True,
            "speaker_id": "known-1",
            "name": "Антон",
            "confidence": 0.93,
        }

        result = _run(
            node._apply_speaker_identity("привет", speaker_context=None)
        )

        assert "[Spkr:Антон]" in result

    def test_unknown_speaker_still_gets_unknown_tag(self, node):
        node._current_speaker = {"is_known": False}

        result = _run(
            node._apply_speaker_identity("привет", speaker_context=None)
        )

        assert "[Speaker:unknown]" in result
