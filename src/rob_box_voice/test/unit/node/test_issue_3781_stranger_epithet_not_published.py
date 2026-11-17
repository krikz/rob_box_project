"""ADR-0164 этап 3 (#3781 п.4): плейсхолдер «Незнакомец» и кличка спорного профиля не уходят в ``SpeakerResult``.

Живой лог 05.11: ``Publishing: is_known=true name=None epithet='Незнакомец' conf=0.736 (name suppressed: contested)``
— спорная фраза несла старую LLM-кличку-плейсхолдер второго профиля «Дэнчик» (``c9e981cb``). Кличка в
результате — только настоящая (словарь/LLM); иначе пусто. Проверяются чистые сборщики payload издателя.
"""

from __future__ import annotations

import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest

_audio_common = types.ModuleType("audio_common_msgs")
_audio_common_msg = types.ModuleType("audio_common_msgs.msg")
_audio_common_msg.AudioData = MagicMock
sys.modules.setdefault("audio_common_msgs", _audio_common)
sys.modules.setdefault("audio_common_msgs.msg", _audio_common_msg)
for _hw in ("pyaudio", "usb", "usb.core", "usb.util", "sounddevice"):
    sys.modules.setdefault(_hw, MagicMock())

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from rob_box_voice import speaker_id_node as sid_node  # noqa: E402
from rob_box_voice.utils.speaker_embeddings import SpeakerMatch  # noqa: E402

BAND_HIGH = 0.80


def _match(epithet, name="Дэнчик", conf=0.736):
    return SpeakerMatch(speaker_id="c9e981cb-0000", name=name, confidence=conf, epithet=epithet)


def test_contested_phrase_publishes_no_epithet_live_case():
    """Живой случай 05.11: contested + «Незнакомец» → в payload и в строке лога клички нет."""
    payload, line = sid_node._named_payload(
        _match("Незнакомец"), None, sid_node.NAME_TENTATIVE_CONTESTED, "u-1", band_high=BAND_HIGH
    )
    assert payload["epithet"] is None
    assert payload["name"] is None
    assert "Незнакомец" not in line
    assert "epithet=None" in line


def test_contested_phrase_drops_real_epithet_of_foreign_profile():
    """Спорная фраза не приписана никому: настоящая кличка ближайшего профиля — чужая, тоже не уходит."""
    payload, line = sid_node._named_payload(
        _match("Струнный мастер эффектов"), None, sid_node.NAME_TENTATIVE_CONTESTED, None, band_high=BAND_HIGH
    )
    assert payload["epithet"] is None
    assert "Струнный" not in line


@pytest.mark.parametrize("placeholder", ["Незнакомец", "Мудрый незнакомец", "Кулибин-Незнакомец", "Гость"])
def test_confident_named_profile_never_publishes_placeholder(placeholder):
    payload, line = sid_node._named_payload(
        _match(placeholder, conf=0.93), None, sid_node.NAME_CONFIDENT, "u-2", band_high=BAND_HIGH
    )
    assert payload["name"] == "Дэнчик"
    assert payload["epithet"] is None
    assert placeholder not in line


def test_confident_named_profile_keeps_real_epithet():
    payload, line = sid_node._named_payload(
        _match("Струнный мастер эффектов", conf=0.93), None, sid_node.NAME_CONFIDENT, "u-3", band_high=BAND_HIGH
    )
    assert payload["epithet"] == "Струнный мастер эффектов"
    assert "epithet='Струнный мастер эффектов'" in line


def test_tentative_single_keeps_real_epithet():
    """Один кандидат (single) — голос приписан ему гипотезой; его настоящая кличка остаётся (как до этапа 3)."""
    payload, _ = sid_node._named_payload(
        _match("Странник-4"), None, sid_node.NAME_TENTATIVE_SINGLE, "u-4", band_high=BAND_HIGH
    )
    assert payload["epithet"] == "Странник-4"
    assert payload["tentative_name"] == "Дэнчик"


def test_unnamed_profile_placeholder_not_published_real_kept():
    payload, line = sid_node._unnamed_payload(_match("Незнакомец", name=None), "u-5", None)
    assert payload["epithet"] is None
    assert "Незнакомец" not in line
    payload, _ = sid_node._unnamed_payload(_match("Странник-4", name=None), "u-6", None)
    assert payload["epithet"] == "Странник-4"
