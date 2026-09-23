"""test_issue_2809_name_confidence_live_param.py -- ``ros2 param set
/speaker_id_node name_confidence_band_high|name_confidence_min_gap ...``
применяется НЕМЕДЛЕННО, без рестарта узла.

Контекст (задача E2E-харнесса, issue #2809, follow-up PR #2818): новый акт
марафона "переспрос личности" форсирует зону сомнения на время акта через
top-level поле сценария ``node_params`` (``{"/speaker_id_node":
{"name_confidence_band_high": 0.99}}``), проверяет решение узла, а в конце
ВСЕГДА восстанавливает исходное значение (см. trap в
``e2e_voice_test.sh:restore_node_params``).

До этой правки ``classify_name_confidence`` читал
``self._name_confidence_band_high`` / ``self._name_confidence_min_gap`` --
снапшот, снятый ОДИН раз в ``__init__`` (см. docstring
``parameters_callback`` до issue #2809: "остальные параметры узла
... читаются один раз в __init__ и здесь не перехватываются"). ``ros2
param set`` на живом узле проходил бы валидацию молча, но решение
классификатора не поменялось бы -- переопределение сценария сработало бы
вхолостую, и харнесс решил бы, что переспрос детерминирован, хотя на самом
деле сработала историческая калибровка 0.80/0.15, а не 0.99 из сценария.
Это ровно тот silent-degrade, который описан в памятке
voice-stack-degrades-silently.

Приём тестирования -- тот же, что в ``test_e2e_db_isolation.py``:
``SpeakerIdNode`` собирается через ``object.__new__`` и получает только
поля, которые трогает ``parameters_callback``.
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


class _FakeParam:
    def __init__(self, name: str, value) -> None:
        self.name = name
        self.value = value


@pytest.fixture()
def node():
    instance = object.__new__(sid_node.SpeakerIdNode)
    instance._e2e_mode_active = False
    instance._name_confidence_band_high = 0.80
    instance._name_confidence_min_gap = 0.15
    instance.get_logger = MagicMock(return_value=MagicMock())
    return instance


def test_band_high_applied_immediately(node):
    result = node.parameters_callback(
        [_FakeParam("name_confidence_band_high", 0.99)]
    )

    assert result.successful is True
    assert node._name_confidence_band_high == pytest.approx(0.99)


def test_min_gap_applied_immediately(node):
    result = node.parameters_callback(
        [_FakeParam("name_confidence_min_gap", 0.30)]
    )

    assert result.successful is True
    assert node._name_confidence_min_gap == pytest.approx(0.30)


def test_classify_name_confidence_uses_new_band_high_without_restart(node):
    """Не только поле меняется -- РЕШЕНИЕ классификатора меняется тоже,
    без пересборки узла (это то, что действительно проверяет харнесс)."""
    match = SpeakerMatch(speaker_id="sid-1", name="Саша", confidence=0.85)
    candidates = [match]

    # Дефолт 0.80: score=0.85 >= 0.80 -> CONFIDENT.
    decision_before = sid_node.classify_name_confidence(
        match,
        candidates,
        band_high=node._name_confidence_band_high,
        min_gap=node._name_confidence_min_gap,
    )
    assert decision_before == sid_node.NAME_CONFIDENT

    node.parameters_callback([_FakeParam("name_confidence_band_high", 0.99)])

    # После live-override 0.99: тот же score=0.85 уже НЕ проходит планку,
    # конкурента с другим именем нет -> single hypothesis, а не confident.
    decision_after = sid_node.classify_name_confidence(
        match,
        candidates,
        band_high=node._name_confidence_band_high,
        min_gap=node._name_confidence_min_gap,
    )
    assert decision_after != sid_node.NAME_CONFIDENT
    assert decision_after == sid_node.NAME_TENTATIVE_SINGLE


def test_restore_to_original_value_after_override(node):
    """Симулирует цикл харнесса: снять оригинал, применить override,
    восстановить -- решение классификатора обязано вернуться к исходному
    поведению, а не залипнуть на override."""
    original = node._name_confidence_band_high

    node.parameters_callback([_FakeParam("name_confidence_band_high", 0.99)])
    assert node._name_confidence_band_high == pytest.approx(0.99)

    node.parameters_callback(
        [_FakeParam("name_confidence_band_high", original)]
    )
    assert node._name_confidence_band_high == pytest.approx(original)


@pytest.mark.parametrize(
    "bad_value", [float("nan"), float("inf"), float("-inf")]
)
def test_non_finite_value_rejected(node, bad_value):
    original = node._name_confidence_band_high

    result = node.parameters_callback(
        [_FakeParam("name_confidence_band_high", bad_value)]
    )

    assert result.successful is False
    assert node._name_confidence_band_high == pytest.approx(original), (
        "невалидное значение не должно было поменять кеш узла"
    )


def test_non_numeric_value_rejected(node):
    original = node._name_confidence_min_gap

    result = node.parameters_callback(
        [_FakeParam("name_confidence_min_gap", "not-a-number")]
    )

    assert result.successful is False
    assert node._name_confidence_min_gap == pytest.approx(original)


def test_unrelated_param_unaffected(node):
    band_before = node._name_confidence_band_high
    gap_before = node._name_confidence_min_gap

    result = node.parameters_callback([_FakeParam("identify_threshold", 0.8)])

    assert result.successful is True
    assert node._name_confidence_band_high == pytest.approx(band_before)
    assert node._name_confidence_min_gap == pytest.approx(gap_before)
