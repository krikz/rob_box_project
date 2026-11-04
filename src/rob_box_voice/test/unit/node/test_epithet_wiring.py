"""test_epithet_wiring.py — обвязка эпитетов в speaker_id_node (issue #1787).

Чистая логика выбора клички проверяется в ``test/test_epithets.py``. Здесь —
решения, которые принимает НОДА поверх неё:

1. регистрация сразу даёт словарную кличку (``first_seen``), тёзки — разные;
2. существующую кличку регистрация не трогает;
3. ``_publish_result`` кладёт эпитет в результат биометрии.

Темы речи (``/voice/speaker/observe``) и кличка от LLM
(``/voice/speaker/{epithet_request,epithet}``) были только у ``dialogue_node``
v1 — цепочка удалена (#4354 A2).

``SpeakerIdNode.__init__`` не вызывается (там ROS-параметры, ThreadPool и
warmup resemblyzer) — узел собирается через ``object.__new__`` и получает
ровно те поля, которые нужны эпитетам. Тот же приём, что в
``test_issue_1101_diagnostics.py``.
"""

from __future__ import annotations

import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
import pytest

from rob_box_core.speaker_wire import speaker_result_to_payload

# ``audio_common_msgs`` не покрыт общим conftest (его тянет только
# speaker_id_node) — доставляем заглушку до импорта ноды.
_audio_common = types.ModuleType("audio_common_msgs")
_audio_common_msg = types.ModuleType("audio_common_msgs.msg")
_audio_common_msg.AudioData = MagicMock
sys.modules.setdefault("audio_common_msgs", _audio_common)
sys.modules.setdefault("audio_common_msgs.msg", _audio_common_msg)

# ``rob_box_voice.utils.__init__`` (нужен ради speaker_embeddings) тянет
# pyaudio/usb через audio_utils и respeaker_interface — железо, которого
# нет на dev-машине и в CI. Нода ими не пользуется (эпитеты и биометрия —
# чистая логика + SQLite), поэтому глушим на уровне sys.modules.
for _hw in ("pyaudio", "usb", "usb.core", "usb.util", "sounddevice"):
    sys.modules.setdefault(_hw, MagicMock())

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from rob_box_voice import speaker_id_node as sid_node  # noqa: E402
from rob_box_voice.core import epithets as ep  # noqa: E402
from rob_box_voice.utils.speaker_embeddings import (  # noqa: E402
    SpeakerDatabase,
    SpeakerMatch,
)


@pytest.fixture()
def node(tmp_path):
    """SpeakerIdNode без ROS-инициализации, с реальной БД во временном файле."""
    instance = object.__new__(sid_node.SpeakerIdNode)
    instance._db = SpeakerDatabase(str(tmp_path / "speakers.db"))
    instance.get_logger = MagicMock(return_value=MagicMock())
    yield instance
    instance._db.close()


def _embedding(seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    v = rng.standard_normal(256).astype(np.float32)
    return v / np.linalg.norm(v)


def test_ensure_epithet_gives_default_before_any_speech(node):
    """Регистрация без единой реплики всё равно даёт кличку (research §5.1)."""
    sid = node._db.register("Молчун", _embedding(8))
    node._ensure_epithet(sid)

    epithet = node._db.get_epithet(sid)
    assert epithet in ep.DEFAULT_POOL_NEUTRAL
    assert node._db.get_speaker_profile(sid)["epithet_history"][0]["reason"] == ep.REASON_FIRST_SEEN


def test_two_namesakes_get_different_epithets(node):
    """Сценарий из issue #1787: оба «Денисы» — клички разные."""
    first = node._db.register("Денис", _embedding(2))
    second = node._db.register("Денис", _embedding(3))

    node._ensure_epithet(first)
    node._ensure_epithet(second)

    a = node._db.get_epithet(first)
    b = node._db.get_epithet(second)
    assert a and b and a != b


def test_ensure_epithet_does_not_overwrite_existing(node):
    sid = node._db.register("Денчик", _embedding(9))
    node._db.set_epithet(sid, "Гроссмейстер", ep.REASON_FIRST_SEEN)

    node._ensure_epithet(sid)

    assert node._db.get_epithet(sid) == "Гроссмейстер"
    assert len(node._db.get_speaker_profile(sid)["epithet_history"]) == 1


def test_ensure_epithet_survives_db_failure(node):
    """Эпитет — вспомогательная метка: сбой логируется, но не пробрасывается."""
    node._db.get_epithet = MagicMock(side_effect=RuntimeError("boom"))

    node._ensure_epithet("any-id")  # не должно бросить

    node.get_logger.return_value.warning.assert_called_once()


def test_publish_result_includes_epithet(node):
    published = []
    node._result_pub = types.SimpleNamespace(
        publish=lambda msg: published.append(speaker_result_to_payload(msg))
    )

    node._publish_result(
        SpeakerMatch(
            speaker_id="uuid-1",
            name="Денис",
            confidence=0.91,
            epithet="Гроссмейстер",
        )
    )

    payload = published[0]
    assert payload["is_known"] is True
    assert payload["epithet"] == "Гроссмейстер"
