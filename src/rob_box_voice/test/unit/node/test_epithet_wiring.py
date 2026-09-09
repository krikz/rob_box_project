"""test_epithet_wiring.py — обвязка эпитетов в speaker_id_node (issue #1787).

Чистая логика выбора клички проверяется в ``test/test_epithets.py``. Здесь —
решения, которые принимает НОДА поверх неё:

1. первая реплика известного спикера сразу даёт кличку (``first_seen``);
2. разговор на ту же тему кличку не трогает;
3. новая доминирующая тема НЕ меняет кличку раньше 30 дней (антидребезг);
4. она же меняет — когда интервал прошёл;
5. ``_publish_result`` кладёт эпитет в JSON для dialogue_node;
6. сбой на этом пути не роняет воркер, который считает эмбеддинги.

``SpeakerIdNode.__init__`` не вызывается (там ROS-параметры, ThreadPool и
warmup resemblyzer) — узел собирается через ``object.__new__`` и получает
ровно те поля, которые нужны эпитетам. Тот же приём, что в
``test_issue_1101_diagnostics.py``.
"""

from __future__ import annotations

import collections
import json
import sys
import threading
import types
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
import pytest

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

CHESS_TALK = "шахматы, дебют и эндшпиль, ферзь, ладья, рокировка, гамбит"
TECH_TALK = "паяльник, микросхема, транзистор, прошивка, мультиметр, припой"


def _embedding(seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    v = rng.standard_normal(256).astype(np.float32)
    return v / np.linalg.norm(v)


@pytest.fixture()
def node(tmp_path):
    """SpeakerIdNode без ROS-инициализации, с реальной БД во временном файле."""
    instance = object.__new__(sid_node.SpeakerIdNode)
    instance._db = SpeakerDatabase(str(tmp_path / "speakers.db"))
    instance._speech_log = {}
    instance._speech_log_lock = threading.Lock()
    instance.get_logger = MagicMock(return_value=MagicMock())
    yield instance
    instance._db.close()


def test_first_observation_assigns_epithet(node):
    sid = node._db.register("Денчик", _embedding(1))
    node._process_observation(sid, CHESS_TALK)

    profile = node._db.get_speaker_profile(sid)
    assert profile["epithet"] in ep.EPITHET_LEXICON["шахматы"]
    assert profile["tags"] == ["шахматы"]
    assert profile["epithet_history"][0]["reason"] == ep.REASON_FIRST_SEEN


def test_two_namesakes_get_different_epithets(node):
    """Сценарий из issue #1787: оба «Денисы», оба про шахматы."""
    first = node._db.register("Денис", _embedding(2))
    second = node._db.register("Денис", _embedding(3))

    node._process_observation(first, CHESS_TALK)
    node._process_observation(second, CHESS_TALK)

    a = node._db.get_speaker_profile(first)["epithet"]
    b = node._db.get_speaker_profile(second)["epithet"]
    assert a and b and a != b


def test_same_topic_does_not_change_epithet(node):
    sid = node._db.register("Денчик", _embedding(4))
    node._process_observation(sid, CHESS_TALK)
    original = node._db.get_speaker_profile(sid)["epithet"]

    for _ in range(5):
        node._process_observation(sid, CHESS_TALK)

    profile = node._db.get_speaker_profile(sid)
    assert profile["epithet"] == original
    assert len(profile["epithet_history"]) == 1


def test_new_topic_is_ignored_within_review_interval(node):
    """Антидребезг: сменил тему на следующей же фразе — кличка та же."""
    sid = node._db.register("Денчик", _embedding(5))
    node._process_observation(sid, CHESS_TALK)
    original = node._db.get_speaker_profile(sid)["epithet"]

    # Окно реплик заполняем техно-темой, чтобы она стала доминирующей.
    for _ in range(10):
        node._process_observation(sid, TECH_TALK)

    assert node._db.get_speaker_profile(sid)["epithet"] == original


def test_new_topic_changes_epithet_after_interval(node):
    sid = node._db.register("Денчик", _embedding(6))
    node._process_observation(sid, CHESS_TALK)
    original = node._db.get_speaker_profile(sid)["epithet"]

    # Отматываем таймер пересмотра на 60 дней назад.
    node._db._conn.execute(
        "UPDATE speakers SET last_epithet_review=? WHERE speaker_id=?",
        (0.0, sid),
    )
    node._db._conn.commit()

    for _ in range(10):
        node._process_observation(sid, TECH_TALK)

    profile = node._db.get_speaker_profile(sid)
    assert profile["epithet"] != original
    assert profile["epithet"] in ep.EPITHET_LEXICON["техно"]
    assert profile["epithet_history"][-1]["reason"].startswith(
        ep.REASON_NEW_TOPIC
    )
    assert profile["epithet_history"][-1]["old"] == original


def test_observation_for_deleted_speaker_is_a_noop(node):
    """Спикера слили/удалили между публикацией и обработкой — не падаем."""
    node._process_observation("no-such-id", CHESS_TALK)
    node.get_logger.return_value.warning.assert_not_called()


def test_observation_survives_db_failure(node):
    """Эпитет — вспомогательная метка: сбой логируется, но не пробрасывается."""
    sid = node._db.register("Денчик", _embedding(7))
    node._db.get_speaker_profile = MagicMock(side_effect=RuntimeError("boom"))

    node._process_observation(sid, CHESS_TALK)  # не должно бросить

    node.get_logger.return_value.warning.assert_called_once()


def test_ensure_epithet_gives_default_before_any_speech(node):
    """Регистрация без единой реплики всё равно даёт кличку (research §5.1)."""
    sid = node._db.register("Молчун", _embedding(8))
    node._ensure_epithet(sid)

    epithet = node._db.get_epithet(sid)
    assert epithet in ep.DEFAULT_POOL_NEUTRAL


def test_ensure_epithet_does_not_overwrite_existing(node):
    sid = node._db.register("Денчик", _embedding(9))
    node._db.set_epithet(sid, "Гроссмейстер", ep.REASON_FIRST_SEEN)

    node._ensure_epithet(sid)

    assert node._db.get_epithet(sid) == "Гроссмейстер"
    assert len(node._db.get_speaker_profile(sid)["epithet_history"]) == 1


def test_publish_result_includes_epithet(node):
    published = []
    node._result_pub = types.SimpleNamespace(
        publish=lambda msg: published.append(msg.data)
    )
    sid_node.String = lambda: types.SimpleNamespace(data="")

    node._publish_result(
        SpeakerMatch(
            speaker_id="uuid-1",
            name="Денис",
            confidence=0.91,
            epithet="Гроссмейстер",
        )
    )

    payload = json.loads(published[0])
    assert payload["is_known"] is True
    assert payload["epithet"] == "Гроссмейстер"


def test_observe_request_rejects_garbage(node):
    """Битый JSON и пустые поля не доходят до executor'а."""
    node._executor = MagicMock()

    for raw in ("{не json", '{"speaker_id": "", "text": "привет"}',
                '{"speaker_id": "x", "text": "   "}'):
        node._on_observe_request(types.SimpleNamespace(data=raw))

    node._executor.submit.assert_not_called()

    node._on_observe_request(
        types.SimpleNamespace(data='{"speaker_id": "x", "text": "привет"}')
    )
    node._executor.submit.assert_called_once()


def test_speech_window_is_bounded(node):
    """Окно реплик не растёт бесконечно — 50 последних (research §4.1)."""
    sid = node._db.register("Болтун", _embedding(10))
    for i in range(120):
        node._process_observation(sid, f"{CHESS_TALK} {i}")

    assert isinstance(node._speech_log[sid], collections.deque)
    assert len(node._speech_log[sid]) == 50


# ── Слой 2: обмен с LLM ──────────────────────────────────────────────────────


def _capture_requests(node):
    """Подменить publisher запросов и вернуть список payload-ов."""
    sent = []
    node._epithet_request_pub = types.SimpleNamespace(
        publish=lambda msg: sent.append(json.loads(msg.data))
    )
    sid_node.String = lambda: types.SimpleNamespace(data="")
    return sent


def test_dictionary_epithet_is_stored_before_asking_llm(node):
    """Порядок важен: сначала рабочая кличка в БД, потом запрос к LLM."""
    sent = _capture_requests(node)
    sid = node._db.register("Денчик", _embedding(11))

    node._process_observation(sid, CHESS_TALK)

    stored = node._db.get_epithet(sid)
    assert stored in ep.EPITHET_LEXICON["шахматы"]
    assert len(sent) == 1
    assert sent[0]["speaker_id"] == sid
    assert sent[0]["fallback"] == stored
    assert sent[0]["cluster"] == "шахматы"
    assert sent[0]["messages"] == [CHESS_TALK]


def test_llm_epithet_replaces_dictionary_one(node):
    sent = _capture_requests(node)
    sid = node._db.register("Денчик", _embedding(12))
    node._process_observation(sid, CHESS_TALK)
    dictionary_label = node._db.get_epithet(sid)

    node._on_epithet_result(
        types.SimpleNamespace(
            data=json.dumps({"speaker_id": sid, "epithet": "Ферзегрыз"})
        )
    )

    profile = node._db.get_speaker_profile(sid)
    assert profile["epithet"] == "Ферзегрыз"
    assert profile["epithet_history"][-1]["old"] == dictionary_label
    assert profile["epithet_history"][-1]["reason"] == ep.REASON_LLM
    assert sent  # запрос к LLM действительно уходил


def test_invalid_llm_epithet_keeps_dictionary_one(node):
    _capture_requests(node)
    sid = node._db.register("Денчик", _embedding(13))
    node._process_observation(sid, CHESS_TALK)
    dictionary_label = node._db.get_epithet(sid)

    for bad in ("", "не знаю что придумать", "Агент007", None):
        node._on_epithet_result(
            types.SimpleNamespace(
                data=json.dumps({"speaker_id": sid, "epithet": bad})
            )
        )

    assert node._db.get_epithet(sid) == dictionary_label


def test_llm_epithet_cannot_steal_taken_label(node):
    """LLM предложила кличку, которая уже занята другим спикером."""
    _capture_requests(node)
    first = node._db.register("Денис", _embedding(14))
    second = node._db.register("Денис", _embedding(15))
    node._db.set_epithet(first, "Кулибин", ep.REASON_LLM)
    node._db.set_epithet(second, "Электрик", ep.REASON_FIRST_SEEN)

    node._on_epithet_result(
        types.SimpleNamespace(
            data=json.dumps({"speaker_id": second, "epithet": "Кулибин"})
        )
    )

    assert node._db.get_epithet(second) == "Электрик"
    assert node._db.get_epithet(first) == "Кулибин"


def test_epithet_result_ignores_garbage_json(node):
    node._on_epithet_result(types.SimpleNamespace(data="{не json"))
    node._on_epithet_result(types.SimpleNamespace(data='{"epithet": "Кулибин"}'))
    # Ни одного спикера в БД не появилось и не сломалось.
    assert node._db.list_speakers() == []
