"""test_e2e_db_isolation.py — /voice/speaker/e2e_mode переключает БД дикторов
без единого прикосновения к боевой (issue #2750).

Контекст: боевую ``/data/speakers.db`` обнуляли внешним ssh-скриптом перед
актом «Знакомство» (бэкап ``.bak-<UTC>Z`` + очистка таблицы ``speakers``) —
логика ни разу не найдена в репозитории ни на одной ветке. Один раз это
стёрло профиль живого человека через 19 минут после регистрации. Замена —
``_on_e2e_mode_request``: явный, залогированный (WARNING) переключатель
между ``db_path`` (боевая) и ``e2e_db_path`` (изолированная), запускаемый
ТОЛЬКО E2E-харнессом через топик.

Приём тестирования — тот же, что в ``test_epithet_wiring.py``:
``SpeakerIdNode`` собирается через ``object.__new__`` (без ROS-инициализации,
ThreadPool, resemblyzer warmup) и получает только те поля, которые трогает
``_on_e2e_mode_request``.
"""

from __future__ import annotations

import json
import sys
import threading
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
from rob_box_voice.utils.speaker_embeddings import SpeakerDatabase  # noqa: E402


class _FakeStringMsg:
    """Заглушка ``std_msgs.msg.String`` — узлу нужен только атрибут ``.data``."""

    def __init__(self, data: str) -> None:
        self.data = data


@pytest.fixture()
def node(tmp_path):
    prod_path = str(tmp_path / "speakers.db")
    e2e_path = str(tmp_path / "speakers.e2e.db")

    instance = object.__new__(sid_node.SpeakerIdNode)
    instance._prod_db_path = prod_path
    instance._e2e_db_path = e2e_path
    instance._e2e_mode_active = False
    instance._db_lock = threading.Lock()
    instance._db = SpeakerDatabase(prod_path)
    instance.get_logger = MagicMock(return_value=MagicMock())

    published = []
    pub = MagicMock()
    pub.publish.side_effect = lambda msg: published.append(json.loads(msg.data))
    instance._result_pub = pub
    instance._published = published

    yield instance
    instance._db.close()


def _ack(node) -> dict:
    return node._published[-1]


def test_enable_switches_off_prod_db_without_touching_it(node):
    """Боевая база не открывается на запись, пока e2e_mode включён."""
    src_sid = node._db.register("Деньчик", _rand_embedding(1))
    node._db.close()  # закрываем перед пере-открытием той же прод-БД ниже

    # Переоткрываем узел на боевой (как это делает __init__).
    node._db = SpeakerDatabase(node._prod_db_path)

    node._on_e2e_mode_request(_FakeStringMsg(json.dumps({"enabled": True})))

    assert node._e2e_mode_active is True
    assert node._db._db_path == node._e2e_db_path
    ack = _ack(node)
    assert ack == {
        "event": "e2e_mode",
        "enabled": True,
        "db_path": node._e2e_db_path,
    }

    # Боевая БД физически не тронута: открываем её НАПРЯМУЮ (не через
    # node._db, который сейчас указывает на e2e-файл) и видим профиль.
    prod_check = SpeakerDatabase(node._prod_db_path)
    try:
        names = {s["name"] for s in prod_check.list_speakers()}
        assert "Деньчик" in names
    finally:
        prod_check.close()


def test_registrations_during_e2e_mode_never_reach_prod_db(node):
    node._on_e2e_mode_request(_FakeStringMsg(json.dumps({"enabled": True})))
    node._db.register("Саша", _rand_embedding(10))
    node._db.register("Борис", _rand_embedding(11))

    prod_check = SpeakerDatabase(node._prod_db_path)
    try:
        assert prod_check.list_speakers() == [], (
            "E2E-регистрации попали в боевую speakers.db — изоляция сломана"
        )
    finally:
        prod_check.close()

    e2e_check = SpeakerDatabase(node._e2e_db_path)
    try:
        names = {s["name"] for s in e2e_check.list_speakers()}
        assert names == {"Саша", "Борис"}
    finally:
        e2e_check.close()


def test_disable_returns_to_prod_db(node):
    node._on_e2e_mode_request(_FakeStringMsg(json.dumps({"enabled": True})))
    node._db.register("E2E-only", _rand_embedding(20))

    node._on_e2e_mode_request(_FakeStringMsg(json.dumps({"enabled": False})))

    assert node._e2e_mode_active is False
    assert node._db._db_path == node._prod_db_path
    ack = _ack(node)
    assert ack == {
        "event": "e2e_mode",
        "enabled": False,
        "db_path": node._prod_db_path,
    }
    # Прод-БД, на которую вернулись, не содержит E2E-регистрации.
    assert node._db.list_speakers() == []


def test_enable_wipes_leftover_e2e_file_from_previous_marathon(node, tmp_path):
    """Coordinator update issue #2750: чистку нельзя перекладывать на
    дисциплину вызывающего кода — узел обязан сам гарантировать пустую
    E2E-БД при каждом включении, иначе оператор вернётся к ручному
    ``DELETE FROM`` по боевой. Здесь — «прошлый марафон» смоделирован
    прямой записью профиля в файл e2e_db_path ДО первого enable.
    """
    stale = SpeakerDatabase(node._e2e_db_path)
    stale.register("Призрак-с-прошлого-марафона", _rand_embedding(99))
    stale.close()

    node._on_e2e_mode_request(_FakeStringMsg(json.dumps({"enabled": True})))

    assert node._db.list_speakers() == [], (
        "старый профиль пережил включение e2e_mode — гарантия чистой базы не держит"
    )


def test_repeated_enable_is_idempotent_no_op(node):
    node._on_e2e_mode_request(_FakeStringMsg(json.dumps({"enabled": True})))
    db_after_first = node._db

    node._on_e2e_mode_request(_FakeStringMsg(json.dumps({"enabled": True})))

    assert node._db is db_after_first, "повторный enable пересоздал соединение — не no-op"
    assert node._e2e_mode_active is True


def test_plain_text_true_false_accepted_like_register_request(node):
    """Топик управляется ssh/`ros2 topic pub` — JSON неудобен, plain-текст
    должен работать так же, как в ``_on_register_request`` (issue #2750)."""
    node._on_e2e_mode_request(_FakeStringMsg("true"))
    assert node._e2e_mode_active is True

    node._on_e2e_mode_request(_FakeStringMsg("false"))
    assert node._e2e_mode_active is False


def _rand_embedding(seed: int):
    import numpy as np

    rng = np.random.default_rng(seed)
    v = rng.standard_normal(256).astype("float32")
    return v / np.linalg.norm(v)
