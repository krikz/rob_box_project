"""test_e2e_db_isolation.py — ``ros2 param set ... e2e_mode`` переключает БД
дикторов без единого прикосновения к боевой (issue #2750).

Контекст: боевую ``/data/speakers.db`` чистили руками по ssh перед каждым
прогоном акта «Знакомство» (``cp`` в ``.bak-<UTC>Z`` + ``DELETE FROM
embeddings; DELETE FROM speakers``, подтверждено владельцем в issue #2750)
— и один раз это стёрло профиль живого человека через 19 минут после
регистрации.

Механизм — параметр узла (``e2e_mode``, bool) с валидирующим
``parameters_callback`` (Humble: только ``add_on_set_parameters_callback``),
а НЕ топик: первая версия этой правки заводила
``/voice/speaker/e2e_mode`` (``std_msgs/String``), но
``scripts/lint/seam_without_consumer.py`` (ADR-0021, issue #2118) справедливо
пометил его новым «швом без потребителя» — паблишер живёт в
``.github/workflows/scripts/e2e_voice_test.sh`` (bash, Python-сканер топиков
под ``src/`` его не видит), и топик, которым можно стереть БД одним
безответным сообщением, — слабая конструкция сама по себе. Параметр —
тот же паттерн, что ``barge_in_policy`` у ``dialogue_node`` и
``volume_db``/voice-параметры у ``tts_node``: синхронный побочный эффект
внутри валидирующего колбэка, успех/провал виден в exit-коде
``ros2 param set`` вызывающему.

Приём тестирования — тот же, что в ``test_epithet_wiring.py``:
``SpeakerIdNode`` собирается через ``object.__new__`` (без ROS-инициализации,
ThreadPool, resemblyzer warmup) и получает только те поля, которые трогает
``parameters_callback``/``_apply_e2e_mode``. ``rclpy``/``rcl_interfaces`` уже
замоканы автоматически ``test/unit/node/conftest.py`` (issue #1601 /
ADR-0027 §3.4) — ``SetParametersResult`` там простой callable-Mock, так что
``result.successful`` отражает переданный kwarg без реального ROS.
"""

from __future__ import annotations

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


class _FakeParam:
    """Заглушка ``rclpy.parameter.Parameter`` — колбэку нужны только
    ``.name``/``.value`` (см. ``speaker_id_node.parameters_callback``:
    ``for param in params: if param.name == "e2e_mode": ...``)."""

    def __init__(self, name: str, value) -> None:
        self.name = name
        self.value = value


def _e2e_mode_param(enabled: bool) -> list[_FakeParam]:
    return [_FakeParam("e2e_mode", enabled)]


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

    yield instance
    instance._db.close()


def test_enable_switches_off_prod_db_without_touching_it(node):
    """Боевая база не открывается на запись, пока e2e_mode включён."""
    node._db.register("Деньчик", _rand_embedding(1))
    node._db.close()  # закрываем перед пере-открытием той же прод-БД ниже

    # Переоткрываем узел на боевой (как это делает __init__).
    node._db = SpeakerDatabase(node._prod_db_path)

    result = node.parameters_callback(_e2e_mode_param(True))

    assert result.successful is True
    assert node._e2e_mode_active is True
    assert node._db._db_path == node._e2e_db_path

    # Боевая БД физически не тронута: открываем её НАПРЯМУЮ (не через
    # node._db, который сейчас указывает на e2e-файл) и видим профиль.
    prod_check = SpeakerDatabase(node._prod_db_path)
    try:
        names = {s["name"] for s in prod_check.list_speakers()}
        assert "Деньчик" in names
    finally:
        prod_check.close()


def test_registrations_during_e2e_mode_never_reach_prod_db(node):
    node.parameters_callback(_e2e_mode_param(True))
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
    node.parameters_callback(_e2e_mode_param(True))
    node._db.register("E2E-only", _rand_embedding(20))

    result = node.parameters_callback(_e2e_mode_param(False))

    assert result.successful is True
    assert node._e2e_mode_active is False
    assert node._db._db_path == node._prod_db_path
    # Прод-БД, на которую вернулись, не содержит E2E-регистрации.
    assert node._db.list_speakers() == []


def test_enable_wipes_leftover_e2e_file_from_previous_marathon(node):
    """Coordinator update issue #2750: чистку нельзя перекладывать на
    дисциплину вызывающего кода — узел обязан сам гарантировать пустую
    E2E-БД при каждом включении, иначе оператор вернётся к ручному
    ``DELETE FROM`` по боевой. Здесь — «прошлый марафон» смоделирован
    прямой записью профиля в файл e2e_db_path ДО первого enable.
    """
    stale = SpeakerDatabase(node._e2e_db_path)
    stale.register("Призрак-с-прошлого-марафона", _rand_embedding(99))
    stale.close()

    node.parameters_callback(_e2e_mode_param(True))

    assert node._db.list_speakers() == [], (
        "старый профиль пережил включение e2e_mode — гарантия чистой базы не держит"
    )


def test_repeated_enable_same_value_does_not_wipe_mid_marathon(node):
    """Coordinator uточнение: повторный ``set_parameters`` с ТЕМ ЖЕ
    значением (``true`` поверх уже включённого ``true``) НЕ должен чистить
    e2e_db_path второй раз — иначе повторный вызов посреди акта обнулил бы
    уже накопленные в этом самом прогоне регистрации."""
    node.parameters_callback(_e2e_mode_param(True))
    node._db.register("Саша", _rand_embedding(30))
    db_after_first = node._db

    result = node.parameters_callback(_e2e_mode_param(True))

    assert result.successful is True
    assert node._db is db_after_first, "повторный True пересоздал соединение — не no-op"
    assert node._e2e_mode_active is True
    names = {s["name"] for s in node._db.list_speakers()}
    assert names == {"Саша"}, (
        "повторный set_parameters(e2e_mode=True) обнулил E2E-базу посреди прогона"
    )


def test_unrelated_param_change_does_not_touch_db(node):
    """``parameters_callback`` — общий роутер узла; ``ros2 param set`` на
    любом другом параметре (например ``identify_threshold``) не должен
    задевать активную БД дикторов."""
    db_before = node._db

    result = node.parameters_callback([_FakeParam("identify_threshold", 0.8)])

    assert result.successful is True
    assert node._db is db_before
    assert node._e2e_mode_active is False


def test_db_switch_failure_returns_unsuccessful_result(node, monkeypatch):
    """Провал переключения (диск недоступен и т. п.) обязан дойти до
    вызывающего ``ros2 param set`` как ``successful=False`` — иначе
    E2E-харнесс решит, что режим включён, хотя активная БД не менялась."""

    def _boom(_path):
        raise OSError("simulated disk failure")

    monkeypatch.setattr(sid_node, "SpeakerDatabase", _boom)

    result = node.parameters_callback(_e2e_mode_param(True))

    assert result.successful is False
    assert node._e2e_mode_active is False, "провал не должен был поменять состояние"


def _rand_embedding(seed: int):
    import numpy as np

    rng = np.random.default_rng(seed)
    v = rng.standard_normal(256).astype("float32")
    return v / np.linalg.norm(v)
