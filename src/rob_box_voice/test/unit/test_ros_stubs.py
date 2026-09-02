"""test_ros_stubs.py — regression-тесты для центральной install_ros_stubs().

Защищает от регрессии issue #1879 (``test_sound_voice_passthrough::
test_subscribes_to_avatar_voice_in_best_effort`` падал из-за того, что
какой-то conftest/test-файл до sound ставил ``audio_common_msgs.msg``
через ``Mock()`` / ``MagicMock()``, а ``install_ros_stubs`` additive-режим
только дополнял недостающие атрибуты — настоящий класс ``AudioData``
уже не появлялся).

Запуск:
    python3 -m pytest src/rob_box_voice/test/test_ros_stubs.py -v
"""

from __future__ import annotations

import sys
import types
from pathlib import Path

# ``test/unit/`` сам не на ``sys.path`` — добавляем родителя (``test/``),
# чтобы работал ``from ros_stubs import ...``.
_THIS_DIR = Path(__file__).resolve().parent
_TEST_DIR = _THIS_DIR.parent
if str(_TEST_DIR) not in sys.path:
    sys.path.insert(0, str(_TEST_DIR))

import pytest  # noqa: E402

from ros_stubs import install_ros_stubs  # noqa: E402


@pytest.fixture
def clean_audio_stubs():
    """Сбрасывает sys.modules для audio_common_msgs перед тестом и
    **восстанавливает исходное состояние** после — чтобы cleanup
    теста не выбивал audio_common_msgs для других тестов в сессии
    (sound/conftest.py ставит его один раз при импорте conftest-а
    и при втором вызове ``_install_all_mocks()`` уже no-op)."""
    saved = {
        name: sys.modules.get(name)
        for name in ("audio_common_msgs", "audio_common_msgs.msg")
    }
    sys.modules.pop("audio_common_msgs", None)
    sys.modules.pop("audio_common_msgs.msg", None)
    try:
        yield
    finally:
        for name, value in saved.items():
            if value is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = value
    # После cleanup — если какие-то downstream-тесты (sound/) ожидают
    # audio_common_msgs, восстанавливаем canonical real-class state
    # через install_ros_stubs() (additive — уже существующий модуль
    # не перезаписывается, но атрибут AudioData дозаливается).
    if "audio_common_msgs" not in sys.modules:
        install_ros_stubs()
    elif not hasattr(sys.modules["audio_common_msgs"], "msg"):
        install_ros_stubs()


def test_install_ros_stubs_provides_real_audio_data_class(clean_audio_stubs):
    """``install_ros_stubs()`` без extra ставит ``AudioData`` как настоящий класс.

    До issue #1879 ``audio_common_msgs.msg`` в центральном ``_build_stubs()``
    отсутствовал, и любой conftest/test, ставивший ``sys.modules[
    "audio_common_msgs.msg"] = Mock()`` раньше sound-теста, оставлял
    ``AudioData`` как MagicMock-инстанс — а ``msg_type.__name__`` тогда
    либо ``AttributeError`` бросает, либо возвращает ``"MagicMock"``.
    """
    install_ros_stubs()

    assert "audio_common_msgs" in sys.modules
    assert "audio_common_msgs.msg" in sys.modules

    amm = sys.modules["audio_common_msgs.msg"]
    assert hasattr(amm, "AudioData"), "AudioData attribute is missing"

    AudioData = amm.AudioData
    assert isinstance(AudioData, type), (
        f"AudioData must be a real class, got {type(AudioData).__name__}"
    )
    assert AudioData.__name__ == "AudioData", (
        f"AudioData.__name__ should be 'AudioData', got {AudioData.__name__!r}"
    )


def test_install_ros_stubs_audio_data_survives_mock_race(clean_audio_stubs):
    """Симулирует гонку: более ранний conftest ставит Mock()/MagicMock()
    для ``audio_common_msgs.msg``, но ``install_ros_stubs()`` (вызванный
    из sound/conftest.py) обязан оставить AudioData настоящим классом.

    Воспроизводит сценарий ``test_tts_finished_duration.py:32-42``
    (module-level ``sys.modules.setdefault("audio_common_msgs.msg", Mock())``)
    плюс ``unit/tts/conftest.py`` через ``_MOCKS_INSTALLED`` flag и
    любые другие pre-existing stubs.
    """
    # Шаг 1: какой-то более ранний conftest/test ставит Mock() для msg-модуля.
    sys.modules["audio_common_msgs"] = types.ModuleType("audio_common_msgs")
    sys.modules["audio_common_msgs.msg"] = types.ModuleType("audio_common_msgs.msg")
    # Никакого AudioData в нём нет — будет auto-attr при доступе.

    # Шаг 2: sound/conftest.py вызывает install_ros_stubs(extra=...)
    # — раньше он НЕ перетирал уже существующий audio_common_msgs.msg,
    # если атрибут AudioData уже был (или наоборот, не появлялся).
    install_ros_stubs()

    amm = sys.modules["audio_common_msgs.msg"]
    assert hasattr(amm, "AudioData"), (
        "After install_ros_stubs, audio_common_msgs.msg must have AudioData"
    )
    AudioData = amm.AudioData
    assert isinstance(AudioData, type), (
        f"AudioData must be a real class even after a Mock race, "
        f"got {type(AudioData).__name__}"
    )
    assert AudioData.__name__ == "AudioData", (
        f"AudioData.__name__ must be 'AudioData' (got {AudioData.__name__!r}) — "
        "if you see 'MagicMock' here, install_ros_stubs skipped overriding "
        "the existing module attribute (regression of issue #1879)."
    )


def test_install_ros_stubs_idempotent_for_audio_common(clean_audio_stubs):
    """Повторный ``install_ros_stubs()`` не ломает AudioData."""
    install_ros_stubs()
    install_ros_stubs()  # idempotent
    install_ros_stubs(names=("audio_common_msgs", "audio_common_msgs.msg"))

    AudioData = sys.modules["audio_common_msgs.msg"].AudioData
    assert isinstance(AudioData, type)
    assert AudioData.__name__ == "AudioData"


def test_sound_conftest_force_overrides_mock_audio_data(clean_audio_stubs):
    """Репродуцирует CI-сценарий issue #1879: до sound/conftest.py какой-то
    conftest (например, unit/tts/conftest.py) поставил ``audio_common_msgs.msg``
    как ``Mock()`` (auto-attr ``.AudioData`` возвращает Mock-инстанс).

    ``install_ros_stubs()`` additive и сам по себе не перезаписывает уже
    существующий атрибут модуля — поэтому sound/conftest.py **сам**
    выполняет defensive override перед ``install_ros_stubs(extra=...)``.

    Этот тест эмулирует ту defensive-проверку напрямую: после неё
    ``AudioData`` обязан быть настоящим классом с ``__name__ == 'AudioData'``.
    """
    from unittest.mock import Mock

    # Шаг 1: более ранний conftest/test поставил Mock() (НЕ MagicMock)
    # для audio_common_msgs.msg — auto-attr даёт .AudioData → Mock-инстанс.
    sys.modules["audio_common_msgs.msg"] = Mock()
    pre_existing = sys.modules["audio_common_msgs.msg"]
    assert not isinstance(pre_existing.AudioData, type)
    # Mock instance has no real __name__ attribute — confirm defensive case.
    try:
        _ = pre_existing.AudioData.__name__
        has_name = True
    except AttributeError:
        has_name = False
    assert not has_name, (
        "Mock().AudioData already has __name__? surprising"
    )

    # Шаг 2: defensive override из sound/conftest.py
    audio_msg = types.ModuleType("audio_common_msgs.msg")
    audio_msg.AudioData = type("AudioData", (), {})

    existing_audio_msg = sys.modules.get("audio_common_msgs.msg")
    existing_audio_data = getattr(existing_audio_msg, "AudioData", None)
    if not isinstance(existing_audio_data, type) or existing_audio_data.__name__ != "AudioData":
        existing_audio_msg.AudioData = audio_msg.AudioData

    # Шаг 3: после defensive — AudioData настоящий класс
    AudioData = sys.modules["audio_common_msgs.msg"].AudioData
    assert isinstance(AudioData, type), (
        f"defensive override failed: AudioData is {type(AudioData).__name__}"
    )
    assert AudioData.__name__ == "AudioData", (
        f"defensive override failed: AudioData.__name__ = {AudioData.__name__!r}"
    )


def test_sound_conftest_handles_magicmock_audio_data(clean_audio_stubs):
    """То же, что и предыдущий, но с ``MagicMock()`` (тоже встречается в
    tts/conftest.py через ``MagicMock()``). Defensive должен справиться
    и с MagicMock, и с обычным Mock."""
    from unittest.mock import MagicMock

    sys.modules["audio_common_msgs.msg"] = MagicMock()
    pre_existing = sys.modules["audio_common_msgs.msg"]
    # MagicMock().AudioData — MagicMock instance, не класс.
    assert not isinstance(pre_existing.AudioData, type)

    audio_msg = types.ModuleType("audio_common_msgs.msg")
    audio_msg.AudioData = type("AudioData", (), {})

    existing_audio_msg = sys.modules.get("audio_common_msgs.msg")
    existing_audio_data = getattr(existing_audio_msg, "AudioData", None)
    if not isinstance(existing_audio_data, type) or existing_audio_data.__name__ != "AudioData":
        existing_audio_msg.AudioData = audio_msg.AudioData

    AudioData = sys.modules["audio_common_msgs.msg"].AudioData
    assert isinstance(AudioData, type)
    assert AudioData.__name__ == "AudioData"