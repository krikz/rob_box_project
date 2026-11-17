"""Контракт ``_ros_stubs.RosStubs``: заглушки не переживают свой модуль."""

import sys
import types

from ._ros_stubs import RosStubs

_NAME = "_rob_box_test_fake_ros_pkg"

_ros = RosStubs([_NAME])
with _ros:
    import _rob_box_test_fake_ros_pkg as _imported  # noqa: F401
_LEFT_AFTER_IMPORT = _NAME in sys.modules
_ros_stubs = _ros.fixture()


def test_stub_removed_right_after_import():
    """Файлы, собранные после этого, заглушку не увидят."""
    assert not _LEFT_AFTER_IMPORT


def test_same_stub_active_during_module_tests():
    """Ленивый импорт в коде инструмента получит тот же объект."""
    assert sys.modules[_NAME] is _imported


def test_loaded_module_is_not_replaced():
    name = _NAME + "_real"
    real = types.ModuleType(name)
    sys.modules[name] = real
    try:
        with RosStubs([name]):
            assert sys.modules[name] is real
        assert sys.modules[name] is real
    finally:
        del sys.modules[name]
