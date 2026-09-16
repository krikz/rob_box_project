"""Юнит-тесты для shared pre-flight helper (issue #2656, F-2).

Покрывает:
  1. ``make_preflight_check('vision_hailo')`` / ``make_preflight_check('vision_face')``
     возвращают разные замыкания, но с одинаковой семантикой.
  2. ``hailo_enabled=false`` → WARN'ов нет, возврат ``[]``.
  3. ``hailo_enabled=true`` без /dev/hailo0 (мы на dev-машине) → WARN с
     правильным ``component_prefix`` (``vision_hailo`` vs ``vision_face``)
     и причиной ``/dev/hailo0 отсутствует``.
  4. ``hailo_enabled=true`` + ``hef_path`` указывает на несуществующий
     файл → WARN с текстом ``hef_path=...`` и фразой про stub-режим.
  5. ``hailo_enabled=true`` + пустой ``hef_path`` → WARN про hef_path
     не генерируется (только device + deps).
  6. Acceptance #3: helper не зависит от launch-директории — фактические
     проверки (os.path.exists / __import__) перенесены в shared module,
     grep по launch/ даёт 0 вхождений ``os.path.exists.*hailo0|hailo_platform``.

Тесты НЕ требуют rclpy / HailoRT — используют ``unittest.mock`` для
подмены ``LaunchConfiguration.perform`` (как и другие unit-тесты в
test/unit/, см. test_vision_hailo_node.py).
"""

from __future__ import annotations

import builtins
import importlib
import io
import os
import re
import sys
from contextlib import redirect_stdout
from pathlib import Path
from typing import Any, Dict
from unittest import mock

import pytest


# ----- import target under test -----
# Тот же приём, что и в test_vision_hailo_node.py / test_vision_event_parity.py:
# parents[2] = .../src/rob_box_perception → содержит rob_box_perception/.
_HERE = Path(__file__).resolve()
_PKG_ROOT = _HERE.parents[2]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

# ``launch.substitutions.LaunchConfiguration`` НЕТ в окружении CI worktree
# (она приходит из ros2 pkg) → мокаем модуль launch, чтобы import прошёл.
_FAKE_LAUNCH_CONFIG_VALUES: Dict[str, str] = {}


def _fake_launch_config_perform(self, context):  # noqa: ARG001 — context unused
    """Возвращает значение по имени launch-аргумента."""
    name = getattr(self, '_fake_name', None) or self.__class__.__name__
    return _FAKE_LAUNCH_CONFIG_VALUES.get(name, '')


class _FakeLaunchConfiguration:
    def __init__(self, name: str):
        # Реальный класс хранит ``_name``, привязываем к фейк-перформу.
        self._fake_name = name

    def perform(self, context):  # noqa: ARG001 — context unused
        return _FAKE_LAUNCH_CONFIG_VALUES.get(self._fake_name, '')


_fake_launch_mod = mock.MagicMock()
_fake_launch_mod.LaunchConfiguration = _FakeLaunchConfiguration
sys.modules.setdefault('launch', mock.MagicMock())
sys.modules['launch.substitutions'] = mock.MagicMock()
sys.modules['launch.substitutions'].LaunchConfiguration = _FakeLaunchConfiguration

preflight = importlib.import_module('rob_box_perception.preflight')


# ----- helpers -----

def _set_hailo_enabled(value: str) -> None:
    _FAKE_LAUNCH_CONFIG_VALUES['hailo_enabled'] = value


def _set_hef_path(value: str) -> None:
    _FAKE_LAUNCH_CONFIG_VALUES['hef_path'] = value


def _clear_values() -> None:
    _FAKE_LAUNCH_CONFIG_VALUES.clear()


@pytest.fixture(autouse=True)
def _reset_launch_config():
    _clear_values()
    yield
    _clear_values()


def _run_preflight(prefix: str) -> tuple[list[str], list[Any]]:
    """Запустить preflight для заданного префикса, вернуть (warnings, return_value)."""
    fn = preflight.make_preflight_check(prefix)
    buf = io.StringIO()
    with redirect_stdout(buf):
        result = fn(context=None)
    output = buf.getvalue()
    warnings = re.findall(r'^\[WARN\] (.+)$', output, flags=re.MULTILINE)
    return warnings, result


# ----- tests -----

def test_make_preflight_returns_callable():
    """Фабрика должна вернуть callable."""
    fn = preflight.make_preflight_check('vision_hailo')
    assert callable(fn)


def test_different_prefixes_return_distinct_closures():
    """Два вызова с разными prefix — разные объекты-замыкания."""
    a = preflight.make_preflight_check('vision_hailo')
    b = preflight.make_preflight_check('vision_face')
    assert a is not b


def test_hailo_disabled_emits_nothing():
    """hailo_enabled=false → WARN'ов нет (capability-honest — только при true)."""
    _set_hailo_enabled('false')
    warnings, result = _run_preflight('vision_hailo')
    assert warnings == []
    assert result == []


def test_hailo_disabled_uppercase_false_also_quiet():
    """Защита от регистра: 'False' / 'FALSE' → тоже ничего не делаем."""
    for value in ('False', 'FALSE', 'false'):
        _clear_values()
        _set_hailo_enabled(value)
        warnings, _ = _run_preflight('vision_face')
        assert warnings == [], f'expected quiet for hailo_enabled={value!r}'


def test_hailo_enabled_true_missing_hailo0_warns():
    """hailo_enabled=true + нет /dev/hailo0 (наш случай на dev) → WARN с prefix."""
    _set_hailo_enabled('true')
    _set_hef_path('')

    warnings, result = _run_preflight('vision_hailo')

    # На CI-окружении без /dev/hailo0 должно быть хотя бы одно предупреждение.
    # Не делаем assert на наличие device — если вдруг у разраба есть dev board,
    # тест не должен падать.
    joined = '\n'.join(warnings)
    assert 'vision_hailo.preflight:' in joined
    assert result == []
    # Если /dev/hailo0 нет — должна быть хотя бы эта строка.
    if not os.path.exists('/dev/hailo0'):
        assert any('/dev/hailo0' in w for w in warnings), (
            f'expected /dev/hailo0 warning, got: {warnings!r}'
        )


def test_prefix_propagates_to_warnings():
    """vision_face и vision_hailo WARN'ят каждый со своим prefix'ом."""
    _set_hailo_enabled('true')
    _set_hef_path('/nonexistent/path/that/does/not/exist.hef')

    face_warnings, _ = _run_preflight('vision_face')
    hailo_warnings, _ = _run_preflight('vision_hailo')

    assert any('vision_face.preflight:' in w for w in face_warnings)
    assert any('vision_hailo.preflight:' in w for w in hailo_warnings)
    assert not any('vision_hailo.preflight:' in w for w in face_warnings)
    assert not any('vision_face.preflight:' in w for w in hailo_warnings)


def test_hef_path_nonexistent_warns():
    """hailo_enabled=true + hef_path указывает на несуществующий файл → WARN."""
    _set_hailo_enabled('true')
    fake_hef = '/tmp/this_hef_does_not_exist_12345.hef'
    _set_hef_path(fake_hef)

    warnings, _ = _run_preflight('vision_hailo')
    assert any(fake_hef in w and 'hef_path=' in w for w in warnings), (
        f'expected hef_path warning for {fake_hef}, got: {warnings!r}'
    )


def test_hef_path_empty_no_hef_warning():
    """Пустой hef_path (по умолчанию) → НЕ должно быть WARN про hef_path."""
    _set_hailo_enabled('true')
    _set_hef_path('')

    warnings, _ = _run_preflight('vision_hailo')
    assert not any('hef_path=' in w for w in warnings), (
        f'empty hef_path should not warn, got: {warnings!r}'
    )


def test_missing_python_dep_warns(monkeypatch):
    """Если один из deps не импортируется → WARN с именем модуля."""
    _set_hailo_enabled('true')
    _set_hef_path('')

    real_import = builtins.__import__

    def _fake_import(name, *args, **kwargs):
        if name == 'cv2':
            raise ImportError('simulated missing cv2')
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, '__import__', _fake_import)

    warnings, _ = _run_preflight('vision_hailo')
    assert any('cv2 не установлен' in w for w in warnings), (
        f'expected cv2 dep warning, got: {warnings!r}'
    )
    # 'cv2 не установлен' содержит prefix 'vision_hailo.preflight:'
    assert any('vision_hailo.preflight:' in w and 'cv2' in w for w in warnings)


def test_preflight_does_not_block_launch():
    """Возвращаемое значение всегда [] (OpaqueFunction-контракт: не блокирует)."""
    _set_hailo_enabled('true')
    _set_hef_path('/nonexistent.hef')

    warnings, result = _run_preflight('vision_face')
    assert result == [], 'preflight must not return actions (capability-honest)'


def test_acceptance_launch_dir_no_hailo_paths():
    """Acceptance #3 (issue #2656): launch/ не должен содержать дубликатов логики.

    Проверяем, что в ``launch/`` нет ``os.path.exists('/dev/hailo0')``
    и нет ``'hailo_platform'`` в deps (только в shared ``preflight.py``).
    """
    launch_dir = _HERE.parents[2] / 'launch'
    pattern = re.compile(
        r"os\.path\.exists.*hailo0|'hailo_platform'",
        flags=re.MULTILINE,
    )
    offenders = []
    for p in launch_dir.glob('*.launch.py'):
        text = p.read_text(encoding='utf-8')
        if pattern.search(text):
            offenders.append(p.name)

    assert offenders == [], (
        f'launch/ files still contain preflight logic: {offenders!r}. '
        f'Issue #2656 acceptance #3: should live in rob_box_perception.preflight.'
    )


def test_both_launch_files_import_shared_helper():
    """Acceptance #1: оба launch-файла импортируют shared make_preflight_check."""
    launch_dir = _HERE.parents[2] / 'launch'
    expected = {'vision_face.launch.py', 'vision_hailo.launch.py'}

    for name in expected:
        path = launch_dir / name
        text = path.read_text(encoding='utf-8')
        assert 'from rob_box_perception.preflight import make_preflight_check' in text, (
            f'{name} must import make_preflight_check from shared preflight module'
        )
        assert 'make_preflight_check(' in text, (
            f'{name} must call make_preflight_check(...) for OpaqueFunction'
        )
        # Acceptance #2: локальный _preflight_check удалён
        assert 'def _preflight_check(' not in text, (
            f'{name} should not define local _preflight_check anymore'
        )