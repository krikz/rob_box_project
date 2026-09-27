"""Шов «Ускоритель»: кто владеет Hailo и кто активирует пайплайн.

Проверено на живом роботе 15.09.2026 (issue #2599) и зафиксировано здесь,
чтобы правило не отъехало обратно:

- сокет ``hailort_service`` достижим → идём через сервис
  (``multi_process_service`` + общий ``group_id``), иначе два продюсера
  дерутся за устройство и проигравший получает
  ``HAILO_OUT_OF_PHYSICAL_DEVICES(74)``;
- под сервисом ручной ``activate()`` ЗАПРЕЩЁН (HailoRT:
  "activate function is not supported when using multi-process service
  or HailoRT Scheduler" → ``HAILO_INVALID_OPERATION(6)``);
- без сервиса ``activate()`` наоборот ОБЯЗАТЕЛЕН (иначе
  ``HAILO_STREAM_NOT_ACTIVATED(72)``, issue #2398).

Тесты pure-Python: ``hailo_platform`` подменяется фейком, железо не нужно.
"""

from __future__ import annotations

import importlib
import sys
import types
from pathlib import Path
from typing import Any, Dict, List

import pytest

# `parents[2]` = .../src/rob_box_perception → содержит пакет rob_box_perception/.
_PKG_ROOT = Path(__file__).resolve().parents[2]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))
hailo_device = importlib.import_module('rob_box_perception.hailo_device')


class _FakeParams:
    def __init__(self) -> None:
        self.assigned: Dict[str, Any] = {}

    def __setattr__(self, name: str, value: Any) -> None:
        if name != 'assigned':
            self.assigned[name] = value
        object.__setattr__(self, name, value)


class _FakeVDevice:
    created: List[Any] = []

    def __init__(self, params: Any = None) -> None:
        self.params = params
        _FakeVDevice.created.append(params)

    @staticmethod
    def create_params() -> _FakeParams:
        return _FakeParams()


def _install_fake_hailo(monkeypatch: pytest.MonkeyPatch, *, modern: bool = True):
    module = types.ModuleType('hailo_platform')
    module.VDevice = _FakeVDevice  # type: ignore[attr-defined]
    if modern:
        algos = types.SimpleNamespace(ROUND_ROBIN='round-robin')
        module.HailoSchedulingAlgorithm = algos  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, 'hailo_platform', module)
    _FakeVDevice.created = []
    return module


def test_service_socket_present_goes_through_service(
    monkeypatch: pytest.MonkeyPatch, tmp_path
) -> None:
    _install_fake_hailo(monkeypatch)
    sock = tmp_path / 'hailort_uds.sock'
    sock.write_text('')
    monkeypatch.setenv('HAILORT_SERVICE_SOCKET', str(sock))
    monkeypatch.setenv('HAILO_GROUP_ID', 'rob_box_test')

    device = hailo_device.open_device()

    assert device.mode == hailo_device.MODE_SERVICE
    assert device.scheduler_managed is True
    # Под планировщиком активация — не наша забота.
    assert device.must_activate is False
    assigned = device.vdevice.params.assigned
    assert assigned['multi_process_service'] is True
    assert assigned['group_id'] == 'rob_box_test'
    assert assigned['scheduling_algorithm'] == 'round-robin'
    # device_id несовместим с групповым владением — не выставляем.
    assert 'device_id' not in assigned


def test_no_socket_falls_back_to_exclusive_ownership(
    monkeypatch: pytest.MonkeyPatch, tmp_path
) -> None:
    _install_fake_hailo(monkeypatch)
    monkeypatch.setenv(
        'HAILORT_SERVICE_SOCKET', str(tmp_path / 'absent.sock')
    )

    device = hailo_device.open_device(device_id=0)

    assert device.mode == hailo_device.MODE_EXCLUSIVE
    assert device.scheduler_managed is False
    # Единоличное владение → activate() обязателен (#2398).
    assert device.must_activate is True
    assigned = device.vdevice.params.assigned
    assert assigned['device_id'] == '0'
    assert 'multi_process_service' not in assigned


def test_legacy_hailort_without_scheduler_api(
    monkeypatch: pytest.MonkeyPatch, tmp_path
) -> None:
    _install_fake_hailo(monkeypatch, modern=False)
    sock = tmp_path / 'hailort_uds.sock'
    sock.write_text('')
    monkeypatch.setenv('HAILORT_SERVICE_SOCKET', str(sock))

    device = hailo_device.open_device()

    assert device.mode == hailo_device.MODE_LEGACY
    assert device.must_activate is True
    assert device.vdevice.params is None


def test_missing_hailo_platform_raises_explicit_import_error(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setitem(sys.modules, 'hailo_platform', None)

    with pytest.raises(ImportError) as exc:
        hailo_device.open_device()

    assert 'HailoRT' in str(exc.value)


def test_service_available_checks_socket_not_systemd(
    monkeypatch: pytest.MonkeyPatch, tmp_path
) -> None:
    # Сервис может быть жив на хосте, но не смонтирован в контейнер —
    # снаружи это «сервис есть», внутри решает именно сокет.
    absent = tmp_path / 'nope.sock'
    assert hailo_device.service_available(str(absent)) is False
    absent.write_text('')
    assert hailo_device.service_available(str(absent)) is True
