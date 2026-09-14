"""Unit-тесты для vision_hailo_loader (ADR-0089 Phase 1).

Запуск:
    pytest src/rob_box_perception/test/unit/test_vision_hailo_node.py -v

Что покрываем:
  1. StubHEFLoader: период между событиями.
  2. StubHEFLoader: точность формата события (поля, типы).
  3. make_loader: factory routing (hailo_enabled+hef_path -> Real; else Stub).
  4. RealHEFLoader.infer: без hailo_platform -> ImportError (fail loudly).
  5. RealHEFLoader.infer: с hailo_platform mock + missing HEF -> FileNotFoundError.
  6. filter_by_confidence: фильтр по порогу (>= threshold).
  7. normalize_event_dict: defaults + типы + coercion (round-trip).

NOTE (ADR-0089, decision Q2=A): `is_available()` удалён из `HEFLoader`
interface — это был мёртвый член (маршрутизация идёт через фабрику
`make_loader`). Phase 1.5, когда `RealHEFLoader` станет настоящим,
решит, нужен ли runtime availability check и где он должен жить.
Соответственно, тесты на `is_available` (test_stub_loader_is_available,
test_real_loader_without_hailort_returns_false,
test_real_loader_missing_hef_returns_false) заменены на тесты
поведения `infer()` в failure-режимах — они и есть deletion test.

Эти тесты НЕ требуют rclpy / HailoRT / железа — поэтому могут
запускаться в CI как обычный pytest (это и есть смысл разделения
loader-модуля от ROS-узла).
"""

from __future__ import annotations

import importlib
import sys
import time
from pathlib import Path
from typing import Any, Dict, List

import pytest


# ---------- import target under test ------------------------------------

def _import_module():
    # Путь к пакету вычисляется относительно этого файла (НЕ хардкод пути
    # build-машины) — тесты должны одинаково проходить на dev-машине и
    # self-hosted CI.
    pkg_root = str(Path(__file__).resolve().parents[2])
    if pkg_root not in sys.path:
        sys.path.insert(0, pkg_root)
    return importlib.import_module('rob_box_perception.vision_hailo_loader')


loader_mod = _import_module()


# ---------- tests ---------------------------------------------------------


# NOTE: тесты на `is_available()` удалены вместе с методом (decision Q2=A).
# Раньше первый тест проверял `StubHEFLoader.is_available() is True` —
# контракт stub "всегда готов" теперь покрывается factory-routing
# (`make_loader(hailo_enabled=False)` -> Stub) и тестом
# `test_make_loader_factory_routing` ниже.


def test_stub_loader_periodicity():
    """Stub loader не должен emit'ить чаще, чем period_sec."""
    period = 0.2
    stub = loader_mod.StubHEFLoader(period_sec=period)
    # Первый вызов сразу же — должен дать событие (last_emit=0, now > period).
    first = stub.infer(frame_id='test_cam', image=None)
    assert len(first) == 1
    # Второй сразу после первого — пустой (интервал слишком мал).
    second = stub.infer(frame_id='test_cam', image=None)
    assert second == []
    # После ожидания — снова событие.
    time.sleep(period * 1.2)
    third = stub.infer(frame_id='test_cam', image=None)
    assert len(third) == 1


def test_stub_loader_event_shape():
    """Stub event имеет все обязательные поля VisionEvent."""
    stub = loader_mod.StubHEFLoader(period_sec=0.05)
    time.sleep(0.1)  # гарантируем, что emit произойдёт
    events = stub.infer(frame_id='oak_rgb_frame', image=None)
    assert len(events) == 1
    ev = events[0]
    required = {
        'source_camera', 'event_type', 'class_name', 'class_id',
        'confidence', 'bbox_cx', 'bbox_cy', 'bbox_w', 'bbox_h',
        'distance_m', 'embedding_id', 'display_name', 'attributes_json',
    }
    missing = required - set(ev.keys())
    assert not missing, f'Отсутствуют поля: {missing}'
    # Конкретные sanity-checks на stub values.
    assert ev['event_type'] == 'person'
    assert ev['class_name'] == 'person'
    assert ev['confidence'] == pytest.approx(0.92, abs=0.01)
    assert 0.0 <= ev['bbox_cx'] <= 1.0
    assert 0.0 <= ev['bbox_cy'] <= 1.0


def test_make_loader_factory_routing():
    """factory выбирает Real когда hailo_enabled+hef_path, иначе Stub."""
    stub = loader_mod.make_loader(
        hailo_enabled=False, hef_path=None, stub_period_sec=1.0
    )
    assert isinstance(stub, loader_mod.StubHEFLoader)

    real_no_hef = loader_mod.make_loader(
        hailo_enabled=True, hef_path=None, stub_period_sec=1.0
    )
    # hailo_enabled=True, но hef_path пустой -> Stub (per contract).
    assert isinstance(real_no_hef, loader_mod.StubHEFLoader)

    real = loader_mod.make_loader(
        hailo_enabled=True,
        hef_path='/tmp/nonexistent.hef',
        stub_period_sec=1.0,
    )
    # hailo_enabled=True + hef_path -> RealHEFLoader.
    assert isinstance(real, loader_mod.RealHEFLoader)


def test_real_loader_without_hailort_raises(monkeypatch):
    """Verify RealHEFLoader.infer() raises ImportError without hailo_platform (fail loudly).

    Замена test_real_loader_without_hailort_returns_false.
    Раньше is_available() глотала ImportError и возвращала False; теперь
    (после удаления capability-probing) `infer()` обязан поднять
    исключение, чтобы нода через `_tick -> except` залогировала ошибку
    и перешла в no-events режим. Это и есть ADR-0018 "capability-honest"
    контракт: НЕ молча fallback'ить, а падать громко.
    """
    import builtins
    original_import = builtins.__import__

    def fake_import(name, *args, **kwargs):
        if name == 'hailo_platform' or name.startswith('hailo_platform'):
            raise ImportError('hailo_platform not available (test mock)')
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, '__import__', fake_import)
    real = loader_mod.RealHEFLoader(hef_path='/tmp/fake.hef')
    with pytest.raises(ImportError):
        real.infer(frame_id='test_cam', image=None)


def test_real_loader_missing_hef_raises(monkeypatch):
    """Verify RealHEFLoader.infer() raises FileNotFoundError for missing HEF (fail loudly).

    Замена test_real_loader_missing_hef_returns_false. Здесь мы
    подсовываем фейковый `hailo_platform`, чтобы дойти до проверки
    `os.path.isfile(hef_path)` внутри `_ensure_initialized` —
    именно она должна поднять FileNotFoundError.
    """
    import sys
    import types

    # Подсовываем минимальный stub модуля hailo_platform, чтобы
    # `_ensure_initialized` прошёл import-check и дошёл до FileNotFoundError.
    # Используем setattr — ModuleType не декларирует произвольные атрибуты
    # в типизации, но в runtime это легальный паттерн (см. types.ModuleType).
    fake_hp = types.ModuleType('hailo_platform')
    setattr(fake_hp, 'VDevice', object)         # placeholder
    setattr(fake_hp, 'HailoRTException', Exception)
    monkeypatch.setitem(sys.modules, 'hailo_platform', fake_hp)

    real = loader_mod.RealHEFLoader(hef_path='/definitely/not/a/real/path.hef')
    with pytest.raises(FileNotFoundError):
        real.infer(frame_id='test_cam', image=None)


def test_filter_by_confidence_threshold():
    """Фильтрация по confidence_threshold должна работать детерминированно."""
    events: List[Dict[str, Any]] = [
        {'confidence': 0.9, 'class_name': 'keep_1'},
        {'confidence': 0.4, 'class_name': 'drop'},
        {'confidence': 0.5, 'class_name': 'keep_2'},  # ровно порог — keep
        {'confidence': 0.0, 'class_name': 'drop_2'},
        {'confidence': 0.7, 'class_name': 'keep_3'},
    ]
    result = loader_mod.filter_by_confidence(events, threshold=0.5)
    names = [e['class_name'] for e in result]
    assert names == ['keep_1', 'keep_2', 'keep_3']

    # Другой порог.
    result_hi = loader_mod.filter_by_confidence(events, threshold=0.7)
    names_hi = [e['class_name'] for e in result_hi]
    assert names_hi == ['keep_1', 'keep_3']


def test_normalize_event_dict_round_trip():
    """normalize_event_dict возвращает типизированный dict.

    Контракт:
      - все поля присутствуют (даже если не переданы, с дефолтами)
      - строки -> str, числа -> float/int с правильными дефолтами
      - round-trip одного и того же значения сохраняется
    """
    src = {
        'source_camera': 'oak-d',
        'event_type': 'object',
        'class_name': 'cup',
        'class_id': 41,
        'confidence': 0.83,
        'bbox_cx': 0.5,
        'bbox_cy': 0.6,
        'bbox_w': 0.2,
        'bbox_h': 0.3,
        'distance_m': 1.5,
        'embedding_id': '',
        'display_name': '',
        'attributes_json': '{}',
    }
    out = loader_mod.normalize_event_dict(src)
    # Каждое поле round-trip'нулось с правильным типом.
    assert out['source_camera'] == 'oak-d'
    assert isinstance(out['source_camera'], str)
    assert out['event_type'] == 'object'
    assert out['class_name'] == 'cup'
    assert out['class_id'] == 41
    assert isinstance(out['class_id'], int)
    assert out['confidence'] == pytest.approx(0.83)
    assert isinstance(out['confidence'], float)
    assert out['bbox_cx'] == pytest.approx(0.5)
    assert isinstance(out['bbox_cx'], float)
    assert out['bbox_w'] == pytest.approx(0.2)
    assert out['bbox_h'] == pytest.approx(0.3)
    assert out['distance_m'] == pytest.approx(1.5)
    assert isinstance(out['distance_m'], float)
    assert out['embedding_id'] == ''
    assert out['display_name'] == ''
    assert out['attributes_json'] == '{}'


def test_normalize_event_dict_defaults():
    """Пустой dict -> все дефолты по контракту VisionEvent."""
    out = loader_mod.normalize_event_dict({})
    assert out == {
        'source_camera': '',
        'event_type': 'scene',
        'class_name': '',
        'class_id': -1,
        'confidence': 0.0,
        'bbox_cx': -1.0,
        'bbox_cy': -1.0,
        'bbox_w': -1.0,
        'bbox_h': -1.0,
        'distance_m': -1.0,
        'embedding_id': '',
        'display_name': '',
        'attributes_json': '',
    }
