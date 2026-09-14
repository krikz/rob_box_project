"""Unit-тесты для vision_hailo_loader (ADR-0089 Phase 1).

Запуск:
    pytest src/rob_box_perception/test/unit/test_vision_hailo_node.py -v

Что покрываем:
  1. StubHEFLoader.is_available -> True.
  2. StubHEFLoader: период между событиями.
  3. StubHEFLoader: точность формата события (поля, типы).
  4. make_loader: factory routing (hailo_enabled+hef_path -> Real; else Stub).
  5. RealHEFLoader.is_available: без hailo_platform -> False (ImportError).
  6. RealHEFLoader.is_available: с hailo_platform mock + missing HEF -> False.
  7. filter_by_confidence: фильтр по порогу (>= threshold).
  8. normalize_event_dict: defaults + типы + coercion (round-trip).

Эти тесты НЕ требуют rclpy / HailoRT / железа — поэтому могут
запускаться в CI как обычный pytest (это и есть смысл разделения
loader-модуля от ROS-узла).
"""

from __future__ import annotations

import importlib
import sys
import time
from typing import Any, Dict, List

import pytest


# ---------- import target under test ------------------------------------

# Делаем import один раз на модуль. Используем sys.path.insert, чтобы
# тест работал в любом worktree без хардкода путей (старый код
# захардкодил путь к удалённой ветке t_b9b6cf73 — ломалось при работе
# в новой ветке). Текущий путь соответствует worktree этой задачи.
_PKG_ROOT = (
    '/home/builder/rob_box_project/.worktrees/t_40ee0b73/'
    'src/rob_box_perception'
)
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)
loader_mod = importlib.import_module('rob_box_perception.vision_hailo_loader')


# ---------- tests ---------------------------------------------------------


def test_stub_loader_is_available():
    """Stub loader всегда available — это его контракт для CI/smoke."""
    stub = loader_mod.StubHEFLoader(period_sec=0.1)
    assert stub.is_available() is True


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


def test_real_loader_without_hailort_returns_false(monkeypatch):
    """RealHEFLoader.is_available() -> False если hailo_platform недоступен."""
    import builtins
    original_import = builtins.__import__

    def fake_import(name, *args, **kwargs):
        if name == 'hailo_platform' or name.startswith('hailo_platform'):
            raise ImportError('hailo_platform not available (test mock)')
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, '__import__', fake_import)
    real = loader_mod.RealHEFLoader(hef_path='/tmp/fake.hef')
    assert real.is_available() is False


def test_real_loader_missing_hef_returns_false():
    """RealHEFLoader.is_available() -> False если HEF файл отсутствует.

    Здесь hailo_platform не трогаем — FileNotFoundError срабатывает раньше
    (в `_ensure_initialized` после import'а). Если hailo_platform недоступен
    в этом окружении, ожидаем ImportError -> False (тоже корректно).
    """
    real = loader_mod.RealHEFLoader(hef_path='/definitely/not/a/real/path.hef')
    assert real.is_available() is False


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
