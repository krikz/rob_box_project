"""Тесты сводки `[лицо]` в ``VisionFaceNode._log_stats`` (issue #2777, PR #2777).

Контекст: 22.09 выкатили фикс лицевого узнавания (issue #2771-2775) на
живого робота. Реальная строка из его лога —

    [лицо] режим=workshop встреч=3 узнано=0 новых=1 слияний=2
    (пропуски: голос_не_опознан=2 нет_лица=1 конфликт_профилей=1)
    ошибок_эмбеддинга=0 кропов_отброшено=59 треков=0 | в базе: людей=3
    с_именем=2 gallery_cohesion=н/д enroll_отклонено=0

— не отвечала на два вопроса разбора инцидента: какой ГЕЙТ съел 59 из 59
кропов (``crop_rejected_by_reason`` уже считался в ``FaceRecognizer``, но
не печатался), и работает ли вообще выравнивание по landmark'ам на живом
HEF (``align_used_total``/``align_fallback_total`` — тоже посчитаны, тоже
не печатались; issue #2773 — единственный пункт PR, не измеренный на
железе). Этот файл проверяет только ФОРМАТИРОВАНИЕ этих чисел в лог —
не поведение ``FaceRecognizer`` (оно покрыто ``test_face_recognition.py``)
и не сеть ROS2.

``VisionFaceNode`` наследует ``VisionHailoNode``, который безусловно
делает ``import rclpy`` на уровне модуля (см. ``vision_hailo_node.py``).
На dev-машине без ROS2 этот импорт падает — тот же барьер, который
``test_perception_bridge.py`` и ``test_context_aggregator.py`` обходят
минимальным shim'ом ``rclpy``/``rclpy.node`` в ``sys.modules`` ДО импорта
модуля ноды; повторяем их приём здесь (см. их докстринги), а не
изобретаем новый. ``test_vision_face_env_namespace.py`` в этом же
каталоге обходит ту же проблему по-другому — вообще не импортирует
``vision_face_node.py`` (голый text/regex-скан compose/yaml/shell) — этот
трюк здесь не подходит, потому что нам нужно вызвать настоящий метод
Python, а не проверить текст файла.

``_log_stats`` при этом дёргает только четыре атрибута ``self``
(``_recognizer``, ``_speaker_unknown_total``, ``_log_stats_calls``,
``get_logger()``) — никакой ``Node.__init__`` не нужен. Вызываем его как
незабинженную функцию класса на лёгком дублёре (``_FakeNode``), не
конструируя ``VisionFaceNode`` целиком: конструктор тянет HEF-лоадер,
параметры ROS и подписки, которые к форматированию строки лога отношения
не имеют.

Запуск:
    python -m pytest src/rob_box_perception/test/unit/test_vision_face_log_stats.py -q --no-cov
"""

from __future__ import annotations

import importlib
import sys
import types
from pathlib import Path
from typing import Any, Dict, List
from unittest.mock import MagicMock

import pytest

# ---------- import target under test (тот же приём, что test_face_recognition.py /
# test_face_tracker.py — пакет не установлен на dev-машине, добавляем его
# корень в sys.path вручную вместо надежды на PYTHONPATH) --------------------

_HERE = Path(__file__).resolve()
_PKG_ROOT = _HERE.parents[2]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

# ── rclpy shim (мирует test_perception_bridge.py / test_context_aggregator.py) ──
if 'rclpy' not in sys.modules:
    _rclpy = types.ModuleType('rclpy')
    _rclpy.init = lambda *a, **k: None
    _rclpy.spin = lambda *a, **k: None
    _rclpy.try_shutdown = lambda *a, **k: None
    _rclpy_node = types.ModuleType('rclpy.node')

    class _StubNode:  # pragma: no cover — только чтобы `from rclpy.node import Node` не упал
        def __init__(self, *a: Any, **k: Any) -> None:
            pass

    _rclpy_node.Node = _StubNode
    _rclpy.node = _rclpy_node
    sys.modules['rclpy'] = _rclpy
    sys.modules['rclpy.node'] = _rclpy_node

m = importlib.import_module('rob_box_perception.vision_face_node')


class _FakeLogger:
    """Захватывает ``.info``/``.warning`` вместо реального rclpy-логгера."""

    def __init__(self) -> None:
        self.info_lines: List[str] = []
        self.warning_lines: List[str] = []

    def info(self, msg: str) -> None:
        self.info_lines.append(msg)

    def warning(self, msg: str) -> None:
        self.warning_lines.append(msg)


class _FakeNode:
    """Дублёр ``VisionFaceNode`` — ровно те атрибуты, что трогает ``_log_stats``."""

    def __init__(self, stats: Dict[str, Any]) -> None:
        self._recognizer = MagicMock()
        self._recognizer.stats.return_value = stats
        self._speaker_unknown_total = 0
        self._log_stats_calls = 0
        self._logger = _FakeLogger()

    def get_logger(self) -> _FakeLogger:
        return self._logger


def _run_log_stats(stats: Dict[str, Any], calls: int = 1) -> _FakeNode:
    """Вызывает настоящий ``VisionFaceNode._log_stats`` на дублёре ``calls`` раз."""
    node = _FakeNode(stats)
    for _ in range(calls):
        m.VisionFaceNode._log_stats(node)
    return node


def _base_stats(**overrides: Any) -> Dict[str, Any]:
    """Минимальный валидный ``FaceRecognizer.stats()`` — как в его докстринге."""
    stats: Dict[str, Any] = {
        'encounters_total': 0,
        'recognized_total': 0,
        'new_people_total': 0,
        'voice_merges_total': 0,
        'voice_merge_skip_no_face': 0,
        'voice_merge_skip_multi_face': 0,
        'voice_merge_skip_stale': 0,
        'voice_merge_skip_conflict': 0,
        'embed_failures': 0,
        'embed_skipped_budget': 0,
        'crop_rejected_total': 0,
        'crop_rejected_by_reason': {'dark': 0, 'flat': 0, 'clipped': 0, 'blurry': 0},
        'align_used_total': 0,
        'align_fallback_total': 0,
        'active_tracks': 0,
        'store': {
            'mode': 'workshop',
            'people': 0,
            'named': 0,
            'gallery_cohesion': None,
            'enroll_rejected_total': 0,
            # issue #2771 — прогрев галереи. На том прогоне его ещё не
            # было; здесь он в нуле именно потому, что все три записи
            # застряли на семени и ни один эмбеддинг не дописался.
            'enroll_warmup_total': 0,
            'gallery_warmup_size': 5,
        },
    }
    stats.update(overrides)
    return stats


# ---------------------------------------------------------------------------
# _format_crop_reject_suffix — та же «печатать только ненулевое», что и
# voice_merge_skip_* (issue #2748), применённая к причинам отказа кропа.
# ---------------------------------------------------------------------------

def test_crop_reject_suffix_prints_only_nonzero_reasons():
    suffix = m._format_crop_reject_suffix(
        {'dark': 0, 'flat': 0, 'clipped': 51, 'blurry': 8}
    )
    assert suffix == ' (clipped=51 blurry=8)'


def test_crop_reject_suffix_empty_when_all_zero():
    assert m._format_crop_reject_suffix({'dark': 0, 'flat': 0, 'clipped': 0, 'blurry': 0}) == ''


def test_crop_reject_suffix_empty_when_dict_empty():
    assert m._format_crop_reject_suffix({}) == ''


def test_crop_reject_suffix_keeps_english_reason_keys():
    """Решение: причины НЕ переводятся на русский (см. докстринг функции) —
    они те же ключи, что в тестах FaceRecognizer и в crop_rejected_by_reason."""
    suffix = m._format_crop_reject_suffix({'dark': 3})
    assert 'dark=3' in suffix
    assert 'тёмный' not in suffix and 'тёмно' not in suffix


# ---------------------------------------------------------------------------
# _align_dead_warning — троттлинг предупреждения «align_used=0».
# ---------------------------------------------------------------------------

def test_align_dead_warning_fires_on_first_call():
    assert m._align_dead_warning(0, 118, 1) is not None


def test_align_dead_warning_silent_between_periods():
    for call in range(2, m.ALIGN_DEAD_WARNING_PERIOD_CALLS):
        assert m._align_dead_warning(0, 118, call) is None, call


def test_align_dead_warning_repeats_every_period():
    period = m.ALIGN_DEAD_WARNING_PERIOD_CALLS
    assert m._align_dead_warning(0, 118, period + 1) is not None


def test_align_dead_warning_silent_when_align_actually_used():
    assert m._align_dead_warning(5, 118, 1) is None


def test_align_dead_warning_silent_when_no_fallback_either():
    """align_used=0 и align_fallback=0 — просто ни одного кадра с лицом ещё
    не было (нода только запустилась), не диагноз "выравнивание умерло"."""
    assert m._align_dead_warning(0, 0, 1) is None


# ---------------------------------------------------------------------------
# _log_stats — интеграция форматирования (без настоящего ROS2/rclpy).
# ---------------------------------------------------------------------------

def test_log_stats_reproduces_robot_incident_line():
    """Числа с реального робота (22.09): 59 отказов из 3 встреч, 2 трека
    умерли без эмбеддинга — воспроизводим их и проверяем новую разбивку."""
    stats = _base_stats(
        encounters_total=3,
        recognized_total=0,
        new_people_total=1,
        voice_merges_total=2,
        voice_merge_skip_no_face=1,
        voice_merge_skip_conflict=1,
        crop_rejected_total=59,
        crop_rejected_by_reason={'dark': 0, 'flat': 0, 'clipped': 51, 'blurry': 8},
        align_used_total=0,
        align_fallback_total=118,
        active_tracks=0,
        store={
            'mode': 'workshop',
            'people': 3,
            'named': 2,
            'gallery_cohesion': None,
            'enroll_rejected_total': 0,
            # issue #2771 — прогрев галереи. На том прогоне его ещё не
            # было; ноль здесь не «не сработал», а «не существовал»: все
            # записи застряли на семени, дозаписей не случалось вовсе.
            'enroll_warmup_total': 0,
            'gallery_warmup_size': 5,
        },
    )
    node = _FakeNode(stats)
    node._speaker_unknown_total = 2  # issue #2748 — считается ДО recognizer
    m.VisionFaceNode._log_stats(node)

    assert len(node._logger.info_lines) == 2
    summary, diagnostics = node._logger.info_lines

    # Существующая часть строки не переименована (по ней ищут в логах) —
    # только добавлен суффикс причин рядом с кропов_отброшено=.
    assert (
        '[лицо] режим=workshop встреч=3 узнано=0 новых=1 слияний=2 '
        '(пропуски: голос_не_опознан=2 нет_лица=1 конфликт_профилей=1) '
        'ошибок_эмбеддинга=0 кропов_отброшено=59 (clipped=51 blurry=8) '
        'треков=0 | в базе: людей=3 с_именем=2 gallery_cohesion=н/д '
        'enroll_отклонено=0 прогрев=0/5 разрешено_неоднозначностей=0'
    ) == summary

    assert diagnostics == (
        "[лицо] выравнивание: align_used=0 align_fallback=118 | "
        'бюджет NPU: embed_skipped_budget=0'
    )

    # align_used=0 при fallback=118 — первая сводка обязана предупредить.
    assert len(node._logger.warning_lines) == 1
    assert 'НЕ РАБОТАЕТ' in node._logger.warning_lines[0]
    assert '#2773' in node._logger.warning_lines[0]


def test_log_stats_align_metrics_always_printed_even_when_zero():
    """Требование задачи: метрики выравнивания печатаются ВСЕГДА, а не
    только при пропусках/проблемах — иначе тихая деградация снова не
    будет видна."""
    node = _run_log_stats(_base_stats())
    diagnostics = node._logger.info_lines[1]
    assert 'align_used=0' in diagnostics
    assert 'align_fallback=0' in diagnostics
    # align_fallback=0 -> нечего диагностировать, предупреждения нет.
    assert node._logger.warning_lines == []


def test_log_stats_crop_suffix_absent_when_nothing_rejected():
    node = _run_log_stats(_base_stats(crop_rejected_total=0))
    summary = node._logger.info_lines[0]
    assert 'кропов_отброшено=0 треков=' in summary  # ни одной скобки-суффикса


def test_log_stats_warning_repeats_every_period_not_every_call():
    period = m.ALIGN_DEAD_WARNING_PERIOD_CALLS
    stats = _base_stats(align_used_total=0, align_fallback_total=10)
    node = _run_log_stats(stats, calls=period + 1)
    # Один на первом вызове + один на (period+1)-м = 2 за period+1 вызовов.
    assert len(node._logger.warning_lines) == 2


def test_log_stats_no_warning_once_alignment_starts_working():
    stats = _base_stats(align_used_total=3, align_fallback_total=10)
    node = _run_log_stats(stats, calls=5)
    assert node._logger.warning_lines == []


if __name__ == '__main__':  # pragma: no cover
    sys.exit(pytest.main([__file__, '-q']))
