"""Тест: supervisor_node стартует, когда rob_box_core.utterance недоступен (issue #2233).

Контекст
--------
PR #2218 / issue #2197 ([voice-vr 12]) ввёл ``rob_box_core.utterance``.
PR #2255 (voice-vr 21) и предыдущие использовали прямой
``from rob_box_core.utterance import Sink, Utterance`` в
:mod:`rob_box_supervisor.supervisor_node`. Если base image не содержит
этого модуля (round-деплои с не-SHA-pinned тегами, issue #1826) —
supervisor падает в __init__ с ``ModuleNotFoundError`` и уходит в
restart-loop.

Этот тест проверяет, что fallback в supervisor_node действительно
держит supervisor живым при отсутствии ``rob_box_core.utterance``.
"""
from __future__ import annotations

import importlib
import sys
import unittest


class TestSupervisorSurvivesMissingUtterance(unittest.TestCase):
    """Симулируем «rob_box_core.utterance недоступен» через mock sys.modules.

    Стратегия:
    1. Убеждаемся, что ``rob_box_core`` импортирован (его __init__.py
       сам делает ``from rob_box_core.utterance import …`` — поэтому
       блокировать ``rob_box_core.utterance`` ДО импорта ``rob_box_core``
       нельзя, иначе __init__.py упадёт первым).
    2. После этого ставим ``sys.modules['rob_box_core.utterance'] = None``
       — любой будущий ``from rob_box_core.utterance import ...`` упадёт
       с ImportError.
    3. Перезагружаем supervisor_node — должен успешно импортнуться через
       fallback-путь.
    4. Sink/Utterance должны быть доступны в supervisor_node как
       символы (через fallback).
    """

    def setUp(self) -> None:
        # Шаг 1: гарантируем, что rob_box_core импортирован ДО блокировки
        # utterance. __init__.py rob_box_core сам делает
        # ``from rob_box_core.utterance import …``, поэтому блок ДО
        # импорта привёл бы к провалу на первом же ``import rob_box_core``
        # (что НЕ связано с нашим фиксом — это базовый invariant
        # пакета, см. src/rob_box_core/rob_box_core/__init__.py:41).
        import rob_box_core  # noqa: F401

        self._utterance_was_in_modules = "rob_box_core.utterance" in sys.modules
        if self._utterance_was_in_modules:
            self._utterance_backup = sys.modules["rob_box_core.utterance"]
        else:
            self._utterance_backup = None
        # Шаг 2: блокируем submodule — теперь любой
        # ``from rob_box_core.utterance import …`` упадёт ImportError.
        sys.modules["rob_box_core.utterance"] = None  # type: ignore[assignment]

        # supervisor_node может быть уже загружен предыдущим тестом —
        # сбрасываем, чтобы try/except-блок отработал с чистого листа.
        self._supervisor_node_backup = sys.modules.get(
            "rob_box_supervisor.supervisor_node"
        )
        sys.modules.pop("rob_box_supervisor.supervisor_node", None)

    def tearDown(self) -> None:
        # Восстанавливаем sys.modules.
        if self._utterance_backup is not None:
            sys.modules["rob_box_core.utterance"] = self._utterance_backup
        else:
            sys.modules.pop("rob_box_core.utterance", None)
        if self._supervisor_node_backup is not None:
            sys.modules["rob_box_supervisor.supervisor_node"] = (
                self._supervisor_node_backup
            )
        else:
            sys.modules.pop("rob_box_supervisor.supervisor_node", None)

    def test_supervisor_node_imports_with_fallback(self) -> None:
        # Шаг 3: импортируем supervisor_node — должен пройти через
        # except ImportError в try/except вокруг utterance.
        try:
            sn = importlib.import_module("rob_box_supervisor.supervisor_node")
        except ImportError as exc:  # pragma: no cover
            self.fail(
                f"supervisor_node не должен падать с ImportError при "
                f"отсутствии rob_box_core.utterance — fallback обязан "
                f"поймать. Получено: {exc!r}"
            )
        # Шаг 4: Sink/Utterance должны быть доступны (через fallback).
        self.assertTrue(hasattr(sn, "Sink"))
        self.assertTrue(hasattr(sn, "Utterance"))
        self.assertIsNotNone(sn.Sink)
        self.assertIsNotNone(sn.Utterance)


if __name__ == "__main__":
    unittest.main()
