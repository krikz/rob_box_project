"""Тесты ядра - правила над речью робота: умолчание вейков на время теста - таблица тестового профиля робота.

Ядро без профиля нейтрально (``knowledge.WAKE_WORDS_WITHOUT_PROFILE`` пуст, ADR-0167 §2.4); хост робота всегда
передаёт таблицу своего профиля явно (это сверяет ``tests/unit/dialog/test_wake_table_passed_0167.py``). Тесты зовут
правила без таблицы - фикстура ставит им таблицу тестового профиля робота (``robot_wake_fixture``). Тест
нейтрального ядра снимает её сам (``neutral_core``).
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from robot_wake_fixture import ROBOT_WAKE  # noqa: E402

from rob_box_dialog import knowledge as kn  # noqa: E402

#: Умолчание ядра как есть (до фикстуры) - его и возвращает ``neutral_core``.
CORE_DEFAULT = kn.WAKE_WORDS_WITHOUT_PROFILE


@pytest.fixture(autouse=True)
def _robot_wake_table(monkeypatch):
    monkeypatch.setattr(kn, "WAKE_WORDS_WITHOUT_PROFILE", ROBOT_WAKE)


@pytest.fixture()
def neutral_core(monkeypatch):
    """Ядро без профиля: умолчание вейков - то, что в ядре на самом деле (:data:`CORE_DEFAULT`), без фикстуры робота."""
    monkeypatch.setattr(kn, "WAKE_WORDS_WITHOUT_PROFILE", CORE_DEFAULT)
