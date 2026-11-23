"""Гард пина ядра гоняется отдельно и не зеленеет без установленного тега (#4932, #4891).

Файл лежит вне ``src/*/test`` и вне ``tests/unit``: штатный ``Unit Tests (ROS2 Humble)`` и
``Offline Unit Tests`` его не собирают. Джоб ``G: Dialog Core Pin Guard`` ставит тег из
Dockerfile в изолированный venv, экспортирует ``ROB_BOX_DIALOG_CORE_PREFIX`` и вызывает
``guard.py``. Без префикса проверка падает, а не пропускается — pytest.skip здесь нет.
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest

_GUARD = Path(__file__).resolve().parent / "guard.py"


def test_missing_prefix_fails_loud(monkeypatch: pytest.MonkeyPatch) -> None:
    """Без ``ROB_BOX_DIALOG_CORE_PREFIX`` гард падает и называет причину, а не пропускается."""
    monkeypatch.delenv("ROB_BOX_DIALOG_CORE_PREFIX", raising=False)
    got = subprocess.run(
        [sys.executable, str(_GUARD)],
        capture_output=True,
        text=True,
        env={k: v for k, v in os.environ.items() if k != "ROB_BOX_DIALOG_CORE_PREFIX"},
        timeout=30,
        check=False,
    )
    assert got.returncode != 0, got.stdout + got.stderr
    assert "нет ROB_BOX_DIALOG_CORE_PREFIX" in (got.stdout + got.stderr)


def test_src_on_pythonpath_is_detected_as_substitution() -> None:
    """Подсунутый ``src/`` детектится: проба падает и пишет «подмена»."""
    got = subprocess.run(
        [sys.executable, str(_GUARD), "--substitution"],
        capture_output=True,
        text=True,
        timeout=60,
        check=False,
    )
    assert got.returncode == 0, got.stdout + got.stderr
    assert "SUBSTITUTION detected" in got.stdout
    assert "подмена" in (got.stdout + got.stderr)


def test_installed_pin_import_is_not_shadowed_by_src() -> None:
    """Чистый префикс с тегом из Dockerfile импортирует ядро из site-packages.

    Префикс готовит джоб (``ROB_BOX_DIALOG_CORE_PREFIX``). Без него тест падает —
    зелёный прогон на локальном ``src/`` этой проверкой не считается.
    """
    got = subprocess.run(
        [sys.executable, str(_GUARD)],
        capture_output=True,
        text=True,
        timeout=60,
        check=False,
    )
    assert got.returncode == 0, got.stdout + got.stderr
    assert "PIN_OK" in got.stdout
    assert "site-packages" in got.stdout
