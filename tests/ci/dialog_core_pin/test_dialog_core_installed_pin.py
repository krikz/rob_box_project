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
from reason_words import hard_key_reads, missing_in_pin

_GUARD = Path(__file__).resolve().parent / "guard.py"


def test_hard_reads_are_literals_not_get() -> None:
    """Обход собирает ``REASON_WORDS['ключ']`` и не принимает ``.get`` за жёсткое чтение."""
    reads = hard_key_reads()
    assert "no_lyrics_path" in reads
    joined = " ".join(site for sites in reads.values() for site in sites)
    assert "instrumental_order.py" in joined
    assert missing_in_pin({"no_lyrics_path"}, set()) == ["no_lyrics_path"]
    assert missing_in_pin({"no_lyrics_path"}, {"no_lyrics_path"}) == []


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


def test_missing_reason_word_is_caught() -> None:
    """Синтетический ключ, которого в пине нет, красится и называет ключ с тегом.

    Без префикса тот же флаг падает с явной причиной, а не зеленеет: контроль
    не считается пройденным, пока пин не поставлен.
    """
    got = subprocess.run(
        [sys.executable, str(_GUARD), "--missing-key"],
        capture_output=True,
        text=True,
        timeout=60,
        check=False,
    )
    blob = got.stdout + got.stderr
    if not os.environ.get("ROB_BOX_DIALOG_CORE_PREFIX"):
        assert got.returncode != 0, blob
        assert "нет ROB_BOX_DIALOG_CORE_PREFIX" in blob
        return
    assert got.returncode == 0, blob
    assert "MISSING_KEY detected" in got.stdout
    assert "монорепо читает ключ" in got.stdout
    assert "в пине" in got.stdout


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
    blob = got.stdout + got.stderr
    assert got.returncode == 0, blob
    assert "PIN_OK" in got.stdout
    assert "site-packages" in got.stdout
    assert "REASON_WORDS_OK" in got.stdout
