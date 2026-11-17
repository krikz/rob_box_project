"""Свод решений Шифу: у каждой записи теги из словаря, оглавление совпадает (scripts/docs/shifu_decisions_tags.py)."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
_spec = importlib.util.spec_from_file_location(
    "shifu_decisions_tags", ROOT / "scripts" / "docs" / "shifu_decisions_tags.py"
)
tags_mod = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = tags_mod  # dataclass ищет свой модуль в sys.modules
_spec.loader.exec_module(tags_mod)

_SAMPLE = """# Свод

<!-- tags:begin -->
- `#голос` — речь.
- `#музыка` — треки.
<!-- tags:end -->

<!-- toc:begin -->
{toc}
<!-- toc:end -->

<a id="d-1"></a>
### 05.11.2026 — Первое

Теги: #голос

Текст.

<a id="d-2"></a>
### 04.11.2026 — Второе

Теги: #голос #музыка

Текст.
"""

_TOC = (
    "- **#голос** (2): [05.11.2026 — Первое](#d-1), [04.11.2026 — Второе](#d-2)\n"
    "- **#музыка** (1): [04.11.2026 — Второе](#d-2)"
)


def _lines(text: str):
    return text.splitlines()


def test_real_file_passes():
    lines = tags_mod.DEFAULT_PATH.read_text(encoding="utf-8").splitlines()
    assert tags_mod.check(lines) == []
    entries, _ = tags_mod.parse_entries(lines)
    assert len(entries) > 0


def test_sample_passes_and_summary():
    lines = _lines(_SAMPLE.format(toc=_TOC))
    assert tags_mod.check(lines) == []
    vocab = tags_mod.parse_dictionary(lines)
    entries, _ = tags_mod.parse_entries(lines)
    assert tags_mod.summary(vocab, entries) == {"#голос": 2, "#музыка": 1}


def test_entry_without_tags_fails():
    text = _SAMPLE.format(toc=_TOC).replace("Теги: #голос\n\nТекст.", "Текст.", 1)
    errors = tags_mod.check(_lines(text))
    assert any("Теги:" in e for e in errors)


def test_unknown_tag_fails():
    text = _SAMPLE.format(toc=_TOC).replace("Теги: #голос\n", "Теги: #погода\n", 1)
    errors = tags_mod.check(_lines(text))
    assert any("#погода" in e for e in errors)


def test_too_many_tags_fails():
    text = _SAMPLE.format(toc=_TOC).replace("Теги: #голос\n", "Теги: #голос #музыка #голос #музыка #голос\n", 1)
    errors = tags_mod.check(_lines(text))
    assert any("не больше" in e for e in errors)


def test_stale_toc_fails():
    errors = tags_mod.check(_lines(_SAMPLE.format(toc="- устарело")))
    assert any("оглавление" in e for e in errors)


def test_missing_anchor_fails():
    text = _SAMPLE.format(toc=_TOC).replace('<a id="d-2"></a>\n', "", 1)
    errors = tags_mod.check(_lines(text))
    assert any("якоря" in e for e in errors)
