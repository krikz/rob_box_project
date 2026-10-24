"""Earcon тишины (#3984, решение Шифу 23.10): звук лежит в звуковом пакете репо рядом с остальными.

Тест звукового узла (``rob_box_voice/test``) в CI видит только установленный пакет без корня репо, поэтому наличие
файлов держим здесь, где корень репо есть.
"""

from __future__ import annotations

import json
from pathlib import Path

PACK = Path(__file__).resolve().parents[3] / "sound_pack"


def test_muffled_sound_and_its_generator_lie_next_to_the_other_earcons():
    assert (PACK / "muffled.mp3").is_file() and (PACK / "make_muffled.py").is_file()
    assert (PACK / "button_click.mp3").is_file()
    assert 0 < (PACK / "muffled.mp3").stat().st_size < 20_000   # короткий звук (~0,26 с), не трек


def test_muffled_sound_is_a_code_sound_not_an_llm_choice():
    catalog = json.loads((PACK / "sound_catalog.json").read_text(encoding="utf-8"))
    assert "muffled.mp3" not in catalog["sounds"]               # в enum play_sound его нет
