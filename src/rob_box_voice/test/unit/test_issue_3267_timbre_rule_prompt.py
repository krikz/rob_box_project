"""Issue #3267 — правило «регион/настроение → тембры палитры» живёт в промптах.

Азиатский сет 01.10: LLM брала epiano+tb303 для китайского фольклора и
imperialbrass для гимна КНР, хотя в палитре есть sitar/karp/bell/marimba/flute.
Словаря «тема → тембр» нет: правило учит LLM принципу, выбор — за ней.
"""

from __future__ import annotations

import re
from pathlib import Path

_SKILLS = Path(__file__).resolve().parents[2] / "prompts" / "skills"
_MASTER = Path(__file__).resolve().parents[2] / "prompts" / "master_prompt_compact.txt"


def _text(name: str) -> str:
    return (_SKILLS / name).read_text(encoding="utf-8")


def test_composer_has_timbre_rule_with_palette_names() -> None:
    text = _text("composer.txt")
    assert "ТЕМБРЫ ПО КУЛЬТУРЕ/НАСТРОЕНИЮ" in text
    rule = text.split("ТЕМБРЫ ПО КУЛЬТУРЕ/НАСТРОЕНИЮ", 1)[1][:900]
    palette = _MASTER.read_text(encoding="utf-8")
    for synth in ("sitar", "karp", "bell", "marimba", "flute", "imperialbrass", "tb303"):
        assert synth in rule, synth
        assert re.search(rf"\b{synth}\b", palette), f"{synth} нет в палитре"


def test_timbre_rule_is_principle_not_theme_dictionary() -> None:
    rule = _text("composer.txt").split("ТЕМБРЫ ПО КУЛЬТУРЕ/НАСТРОЕНИЮ", 1)[1][:900]
    assert "chinese" not in rule.lower() and "кита" not in rule.lower()


def test_dj_forbids_anthem_as_party_hook_without_naming_composer_tools() -> None:
    text = _text("dj.txt")
    assert "anthem" in text and "явно заказал" in text
    for tool in ("compose_music", "preview_arrangement", "execute_music_code"):
        assert tool not in text
