"""ADR-0153 S2: стиль сета ``dj_set`` решает код — слова человека этого хода, затем ключ, затем тема и её строка."""

from __future__ import annotations

import pytest

from rob_box_mcp_tools.engine.tools_v2 import STYLE_CHOICES, set_style


def test_style_choices_name_the_new_styles():
    assert {"auto", "club", "rave", "synthwave", "chiptune"} <= set(STYLE_CHOICES)


@pytest.mark.parametrize("style,theme,heard,expected", [
    (None, "марио", "Робот, включи восьмибитный сет на тему марио", "chiptune"),
    ("club", "денди", "Ты диджей 8битный и у нас сегодня вечеринка любителей денди", "chiptune"),
    ("auto", "киберпанк", "Робот, включи синтвейв сет на тему киберпанк", "synthwave"),
    ("rave", "космос", "Робот, включи рейв на тему космос", "rave"),
    ("synthwave", "космос", "Робот, давай сет про космос", "synthwave"),
    ("auto", "киберпанк", None, "synthwave"),
    (None, "детский праздник", "", "chiptune"),
    ("auto", "космос", None, "club"),
])
def test_heard_words_then_key_then_theme_row(style, theme, heard, expected):
    assert set_style(style, theme, heard) == expected


def test_unknown_key_without_style_words_is_refused():
    with pytest.raises(ValueError, match="style='jazz'"):
        set_style("jazz", "космос", "Робот, давай сет про космос")
