"""ADR-0153 S5: рок-сет и его окно решает код (ADR-0148) — «гранж» в реплике человека выбирает окно ``grunge`` стиля
``rock``; без слов окна его выбирает план по сиду. Слово окна чужого стиля окно не выбирает."""

from __future__ import annotations

import pytest

from rob_box_music import knowledge as kn
from rob_box_music.theme import match_window_text

from .test_engine_request_music import _tools
from .test_engine_session import _rig

pytestmark = pytest.mark.unit


@pytest.mark.parametrize("style,texts,window", [
    ("rock", ("Робот, включи гранж сет на тему космос", "космос"), "grunge"),
    ("rock", (None, "гранжевый вечер"), "grunge"),
    ("rock", ("Робот, включи рок сет на тему в пещере горного короля",), None),
    ("club", ("Робот, включи гранж сет",), None),
])
def test_window_words_pick_a_window_of_the_set_style_only(style, texts, window):
    assert match_window_text(style, *texts) == window


def test_grunge_phrase_starts_a_rock_set_in_the_grunge_window():
    dj, _req = _tools(_rig())
    data = dj.execute(action="start", theme="космос", heard_text="Робот, включи гранж сет на тему космос").data
    assert data["style"] == "rock" and data["genre"] == "grunge"


def test_rock_phrase_without_window_words_gets_a_rock_window_by_seed():
    dj, _req = _tools(_rig())
    data = dj.execute(action="start", theme="в пещере горного короля",
                      heard_text="Робот, включи рок сет на тему в пещере горного короля").data
    assert data["style"] == "rock" and data["genre"] in kn.STYLES["rock"].genre_windows
