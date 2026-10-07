"""Живой случай 07.10 ~09:08 UTC (товарищ Шифу, чистый образ develop): «путешествие по Хогвардсу» играло пул.

«Ты диджей Гарри Поттер и у нас сегодня вечеринка путешествие по Хогвардсу» → тема ``'путешествие хогвардсу'``,
``source=pool``, хуки ``popcorn_6, kalinkav, …``. Тема верная (решение Шифу: персона — не тема); из «Хогвардса»
сами собой должны родиться мелодии Гарри Поттера. «Хогвардсу» (STT-написание «Хогвартс» в дательном) — понятие
``knowledge.THEME_CONCEPTS`` по началу основы «хогвар» → запрос «potter» к архиву: записи франшизы. Тест — по
настоящему архиву и пути ``DjSetTool.execute``.
"""

from __future__ import annotations

import pytest

from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary
from rob_box_mcp_tools.engine.search import theme_search
from rob_box_mcp_tools.engine.theme_grounding import heard_theme
from rob_box_mcp_tools.engine.tools_v2 import DjSetTool, library_melodies, theme_finder

from .test_engine_session import _rig, _started

pytestmark = pytest.mark.unit

HARRY = "Ты диджей Гарри Поттер и у нас сегодня вечеринка путешествие по Хогвардсу"
#: Записи франшизы в архиве (поиск «potter»: название или «исполнитель» Harry Potter).
FRANCHISE = {"harrypot", "harrypot_2", "harrypot_3", "harrypot_4", "harrypot_5", "harrypot_6", "theme_83",
             "mr_longb", "diagonal", "hedwigst", "thenorwe", "fluffy_s"}


@pytest.fixture(scope="module")
def library(tmp_path_factory):
    return RtttlLibrary(db_path=str(tmp_path_factory.mktemp("hogwarts") / "voice_memory.db"))


@pytest.mark.parametrize("theme", ["путешествие хогвардсу", "Хогвардсу", "хогвартс", "в Хогвартсе", "hogwarts"])
def test_hogwarts_in_any_case_and_spelling_finds_the_franchise(library, theme):
    names = theme_search(library, theme).names
    assert names and set(names) <= FRANCHISE, names


def test_live_phrase_keeps_its_theme_and_plays_harry_potter(library):
    assert heard_theme(HARRY) == "путешествие хогвардсу"  # персона — не тема (решение Шифу 07.10)
    rig = _rig()
    dj = DjSetTool(None, rig.owner, melodies=library_melodies(lambda: library), finder=theme_finder(lambda: library),
                   seed=lambda: 4242)
    data = dj.execute(action="start", persona="диджей Гарри Поттер", heard_text=HARRY).data
    assert data["theme"] == "путешествие хогвардсу" and data["theme_source"] == "theme"
    assert dj._session.dj_fields(1)["persona"] == "диджей Гарри Поттер"
    rig.clock.run_until(rig.clock.beat + 2)
    assert _started(rig)[0]["track_id"] == data["track_id"]
    assert set(dj.theme_profile(data["theme"]).hook_ids) <= FRANCHISE
