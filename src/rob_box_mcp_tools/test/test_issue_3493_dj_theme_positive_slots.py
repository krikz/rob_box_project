"""#3493 — тема сета выделяется положительно по слотам, а не вычитанием растущих стоп-таблиц.

Корпус: живые заказы 06–07.10 из ``test_dj_theme_from_phrase_clean``, ``test_dj_theme_hogwarts_concept`` и
``test_dj_theme_links``; обрамление из issue («на днюху», «ламповое», «чтоб качало», «для бабушки»). Явно названные
произведения остаются входом #3503 (проверка каталогом/LLM), а непонятые слова вне слотов — только в журнале.
"""

from __future__ import annotations

import logging

import pytest

from rob_box_mcp_tools.engine.theme_grounding import heard_theme, set_slots

pytestmark = pytest.mark.unit

DENDY = ("Робот, ты диджей 8битный и у нас сегодня вечеринка любителей денди и классической музыки "
         "замути сэт на 30 минут")
ATLAS = "Ты диджей Атлас и у нас сегодня вечеринка любителей кино, играй мелодии из топовых фильмов"
HARRY = "Ты диджей Гарри Поттер и у нас сегодня вечеринка путешествие по Хогвардсу"
LIST = "танец маленьких утят, лебединое озеро, король лев"


@pytest.mark.parametrize("phrase,theme,unknown", [
    (DENDY, "денди и классической музыки", ""),
    (ATLAS, "кино, фильмов", ""),
    (HARRY, "путешествие хогвардсу", ""),
    ("Робот, включи рейв сет на тему в пещере горного короля", "пещере горного короля", ""),
    ("сыграй интерстеллар сэт", "интерстеллар", ""),
    ("ты диджей Робокс на тему детский праздник", "детский праздник", ""),
    ("поставь сет в стиле Stranger Things", "stranger things", ""),
    ("включи сет на 3 трека: Марио, Тетрис, Зельда", "марио, тетрис, зельда", ""),
    (f"Робот, включи сет: {LIST}", LIST, ""),
    ("Робот, включи сет на тему динозавров", "динозавров", ""),
    ("Робот, поставь сет на днюху с Марио на 5 треков", "марио", "днюху"),
    ("Робот, поставь сет ламповое на 3 трека", "", "ламповое"),
    ("Робот, поставь сет для бабушки на 3 трека", "", "бабушки"),
    ("Робот, поставь сет чтоб качало на 5 треков", "", "качало"),
    ("Робот, вечеринка любителей кино для бабушки, поставь сет", "кино", "бабушки"),
    ("Робот, включи сет на тему динозавров для бабушки", "динозавров", "бабушки"),
    ("поставь сайт из трех треков объявляй каждый", "", "сайт"),
])
def test_positive_theme_slots_and_unknown_journal(phrase, theme, unknown, caplog):
    assert heard_theme(phrase) == theme
    with caplog.at_level(logging.INFO):
        set_slots(None, None, phrase, None, logging.getLogger("test.theme_slots"))
    lines = [r.getMessage() for r in caplog.records if "непонятые слова" in r.getMessage()]
    if unknown:
        assert len(lines) == 1 and unknown in lines[0] and theme in lines[0]
    else:
        assert lines == []


@pytest.mark.parametrize("word", ["ламповое", "бабушки", "качало"])
def test_llm_unknown_theme_is_positive_filtered_and_never_generated(tmp_path, monkeypatch, word):
    """LLM повторила непонятое слово человека: положительной темы нет ни у кода, ни у LLM, fake KIE не вызывается."""
    from rob_box_mcp_tools.engine.theme_choice import dj_set_slots

    from .test_n605_theme_guard_junk_m6 import _running_set

    dj, kie = _running_set(tmp_path, monkeypatch, cached=0)
    heard = f"поставь сет {word} на 1 трек"
    slots = dj_set_slots(dj, word, None, heard)
    assert slots.free is True and slots.theme == ""
    ordered = len(kie.jobs)
    data = dj.execute(action="start", theme=word, tracks=1, heard_text=heard).data
    assert data["theme"] == "" and len(kie.jobs) == ordered


@pytest.mark.parametrize("phrase,theme,unknown", [
    ("поставь сет днюху", "", ["днюху"]),
    ("поставь сет бабушки", "", ["бабушки"]),
    ("поставь сет качало", "", ["качало"]),
    ("поставь сет днюху Марио", "марио", ["днюху"]),
    ("поставь сет Марио днюху", "марио", ["днюху"]),
    ("поставь сет качало Марио", "марио", ["качало"]),
    ("поставь сет сайт Марио", "марио", ["сайт"]),
    ("включи сет динозавры", "", ["динозавры"]),  # одиночное неизвестное после SET — не положительный слот
])
def test_unknown_words_cannot_precede_or_follow_a_positive_theme(phrase, theme, unknown):
    """Порядок обрамления не меняет решение: повод/публика/качество — в журнал, положительная тема — отдельно."""
    from rob_box_mcp_tools.engine.theme_grounding import _heard_slots

    heard, dropped = _heard_slots(phrase)
    assert heard == theme
    assert dropped == unknown


def test_production_slots_log_unknown_next_to_positive_theme(tmp_path, monkeypatch, caplog):
    """Production ``dj_set_slots`` не прячет непонятое слово, когда рядом нашлась опорная тема."""
    from rob_box_mcp_tools.engine.theme_choice import dj_set_slots

    from .test_n605_theme_guard_junk_m6 import _running_set

    dj, _kie = _running_set(tmp_path, monkeypatch, cached=0)
    with caplog.at_level(logging.INFO):
        slots = dj_set_slots(dj, None, None, "поставь сет ламповое кино")
    assert slots.theme == "кино"
    lines = [r.getMessage() for r in caplog.records if "непонятые слова" in r.getMessage()]
    assert len(lines) == 1 and "ламповое" in lines[0] and "кино" in lines[0]
