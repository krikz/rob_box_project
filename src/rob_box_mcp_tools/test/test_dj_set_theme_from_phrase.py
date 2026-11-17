"""06.10 18:04–18:05 UTC (живой лог): человек написал «Ты диджей 8битный и нас сегодня вечеринка любителей денди и
классической музыки, замути сэт на 30 минут», а LLM вызвала ``dj_set`` с темой из СТАРОЙ реплики истории («Увертюра
1812, мегасет для геймеров … Марио, Аладдин, Тетрис …») — классика пропала.

Решает код (ADR-0148): реплика хода едет скрытым ``heard_text`` (``llm_adapter.TURN_CONTEXT_ARGS``); тема LLM, не
делящая с ней значимых слов, заменяется темой реплики. Перефраз остаётся; нет реплики — старое поведение.
"""

import logging

import pytest

from rob_box_mcp_tools.engine.search import genre_of
from rob_box_mcp_tools.engine.theme_grounding import grounded_theme, heard_theme

from .test_engine_request_music import _tools
from .test_engine_session import _rig

pytestmark = pytest.mark.unit

PHRASE = "Ты диджей 8битный и нас сегодня вечеринка любителей денди и классической музыки, замути сэт на 30 минут"
OLD_THEME = ("Увертюра тысяча восемьсот двенадцатого года, мегасет для геймеров: разные треки из РТТЛ из разных игр — "
             "Марио, Аладдин, Тетрис, Чёрный Плащ, Контра, и так далее, десятки три")
PARAPHRASE = "8-битный диджей, вечеринка любителей денди и классической музыки"


def test_heard_theme_drops_request_persona_and_length_but_keeps_classical():
    theme = heard_theme(PHRASE)
    assert "денди" in theme and "классической" in theme
    for service in ("диджей", "замути", "сэт", "минут", "30", "ты"):
        assert service not in theme.split()
    assert genre_of(theme) == "classical"


def test_theme_from_an_old_utterance_is_replaced_by_the_current_phrase():
    theme, replaced = grounded_theme(OLD_THEME, PHRASE)
    assert replaced
    assert "классической" in theme and "денди" in theme
    assert "Марио" not in theme and "Увертюра" not in theme


def test_llm_paraphrase_of_the_current_phrase_is_kept():
    assert grounded_theme(PARAPHRASE, PHRASE) == (PARAPHRASE, False)


def test_genre_synonym_counts_as_grounded():
    assert grounded_theme("классика", PHRASE) == ("классика", False)


@pytest.mark.parametrize("heard", [None, "", "   ", "да", "давай", "Робот, замути сэт на 30 минут"])
def test_no_context_or_no_theme_words_keeps_the_llm_theme(heard):
    assert grounded_theme(OLD_THEME, heard) == (OLD_THEME, False)


def test_empty_llm_theme_takes_the_phrase_theme():
    theme, replaced = grounded_theme(None, PHRASE)
    assert replaced and "классической" in theme


def test_dj_set_starts_with_the_current_phrase_theme_and_logs_the_swap(caplog):
    dj, _req = _tools(_rig())
    with caplog.at_level(logging.WARNING):
        result = dj.execute(action="start", theme=OLD_THEME, persona="диджей 8битный", style="club", tracks=30,
                            heard_tracks=24, heard_text=PHRASE)
    assert result.data["theme"] == heard_theme(PHRASE)
    assert genre_of(result.data["theme"]) == "classical"
    assert any("[dj_set] тема LLM не из текущей реплики → из реплики" in r.getMessage() for r in caplog.records)


#: Замер 06.10 19:08 UTC — та же фраза без запятой (STT); её запускает роутер медиакоманд без LLM.
LIVE_1908 = ("Робот, ты диджей 8битный и нас сегодня вечеринка любителей денди и классической музыки "
             "замути сэт на 30 минут")


def test_router_command_gets_the_same_theme_and_length_as_the_llm_path():
    """Роутер (``rob_box_voice.core.media_router``) шлёт ``dj_set(start, persona, tracks=24)`` без темы, реплика —
    в ``heard_text``; путь LLM — с темой из истории. Тема и длина у обоих — из слов реплики, одной функцией."""
    command = {"action": "start", "persona": "диджей 8битный", "tracks": 24}
    dj, _req = _tools(_rig())
    by_code = dj.execute(**command, heard_tracks=24, heard_text=LIVE_1908).data
    dj, _req = _tools(_rig())
    by_llm = dj.execute(action="start", theme=OLD_THEME, persona="диджей 8битный", heard_tracks=24,
                        heard_text=LIVE_1908).data
    assert by_code["theme"] == by_llm["theme"] == heard_theme(LIVE_1908)
    assert "денди" in by_code["theme"] and "классической" in by_code["theme"]
    assert genre_of(by_code["theme"]) == "classical"
    assert by_code["tracks"] == by_llm["tracks"] == 24


def test_dj_set_keeps_paraphrase_and_old_calls_without_context():
    dj, _req = _tools(_rig())
    assert dj.execute(action="start", theme=PARAPHRASE, heard_text=PHRASE).data["theme"] == PARAPHRASE
    dj, _req = _tools(_rig())
    assert dj.execute(action="start", theme=OLD_THEME).data["theme"] == OLD_THEME
