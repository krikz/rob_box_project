"""Марафон-6 n605 (#4217, #3845): guard темы подменил тему LLM огрызком распознавания и заказал платную генерацию.

Живой лог М6 (29.10, dev-980c053)::

    STT: робот поставь сайт из трех треков объявляй каждый               ← «сайт» - огрызок слова «сет»
    dj_set {'action': 'start', 'theme': 'рок', 'style': 'rock', 'tracks': 3,
            'heard_text': 'поставь сайт из трех треков объявляй каждый'}
    [dj_set] тема LLM не из текущей реплики → из реплики: LLM='рок' → 'сайт'
    [gen set] слоты: тема='сайт' стили=['rock'] треков=3 по теме из кэша=0 заказ=3 (задач ≈2) …

Две задачи Suno по 12 кредитов ушли в пустоту. Решает код (ADR-0148): голое слово реплики без опоры (не жанр, не
вселенная, не строка таблицы тем, не латиница, не «про X», нет слова «сет») тему LLM не подменяет
(``theme_grounding.names_theme``), а тема, оставшаяся догадкой кода, платной генерации не заказывает
(``theme_grounding.guessed_theme`` → ``DjSetTool._start(free=True)``: кэш или синтез).
"""

from __future__ import annotations

import logging

import pytest

from rob_box_mcp_tools.engine.theme_grounding import grounded_theme, guessed_theme, heard_theme, names_theme

from .test_adr0158_pr8_gen_set import Kie, _dj, _fake_audio_track, _lib

pytestmark = pytest.mark.unit

#: Дословно из лога М6 (``heard_text`` вызова ``dj_set``).
N605_M6 = "поставь сайт из трех треков объявляй каждый"


def test_the_stt_scrap_stays_what_it_is_a_bare_word_without_support():
    assert heard_theme(N605_M6) == "сайт"
    assert not names_theme("сайт", N605_M6)


def test_the_llm_theme_is_not_replaced_by_the_stt_scrap():
    assert grounded_theme("рок", N605_M6) == ("рок", False)
    assert grounded_theme("ночная дискотека", N605_M6) == ("ночная дискотека", False)


@pytest.mark.parametrize("phrase", [
    "поставь сет про космос",              # слово таблицы тем
    "поставь сет про динозавров",          # человек ввёл тему сам («про»)
    "включи сет динозавры",                # слово сета названо - слова после него тема
    "поставь сет в стиле Stranger Things",  # описание из слов
    "поставь Nirvana из трех треков",      # название латиницей
    "поставь сет Звездные войны",          # вселенная
])
def test_a_word_the_human_named_still_replaces_a_stale_llm_theme(phrase):
    heard = heard_theme(phrase)
    assert heard and names_theme(heard, phrase), phrase
    assert grounded_theme("Увертюра 1812, мегасет для геймеров", phrase) == (heard, True), phrase


@pytest.mark.parametrize("theme,llm,heard_text,expect", [
    ("сайт", None, N605_M6, True),                         # роутер/LLM без темы: тему выделил код из огрызка
    ("сайт", "", N605_M6, True),
    ("рок", "рок", N605_M6, False),                        # тема LLM - не догадка кода
    ("динозавры", None, "поставь сет про динозавров", False),  # названа человеком
    ("stranger things", None, "поставь сет в стиле Stranger Things", False),
    ("", None, N605_M6, False),
])
def test_guessed_theme(theme, llm, heard_text, expect):
    assert guessed_theme(theme, llm, heard_text) is expect


def _running_set(tmp_path, monkeypatch, cached=2):
    """Идёт предыдущий сет (в М6 - n604: ``dj: on``, ``set_theme``): ``dj_set(start)`` без слова «сет» в реплике не
    отказывает (``set_refusal``: смена темы посреди сета, #3400). ``(dj, kie)``; KIE уже мог получить заказы первого."""
    monkeypatch.setattr("rob_box_mcp_tools.engine.gen.gen_session.audio_track", _fake_audio_track)
    kie = Kie()
    dj = _dj(_lib(tmp_path, cached=cached), kie)
    first = dj.execute(action="start", style="rock", tracks=3, heard_text="включи рок-сет на 3 трека")
    assert first.success and dj.running
    return dj, kie


def _routes(caplog):
    return [r.getMessage() for r in caplog.records if "[source] сет" in r.getMessage()]


def test_n605_m6_verbatim_keeps_the_llm_theme_and_never_logs_the_substitution(tmp_path, monkeypatch, caplog):
    dj, _kie = _running_set(tmp_path, monkeypatch)
    caplog.clear()
    with caplog.at_level(logging.INFO):
        data = dj.execute(action="start", theme="рок", style="rock", tracks=3, heard_text=N605_M6).data
    assert data["ok"] is True and data["theme"] == "рок" and data["theme"] != "сайт"
    assert not any("тема LLM не из текущей реплики" in r.getMessage() for r in caplog.records)
    assert not any("догадка кода" in r.getMessage() for r in caplog.records), "тема LLM - не догадка кода"


def test_n605_m6_without_an_llm_theme_the_scrap_orders_nothing_paid(tmp_path, monkeypatch, caplog):
    """Роутер медиакоманд шлёт ``dj_set`` без темы: тему выделяет код, и это огрызок «сайт» - сет из кэша/синтезом."""
    dj, kie = _running_set(tmp_path, monkeypatch, cached=0)
    ordered = len(kie.jobs)
    caplog.clear()
    with caplog.at_level(logging.INFO):
        data = dj.execute(action="start", tracks=3, heard_text=N605_M6).data
    assert data["ok"] is True and data["theme"] == "сайт"
    assert len(kie.jobs) == ordered, "по огрызку распознавания заказана платная генерация"
    lines = _routes(caplog)
    assert not any("generate:" in line for line in lines), lines
    assert any("догадка кода" in r.getMessage() for r in caplog.records)
    assert dj._session.set_id.startswith("set")  # noqa: SLF001 - сет синтезом, не из генерации


def test_a_theme_named_by_the_human_still_orders_the_generation_as_before(tmp_path, monkeypatch, caplog):
    monkeypatch.setattr("rob_box_mcp_tools.engine.gen.gen_session.audio_track", _fake_audio_track)
    kie = Kie()
    dj = _dj(_lib(tmp_path, cached=0), kie)
    phrase = "поставь сет в стиле Stranger Things из трех треков"
    with caplog.at_level(logging.INFO):
        data = dj.execute(action="start", style="rock", tracks=3, heard_text=phrase).data
    assert data["ok"] is True and "stranger" in data["theme"].lower()
    assert kie.jobs, "названная человеком тема перестала заказывать генерацию"
    assert any("generate:" in line for line in _routes(caplog))
    assert not any("догадка кода" in r.getMessage() for r in caplog.records)
