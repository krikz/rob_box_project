"""Тема сета — выбором чойсера из известных тем и слов реплики (решение Шифу 29.10; #3845 #4217 #3683).

Шифу (Telegram 29.10 05:30 UTC): «мусорной темы не должно быть, можно из запроса пользователя сгенрить промт для
чойсера в чойсере пусть будут все слова из запроса плюс все известные темы что есть, и он по идее не ошибётся».

Марафоны: темы «сайт», «объявляй каждый», «уводи сторону тетя гриша подруг добреет» — и сет по такой теме с пустым
кэшем (``по теме из кэша=0``) заказывал платную генерацию при 266+ треках библиотеки. Без сети, денег и реальной
модели: фейковый KIE (``Kie`` из PR-8), фейковая модель чойсера (:class:`FakeModel`), настоящий кэш
``GeneratedMusicLibrary``.
"""

from __future__ import annotations

import logging
import time

import pytest

from rob_box_mcp_tools.engine.gen import gen_session as gs
from rob_box_mcp_tools.engine.theme_choice import (KNOWN_PREFIX, LLM_ID, NEW_PREFIX, NONE_ID, choose_theme,
                                                   open_question, theme_question, valid_theme)
from rob_box_music import knowledge as kn
from rob_box_music.choice import Choice, DeterministicChooser
from rob_box_music.known_themes import CACHE, KNOWLEDGE, UNIVERSE, KnownTheme, cache_theme, known_themes
from rob_box_music.source import CacheTrack, theme_tags
from rob_box_music.theme import match_row
from rob_box_music.universes import universe_of

from .test_adr0158_pr8_gen_set import Kie, _fake_audio_track
from .test_adr0158_pr8_gen_set import _dj as _pr8_dj
from .test_adr0158_pr8_gen_set import _lib

pytestmark = pytest.mark.unit

#: Марафон (лог Шифу): реплика, из которой guard сделал тему «уводи сторону тетя гриша подруг добреет».
ROCK_JUNK = "уводи в сторону рока, тётя Гриша подруг добреет"
#: Марафон-6 n605 (``test_n605_theme_guard_junk_m6``): «сайт» — огрызок слова «сет».
SITE = "поставь сайт из трёх треков"
STRANGER = "поставь сет в стиле Stranger Things"


class FakeModel:
    """Модель чойсера без сети: на вопрос темы (``тема сета:``) отвечает ``answer(question)``, остальные вопросы
    (путь трека) не берёт — ``ModelUnavailable``-подобный отказ, ответ кода. Помнит заданные вопросы темы."""

    def __init__(self, answer):
        self.answer, self.asked = answer, []

    def choose(self, question, *, deadline_ms=0):
        if not question.text.startswith("тема сета:"):
            raise RuntimeError("не вопрос темы")
        self.asked.append(question)
        return Choice(self.answer(question), 0.9, "llm")


def _save(lib, n, style, theme="", title="t"):
    """``n`` инструменталов кэша стиля ``style`` с меткой темы ``theme`` в ``prompt`` (как ``GenSet._job``), каждый —
    своя задача (соседями ставятся свободно)."""
    tags = ", ".join(t for t in (kn.GEN_STYLES[style].tags, theme) if t)
    for i in range(n):
        lib.save_track(b"MP3:150.0", prompt=tags, title=f"{title} {i}", is_instrumental=True, kind="instrumental",
                       style_key=style, genre=style, task_id=f"{style}-{theme}-{i}", variant=0, duration_ms=150000)


def _dj(lib, kie, model=None):
    dj = _pr8_dj(lib, kie)
    if model is not None:
        dj.gen_sets = gs.GenSetMaker(kie, lambda: lib, DeterministicChooser(model))
    return dj


def _line(caplog, marker):
    return next(r.getMessage() for r in caplog.records if marker in r.getMessage())


# ── Известные темы — одно место (rob_box_music.known_themes) ──────────────────────────────────────────────────────


def test_every_knowledge_theme_has_support_in_tables():
    """Тема знания — не просто слово: строка таблицы тем или теги заказа; вселенные — алиас ``UNIVERSES``."""
    for name in kn.KNOWN_THEMES:
        assert match_row(name) is not None or theme_tags(name) != name, name
    themes = {t.name: t for t in known_themes()}
    for universe in kn.UNIVERSES.values():
        assert themes[universe.theme].source == UNIVERSE and universe_of(universe.theme)
    assert {t.source for t in themes.values()} == {UNIVERSE, KNOWLEDGE}


def test_cache_theme_is_the_order_label_after_style_tags():
    rock = kn.GEN_STYLES["rock"].tags
    assert cache_theme(CacheTrack("a", "x", style_key="rock", prompt=f"{rock}, stranger things")) == "stranger things"
    assert cache_theme(CacheTrack("b", "RoBBox rock set", style_key="rock", prompt=rock)) == ""  # заказ без темы
    assert cache_theme(CacheTrack("c", "x", kind="cover", style_key="rock", prompt=f"{rock}, x")) == ""
    assert cache_theme(CacheTrack("d", "x", style_key="rock", prompt="grunge")) == ""  # чужой формат


def test_cache_themes_join_the_list_and_table_tags_count_for_the_knowledge_theme():
    club = kn.GEN_STYLES["club"].tags
    cache = ([CacheTrack(f"s{i}", "x", style_key="club", prompt=f"{club}, stranger things") for i in range(3)]
             + [CacheTrack("c", "x", style_key="club", prompt=f"{club}, {theme_tags('космос')}")])
    themes = {t.name: t for t in known_themes(cache)}
    assert themes["stranger things"] == KnownTheme("stranger things", "stranger things", CACHE, 3)
    assert themes["космос"].source == KNOWLEDGE and themes["космос"].tracks == 1


def test_cache_themes_are_capped(monkeypatch):
    monkeypatch.setattr(kn, "KNOWN_THEMES_CACHE_MAX", 2)
    club = kn.GEN_STYLES["club"].tags
    cache = [CacheTrack(f"{i}", "x", style_key="club", prompt=f"{club}, тема{i}") for i in range(5)]
    assert sum(t.source == CACHE for t in known_themes(cache)) == 2


# ── Вопрос чойсера: закрытый перечень, текст варианта строит код ──────────────────────────────────────────────────


def _ids(question):
    return [o.id for o in question.options]


def test_schema_is_none_request_words_llm_and_known_themes_only():
    known = known_themes()
    question, themes = theme_question("поставь сет про космос и динозавров", "космос и динозавры", known)
    for option_id in _ids(question):
        assert option_id in (NONE_ID, LLM_ID) or option_id.startswith((NEW_PREFIX, KNOWN_PREFIX)), option_id
    assert {themes[f"{KNOWN_PREFIX}{t.name}"] for t in known} == {t.name for t in known}  # все известные темы
    words = [themes[i] for i in _ids(question) if i.startswith(NEW_PREFIX) or i == LLM_ID]
    assert "космос и динозавров" in words and "динозавров" in words and "космос и динозавры" in words
    assert question.weights is kn.THEME_CHOICE_WEIGHTS


def test_the_stt_scrap_is_not_an_option():
    question, themes = theme_question(SITE, None, known_themes())
    assert "сайт" not in themes.values()
    assert DeterministicChooser().choose(question).id == NONE_ID


def test_validator_takes_known_or_request_words_only():
    known = known_themes()
    assert valid_theme("", NONE_ID, known, SITE, None)
    assert valid_theme("космос", f"{KNOWN_PREFIX}космос", known, SITE, None)
    assert not valid_theme("сайт", f"{KNOWN_PREFIX}сайт", known, SITE, None)
    assert valid_theme("динозавров", f"{NEW_PREFIX}0", known, "сет про динозавров", None)
    assert not valid_theme("тётя гриша", f"{NEW_PREFIX}0", known, SITE, None)  # не из слов реплики
    assert not valid_theme("", f"{NEW_PREFIX}0", known, SITE, None)


def test_model_answer_outside_the_list_falls_back_to_code(caplog):
    model = FakeModel(lambda q: "known:тётя гриша")
    with caplog.at_level(logging.INFO):
        pick = choose_theme(None, ROCK_JUNK, known_themes(), DeterministicChooser(model))
    assert model.asked, "открытый вопрос (свои слова, известная тема не узнана) — модели"
    assert pick.id == NONE_ID and pick.theme == "" and pick.provider == "deterministic"


@pytest.mark.parametrize("llm_theme", [None, "рок", "уводи сторону тетя гриша подруг добреет"])
def test_rock_junk_without_a_model_is_no_theme(llm_theme):
    """Модели нет (или не ответила): огрызок без опоры проигрывает «без темы» — и когда LLM его повторила."""
    pick = choose_theme(llm_theme, ROCK_JUNK, known_themes(), DeterministicChooser())
    assert pick.id == NONE_ID and pick.theme == ""


def test_open_question_goes_to_the_model_even_when_code_is_sure():
    """«сет на тему очень странные дела»: код видит новую тему (3 очка против 0.5), но перефраз Stranger Things сверяет
    только модель — вопрос открытый, модель выбрала известную тему кэша."""
    club = kn.GEN_STYLES["club"].tags
    known = known_themes([CacheTrack("s", "x", style_key="club", prompt=f"{club}, stranger things")])
    question, _themes = theme_question("сет на тему очень странные дела", None, known)
    assert open_question(question)
    model = FakeModel(lambda q: "known:stranger things")
    pick = choose_theme(None, "сет на тему очень странные дела", known, DeterministicChooser(model))
    assert pick.theme == "stranger things" and pick.provider == "llm"
    assert not open_question(theme_question(STRANGER, None, known)[0])  # код узнал тему сам


def test_dice_of_the_chain_does_not_throw_the_theme():
    class Dice:
        def choose(self, question, *, deadline_ms=0):
            return Choice(NONE_ID, 0.1, "code")  # кубик выпал на худший вариант

    pick = choose_theme(None, "поставь сет про динозавров", known_themes(), DeterministicChooser(Dice()))
    assert pick.theme == "динозавров" and pick.provider == "deterministic"


# ── Сет: тема из чойсера, кэш по ней, кэш стиля раньше заказа ─────────────────────────────────────────────────────


@pytest.mark.parametrize("llm_theme", [None, "рок", "уводи сторону тетя гриша подруг добреет"])
def test_rock_junk_takes_rock_from_cache_without_orders(tmp_path, monkeypatch, caplog, llm_theme):
    """«уводи в сторону рока, тётя Гриша…» при рок-треках в кэше: тема не мусор (без темы — рок-стилем), треки — из
    кэша, ни одного заказа."""
    monkeypatch.setattr("rob_box_mcp_tools.engine.gen.gen_session.audio_track", _fake_audio_track)
    lib, kie = _lib(tmp_path), Kie()
    _save(lib, 4, "rock")
    model = FakeModel(lambda q: NONE_ID)
    dj = _dj(lib, kie, model)
    with caplog.at_level(logging.INFO):
        out = dj.execute(action="start", theme=llm_theme, tracks=3, heard_text=ROCK_JUNK)
    assert out.success, out.error
    assert out.data["theme"] == "" and out.data["style"] == "rock"
    assert out.data["routes"] and all(r.startswith("cache:") for r in out.data["routes"])
    assert kie.jobs == [], "при рок-треках в кэше заказана платная генерация"
    assert "тетя гриша" not in _line(caplog, "[gen set] слоты").lower()
    assert "→ none" in _line(caplog, "[theme]")


def test_site_scrap_never_becomes_the_theme(tmp_path, monkeypatch):
    """Как n605: идёт сет (реплика без слова «сет» — смена темы посреди сета), «сайт» темой не становится."""
    monkeypatch.setattr("rob_box_mcp_tools.engine.gen.gen_session.audio_track", _fake_audio_track)
    lib, kie = _lib(tmp_path), Kie()
    _save(lib, 6, "rock")
    dj = _dj(lib, kie)
    assert dj.execute(action="start", style="rock", tracks=3, heard_text="включи рок-сет на 3 трека").success
    for theme in ("рок", None):
        out = dj.execute(action="start", theme=theme, style="rock", tracks=3, heard_text=SITE)
        assert out.success, out.error
        assert out.data["theme"] != "сайт" and out.data["theme"] == ""
    assert kie.jobs == []


def test_stranger_things_in_cache_plays_from_cache(tmp_path, monkeypatch, caplog):
    """Тема есть в кэше: код узнал известную тему (вариант кэша на 0.5 выше слов реплики — модель спрошена и выбрала
    известную тему), сет — её треки, ни одного заказа."""
    monkeypatch.setattr("rob_box_mcp_tools.engine.gen.gen_session.audio_track", _fake_audio_track)
    lib, kie = _lib(tmp_path), Kie()
    _save(lib, 3, "rock", "stranger things", title="Stranger Things")
    _save(lib, 3, "rock")
    model = FakeModel(lambda q: f"{KNOWN_PREFIX}stranger things")
    dj = _dj(lib, kie, model)
    with caplog.at_level(logging.INFO):
        out = dj.execute(action="start", style="rock", tracks=3, heard_text=f"{STRANGER} из трёх треков")
    assert out.success, out.error
    assert model.asked, "известная тема против слов реплики — разрыв меньше CLEAR_GAP: спросить модель"
    assert out.data["theme"] == "stranger things"
    assert kie.jobs == []
    assert "по теме из кэша=3 заказ=0" in _line(caplog, "[gen set] слоты")
    assert f"→ {KNOWN_PREFIX}stranger things" in _line(caplog, "[theme]")


# ── Решение Шифу 29.10 «давай В»: тему назвал человек — заказ темы; угадал код/модель — сначала кэш стиля ──────────

ESCOBAR = "Ты диджей Воландеморт и у нас сегодня сэт для пленников прибывших в Эскобар"


@pytest.mark.parametrize("heard,named", [
    (ESCOBAR, True),                                   # «сэт X»
    ("поставь сет про динозавров", True),              # «про X»
    ("включи сет на тему очень странные дела", True),  # «на тему X»
    ("поставь сет в стиле Stranger Things", True),     # латиница, слово сета
    ("сет про космос", True),                          # известная тема, названная человеком
    (ROCK_JUNK, False),                                # огрызок без опоры
    (SITE, False),                                     # «сайт» — не тема
    ("включи рок-сет на 3 трека", False),              # жанр без темы
    ("давай", False),                                  # тема только у LLM (контекст)
])
def test_named_by_human_is_one_predicate(heard, named):
    from rob_box_mcp_tools.engine.theme_grounding import named_by_human

    assert named_by_human(heard) is named


def test_named_theme_without_theme_cache_orders_the_theme_not_the_style_cache(tmp_path, monkeypatch, caplog):
    """«сэт для пленников прибывших в Эскобар» — тему назвал человек, по ней в кэше ничего, club-треки есть: заказ
    треков темы (решение 21.10, #3775), кэш стиля не берётся."""
    monkeypatch.setattr("rob_box_mcp_tools.engine.gen.gen_session.audio_track", _fake_audio_track)
    lib, kie = _lib(tmp_path), Kie()
    _save(lib, 4, "club")
    dj = _dj(lib, kie)
    with caplog.at_level(logging.INFO):
        out = dj.execute(action="start", style="club", tracks=3, heard_text=ESCOBAR)
    assert out.success, out.error
    assert "эскобар" in out.data["theme"]
    assert set(out.data["routes"]) == {"generate:instrumental:V6"}, out.data["routes"]
    assert "по теме из кэша=0 заказ=3" in _line(caplog, "[gen set] слоты")
    assert "названа человеком" in _line(caplog, "[dj_set] тема")


def test_guessed_theme_without_theme_cache_takes_style_cache_first(tmp_path, monkeypatch, caplog):
    """Тема не названа человеком (план с ``theme_named=False``), кэша по ней нет, рок-треки есть: сначала они, заказ —
    только того, что не закрыли."""
    monkeypatch.setattr("rob_box_mcp_tools.engine.gen.gen_session.audio_track", _fake_audio_track)
    lib, kie = _lib(tmp_path), Kie()
    _save(lib, 2, "rock")
    dj = _dj(lib, kie)
    with caplog.at_level(logging.INFO):
        plan = dj.gen_sets.plan("динозавры", "rock", "поставь рок-сет", 4, named=True, theme_named=False)
    assert [r.kind for r in plan.routes] == ["cache", "cache", "generate", "generate"]
    line = _line(caplog, "[gen set] слоты")
    assert "по теме из кэша=0 заказ=2" in line and "из кэша стиля=2, заказ только недостающих" in line
    named = dj.gen_sets.plan("динозавры", "rock", "поставь рок-сет про динозавров", 4, named=True)
    assert [r.kind for r in named.routes] == ["generate"] * 4  # названа человеком — заказ темы


def test_llm_context_theme_takes_style_cache_without_orders(tmp_path, monkeypatch, caplog):
    """Тема только у LLM (в реплике слов темы нет: «поставь сет»), по ней в кэше ничего: рок из кэша, 0 заказов."""
    monkeypatch.setattr("rob_box_mcp_tools.engine.gen.gen_session.audio_track", _fake_audio_track)
    lib, kie = _lib(tmp_path), Kie()
    _save(lib, 4, "rock")
    dj = _dj(lib, kie)
    with caplog.at_level(logging.INFO):
        out = dj.execute(action="start", theme="динозавры", style="rock", tracks=3, heard_text="поставь сет на 3 трека")
    assert out.success, out.error
    assert out.data["theme"] == "динозавры" and kie.jobs == []
    assert "из кэша стиля=3, без заказа" in _line(caplog, "[gen set] слоты")
    assert "не названа человеком" in _line(caplog, "[dj_set] тема")


def test_new_theme_without_any_cache_orders_generation_as_before(tmp_path, monkeypatch, caplog):
    """Новая содержательная тема, ни кэша темы, ни треков стиля — генерация, как раньше."""
    monkeypatch.setattr("rob_box_mcp_tools.engine.gen.gen_session.audio_track", _fake_audio_track)
    lib, kie = _lib(tmp_path), Kie()
    _save(lib, 3, "jazz")  # чужой стиль — не кэш этого сета
    dj = _dj(lib, kie)
    with caplog.at_level(logging.INFO):
        out = dj.execute(action="start", style="club", tracks=3, heard_text="поставь клубный сет про динозавров")
    assert out.success, out.error
    assert out.data["theme"] == "динозавров"
    deadline = time.monotonic() + 5.0  # заказ трека 1 уходит в потоке сета (``GenSet._spawn``)
    while not kie.jobs and time.monotonic() < deadline:
        time.sleep(0.01)
    assert kie.jobs, "новая тема без кэша перестала заказывать генерацию"
    assert all(job.style_tags.endswith("динозавров") for job in kie.jobs)


def test_continue_set_keeps_the_last_theme_without_the_chooser(tmp_path, monkeypatch, caplog):
    monkeypatch.setattr("rob_box_mcp_tools.engine.gen.gen_session.audio_track", _fake_audio_track)
    lib, kie = _lib(tmp_path), Kie()
    _save(lib, 6, "club")
    dj = _dj(lib, kie)
    assert dj.execute(action="start", tracks=2, heard_text="поставь клубный сет про динозавров").success
    caplog.clear()
    with caplog.at_level(logging.INFO):
        out = dj.execute(action="start", tracks=2, heard_text="продолжай сэт")
    assert out.success and out.data["theme"] == "динозавров"
    assert not any("[theme]" in r.getMessage() for r in caplog.records)
