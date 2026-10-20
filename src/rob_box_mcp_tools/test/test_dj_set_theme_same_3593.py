"""#3593, живой сет Шифу 13.10: «переходим на Рок сэт тема таже» — LLM передала верную тему «проникновение нирвану»,
код подменил её словами «переходим таже» → рок-сет заиграл Chicken Dance / Macarena / Dancing Queen.

Решает код (ADR-0148): «тема та же / таже / тот же / прежняя» и глаголы смены стиля («переходим», «переключись»,
«смени») — просьба, не тема; без новой темы тема и персона — последнего сета; тема реплики из служебных слов тему
LLM не заменяет. Слова — одна таблица ``media_command_grammar`` (``SET_SWITCH_WORDS``, ``SET_SAME_WORDS``).
"""

from __future__ import annotations

import logging

import pytest

from rob_box_mcp_tools.engine.theme_grounding import continued, grounded_theme, heard_theme, set_slots

pytestmark = pytest.mark.unit

NIRVANA = "проникновение нирвану"
KURT = "диджей Курт Кабейн"
LAST = (NIRVANA, KURT)
LIVE = "переходим на Рок сэт тема таже"
LOG = logging.getLogger("test3593")


def test_live_phrase_keeps_the_llm_theme_and_is_not_replaced():
    assert heard_theme(LIVE) == ""
    assert grounded_theme(NIRVANA, LIVE) == (NIRVANA, False)
    assert set_slots(NIRVANA, KURT, LIVE, LAST, LOG) == (NIRVANA, KURT)


@pytest.mark.parametrize("phrase", [
    LIVE,
    "Робот, переходим на рок сет, тема та же",
    "давай то же самое, но роком",
    "смени стиль на джаз",
    "та же тема, только техно",
    "тот же сет но джаз",
    "переключись на рок сет",
    "переходи на джаз, прежняя тема",
    "старая тема, только джаз",
])
def test_same_theme_or_style_switch_takes_the_last_set(phrase):
    assert continued(phrase, LAST) == LAST
    assert set_slots("", None, phrase, LAST, LOG) == LAST


@pytest.mark.parametrize("phrase", [LIVE, "та же тема, только техно", "смени стиль на джаз"])
def test_llm_theme_survives_a_style_switch_without_last_set(phrase):
    assert set_slots(NIRVANA, KURT, phrase, None, LOG) == (NIRVANA, KURT)


@pytest.mark.parametrize("phrase", [
    "переходим на рок сет про космос",
    "переключись на джаз, тема космос",
    "ты диджей Курт Кобейн и у нас сегодня космос",
    "давай ещё сет на тему космос",
])
def test_a_new_theme_in_the_phrase_still_wins(phrase):
    assert continued(phrase, LAST) is None
    assert set_slots(NIRVANA, None, phrase, LAST, LOG)[0] == "космос"


def test_regression_3581_continue_set_still_takes_the_last_set():
    assert continued("продолжай сэт", LAST) == LAST
    assert continued("продолжай сэт", None) is None


def test_llm_theme_from_an_old_utterance_is_still_replaced_by_the_new_one():
    theme, replaced = grounded_theme("Увертюра 1812, мегасет для геймеров", "ты диджей X и у нас сегодня космос")
    assert (theme, replaced) == ("космос", True)


def test_switch_verbs_are_request_words_and_never_the_theme():
    for word in ("переходим", "переключись", "смени", "таже"):
        assert heard_theme(f"{word} сет") == ""
