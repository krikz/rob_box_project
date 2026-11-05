"""Тема DJ-сета без служебных слов просьбы (#4393, HIL 04.11 23:00–23:41 UTC; связано с #3493).

«Останови этот сет и поставь вместо него ночной рок» при ``theme='ночной рок'`` у модели дало тему сета «останови и
вместо него ночной»: кэш промахнулся, ушла генерация KIE (12 кр.). Слова стопа и смены («останови», «вместо него»,
«поставь», «уводи в сторону»), вейк и связки — одна таблица ``rob_box_dialog.knowledge.MUSIC_SERVICE_WORDS``;
тема из слов человека строится без них до похода в кэш, и тема сета — тема модели, если слова человека её
подтверждают. Без сети, денег и модели: чойсер кода (``DeterministicChooser``).
"""

from __future__ import annotations

import pytest

from rob_box_dialog.knowledge import MUSIC_SERVICE_WORDS
from rob_box_mcp_tools.engine.theme_choice import LLM_ID, NEW_PREFIX, NONE_ID, choose_theme, chosen_slots
from rob_box_mcp_tools.engine.theme_grounding import confirmed_by_words, request_theme
from rob_box_music.choice import DeterministicChooser
from rob_box_music.known_themes import CACHE, KnownTheme
from rob_box_music.source import theme_tags
from rob_box_voice.core.media_command_grammar import SET_REQUEST_WORDS

pytestmark = pytest.mark.unit

LLM = "ночной рок"
#: HIL 04.11 и вариации той же просьбы со словом сета: тема — «ночной рок» модели.
WITH_SET = (
    "Останови этот сет и поставь вместо него ночной рок",
    "Робот, останови этот сет и поставь вместо него ночной рок",
    "Робот, хватит этого сета, а теперь поставь сет ночной рок",
    "выключи этот сет и включи вместо него ночной рок",
    "смени сет на ночной рок",
)
#: Те же просьбы без слова сета: тема та же, что у голого «поставь ночной рок».
WITHOUT_SET = (
    "поставь ночной рок",
    "Робот, давай вместо этого ночной рок",
    "хватит этого, поставь лучше ночной рок",
    "выключи это и включи ночной рок",
)


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, msg):
        self.lines.append(msg)

    warning = info


def _pick(heard_text, known=()):
    return choose_theme(LLM, heard_text, known, DeterministicChooser(), log=_Log())


@pytest.mark.parametrize("heard_text", WITH_SET + WITHOUT_SET)
def test_service_words_are_not_theme(heard_text):
    """Слова темы реплики — без стопа, «вместо него», «поставь», вейка и связок: остаётся «ночной»."""
    assert request_theme(heard_text) == "ночной"


def test_hil_phrase_takes_model_theme():
    """HIL 04.11: тема сета — «ночной рок» модели, а не «останови и вместо него ночной»."""
    pick = _pick(WITH_SET[0])
    assert (pick.theme, pick.id) == (LLM, LLM_ID)


@pytest.mark.parametrize("heard_text", WITH_SET)
def test_theme_stable_with_set_word(heard_text):
    """Кэш-ключ стабилен: любая формулировка смены со словом сета — тема модели «ночной рок», названа человеком."""
    theme, _persona, named = chosen_slots(LLM, None, heard_text, None, (), DeterministicChooser(), _Log())
    assert (theme, named) == (LLM, True)


@pytest.mark.parametrize("heard_text", WITHOUT_SET)
def test_theme_stable_without_set_word(heard_text):
    """Без слова сета — то же, что у голого «поставь ночной рок» (стиль из слов человека, без огрызков просьбы)."""
    bare = chosen_slots(LLM, None, WITHOUT_SET[0], None, (), DeterministicChooser(), _Log())
    assert chosen_slots(LLM, None, heard_text, None, (), DeterministicChooser(), _Log()) == bare
    assert not any(w in bare[0].split() for w in ("останови", "вместо", "него", "этого", "лучше"))


@pytest.mark.parametrize("heard_text", WITH_SET + WITHOUT_SET)
def test_cached_theme_hit_for_every_wording(heard_text):
    """Тема «ночной рок» в кэше (3 трека): чойсер выбирает её при любой формулировке — платной генерации нет."""
    known = (KnownTheme(LLM, theme_tags(LLM), CACHE, 3),)
    pick = _pick(heard_text, known)
    assert pick.theme == LLM and not pick.id.startswith(NEW_PREFIX)


def test_rock_switch_n604_unchanged():
    """Марафон n604 «уводи в сторону рока…»: «уводи сторону» темой не становится, огрызок STT — «без темы»."""
    assert request_theme("Робот, уводи в сторону рока") == ""
    pick = choose_theme("рок", "уводи в сторону рока, тётя Гриша подруг добреет", (), DeterministicChooser(),
                        log=_Log())
    assert pick.id == NONE_ID and "уводи" not in pick.theme


def test_one_table_feeds_set_grammar():
    """Одно место правды: служебные слова — часть таблицы просьбы о сете грамматики."""
    assert MUSIC_SERVICE_WORDS <= SET_REQUEST_WORDS
    assert {"останови", "вместо", "него", "уводи", "сторону", "робот"} <= MUSIC_SERVICE_WORDS


# -- решение Шифу 05.11 «а»: тема модели, если слова человека её подтверждают, и без слова «сет» --------------------


@pytest.mark.parametrize("heard_text", WITHOUT_SET)
def test_model_theme_confirmed_by_words_without_set_word(heard_text):
    """«Поставь ночной рок» (без «сет»): слова темы модели «ночной рок» все сказаны человеком после снятия служебных -
    тема сета «ночной рок» модели, а не «без темы» (сет по стилю)."""
    assert confirmed_by_words(LLM, heard_text)
    pick = _pick(heard_text)
    assert (pick.theme, pick.id) == (LLM, LLM_ID)


def test_model_theme_words_not_said_are_not_confirmed():
    """Слово темы, которого человек не говорил, - не подтверждение: n604 огрызок STT и чужая тема остаются без темы."""
    assert not confirmed_by_words("рок", "уводи сторону тётя гриша подруг добреет")
    assert not confirmed_by_words("ночной город", "поставь ночной рок")
    assert not confirmed_by_words("вместо него", "Робот, давай вместо этого ночной рок")  # служебные слова - не тема
    assert not confirmed_by_words("", "поставь ночной рок")
    pick = choose_theme("ночной город", "поставь ночной рок", (), DeterministicChooser(), log=_Log())
    assert pick.theme != "ночной город"
