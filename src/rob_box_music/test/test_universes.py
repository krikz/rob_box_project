"""#3581: вселенная темы сета по словам человека — закрытая таблица ``knowledge.UNIVERSES``, падеж — окончание из
закрытого списка, опечатка STT — одна правка. Живой сет 12.10: «анакен скайвокер», «звезде смерте»."""

from __future__ import annotations

import pytest

from rob_box_music import knowledge as kn
from rob_box_music.universes import is_mood, universe_of, universe_spans
from rob_box_music.works import alias_pairs


@pytest.mark.parametrize("text", [
    "диджей анакен скайвокер",  # STT 12.10
    "Анакин Скайуокер",
    "веселье на звезде смерте",  # STT 12.10
    "звезда смерти",
    "звёздных войн",
    "диджей Дарт Вейдер",
    "джедаи и ситхи",
    "Star Wars",
])
def test_star_wars_names_places_and_typos(text):
    assert universe_of(text) == "starwars"


@pytest.mark.parametrize("text", [
    "космос", "звёздное небо", "звезда", "потери", "дартс", "тёмная сторона луны", "смерть", "гарри", "", None,
])
def test_ordinary_words_are_not_a_universe(text):
    assert universe_of(text) is None


def test_two_word_alias_is_one_span_and_spans_do_not_overlap():
    words = "веселье звезде смерте анакин".split()
    assert universe_spans(words) == [(1, 3, "starwars"), (3, 4, "starwars")]


def test_universe_theme_is_a_seed_phrase_so_the_archive_finds_it():
    phrases = {phrase for phrase, _query in alias_pairs()}
    for key, universe in kn.UNIVERSES.items():
        assert universe.theme in phrases, key
        assert universe_of(universe.theme) == key


@pytest.mark.parametrize("word, mood", [
    ("веселье", True), ("весёлый", True), ("праздничный", True), ("космос", False), ("звезде", False),
])
def test_mood_words(word, mood):
    assert is_mood(word) is mood
