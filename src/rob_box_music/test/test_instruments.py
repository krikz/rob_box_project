"""ADR-0157 PR-1 (#3558): таблица ``INSTRUMENTS`` и выбор лида по инструменту заказа.

Прогон 09.10 (§1.1): «сыграй Баха на балалайке/трамбоне/арфе», «Токката на кселофоне» — инструмент учтён 0 из 10.
Здесь — поведение модуля ``rob_box_music.instruments``: слово человека (падеж, опечатка STT) → ключ таблицы → синт
лида; синта нет — первый сосед той же семьи; лид песни звучит этим синтом.
"""

from __future__ import annotations

import random

import pytest

from rob_box_music import knowledge as kn
from rob_box_music.arrange.compose import compose
from rob_box_music.arrange.song import song_track
from rob_box_music.instruments import (
    instrument_of, lead_choice, near, neighbors, shifted, split_instrument, within_one,
)
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import seeded_profile

from test_song import material


def test_every_synth_of_the_table_is_a_lead_of_the_palette():
    """Синт инструмента — только существующий лид палитры: иначе валидатор трека откажет и заказ замолчит."""
    for key, ins in kn.INSTRUMENTS.items():
        assert ins.synth is None or ins.synth in kn.SYNTH_PALETTE["lead"], key


def test_first_lineup_of_adr_0157():
    """Состав §3.4 + #3592: ситар, маримба, калимба."""
    assert set(kn.INSTRUMENTS) == {"balalaika", "harp", "trombone", "xylophone", "piano", "guitar", "flute", "organ",
                                   "sitar", "marimba", "kalimba"}
    assert kn.INSTRUMENTS["balalaika"].synth is None and kn.INSTRUMENTS["trombone"].synth == "brass"
    assert kn.INSTRUMENTS["xylophone"].synth == "marimba"


@pytest.mark.parametrize("word,key", [
    ("балалайке", "balalaika"), ("балалайка", "balalaika"), ("балалайку", "balalaika"), ("Балалайке", "balalaika"),
    ("трамбоне", "trombone"), ("тромбоне", "trombone"), ("трамбон", "trombone"), ("тромбонов", "trombone"),
    ("кселофоне", "xylophone"), ("ксилофоне", "xylophone"), ("ксилафоне", "xylophone"),
    ("арфе", "harp"), ("арфой", "harp"), ("пианино", "piano"), ("рояле", "piano"), ("фортепиано", "piano"),
    ("гитаре", "guitar"), ("флейте", "flute"), ("органе", "organ"),
    ("ситаре", "sitar"), ("ситару", "sitar"), ("маримбе", "marimba"), ("калимбе", "kalimba"),
])
def test_word_forms_and_stt_typos(word, key):
    assert instrument_of(word) == key


@pytest.mark.parametrize("word", ["организуй", "тему", "арене", "космосе", "гитарист", "баха"])
def test_not_an_instrument(word):
    """Падежи — закрытый список окончаний, опечатки — одна правка и не для коротких слов."""
    assert instrument_of(word) is None


@pytest.mark.parametrize("word,key", [
    ("ситар", "sitar"), ("ситаре", "sitar"), ("ситару", "sitar"), ("гитар", "guitar"), ("гитаре", "guitar"),
    ("гитару", "guitar"), ("ситера", "sitar"),
])
def test_exact_word_is_never_pulled_into_a_neighbour(word, key):
    """#3592: «ситар» на расстоянии 1 от «гитар» — точная форма таблицы побеждает опечаточный поиск."""
    assert instrument_of(word) == key


@pytest.mark.parametrize("word", ["китар", "китару", "китаре"])
def test_typo_one_edit_from_two_instruments_is_refused_not_guessed(word):
    """Слово на одной правке от форм ДВУХ разных инструментов («гитар»/«ситар») — ``None``, а не первый по таблице."""
    assert instrument_of(word) is None


@pytest.mark.parametrize("word", ["банджо", "мандолине", "скрипке", "баяне", "трубе"])
def test_known_instrument_without_a_timbre_is_not_rewritten_into_a_table_one(word):
    """Инструмент, которого у робота нет, не подменяется ближайшим по правке: слот не выделяется."""
    assert instrument_of(word) is None
    assert split_instrument(["сыграй", "на", word]) == ("", ["сыграй", "на", word])


def test_unsupported_stems_do_not_overlap_the_table():
    stems = {s for ins in kn.INSTRUMENTS.values() for s in ins.stems}
    assert not stems & set(kn.UNSUPPORTED_INSTRUMENT_STEMS)


def test_within_one_is_one_edit():
    assert within_one("сфграй", "сыграй") and within_one("трамбон", "тромбон") and within_one("арфа", "арфа")
    assert within_one("сыгай", "сыграй") and within_one("сыграйй", "сыграй")
    assert not within_one("сфгрпй", "сыграй") and not within_one("сыг", "сыграй")
    assert near("сфграй", ["включи", "сыграй"]) == "сыграй" and near("сыгр", ["сыграй"]) is None


@pytest.mark.parametrize("words,key,rest", [
    ("сыграй баха на балалайке", "balalaika", "сыграй баха"),
    ("сыграть баха на инструменте трамбон", "trombone", "сыграть баха"),
    ("сыграй на пианино пожалуйста", "piano", "сыграй пожалуйста"),
    ("включи сет на тему космос", "", "включи сет на тему космос"),
    ("сыграй баха", "", "сыграй баха"),
])
def test_slot_on_instrument_is_cut_out_of_the_words(words, key, rest):
    got, left = split_instrument(words.split())
    assert (got, " ".join(left)) == (key, rest)


def test_lead_choice_honest_degradation_by_family():
    """Синта нет — играет первый сосед той же семьи из таблицы; семья без соседей — свой тембр трека."""
    assert lead_choice("trombone").synth == "brass" and not lead_choice("trombone").substituted
    bal = lead_choice("balalaika")
    assert bal.substituted and bal.played == neighbors("balalaika")[0] == "guitar" and bal.synth == "pluck"
    assert neighbors("balalaika") == ("guitar", "harp", "sitar")
    assert lead_choice("organ").played == "piano" and lead_choice("organ").synth == "rhpiano"
    flute = lead_choice("flute")
    assert flute.played == "" and flute.synth is None
    assert lead_choice("") is None and lead_choice("kazoo") is None


def test_shift_only_inside_the_corridor():
    assert shifted([60, 72], -12, (21, 108)) == -12
    assert shifted([30, 72], -12, (21, 108)) == 0
    assert shifted([], 12, (21, 108)) == 0


@pytest.mark.parametrize("key", sorted(kn.INSTRUMENTS))
def test_song_lead_plays_the_instrument_synth(key):
    """Лид песни — синт инструмента (или соседа); трек проходит валидатор, остальные роли не меняются."""
    lead = lead_choice(key)
    plain = song_track(material(), seed=1)
    track = song_track(material(), seed=1, lead_synth=lead.synth, lead_shift=lead.octave_shift)
    assert track.parts["lead"].synth_or_sample == (lead.synth or plain.parts["lead"].synth_or_sample)
    step = shifted([e.midi for e in plain.parts["lead"].pitches], lead.octave_shift, kn.SONG_REGISTERS["lead"])
    assert [e.midi for e in track.parts["lead"].pitches] == [e.midi + step for e in plain.parts["lead"].pitches]
    assert all(track.parts[r].synth_or_sample == plain.parts[r].synth_or_sample for r in ("bass", "pad"))


def test_club_track_lead_takes_the_instrument_synth():
    """«Сыграй на арфе» без названия — одиночный трек, лид — синт арфы вместо тембра семьи темы."""
    plan = seeded_plan(seeded_profile("музыку"), random.Random(3).randint(0, 999))
    track = compose(plan, 1, lead_synth=kn.INSTRUMENTS["harp"].synth)
    assert track.parts["lead"].synth_or_sample == "karp"
