"""#4973: «с этими словами» / «с этим текстом» — просьба петь, не инструментал без поля слов.

20.11 заказ «сгенерируй трек … с этими словами» ушёл в ``generate:instrumental``: у инструментала нет поля слов,
робот сказал, что трек готов, а данные человеком слова в заказ не попали. «Со словами» и «с текстом» пение уже
просили; «этими словами» и «этим текстом» — те же фразы в :data:`rob_box_music.knowledge.VOCAL_PHRASES`. «Своими
словами» и «без слов» инструменталом остаются. Решает код (``source._voiced`` читает таблицу), не правило в промпте.
"""

import pytest

from rob_box_music import knowledge as kn
from rob_box_music import source
from rob_box_music.choice import DeterministicChooser
from rob_box_music.source import (
    SingContext,
    SourceContext,
    choose_route,
    choose_sing,
    sing_kind,
    source_ask,
)

DET = DeterministicChooser()
GEN = SourceContext(gen_ready=True, balance=900.0, day_left=120.0)

#: Просьбы спеть данными словами: объект «трек» сам по себе пением не считается — только фраза таблицы.
THESE_WORDS = [
    "сгенерируй трек с этими словами",
    "сгенерируй трек этими словами",
    "сгенерируй трек с этим текстом",
    "сгенерируй трек этим текстом",
    "Робот, сгенерируй трек на китайском голосом с этими словами",
    "еще раз давай попробуем сгенерируй трек на китайским голосом с этими словами через КАИ суно",
    "сделай трек с этим текстом",
    "сгенерируй песню с этими словами",
]

#: Те же глаголы без просьбы петь данными словами: инструментал, как был.
NOT_THESE_WORDS = [
    "сгенерируй трек",
    "сгенерируй трек своими словами",
    "сгенерируй трек своими словами про осень",
    "сгенерируй песню без слов",
    "сгенерируй песню без текста",
    "сделай песню с эхом",
]


@pytest.mark.parametrize("text", THESE_WORDS)
def test_these_words_ask_for_a_song_not_an_instrumental(text):
    """Фраза таблицы делает «трек» песней: путь — платный ``generate:song``, чойсер не зовётся."""
    ask = source_ask(text)
    assert sing_kind(text) == "song" and ask.vocal and not ask.robot_voice, text
    assert choose_sing(text, SingContext(), DET).id == "generate:song:V6", text
    route = choose_route(ask, GEN, DET)
    assert (route.kind, route.id) == ("generate", "generate:song:V6"), text


@pytest.mark.parametrize("text", NOT_THESE_WORDS)
def test_own_words_and_a_bare_track_stay_instrumental(text):
    """«Своими словами», голое «трек», «без слов/текста», правка «с эхом» фразой пения не становятся."""
    ask = source_ask(text)
    assert sing_kind(text) == "" and not ask.vocal, text
    assert choose_route(ask, GEN, DET).id == "generate:instrumental:V6", text


def test_the_phrases_are_read_from_knowledge_not_copied(monkeypatch):
    """Подмена таблицы меняет поведение: без «этими словами» трек снова инструментал, выдуманная фраза — песня."""
    assert "этими словами" in kn.VOCAL_PHRASES and "этим текстом" in kn.VOCAL_PHRASES
    assert sing_kind("сгенерируй трек с этими словами") == "song"
    monkeypatch.setattr(
        kn, "VOCAL_PHRASES", ("со словами", "с текстом", "вот этим куплетом")
    )
    assert sing_kind("сгенерируй трек с этими словами") == ""
    assert sing_kind("сгенерируй трек вот этим куплетом") == "song"
    assert source._voiced(["с", "этими", "словами"]) is False
