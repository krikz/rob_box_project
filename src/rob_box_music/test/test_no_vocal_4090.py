"""#4090 E: ветка ``_no_vocal`` - «сгенерируй песню без вокала» - инструментал, а не песня (ADR-0148, ревью #4071).

Мутация «``_no_vocal`` всегда ложь» выживала: рядом с «без вокала» были только фразы, где пения нет и без неё
(«сгенерируй трек без вокала» - дополнение «трек» без голоса не пение). Здесь отказ от пения - единственное, что отделяет
песню от инструментала: дополнение «песню» при глаголе «сгенерируй» без него дало бы ``generate:song``. Синонимы
(«без слов», «без пения», «инструментальную») - одна таблица ``knowledge.NO_VOCAL_STEMS`` / ``INSTRUMENTAL_STEMS``.
"""

import pytest

from rob_box_music import knowledge as kn
from rob_box_music import source
from rob_box_music.choice import DeterministicChooser
from rob_box_music.source import SingContext, SourceContext, choose_route, choose_sing, sing_kind, source_ask

DET = DeterministicChooser()
GEN = SourceContext(gen_ready=True, balance=900.0, day_left=120.0)

#: Просьбы сгенерировать песню, где человек отказывается от пения: инструментал.
INSTRUMENTAL_ASKS = [
    "сгенерируй песню без вокала",
    "Робот, сгенерируй песню без вокала",
    "сгенерируй песню без слов",
    "сгенерируй песню про енота без слов",
    "сгенерируй песню без пения",
    "сгенерируй песню без голоса",
    "сгенерируй песню без текста",
    "сгенерируй инструментальную песню",
    "сгенерируй инструментальную песню про енота",
    "сочини песню без вокала",
    "запиши песню без слов",
    "сделай песню без пения",
    "сгенерируй песню без вокала и без слов",
    "сгенерируй мне весёлую песенку без слов",
]
#: Те же слова, но пение просят: отказ в них не про вокал или вокал просят прямо.
STILL_SINGING = [
    "сгенерируй песню",
    "сгенерируй песню с вокалом",
    "сгенерируй песню со словами",
    "сгенерируй песню с текстом про енота",
    "сгенерируй песню без лишних слов",
    "сгенерируй песню без шума",
    "сгенерируй песню про слова и голос",
    "сгенерируй песню про енота без паузы",
]


@pytest.mark.parametrize("text", INSTRUMENTAL_ASKS)
def test_a_song_without_vocal_is_asked_as_an_instrumental_not_a_song(text):
    """Отказ от пения: не пение (``sing_kind`` пуст), слоты без ``vocal``/``robot_voice``, путь пения не выбирается, а
    путь музыки - платный инструментал (кандидатов-песен нет)."""
    ask = source_ask(text)
    assert sing_kind(text) == "" and not ask.vocal and not ask.robot_voice and ask.live == ""
    assert choose_sing(text, SingContext(), DET).id == ""
    route = choose_route(ask, GEN, DET)
    assert (route.kind, route.id) == ("generate", "generate:instrumental:V6"), text


@pytest.mark.parametrize("text", STILL_SINGING)
def test_the_same_words_without_a_refusal_of_singing_stay_a_song(text):
    """Контроль: ветка не шире, чем «без» + слово пения: «без лишних слов», «без шума», «с вокалом» - песня как была."""
    ask = source_ask(text)
    assert sing_kind(text) == "song" and ask.vocal
    assert choose_route(ask, GEN, DET).id == "generate:song:V6", text


def test_the_synonyms_live_in_one_table_of_knowledge(monkeypatch):
    """Слова отказа читаются из ``knowledge``, а не из копии в коде: подмена таблицы меняет поведение - и «без вокала»
    перестаёт быть отказом, а выдуманное слово им становится."""
    assert source_ask("сгенерируй песню без вокала").vocal is False
    monkeypatch.setattr(kn, "NO_VOCAL_STEMS", ("нотк",))
    monkeypatch.setattr(kn, "INSTRUMENTAL_STEMS", ("минусов",))
    assert sing_kind("сгенерируй песню без вокала") == "song"
    assert sing_kind("сгенерируй песню без нотки") == "" and sing_kind("сгенерируй песню-минусовку") == ""
    assert sing_kind("сгенерируй инструментальную песню") == "song"


def test_every_stem_of_the_table_is_a_refusal_after_without_and_not_a_bare_word():
    """Каждая основа таблицы - отказ только сразу после «без» (``VOCAL_NEGATIONS``); «слов» без «без» - не отказ."""
    forms = {"вокал": "вокала", "слов": "слов", "пени": "пения", "голос": "голоса", "текст": "текста"}
    assert set(forms) == set(kn.NO_VOCAL_STEMS), "новая основа таблицы - новая строка этого теста"
    for stem, form in forms.items():
        assert source._no_vocal(["сгенерируй", "песню", kn.VOCAL_NEGATIONS[0], form]), stem
        assert not source._no_vocal(["сгенерируй", "песню", "про", form]), stem
        assert not source._no_vocal(["сгенерируй", "песню", kn.VOCAL_NEGATIONS[0], "лишних", form]), stem
    assert source._no_vocal(["инструментальную", "песню"]) and not source._no_vocal(["песню", "про", "енота"])


def test_a_singing_verb_still_sings_even_with_the_refusal_word():
    """Правка не меняет прежнего: глагол пения («спой») пение, даже с «без вокала» (ветка ``rule.bare``)."""
    assert sing_kind("спой песню без вокала") == "song"
