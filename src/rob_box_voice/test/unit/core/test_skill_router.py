"""Фаза 3, задача 3.1/3.4 — детерминированный пред-роутер домена.

Спека, сценарий «детерминированный роутер уверенно распознал домен»:
скилл активируется ДО первого обращения к LLM, то есть фрагмент есть
уже в первом запросе хода и лишнего round-trip нет.

Роутер намеренно консервативен: он обязан быть дешёвым и безопасным, а
не точным. ``None`` («не уверен») — штатный, а не аварийный исход:
модель увидит полный каталог и при необходимости позовёт ``load_skill``
сама. Поэтому тесты ниже проверяют ДВЕ вещи — что явные формулировки
попадают в свой домен, и что двусмысленные честно возвращают ``None``.
"""

from __future__ import annotations

import pytest

from rob_box_core.tool_catalog import skill_names
from rob_box_voice.core.command_parser import CommandParser
from rob_box_voice.core.skill_router import SkillRouter


@pytest.fixture
def router() -> SkillRouter:
    return SkillRouter(
        CommandParser(wake_words=["робот", "робокс"], confidence_base=0.8),
        known_skills=skill_names(),
    )


@pytest.mark.parametrize(
    "text,expected",
    [
        # Музыка разложена по трём разным ментальным моделям — ровно та
        # граница, на которой сидит баг ef73a5fa (gen_play_from_library
        # против Renardo-трека).
        ("сыграй жёсткий барабанный бит", "composer"),
        ("сочини мелодию под дождь", "composer"),
        ("стоп музыку", "composer"),
        ("включи тот грустный трек", "player"),
        ("что у нас в библиотеке", "player"),
        ("сохрани трек в медиатеку", "renardo-library"),
        ("ты диджей теперь", "dj"),
        ("устроим вечеринку", "dj"),
        # Навигация и карта.
        ("поезжай на кухню", "navigation"),
        ("вперёд", "navigation"),
        ("это кухня", "navigation"),
        ("начни картографирование", "mapping"),
        ("построй карту квартиры", "mapping"),
        # Остальные домены.
        ("говори женским голосом", "voice-tts"),
        ("сделай погромче", "voice-tts"),
        ("запомни что я люблю джаз", "memory"),
        ("как меня зовут", "memory"),
        ("поищи погоду в москве", "knowledge"),
        ("помигай синим", "expression"),
    ],
)
def test_explicit_phrases_route_to_their_domain(
    router: SkillRouter, text: str, expected: str
) -> None:
    assert router.route(text) == expected


def test_imperative_beats_genre_mention(router: SkillRouter) -> None:
    """«запомни, что я люблю джаз» — память, а не музыка.

    ``user_wants_music`` срабатывает на слово «джаз», поэтому императивы
    проверяются раньше музыкальных детекторов. Без этого порядка запрос
    уезжал в composer (проверено на живом прогоне при разработке).
    """
    assert router.route("запомни что я люблю джаз") == "memory"
    assert router.route("сыграй джаз") == "composer"


@pytest.mark.parametrize(
    "text",
    [
        "привет как дела",
        "расскажи анекдот",
        "а ты вообще как себя чувствуешь сегодня",
        "",
        "   ",
    ],
)
def test_ambiguous_input_returns_none(router: SkillRouter, text: str) -> None:
    """«Не уверен» — штатный исход, а не ошибка."""
    assert router.route(text) is None


def test_never_returns_a_skill_without_a_fragment() -> None:
    """Активировать домен, у которого нет текста, бессмысленно."""
    router = SkillRouter(
        CommandParser(wake_words=["робот"], confidence_base=0.8),
        known_skills=("composer",),
    )
    assert router.route("сыграй бит") == "composer"
    # navigation известен каталогу, но фрагмента у него нет.
    assert router.route("поезжай на кухню") is None


def test_unknown_domain_does_not_hide_later_ones() -> None:
    """Домен без фрагмента пропускается, а не обрывает поиск.

    Иначе один невключённый скилл прятал бы все следующие в таблице.
    """
    router = SkillRouter(
        CommandParser(wake_words=["робот"], confidence_base=0.8),
        known_skills=("expression",),
    )
    # navigation стоит в таблице раньше expression и не имеет фрагмента.
    assert router.route("помигай синим") == "expression"


def test_router_returns_only_known_skill_names(router: SkillRouter) -> None:
    """Всё, что возвращает роутер, обязано быть именем скилла каталога."""
    known = set(skill_names())
    phrases = [
        "сыграй бит", "включи трек", "поезжай на кухню", "запомни это",
        "поищи новости", "говори тише", "начни картографирование",
        "ты диджей", "помигай", "медиатека",
    ]
    for phrase in phrases:
        skill = router.route(phrase)
        assert skill is None or skill in known


def test_parser_failure_does_not_propagate() -> None:
    """Классификация не имеет права ронять ход."""

    class _Exploding:
        def parse(self, text):  # noqa: ANN001
            raise RuntimeError("парсер упал")

    router = SkillRouter(_Exploding(), known_skills=skill_names())  # type: ignore[arg-type]
    # Фраза без музыкальных и императивных признаков доходит до парсера.
    assert router.route("что там снаружи") is None
