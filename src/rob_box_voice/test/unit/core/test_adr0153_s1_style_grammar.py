"""ADR-0153 S1 — стиль сета решает грамматика роутера (код), не LLM: «рейв на тему X» → ``dj_set(style=rave)``.

Поведение: фраза → интент, стиль, тема и аргументы тула. Слова стиля — таблица ``knowledge.STYLE_WORDS`` (одна).
"""

from __future__ import annotations

import pytest

from rob_box_music import knowledge as kn
from rob_box_voice.core.media_command_grammar import MediaIntent, parse_media_command
from rob_box_voice.core.media_router import MediaRouter, MediaState

V2 = MediaRouter()


def _dj_args(text: str):
    plan = V2.route(text, MediaState())
    assert plan is not None and [c.name for c in plan.tool_calls] == ["dj_set"], text
    return plan.tool_calls[0].arguments


@pytest.mark.parametrize("text,theme", [
    ("включи рейв", None),
    ("сыграй эйсид", None),
    ("поставь рэйв музыку", None),
    ("давай рейв на тему космос", "космос"),
    ("рейв-сет про котов", "котов"),
    ("включи хардкор вечеринку на тему роботов", "роботов"),
    ("включи диджей сет на тему космос в стиле рейв", "космос"),
    ("включи диджей сет в стиле хардкор на тему зима", "зима"),
])
def test_style_words_start_a_rave_set_and_stay_out_of_the_theme(text, theme):
    command = parse_media_command(text)
    assert command.intent is MediaIntent.DJ and command.style == "rave" and command.style in kn.STYLES
    args = _dj_args(text)
    assert args["action"] == "start" and args["style"] == "rave" and args.get("theme") == theme


def test_persona_and_style_from_the_hint_phrase():
    args = _dj_args("ты диджей Вася, у нас сегодня рейв")
    assert args == {"action": "start", "persona": "диджей Вася", "style": "rave"}


@pytest.mark.parametrize("text", ["включи диджей сет на тему космос", "ты диджей Вася"])
def test_set_without_style_words_names_no_style(text):
    """Нет слов стиля — ``style`` в тул не идёт: тул играет клуб (``auto``)."""
    assert parse_media_command(text).style == "" and "style" not in _dj_args(text)


@pytest.mark.parametrize("text", ["что такое рейв", "расскажи про рейв", "поставь калинку", "давай тему терминатора"])
def test_style_word_in_another_request_is_not_a_set(text):
    assert parse_media_command(text).intent is not MediaIntent.DJ


def test_every_style_word_maps_to_a_style_key():
    assert set(kn.STYLE_WORDS.values()) <= set(kn.STYLES)
    for stem in kn.STYLE_WORDS:
        assert parse_media_command(f"включи {stem} сет").style == kn.STYLE_WORDS[stem], stem


@pytest.mark.parametrize("text,theme", [
    ("включи клубный сет на тему киберпанк", "киберпанк"),
    ("включи клубную музыку на тему космос", "космос"),
    ("давай клубняк про роботов", "роботов"),
    ("включи club сет", None),
    ("включи клубный диджей сет", None),
])
def test_club_words_start_a_club_set_even_when_the_theme_row_has_another_style(text, theme):
    """#3508: клуб — стиль по умолчанию, но «киберпанк» без слова даёт synthwave; слово клуба выбирает его явно."""
    args = _dj_args(text)
    assert args["action"] == "start" and args["style"] == "club" and args.get("theme") == theme


@pytest.mark.parametrize("text", ["поставь клубный трек", "включи клубную музыку", "включи клубняк"])
def test_club_word_without_set_word_or_theme_is_still_a_single_track(text):
    """Клуб — стиль одиночного трека по умолчанию: без слова сета и темы это не заказ сета."""
    plan = V2.route(text, MediaState())
    assert plan is not None and [c.name for c in plan.tool_calls] == ["request_music"], text
