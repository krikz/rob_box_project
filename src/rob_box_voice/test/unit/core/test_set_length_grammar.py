"""Длина DJ-сета из фразы — решает код (решение Шифу 06.10: сет set80093 дошёл до 53-го трека).

Поведение: «сет на 3 трека», «три десятка», «на полчаса» → ``dj_set(tracks=N)``; без числа ``tracks`` в вызове нет
(длину решает тул: 10 или остаток идущего сета); тема-перечисление после двоеточия уходит в ``theme`` как сказана.
"""

from __future__ import annotations

import pytest

from rob_box_music.set_plan import MAX_TRACKS, TRACK_SECONDS
from rob_box_voice.core.media_command_grammar import MediaIntent, parse_media_command
from rob_box_voice.core.media_router import MediaRouter, MediaState
from rob_box_voice.core.set_length_words import split_set_length

V2 = MediaRouter()


def _dj_args(text: str, media: MediaState = MediaState()):
    plan = V2.route(text, media)
    assert plan is not None and [c.name for c in plan.tool_calls] == ["dj_set"], text
    return plan.tool_calls[0].arguments


@pytest.mark.parametrize("text,args", [
    ("Робот, включи сет на 3 трека: Марио, Тетрис, Зельда",
     {"action": "start", "theme": "Марио, Тетрис, Зельда", "tracks": 3}),
    ("включи сет на три десятка треков про космос", {"action": "start", "theme": "космос", "tracks": 30}),
    ("сет на 20 треков на тему космос", {"action": "start", "theme": "космос", "tracks": 20}),
    ("включи диджей сет на двадцать пять треков", {"action": "start", "tracks": 25}),
    ("включи сет на десяток треков", {"action": "start", "tracks": 10}),
    ("включи диджей сет на полчаса", {"action": "start", "tracks": round(30 * 60 / TRACK_SECONDS)}),
    ("включи рейв на 40 минут про котов",
     {"action": "start", "theme": "котов", "style": "rave", "tracks": round(40 * 60 / TRACK_SECONDS)}),
    ("включи диджей сет на два часа", {"action": "start", "tracks": MAX_TRACKS}),  # зажато, а не отказ
    ("включи сет: Марио, Тетрис и Контра", {"action": "start", "theme": "Марио, Тетрис и Контра"}),
    ("включи сет на тему космос", {"action": "start", "theme": "космос"}),
    ("сыграй mambo nr 5 сет", {"action": "start", "theme": "mambo nr 5"}),  # цифра темы — не длина
])
def test_set_length_from_the_phrase_goes_to_dj_set_as_a_number(text, args):
    assert _dj_args(text) == args


def test_no_number_means_no_tracks_argument_and_non_dj_phrases_keep_their_meaning():
    assert parse_media_command("включи диджей сет").tracks == 0
    assert parse_media_command("поставь 3 трека").intent is not MediaIntent.DJ
    assert parse_media_command("ты диджей Вася: давай").set_theme == ""  # двоеточие без темы — не перечисление


def test_split_keeps_the_punctuation_of_the_list_theme():
    assert split_set_length("включи сет на 3 трека: Марио, Тетрис") == (3, "включи сет : Марио, Тетрис")
    assert split_set_length("сыграй mambo nr 5 сет") == (0, "сыграй mambo nr 5 сет")
