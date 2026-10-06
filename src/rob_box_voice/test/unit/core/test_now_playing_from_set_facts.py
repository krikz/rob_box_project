"""«Что играет / когда будет X» отвечает код по фактам снимка сета (06.10: «Марио реально играет», когда звучал
Тетрис). Поведение: фраза → интент, факты снимка → фраза, факты → атрибуты ``<music_state>``."""

from __future__ import annotations

import pytest

from rob_box_voice.core.media_command_grammar import MediaIntent, parse_media_command
from rob_box_voice.core.media_router import (
    NOTHING_PLAYING_TEXT,
    MediaRouter,
    MediaState,
    media_state_from_snapshot,
)
from rob_box_voice.core.music_player_state import build_music_state_payload, parse_music_state
from rob_box_voice.core.music_state_prompt import MusicStateMemory

FACTS = {"enabled": True, "theme": "Марио, Тетрис, Аладдин", "track_no": 2, "tracks": 4, "melody": "Тетрис",
         "next_melodies": ["Аладдин"], "not_found": ["Марио"], "title": "«Тетрис» · трек 2 из 4"}


def _snap(dj=FACTS):
    return parse_music_state(build_music_state_payload(playing=True, track_id="s:02:B:x", form_ends_at=None,
                                                       dj=dj, ts=100.0))


def _media(dj=FACTS, playing=True):
    return media_state_from_snapshot(playing, _snap(dj))


@pytest.mark.parametrize("text,name", [
    ("что сейчас играет", ""),
    ("а что это за трек", ""),
    ("какая мелодия играет", ""),
    ("робот что играет", ""),
    ("когда будет марио", "марио"),
    ("где марио все его треки", "марио"),
    ("когда будет аладдин", "аладдин"),
])
def test_questions_about_the_set_are_now_playing(text, name):
    command = parse_media_command(text)
    assert command.intent is MediaIntent.NOW_PLAYING and command.name == name


@pytest.mark.parametrize("text", ["где ты", "что ты умеешь", "сделай погромче", "поставь тетрис", "когда ужин"])
def test_other_phrases_are_not_now_playing(text):
    assert parse_media_command(text).intent is not MediaIntent.NOW_PLAYING or MediaRouter().route(
        text, _media()) is None


def test_what_is_playing_names_the_melody_from_snapshot():
    plan = MediaRouter().route("что сейчас играет", _media())
    assert plan.tool_calls == ()
    assert plan.say_ok == ("Сейчас трек 2 из 4: «Тетрис». Дальше по плану: «Аладдин». "
                           "Не нашлось в библиотеке: «Марио».")


def test_when_is_missing_melody_says_it_was_not_found():
    say = MediaRouter().route("когда будет марио", _media()).say_ok
    assert say.startswith("«Марио» в библиотеке мелодий не нашлось") and "«Тетрис»" in say
    assert "играет прямо сейчас" not in say


def test_when_is_next_and_current_melody():
    assert MediaRouter().route("когда будет аладдин", _media()).say_ok == "«Аладдин» по плану через 1 трек — трек 3."
    assert MediaRouter().route("где тетрис", _media()).say_ok.startswith("«Тетрис» играет прямо сейчас")


def test_unknown_name_or_no_facts_goes_to_llm():
    assert MediaRouter().route("где мой телефон", _media()) is None
    no_facts = {"enabled": True, "theme": "космос", "title": "космос · трек 1"}
    assert MediaRouter().route("когда будет марио", _media(no_facts)) is None


def test_nothing_playing():
    assert MediaRouter().route("что играет", MediaState()).say_ok == NOTHING_PLAYING_TEXT


def test_music_state_shows_melody_facts_to_llm():
    memory = MusicStateMemory()
    memory.observe_state(_snap())
    tag = memory.render(now=100.0)
    assert 'melody="Тетрис"' in tag and 'next_melodies="Аладдин"' in tag and 'not_found="Марио"' in tag
    assert 'track="«Тетрис» · трек 2 из 4"' in tag and 'set_tracks="4"' in tag


LAST = {"enabled": True, "theme": "Марио, Тетрис", "track_no": 2, "tracks": 2, "melody": "Super Mario World",
        "played": [[1, "Тетрис"]], "next_melodies": [], "next_known": False, "not_found": [],
        "title": "«Super Mario World» · трек 2 из 2"}


def test_live_0610_last_track_answers():
    """Живой прогон 06.10: «когда будет Тетрис?» на треке 2 из 2 — было «Тетрис будет следующим» (LLM)."""
    say = MediaRouter().route("когда будет тетрис", _media(LAST)).say_ok
    assert say.startswith("«Тетрис» уже был — трек 1.") and "больше не будет" in say and "следующ" not in say
    assert MediaRouter().route("что сейчас играет", _media(LAST)).say_ok == (
        "Сейчас трек 2 из 2: «Super Mario World». Это последний трек сета.")


def test_played_melodies_reach_music_state():
    memory = MusicStateMemory()
    memory.observe_state(_snap(LAST))
    assert 'played="1: Тетрис"' in memory.render(now=100.0)
