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
from rob_box_dialog.music_state import MusicStateMemory

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
    assert plan.say_ok == ("Сейчас играет «Тетрис». Дальше по плану: «Аладдин». "
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
    tag = memory.section(now=100.0)
    assert tag["melody"] == "Тетрис" and tag["next_melodies"] == "Аладдин" and tag["not_found"] == "Марио"
    assert tag["track"] == "«Тетрис» · трек 2 из 4" and tag["set_tracks"] == "4"


LAST = {"enabled": True, "theme": "Марио, Тетрис", "track_no": 2, "tracks": 2, "melody": "Super Mario World",
        "played": [[1, "Тетрис"]], "next_melodies": [], "next_known": False, "not_found": [],
        "title": "«Super Mario World» · трек 2 из 2"}


def test_live_0610_last_track_answers():
    """Живой прогон 06.10: «когда будет Тетрис?» на треке 2 из 2 — было «Тетрис будет следующим» (LLM)."""
    say = MediaRouter().route("когда будет тетрис", _media(LAST)).say_ok
    assert say.startswith("«Тетрис» уже был — трек 1.") and "больше не будет" in say and "следующ" not in say
    assert MediaRouter().route("что сейчас играет", _media(LAST)).say_ok == (
        "Сейчас играет «Super Mario World». Это последний трек сета.")


def test_played_melodies_reach_music_state():
    memory = MusicStateMemory()
    memory.observe_state(_snap(LAST))
    assert memory.section(now=100.0)["played"] == "1: Тетрис"


# ── #3613: одиночный трек и пьеса деки v2 — «что играет» отвечает код по фактам владельца плеера ──────────────────

#: Свой синт-трек (#4060): названия нет - ``title`` это метка деки «тема · темп», слова заказа в ``name`` не кладутся.
CLUB = {"enabled": False, "title": "космос · 124 BPM", "source": "synth", "style": "club",
        "bpm": 124, "started_at": 40.0}
PIECE = {"enabled": False, "title": "Toccata", "name": "Toccata, Бах", "source": "piece", "bpm": 96,
         "instrument_played": "organ", "started_at": 70.0, "length_s": 120.0}


def _track(dj, now=100.0):
    return media_state_from_snapshot(True, _snap(dj), now=now)


@pytest.mark.parametrize("text", ["что сейчас играет", "что ты включил", "а что это за трек",
                                  "а раскажи что ты сейчас включил", "скажи какой трек играет"])
def test_live_3613_single_club_track_is_named_by_code(text):
    """Живой лог #3613: при ``[music v2] started req99682…`` LLM сказала «в плеере тишина». Теперь — код, без LLM."""
    plan = MediaRouter().route(text, _track(CLUB))
    assert plan.tool_calls == ()
    assert plan.say_ok == ("Сейчас играет музыка — клуб, синтез Ренардо, 124 удара в минуту. "
                           "Играет уже 1 минуту и повторяется, пока не скажешь «стоп».")


def test_4060_order_words_of_an_old_server_are_not_the_title_of_the_club_track():
    """Живой 28.10: слова заказа в ``name`` старого сервера («какую нибудь спокойную музыку») названием не служат."""
    old_server = {**CLUB, "name": "какую нибудь спокойную музыку"}
    say = MediaRouter().route("что сейчас играет", _track(old_server)).say_ok
    assert say == MediaRouter().route("что сейчас играет", _track(CLUB)).say_ok and "спокойную" not in say


def test_piece_says_source_instrument_and_time_left():
    say = MediaRouter().route("что играет", _track(PIECE)).say_ok
    assert say == ("Сейчас играет «Toccata, Бах» — пьеса по нотам партитуры, 96 ударов в минуту, на органе. "
                   "Играет уже 30 секунд, осталось 1 минута 30 секунд.")


def test_piece_author_stays_outside_the_quotes_like_the_launch_phrase():
    """Живой прогон 13.10: запуск «Ставлю «…Toccata», Louis Vierne.» — ответ «что играет» называет так же."""
    say = MediaRouter().route("что играет", _track({**PIECE, "name": "Toccata", "composer": "Louis Vierne"})).say_ok
    assert say.startswith("Сейчас играет «Toccata», Louis Vierne — пьеса по нотам партитуры")


def test_music_state_tag_tells_llm_the_piece_ends_and_its_source():
    """``<music_state>`` для хода LLM: пьеса — один проход (``repeat=once``, сколько осталось), источник — из снимка."""
    memory = MusicStateMemory()
    memory.observe_state(_snap(PIECE))
    tag = memory.section(now=100.0)
    assert tag["repeat"] == "once" and tag["ends_in_s"] == "90" and tag["source"] == "piece"
    memory.observe_state(_snap(CLUB))
    assert memory.section(now=100.0)["repeat"] == "loop"


def test_old_snapshot_without_facts_still_names_the_title():
    """Снимок без фактов заказа (сервер старее подписчика) — название из ``title``, без выдумок об источнике."""
    say = MediaRouter().route("что играет", _track({"enabled": False, "title": "космос · 124 BPM"})).say_ok
    assert say == "Сейчас играет «космос · 124 BPM»."


# ── #4060: «что играет» - название, стиль и темп трека, а не номер и id ────────────────────────────────────────────

GEN_SET = {"enabled": True, "theme": "Неоновая ночь", "track_no": 3, "tracks": 10, "melody": "", "next_melodies": [],
           "next_known": False, "not_found": [], "name": "Ночной город", "style": "synthwave", "source": "generated",
           "bpm": 130.0, "library_id": "a1b2c3", "title": "Неоновая ночь · трек 3 из 10"}


def test_4060_set_from_generation_answers_with_title_and_style_not_with_the_track_number():
    say = MediaRouter().route("что сейчас играет", _media(GEN_SET)).say_ok
    assert say == "Сейчас играет «Ночной город», синтвейв."      # темп файла неизвестен: 130 - темп деки, не называем
    assert "3 из 10" not in say and "a1b2c3" not in say


def test_4060_synth_set_track_without_melody_says_style_tempo_and_its_own_motif():
    facts = {"enabled": True, "theme": "космос", "track_no": 2, "tracks": 5, "melody": "", "style": "club", "bpm": 130}
    assert MediaRouter().route("что играет", _media(facts)).say_ok == (
        "Сейчас играет трек в стиле клуб, 130 ударов в минуту — свой мотив диджея, без мелодии из библиотеки.")


def test_4060_single_generated_cover_track_is_told_by_title_original_and_style():
    dj = {"enabled": False, "name": "Тихий вечер", "source": "generated", "style": "rock", "cover": "Группа крови",
          "bpm": 60.0, "started_at": 40.0, "track_id": "gen_abc_1", "library_id": "a1b2c3"}
    say = MediaRouter().route("что играет", _track(dj)).say_ok
    assert say == ("Сейчас играет «Тихий вечер» — сгенерированный трек из библиотеки, кавер на «Группа крови», рок. "
                   "Играет уже 1 минуту и повторяется, пока не скажешь «стоп».")
    assert "gen_abc_1" not in say and "60" not in say
