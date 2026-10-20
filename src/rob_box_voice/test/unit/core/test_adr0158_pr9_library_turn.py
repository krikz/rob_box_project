"""ADR-0158 PR-9 — итог ``music_library`` (список, играет, оценка, удаление) говорит код, а не модель.

Живой прогон 14.10 (Vision Pi): на «что у тебя есть сгенерированного» тул вернул «В библиотеке 23 трека…», а модель
сказала своё через ``speak_text`` — «двадцать треков… «Полетели валькирий»» (названий в кэше нет). ADR-0148: фразу об
итоге строит код из результата тула; ``speak_text`` модели в таком ходе не исполняется.
"""

from __future__ import annotations

import json

from rob_box_voice.core.media_phrases import (
    LIBRARY_FAIL_TEXT,
    LIBRARY_NO_TRACK_TEXT,
    library_list_text,
    library_rate_text,
)
from rob_box_voice.core.music_turn import SPEECH_LIBRARY_MESSAGE, MusicTurn, speech_refusal_content, turn_reply

LISTED = "В библиотеке 23 трека, например: «Утро», «Дождь».\n{'tracks': [], 'total': 23}"


def _turn(action, content, is_error=False) -> MusicTurn:
    turn = MusicTurn()
    turn.record("music_library", {"action": action}, is_error=is_error, content=content)
    return turn


def test_list_phrase_is_the_first_line_of_the_result_and_blocks_model_speech():
    turn = _turn("list", LISTED)
    assert turn.phrase() == "В библиотеке 23 трека, например: «Утро», «Дождь»."
    assert not turn.speech_allowed()
    assert turn_reply(turn, requested=False, voiced=0) == turn.phrase()


def test_rate_play_delete_update_are_voiced_by_code_too():
    for action in ("rate", "play", "delete", "update"):
        assert not _turn(action, "Готово.\n{}").speech_allowed(), action


def test_search_and_info_are_left_to_the_model():
    for action in ("search", "info"):
        turn = _turn(action, "Найдено 1.\n{}")
        assert turn.speech_allowed() and turn.phrase() is None
        assert turn_reply(turn, requested=False, voiced=0) is None


def test_failure_gets_an_honest_phrase_of_code_not_the_internal_error():
    assert _turn("play", "трек не заиграл: missing_sample", is_error=True).phrase() == LIBRARY_FAIL_TEXT
    assert _turn("rate", LIBRARY_NO_TRACK_TEXT, is_error=True).phrase() == LIBRARY_NO_TRACK_TEXT


def test_turn_boundary_forgets_the_library_phrase():
    turn = _turn("list", LISTED)
    turn.reset()
    assert turn.phrase() is None and turn.speech_allowed()


def test_a_real_launch_in_the_same_turn_keeps_the_launch_rules():
    turn = _turn("list", LISTED)
    turn.record("request_music", {"intent": "track"}, is_error=False, content="{'ok': True, 'title': 'x'}")
    assert not turn.library_answered()


def test_refusal_message_names_the_library():
    body = json.loads(speech_refusal_content(library=True))
    assert body["message"] == SPEECH_LIBRARY_MESSAGE and "music_library" in body["message"]


def test_list_phrase_names_distinct_titles_and_the_whole_library_count():
    text = library_list_text(["Рок", "Рок", "Рок", "Гранж"], 23)
    assert text == "В библиотеке 23 трека, например: «Рок», «Гранж»."


def test_rate_phrases():
    assert library_rate_text("Пыль", True) == "Запомнил: «Пыль» нравится."
    assert library_rate_text("", False) == "Запомнил: этот трек не нравится, больше не включу."
