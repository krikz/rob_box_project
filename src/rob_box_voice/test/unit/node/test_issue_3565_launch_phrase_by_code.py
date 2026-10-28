"""Issue #3565 / #3461 — фраза запуска музыки не доходит до человека без ``started``.

Просили пьесу по имени (``PLAY_NAMED``), а в ходе ничего не запустилось — фразу говорит код
(:data:`NOT_PLAYED_TEXT`), а не текст модели. Здесь — правила ``core.music_turn``: что считается просьбой
пьесы и какая фраза хода положена.

PR-15 (#3711, часть 15p): проводка этих правил в ноде диалога v1 удалена вместе с ней.
"""

from __future__ import annotations

import pytest

from rob_box_voice.core.media_phrases import NOT_PLAYED_TEXT
from rob_box_voice.core.music_turn import MusicTurn, piece_requested, turn_reply


@pytest.mark.parametrize("user", ["сыграй Баха на трамбоне", "сыграй Токкату ре минор", "сыграй К Элизе"])
def test_named_piece_request_is_a_piece_request(user):
    assert piece_requested(user)


@pytest.mark.parametrize("user", ["ну что, будет Бах?", "стоп музыка", "как дела", "включи диджей сет на тему космос"])
def test_other_replies_are_not_piece_requests(user):
    assert not piece_requested(user)


def test_turn_reply_piece_without_launch_is_code_phrase():
    assert turn_reply(MusicTurn(), requested=False, voiced=0, piece=True) == NOT_PLAYED_TEXT


def test_turn_reply_piece_launched_by_another_music_tool_is_left_alone():
    assert turn_reply(MusicTurn(), requested=False, voiced=0, piece=True, other_launch=True) is None
