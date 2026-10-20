"""#3599 (ADR-0157 R1/R5): «продолжай играть Вагнера» — следующая пьеса автора, не повтор; всё сыграно — фраза кода.

Поведение ``request_music`` на настоящем владельце плеера (``_rig``) с подставными каталогами, как в
``test_piece_fail_open``; «ещё» читается из реплики хода (``heard_text``), текст заказа — слова автора.
"""

from __future__ import annotations

import pytest

from rob_box_mcp_tools.engine.tools_v2 import NO_MORE, asks_more
from rob_box_music import knowledge as kn
from rob_box_music.diversity import MusicHistory
from rob_box_voice.core.media_phrases import NO_MORE_REASON, no_more_phrase

from .test_engine_session import _rig
from .test_piece_fail_open import BACH, _tools

pytestmark = pytest.mark.unit


def _play(history, text, heard):
    req = _tools(_rig(), hooks=tuple(BACH), table=BACH, history=history)
    return req.execute(intent="melody", text=text, heard_text=heard)


def test_codes_are_one_value_on_both_sides():
    assert NO_MORE == NO_MORE_REASON


@pytest.mark.parametrize("heard,more", [("продолжай играть баха", True), ("ещё баха", True), ("давай ещё баха", True),
                                        ("ещё раз баха", False), ("сыграй баха", False), ("продолжай сэт", False),
                                        (None, False)])
def test_asks_more_is_the_grammar_verdict(heard, more):
    assert asks_more(heard) is more


def test_more_plays_pieces_that_were_not_played_and_then_says_there_is_no_more():
    history = MusicHistory(":memory:")
    first = _play(history, "сыграй баха", "сыграй баха").data["melody_id"]
    played = [first]
    for _ in range(2):
        data = _play(history, "баха", "продолжай играть баха").data
        assert data["ok"] and data["melody_id"] not in played, (data["melody_id"], played)
        played.append(data["melody_id"])
    assert sorted(played) == sorted(BACH)  # все три пьесы сыграны по разу
    result = _play(history, "баха", "продолжай играть баха")
    assert not result.success and result.data["reason"] == NO_MORE
    phrase = no_more_phrase(result.error)
    assert phrase == result.data["message"] and "«баха»" in phrase and "всё уже играло" in phrase
    assert "Могу повторить «" in phrase  # что повторить - название из каталога, не выдумка
    rows = history.recent(10, kit=kn.CLASSIC_HISTORY_KIT)
    assert len(rows) == 3  # отказ ничего не заиграл и в историю не лёг


def test_plain_repeat_request_still_replays_when_everything_was_played():
    history = MusicHistory(":memory:")
    for _ in range(3):
        _play(history, "сыграй баха", "сыграй баха")
    assert _play(history, "сыграй баха", "сыграй баха").data["ok"]
    assert _play(history, "баха", "ещё раз баха").data["ok"]  # «ещё раз» - повтор, не «следующая пьеса»
