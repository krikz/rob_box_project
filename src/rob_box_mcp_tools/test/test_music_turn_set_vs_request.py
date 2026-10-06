"""06.10 (set98207, 14:50 UTC): в одном ходе LLM ``dj_set`` → ``request_music('1812 Overture')`` → заказ погасил
только что запущенный сет (``stop reason=request_music``), сам не заиграл — тишина.

Решает владелец деки (ADR-0148): ``request_music`` не снимает сет, заигравший в этом же ходе (``turn_id`` —
скрытый аргумент хода, ``llm_adapter.TURN_CONTEXT_ARGS``; его проброс — ``test_register_speaker_ros_mcp_utterance``),
и снимает идущий сет только собранным заказом.
Настоящие ``PlayerOwner`` + ``RenardoAdapter`` + ``compose``/``render`` на симуляторе клока (``test_engine_session``).
"""

import json
from unittest.mock import patch

import pytest

from rob_box_mcp_tools.engine.classic import ClassicPick
from rob_box_mcp_tools.engine.tools_v2 import SET_PLAYING
from rob_box_voice.core.media_phrases import SET_PLAYING_REASON
from rob_box_voice.core.music_player_state import MusicEvent

from .test_engine_request_music import _tools
from .test_engine_session import _rig, _started

pytestmark = pytest.mark.unit

TURN = "turn-a"


def _set_running(rig, turn=TURN, confirm=None):
    dj, req = _tools(rig, confirm=confirm)
    started = dj.execute(action="start", theme="космос", turn_id=turn)
    return dj, req, started


def test_request_in_the_same_turn_keeps_the_set_and_says_set_playing():
    rig = _rig()
    dj, req, started = _set_running(rig)
    assert started.success
    set_track = started.data["track_id"]

    result = req.execute(intent="melody", text="сыграй 1812 Overture", turn_id=TURN)

    assert result.success is False and result.data["reason"] == SET_PLAYING
    assert result.data["set_id"] == started.data["set_id"]
    assert dj.running and dj._session is not None  # сет не закрыт
    assert rig.owner.is_playing()  # дека не снята
    assert not [s for s in rig.states if json.loads(s)["state"] == "idle"]  # стопа деки не было
    assert not [e for e in rig.events if e["event"] == "rejected"]
    rig.clock.run_until(rig.clock.beat + 2)
    assert [e["track_id"] for e in _started(rig)] == [set_track]  # звучит трек 1 сета, не заказ


def test_request_in_the_next_turn_replaces_the_set():
    rig = _rig()
    dj, req, _started = _set_running(rig)

    result = req.execute(intent="track", text="поставь клубный трек", turn_id="turn-b")

    assert result.success and result.data["track_id"].startswith("req")
    assert dj._session is None and not dj.running


def test_request_without_turn_id_keeps_the_old_contract():
    """Роутер медиакоманд и харнессы зовут без хода: заказ, как раньше, сменяет сет."""
    rig = _rig()
    dj, req, _started = _set_running(rig)

    assert req.execute(intent="track", text="поставь клубный трек").success
    assert dj._session is None


def test_set_that_did_not_start_is_not_a_reason_to_refuse():
    """14:38 (set97492): ``dj_set`` → ``not_started``. Сет не заиграл — заказ того же хода не отказывается."""
    rig = _rig()
    dj, req, started = _set_running(rig, confirm=lambda tid: None)
    assert started.success is False and started.data["reason"] == "not_started"
    assert dj.started_in_turn(TURN) is None

    later = req.execute(intent="track", text="поставь клубный трек", turn_id=TURN)
    assert later.data.get("reason") != SET_PLAYING


def test_melody_not_found_does_not_stop_the_running_set():
    """Не нашлось — снимать идущий сет нечем (раньше ``close_set`` шёл ДО поиска → тишина)."""
    rig = _rig()
    dj, _req, _started = _set_running(rig)
    _dj2, req = _tools(rig, classic=lambda query, seed: ClassicPick(query=query, found=False, reason="нет в базе"))
    req._dj = dj

    result = req.execute(intent="melody", text="сыграй 1812 Overture", turn_id="turn-b")

    assert result.success is False and result.data["reason"] == "not_found"
    assert dj.running and rig.owner.is_playing()


def test_compose_error_does_not_stop_the_running_set():
    rig = _rig()
    dj, req, _started = _set_running(rig)

    with patch("rob_box_mcp_tools.engine.tools_v2.compose", side_effect=RuntimeError("boom")):
        result = req.execute(intent="track", text="поставь клубный трек", turn_id="turn-b")

    assert result.success is False and result.data["reason"] == "compose_error"
    assert dj.running and rig.owner.is_playing()


def test_set_stopped_in_the_turn_frees_the_deck_for_the_request():
    """«выключи сет и поставь трек» в одном ходе: сета нет — заказ играет."""
    rig = _rig()
    dj, req, _started = _set_running(rig)
    with patch("rob_box_mcp_tools.engine.renardo_adapter.time.sleep"):
        assert dj.execute(action="stop", turn_id=TURN).success

    assert req.execute(intent="track", text="поставь клубный трек", turn_id=TURN).success


def test_refusal_code_is_the_one_the_voice_side_reads():
    assert SET_PLAYING == SET_PLAYING_REASON


def test_started_event_of_the_set_marks_its_turn():
    """``started_in_turn`` — только для сета, который реально заиграл (``ok`` по ``started``, A14)."""
    rig = _rig()
    dj, _req, started = _set_running(rig, confirm=lambda tid: MusicEvent("started", tid, fields={}))
    assert started.success
    assert dj.started_in_turn(TURN) == started.data["set_id"]
    assert dj.started_in_turn("turn-b") is None and dj.started_in_turn(None) is None
