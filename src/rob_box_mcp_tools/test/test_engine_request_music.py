"""ADR-0149 PR-6 — ``request_music`` v2, ``ok`` только по ``started``, каталог тулов при v1/v2 (эпик #3312).

Настоящие ``PlayerOwner`` + ``RenardoAdapter`` + ``compose``/``render`` на симуляторе клока из
``test_engine_session``; события плеера — тот же ``MusicEventLog``, что слушает ``dialogue_node``.
"""

from unittest.mock import patch

import pytest

from rob_box_core.tool_catalog import llm_visible_tools, operator_visible_tools
from rob_box_harness.core.tool_registry import ToolRegistry
from rob_box_mcp_tools.engine.tools_v2 import DjSetTool, RequestMusicTool
from rob_box_music import knowledge as kn
from rob_box_voice.core.music_player_state import MusicEvent, MusicEventLog

from .test_engine_session import _rig, _started

pytestmark = pytest.mark.unit

OLD_LLM_MUSIC_TOOLS = {"compose_music", "preview_arrangement", "execute_music_code", "set_dj_mode"}
V2_TOOLS = {"dj_set", "request_music"}


def _tools(rig, confirm=None, classic=None):
    dj = DjSetTool(None, rig.owner, melodies=lambda ids: {}, seed=lambda: 4242, confirm=confirm)
    req = RequestMusicTool(None, rig.owner, dj, melodies=lambda ids: {}, seed=lambda: 777, confirm=confirm,
                           classic=classic)
    return dj, req


def test_request_music_plays_one_club_track_on_the_v2_deck():
    rig = _rig()
    _dj, req = _tools(rig)
    result = req.execute(intent="track", text="поставь клубный трек")
    assert result.success and result.data["track_id"].startswith("req00777:01:A:")
    lo, hi = kn.GENRE_WINDOWS["club"].bpm
    assert lo <= result.data["bpm"] <= hi and result.data["energy"] == kn.ENERGY_WAVE[0]
    rig.clock.run_until(rig.clock.beat + 2)
    started = _started(rig)
    assert [e["track_id"] for e in started] == [result.data["track_id"]] and started[0]["phase_in_form"] == 0.0
    assert rig.states[-1] and '"title"' in rig.states[-1]  # имя трека — в снимке для <music_state>


def test_mood_picks_the_energy_of_the_track():
    rig = _rig()
    _dj, req = _tools(rig)
    result = req.execute(intent="track", text="что-нибудь эпичное", mood="epic")
    assert result.success and result.data["energy"] == kn.MOOD_ENERGY["epic"]
    assert result.data["track_id"].split(":")[1] == f"{kn.ENERGY_WAVE.index(5) + 1:02d}"


def test_mood_energy_is_levels_of_the_one_energy_table():
    """``MOOD_ENERGY`` — только имена настроений; уровни — из ``ENERGY_LEVELS``, трек — по ``ENERGY_WAVE``."""
    assert set(kn.MOOD_ENERGY.values()) <= set(kn.ENERGY_LEVELS)
    assert set(kn.MOOD_ENERGY.values()) <= set(kn.ENERGY_WAVE)  # у каждого настроения есть трек волны


@pytest.mark.parametrize("event,ok,reason", [
    (MusicEvent("started", "x", fields={}), True, None),
    (MusicEvent("rejected", "x", fields={"reason": "server_fail", "detail": "/s_new not found"}), False, "server_fail"),
    (None, False, "not_started"),
])
def test_ok_only_after_started_of_this_track(event, ok, reason):
    rig = _rig()
    asked = []
    _dj, req = _tools(rig, confirm=lambda tid: asked.append(tid) or event)
    result = req.execute(intent="track", text="поставь музыку")
    assert asked == [result.data["track_id"]]  # ждали событие именно своего трека
    assert result.success is ok and result.data.get("reason") == reason
    assert result.data.get("started", False) is ok


def test_rejected_before_exec_is_not_ok_and_nobody_waits():
    rig = _rig()
    asked = []
    _dj, req = _tools(rig, confirm=lambda tid: asked.append(tid))
    with patch.object(rig.adapter, "check", return_value=("missing_synth", "sinepad")):  # I15
        result = req.execute(intent="track", text="поставь музыку")
    assert result.success is False and asked == []
    assert [e["event"] for e in rig.events] == ["rejected"]


def test_dj_set_waits_for_started_too():
    rig = _rig()
    log = MusicEventLog()
    dj, _req = _tools(rig, confirm=lambda tid: log.wait(tid, 0.05))
    result = dj.execute(action="start", theme="космос")
    assert result.success is False and result.data["reason"] == "not_started"  # клок не дошёл до такта
    rig.clock.run_until(rig.clock.beat + 2)
    log.observe(MusicEvent("started", _started(rig)[0]["track_id"]))
    assert log.wait(_started(rig)[0]["track_id"], 0.0).event == "started"


def test_request_closes_a_running_set_and_dj_stop_stops_a_single_track():
    rig = _rig()
    dj, req = _tools(rig)
    assert dj.execute(action="start", theme="космос").success
    assert req.execute(intent="track", text="поставь клубный трек").success
    assert dj._session is None  # один владелец деки: сет закрыт, играет заказ
    with patch("rob_box_mcp_tools.engine.renardo_adapter.time.sleep"):
        stopped = dj.execute(action="stop")
    assert stopped.success and stopped.data["was_playing"] is True
    assert '"state": "idle"' in rig.states[-1]


def test_old_music_tools_are_gone_and_v2_tools_are_the_music_path():
    """PR-13b ADR-0149 §8.2: ``compose_music``/``preview_arrangement``/``set_dj_mode`` удалены из каталога;
    ``execute_music_code`` остался только для харнессов (``llm_visible=False``)."""
    from rob_box_core.tool_catalog import TOOL_CATALOG

    names = {e.name for e in TOOL_CATALOG}
    assert not ({"compose_music", "preview_arrangement", "set_dj_mode", "set_vibe_preset",
                 "save_arrangement_preset"} & names)
    harness_only = next(e for e in TOOL_CATALOG if e.name == "execute_music_code")
    assert harness_only.llm_visible is False and not harness_only.operator_visible
    visible = {e.name for e in llm_visible_tools()}
    assert V2_TOOLS <= visible and not (OLD_LLM_MUSIC_TOOLS & visible)
    # PR-15: фильтра по движку нет — реестр оператора (ТАРС) тоже предъявляет тулы v2
    assert V2_TOOLS <= {e.name for e in operator_visible_tools()}


def test_dialogue_registry_offers_tools_of_its_engine():
    v2 = {spec.name for spec in ToolRegistry().list_tools()}
    assert {"dj_set", "request_music"} <= v2 and not (OLD_LLM_MUSIC_TOOLS & v2)
    narrowed = {spec.name for spec in ToolRegistry().list_tools(skills=("dj",))}
    assert {"dj_set", "request_music"} <= narrowed


def test_server_tees_player_events_to_the_in_process_log(monkeypatch):
    from .test_mcp_server import _load_mcp_server_module

    module = _load_mcp_server_module(monkeypatch)
    published, log = [], MusicEventLog()
    publish = module._teed_events(published.append, log)
    publish('{"event": "started", "track_id": "t1", "ts": 1.0}')
    assert published and log.wait("t1", 0.0).event == "started"
    assert module.V2_STARTED_WAIT_S <= 6.0  # A2 p100
