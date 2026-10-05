"""Issue #3400 — «давай тему X» посреди DJ-сета: мимо базы мелодий → dj_set(theme), без LLM.

Харнесс — как в ``test_issue_3176_play_named_node`` (реальные ``_on_stt`` →
роутер → ``_execute_play_named``; мок — исполнитель тулов, LLM-вход, TTS).
"""

from __future__ import annotations

import pytest

from rob_box_voice.core import media_plan_run
from rob_box_voice.core.media_command_grammar import MediaCommand, MediaIntent
from rob_box_voice.core.media_router import (
    DJ_FAIL_TEXT,
    MediaPlan,
    MediaState,
    dj_theme_switch_plan,
)
from rob_box_voice.core.music_player_state import MusicEventLog, build_music_event_payload

from . import test_issue_3176_play_named_node as _base

_make_node, _MelodyExecutor, _stt = _base._make_node, _base._MelodyExecutor, _base._stt
run_plans = _base.run_plans  # фикстура: корутины run_coroutine_threadsafe — сразу

_TRACK = "set1:01:A:ab"


class _SetExecutor(_MelodyExecutor):
    def __init__(self, *, dj_ok=True, **kw):
        super().__init__(**kw)
        self._dj_ok = dj_ok

    async def execute(self, call):
        if call.name != "dj_set":
            return await super().execute(call)
        self.calls.append((call.name, dict(call.arguments)))
        if self._dj_ok:
            return self._ok(call, f"Сет начат\n{{'ok': True, 'track_id': '{_TRACK}'}}")
        return self._err(call, "сет не начался")


def _set_node(*, persona="Снупдог", started=True, **exec_kw):
    n = _make_node(playing=True, dj=True, track="club #3", state_name="DIALOGUE")
    n._music_player_state.dj_info["persona"] = persona
    n._scheduler_executor = _SetExecutor(**exec_kw)
    n._music_events = MusicEventLog()
    if started:
        n._music_events.observe_json(
            build_music_event_payload("started", _TRACK, ts=1.0, phase_in_form=0.0)
        )
    return n


@pytest.fixture(autouse=True)
def _fast(monkeypatch):
    monkeypatch.setattr(media_plan_run, "STARTED_WAIT_S", 0.1)


def test_theme_miss_during_set_starts_set_with_theme_and_persona(run_plans):
    n = _set_node()
    _stt(n, "Робот, давай тему терминатора")
    run_plans()
    assert n._scheduler_executor.calls == [
        ("lookup_melody", {"name": "терминатора"}),
        ("dj_set", {"action": "start", "theme": "терминатора", "persona": "Снупдог"}),
    ]
    n._dispatch_turn.assert_not_called()  # LLM не на пути
    n._speak_direct.assert_called_once_with("Я Снупдог, включаю сет. Тема — терминатора.")
    n._cancel_run.assert_called_once()
    assert n._llm_skipped_counter["media_command"] == 1


def test_theme_during_set_rejected_is_honest_and_not_llm(run_plans):
    n = _set_node(dj_ok=False, started=False)
    _stt(n, "Робот, давай тему терминатора")
    run_plans()
    n._dispatch_turn.assert_not_called()
    n._speak_direct.assert_called_once_with(DJ_FAIL_TEXT)


def test_melody_found_during_set_still_plays_over_set(run_plans):
    n = _set_node()
    _stt(n, "Робот, поставь к элизе")
    run_plans()
    assert [c[0] for c in n._scheduler_executor.calls] == ["lookup_melody", "request_music"]


def test_miss_without_set_still_goes_to_llm(run_plans):
    n = _make_node(playing=True, track="Still Dre", state_name="DIALOGUE")
    _stt(n, "Робот, давай тему терминатора")
    run_plans()
    assert n._scheduler_executor.calls == [("lookup_melody", {"name": "терминатора"})]
    n._dispatch_turn.assert_called_once()
    n._speak_direct.assert_not_called()


def test_switch_plan_is_none_without_set_or_name():
    command = MediaCommand(intent=MediaIntent.PLAY_NAMED)
    plan = MediaPlan(command=command, play_name="терминатора")
    assert dj_theme_switch_plan(plan, MediaState(music_playing=True)) is None
    assert dj_theme_switch_plan(MediaPlan(command=command), MediaState(dj_enabled=True)) is None
