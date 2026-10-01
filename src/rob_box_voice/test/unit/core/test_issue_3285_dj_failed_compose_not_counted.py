"""test_issue_3285_dj_failed_compose_not_counted.py — упавший compose_music не трек сета.

Issue #3285: ``note_turn_tools`` считал трек по ИМЕНАМ из ``tools_called``.
Живой лог 01.10 11:12:03: ``compose_music(name='Калинка')`` → «не найдена в
библиотеке» → «🎧 DJ трек #1 запущен (переход #0)», ретраи Bug C трек не
запустили. 30.09 23:49:45: compose_music → таймаут → трек #3, Bug B-ретрай того
же перехода #2 → трек #4. Нода теперь передаёт ``result.succeeded_tools``.
"""

from __future__ import annotations

import json
import logging

from rob_box_voice.core.dj_mode import DJHook, DJModeController

MUSIC = ("compose_music", "execute_music_code")


def _controller() -> DJModeController:
    ctrl = DJModeController(
        hook=DJHook(
            dispatch=lambda *a, **k: None,
            is_active=lambda: False,
            is_dialogue_active=lambda: False,
        ),
        logger=logging.getLogger("t"),
    )
    ctrl.handle_message(json.dumps({"enabled": True, "plan": "Трек 1: a\nТрек 2: b"}))
    return ctrl


def test_failed_compose_in_set_turn_is_not_counted() -> None:
    """«Калинка»: set_dj_mode ок, compose_music упал — трек #1 не засчитан."""
    ctrl = _controller()

    counted = ctrl.note_turn_tools(
        ["compose_music", "set_dj_mode"], MUSIC, succeeded_tools=["set_dj_mode"],
    )

    assert not counted
    assert ctrl.state.tracks_started == 0


def test_failed_then_retried_transition_counts_once() -> None:
    """30.09 23:49: таймаут + Bug B-ретрай одного перехода — один трек, не два."""
    ctrl = _controller()

    ctrl.note_turn_tools(
        ["speak_text", "compose_music"], MUSIC, is_dj_auto=True,
        succeeded_tools=["speak_text"],
    )
    ctrl.note_turn_tools(
        ["compose_music"], MUSIC, turn_text="[DJ_AUTO переход #2] ...",
        succeeded_tools=["compose_music"],
    )

    assert ctrl.state.tracks_started == 1


def test_validation_error_then_success_in_same_turn_counts() -> None:
    """#3004: упал на валидации и тут же успешно перезван — трек запущен."""
    ctrl = _controller()

    assert ctrl.note_turn_tools(
        ["compose_music"], MUSIC, is_dj_auto=True, succeeded_tools=["compose_music"],
    )
    assert ctrl.state.tracks_started == 1


def test_failed_guest_request_does_not_hold_transition() -> None:
    """Упавший заказ гостя ничего не играет — переход не откладываем."""
    ctrl = _controller()
    before = ctrl.state.next_transition_at

    ctrl.note_turn_tools(["compose_music"], MUSIC, succeeded_tools=[])

    assert ctrl.state.next_transition_at == before


def test_unknown_success_keeps_name_based_count() -> None:
    """``succeeded_tools=None`` (роутер #3176, старые стабы) — как раньше."""
    ctrl = _controller()

    assert ctrl.note_turn_tools(["compose_music"], MUSIC, is_dj_auto=True)
    assert ctrl.state.tracks_started == 1
