"""Issue #3217 — закрытая DJ-команда медиароутера снимает забор #2835.

После «забудь все» забор ``SessionEpoch`` режет ``/voice/dj_mode
enabled=true`` до первого хода LLM. Медиароутер исполняет «ты диджей X»
кодом, без хода LLM, — его собственный ``set_dj_mode`` отбрасывался, и
DJ-контроллер оставался OFF (вечное club-превью без переходов).
"""

from __future__ import annotations

import json

from .test_issue_3134_media_router_node import (  # noqa: F401
    _make_node, _stt, run_plans,
)

_DJ_ON = json.dumps({"enabled": True, "persona": "диджей Снупдог"})


def _node_after_new_session():
    n = _make_node()
    gate = n._session_epoch_gate()
    gate.advance()  # «забудь все»: поколение +1, забор поднят
    return n, gate


def test_router_dj_command_lifts_fence(run_plans):  # noqa: F811
    n, gate = _node_after_new_session()
    assert not gate.admits_dj_payload(_DJ_ON)  # забор стоит
    _stt(n, "Робот, ты диджей Снупдог")
    run_plans()
    n._dispatch_turn.assert_not_called()  # LLM не вызывался
    assert [c[0] for c in n._scheduler_executor.calls] == [
        "compose_music", "set_dj_mode",
    ]
    # Забор снят: /voice/dj_mode enabled=true доходит до DJ-контроллера.
    n._last_stt_text = ""
    n._on_dj_mode_msg(_DJ_ON)
    n._dj.handle_message.assert_called_once()
    assert n._dj.handle_message.call_args.args[0] == _DJ_ON


def test_late_enable_without_new_session_turn_still_fenced():
    n, gate = _node_after_new_session()
    n._on_dj_mode_msg(_DJ_ON)
    n._dj.handle_message.assert_not_called()


def test_stale_generation_command_does_not_lift_fence():
    n, gate = _node_after_new_session()
    gate.note_turn_started(gate.current - 1)  # ход старой сессии
    n._on_dj_mode_msg(_DJ_ON)
    n._dj.handle_message.assert_not_called()


def test_non_dj_media_command_keeps_fence(run_plans):  # noqa: F811
    n, gate = _node_after_new_session()
    _stt(n, "Робот, стоп")
    run_plans()
    n._on_dj_mode_msg(_DJ_ON)
    n._dj.handle_message.assert_not_called()
