"""quest_node -> консоль ТАРС 1 (issue #3253, Ш4): хуки в двух обработчиках.

Фикстура ``quest_node_mod`` (conftest.py) ставит stubs ROS-msg, поэтому тест
работает и на dev-env, и в Docker.
"""

from __future__ import annotations

import json
from unittest.mock import MagicMock


def _host() -> MagicMock:
    host = MagicMock()
    host.ws_server = MagicMock()
    host.get_logger = MagicMock(return_value=MagicMock())
    return host


def _msg(payload: dict) -> MagicMock:
    m = MagicMock()
    m.data = json.dumps(payload, ensure_ascii=False)
    return m


def _console(host: MagicMock) -> list[dict]:
    return [
        c.args[0]
        for c in host.ws_server.broadcast_json_event.call_args_list
        if c.args[0].get("type") == "tars_console"
    ]


def test_stt_result_sends_operator_line_after_accepted(quest_node_mod):
    host = _host()
    quest_node_mod.QuestNode._on_avatar_stt_result(
        host, _msg({"client_id": "s1", "text": "включи свет", "ts_ms": 77})
    )
    types = [c.args[0]["type"] for c in host.ws_server.broadcast_json_event.call_args_list]
    assert types == ["tars_state", "tars_console"]
    ev = _console(host)[0]
    assert ev["kind"] == "operator" and ev["text"] == "включи свет"
    assert ev["request_id"] == "s1:77"


def test_stt_result_empty_text_sends_no_console_line(quest_node_mod):
    host = _host()
    quest_node_mod.QuestNode._on_avatar_stt_result(host, _msg({"client_id": "s", "text": ""}))
    assert _console(host) == []


def test_stt_result_console_failure_does_not_raise(quest_node_mod):
    host = _host()
    host.ws_server.broadcast_json_event.side_effect = RuntimeError("ws closed")
    quest_node_mod.QuestNode._on_avatar_stt_result(host, _msg({"client_id": "s", "text": "привет"}))


def test_command_result_sends_tool_and_error_events(quest_node_mod):
    host = _host()
    quest_node_mod.QuestNode._on_avatar_command_result(
        host,
        _msg(
            {
                "request_id": "r1",
                "ok": False,
                "summary": "x",
                "error": "boom sk-1",
                "tool_calls": [{"name": "play_music", "args": {"k": "secret"}}],
            }
        ),
    )
    texts = [e["text"] for e in _console(host)]
    assert texts == ["tool: play_music", "ошибка хода"]
    assert "secret" not in json.dumps(_console(host)) and "sk-1" not in json.dumps(_console(host))


def test_command_result_ok_without_tools_sends_nothing(quest_node_mod):
    host = _host()
    quest_node_mod.QuestNode._on_avatar_command_result(
        host, _msg({"request_id": "r", "ok": True, "summary": "ок", "tool_calls": []})
    )
    assert _console(host) == []
