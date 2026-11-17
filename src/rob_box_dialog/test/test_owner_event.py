"""Исход тула -> событие владельца (``execute.owner_event``; перенесено из ``command_turn`` v1, #4354 A3b).

Результат тула — утиный (``data``, ``is_error``, ``content``, ``message``), как ``rob_box_llm.provider.ToolResult``.
"""

from __future__ import annotations

from types import SimpleNamespace
from typing import Any, Mapping, Optional

from rob_box_dialog import knowledge as kn
from rob_box_dialog.execute import ActionRequest, owner_event

HAPPY = {"sound": "robot_happy"}


def _request(tool: str, args: Optional[Mapping[str, Any]] = None) -> ActionRequest:
    return ActionRequest("t/0", 0, "personality", tool, dict(args or {}), kn.TOOL_CLASSES[tool], 3.0)


def _tool(content: str, is_error: bool = False, data: Optional[dict] = None, message: Optional[str] = None):
    return SimpleNamespace(content=content, is_error=is_error, data=data, message=message)


def test_owner_event_takes_status_and_reason_from_result_data() -> None:
    busy = _tool("Звук не сыграл", is_error=True, data={"status": "rejected", "reason": "busy"})
    event = owner_event(_request("play_sound", HAPPY), busy)
    assert (event.ref, event.status, event.reason) == ("t/0", "rejected", "busy")
    assert event.data["sound"] == "robot_happy"  # аргумент вызова — слот фразы


def test_owner_event_keeps_mcp_refusal_text_as_reason() -> None:
    refused = _tool("sender 'x' не в списке разрешённых", is_error=True)
    assert owner_event(_request("get_current_time"), refused).reason == "sender 'x' не в списке разрешённых"


def test_plan_outcome_and_missing_status_follows_is_error() -> None:
    waited = _tool("{}", is_error=True, data={"status": "timeout", "reason": "plan_wait"})
    event = owner_event(_request("request_music"), waited)
    assert (event.status, event.reason) == ("timeout", "plan_wait")
    assert owner_event(_request("memory_save"), _tool("ok")).status == "done"
    assert owner_event(_request("memory_save"), _tool("нет", is_error=True)).status == "rejected"
    applied = _tool("ok", data={"status": "applied"})  # не статус исполнителя — решает is_error
    assert owner_event(_request("set_voice", {"voice": "alena"}), applied).status == "done"


def test_owner_event_of_a_done_query_carries_the_tool_phrase() -> None:
    """#4157 К2: фраза тула (``message``) у выполненного ``query`` — в данных события; у действий и отказов нет."""
    state = _tool("Сейчас ничего не играет.", data={"playing": False}, message="Сейчас ничего не играет.")
    assert owner_event(_request("get_music_state"), state).data[kn.TOOL_MESSAGE_FIELD] == "Сейчас ничего не играет."
    refused = _tool("нет", is_error=True, message="нет")
    assert kn.TOOL_MESSAGE_FIELD not in owner_event(_request("get_music_state"), refused).data
    sound = _tool("ok", data={"status": "done"}, message="Сыграл.")
    assert kn.TOOL_MESSAGE_FIELD not in owner_event(_request("play_sound", HAPPY), sound).data


def test_owner_event_carries_event_ref_only_when_it_is_a_string() -> None:
    pending = _tool("ok", data={"status": "pending", "event_ref": "sp-1"})
    assert owner_event(_request("register_speaker", {"name": "Саша"}), pending).event_ref == "sp-1"
    assert owner_event(_request("memory_save"), _tool("ok", data={"event_ref": 5})).event_ref is None
