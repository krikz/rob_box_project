"""Тулы пульта Telegram по лицам: ``people_list``, ``person_photo``, ``name_person`` (issue #4469, ADR-0148).

Тул только зовёт владельца лиц и переводит его ответ в результат: статус и причина - от владельца, фраза «Готово» -
код из его данных. Видит эти тулы только агент чата Telegram. Сеть и ROS не нужны: вызов сервиса подставной.
"""

from __future__ import annotations

import sys
from unittest.mock import MagicMock, Mock

import pytest

try:
    import rclpy  # noqa: F401
except ImportError:
    for _mod in ("rob_box_mcp_tools.tools.navigation", "rob_box_mcp_tools.tools.system",
                 "rob_box_mcp_tools.tools.perception", "rob_box_mcp_tools.tools.mapping",
                 "rob_box_mcp_tools.tools.memory", "rob_box_mcp_tools.tools.music"):
        sys.modules.setdefault(_mod, Mock())

from rob_box_core.tool_catalog import (  # noqa: E402
    get_tool,
    llm_visible_tools,
    operator_visible_tools,
    telegram_visible_tools,
)
from rob_box_mcp_tools.mcp_auth import PERSONALITY_SENDER, TELEGRAM_SENDER  # noqa: E402
from rob_box_mcp_tools.slice_authority import load_default_authority  # noqa: E402
from rob_box_mcp_tools.tools import people  # noqa: E402

TOOLS = ("people_list", "person_photo", "name_person")


class Owner:
    """Владелец лиц в тесте: записывает вызовы, отвечает заготовленным ``(ok, reason, data)``."""

    def __init__(self, reply):
        self.reply, self.calls = reply, []

    def __call__(self, op, flt, person_id, name):
        self.calls.append((op, flt, person_id, name))
        return self.reply


def make(cls, reply):
    owner = Owner(reply)
    return cls(MagicMock(), owner), owner


ROWS = [
    {"id": "fff371fe", "name": None, "epithet": "Бородач в синем", "last_encounter": "2026-11-05T21:17:00Z",
     "encounters": 4, "voice_linked": False, "has_snapshot": True},
    {"id": "a1b2c3d4", "name": "Кирилл", "epithet": "Очкарик", "last_encounter": None, "encounters": 9,
     "voice_linked": True, "has_snapshot": False},
]


def test_list_passes_filter_and_builds_text_from_owner_data():
    tool, owner = make(people.PeopleListTool, (True, "", {"people": ROWS, "total": 2, "shown": 2, "filter": "unnamed"}))

    result = tool.execute(filter="unnamed")

    assert owner.calls == [(people.OP_LIST, "unnamed", "", "")]
    assert result.success and result.data["status"] == "done" and result.data["people"] == ROWS
    assert "fff371fe - Бородач в синем; встреч 4; последняя 2026-11-05T21:17:00Z; голос: нет" in result.message
    assert "a1b2c3d4 - Кирилл (Очкарик); встреч 9; последняя нет данных; голос: да" in result.message


def test_list_filter_defaults_to_all_and_empty_list_is_said_plainly():
    tool, owner = make(people.PeopleListTool, (True, "", {"people": [], "total": 0, "shown": 0, "filter": "all"}))

    result = tool.execute()

    assert owner.calls[0][1] == "all" and result.message == "Лиц по этому фильтру нет."


def test_list_refusal_carries_the_owners_reason():
    tool, _ = make(people.PeopleListTool, (False, "bad_filter", {}))

    result = tool.execute(filter="x")

    assert result.success is False and result.data == {"status": "rejected", "reason": "bad_filter"}


def test_photo_returns_id_and_caption_for_the_bot():
    tool, owner = make(people.PersonPhotoTool, (True, "", {"id": "fff371fe", "size": 4, "name": None}))

    result = tool.execute(id="fff371fe")

    assert owner.calls == [(people.OP_PHOTO, "", "fff371fe", "")]
    assert result.success and result.data["photo_id"] == "fff371fe" and "photo_path" not in result.data
    assert result.data["caption"] == "fff371fe - без имени" and result.data["status"] == "done"


@pytest.mark.parametrize("reason", ["person_not_found", "exhibition_mode", "no_snapshot", "bad_id"])
def test_photo_refusal_is_honest_and_sends_no_path(reason):
    tool, _ = make(people.PersonPhotoTool, (False, reason, {}))

    result = tool.execute(id="deadbeef")

    assert result.success is False and result.data == {"status": "rejected", "reason": reason}


def test_name_person_phrase_comes_from_stored_name_not_from_the_argument():
    tool, owner = make(people.NamePersonTool,
                       (True, "", {"id": "fff371fe", "name": "Кирилл", "previous_name": None}))

    result = tool.execute(id="fff371fe", name="кирилл!!! ")

    assert owner.calls == [(people.OP_SET_NAME, "", "fff371fe", "кирилл!!! ")]
    assert result.success and result.data["stored_name"] == "Кирилл"
    assert result.data["previous_name"] is None and result.message == "Готово: лицо fff371fe теперь Кирилл."


def test_name_person_rename_mentions_previous_name():
    tool, _ = make(people.NamePersonTool, (True, "", {"id": "fff371fe", "name": "Кирилл", "previous_name": "Иван"}))

    result = tool.execute(id="fff371fe", name="Кирилл")

    assert result.data["previous_name"] == "Иван"
    assert result.message == "Готово: лицо fff371fe (было «Иван») теперь Кирилл."


def test_name_person_same_name_is_not_reported_as_a_rename():
    tool, _ = make(people.NamePersonTool, (True, "", {"id": "fff371fe", "name": "Кирилл", "previous_name": "Кирилл"}))

    assert tool.execute(id="fff371fe", name="Кирилл").data["previous_name"] is None


def test_name_person_without_a_name_read_back_is_not_success():
    tool, _ = make(people.NamePersonTool, (True, "", {"id": "fff371fe", "name": None, "previous_name": None}))

    result = tool.execute(id="fff371fe", name="Кирилл")

    assert result.success is False and result.data == {"status": "rejected", "reason": "no_evidence"}


@pytest.mark.parametrize("reason", ["bad_name", "person_not_found", "ambiguous_id", "owner_unavailable"])
def test_name_person_refusal_has_no_done_phrase(reason):
    tool, _ = make(people.NamePersonTool, (False, reason, {}))

    result = tool.execute(id="fff371fe", name="Кирилл")

    assert result.success is False and result.data["reason"] == reason and "stored_name" not in result.data


def test_ros_call_without_a_ready_service_is_owner_unavailable():
    node = MagicMock()
    node.__dict__["_people_clients"] = {"client": MagicMock(service_is_ready=lambda: False)}
    sys.modules.setdefault("rob_box_perception_msgs", MagicMock())
    sys.modules.setdefault("rob_box_perception_msgs.srv", MagicMock())

    assert people.ros_people_call(node)(people.OP_LIST, "all", "", "") == (False, "owner_unavailable", {})


def test_tools_are_visible_only_to_the_telegram_agent():
    for name in TOOLS:
        entry = get_tool(name)
        assert entry.telegram_visible and not entry.llm_visible and not entry.operator_visible
        assert name in {e.name for e in telegram_visible_tools()}
        assert name not in {e.name for e in llm_visible_tools()}
        assert name not in {e.name for e in operator_visible_tools()}


def test_slice_policy_gives_the_tools_to_the_chat_agent_only():
    authority = load_default_authority()
    for name in TOOLS:
        assert authority.is_allowed(TELEGRAM_SENDER, name).allowed
        assert not authority.is_allowed(PERSONALITY_SENDER, name).allowed
        assert not authority.is_allowed("avatar_supervisor", name).allowed


def test_schemas_are_narrow():
    schema = {n: get_tool(n).parameters for n in TOOLS}
    assert schema["people_list"]["properties"]["filter"]["enum"] == ["all", "unnamed", "today"]
    assert schema["people_list"]["required"] == []
    for name in ("person_photo", "name_person"):
        assert schema[name]["required"][0] == "id" and schema[name]["additionalProperties"] is False
    assert schema["name_person"]["required"] == ["id", "name"]
