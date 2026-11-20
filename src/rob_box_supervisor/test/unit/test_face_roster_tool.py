"""Тесты инструмента ТАРС ``list_faces`` (issue #3655).

Источник — ответ владельца ``/perception/people`` ``OP_LIST``, не диск
``/data/faces``. ROS нет: ``call`` подставляется.

Запуск::

    pytest -v src/rob_box_supervisor/test/unit/test_face_roster_tool.py
"""

from __future__ import annotations

import json
import sys
from typing import Any, Dict, List, Tuple
from unittest.mock import MagicMock

from rob_box_supervisor.face_roster_tool import (
    DEFAULT_LIMIT,
    MAX_LIMIT,
    OWNER_UNAVAILABLE,
    PEOPLE_SERVICE,
    TOOL_NAME,
    listing_from_owner,
    register_list_faces_tool,
    ros_people_list,
    unavailable_listing,
)

NOW = 1_800_000_000.0


def _person(
    pid: str,
    *,
    name: str | None = None,
    encounters: int = 0,
    last: float | None = None,
) -> Dict[str, Any]:
    return {
        "id": pid[:8],
        "name": name,
        "epithet": "не должен утечь",
        "encounters": encounters,
        "last_encounter_ts": last or 0.0,
        "voice_linked": True,
        "has_snapshot": True,
    }


OWNER_PEOPLE: List[Dict[str, Any]] = [
    _person("aaaaaaaa11", name="Деньчик", encounters=27, last=NOW - 120),
    _person("dddddddd44", name=None, encounters=1, last=NOW - 600),
    _person("bbbbbbbb22", name=None, encounters=5, last=NOW - 7200),
    _person("cccccccc33", name="Борис", encounters=3, last=NOW - 3 * 86400),
]


def _owner_data(people: List[Dict[str, Any]] | None = None) -> Dict[str, Any]:
    rows = OWNER_PEOPLE if people is None else people
    return {"people": rows, "total": len(rows), "shown": len(rows), "filter": "all"}


def test_listing_sorted_as_owner_sent() -> None:
    out = listing_from_owner(_owner_data(), now=NOW)
    assert out["status"] == "ok"
    assert [f["id"] for f in out["faces"]] == [
        "aaaaaaaa",
        "dddddddd",
        "bbbbbbbb",
        "cccccccc",
    ]
    first = out["faces"][0]
    assert first["name"] == "Деньчик"
    assert first["encounters"] == 27
    assert first["last_seen_ago"] == "2 мин назад"
    assert first["last_seen"].endswith("UTC")
    assert out["faces"][2]["name"] == "незнакомец"
    assert out["faces"][3]["last_seen_ago"] == "3 дн назад"
    assert (out["total_in_base"], out["named_in_base"]) == (4, 2)


def test_listing_leaks_no_embeddings_paths_or_owner_extras() -> None:
    out = listing_from_owner(_owner_data(), now=NOW)
    blob = json.dumps(out, ensure_ascii=False)
    assert "epithet" not in blob
    assert "voice_linked" not in blob
    assert "has_snapshot" not in blob
    assert "embeddings" not in blob
    assert "/data/faces" not in blob
    assert ".jpg" not in blob
    assert set(out["faces"][0]) == {
        "id",
        "name",
        "encounters",
        "last_seen",
        "last_seen_ago",
    }


def test_limit_and_named_only() -> None:
    out = listing_from_owner(_owner_data(), limit=2, now=NOW)
    assert out["shown"] == 2 and out["total_in_base"] == 4
    named = listing_from_owner(_owner_data(), named_only=True, now=NOW)
    assert [f["name"] for f in named["faces"]] == ["Деньчик", "Борис"]
    assert named["total_in_base"] == 4


def test_limit_is_clamped() -> None:
    people = [_person(f"{i:08x}", name=f"p{i}", last=NOW - i) for i in range(MAX_LIMIT + 5)]
    data = {"people": people, "total": len(people)}
    assert listing_from_owner(data, limit=10_000, now=NOW)["shown"] == MAX_LIMIT
    assert listing_from_owner(data, limit=0, now=NOW)["shown"] == 1


def test_empty_owner_list_is_honest_empty_not_unavailable() -> None:
    out = listing_from_owner({"people": [], "total": 0}, now=NOW)
    assert out["status"] == "ok"
    assert out["faces"] == [] and out["total_in_base"] == 0
    assert "пуста" in out["message"]
    assert "Vision Pi" not in out["message"]


def test_owner_total_can_exceed_returned_rows() -> None:
    data = {"people": OWNER_PEOPLE[:2], "total": 107, "shown": 2, "filter": "all"}
    out = listing_from_owner(data, now=NOW)
    assert out["total_in_base"] == 107
    assert out["shown"] == 2


def test_unavailable_is_not_empty() -> None:
    out = unavailable_listing()
    assert out["status"] == "unavailable"
    assert "faces" not in out
    assert out["reason"] == OWNER_UNAVAILABLE
    assert "Vision Pi" in out["message"]


def test_register_tool_uses_owner_call() -> None:
    registry = MagicMock()

    def call() -> Tuple[bool, str, Any]:
        return True, "", _owner_data()

    register_list_faces_tool(registry, call=call)
    spec, handler = registry.register.call_args.args
    assert spec.name == TOOL_NAME == "list_faces"
    assert spec.parameters["required"] == []
    assert "/perception/people" in spec.description
    out = handler({"limit": "2"})
    assert out["status"] == "ok" and out["shown"] == 2
    assert handler({"limit": "abc"})["shown"] == min(4, DEFAULT_LIMIT)
    assert registry.register.call_args.kwargs == {"override": True}


def test_register_without_call_is_unavailable() -> None:
    registry = MagicMock()
    register_list_faces_tool(registry)
    _, handler = registry.register.call_args.args
    out = handler({})
    assert out["status"] == "unavailable"
    assert "Vision Pi" in out["message"]


def test_register_owner_refusal_is_unavailable() -> None:
    registry = MagicMock()
    register_list_faces_tool(registry, call=lambda: (False, "store_error", {}))
    _, handler = registry.register.call_args.args
    out = handler({})
    assert out["status"] == "unavailable"
    assert out["reason"] == "store_error"
    assert "faces" not in out


def test_ros_call_without_a_ready_service_is_owner_unavailable() -> None:
    node = MagicMock()
    node.__dict__["_tars_people_clients"] = {"client": MagicMock(service_is_ready=lambda: False)}
    sys.modules.setdefault("rob_box_perception_msgs", MagicMock())
    sys.modules.setdefault("rob_box_perception_msgs.srv", MagicMock())
    assert ros_people_list(node)() == (False, OWNER_UNAVAILABLE, {})


def test_service_name_is_existing_people_contract() -> None:
    assert PEOPLE_SERVICE == "/perception/people"
