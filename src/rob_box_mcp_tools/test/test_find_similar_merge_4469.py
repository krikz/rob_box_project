"""``find_similar`` и ``merge_people`` (issue #4469, ADR-0148): номера коллажа -> id переводит код, не модель.

Владелец лиц подставной: тест проверяет, что тул кладёт в вызов владельца, что отказывает сам и какую фразу строит.
"""

from __future__ import annotations

import sys
from unittest.mock import MagicMock, Mock

import pytest

try:
    import rclpy  # noqa: F401
except ImportError:
    for _mod in (
        "rob_box_mcp_tools.tools.navigation",
        "rob_box_mcp_tools.tools.system",
        "rob_box_mcp_tools.tools.perception",
        "rob_box_mcp_tools.tools.mapping",
        "rob_box_mcp_tools.tools.memory",
        "rob_box_mcp_tools.tools.music",
    ):
        sys.modules.setdefault(_mod, Mock())

from rob_box_core.tool_catalog import get_tool  # noqa: E402
from rob_box_mcp_tools.tools import people  # noqa: E402

TARGET = "71cda108"
IDS = ["aaaaaaa1", "bbbbbbb2", "ccccccc3", "ddddddd4", "eeeeeee5"]
CANDIDATES = [
    {"n": n, "id": pid, "name": None, "similarity": 0.9 - n / 10, "same_name": n == 1} for n, pid in enumerate(IDS, 1)
]


class Clock:
    def __init__(self) -> None:
        self.now = 1000.0

    def __call__(self) -> float:
        return self.now


class Owner:
    """Владелец: отвечает по ``op``, записывает вызовы."""

    def __init__(self, replies=None):
        self.replies, self.calls = replies or {}, []

    def __call__(self, op, flt, person_id, name):
        self.calls.append((op, flt, person_id, name))
        return self.replies[op]


def make(cls, owner, memory):
    return cls(MagicMock(), owner, memory)


def found(owner_name="Денчик"):
    return (True, "", {"id": TARGET, "name": owner_name, "candidates": CANDIDATES, "size": 1234})


def searched(clock=None, name="Денчик", merge=None):
    """Память с коллажем, сделанным настоящим ``find_similar``; возвращает ``(memory, owner, merge_tool)``."""
    memory = people.CollageMemory(clock or Clock())
    owner = Owner(
        {
            people.OP_SIMILAR: found(name),
            people.OP_MERGE: merge
            or (
                True,
                "",
                {
                    "id": TARGET,
                    "merged": 3,
                    "name": name,
                    "gallery": 14,
                    "backup": "/data/faces_backup/x",
                    "merged_ids": [],
                },
            ),
        }
    )
    assert make(people.FindSimilarTool, owner, memory).execute(id=TARGET).success
    return memory, owner, make(people.MergePeopleTool, owner, memory)


# --- find_similar -----------------------------------------------------------------------------------------------


def test_find_similar_sends_the_collage_and_remembers_numbers_in_code():
    memory = people.CollageMemory(Clock())
    owner = Owner({people.OP_SIMILAR: found()})

    result = make(people.FindSimilarTool, owner, memory).execute(id=TARGET)

    assert owner.calls == [(people.OP_SIMILAR, "", TARGET, "")]
    assert result.success and result.data["photo_id"] == TARGET and result.data["photo_kind"] == "collage"
    assert result.data["caption"] == f"Похожие на {TARGET} (Денчик)" and len(result.data["candidates"]) == 5
    assert memory.latest().ids == tuple(IDS) and memory.latest().target == TARGET
    assert "1. aaaaaaa1" in result.message and "(тёзка)" in result.message and "сходство 80%" in result.message
    assert result.data["summary"].startswith(f"Похожих на {TARGET}: 5")


def test_find_similar_without_candidates_says_so_and_drops_the_old_numbers():
    memory = people.CollageMemory(Clock())
    memory.remember("00000000", None, ["11111111"])
    owner = Owner({people.OP_SIMILAR: (True, "", {"id": TARGET, "name": None, "candidates": []})})

    result = make(people.FindSimilarTool, owner, memory).execute(id=TARGET)

    assert result.success and result.data["candidates"] == [] and "photo_id" not in result.data
    assert result.data["summary"] == f"Похожих на {TARGET} (без имени) не нашёл." and memory.latest() is None


@pytest.mark.parametrize("reason", ["exhibition_mode", "person_not_found", "collage_failed", "owner_unavailable"])
def test_find_similar_refusal_is_the_owners_reason(reason):
    memory = people.CollageMemory(Clock())

    result = make(people.FindSimilarTool, Owner({people.OP_SIMILAR: (False, reason, {})}), memory).execute(id=TARGET)

    assert result.success is False and result.data == {"status": "rejected", "reason": reason}
    assert memory.latest() is None


# --- перевод номеров в id ---------------------------------------------------------------------------------------


def _collage():
    memory = people.CollageMemory(Clock())
    memory.remember(TARGET, "Денчик", IDS)
    return memory.latest()


@pytest.mark.parametrize(
    "numbers, all_except, ids",
    [
        ([1, 3, 5], None, ["aaaaaaa1", "ccccccc3", "eeeeeee5"]),
        ([5, 1, 1], None, ["aaaaaaa1", "eeeeeee5"]),  # порядок и повтор не важны
        (None, [2], ["aaaaaaa1", "ccccccc3", "ddddddd4", "eeeeeee5"]),
        ([], [1, 2, 3, 4], ["eeeeeee5"]),
    ],
)
def test_numbers_become_ids_by_the_last_collage(numbers, all_except, ids):
    assert people.choose_ids(_collage(), numbers, all_except) == (ids, "")


@pytest.mark.parametrize(
    "numbers, all_except, reason",
    [
        ([6], None, "bad_number"),
        ([0], None, "bad_number"),
        ([-1], None, "bad_number"),
        (None, [9], "bad_number"),
        ([1.5], None, "bad_number"),
        (["1"], None, "bad_number"),
        ([True], None, "bad_number"),
        (None, None, "bad_selection"),
        ([], [], "bad_selection"),
        ([1], [2], "bad_selection"),
        (None, [1, 2, 3, 4, 5], "bad_selection"),
    ],
)
def test_bad_numbers_are_refused_with_a_reason(numbers, all_except, reason):
    assert people.choose_ids(_collage(), numbers, all_except) == ([], reason)


# --- merge_people -----------------------------------------------------------------------------------------------


def test_merge_without_a_collage_refuses_and_does_not_call_the_owner():
    owner = Owner()

    result = make(people.MergePeopleTool, owner, people.CollageMemory(Clock())).execute(target=TARGET, numbers=[1])

    assert result.success is False and result.data["reason"] == "no_collage" and owner.calls == []


def test_a_collage_expires_and_the_model_cannot_substitute_ids():
    clock = Clock()
    memory, owner, tool = searched(clock)
    owner.calls.clear()

    clock.now += people.COLLAGE_TTL_S + 1

    assert tool.execute(target=TARGET, numbers=[1], confirm=True).data["reason"] == "no_collage"
    assert owner.calls == []


def test_merge_for_another_target_or_out_of_range_never_reaches_the_owner():
    memory, owner, tool = searched()
    owner.calls.clear()

    assert tool.execute(target="12345678", numbers=[1], confirm=True).data["reason"] == "collage_mismatch"
    assert tool.execute(target=TARGET, numbers=[7], confirm=True).data["reason"] == "bad_number"
    assert tool.execute(target=TARGET, confirm=True).data["reason"] == "bad_selection"
    assert owner.calls == []


def test_merge_first_asks_one_line_and_changes_nothing():
    memory, owner, tool = searched()
    owner.calls.clear()

    result = tool.execute(target=TARGET, numbers=[1, 3, 5])

    assert result.success is False and result.data["reason"] == people.NEEDS_MERGE_CONFIRMATION
    assert result.data["question"] == "Склеиваю 3 записи в «Денчик» (71cda108)? Скажи «да, склей»."
    assert owner.calls == [], "до «да» владелец не зовётся: ни слияния, ни бэкапа"
    assert memory.latest() is not None, "коллаж живёт до ответа: «да» исполнит тот же вызов"


@pytest.mark.parametrize(
    "count, word",
    [
        (1, "запись"),
        (2, "записи"),
        (4, "записи"),
        (5, "записей"),
        (11, "записей"),
        (12, "записей"),
        (21, "запись"),
        (22, "записи"),
    ],
)
def test_question_declines_the_noun(count, word):
    assert (
        people.merge_question(count, "Имя", TARGET) == f"Склеиваю {count} {word} в «Имя» ({TARGET})? Скажи «да, склей»."
    )


def test_confirmed_merge_sends_the_ids_of_the_collage_and_builds_the_phrase_from_the_fact():
    memory, owner, tool = searched()
    owner.calls.clear()

    result = tool.execute(target=TARGET, numbers=[1, 3, 5], confirm=True)

    assert owner.calls == [(people.OP_MERGE, "aaaaaaa1,ccccccc3,eeeeeee5", TARGET, "")]
    assert result.success and result.data["status"] == "done" and result.data["merged"] == 3
    assert result.data["summary"] == result.message == "Склеено 3, теперь у Денчик 14 снимков."
    assert result.data["stored_name"] == "Денчик" and result.data["backup"] == "/data/faces_backup/x"
    assert memory.latest() is None, "слитые записи исчезли: номера коллажа больше не про них"


def test_all_except_merges_the_rest():
    memory, owner, tool = searched()
    owner.calls.clear()

    tool.execute(target=TARGET, all_except=[2], confirm="true")

    assert owner.calls[0][1] == "aaaaaaa1,ccccccc3,ddddddd4,eeeeeee5"


@pytest.mark.parametrize("gallery, tail", [(1, "1 снимок"), (3, "3 снимка"), (5, "5 снимков"), (21, "21 снимок")])
def test_phrase_is_built_from_what_the_store_reports(gallery, tail):
    data = {"id": TARGET, "merged": 2, "name": "Имя", "gallery": gallery}
    assert people.merge_summary(data) == f"Склеено 2, теперь у Имя {tail}."
    assert people.merge_summary({**data, "name": None}) == f"Склеено 2, теперь у лица {TARGET} {tail}."


@pytest.mark.parametrize("reason", ["person_not_found", "ambiguous_id", "merge_failed", "owner_unavailable"])
def test_owner_refusal_is_not_a_success_phrase(reason):
    memory, owner, tool = searched(merge=(False, reason, {}))

    result = tool.execute(target=TARGET, numbers=[1], confirm=True)

    assert result.success is False and result.data == {"status": "rejected", "reason": reason}
    assert memory.latest() is not None, "склейки не было - коллаж остаётся для повтора"


def test_owner_that_merged_nothing_is_not_trusted():
    memory, owner, tool = searched(merge=(True, "", {"id": TARGET, "merged": 0, "name": "Денчик", "gallery": 4}))

    result = tool.execute(target=TARGET, numbers=[1], confirm=True)

    assert result.success is False and result.data["reason"] == "merge_unconfirmed"


# --- видимость и схема ------------------------------------------------------------------------------------------


def test_new_tools_are_for_the_telegram_agent_only_and_the_schema_is_narrow():
    for name in ("find_similar", "merge_people"):
        entry = get_tool(name)
        assert entry.telegram_visible and not entry.llm_visible and not entry.operator_visible
    merge = get_tool("merge_people").parameters
    assert merge["required"] == ["target"] and merge["additionalProperties"] is False
    assert merge["properties"]["numbers"]["items"]["type"] == "integer"
    assert merge["properties"]["all_except"]["type"] == "array" and merge["properties"]["confirm"]["type"] == "boolean"
    assert get_tool("find_similar").parameters["required"] == ["id"]
