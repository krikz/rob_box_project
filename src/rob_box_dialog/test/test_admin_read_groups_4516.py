"""Срез тулов чтения ROS и VESC (#4516): чат и ТАРС открывают группы словами, голос - нет."""

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.session import Session
from rob_box_dialog.tools_view import available_tools, groups_for

GRAPH = {"ros_list", "topic_echo", "topic_stats"}
READ = {"read_logs", "ros2_node_status", "container_status", "show_metrics"}


def _names(text, agent):
    return {t.name for t in available_tools(Session(agent).snapshot, None, agent, text)}


@pytest.mark.parametrize("agent", ["telegram", "operator"])
@pytest.mark.parametrize(
    "text",
    [
        "какие есть топики",
        "что в топике /battery_state",
        "померяй частоту /scan",
        "какой hz у камеры",
        "покажи полосу по /odom",
        "список сервисов ros",
    ],
)
def test_graph_tools_open_by_words(agent, text):
    assert GRAPH <= _names(text, agent)


@pytest.mark.parametrize("agent", ["telegram", "operator"])
@pytest.mark.parametrize(
    "text", ["что с vesc", "напряжение на колёсах", "температура моторов", "какие обороты колеса", "ток мотора"]
)
def test_vesc_tool_opens_by_words(agent, text):
    assert "vesc_telemetry" in _names(text, agent)


@pytest.mark.parametrize(
    "text,tool",
    [
        ("покажи логи voice-assistant", "read_logs"),
        ("какие ноды живы", "ros2_node_status"),
        ("статус контейнеров", "container_status"),
        ("покажи метрики cpu", "show_metrics"),
    ],
)
def test_existing_read_tools_are_in_chat_slice(text, tool):
    assert tool in _names(text, "telegram")


@pytest.mark.parametrize("text", ["какие есть топики", "что с vesc", "какое напряжение", "померяй частоту /scan"])
def test_voice_slice_never_gets_admin_tools_and_keeps_status(text):
    names = _names(text, "personality")
    assert not (GRAPH | READ | {"vesc_telemetry"}) & names
    assert set(kn.TOOL_GROUPS["status"].tools) & names
    groups = groups_for(Session("personality").snapshot, text, agent="personality")
    assert "ros_graph" not in groups and "vesc" not in groups


def test_unrelated_text_does_not_open_admin_groups():
    for text in ("расскажи анекдот", "привет", "включи музыку"):
        assert not (GRAPH | {"vesc_telemetry"}) & _names(text, "telegram")


def test_chat_slice_stays_within_cap():
    for text in ("топики и vesc и логи и ноды", "частота топика /scan и напряжение колёс"):
        assert len(_names(text, "telegram")) <= kn.TOOLS_VISIBLE_MAX
