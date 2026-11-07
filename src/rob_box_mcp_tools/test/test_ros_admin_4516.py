"""Админские тулы ЧТЕНИЯ для пульта TG и ТАРС (issue #4516, шаг 1, ADR-0148).

Числа и фразы строит код (``ros_introspect``), модель получает готовый текст. ROS не нужен: граф и подписка подставные,
rclpy-часть (``RosProbe.collect``/``render``) подменяется. Секреты в ответе маскируются, тяжёлые типы - только сводка.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace
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

# rob_box_voice.utils тянет pyaudio (на Windows/CI без него пакет не импортируется), а redact.py - чистая stdlib: грузим
# настоящий файл по пути, как test_operator_admin.py; без него _sanitize_text честно скрывает строки целиком.
_redact_path = Path(__file__).resolve().parents[2] / "rob_box_voice" / "rob_box_voice" / "utils" / "redact.py"
try:
    import rob_box_voice.utils.redact  # noqa: F401
except ImportError:
    _spec = importlib.util.spec_from_file_location("rob_box_voice.utils.redact", _redact_path)
    _redact_mod = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(_redact_mod)
    sys.modules.setdefault("rob_box_voice.utils", Mock())
    sys.modules["rob_box_voice.utils.redact"] = _redact_mod

from rob_box_core.tool_catalog import (  # noqa: E402
    get_tool,
    llm_visible_tools,
    operator_visible_tools,
    telegram_visible_tools,
)
from rob_box_mcp_tools import ros_introspect as ri  # noqa: E402
from rob_box_mcp_tools.mcp_auth import PERSONALITY_SENDER, TELEGRAM_SENDER  # noqa: E402
from rob_box_mcp_tools.slice_authority import load_default_authority  # noqa: E402
from rob_box_mcp_tools.tools import operator_admin, ros_admin  # noqa: E402

NEW_TOOLS = ("ros_list", "topic_echo", "topic_stats", "vesc_telemetry")
READ_TOOLS = ("read_logs", "ros2_node_status", "container_status", "show_metrics")


def make_node(topics=None, services=None, publishers=1):
    node = MagicMock()
    node.get_topic_names_and_types.return_value = (
        topics
        if topics is not None
        else [
            ("/battery_state", ["sensor_msgs/msg/BatteryState"]),
            ("/scan", ["sensor_msgs/msg/LaserScan"]),
            ("/camera/image_raw", ["sensor_msgs/msg/Image"]),
            ("/cmd_vel", ["geometry_msgs/msg/Twist"]),
        ]
    )
    node.get_service_names_and_types.return_value = services or [("/dialog/control", ["std_srvs/srv/SetBool"])]
    node.get_publishers_info_by_topic.return_value = [object()] * publishers
    return node


def samples(*pairs):
    return [ri.Sample(at, size, b"x" * min(size, 10)) for at, size in pairs]


# ------------------------------------------------------------------------------------------------ чистая часть


def test_rate_meter_hz_and_bw_come_from_arrivals():
    meter = ri.RateMeter([(0.0, 100), (0.5, 100), (1.0, 100), (1.5, 100), (2.0, 100)])
    stats = meter.summary(5.0)
    assert stats["count"] == 5 and stats["hz"] == 2.0
    assert stats["bw_bytes_s"] == 100.0 and stats["avg_size_bytes"] == 100


def test_rate_meter_does_not_claim_zero_hz_without_messages():
    stats = ri.RateMeter().summary(5.0)
    assert stats["hz"] is None and stats["bw_bytes_s"] is None
    assert "ни одного" in ri.rate_message("/x", "T", stats)
    one = ri.RateMeter([(0.0, 10)]).summary(5.0)
    assert one["hz"] is None and "мало данных" in ri.rate_message("/x", "T", one)


@pytest.mark.parametrize(
    "type_name,bulk",
    [
        ("sensor_msgs/msg/Image", True),
        ("sensor_msgs/msg/CompressedImage", True),
        ("sensor_msgs/msg/PointCloud2", True),
        ("nav_msgs/msg/OccupancyGrid", True),
        ("sensor_msgs/msg/LaserScan", True),
        ("sensor_msgs/msg/BatteryState", False),
        ("geometry_msgs/msg/Twist", False),
    ],
)
def test_bulk_types(type_name, bulk):
    assert ri.is_bulk_type(type_name) is bulk


def test_ceilings_clamp_whatever_the_model_asks():
    assert ri.clamp_int(99, 1, ri.ECHO_COUNT_MAX) == 5 and ri.clamp_int("abc", 1, 5) == 1 and ri.clamp_int(0, 1, 5) == 1
    assert ri.clamp_seconds(999, 5.0, 10.0) == 10.0 and ri.clamp_seconds(-3, 5.0, 10.0) == 5.0
    assert ri.clamp_seconds("nan", 5.0, 10.0) == 5.0 and ri.clamp_seconds(None, 5.0, 10.0) == 5.0


@pytest.mark.parametrize(
    "name,ok",
    [
        ("/battery_state", True),
        ("/a/b_c", True),
        ("battery", False),
        ("/a b", False),
        ("/a;rm -rf /", False),
        ("/a$(x)", False),
        (None, False),
        ("/" + "a" * 300, False),
    ],
)
def test_topic_name_validation(name, ok):
    assert ri.valid_topic_name(name) is ok


def test_graph_summary_filters_sorts_and_caps_rows():
    pairs = [(f"/t{i:03d}", ["pkg/msg/Type"]) for i in range(100)] + [("/lidar/scan", ["sensor_msgs/msg/LaserScan"])]
    data = ri.summarize_graph(pairs)
    assert data["total"] == 101 and len(data["items"]) == ri.LIST_ROWS_MAX and data["truncated"]
    assert data["items"][0]["name"] == "/lidar/scan"
    only = ri.summarize_graph(pairs, "laserscan")
    assert only["total"] == 1 and "[sensor_msgs/msg/LaserScan]" in ri.graph_message("topics", only)
    assert "ничего нет" in ri.graph_message("services", ri.summarize_graph(pairs, "нет такого"))


def iv(**values):
    return SimpleNamespace(interface_names=list(values), values=list(values.values()))


def test_vesc_report_from_joint_states_marks_missing_measures():
    names = ["left_front_wheel_joint", "right_front_wheel_joint", "battery"]
    values = [iv(velocity=1.5, position=3.0), iv(velocity=-1.0, position=2.0), iv(voltage=38.4)]
    report = ri.vesc_report(ri.vesc_from_joint_states(names, values))
    assert set(report["wheels"]) == {"front_left", "front_right"}
    assert report["wheels"]["front_left"]["raw"] == {"velocity": 1.5, "position": 3.0}
    assert report["pack_voltage_v"] == 38.4
    assert report["missing"] == ["current_a", "rpm", "temp_fet_c", "temp_motor_c"]
    text = ri.vesc_message(report)
    assert "38.4 В" in text and "Нет данных" in text and "CAN" in text and "velocity=1.5" in text


def test_vesc_report_takes_measures_by_known_interface_names():
    names = ["left_rear_wheel_joint"]
    values = [iv(voltage_input=40.0, current_motor=2.5, erpm=900.0, temperature_pcb=51.0, temperature_motor=44.0)]
    report = ri.vesc_report(ri.vesc_from_joint_states(names, values))
    row = report["wheels"]["rear_left"]
    assert (row["voltage_v"], row["current_a"], row["rpm"], row["temp_fet_c"], row["temp_motor_c"]) == (
        40.0,
        2.5,
        900.0,
        51.0,
        44.0,
    )
    assert report["missing"] == []


def test_vesc_without_wheel_joints_says_so():
    report = ri.vesc_report(ri.vesc_from_joint_states(["battery"], [iv(voltage=38.0)]))
    assert report["wheels"] == {} and "нет данных по колёсам" in ri.vesc_message(report).lower()


# ------------------------------------------------------------------------------------------------------------- тулы


def test_ros_list_topics_and_services():
    tool = ros_admin.RosListTool(make_node())
    result = tool.execute("topics", "scan")
    assert result.success and result.data["total"] == 1 and "/scan [sensor_msgs/msg/LaserScan]" in result.message
    services = tool.execute("services")
    assert services.success and "/dialog/control" in services.message


def test_tools_refuse_honestly_without_ros_node():
    for tool, args in (
        (ros_admin.RosListTool(None), ()),
        (ros_admin.TopicEchoTool(None), ("/x",)),
        (ros_admin.TopicStatsTool(None), ("/x",)),
        (ros_admin.VescTelemetryTool(None), ()),
    ):
        result = tool.execute(*args)
        assert not result.success and result.data["reason"] == ri.ROS_UNAVAILABLE


@pytest.mark.parametrize("tool_cls", [ros_admin.TopicEchoTool, ros_admin.TopicStatsTool])
def test_topic_tools_reject_bad_unknown_and_silent_topics(tool_cls):
    assert tool_cls(make_node()).execute("/a;rm -rf /").data["reason"] == ri.BAD_TOPIC
    assert tool_cls(make_node()).execute("/no/such").data["reason"] == ri.NO_SUCH_TOPIC
    assert tool_cls(make_node(publishers=0)).execute("/battery_state").data["reason"] == ri.NO_PUBLISHERS


def test_echo_reads_requested_count_capped_and_returns_text(monkeypatch):
    seen = {}

    def collect(self, topic, type_name, count, timeout_s, keep_raw=True, clock=None):
        seen.update(topic=topic, type=type_name, count=count, timeout=timeout_s)
        return samples((0.0, 50), (0.1, 50))

    monkeypatch.setattr(ri.RosProbe, "collect", collect)
    monkeypatch.setattr(ri.RosProbe, "render", staticmethod(lambda type_name, raw: "voltage: 38.4"))
    result = ros_admin.TopicEchoTool(make_node()).execute("/battery_state", count=99, timeout_s=999)
    assert seen == {"topic": "/battery_state", "type": "sensor_msgs/msg/BatteryState", "count": 5, "timeout": 10.0}
    assert result.success and result.data["count"] == 2 and "voltage: 38.4" in result.message


def test_echo_masks_secrets_and_cuts_length(monkeypatch):
    secret = "sk-ABCDEF0123456789ABCDEF0123456789"
    monkeypatch.setattr(ri.RosProbe, "collect", lambda *a, **k: samples((0.0, 50)))
    monkeypatch.setattr(
        ri.RosProbe, "render", staticmethod(lambda t, r: f"data: DEEPSEEK_API_KEY={secret} " + "x" * 5000)
    )
    result = ros_admin.TopicEchoTool(make_node()).execute("/battery_state")
    assert secret not in result.message and secret not in str(result.data)
    assert len(result.message) < ri.TEXT_CHARS_MAX + 200 and result.data["truncated"]


@pytest.mark.parametrize("topic", ["/camera/image_raw", "/scan"])
def test_echo_gives_only_a_summary_for_images_and_scans(monkeypatch, topic):
    def render(type_name, raw):
        raise AssertionError("содержимое тяжёлого типа не читается")

    monkeypatch.setattr(ri.RosProbe, "collect", lambda *a, **k: samples((0.0, 900_000), (0.03, 900_000)))
    monkeypatch.setattr(ri.RosProbe, "render", staticmethod(render))
    result = ros_admin.TopicEchoTool(make_node()).execute(topic)
    assert result.success and result.data["summary_only"] and "messages" not in result.data
    assert "содержимое не отдаётся" in result.message and "900000" in result.message


def test_echo_big_message_of_plain_type_is_summary_only(monkeypatch):
    monkeypatch.setattr(ri.RosProbe, "collect", lambda *a, **k: samples((0.0, ri.BULK_MESSAGE_BYTES + 1)))
    result = ros_admin.TopicEchoTool(make_node()).execute("/cmd_vel")
    assert result.data["summary_only"]


def test_echo_no_messages_is_honest_refusal(monkeypatch):
    monkeypatch.setattr(ri.RosProbe, "collect", lambda *a, **k: [])
    result = ros_admin.TopicEchoTool(make_node()).execute("/battery_state", timeout_s=2)
    assert not result.success and result.data["reason"] == ri.NO_MESSAGES and "2 с" in result.message


def test_stats_measures_window_without_keeping_bytes(monkeypatch):
    seen = {}

    def collect(self, topic, type_name, count, timeout_s, keep_raw=True, clock=None):
        seen.update(keep_raw=keep_raw, timeout=timeout_s)
        return samples((0.0, 1000), (0.1, 1000), (0.2, 1000), (0.3, 1000))

    monkeypatch.setattr(ri.RosProbe, "collect", collect)
    result = ros_admin.TopicStatsTool(make_node()).execute("/camera/image_raw", window_s=500)
    assert seen == {"keep_raw": False, "timeout": 10.0}
    assert result.data["hz"] == 10.0 and result.data["bw_bytes_s"] == 400.0 and result.data["count"] == 4
    assert "10.0 Гц" in result.message and "image_raw" in result.message


def test_vesc_tool_reads_joint_states_from_ros_not_can(monkeypatch):
    names = ["left_front_wheel_joint", "battery"]
    monkeypatch.setattr(
        ri.RosProbe, "read_joint_states", lambda self, timeout_s=2.0: (names, [iv(velocity=2.0), iv(voltage=39.0)])
    )
    result = ros_admin.VescTelemetryTool(make_node()).execute()
    assert result.success and result.data["source"] == "/dynamic_joint_states"
    assert result.data["pack_voltage_v"] == 39.0 and "current_a" in result.data["missing"]


def test_vesc_tool_refuses_when_ros_control_is_silent(monkeypatch):
    monkeypatch.setattr(ri.RosProbe, "read_joint_states", lambda self, timeout_s=2.0: None)
    result = ros_admin.VescTelemetryTool(make_node()).execute()
    assert not result.success and "ros2_control" in result.message


# ---------------------------------------------------------- read_logs не отдаёт секреты (проверка, не новый код)


def test_read_logs_masks_secrets_from_env_style_lines():
    secret = "sk-ABCDEF0123456789ABCDEF0123456789"
    line = f"start DEEPSEEK_API_KEY={secret} Authorization: Bearer {secret} TELEGRAM_BOT_TOKEN=123456:ABC-defGHIjkl"
    out = operator_admin._sanitize_text(line)
    assert secret not in out and "123456:ABC-defGHIjkl" not in out


# --------------------------------------------------------------------------------------------- видимость и права


def test_all_read_tools_are_visible_to_chat_and_operator_not_to_voice():
    for name in NEW_TOOLS + READ_TOOLS:
        entry = get_tool(name)
        assert entry.telegram_visible and entry.operator_visible, name
        assert name in {e.name for e in telegram_visible_tools()}
        assert name in {e.name for e in operator_visible_tools()}
        assert name not in {e.name for e in llm_visible_tools()}, name


def test_slice_policy_gives_read_tools_to_chat_and_operator_not_to_personality():
    authority = load_default_authority()
    for name in NEW_TOOLS + READ_TOOLS:
        assert authority.is_allowed(TELEGRAM_SENDER, name).allowed, name
        assert authority.is_allowed("avatar_supervisor", name).allowed, name
        assert not authority.is_allowed(PERSONALITY_SENDER, name).allowed, name


def test_all_new_tools_are_read_only_and_no_write_tool_is_given():
    for name in NEW_TOOLS:
        assert get_tool(name).read_only
    assert not {"topic_pub", "publish_topic", "ros_publish"} & {e.name for e in telegram_visible_tools()}


def test_schemas_are_narrow():
    assert get_tool("ros_list").parameters["properties"]["kind"]["enum"] == ["topics", "services"]
    assert get_tool("topic_echo").parameters["required"] == ["topic"]
    assert get_tool("topic_stats").parameters["required"] == ["topic"]
    assert get_tool("vesc_telemetry").parameters["properties"] == {}
