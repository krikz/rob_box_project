"""Публикация в топики с предохранителем (issue #4516, шаг 2а, ADR-0148): запрет по имени и по типу, проверка полей по
интерфейсу, потолок count, журнал, фразы кода. ROS не нужен: граф и издатель подставные, классы сообщений - фейки."""

from __future__ import annotations

import importlib.util
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
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

# redact.py - чистая stdlib: грузим настоящий файл по пути (rob_box_voice.utils тянет pyaudio), как в test_ros_admin.
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
from rob_box_mcp_tools import topic_publish as tp  # noqa: E402
from rob_box_mcp_tools.mcp_auth import PERSONALITY_SENDER, TELEGRAM_SENDER  # noqa: E402
from rob_box_mcp_tools.slice_authority import load_default_authority  # noqa: E402
from rob_box_mcp_tools.tools import ros_admin  # noqa: E402

# ------------------------------------------------------------------------------------------------ фейки


class Vec3:
    @staticmethod
    def get_fields_and_field_types():
        return {"x": "double", "y": "double", "z": "double"}


class StringMsg:
    @staticmethod
    def get_fields_and_field_types():
        return {"data": "string"}


class RichMsg:
    @staticmethod
    def get_fields_and_field_types():
        return {
            "flag": "boolean",
            "small": "uint8",
            "level": "int32",
            "gain": "float",
            "name": "string<=5",
            "vec": "geometry_msgs/Vector3",
            "values": "sequence<double>",
            "three": "double[3]",
            "tags": "sequence<string>",
        }


CLASSES = {
    "std_msgs/msg/String": StringMsg,
    "test_msgs/msg/Rich": RichMsg,
    "geometry_msgs/msg/Vector3": Vec3,
}


class FakePublisher:
    """Издатель вместо rclpy: помнит вызовы, может сломаться на k-м сообщении."""

    def __init__(self, subscribers=2, fail_after=None):
        self.calls = []
        self.subscribers = subscribers
        self.fail_after = fail_after

    def message_class(self, type_name):
        if type_name not in CLASSES:
            raise ri.TypeUnavailable(type_name)
        return CLASSES[type_name]

    def publish(self, topic, cls, fields, count, has_listeners):
        self.calls.append((topic, cls, dict(fields), count, has_listeners))
        if self.fail_after is not None:
            raise tp.PublishInterrupted(self.fail_after, "RuntimeError: boom")
        return tp.PublishReport(count, self.subscribers)


class FakeProbe:
    def __init__(self, topics=None, pubs=1, subs=1, available=True):
        self.graph = (
            topics
            if topics is not None
            else {
                "/animations/trigger": ["std_msgs/msg/String"],
                "/rich": ["test_msgs/msg/Rich"],
                "/odd": ["geometry_msgs/msg/Twist"],
                "/two": ["std_msgs/msg/String", "test_msgs/msg/Rich"],
                "/scan": ["std_msgs/msg/String"],
                "/lonely": ["std_msgs/msg/String"],
                "/nolib": ["no_such_pkg/msg/Thing"],
            }
        )
        self.pubs, self.subs, self._available = pubs, subs, available

    def available(self):
        return self._available

    def topic_types(self, topic):
        return list(self.graph.get(topic, []))

    def publishers_count(self, topic):
        return 0 if topic == "/lonely" else self.pubs

    def subscribers_count(self, topic):
        return 0 if topic == "/lonely" else self.subs


def make_guard(tmp_path, probe=None, publisher=None, author="TG"):
    journal = tp.Journal(str(tmp_path), clock=lambda: datetime(2026, 11, 7, 12, 0, 0, tzinfo=timezone.utc))
    pub = publisher or FakePublisher()
    guard = tp.TopicPublishGuard(probe or FakeProbe(), pub, journal, author=lambda: author)
    return guard, pub, journal


def rows(journal):
    return [json.loads(line) for line in journal.path.read_text(encoding="utf-8").splitlines()]


# ------------------------------------------------------------------------------------------------ запрет по имени


@pytest.mark.parametrize(
    "topic",
    [
        "/cmd_vel",
        "/cmd_vel_emergency",
        "/cmd_vel_joy",
        "/cmd_vel_voice",
        "/robot/cmd_vel",
        "/CMD_VEL",
        "/cmd_vel/",
        "/goal_pose",
        "/navigate_to_pose/_action/goal",
        "/navigate_through_poses/_action/status",
        "/navigate_to_pose",
        "/initialpose",
        "/bt_navigator/transition_event",
        "/diff_drive_controller/cmd_vel_unstamped",
        "/arm_controller",
        "/controller_manager/robot_description",
        "/dynamic_joint_states",
        "/joint_states",
        "/vesc/status",
        "/vesc_front/duty",
        "/can0/tx",
        "/can/raw",
        "/tf",
        "/tf_static",
        "/clock",
        "/parameter_events",
        "/rosout",
        "/mcp/execute",
        "/mcp/result",
        "/joystick_lock",
        "/teleop_lock",
        "/joy",
        "/emergency_stop",
        "/safety/stop",
        "/motor_enable",
        "/left_front_wheel/cmd",
        "/foo/_action/feedback",
        "/robot_description",
    ],
)
def test_forbidden_topic_names_are_refused_before_the_graph_is_even_asked(tmp_path, topic):
    probe = FakeProbe(available=False)  # запрет не зависит от ROS: срабатывает раньше графа
    guard, pub, _ = make_guard(tmp_path, probe=probe)

    out = guard.run(topic, {"data": "x"}, 1, True)

    assert out.status == "rejected" and out.reason == tp.TOPIC_FORBIDDEN
    assert "публиковать запрещено" in out.text and topic in out.text
    assert pub.calls == []


def test_refusal_phrase_names_the_category():
    assert tp.refusal_text(tp.TOPIC_FORBIDDEN, "/cmd_vel", category=tp.MOTION) == (
        "Отказ: /cmd_vel - топик движения, публиковать запрещено."
    )
    assert "приводов" in tp.refusal_text(tp.TOPIC_FORBIDDEN, "/vesc/x", category=tp.DRIVES)
    assert "навигации" in tp.refusal_text(tp.TOPIC_FORBIDDEN, "/goal_pose", category=tp.NAV)
    assert "служебный" in tp.refusal_text(tp.TOPIC_FORBIDDEN, "/tf", category=tp.SYSTEM)


@pytest.mark.parametrize(
    "topic,category",
    [
        ("/cmd_vel", tp.MOTION),
        ("/vesc/x", tp.DRIVES),
        ("/goal_pose", tp.NAV),
        ("/rosout", tp.SYSTEM),
        ("/mcp/execute", tp.SYSTEM),
    ],
)
def test_forbidden_category_by_name(topic, category):
    assert tp.forbidden_topic_category(topic) == category


@pytest.mark.parametrize(
    "topic",
    [
        "/scan",
        "/animations/trigger",
        "/battery_state",
        "/voice/stt/utterance",
        "/canvas/x",
        "/scanner/data",
        "/diagnostics",
    ],
)
def test_ordinary_topics_are_not_caught_by_the_name_patterns(topic):
    """``can`` как слово не ловит ``/scan``, ``/canvas``: шаблоны CAN идут по сегменту пути."""
    assert tp.forbidden_topic_category(topic) is None


# ------------------------------------------------------------------------------------------------ запрет по типу


@pytest.mark.parametrize(
    "type_name",
    [
        "geometry_msgs/msg/Twist",
        "geometry_msgs/msg/TwistStamped",
        "geometry_msgs/msg/TwistWithCovarianceStamped",
        "nav2_msgs/msg/Particle",
        "nav2_msgs/action/NavigateToPose",
        "control_msgs/msg/DynamicJointState",
        "control_msgs/msg/JointJog",
        "trajectory_msgs/msg/JointTrajectory",
        "tf2_msgs/msg/TFMessage",
        "sensor_msgs/msg/Joy",
        "sensor_msgs/msg/JointState",
        "rcl_interfaces/msg/Log",
        "rcl_interfaces/msg/ParameterEvent",
        "action_msgs/msg/GoalStatusArray",
        "lifecycle_msgs/msg/TransitionEvent",
        "pkg/action/Thing",
        "pkg/srv/Thing",
        "robot_sensor_hub_msg/msg/DeviceCommand",
        "rob_box_supervisor_msgs/msg/TeleopHeartbeat",
        "ackermann_msgs/msg/AckermannDrive",
    ],
)
def test_forbidden_types_are_refused_under_an_innocent_name(tmp_path, type_name):
    probe = FakeProbe(topics={"/innocent": [type_name]})
    guard, pub, _ = make_guard(tmp_path, probe=probe)

    out = guard.run("/innocent", {}, 1, True)

    assert out.reason == tp.TYPE_FORBIDDEN and "публиковать запрещено" in out.text
    assert pub.calls == []


def test_twist_under_a_plain_name_is_refused(tmp_path):
    guard, pub, _ = make_guard(tmp_path)

    out = guard.run("/odd", {"linear": {"x": 1.0}}, 1, True)

    assert out.reason == tp.TYPE_FORBIDDEN and "движения" in out.text and pub.calls == []


@pytest.mark.parametrize("type_name", ["std_msgs/msg/String", "sensor_msgs/msg/BatteryState", "std_msgs/msg/Bool"])
def test_ordinary_types_are_not_forbidden(type_name):
    assert tp.forbidden_type_category(type_name) is None


# ------------------------------------------------------------------------------------------------ граф


def test_topic_must_exist_and_type_comes_from_the_graph(tmp_path):
    guard, pub, _ = make_guard(tmp_path)

    out = guard.run("/nope", {"data": "x"}, 1, True)

    assert out.reason == tp.NO_SUCH_TOPIC and "в ROS-графе нет" in out.text and pub.calls == []


def test_topic_without_publishers_and_subscribers_is_refused(tmp_path):
    guard, pub, _ = make_guard(tmp_path)

    out = guard.run("/lonely", {"data": "x"}, 1, True)

    assert out.reason == tp.NO_ENDPOINTS and pub.calls == []


def test_topic_with_only_subscribers_is_publishable(tmp_path):
    guard, pub, _ = make_guard(tmp_path, probe=FakeProbe(pubs=0, subs=1))

    assert guard.run("/animations/trigger", {"data": "x"}, 1, True).done
    assert pub.calls[0][4] is True  # подписчики есть: издатель дождётся сопоставления


def test_topic_with_two_types_is_refused_not_guessed(tmp_path):
    guard, pub, _ = make_guard(tmp_path)

    out = guard.run("/two", {"data": "x"}, 1, True)

    assert out.reason == tp.AMBIGUOUS_TYPE and pub.calls == []


def test_type_missing_in_the_image_is_an_honest_refusal(tmp_path):
    guard, pub, _ = make_guard(tmp_path)

    out = guard.run("/nolib", {"data": "x"}, 1, True)

    assert out.reason == ri.TYPE_UNAVAILABLE and "не установлен" in out.text and pub.calls == []


def test_no_ros_on_this_node_is_an_honest_refusal(tmp_path):
    guard, pub, _ = make_guard(tmp_path, probe=FakeProbe(available=False))

    out = guard.run("/animations/trigger", {"data": "x"}, 1, True)

    assert out.reason == ri.ROS_UNAVAILABLE and pub.calls == []


@pytest.mark.parametrize("name", [None, "no_slash", "/a b", "/a;rm -rf /", "/a$(x)", 5, "/" + "a" * 300])
def test_unsafe_topic_names_are_refused(tmp_path, name):
    guard, pub, _ = make_guard(tmp_path)

    assert guard.run(name, {"data": "x"}, 1, True).reason == tp.BAD_TOPIC and pub.calls == []


# ------------------------------------------------------------------------------------------------ поля


@pytest.mark.parametrize(
    "fields,fragment",
    [
        ({"dta": "x"}, "неизвестное поле 'dta'"),
        ({"data": 5}, "ждали строку"),
        ({"data": None}, "ждали строку"),
        ({"data": ["a"]}, "ждали строку"),
        ("data=hello", "объектом"),
        (["data"], "объектом"),
    ],
)
def test_string_message_fields_are_checked_against_the_interface(tmp_path, fields, fragment):
    guard, pub, _ = make_guard(tmp_path)

    out = guard.run("/animations/trigger", fields, 1, True)

    assert out.reason == tp.BAD_FIELDS and fragment in out.text and pub.calls == []


@pytest.mark.parametrize(
    "fields,fragment",
    [
        ({"flag": 1}, "true/false"),
        ({"flag": "true"}, "true/false"),
        ({"small": 256}, "вне диапазона"),
        ({"small": -1}, "вне диапазона"),
        ({"small": 3.5}, "целое"),
        ({"level": "7"}, "целое"),
        ({"level": True}, "целое"),
        ({"gain": "1.5"}, "число"),
        ({"gain": float("nan")}, "число"),
        ({"gain": float("inf")}, "число"),
        ({"name": 5}, "строку"),
        ({"vec": 3}, "объект"),
        ({"vec": {"x": "1"}}, "'x'"),
        ({"vec": {"w": 1.0}}, "неизвестное поле 'w'"),
        ({"values": 1.0}, "список"),
        ({"values": [1.0, "2"]}, "[1]"),
        ({"three": "abc"}, "список"),
        ({"tags": [1]}, "[0]"),
    ],
)
def test_value_types_are_strict_no_silent_coercion(tmp_path, fields, fragment):
    guard, pub, _ = make_guard(tmp_path)

    out = guard.run("/rich", fields, 1, True)

    assert out.reason == tp.BAD_FIELDS and fragment in out.text and pub.calls == []


def test_valid_fields_of_every_kind_pass_and_ints_are_numbers_for_floats(tmp_path):
    fields = {
        "flag": True,
        "small": 255,
        "level": -5,
        "gain": 3,
        "name": "abc",
        "vec": {"x": 1, "y": 2.5, "z": 0},
        "values": [1, 2.5],
        "three": [1.0, 2.0, 3.0],
        "tags": ["a", "b"],
    }
    guard, pub, _ = make_guard(tmp_path)

    out = guard.run("/rich", fields, 1, True)

    assert out.done and pub.calls[0][2] == fields


def test_empty_fields_are_valid_for_a_message_with_defaults(tmp_path):
    guard, pub, _ = make_guard(tmp_path)

    assert guard.run("/animations/trigger", {}, 1, True).done


def test_rosidl_failure_while_building_is_a_refusal_not_an_exception(tmp_path):
    class Breaking(FakePublisher):
        def publish(self, *a):
            raise ValueError("AssertionError: The 'data' field must be of type 'str'")

    guard, _, _ = make_guard(tmp_path, publisher=Breaking())

    out = guard.run("/animations/trigger", {"data": "x"}, 1, True)

    assert out.reason == tp.BAD_FIELDS and "AssertionError" in out.text


# ------------------------------------------------------------------------------------------------ count


@pytest.mark.parametrize("count", [0, -1, 6, 100, "3", 2.0, True, [], "abc"])
def test_count_outside_1_to_5_is_refused_not_clamped(tmp_path, count):
    guard, pub, _ = make_guard(tmp_path)

    out = guard.run("/animations/trigger", {"data": "x"}, count, True)

    assert out.reason == tp.BAD_COUNT and "от 1 до 5" in out.text and pub.calls == []


@pytest.mark.parametrize("count", [None, 1, 3, 5])
def test_count_within_ceiling_goes_through(tmp_path, count):
    guard, pub, _ = make_guard(tmp_path)

    out = guard.run("/animations/trigger", {"data": "x"}, count, True)

    assert out.done and pub.calls[0][3] == (count or 1)


def test_ceiling_and_pause_are_fixed_constants():
    assert tp.COUNT_MAX == 5 and tp.PAUSE_S == 0.2


def test_tool_schema_and_constant_agree_on_the_ceiling():
    tool = ros_admin.TopicPublishTool(None, journal=tp.Journal("."))
    desc = next(p for p in tool.parameters if p.name == "count").description
    assert f"1-{tp.COUNT_MAX}" in desc


def test_rclpy_publisher_pauses_between_messages_and_releases_the_publisher(monkeypatch):
    sleeps, sent = [], []

    class Pub:
        def publish(self, msg):
            sent.append(msg)

        def get_subscription_count(self):
            return 2

    node = MagicMock()
    pub = Pub()
    node.create_publisher.return_value = pub
    adapter = tp.RclpyPublisher(node, sleep=sleeps.append)
    monkeypatch.setattr(tp.RclpyPublisher, "build", staticmethod(lambda cls, fields: {"msg": fields}))
    monkeypatch.setattr(tp.RclpyPublisher, "_qos", lambda self, topic: "qos")

    report = adapter.publish("/x", StringMsg, {"data": "x"}, 3, True)

    assert report.published == 3 and report.subscribers == 2 and len(sent) == 3
    assert sleeps == [tp.PAUSE_S, tp.PAUSE_S]
    node.destroy_publisher.assert_called_once_with(pub)


def test_rclpy_publisher_reports_how_many_went_out_when_it_breaks(monkeypatch):
    class Pub:
        n = 0

        def publish(self, msg):
            self.n += 1
            if self.n == 3:
                raise RuntimeError("boom")

        def get_subscription_count(self):
            return 1

    node = MagicMock()
    node.create_publisher.return_value = Pub()
    adapter = tp.RclpyPublisher(node, sleep=lambda s: None)
    monkeypatch.setattr(tp.RclpyPublisher, "build", staticmethod(lambda cls, fields: {}))
    monkeypatch.setattr(tp.RclpyPublisher, "_qos", lambda self, topic: "qos")

    with pytest.raises(tp.PublishInterrupted) as err:
        adapter.publish("/x", StringMsg, {}, 5, False)

    assert err.value.sent == 2 and "boom" in err.value.detail
    node.destroy_publisher.assert_called_once()


# ------------------------------------------------------------------------------------------------ вопрос и фраза


def test_without_confirm_the_tool_only_asks_one_line_and_publishes_nothing(tmp_path):
    guard, pub, journal = make_guard(tmp_path)

    out = guard.run("/animations/trigger", {"data": "hi"}, 2, False)

    assert out.status == "rejected" and out.reason == tp.NEEDS_CONFIRMATION
    assert out.data["question"] == (
        'Публикую в /animations/trigger (std_msgs/msg/String) {"data": "hi"}, 2 раза? Скажи «да, публикуй».'
    )
    assert pub.calls == [] and rows(journal)[0]["outcome"] == "asked"


def test_confirm_must_be_literally_true(tmp_path):
    guard, pub, _ = make_guard(tmp_path)

    for bad in ("true", 1, "yes", None):
        assert guard.run("/animations/trigger", {"data": "x"}, 1, bad).reason == tp.NEEDS_CONFIRMATION
    assert pub.calls == []


def test_success_phrase_is_built_by_code_from_the_fact(tmp_path):
    guard, pub, _ = make_guard(tmp_path, publisher=FakePublisher(subscribers=2))

    out = guard.run("/animations/trigger", {"data": "hi"}, 1, True)

    assert out.done
    assert out.text == "Опубликовал в /animations/trigger (std_msgs/msg/String) 1 раз, подписчиков 2."
    assert out.data["published"] == 1 and out.data["subscribers"] == 2 and out.data["summary"] == out.text


def test_zero_subscribers_is_said_honestly(tmp_path):
    guard, _, _ = make_guard(tmp_path, publisher=FakePublisher(subscribers=0), probe=FakeProbe(pubs=1, subs=0))

    out = guard.run("/animations/trigger", {"data": "x"}, 2, True)

    assert "2 раза, подписчиков 0" in out.text and "никто не принял" in out.text


def test_interrupted_publication_reports_how_many_went_out(tmp_path):
    guard, _, journal = make_guard(tmp_path, publisher=FakePublisher(fail_after=2))

    out = guard.run("/animations/trigger", {"data": "x"}, 4, True)

    assert out.reason == tp.PUBLISH_FAILED and "ушло 2 из 4" in out.text and "boom" in out.text
    assert rows(journal)[0]["outcome"] == "failed"


@pytest.mark.parametrize(
    "n,word", [(1, "1 раз"), (2, "2 раза"), (4, "4 раза"), (5, "5 раз"), (11, "11 раз"), (22, "22 раза")]
)
def test_times_word(n, word):
    assert tp.times_word(n) == word


# ------------------------------------------------------------------------------------------------ журнал


def test_every_outcome_writes_a_jsonl_row_with_author_utc_topic_type_fields(tmp_path):
    guard, _, journal = make_guard(tmp_path, author="ТАРС")

    guard.run("/animations/trigger", {"data": "hi"}, 1, True)
    guard.run("/cmd_vel", {"linear": {"x": 1}}, 1, True)
    guard.run("/animations/trigger", {"nope": 1}, 1, True)
    guard.run("/animations/trigger", {"data": "x"}, 9, True)
    guard.run("/animations/trigger", {"data": "x"}, 1, False)

    got = rows(journal)
    assert [r["outcome"] for r in got] == ["published", "refused", "refused", "refused", "asked"]
    assert [r["reason"] for r in got] == ["", tp.TOPIC_FORBIDDEN, tp.BAD_FIELDS, tp.BAD_COUNT, tp.NEEDS_CONFIRMATION]
    first = got[0]
    assert first["ts"] == "2026-11-07T12:00:00+00:00" and first["author"] == "ТАРС"
    assert first["topic"] == "/animations/trigger" and first["type"] == "std_msgs/msg/String"
    assert json.loads(first["fields"]) == {"data": "hi"} and first["count"] == 1
    assert first["text"].startswith("Опубликовал в /animations/trigger")
    assert got[1]["topic"] == "/cmd_vel" and "type" not in got[1]


def test_long_fields_are_cut_in_the_journal(tmp_path):
    guard, _, journal = make_guard(tmp_path)

    guard.run("/animations/trigger", {"data": "x" * 5000}, 1, True)

    cut = rows(journal)[0]["fields"]
    assert len(cut) <= tp.FIELDS_LOG_CHARS and cut.endswith("…")


def test_secrets_in_fields_are_redacted_through_the_sanitizer(tmp_path):
    journal = tp.Journal(str(tmp_path))
    guard = tp.TopicPublishGuard(
        FakeProbe(), FakePublisher(), journal, sanitize=lambda s: s.replace("sk-secret", "***"), author=lambda: "TG"
    )

    guard.run("/animations/trigger", {"data": "key sk-secret"}, 1, True)

    assert "sk-secret" not in journal.path.read_text(encoding="utf-8")


def test_unwritable_journal_does_not_hide_it_the_log_line_stays_and_the_result_says_so(tmp_path, caplog):
    blocker = tmp_path / "file"
    blocker.write_text("x")  # на месте каталога - файл: mkdir упадёт
    guard = tp.TopicPublishGuard(FakeProbe(), FakePublisher(), tp.Journal(str(blocker / "sub")), author=lambda: "TG")

    with caplog.at_level("INFO", logger="topic_publish"):
        out = guard.run("/animations/trigger", {"data": "x"}, 1, True)

    assert out.done and out.data["journal"] == "log_only"
    assert any("TOPIC_PUBLISH_AUDIT" in r.getMessage() for r in caplog.records)


def test_journal_row_is_in_the_container_log_too(tmp_path, caplog):
    guard, _, _ = make_guard(tmp_path)

    with caplog.at_level("INFO", logger="topic_publish"):
        guard.run("/animations/trigger", {"data": "x"}, 1, True)

    assert any(
        "TOPIC_PUBLISH_AUDIT" in r.getMessage() and "/animations/trigger" in r.getMessage() for r in caplog.records
    )


def test_journal_rotates_when_too_big(tmp_path, monkeypatch):
    monkeypatch.setattr(tp, "JOURNAL_MAX_BYTES", 10)
    journal = tp.Journal(str(tmp_path))

    journal.record({"a": 1})
    journal.record({"a": 2})

    assert (tmp_path / "topic_publish.jsonl.1").exists() and len(rows(journal)) == 1


def test_journal_directory_is_not_the_people_data_dir():
    assert tp.JOURNAL_DIR_DEFAULT != "/data" and not tp.JOURNAL_DIR_DEFAULT.startswith("/data/")


def test_author_comes_from_the_signed_sender_and_unknown_is_said_honestly():
    assert tp.current_author() == "unknown"
    with tp.caller_sender(TELEGRAM_SENDER):
        assert tp.current_author() == "TG"
    with tp.caller_sender("avatar_supervisor"):
        assert tp.current_author() == "ТАРС"
    with tp.caller_sender("somebody_else"):
        assert tp.current_author() == "unknown"
    assert tp.current_author() == "unknown"


# ------------------------------------------------------------------------------------------------ тул и срезы


def make_tool(tmp_path):
    node = MagicMock()
    node.get_topic_names_and_types.return_value = [("/animations/trigger", ["std_msgs/msg/String"])]
    node.get_publishers_info_by_topic.return_value = [object()]
    node.get_subscriptions_info_by_topic.return_value = [object()]
    return ros_admin.TopicPublishTool(node, journal=tp.Journal(str(tmp_path)))


def test_tool_forbidden_topic_is_a_failed_result_with_code_text(tmp_path):
    res = make_tool(tmp_path).execute(topic="/cmd_vel", fields={"linear": {"x": 1.0}}, confirm=True)

    assert res.success is False and res.data["reason"] == tp.TOPIC_FORBIDDEN
    assert res.error == "Отказ: /cmd_vel - топик движения, публиковать запрещено." == res.data["summary"]


def test_tool_asks_before_publishing(tmp_path, monkeypatch):
    monkeypatch.setattr(tp.RclpyPublisher, "message_class", lambda self, t: StringMsg)

    res = make_tool(tmp_path).execute(topic="/animations/trigger", fields={"data": "hi"})

    assert res.success is False and res.data["reason"] == tp.NEEDS_CONFIRMATION and "question" in res.data


def test_tool_publishes_after_confirm_with_author_from_the_context(tmp_path, monkeypatch):
    sent = []
    monkeypatch.setattr(tp.RclpyPublisher, "message_class", lambda self, t: StringMsg)
    monkeypatch.setattr(
        tp.RclpyPublisher,
        "publish",
        lambda self, topic, cls, f, n, has: sent.append((topic, f, n)) or tp.PublishReport(n, 1),
    )
    tool = make_tool(tmp_path)

    with tp.caller_sender(TELEGRAM_SENDER):
        res = tool.execute(topic="/animations/trigger", fields={"data": "hi"}, count=2, confirm=True)

    assert res.success
    assert res.message == "Опубликовал в /animations/trigger (std_msgs/msg/String) 2 раза, подписчиков 1."
    assert sent == [("/animations/trigger", {"data": "hi"}, 2)]
    row = rows(tool._journal)[0]
    assert row["author"] == "TG" and row["outcome"] == "published"


def test_tool_is_in_operator_admin_for_tg_and_tars_and_hidden_from_voice():
    tool = ros_admin.TopicPublishTool(None, journal=tp.Journal("."))
    assert tool.slice == "operator.admin" and tool.llm_visible is False and tool.telegram_visible is True
    assert tool.read_only is False
    authority = load_default_authority()
    assert authority.is_allowed(TELEGRAM_SENDER, "topic_publish").allowed
    assert authority.is_allowed("avatar_supervisor", "topic_publish").allowed
    assert not authority.is_allowed(PERSONALITY_SENDER, "topic_publish").allowed
    assert not authority.is_allowed("harness", "topic_publish").allowed
    assert get_tool("topic_publish").telegram_visible is True
    assert "topic_publish" in {e.name for e in telegram_visible_tools()}
    assert "topic_publish" in {e.name for e in operator_visible_tools()}
    assert "topic_publish" not in {e.name for e in llm_visible_tools()}


def test_tool_is_registered_on_the_server_and_the_context_is_held_around_execute():
    src = (Path(__file__).resolve().parents[1] / "rob_box_mcp_tools" / "mcp_server.py").read_text(encoding="utf-8")
    assert "TopicPublishTool(self)" in src
    assert "execute_as(self.registry, sender, tool_name, parameters)" in src


def test_execute_as_holds_the_sender_for_the_tool_and_releases_it_after():
    seen = []

    class Registry:
        def execute(self, name, **kwargs):
            seen.append((name, kwargs, tp.current_author()))
            return "res"

    assert tp.execute_as(Registry(), TELEGRAM_SENDER, "topic_publish", {"topic": "/x"}) == "res"
    assert seen == [("topic_publish", {"topic": "/x"}, "TG")]
    assert tp.current_author() == "unknown"
