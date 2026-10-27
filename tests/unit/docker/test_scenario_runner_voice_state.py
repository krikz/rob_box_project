"""scenario_runner: «ход закончен» читается из контрактов диалога v2, а не из ``/voice/dialogue/state`` (#3711, 15-0a').

Раннер интеграционного стенда (``docker/vision/test/scenario_runner/runner.py``) ждёт конца хода в ``wait_for_idle``.
Раньше он брал состояние из строки старого топика ``/voice/dialogue/state`` (писал только ``dialogue_node`` v1).
Теперь - из ``rob_box_core.voice_state`` (``TurnEvent``/``SpeechEvent``/``SessionState``), той же таблицы, что у шлема.

Без ROS: ``rclpy`` и ``std_msgs`` заменены заглушками, сообщения диалога собраны из настоящих ``.msg``
(``rob_box_core.msg_stub``), сам ``runner.py`` загружается из файла как есть.
"""

from __future__ import annotations

import importlib.util
import sys
import threading
import time
import types
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / "src" / "rob_box_core"))

from rob_box_core.msg_stub import build_dialog_msgs_module  # noqa: E402

RUNNER = REPO_ROOT / "docker" / "vision" / "test" / "scenario_runner" / "runner.py"

TURN_TOPIC = "/dialog/turn_event"
SPEECH_TOPIC = "/voice/speech/event"
SESSION_TOPIC = "/dialog/session"
V1_STATE_TOPIC = "/voice/dialogue/state"
KIND_STARTED, KIND_FINISHED = 2, 4


class _FakeNode:
    """Минимум ``rclpy.node.Node`` для ``ScenarioRunner.__init__``: подписки запоминаются, остальное - пустышки."""

    def __init__(self, name):
        self.name = name
        self.subscriptions = []

    def get_logger(self):
        noop = lambda *a, **k: None  # noqa: E731
        return types.SimpleNamespace(info=noop, warning=noop, debug=noop, error=noop)

    def create_publisher(self, *args, **kwargs):
        return types.SimpleNamespace(publish=lambda msg: None)

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscriptions.append((topic, msg_type, callback))

    def create_timer(self, period, callback):
        return types.SimpleNamespace(cancel=lambda: None)

    def deliver(self, topic, msg):
        for sub_topic, _msg_type, callback in self.subscriptions:
            if sub_topic == topic:
                callback(msg)


def _module(name, **attrs):
    mod = types.ModuleType(name)
    for key, value in attrs.items():
        setattr(mod, key, value)
    return mod


@pytest.fixture()
def runner_module(monkeypatch):
    """``runner.py`` с заглушками ROS; модули убираются из ``sys.modules`` после теста."""
    msgs = build_dialog_msgs_module()
    policy = types.SimpleNamespace(BEST_EFFORT=1, RELIABLE=2, KEEP_LAST=3, TRANSIENT_LOCAL=4)
    stubs = {
        "rclpy": _module("rclpy", ok=lambda: True),
        "rclpy.node": _module("rclpy.node", Node=_FakeNode),
        "rclpy.executors": _module("rclpy.executors", MultiThreadedExecutor=object),
        "rclpy.qos": _module(
            "rclpy.qos",
            QoSProfile=lambda **kw: types.SimpleNamespace(**kw),
            ReliabilityPolicy=policy,
            HistoryPolicy=policy,
            DurabilityPolicy=policy,
        ),
        "std_msgs": _module("std_msgs"),
        "std_msgs.msg": _module("std_msgs.msg", Bool=type("Bool", (), {}), String=type("String", (), {})),
        "rob_box_dialog_msgs": _module("rob_box_dialog_msgs", msg=msgs),
        "rob_box_dialog_msgs.msg": msgs,
    }
    for name, module in stubs.items():
        monkeypatch.setitem(sys.modules, name, module)
    spec = importlib.util.spec_from_file_location("scenario_runner_under_test", RUNNER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.dialog_msgs = msgs
    return module


@pytest.fixture()
def node(runner_module):
    return runner_module.ScenarioRunner()


def _turn(msgs, stage, status="started"):
    return msgs.TurnEvent(agent="personality", stage=stage, status=status)


def _speech(msgs, kind):
    return msgs.SpeechEvent(agent="personality", kind=kind, speech_id="s1")


def test_runner_reads_voice_state_from_dialog_v2_contracts_not_from_v1_topic(node):
    topics = {topic for topic, _msg_type, _callback in node.subscriptions}
    assert {TURN_TOPIC, SPEECH_TOPIC, SESSION_TOPIC} <= topics
    assert V1_STATE_TOPIC not in topics


def test_voice_state_follows_turn_and_speech(node, runner_module):
    msgs = runner_module.dialog_msgs
    assert node._voice_state == "idle"
    node.deliver(TURN_TOPIC, _turn(msgs, "addressed"))
    assert node._voice_state == "listening"
    node.deliver(TURN_TOPIC, _turn(msgs, "thinking"))
    assert node._voice_state == "thinking"
    node.deliver(SPEECH_TOPIC, _speech(msgs, KIND_STARTED))
    node.deliver(TURN_TOPIC, _turn(msgs, "done", "finished"))  # ход закрылся сразу после запроса речи
    assert node._voice_state == "speaking"
    node.deliver(SPEECH_TOPIC, _speech(msgs, KIND_FINISHED))
    assert node._voice_state == "idle"


def test_wait_for_idle_returns_only_after_the_turn_and_the_speech_end(node, runner_module):
    msgs = runner_module.dialog_msgs
    node.deliver(TURN_TOPIC, _turn(msgs, "thinking"))
    result = {}
    waiter = threading.Thread(
        target=lambda: result.update(idle=node.wait_for_idle(first_non_idle_timeout_s=0.2, idle_timeout_s=10.0))
    )
    waiter.start()
    try:
        time.sleep(0.4)
        assert waiter.is_alive(), "wait_for_idle вернулся, пока ход ещё думает"
        node.deliver(SPEECH_TOPIC, _speech(msgs, KIND_STARTED))
        node.deliver(TURN_TOPIC, _turn(msgs, "done", "finished"))
        time.sleep(0.4)
        assert waiter.is_alive(), "wait_for_idle вернулся, пока реплика ещё звучит"
        node.deliver(SPEECH_TOPIC, _speech(msgs, KIND_FINISHED))
        waiter.join(timeout=5.0)
    finally:
        node.deliver(SPEECH_TOPIC, _speech(msgs, KIND_FINISHED))
        waiter.join(timeout=5.0)
    assert not waiter.is_alive()
    assert result == {"idle": True}


def test_wait_for_idle_times_out_while_the_turn_is_not_closed(node, runner_module):
    node.deliver(TURN_TOPIC, _turn(runner_module.dialog_msgs, "thinking"))
    assert node.wait_for_idle(first_non_idle_timeout_s=0.3, idle_timeout_s=0.6) is False
    assert node._voice_state == "thinking"


def test_wait_for_idle_is_instant_when_the_robot_never_left_idle(node):
    assert node.wait_for_idle(first_non_idle_timeout_s=0.3, idle_timeout_s=0.6) is True
