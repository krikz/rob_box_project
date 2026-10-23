"""#3904: один владелец режима e2e (``rob_box_core.e2e_mode``) — владелец, таймер снятия, последователь.

Ноды фейковые (без ROS): ``set_parameters`` прогоняет все колбэки параметров по одному, как rclpy, и пишет значение
в реестр, только если колбэки согласились. Шина ``/e2e/mode`` — список подписчиков.
"""

from __future__ import annotations

import sys
import types
from typing import Any, Dict, List

import pytest

from rob_box_core import e2e_mode as M


class _Result:
    def __init__(self, successful: bool = True, reason: str = "") -> None:
        self.successful, self.reason = successful, reason


class _Param:
    def __init__(self, name: str, value: Any = None) -> None:
        self.name, self.value = name, value


class _Bool:
    def __init__(self) -> None:
        self.data = False


@pytest.fixture(autouse=True)
def _ros_stubs(monkeypatch):
    mods = {name: types.ModuleType(name) for name in (
        "rclpy", "rclpy.qos", "rclpy.parameter", "std_msgs", "std_msgs.msg", "rcl_interfaces", "rcl_interfaces.msg")}
    qos = mods["rclpy.qos"]
    qos.QoSProfile = lambda **kw: kw
    qos.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL="transient_local")
    qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST="keep_last")
    qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE="reliable")
    mods["rclpy.parameter"].Parameter = _Param
    mods["std_msgs.msg"].Bool = _Bool
    mods["rcl_interfaces.msg"].SetParametersResult = _Result
    for name, mod in mods.items():
        monkeypatch.setitem(sys.modules, name, mod)


class _Bus:
    def __init__(self) -> None:
        self.subs: List[Any] = []
        self.sent: List[bool] = []

    def publish(self, msg) -> None:
        self.sent.append(msg.data)
        for cb in list(self.subs):
            cb(msg)


class _Logger:
    def __init__(self) -> None:
        self.lines: List[str] = []

    def warning(self, text: str) -> None:
        self.lines.append(text)

    info = error = warning


class _Node:
    def __init__(self, bus: _Bus) -> None:
        self.bus = bus
        self.params: Dict[str, Any] = {}
        self.callbacks: List[Any] = []
        self.timers: List[Any] = []
        self.logger = _Logger()

    def declare_parameter(self, name, value):
        self.params[name] = value

    def add_on_set_parameters_callback(self, cb):
        self.callbacks.append(cb)

    def create_publisher(self, msg_type, topic, qos):
        assert (msg_type, topic) == (_Bool, M.TOPIC) and qos["durability"] == "transient_local"
        return self.bus

    def create_subscription(self, msg_type, topic, cb, qos):
        assert (msg_type, topic) == (_Bool, M.TOPIC) and qos["durability"] == "transient_local"
        self.bus.subs.append(cb)

    def create_timer(self, period, cb):
        self.timers.append(cb)

    def get_logger(self):
        return self.logger

    def set_parameters(self, params):
        results = []
        for p in params:
            res = [cb([p]) for cb in self.callbacks]
            ok = all(r.successful for r in res)
            if ok:
                self.params[p.name] = p.value
            results.append(_Result(ok, "; ".join(r.reason for r in res if r.reason)))
        return results


class _Clock:
    def __init__(self) -> None:
        self.t = 1000.0

    def __call__(self) -> float:
        return self.t


class _SpeakerNode(_Node):
    """Как ``speaker_id_node``: свой колбэк ``e2e_mode`` переключает БД и зовёт ``mode_changed``."""

    def __init__(self, bus: _Bus, clock: _Clock) -> None:
        super().__init__(bus)
        self.active = False
        self.declare_parameter(M.OWNER_PARAM, False)
        self.add_on_set_parameters_callback(self._own)
        self.owner = M.E2EModeOwner(self, lambda: self.active, clock=clock)

    def _own(self, params):
        for p in params:
            if p.name == M.OWNER_PARAM and bool(p.value) != self.active:
                self.active = bool(p.value)
                self.owner.mode_changed()
        return _Result(True)


class _MemoryNode(_Node):
    """Как ``mcp_server``: применяет режим своей БД памяти; ``refuse`` — бэкенд без e2e-изоляции."""

    def __init__(self, bus: _Bus, refuse: bool = False) -> None:
        super().__init__(bus)
        self.memory, self.refuse, self.wipes = "live", refuse, 0
        self.follower = M.E2EModeFollower(self, self._apply)

    def _apply(self, enabled: bool) -> bool:
        if self.refuse:
            return False
        if enabled:
            self.wipes += 1
        self.memory = "e2e" if enabled else "live"
        return True


def _robot(refuse=False):
    bus, clock = _Bus(), _Clock()
    memory = _MemoryNode(bus, refuse=refuse)
    speaker = _SpeakerNode(bus, clock)
    return speaker, memory, clock


def _harness_set(node, name, value):
    (res,) = node.set_parameters([_Param(name, value)])
    return res.successful


def test_one_switch_drives_every_follower():
    speaker, memory, _ = _robot()
    assert speaker.bus.sent == [False] and memory.memory == "live"

    assert _harness_set(speaker, M.OWNER_PARAM, True)
    assert memory.memory == "e2e" and memory.params[M.APPLIED_PARAM] is True

    assert _harness_set(speaker, M.OWNER_PARAM, False)
    assert memory.memory == "live" and memory.params[M.APPLIED_PARAM] is False


def test_follower_has_no_switch_of_its_own():
    """Дубль флага — это и есть баг #3904: у последователя нет записываемого режима."""
    _, memory, _ = _robot()
    assert M.OWNER_PARAM not in memory.params
    assert not _harness_set(memory, M.APPLIED_PARAM, True)
    assert memory.params[M.APPLIED_PARAM] is False and memory.memory == "live"


def test_late_follower_gets_mode_from_republish():
    bus, clock = _Bus(), _Clock()
    speaker = _SpeakerNode(bus, clock)
    _harness_set(speaker, M.OWNER_PARAM, True)
    memory = _MemoryNode(bus)  # стартовал позже (рестарт mcp_server): transient_local на zenoh не гарантирован
    assert memory.memory == "live"
    speaker.owner.publish()  # таймер переиздания
    assert memory.memory == "e2e" and memory.wipes == 1
    speaker.owner.publish()
    assert memory.wipes == 1, "повтор того же режима не стирает e2e-БД посреди акта"


def test_refused_apply_is_visible_to_harness():
    speaker, memory, _ = _robot(refuse=True)
    _harness_set(speaker, M.OWNER_PARAM, True)
    assert memory.params[M.APPLIED_PARAM] is False, "харнесс обязан увидеть, что память не переключилась"


def test_marathon_hold_survives_the_gap_between_acts_and_is_adopted():
    """Акт 2 уходит, акт 3 (наследует) подхватывает до таймера: режим не снимался ни на миг."""
    speaker, memory, clock = _robot()
    _harness_set(speaker, M.OWNER_PARAM, True)
    assert _harness_set(speaker, M.RELEASE_PARAM, 1800.0)  # trap акта 2: следующий наследует
    clock.t += 600
    for tick in speaker.timers:
        tick()
    assert speaker.active and memory.memory == "e2e"
    assert _harness_set(speaker, M.RELEASE_PARAM, 0.0)  # акт 3 подхватил
    clock.t += 7200
    for tick in speaker.timers:
        tick()
    assert speaker.active, "подхваченный режим не снимается таймером посреди акта"
    assert memory.wipes == 1


def test_abandoned_hold_returns_robot_to_live_dbs():
    """Марафон оборвался после акта 2: владелец сам возвращает робота на боевые БД."""
    speaker, memory, clock = _robot()
    _harness_set(speaker, M.OWNER_PARAM, True)
    _harness_set(speaker, M.RELEASE_PARAM, 1800.0)
    clock.t += 1801
    for tick in speaker.timers:
        tick()
    assert not speaker.active and memory.memory == "live"
    assert speaker.params[M.OWNER_PARAM] is False and speaker.params[M.RELEASE_PARAM] == 0.0
    assert any("таймер" in line for line in speaker.logger.lines)


def test_new_switch_disarms_stale_timer():
    speaker, _, clock = _robot()
    _harness_set(speaker, M.OWNER_PARAM, True)
    _harness_set(speaker, M.RELEASE_PARAM, 60.0)
    _harness_set(speaker, M.OWNER_PARAM, False)
    _harness_set(speaker, M.OWNER_PARAM, True)  # новый акт включил свежий режим
    clock.t += 3600
    for tick in speaker.timers:
        tick()
    assert speaker.active


@pytest.mark.parametrize("value", [-1.0, float("nan"), float("inf"), "soon"])
def test_release_value_is_validated(value):
    speaker, _, _ = _robot()
    _harness_set(speaker, M.OWNER_PARAM, True)
    assert not _harness_set(speaker, M.RELEASE_PARAM, value)
    assert not speaker.owner.release_armed


def test_release_refused_when_mode_is_off():
    speaker, _, _ = _robot()
    assert not _harness_set(speaker, M.RELEASE_PARAM, 1800.0)


def test_release_deadline_is_pure():
    clock = _Clock()
    d = M.ReleaseDeadline(clock)
    assert not d.armed and not d.due()
    d.arm(10)
    assert d.armed and not d.due()
    clock.t += 10
    assert d.due()
    d.arm(0)
    assert not d.armed
