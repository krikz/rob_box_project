"""``shared_publisher`` — один publisher на (нода, тип, топик, QoS).

Issue #3108: живой граф (run 36422274238, ``ros2 topic info --verbose``)
показал, что нода ``mcp_server`` держит несколько publisher'ов на одном
топике:

* ``/voice/generated_music/state`` — 3 (mcp_server, StopMusicTool,
  PlayGeneratedMusicTool);
* ``/voice/animation/request``     — 2 (SpeakTextTool, PlayAnimationTool);
* ``/voice/sound/stop``            — 2 (mcp_server, StopMusicTool);
* ``/voice/tts/set_provider``      — 2 (SetVoiceTool, SetTtsProviderTool).

Каждый тул звал ``node.create_publisher`` сам. Теперь все они идут через
``base.shared_publisher``, который кэширует publisher на ноде.
"""

import importlib.util
import re
import sys
import types
from pathlib import Path

import pytest

_PKG = Path(__file__).resolve().parents[1] / "rob_box_mcp_tools"

# ``base.py`` грузится по пути — так же, как в test_wait_future.py:
# test_mcp_server.py подменяет подмодули ``rob_box_mcp_tools`` в
# ``sys.modules``, обычный импорт зависит от порядка сборки.
_spec = importlib.util.spec_from_file_location("rob_box_mcp_tools_base_for_shared_publisher", str(_PKG / "base.py"))
assert _spec is not None and _spec.loader is not None
_base = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = _base
_spec.loader.exec_module(_base)

shared_publisher = _base.shared_publisher


class _Msg:
    pass


class _OtherMsg:
    pass


class _FakeNode:
    """Нода, которая считает вызовы ``create_publisher``."""

    def __init__(self):
        self.calls = []

    def create_publisher(self, msg_type, topic, qos):
        self.calls.append((msg_type, topic, qos))
        return object()  # уникальный объект на каждый вызов


class _FakeQoSProfile:
    """Стенд-ин ``rclpy.qos.QoSProfile`` с тем же контрактом равенства:
    ``QoSProfile(depth=N)`` — RELIABLE / KEEP_LAST / VOLATILE, профили
    равны, если равны все поля."""

    def __init__(self, depth, reliability="reliable", history="keep_last", durability="volatile"):
        self.depth = depth
        self.reliability = reliability
        self.history = history
        self.durability = durability

    def __eq__(self, other):
        return isinstance(other, _FakeQoSProfile) and vars(self) == vars(other)

    __hash__ = None


@pytest.fixture
def fake_rclpy_qos(monkeypatch):
    rclpy = types.ModuleType("rclpy")
    qos = types.ModuleType("rclpy.qos")
    qos.QoSProfile = _FakeQoSProfile
    rclpy.qos = qos
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.qos", qos)
    return qos


@pytest.mark.unit
def test_same_topic_type_qos_returns_one_publisher():
    node = _FakeNode()

    first = shared_publisher(node, _Msg, "/voice/animation/request", 10)
    second = shared_publisher(node, _Msg, "/voice/animation/request", 10)
    third = shared_publisher(node, _Msg, "/voice/animation/request", 10)

    assert first is second is third
    assert node.calls == [(_Msg, "/voice/animation/request", 10)]


@pytest.mark.unit
def test_different_topics_get_different_publishers():
    node = _FakeNode()

    a = shared_publisher(node, _Msg, "/voice/sound/stop", 10)
    b = shared_publisher(node, _Msg, "/voice/generated_music/state", 10)

    assert a is not b
    assert len(node.calls) == 2


@pytest.mark.unit
def test_different_msg_type_is_not_merged():
    node = _FakeNode()

    a = shared_publisher(node, _Msg, "/t", 10)
    b = shared_publisher(node, _OtherMsg, "/t", 10)

    assert a is not b
    assert len(node.calls) == 2


@pytest.mark.unit
def test_different_qos_is_not_silently_merged():
    """Разный QoS — это разный контракт; склеивать молча нельзя."""
    node = _FakeNode()

    a = shared_publisher(node, _Msg, "/t", 10)
    b = shared_publisher(node, _Msg, "/t", 1)

    assert a is not b
    assert len(node.calls) == 2
    # Повторный запрос каждого варианта возвращает свой кэш.
    assert shared_publisher(node, _Msg, "/t", 10) is a
    assert shared_publisher(node, _Msg, "/t", 1) is b
    assert len(node.calls) == 2


@pytest.mark.unit
def test_depth_int_and_equivalent_profile_are_merged(fake_rclpy_qos):
    """mcp_server создаёт ``/voice/sound/stop`` с явным
    ``QoSProfile(RELIABLE, KEEP_LAST, depth=10)``, а StopMusicTool — с
    ``10``. rclpy превращает ``10`` в ``QoSProfile(depth=10)``; это тот же
    профиль, значит и publisher один."""
    node = _FakeNode()
    explicit = _FakeQoSProfile(depth=10)

    a = shared_publisher(node, _Msg, "/voice/sound/stop", explicit)
    b = shared_publisher(node, _Msg, "/voice/sound/stop", 10)

    assert a is b
    assert len(node.calls) == 1


@pytest.mark.unit
def test_depth_int_and_different_profile_are_not_merged(fake_rclpy_qos):
    node = _FakeNode()
    latched = _FakeQoSProfile(depth=10, durability="transient_local")

    a = shared_publisher(node, _Msg, "/t", latched)
    b = shared_publisher(node, _Msg, "/t", 10)

    assert a is not b
    assert len(node.calls) == 2


@pytest.mark.unit
def test_cache_is_per_node():
    n1, n2 = _FakeNode(), _FakeNode()

    a = shared_publisher(n1, _Msg, "/t", 10)
    b = shared_publisher(n2, _Msg, "/t", 10)

    assert a is not b
    assert len(n1.calls) == 1 and len(n2.calls) == 1


# ---------------------------------------------------------------------------
# Регресс-гард по исходникам: ни одного прямого ``create_publisher`` на
# четыре топика из #3108 в пакете. Статический, потому что импорт
# ``rob_box_mcp_tools.tools`` тянет nav2_msgs и без ROS не собирается.
# ---------------------------------------------------------------------------

_ISSUE_3108_TOPICS = (
    "/voice/generated_music/state",
    "/voice/animation/request",
    "/voice/sound/stop",
    "/voice/tts/set_provider",
)

_CREATE_PUBLISHER_CALL = re.compile(r"\.create_publisher\(\s*[^,]+,\s*\"([^\"]+)\"", re.S)
_SHARED_PUBLISHER_CALL = re.compile(r"shared_publisher\(\s*[^,]+,\s*[^,]+,\s*\"([^\"]+)\"", re.S)


def _package_sources():
    return sorted(p for p in _PKG.rglob("*.py") if "test" not in p.parts)


@pytest.mark.unit
@pytest.mark.parametrize("topic", _ISSUE_3108_TOPICS)
def test_no_direct_create_publisher_for_issue_3108_topics(topic):
    offenders = []
    for path in _package_sources():
        for m in _CREATE_PUBLISHER_CALL.finditer(path.read_text(encoding="utf-8")):
            if m.group(1) == topic:
                offenders.append(str(path.relative_to(_PKG)))
    assert offenders == [], f"{topic}: прямой create_publisher в {offenders} — используйте shared_publisher"


@pytest.mark.unit
@pytest.mark.parametrize("topic", _ISSUE_3108_TOPICS)
def test_issue_3108_topics_go_through_shared_publisher(topic):
    users = []
    for path in _package_sources():
        for m in _SHARED_PUBLISHER_CALL.finditer(path.read_text(encoding="utf-8")):
            if m.group(1) == topic:
                users.append(path.name)
    assert len(users) >= 2, f"{topic}: ожидалось >= 2 пользователей shared_publisher, нашли {users}"
