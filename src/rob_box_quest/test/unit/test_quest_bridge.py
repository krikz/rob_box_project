"""Unit-тесты QuestBridge с mock-publishers (без rclpy).

Цель: убедиться что publish_quest / publish_emergency / feed_client_alive /
emergency_stop правильно роутят в publishers и обновляют TeleopController
+ Watchdog.

Эти тесты требуют rclpy/geometry_msgs (только в Docker image). На dev-env
без ROS — skip через importorskip. На роботе (Phase 1.7 e2e) — реальный
test.
"""

import pytest

from rob_box_quest.core.safety import WATCHDOG_TIMEOUT_S
from rob_box_quest.core.teleop import (
    DEADMAN_TIMEOUT_S,
    MAX_ANGULAR_RAD_S,
    MAX_LINEAR_M_S,
)


class _MockPublisher:
    """Имитация rclpy.Publisher.publish(msg) — собирает все сообщения."""

    def __init__(self) -> None:
        self.published: list[tuple] = []

    def publish(self, msg) -> None:
        self.published.append(msg)


class _MockNode:
    """Минимум для QuestBridge — нужен только get_logger().warning."""

    def __init__(self) -> None:
        self.warnings: list[str] = []

    def get_logger(self):
        return _Logger(self.warnings)


class _Logger:
    def __init__(self, sink: list[str]) -> None:
        self._sink = sink

    def warning(self, msg: str) -> None:
        self._sink.append(msg)


def _make_bridge():
    """Lazy import — quest_node тянет rclpy + geometry_msgs; для теста моста
    нужен только QuestBridge + mock publishers.

    Если geometry_msgs недоступен (типичный dev-env на Windows без
    colcon build) — пропускаем, мост покрывается e2e-тестом Phase 1.7.
    """
    pytest.importorskip("geometry_msgs", reason="QuestBridge требует rclpy/geometry_msgs (только в Docker image)")
    from rob_box_quest.quest_node import QuestBridge

    node = _MockNode()
    pub_quest = _MockPublisher()
    pub_emergency = _MockPublisher()
    bridge = QuestBridge(
        node=node,
        cmd_vel_quest_pub=pub_quest,
        cmd_vel_emergency_pub=pub_emergency,
    )
    return bridge, pub_quest, pub_emergency, node


def test_publish_quest_publishes_twist():
    bridge, pub_quest, pub_emergency, _ = _make_bridge()
    bridge.publish_quest(0.5, 0.2)
    assert len(pub_quest.published) == 1
    msg = pub_quest.published[0]
    assert abs(msg.linear.x - 0.5) < 1e-9
    assert abs(msg.angular.z - 0.2) < 1e-9
    assert len(pub_emergency.published) == 0


def test_publish_quest_clamps_values():
    bridge, pub_quest, _, _ = _make_bridge()
    bridge.publish_quest(99.0, -99.0)
    msg = pub_quest.published[0]
    assert msg.linear.x == MAX_LINEAR_M_S
    assert msg.angular.z == -MAX_ANGULAR_RAD_S


def test_publish_emergency_publishes_to_emergency_topic_and_logs():
    bridge, pub_quest, pub_emergency, node = _make_bridge()
    bridge.publish_emergency()
    assert len(pub_emergency.published) == 1
    # log warning emitted
    assert any("EMERGENCY" in w for w in node.warnings)


def test_feed_client_alive_resets_watchdog():
    bridge, _, _, _ = _make_bridge()
    # Свежий watchdog → не tripped.
    import time as _t

    t0 = _t.monotonic()
    assert bridge.watchdog_check(t0) is False
    # Перематываем на час вперёд → tripped.
    assert bridge.watchdog_check(t0 + 3600.0) is True
    # Feed → снова не tripped.
    bridge.feed_client_alive()
    assert bridge.watchdog_check(_t.monotonic() + 1.0) is False


def test_watchdog_check_uses_session_timeout():
    """Watchdog настроен на SESSION_WATCHDOG_TIMEOUT_S = 0.6 с."""
    bridge, _, _, _ = _make_bridge()
    assert bridge._watchdog.timeout_s == WATCHDOG_TIMEOUT_S


def test_emergency_stop_locks_teleop_and_zeroes_output():
    bridge, pub_quest, _, _ = _make_bridge()
    bridge.publish_quest(0.5, 0.2)
    bridge.emergency_stop()
    # consume после emergency → None, публикуется zero.
    bridge.publish_quest(0.5, 0.2)
    last = pub_quest.published[-1]
    assert last.linear.x == 0.0
    assert last.angular.z == 0.0


def test_tick_publishes_zero_after_deadman_timeout():
    import time as _t

    bridge, pub_quest, _, _ = _make_bridge()
    t0 = _t.monotonic()
    bridge.publish_quest(0.5, 0.1)  # sets last_input = t0
    # Свежий tick — публикуем тот же twist.
    bridge.tick_publish(t0 + DEADMAN_TIMEOUT_S - 0.05)
    msg = pub_quest.published[-1]
    assert msg.linear.x == 0.5
    # Через > DEADMAN_TIMEOUT_S — публикуем zero.
    bridge.tick_publish(t0 + DEADMAN_TIMEOUT_S + 0.1)
    msg = pub_quest.published[-1]
    assert msg.linear.x == 0.0
    assert msg.angular.z == 0.0


def test_reset_clears_emergency_and_watchdog():
    import time as _t

    bridge, _, _, _ = _make_bridge()
    bridge.publish_quest(0.5, 0.2)
    bridge.emergency_stop()
    bridge.reset()
    # Teleop снова работает.
    bridge.publish_quest(0.3, 0.1)
    # Watchdog тоже сброшен (свежий tick).
    assert bridge.watchdog_check(_t.monotonic()) is False
