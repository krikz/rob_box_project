"""issue #1992 observability: QuestBridge.publish_quest_wake_audio раньше не
логировал ни единой публикации в /audio/quest_wake — на роботе ``docker
logs rob-box-quest`` показывал ту же тишину что при живом потоке, что и при
клиенте, который вообще ничего не шлёт (симптом отчёта: владелец сказал
«ТАРС» в шлем несколько раз, в логах ни строки по wake/VOICE_AUDIO).

Требует rclpy/geometry_msgs/audio_common_msgs (Docker image) — на dev-env
без ROS пропускается через importorskip, как остальные тесты QuestBridge
(см. test/unit/test_quest_bridge.py).
"""

import time

import pytest


class _MockPublisher:
    """Имитация rclpy.Publisher.publish(msg) — собирает все сообщения."""

    def __init__(self) -> None:
        self.published: list = []

    def publish(self, msg) -> None:
        self.published.append(msg)


class _Logger:
    def __init__(self) -> None:
        self.infos: list[str] = []
        self.warnings: list[str] = []

    def info(self, msg: str) -> None:
        self.infos.append(msg)

    def warning(self, msg: str) -> None:
        self.warnings.append(msg)


class _MockNode:
    def __init__(self) -> None:
        self.logger = _Logger()

    def get_logger(self):
        return self.logger


def _make_bridge():
    pytest.importorskip(
        "geometry_msgs", reason="QuestBridge требует rclpy/geometry_msgs (только в Docker image)"
    )
    from rob_box_quest.quest_node import QuestBridge

    node = _MockNode()
    quest_wake_pub = _MockPublisher()
    bridge = QuestBridge(
        node=node,
        cmd_vel_quest_pub=_MockPublisher(),
        cmd_vel_emergency_pub=_MockPublisher(),
        quest_wake_pub=quest_wake_pub,
    )
    return bridge, quest_wake_pub, node


def test_first_wake_publish_logs_immediately():
    bridge, quest_wake_pub, node = _make_bridge()
    bridge.publish_quest_wake_audio(b"\x00\x01" * 160)
    assert len(quest_wake_pub.published) == 1
    assert any(
        "first packet" in m and "/audio/quest_wake" in m for m in node.logger.infos
    )


def test_second_publish_within_window_does_not_log_again(monkeypatch):
    bridge, quest_wake_pub, node = _make_bridge()
    from rob_box_quest import quest_node as quest_node_mod
    now = [5000.0]
    monkeypatch.setattr(quest_node_mod.time, "monotonic", lambda: now[0])
    bridge.publish_quest_wake_audio(b"\x00\x01" * 160)
    node.logger.infos.clear()
    now[0] += 0.02  # один чанк 20мс позже
    bridge.publish_quest_wake_audio(b"\x00\x01" * 160)
    assert node.logger.infos == []
    assert len(quest_wake_pub.published) == 2  # публикация в ROS не блокируется логом


def test_summary_logged_after_interval_elapses(monkeypatch):
    bridge, quest_wake_pub, node = _make_bridge()
    from rob_box_quest import quest_node as quest_node_mod

    now = [5000.0]
    monkeypatch.setattr(quest_node_mod.time, "monotonic", lambda: now[0])
    bridge.publish_quest_wake_audio(b"\x00\x01" * 160)  # first packet, count=1
    node.logger.infos.clear()
    for _ in range(9):
        now[0] += 0.02
        bridge.publish_quest_wake_audio(b"\x00\x01" * 160)  # count=2..10
    assert node.logger.infos == []
    now[0] += quest_node_mod.WAKE_AUDIO_LOG_INTERVAL_S
    bridge.publish_quest_wake_audio(b"\x00\x01" * 160)  # count=11 -> summary
    summaries = [m for m in node.logger.infos if "publish summary" in m]
    assert len(summaries) == 1
    assert "10 packets" in summaries[0]
    assert "total 11 packets" in summaries[0]


def test_none_publisher_stays_silent_no_op():
    """quest_wake_pub=None (unit-тесты моста без ROS) — no-op, без AttributeError
    на логгере: publish_quest_wake_audio должен вернуться до любого учёта."""
    pytest.importorskip(
        "geometry_msgs", reason="QuestBridge требует rclpy/geometry_msgs (только в Docker image)"
    )
    from rob_box_quest.quest_node import QuestBridge

    node = _MockNode()
    bridge = QuestBridge(
        node=node,
        cmd_vel_quest_pub=_MockPublisher(),
        cmd_vel_emergency_pub=_MockPublisher(),
        quest_wake_pub=None,
    )
    bridge.publish_quest_wake_audio(b"\x00\x01")
    assert node.logger.infos == []
