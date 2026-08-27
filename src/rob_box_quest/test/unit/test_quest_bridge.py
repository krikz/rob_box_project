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
    """DISARMED по умолчанию → нет trip без feed().
    После feed() → ARMED, в окне timeout — не tripped, после — tripped."""
    bridge, _, _, _ = _make_bridge()
    import time as _t

    t0 = _t.monotonic()
    # Без feed() — DISARMED, не триггерится (анти-спам на старте ноды).
    assert bridge.watchdog_check(t0) is False
    assert bridge.watchdog_check(t0 + 3600.0) is False
    # Feed → ARMED.
    bridge.feed_client_alive()
    assert bridge._watchdog.armed is True
    # Свежий — не tripped.
    assert bridge.watchdog_check(_t.monotonic() + 1.0) is False
    # После timeout — tripped.
    assert bridge.watchdog_check(_t.monotonic() + 3600.0) is True


def test_publish_emergency_is_edge_triggered():
    """publish_emergency() публикует ОДИН раз, повторные вызовы — no-op."""
    bridge, pub_quest, pub_emergency, node = _make_bridge()
    bridge.publish_emergency()
    assert len(pub_emergency.published) == 1
    first_warning_count = sum(1 for w in node.warnings if "EMERGENCY" in w)
    # 10 повторных вызовов — должно остаться ровно 1 warning и 1 publish.
    for _ in range(10):
        bridge.publish_emergency()
    assert len(pub_emergency.published) == 1, "спам в cmd_vel_emergency!"
    final_warning_count = sum(1 for w in node.warnings if "EMERGENCY" in w)
    assert final_warning_count == first_warning_count, "спам WARNING в логах!"


def test_feed_client_alive_clears_emergency_edge():
    """feed_client_alive() снимает _emergency_published → можно снова
    публиковать emergency если клиент опять пропал."""
    bridge, _, pub_emergency, _ = _make_bridge()
    bridge.publish_emergency()
    assert len(pub_emergency.published) == 1
    bridge.publish_emergency()
    assert len(pub_emergency.published) == 1, "не должно быть второго publish до feed"
    # Клиент вернулся → feed → edge снят.
    bridge.feed_client_alive()
    # Снова публикуем — должно быть уже 2.
    bridge.publish_emergency()
    assert len(pub_emergency.published) == 2


def test_watchdog_consume_trip_does_not_spam():
    """watchdog_consume_trip() возвращает True ОДИН раз, потом False —
    это и есть анти-спам для timer 10 Гц."""
    bridge, _, _, _ = _make_bridge()
    import time as _t

    bridge.feed_client_alive()  # ARMED
    later = _t.monotonic() + 3600.0  # гарантированно tripped
    trips = 0
    for _ in range(100):  # 100 тиков timer'а
        if bridge.watchdog_consume_trip(later):
            trips += 1
        later += 0.1
    assert trips == 1, f"должен быть ровно 1 trip, получили {trips}"


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


# --- Робот-голос (P7): буфер PCM → /audio/quest_in -----------------------


def _make_voice_bridge():
    """QuestBridge с mock-паблишерами голосового пути (radio + robot voice)."""
    pytest.importorskip("geometry_msgs", reason="QuestBridge требует rclpy/geometry_msgs (только в Docker image)")
    from rob_box_quest.quest_node import QuestBridge

    node = _MockNode()
    voice_in = _MockPublisher()
    tts_control = _MockPublisher()
    sound_stop = _MockPublisher()
    stt_in = _MockPublisher()
    set_voice_mode = _MockPublisher()
    bridge = QuestBridge(
        node=node,
        cmd_vel_quest_pub=_MockPublisher(),
        cmd_vel_emergency_pub=_MockPublisher(),
        voice_in_pub=voice_in,
        tts_control_pub=tts_control,
        sound_stop_pub=sound_stop,
        stt_in_pub=stt_in,
        set_voice_mode_pub=set_voice_mode,
    )
    return bridge, voice_in, tts_control, sound_stop, stt_in, set_voice_mode


def test_voice_radio_mode_streams_to_voice_in():
    bridge, voice_in, _tts, _sound, stt_in, _svm = _make_voice_bridge()
    bridge.publish_voice_audio(b"\x00\x00")
    assert len(voice_in.published) == 1
    assert voice_in.published[0].data == [0, 0]
    assert len(stt_in.published) == 0


def test_voice_robot_mode_buffers_and_flushes_to_stt():
    bridge, voice_in, tts_control, sound_stop, stt_in, _svm = _make_voice_bridge()
    # PTT start (robot) → barge-in (STOP TTS + sound) + buffer reset.
    bridge.publish_voice_robot_start()
    assert len(tts_control.published) == 1
    assert len(sound_stop.published) == 1
    # PCM буферизуется, НЕ идёт в /avatar/voice_in.
    bridge.publish_voice_audio(b"\x01\x00")
    bridge.publish_voice_audio(b"\x02\x00")
    assert len(voice_in.published) == 0
    assert len(stt_in.published) == 0
    # PTT stop (robot) → конкатенация в /audio/quest_in + возврат в radio.
    bridge.publish_voice_robot_stop()
    assert len(stt_in.published) == 1
    assert stt_in.published[0].data == [1, 0, 2, 0]
    # После stop — снова radio: следующий чанк стримится в /avatar/voice_in.
    bridge.publish_voice_audio(b"\x03\x00")
    assert len(voice_in.published) == 1
    assert len(stt_in.published) == 1


def test_voice_robot_stop_with_empty_buffer_publishes_nothing():
    bridge, voice_in, _tts, _sound, stt_in, _svm = _make_voice_bridge()
    bridge.publish_voice_robot_start()
    bridge.publish_voice_robot_stop()  # пусто → no-op в STT
    assert len(stt_in.published) == 0
    assert len(voice_in.published) == 0


def test_set_voice_mode_maps_wire_to_param_and_publishes():
    bridge, _voice_in, _tts, _sound, _stt_in, set_voice_mode = _make_voice_bridge()
    bridge.set_voice_mode("ttts_proxy")
    assert len(set_voice_mode.published) == 1
    assert set_voice_mode.published[0].data == "quest_ttts"


def test_set_voice_mode_off_maps_to_respeaker():
    bridge, _voice_in, _tts, _sound, _stt_in, set_voice_mode = _make_voice_bridge()
    bridge.set_voice_mode("off")
    assert set_voice_mode.published[0].data == "respeaker"


def test_set_voice_mode_unknown_ignored():
    bridge, _voice_in, _tts, _sound, _stt_in, set_voice_mode = _make_voice_bridge()
    bridge.set_voice_mode("bogus_mode")
    assert len(set_voice_mode.published) == 0
