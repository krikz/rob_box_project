"""Unit-тесты QuestBridge с mock-publishers (без rclpy).

Цель: убедиться что publish_quest / publish_emergency / feed_client_alive /
emergency_stop правильно роутят в publishers и обновляют TeleopController
+ Watchdog.

Эти тесты требуют rclpy/geometry_msgs (только в Docker image). На dev-env
без ROS — skip через importorskip. На роботе (Phase 1.7 e2e) — реальный
test.
"""

import msgpack
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


def _pcm_chunk(*samples: int) -> bytes:
    """int16 LE PCM-чанк из заданных сэмплов."""
    out = bytearray()
    for s in samples:
        s &= 0xFFFF
        out.append(s & 0xFF)
        out.append((s >> 8) & 0xFF)
    return bytes(out)


def _pcm20ms(value: int) -> bytes:
    """20 мс @ 16 kHz = 320 int16-сэмплов одного значения (реальный чанк webxr_client)."""
    return _pcm_chunk(*([value] * 320))


def test_voice_radio_mode_streams_to_voice_in():
    bridge, voice_in, _tts, _sound, stt_in, _svm = _make_voice_bridge()
    bridge.publish_voice_audio(_pcm_chunk(4000, 4000))
    assert len(voice_in.published) == 1
    assert voice_in.published[0].data == [0xA0, 0x0F, 0xA0, 0x0F]
    assert len(stt_in.published) == 0


def test_voice_robot_mode_flushes_on_silence():
    """EOU: фраза уходит в STT по тишине, пока грип ещё зажат (не ждём release)."""
    bridge, voice_in, tts_control, sound_stop, stt_in, _svm = _make_voice_bridge()
    bridge.publish_voice_robot_start()
    assert len(tts_control.published) == 1  # barge-in STOP TTS
    assert len(sound_stop.published) == 1  # barge-in STOP sound
    # Речь буферизуется, НЕ идёт в /avatar/voice_in и НЕ в STT.
    bridge.publish_voice_audio(_pcm20ms(4000))
    bridge.publish_voice_audio(_pcm20ms(4000))
    assert len(voice_in.published) == 0
    assert len(stt_in.published) == 0
    # 15 × 20 мс = 300 мс тишины → конец фразы → флаш в /audio/quest_in.
    for _ in range(15):
        bridge.publish_voice_audio(_pcm20ms(0))
    assert len(stt_in.published) == 1
    assert len(stt_in.published[0].data) > 0
    assert len(voice_in.published) == 0


def test_voice_robot_leading_silence_ignored():
    """Ведущая тишина (до речи) не публикует пустой буфер."""
    bridge, _voice_in, _tts, _sound, stt_in, _svm = _make_voice_bridge()
    bridge.publish_voice_robot_start()
    for _ in range(20):
        bridge.publish_voice_audio(_pcm20ms(0))
    assert len(stt_in.published) == 0


def test_voice_robot_stop_flushes_remainder():
    """Release без завершающей тишины → остаток фразы уходит в STT."""
    bridge, _voice_in, _tts, _sound, stt_in, _svm = _make_voice_bridge()
    bridge.publish_voice_robot_start()
    bridge.publish_voice_audio(_pcm20ms(4000))
    bridge.publish_voice_audio(_pcm20ms(-32000))
    assert len(stt_in.published) == 0
    bridge.publish_voice_robot_stop()
    assert len(stt_in.published) == 1
    assert len(stt_in.published[0].data) == 640 * 2


def test_voice_robot_stop_with_empty_buffer_publishes_nothing():
    bridge, voice_in, _tts, _sound, stt_in, _svm = _make_voice_bridge()
    bridge.publish_voice_robot_start()
    bridge.publish_voice_robot_stop()  # пусто → no-op в STT
    assert len(stt_in.published) == 0
    assert len(voice_in.published) == 0


def test_chunk_is_silent_threshold():
    pytest.importorskip("geometry_msgs", reason="QuestBridge требует rclpy/geometry_msgs (только в Docker image)")
    from rob_box_quest.quest_node import _chunk_is_silent

    assert _chunk_is_silent(_pcm_chunk(0, 0, 0)) is True
    assert _chunk_is_silent(_pcm_chunk(499, -499)) is True
    assert _chunk_is_silent(_pcm_chunk(500, 0)) is False
    assert _chunk_is_silent(_pcm_chunk(-500)) is False
    assert _chunk_is_silent(b"") is True
    assert _chunk_is_silent(b"\x01") is True  # нечётная длина → тишина


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


# --- voice_state (AV-20, 0x1202) --------------------------------------------
#
# Тесты callback'а ``QuestNode._on_dialogue_state``. Чтобы не поднимать
# rclpy/init-спиннер, инстанцируем QuestNode через ``__new__`` (без
# ``__init__``) и руками проставляем минимальный набор атрибутов. Тест
# проверяет, что нормализация + encode + publish_frame связаны в одну
# цепочку и любой переход FSM превращается в msgpack-фрейм с правильным
# state/detail.
#
# ``_MockNode`` и ``_Logger`` уже определены в начале модуля — переиспользуем.


def _make_node_only_for_callback_test():
    """Вернуть (node, capture_ws_frames, RosString) для изолированного теста callback'а.

    Используем ``QuestNode.__new__`` чтобы пропустить ``__init__`` (там
    create_publisher/create_subscription — нужен rclpy.init). Это
    стандартный приём для unit-теста ROS-нод без поднятия DDS.

    Тест под ``pytest.importorskip("rclpy")`` потому что ``QuestNode.__new__``
    всё равно тянет в MRO базовый ``rclpy.node.Node``, у которого
    метакласс проверяет наличие rclpy. Если rclpy недоступен — пропускаем
    (CI имеет).
    """
    pytest.importorskip("rclpy", reason="QuestNode callback test requires rclpy")
    from std_msgs.msg import String as RosString  # noqa: WPS433
    from rob_box_quest.quest_node import QuestNode  # noqa: WPS433

    # ``__new__`` обходит ``__init__``, но всё равно пройдёт через
    # метакласс Node'а. На dev-env без rclpy это падает — importorskip
    # выше это уже отфильтровал.
    node = QuestNode.__new__(QuestNode)
    node._last_voice_state = {"state": "idle", "detail": None}
    node.warnings = []

    class _LoggerCapture:
        def warning(self_inner, msg: str) -> None:
            node.warnings.append(msg)

        def debug(self_inner, msg: str) -> None:
            pass

        def info(self_inner, msg: str) -> None:
            pass

    node.get_logger = lambda: _LoggerCapture()  # type: ignore[method-assign]

    captured: list[tuple[str, bytes]] = []

    class _BridgeSpy:
        def publish_frame(self_inner, ui_name: str, payload: bytes) -> None:
            captured.append((ui_name, payload))

    node.bridge = _BridgeSpy()  # type: ignore[assignment]
    return node, captured, RosString


def test_dialogue_state_callback_emits_idle_when_idle():
    node, captured, RosString = _make_node_only_for_callback_test()
    msg = RosString()
    msg.data = "IDLE"
    node._on_dialogue_state(msg)
    assert len(captured) == 1
    ui_name, payload = captured[0]
    assert ui_name == "voice_state"
    decoded = msgpack.unpackb(payload, raw=False)
    assert decoded["state"] == "idle"
    assert "detail" not in decoded
    assert isinstance(decoded["ts_ms"], int)


def test_dialogue_state_callback_emits_speaking_for_dialogue():
    node, captured, RosString = _make_node_only_for_callback_test()
    msg = RosString()
    msg.data = "DIALOGUE"
    node._on_dialogue_state(msg)
    assert captured[0][0] == "voice_state"
    decoded = msgpack.unpackb(captured[0][1], raw=False)
    assert decoded["state"] == "speaking"


def test_dialogue_state_callback_marks_silenced():
    node, captured, RosString = _make_node_only_for_callback_test()
    msg = RosString()
    msg.data = "SILENCED"
    node._on_dialogue_state(msg)
    decoded = msgpack.unpackb(captured[0][1], raw=False)
    assert decoded["state"] == "idle"
    assert decoded["detail"] == "silenced"


def test_dialogue_state_callback_unknown_falls_back_silently_to_ws():
    """Неизвестная строка → WARNING + state=idle (НО фрейм всё равно шлём).

    Это контракт: «не молчим, но и не крашим». Клиент увидит ``idle`` и
    WARNING появится в логах супервизора.
    """
    node, captured, RosString = _make_node_only_for_callback_test()
    msg = RosString()
    msg.data = "WEIRD_NEW_STATE"
    node._on_dialogue_state(msg)
    assert len(captured) == 1
    decoded = msgpack.unpackb(captured[0][1], raw=False)
    assert decoded["state"] == "idle"
    assert "detail" not in decoded


def test_dialogue_state_callback_updates_last_voice_state_cache():
    """``_last_voice_state`` обновляется при каждом callback'е."""
    node, captured, RosString = _make_node_only_for_callback_test()
    msg = RosString()
    msg.data = "LISTENING"
    node._on_dialogue_state(msg)
    assert node._last_voice_state["state"] == "listening"
    msg.data = "DIALOGUE"
    node._on_dialogue_state(msg)
    assert node._last_voice_state["state"] == "speaking"
