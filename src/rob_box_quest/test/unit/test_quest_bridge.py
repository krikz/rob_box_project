"""Unit-тесты QuestBridge с mock-publishers (без rclpy).

Цель: убедиться что publish_quest / publish_emergency / feed_client_alive /
emergency_stop правильно роутят в publishers и обновляют TeleopController
+ Watchdog.

Эти тесты требуют rclpy/geometry_msgs (только в Docker image). На dev-env
без ROS — skip через importorskip. На роботе (Phase 1.7 e2e) — реальный
test.
"""

import msgpack
import json
import time

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
    """Минимум для QuestBridge — нужен get_logger().warning + .info."""

    def __init__(self) -> None:
        self.warnings: list[str] = []
        # ADR-0073 / issue #2138.C: ловим .info() для регрессии, чтобы
        # зафиксировать «publishing…» / «published…» в bridge.set_voice.
        self.infos: list[str] = []

    def get_logger(self):
        return _Logger(self.warnings, self.infos)


class _Logger:
    def __init__(self, sink_warn: list[str], sink_info: list[str]) -> None:
        self._sink_warn = sink_warn
        self._sink_info = sink_info

    def warning(self, msg: str) -> None:
        self._sink_warn.append(msg)

    def info(self, msg: str) -> None:
        # None означает «sink недоступен» — даунгрейд до no-op, чтобы старые
        # тесты, которые создают _Logger с одним аргументом, не падали (см.
        # _LoggerCapture в _make_node_only_for_callback_test ниже).
        if self._sink_info is not None:
            self._sink_info.append(msg)


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


# --- Wake-канал (stream_id=2): буфер PCM → /audio/quest_wake (issue #1992) ---


def _make_wake_bridge():
    """QuestBridge с mock-паблишером wake-канала (/audio/quest_wake)."""
    pytest.importorskip("geometry_msgs", reason="QuestBridge требует rclpy/geometry_msgs (только в Docker image)")
    from rob_box_quest.quest_node import QuestBridge

    node = _MockNode()
    quest_wake = _MockPublisher()
    bridge = QuestBridge(
        node=node,
        cmd_vel_quest_pub=_MockPublisher(),
        cmd_vel_emergency_pub=_MockPublisher(),
        quest_wake_pub=quest_wake,
    )
    return bridge, quest_wake


def test_publish_quest_wake_audio_publishes_to_quest_wake_pub():
    """publish_quest_wake_audio публикует ровно тот payload, что пришёл,
    одним AudioData — без буферизации/EOU-логики (в отличие от robot_voice
    на PTT-канале: клиент уже отфильтровал тишину через RMS VAD)."""
    bridge, quest_wake = _make_wake_bridge()
    payload = _pcm_chunk(4000, 4000)
    bridge.publish_quest_wake_audio(payload)
    assert len(quest_wake.published) == 1
    # rclpy AudioData.data — array.array('B', ...) под настоящим сообщением
    # (uint8[] сериализуется так), list() приводит к сравнимому виду.
    assert list(quest_wake.published[0].data) == [0xA0, 0x0F, 0xA0, 0x0F]


def test_publish_quest_wake_audio_multiple_chunks_publish_individually():
    bridge, quest_wake = _make_wake_bridge()
    bridge.publish_quest_wake_audio(_pcm20ms(4000))
    bridge.publish_quest_wake_audio(_pcm20ms(-4000))
    assert len(quest_wake.published) == 2
    assert quest_wake.published[0].data != quest_wake.published[1].data


def test_publish_quest_wake_audio_none_publisher_is_noop():
    """Юнит-тесты моста конструируют QuestBridge без ROS (quest_wake_pub=None
    по умолчанию) — publish_quest_wake_audio обязан молчать, а не падать."""
    pytest.importorskip("geometry_msgs", reason="QuestBridge требует rclpy/geometry_msgs (только в Docker image)")
    from rob_box_quest.quest_node import QuestBridge

    bridge = QuestBridge(
        node=_MockNode(),
        cmd_vel_quest_pub=_MockPublisher(),
        cmd_vel_emergency_pub=_MockPublisher(),
    )
    # Не должно бросить исключение.
    bridge.publish_quest_wake_audio(_pcm_chunk(4000, 4000))


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


# ── Шаг 4б grip-pipeline config (t_80e7aa1e) ──────────────────────────
# Контракт: ws_server → QuestBridge.publish_voice_pipeline → ROS String в
# /avatar/voice_pipeline → supervisor._on_grip_voice_pipeline.
# Тест проверяет, что JSON, который supervisor реально распарсит,
# содержит именно {llm_enabled: bool, preset: str, language: str}.


def _make_voice_pipeline_only_bridge(quest_node_mod):
    """QuestBridge с mock-publisher'ом для /avatar/voice_pipeline.

    Использует фикстуру ``quest_node_mod`` из conftest.py — она подменяет
    ROS-пакеты (rclpy, audio_common_msgs) заглушками. Без этого фикстура
    падает ``ModuleNotFoundError: No module named 'audio_common_msgs'``
    на dev-env без ROS (§4.3 ловушка из operator-handoff).
    """
    QuestBridge = quest_node_mod.QuestBridge

    node = _MockNode()
    voice_pipeline_pub = _MockPublisher()
    bridge = QuestBridge(
        node=node,
        cmd_vel_quest_pub=_MockPublisher(),
        cmd_vel_emergency_pub=_MockPublisher(),
        voice_pipeline_pub=voice_pipeline_pub,
    )
    return bridge, voice_pipeline_pub


def test_publish_voice_pipeline_publishes_json_to_ros(quest_node_mod):
    """publish_voice_pipeline → 1 String в /avatar/voice_pipeline."""
    bridge, voice_pipeline_pub = _make_voice_pipeline_only_bridge(quest_node_mod)
    bridge.publish_voice_pipeline(True, "translate", "en")
    assert len(voice_pipeline_pub.published) == 1
    msg = voice_pipeline_pub.published[0]
    # Payload — String.data, формат сериализации делает bridge, не ws_server.
    data = msg.data
    # Проверяем: это JSON-строка с ровно тремя контрактными полями.
    parsed = json.loads(data)
    assert parsed == {"llm_enabled": True, "preset": "translate", "language": "en"}


def test_publish_voice_pipeline_style_off_payload(quest_node_mod):
    """«Без стиля»: llm_enabled=False + preset='' → payload с пустым preset.

    Supervisor трактует preset='' как «Без стиля» (GRIP_OFF_PRESETS, см.
    supervisor_node.py:359). Важно, чтобы bridge не дропал пустой
    preset и не нормализовал в «none» — supervisor ждёт строку из
    списка GRIP_OFF_PRESETS = {"", "none", "off"}.
    """
    bridge, voice_pipeline_pub = _make_voice_pipeline_only_bridge(quest_node_mod)
    bridge.publish_voice_pipeline(False, "", "ru")
    assert len(voice_pipeline_pub.published) == 1
    parsed = json.loads(voice_pipeline_pub.published[0].data)
    assert parsed == {"llm_enabled": False, "preset": "", "language": "ru"}


def test_publish_voice_pipeline_missing_publisher_logs_and_silently_drops(
    quest_node_mod,
):
    """Если publisher не сконфигурирован — no-op + warning, не exception.

    Мост создаётся раньше publisher'а (см. quest_node.py — set_voice_*
    публикаторы тоже None до конструктора ноды). Чтобы ws_server мог
    жить, мост должен корректно падать в no-op, иначе первый
    voice_pipeline cmd отвалит handler.
    """
    QuestBridge = quest_node_mod.QuestBridge

    node = _MockNode()
    bridge = QuestBridge(
        node=node,
        cmd_vel_quest_pub=_MockPublisher(),
        cmd_vel_emergency_pub=_MockPublisher(),
        # voice_pipeline_pub явно не передаём → None
    )
    bridge.publish_voice_pipeline(True, "translate", "en")
    # Никаких exceptions. Warning залогирован.
    assert any("voice_pipeline" in w for w in node.warnings)


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


# ── AV-27 / issue #1919 — TTS picker (list_voices / set_voice / preview_voice) ─


class _MockStringPublisher(_MockPublisher):
    """Publisher, ожидающий std_msgs/String payload как .data."""

    def publish(self, msg) -> None:
        # Сохраняем .data (как rclpy.String) + repr для отладки.
        super().publish(getattr(msg, "data", msg))


def _make_voice_bridge(set_voice_pub=None, preview_voice_pub=None, voices_cache_ttl_sec=300.0):
    """Construct QuestBridge с mock-publishers для voice-picker.

    set_voice_pub / preview_voice_pub опциональны (None → мост будет
    возвращать nack для set_voice / молча drop для preview_voice).

    QuestBridge живёт в rob_box_quest.quest_node, который тянет rclpy +
    audio_common_msgs — пропускаем в dev-env (см. _make_bridge выше).

    NB: возвращает ТОЛЬКО (bridge, svp, pvp) — node доступен через
    ``bridge._node`` напрямую (тест-наблюдатель может подменить его на
    свой ``_MockNode`` для сбора .info() — см. test_set_voice_*).
    """
    pytest.importorskip("audio_common_msgs", reason="QuestBridge требует rclpy/audio_common_msgs (только в Docker image)")
    from rob_box_quest.quest_node import QuestBridge

    node = _MockNode()
    pub_quest = _MockPublisher()
    pub_emergency = _MockPublisher()
    svp = set_voice_pub or _MockStringPublisher()
    pvp = preview_voice_pub or _MockStringPublisher()
    bridge = QuestBridge(
        node=node,
        cmd_vel_quest_pub=pub_quest,
        cmd_vel_emergency_pub=pub_emergency,
        set_voice_pub=svp,
        preview_voice_pub=pvp,
        voices_cache_ttl_sec=voices_cache_ttl_sec,
    )
    return bridge, svp, pvp


def _string_msg(payload_str: str):
    """Создаёт fake ros String без зависимости от rclpy."""
    class _Msg:
        def __init__(self, d: str) -> None:
            self.data = d
    return _Msg(payload_str)


def test_voices_cache_empty_snapshot_before_latched_publish():
    """Без latched-publish от tts_node — snapshot возвращает voices=[]."""
    bridge, _, _ = _make_voice_bridge()
    snap = bridge.list_voices_snapshot()
    assert snap["voices"] == []
    assert snap["active_provider"] == ""
    assert snap["active_voice"] == ""


def test_voices_cache_hit_after_latched_publish():
    """После on_voices_message с приличным payload — snapshot содержит voices."""
    bridge, _, _ = _make_voice_bridge()
    payload = json.dumps({
        "provider": "yandex",
        "voice": "alena",
        "default_voice": "anton",
        "voices": [
            {"voice_id": "alena", "display_name": "Алёна", "language": "ru-RU", "gender": "female"},
            {"voice_id": "anton", "display_name": "Антон", "language": "ru-RU", "gender": "male"},
        ],
        "ts": 12345.0,
    })
    bridge.on_voices_message(_string_msg(payload))
    snap = bridge.list_voices_snapshot()
    assert len(snap["voices"]) == 2
    assert snap["voices"][0]["voice_id"] == "alena"
    assert snap["active_provider"] == "yandex"
    assert snap["active_voice"] == "alena"


def test_voices_cache_expiry_returns_empty():
    """voices_cache_ttl_sec=0.1 → через 0.2 с snapshot пустой (TTL истёк)."""
    bridge, _, _ = _make_voice_bridge(voices_cache_ttl_sec=0.1)
    payload = json.dumps({
        "provider": "yandex",
        "voice": "alena",
        "default_voice": "anton",
        "voices": [{"voice_id": "alena"}],
        "ts": 1.0,
    })
    bridge.on_voices_message(_string_msg(payload))
    snap1 = bridge.list_voices_snapshot()
    assert snap1["voices"] != []
    time.sleep(0.15)
    snap2 = bridge.list_voices_snapshot()
    assert snap2["voices"] == [], f"cache should expire but got {snap2['voices']}"


def test_on_provider_state_message_invalidates_cache_on_provider_change():
    """Смена провайдера → invalidate cache (TTL=0, чтобы следующий list увидел [])."""
    bridge, _, _ = _make_voice_bridge()
    payload = json.dumps({
        "provider": "yandex",
        "voice": "alena",
        "default_voice": "anton",
        "voices": [{"voice_id": "alena"}],
        "ts": 1.0,
    })
    bridge.on_voices_message(_string_msg(payload))
    assert bridge.list_voices_snapshot()["voices"] != []
    # Провайдер сменился на minimax → cache invalidates.
    bridge.on_provider_state_message(_string_msg(json.dumps({"provider": "minimax", "voice": "male-qn-qingse"})))
    snap = bridge.list_voices_snapshot()
    assert snap["voices"] == [], "cache should be invalidated by provider change"
    assert snap["active_provider"] == "minimax"
    assert snap["active_voice"] == "male-qn-qingse"


def test_set_voice_unknown_returns_nack_with_available():
    """set_voice(bogus) при активном yandex → nack + available=текущий список."""
    bridge, svp, _ = _make_voice_bridge()
    # Актитируем активный провайдер через provider_state (как сделал бы tts_node).
    bridge.on_provider_state_message(_string_msg(json.dumps({"provider": "yandex", "voice": "alena"})))
    # Загружаем voices_payload (через on_voices_message).
    bridge.on_voices_message(_string_msg(json.dumps({
        "provider": "yandex",
        "voice": "alena",
        "default_voice": "anton",
        "voices": [
            {"voice_id": "alena"},
            {"voice_id": "anton"},
        ],
        "ts": 1.0,
    })))
    ok, applied, reason, available = bridge.set_voice("bogus", None)
    assert ok is False
    assert applied is None
    assert reason == "voice_unavailable"
    assert sorted(available) == ["alena", "anton"]
    assert svp.published == [], "set_voice publisher must not be called on nack"


def test_set_voice_success_publishes_json_with_provider_hint():
    """set_voice(alena) при активном yandex → ack + publish в /avatar/set_voice.

    ADR-0073 / issue #2138.C: дополнительно проверяем две отладочные
    строки, которые позволяют отличить «picker ничего не прислал» (H5)
    от «pub/sub не доезжает до supervisor» (H2/H3) при поиске пропавших
    смен голоса в docker logs (см. PR-body).
    """
    bridge, svp, _ = _make_voice_bridge()
    bridge.on_provider_state_message(_string_msg(json.dumps({"provider": "yandex", "voice": "alena"})))
    bridge.on_voices_message(_string_msg(json.dumps({
        "provider": "yandex",
        "voice": "alena",
        "default_voice": "anton",
        "voices": [{"voice_id": "alena"}],
        "ts": 1.0,
    })))
    # Подменяем ``_node`` на наш логгер со сбором .info(), чтобы сверить,
    # что debug-логи моста действительно пишутся (это и есть contract
    # observability-фикса в #2138.C).
    node = _MockNode()
    bridge._node = node  # type: ignore[assignment]  # pyright: QuestNode vs MockNode
    ok, applied, reason, available = bridge.set_voice("alena", "friendly")
    assert ok is True
    assert applied == "alena"
    assert reason is None
    assert available is None
    assert len(svp.published) == 1
    parsed = json.loads(svp.published[0])
    assert parsed["voice_id"] == "alena"
    assert parsed["preset"] == "friendly"
    assert parsed["provider"] == "yandex"
    assert "ts_ms" in parsed
    # Две info-строки: «publishing…» ДО publish + «published…» ПОСЛЕ publish.
    publishing_lines = [m for m in node.infos if "set_voice: publishing" in m]
    published_lines = [m for m in node.infos if "set_voice: published" in m]
    assert publishing_lines, (
        f"bridge.set_voice должна логировать «publishing…» ДО publish; "
        f"получили infos={node.infos}"
    )
    assert published_lines, (
        f"bridge.set_voice должна логировать «published…» ПОСЛЕ publish; "
        f"получили infos={node.infos}"
    )
    assert "voice_id=alena" in publishing_lines[0]
    assert "voice_id=alena" in published_lines[0]
    assert "/avatar/set_voice" in publishing_lines[0]
    # «published» идёт ПОСЛЕ «publishing» — порядок критичен (если видна
    # только первая — publish завис/упал, и это явный сигнал проблемы).
    assert node.infos.index(publishing_lines[0]) < node.infos.index(published_lines[0]), (
        "«publishing» ОБЯЗАН появиться раньше «published» — иначе теряется "
        "смысл observability-фикса (см. ADR-0073 §3.3)"
    )


def test_set_voice_no_active_provider_returns_tts_unreachable():
    """Без provider_state — set_voice возвращает tts_unreachable, ничего не публикует."""
    bridge, svp, _ = _make_voice_bridge()
    ok, _, reason, _ = bridge.set_voice("alena", None)
    assert ok is False
    assert reason == "tts_unreachable"
    assert svp.published == []


def test_publish_preview_voice_emits_json():
    """publish_preview_voice → JSON в preview_voice_pub с request_id/voice_id/text."""
    bridge, _, pvp = _make_voice_bridge()
    bridge.on_provider_state_message(_string_msg(json.dumps({"provider": "yandex", "voice": "alena"})))
    bridge.publish_preview_voice("req-1", "alena", "Привет, оператор!")
    assert len(pvp.published) == 1
    parsed = json.loads(pvp.published[0])
    assert parsed["request_id"] == "req-1"
    assert parsed["voice_id"] == "alena"
    assert parsed["text"] == "Привет, оператор!"
    assert parsed["provider"] == "yandex"
    assert "ts_ms" in parsed


def test_publish_preview_voice_without_provider_still_emits():
    """preview_voice без provider_state (холодный старт) — provider="", но payload валиден."""
    bridge, _, pvp = _make_voice_bridge()
    bridge.publish_preview_voice("req-x", "alena", "test")
    assert len(pvp.published) == 1
    parsed = json.loads(pvp.published[0])
    assert parsed["provider"] == ""
    assert parsed["request_id"] == "req-x"


# ── issue #2138 — регресс «set_voice всегда tts_unreachable» ──────────────
#
# До фикса ``QuestBridge.set_voice`` валидировал ``voice_id`` через
# компайл-тайм реестр ``rob_box_voice.tts_voice_registry``. В образе
# ``rob-box-quest`` пакета нет (конвенция «пакет должен оставаться
# импортируемым без rob_box_voice»), и защищённый fallback возвращал
# ``[]`` → ``set_voice`` ВСЕГДА отдавал ``tts_unreachable`` при живом TTS.
#
# Фикс: валидация по latched-кэшу ``/voice/tts/voices``
# (``streams.voice_picker.pick_voice``). Тесты ниже фиксируют НОВЫЙ
# контракт — что ``quest_node.py`` НЕ зависит от ``tts_voice_registry``
# напрямую и НЕ требует его наличия в образе. Если кто-то снова
# потащит реестр в quest-узел (нарушая конвенцию импортируемости без
# rob_box_voice) — эти тесты укажут на регресс.
#
# Сама чистая логика валидации покрыта ``test_voice_picker.py`` —
# работает без rclpy/audio_common_msgs и на dev-env, и в Docker image.


def test_quest_node_does_not_import_tts_voice_registry():
    """Source-level регресс #2138: ``quest_node.py`` НЕ должен напрямую
    импортировать ``tts_voice_registry``.

    Парсим исходник AST-ом (работает на любом окружении). Если кто-то
    снова потащит реестр голосов прямо в quest-узел — образ
    ``rob-box-quest`` (без rob_box_voice) сломается с ``Restarting``
    loop на старте, как в регрессии PR #2105 (issue #2099).
    """
    import ast
    from pathlib import Path

    repo_root = Path(__file__).resolve().parents[4]  # test/unit/... → repo root
    quest_node_path = repo_root / "src" / "rob_box_quest" / "rob_box_quest" / "quest_node.py"
    assert quest_node_path.exists(), f"quest_node.py not found at {quest_node_path}"

    source = quest_node_path.read_text(encoding="utf-8")
    tree = ast.parse(source)

    bad_imports = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.ImportFrom):
            continue
        if node.module != "rob_box_voice.tts_voice_registry":
            continue
        bad_imports.append(ast.dump(node))

    assert not bad_imports, (
        "quest_node.py НЕ должен импортировать rob_box_voice.tts_voice_registry — "
        "пакет rob_box_voice отсутствует в образе rob-box-quest, и жёсткий/даже "
        "защищённый импорт — мёртвая ловушка. Валидация голосов теперь идёт по "
        "latched-кэшу /voice/tts/voices (см. streams.voice_picker.pick_voice, "
        "issue #2138 fix). Если ты вернул этот импорт — остановись и проверь "
        "issue #2138, прежде чем мёржить."
    )


def test_quest_node_does_not_define_voices_for_fallback():
    """Source-level регресс #2138: ``quest_node.py`` НЕ должен содержать
    собственный fallback ``_voices_for``.

    До фикса в файле был блок
    ``try: from rob_box_voice.tts_voice_registry import voices_for as _voices_for
    except ImportError: def _voices_for(provider): return []``.
    Этот блок — корень #2138: на роботе fallback всегда возвращал ``[]``,
    и ``set_voice`` не пропускал ни одного голоса. После фикса валидация
    идёт через ``pick_voice`` (cache), и модуль ``quest_node`` ничем
    подобным не занимается.
    """
    import ast
    from pathlib import Path

    repo_root = Path(__file__).resolve().parents[4]
    quest_node_path = repo_root / "src" / "rob_box_quest" / "rob_box_quest" / "quest_node.py"
    source = quest_node_path.read_text(encoding="utf-8")
    tree = ast.parse(source)

    # Ищем любую ``def _voices_for(...)`` внутри модуля. Если она
    # появится — это регресс #2138.
    fallback_defs = []
    for node in ast.walk(tree):
        if (
            isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
            and node.name == "_voices_for"
        ):
            fallback_defs.append(ast.dump(node))

    assert not fallback_defs, (
        "quest_node.py не должен определять локальный fallback _voices_for — "
        "это путь к регрессу issue #2138. Валидация голоса теперь — "
        "streams.voice_picker.pick_voice по latched-кэшу /voice/tts/voices."
    )


def test_quest_node_uses_pick_voice_from_voice_picker_module():
    """Source-level регресс #2138: ``QuestBridge.set_voice`` обязан вызывать
    ``pick_voice`` из ``streams.voice_picker``.

    Без этого вызова нет фикса #2138 — кто-то может случайно откатить
    bridge на старый путь через ``_voices_for(provider)``. Тест находит
    метод ``set_voice`` и проверяет, что его тело содержит обращение к
    ``pick_voice``.
    """
    import ast
    from pathlib import Path

    repo_root = Path(__file__).resolve().parents[4]
    quest_node_path = repo_root / "src" / "rob_box_quest" / "rob_box_quest" / "quest_node.py"
    tree = ast.parse(quest_node_path.read_text(encoding="utf-8"))

    # Найти класс QuestBridge → метод set_voice → имена в его теле.
    set_voice_found = False
    uses_pick_voice = False
    for node in ast.walk(tree):
        if not isinstance(node, ast.ClassDef):
            continue
        if node.name != "QuestBridge":
            continue
        for item in node.body:
            if (
                isinstance(item, ast.FunctionDef)
                and item.name == "set_voice"
            ):
                set_voice_found = True
                for sub in ast.walk(item):
                    if isinstance(sub, ast.Name) and sub.id == "pick_voice":
                        uses_pick_voice = True
                    if isinstance(sub, ast.Attribute) and sub.attr == "pick_voice":
                        uses_pick_voice = True

    assert set_voice_found, "QuestBridge.set_voice не найден — структура файла изменилась?"
    assert uses_pick_voice, (
        "QuestBridge.set_voice обязан вызывать pick_voice (issue #2138 fix). "
        "Если ты это убрал — это регресс: set_voice снова ходит через "
        "реестр/кэш неверным путём и ломает выбор голоса на роботе."
    )
