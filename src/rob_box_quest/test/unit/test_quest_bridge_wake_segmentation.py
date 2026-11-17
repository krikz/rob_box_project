"""issue #2135: QuestBridge публикует ОДНУ AudioData на фразу, а не на кадр.

Почему этот файл не пользуется ``pytest.importorskip`` как соседние
``test_quest_bridge*.py``
-------------------------------------------------------------------------
На dev-машине нет ни ``rclpy``, ни ``audio_common_msgs`` — все тесты
``QuestBridge`` там скипаются ЦЕЛИКОМ. Ровно из-за этого однажды по ошибке
закрыли карточку #1992: «тесты зелёные» означало «78 тестов пропущено».

Здесь ROS-модули подменяются заглушками через фикстуру ``quest_node_mod``
(``test/unit/conftest.py``): подмена ставится только для модулей, которых
реально нет, и снимается в teardown — в Docker-образе с ROS тест гоняет
настоящие сообщения, а соседние файлы с ``importorskip`` продолжают честно
скипаться.

issue #2199: тест переведён с ``rob_box_quest.core.wake_segmenter`` (legacy
shim) на ``rob_box_core.speech_segmentation.DEFAULT_WAKE_CONFIG``. Контракт
тот же — поток 20мс-кадров → ровно одна публикация на фразу.

Что проверяется
---------------
* поток 20мс-кадров + пауза → ровно одна публикация с суммарным payload
  (на коде ДО фикса здесь было 50 публикаций по 640 байт — и каждая
  запускала в ``stt_node`` полный цикл распознавания на 0.02 с);
* потолок буфера (бесконечный поток не съедает память);
* сброс буфера при разрыве WS-сессии.
"""

from rob_box_core.speech_segmentation import DEFAULT_WAKE_CONFIG


GAP_TIMEOUT_S = DEFAULT_WAKE_CONFIG.gap_timeout_s
WAKE_PHRASE_MAX_BYTES = DEFAULT_WAKE_CONFIG.max_phrase_bytes


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


FRAME = b"\x11\x22" * 320  # 640 байт = 20 мс @ 16 кГц int16
FRAME_PERIOD_S = 0.02


def _make_bridge(quest_node_mod, monkeypatch):
    """QuestBridge с mock-publisher'ом /audio/quest_wake и ручными часами."""
    clock = [1000.0]
    monkeypatch.setattr(quest_node_mod.time, "monotonic", lambda: clock[0])
    node = _MockNode()
    quest_wake_pub = _MockPublisher()
    bridge = quest_node_mod.QuestBridge(
        node=node,
        cmd_vel_quest_pub=_MockPublisher(),
        cmd_vel_emergency_pub=_MockPublisher(),
        quest_wake_pub=quest_wake_pub,
    )
    return bridge, quest_wake_pub, node, clock


def _stream(bridge, clock, n_frames: int) -> None:
    for _ in range(n_frames):
        bridge.publish_quest_wake_audio(FRAME)
        clock[0] += FRAME_PERIOD_S


def test_frame_stream_plus_pause_publishes_one_audiodata(quest_node_mod, monkeypatch):
    """DoD #2135. На коде ДО фикса тут было 50 сообщений по 640 байт."""
    bridge, wake_pub, _node, clock = _make_bridge(quest_node_mod, monkeypatch)

    _stream(bridge, clock, 50)  # 1.0 с речи
    assert wake_pub.published == [], "кадры не должны уходить по одному"

    clock[0] += GAP_TIMEOUT_S
    bridge.tick_wake_audio(clock[0])

    assert len(wake_pub.published) == 1
    assert bytes(wake_pub.published[0].data) == FRAME * 50
    assert len(wake_pub.published[0].data) == 32000  # 1.0 с, а не 0.02 с


def test_tick_without_audio_publishes_nothing(quest_node_mod, monkeypatch):
    """Таймер 30 Гц крутится всегда — без речи он не должен ничего слать."""
    bridge, wake_pub, _node, clock = _make_bridge(quest_node_mod, monkeypatch)
    for _ in range(100):
        clock[0] += 1.0 / 30.0
        bridge.tick_wake_audio(clock[0])
    assert wake_pub.published == []


def test_two_replies_publish_two_messages(quest_node_mod, monkeypatch):
    """Две реплики через паузу — два сообщения, а не одно склеенное."""
    bridge, wake_pub, _node, clock = _make_bridge(quest_node_mod, monkeypatch)

    _stream(bridge, clock, 40)
    clock[0] += GAP_TIMEOUT_S
    bridge.tick_wake_audio(clock[0])

    clock[0] += 3.0
    _stream(bridge, clock, 25)
    clock[0] += GAP_TIMEOUT_S
    bridge.tick_wake_audio(clock[0])

    assert len(wake_pub.published) == 2
    assert bytes(wake_pub.published[0].data) == FRAME * 40
    assert bytes(wake_pub.published[1].data) == FRAME * 25


def test_buffer_cap_publishes_and_frees_memory(quest_node_mod, monkeypatch):
    """Бесконечный поток кадров (залипший клиентский VAD) не копится без
    границы: на потолке фраза уходит, буфер обнуляется."""
    bridge, wake_pub, _node, clock = _make_bridge(quest_node_mod, monkeypatch)

    frames_to_cap = WAKE_PHRASE_MAX_BYTES // len(FRAME)
    _stream(bridge, clock, frames_to_cap + 10)

    assert len(wake_pub.published) == 1
    assert len(wake_pub.published[0].data) == WAKE_PHRASE_MAX_BYTES
    assert bridge._wake_segmenter.buffered_bytes == 10 * len(FRAME)


def test_session_disconnect_drops_unfinished_phrase(quest_node_mod, monkeypatch):
    """Разрыв WS: хвост фразы ушедшего оператора не долетает до STT и не
    склеивается с речью следующей сессии."""
    bridge, wake_pub, node, clock = _make_bridge(quest_node_mod, monkeypatch)

    _stream(bridge, clock, 40)
    bridge.reset_wake_audio()
    assert bridge._wake_segmenter.buffered_bytes == 0

    clock[0] += GAP_TIMEOUT_S
    bridge.tick_wake_audio(clock[0])
    assert wake_pub.published == [], "выброшенный буфер не должен воскреснуть"
    assert any("dropped" in m for m in node.logger.infos)

    # Новая сессия — фраза собирается с нуля.
    clock[0] += 5.0
    _stream(bridge, clock, 30)
    clock[0] += GAP_TIMEOUT_S
    bridge.tick_wake_audio(clock[0])
    assert len(wake_pub.published) == 1
    assert bytes(wake_pub.published[0].data) == FRAME * 30


def test_ws_unregister_session_resets_wake_buffer(quest_node_mod, monkeypatch):
    """Шов «WS-сессия закончилась → сброс буфера» реально соединён:
    WSSServer._unregister_session зовёт Bridge.reset_wake_audio()."""
    from rob_box_quest.server.ws_server import WSSServer
    from rob_box_quest.server.session import ClientSession

    bridge, wake_pub, _node, clock = _make_bridge(quest_node_mod, monkeypatch)
    _stream(bridge, clock, 40)
    assert bridge._wake_segmenter.buffered_bytes > 0

    server = WSSServer(bridge=bridge, pin="123456")
    session = ClientSession(session_id="s-2135")
    server._unregister_session(session)

    assert bridge._wake_segmenter.buffered_bytes == 0
    clock[0] += GAP_TIMEOUT_S
    bridge.tick_wake_audio(clock[0])
    assert wake_pub.published == []


def test_voice_listen_stop_resets_wake_buffer(quest_node_mod, monkeypatch):
    """Тумблер «всегда слушать» выключен → недособранная фраза выбрасывается."""
    bridge, wake_pub, _node, clock = _make_bridge(quest_node_mod, monkeypatch)

    bridge.set_wake_stream_state(True)
    _stream(bridge, clock, 40)
    bridge.set_wake_stream_state(False)

    assert bridge._wake_segmenter.buffered_bytes == 0
    clock[0] += GAP_TIMEOUT_S
    bridge.tick_wake_audio(clock[0])
    assert wake_pub.published == []


def test_none_publisher_stays_silent_no_op(quest_node_mod, monkeypatch):
    """quest_wake_pub=None (моста без ROS) — ни публикаций, ни исключений."""
    clock = [1000.0]
    monkeypatch.setattr(quest_node_mod.time, "monotonic", lambda: clock[0])
    node = _MockNode()
    bridge = quest_node_mod.QuestBridge(
        node=node,
        cmd_vel_quest_pub=_MockPublisher(),
        cmd_vel_emergency_pub=_MockPublisher(),
        quest_wake_pub=None,
    )
    for _ in range(50):
        bridge.publish_quest_wake_audio(FRAME)
        clock[0] += FRAME_PERIOD_S
    bridge.tick_wake_audio(clock[0] + GAP_TIMEOUT_S)
    assert node.logger.infos == []
