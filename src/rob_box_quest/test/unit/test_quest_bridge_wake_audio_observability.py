"""issue #1992 observability: QuestBridge.publish_quest_wake_audio раньше не
логировал ни единой публикации в /audio/quest_wake — на роботе ``docker
logs rob-box-quest`` показывал ту же тишину что при живом потоке, что и при
клиенте, который вообще ничего не шлёт (симптом отчёта: владелец сказал
«ТАРС» в шлем несколько раз, в логах ни строки по wake/VOICE_AUDIO).

issue #2135: единица учёта здесь — ФРАЗА, а не 20мс-кадр. Мост копит кадры
в ``WakePhraseSegmenter`` и публикует одну ``AudioData`` на фразу, поэтому
и «first packet», и сводка считают фразы. Тесты переведены на этот контракт
(раньше «одна публикация» получалась от одного кадра).

Тесты выполняются и без ROS: фикстура ``quest_node_mod``
(``test/unit/conftest.py``) подставляет заглушки ROS-модулей, если
настоящих нет. Раньше файл целиком скипался через ``importorskip`` — ровно
это однажды привело к ложному закрытию карточки #1992.
"""

from rob_box_quest.core.wake_segmenter import WAKE_PHRASE_GAP_TIMEOUT_S

FRAME = b"\x00\x01" * 320  # 640 байт = 20 мс @ 16 кГц int16
FRAME_PERIOD_S = 0.02
FRAMES_PER_PHRASE = 25  # 0.5 с — заведомо выше порога «блип VAD»


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


def _make_bridge(quest_node_mod, monkeypatch, quest_wake_pub=None):
    """QuestBridge с ручными часами (clock[0] — то, что вернёт monotonic)."""
    clock = [5000.0]
    monkeypatch.setattr(quest_node_mod.time, "monotonic", lambda: clock[0])
    node = _MockNode()
    if quest_wake_pub is None:
        quest_wake_pub = _MockPublisher()
    bridge = quest_node_mod.QuestBridge(
        node=node,
        cmd_vel_quest_pub=_MockPublisher(),
        cmd_vel_emergency_pub=_MockPublisher(),
        quest_wake_pub=quest_wake_pub,
    )
    return bridge, quest_wake_pub, node, clock


def _say_one_phrase(bridge, clock) -> None:
    """Одна фраза: поток кадров + пауза, которую замечает таймер."""
    for _ in range(FRAMES_PER_PHRASE):
        bridge.publish_quest_wake_audio(FRAME)
        clock[0] += FRAME_PERIOD_S
    clock[0] += WAKE_PHRASE_GAP_TIMEOUT_S
    bridge.tick_wake_audio(clock[0])


def test_first_wake_publish_logs_immediately(quest_node_mod, monkeypatch):
    bridge, quest_wake_pub, node, clock = _make_bridge(quest_node_mod, monkeypatch)
    _say_one_phrase(bridge, clock)
    assert len(quest_wake_pub.published) == 1
    assert any(
        "first packet" in m and "/audio/quest_wake" in m for m in node.logger.infos
    )


def test_second_publish_within_window_does_not_log_again(quest_node_mod, monkeypatch):
    bridge, quest_wake_pub, node, clock = _make_bridge(quest_node_mod, monkeypatch)
    _say_one_phrase(bridge, clock)
    node.logger.infos.clear()
    _say_one_phrase(bridge, clock)
    assert node.logger.infos == []
    assert len(quest_wake_pub.published) == 2  # публикация не блокируется логом


def test_summary_logged_after_interval_elapses(quest_node_mod, monkeypatch):
    bridge, quest_wake_pub, node, clock = _make_bridge(quest_node_mod, monkeypatch)
    _say_one_phrase(bridge, clock)  # first packet, count=1
    node.logger.infos.clear()
    for _ in range(9):
        _say_one_phrase(bridge, clock)  # count=2..10
    assert node.logger.infos == []
    clock[0] += quest_node_mod.WAKE_AUDIO_LOG_INTERVAL_S
    _say_one_phrase(bridge, clock)  # count=11 -> summary
    summaries = [m for m in node.logger.infos if "publish summary" in m]
    assert len(summaries) == 1
    assert "10 packets" in summaries[0]
    assert "total 11 packets" in summaries[0]


def test_none_publisher_stays_silent_no_op(quest_node_mod, monkeypatch):
    """quest_wake_pub=None (unit-тесты моста без ROS) — no-op, без
    AttributeError на логгере: publish_quest_wake_audio должен вернуться до
    любого учёта."""
    clock = [5000.0]
    monkeypatch.setattr(quest_node_mod.time, "monotonic", lambda: clock[0])
    node = _MockNode()
    bridge = quest_node_mod.QuestBridge(
        node=node,
        cmd_vel_quest_pub=_MockPublisher(),
        cmd_vel_emergency_pub=_MockPublisher(),
        quest_wake_pub=None,
    )
    bridge.publish_quest_wake_audio(b"\x00\x01")
    assert node.logger.infos == []
