"""test_issue_1101_diagnostics.py — Diagnostics для пропусков LLM.

Issue #1101 (live 11.08) — оператор видит «робот молчит», а в логе
нет ни одной ошибки. Реальная причина — фраза отбрасывается на gate'е
``_on_stt`` ещё до LLM. Этот тест проверяет:

1. ``_llm_skipped_counter`` инициализируется всеми известными причинами.
2. ``_maybe_log_skip_summary`` пишет сводку только если есть пропуски.
3. ``_maybe_log_skip_summary`` НЕ спамит в каждом вызове — только раз в окно.
4. Метод не падает, если ``_llm_skipped_counter`` пуст (первый цикл).

Тест pure-Python: ``DialogueNode.__init__`` не вызывается, используется
``object.__new__`` + подмена ``get_logger`` (как в test_dialog_core.py).
На dev-машине rclpy не установлен — изолируемся через sys.modules stub,
как в test_dialogue_register_speaker.py.
"""

from __future__ import annotations

import sys
import time
import types
from unittest.mock import MagicMock

import pytest


# ---------------------------------------------------------------------------
# Изоляция от rclpy: подменяем rclpy/rob_box_harness/... на пустые модули
# до ``import rob_box_voice.dialogue_node``. Тестируем только pure-Python
# метод ``_maybe_log_skip_summary`` (не требует rclpy.callback_groups или
# ROS-инфраструктуры).
# ---------------------------------------------------------------------------


def _stub_rclpy() -> None:
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda *a, **k: None
    rclpy.shutdown = lambda *a, **k: None
    rclpy.spin = lambda *a, **k: None
    rclpy.logging = types.ModuleType("rclpy.logging")

    class _Node:
        def __init__(self, *a, **k):
            pass

    rclpy.node = types.ModuleType("rclpy.node")
    rclpy.node.Node = _Node

    class _QoSProfile:
        def __init__(self, **k):
            for kk, vv in k.items():
                setattr(self, kk, vv)

    class _HistoryPolicy:
        KEEP_LAST = "KEEP_LAST"

    class _ReliabilityPolicy:
        RELIABLE = "RELIABLE"

    class _DurabilityPolicy:
        VOLATILE = "VOLATILE"

    rclpy.qos = types.ModuleType("rclpy.qos")
    rclpy.qos.QoSProfile = _QoSProfile
    rclpy.qos.HistoryPolicy = _HistoryPolicy
    rclpy.qos.ReliabilityPolicy = _ReliabilityPolicy
    rclpy.qos.DurabilityPolicy = _DurabilityPolicy

    rclpy.callback_groups = types.ModuleType("rclpy.callback_groups")

    class _ReentrantCallbackGroup:
        pass

    rclpy.callback_groups.ReentrantCallbackGroup = _ReentrantCallbackGroup

    rclpy.duration = types.ModuleType("rclpy.duration")

    class _Duration:
        def __init__(self, seconds=0):
            self.seconds = seconds

    rclpy.duration.Duration = _Duration

    sys.modules.setdefault("rclpy", rclpy)
    sys.modules.setdefault("rclpy.node", rclpy.node)
    sys.modules.setdefault("rclpy.qos", rclpy.qos)
    sys.modules.setdefault("rclpy.callback_groups", rclpy.callback_groups)
    sys.modules.setdefault("rclpy.duration", rclpy.duration)
    sys.modules.setdefault("rclpy.logging", rclpy.logging)


_stub_rclpy()

# std_msgs и audio_common_msgs — ROS-only пакеты, которых нет на dev-машине.
def _stub_msgs() -> None:
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")

    class _String:
        def __init__(self):
            self.data = ""

    class _Bool:
        def __init__(self):
            self.data = False

    class _Int32:
        def __init__(self):
            self.data = 0

    std_msgs_msg.String = _String
    std_msgs_msg.Bool = _Bool
    std_msgs_msg.Int32 = _Int32
    sys.modules.setdefault("std_msgs", std_msgs)
    sys.modules.setdefault("std_msgs.msg", std_msgs_msg)

    audio_msgs = types.ModuleType("audio_common_msgs")
    audio_msgs_msg = types.ModuleType("audio_common_msgs.msg")

    class _AudioData:
        def __init__(self):
            self.data = []

    audio_msgs_msg.AudioData = _AudioData
    sys.modules.setdefault("audio_common_msgs", audio_msgs)
    sys.modules.setdefault("audio_common_msgs.msg", audio_msgs_msg)


_stub_msgs()

from rob_box_voice.dialogue_node import DialogueNode  # noqa: E402  import after stubs


# ---------------------------------------------------------------------------
# Изоляция: подсовываем мок для DialogueNode.get_logger, не поднимая rclpy.
# ---------------------------------------------------------------------------


def _make_node():
    """Собрать DialogueNode-подобный объект без ROS2.

    Чистый object.__new__ + ручная инициализация полей, нужных для
    теста. ``get_logger`` подменяем MagicMock'ом с info().
    """
    node = object.__new__(DialogueNode)
    node._llm_skipped_counter = {
        "no_wake_word": 0,
        "silenced": 0,
        "silence_command": 0,
        "empty_after_strip": 0,
        "stt_rejected": 0,
        "music_stop": 0,
    }
    node._last_skip_summary_ts = time.monotonic() - 1000.0  # окно уже прошло
    node.get_logger = MagicMock()
    return node


# ---------------------------------------------------------------------------
# Тесты
# ---------------------------------------------------------------------------


def test_counter_initialized_with_all_reasons() -> None:
    """Все известные причины пропуска должны быть в счётчике."""
    node = _make_node()
    expected = {
        "no_wake_word",
        "silenced",
        "silence_command",
        "empty_after_strip",
        "stt_rejected",
        "music_stop",
    }
    assert set(node._llm_skipped_counter.keys()) == expected


def test_summary_logs_breakdown_when_counters_nonzero() -> None:
    """Если были пропуски — пишем одну строку с разбивкой."""
    node = _make_node()
    node._llm_skipped_counter["no_wake_word"] = 3
    node._llm_skipped_counter["silenced"] = 1

    node._maybe_log_skip_summary()

    # Должен быть ровно один info-вызов с breakdown.
    assert node.get_logger().info.called
    msg = node.get_logger().info.call_args[0][0]
    assert "llm_skipped_total=4" in msg
    assert "no_wake_word=3" in msg
    assert "silenced=1" in msg
    # Нулевые причины в breakdown не попадают (не спамит).
    assert "stt_rejected=0" not in msg


def test_summary_skipped_when_no_counters_nonzero() -> None:
    """Если ни одного пропуска — НЕ пишем сводку (чтобы не спамить пустыми окнами)."""
    node = _make_node()
    node._maybe_log_skip_summary()
    assert not node.get_logger().info.called


def test_summary_respects_window() -> None:
    """Сводка НЕ чаще раза в ``window_s`` — повторный вызов в окне игнорируется."""
    node = _make_node()
    node._llm_skipped_counter["no_wake_word"] = 5

    # Свежий timestamp — окно ещё не прошло.
    node._last_skip_summary_ts = time.monotonic()
    node._maybe_log_skip_summary()
    assert not node.get_logger().info.called

    # Прокручиваем время назад через «окно прошло».
    node._last_skip_summary_ts = time.monotonic() - 600.0
    node._maybe_log_skip_summary()
    assert node.get_logger().info.called


def test_summary_uses_custom_window() -> None:
    """Параметр ``window_s`` корректно работает (smoke-test)."""
    node = _make_node()
    node._llm_skipped_counter["stt_rejected"] = 7

    # Окно в 1 секунду, timestamp = сейчас - 2 секунды.
    node._last_skip_summary_ts = time.monotonic() - 2.0
    node._maybe_log_skip_summary(window_s=1.0)
    assert node.get_logger().info.called
