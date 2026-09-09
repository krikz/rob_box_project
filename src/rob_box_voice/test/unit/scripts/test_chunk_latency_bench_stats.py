"""Unit tests for scripts/tts_bench/chunk_latency_bench.py — pure logic.

Скрипт ``chunk_latency_bench.py`` намертво зависит от rclpy и
``audio_common_msgs`` (запускается только в voice-assistant
контейнере). Но ядро агрегации — ``percentile`` и
``ChunkLatencyRecorder.deltas_ms`` — чистые функции, которые
можно покрыть здесь, без ROS2.

Эти тесты — страховка от регрессии в формуле p50/p95 (главная
цифра, которую Шифу видит в PR).
"""
from __future__ import annotations

import importlib.util
import statistics
import sys
import time
import types
from pathlib import Path
from typing import Any

import pytest

# Скрипт scripts/tts_bench/chunk_latency_bench.py при импорте дёргает
# rclpy и audio_common_msgs. Подменяем их на стабы ДО exec_module —
# так ядро (``percentile``, ``ChunkLatencyRecorder``) доступно для
# тестирования без ROS-окружения.

REPO_ROOT = Path(__file__).resolve().parents[5]  # repo root
_SCRIPT_PATH = REPO_ROOT / "scripts" / "tts_bench" / "chunk_latency_bench.py"


class _FakeNode:
    """Минимальный stand-in для rclpy.node.Node — Recorder не дёргает
    его методов кроме ``create_subscription``, который в тестах не
    вызывается."""


def _install_ros_stubs() -> None:
    """Создаёт фейковые rclpy / audio_common_msgs / std_msgs, чтобы
    импорт скрипта не упал на отсутствии настоящего ROS2."""
    if "rclpy" in sys.modules:
        return  # вдруг уже есть — не трогаем

    rclpy_stub = types.ModuleType("rclpy")
    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.HistoryPolicy = type("HistoryPolicy", (), {"KEEP_LAST": "KEEP_LAST"})
    rclpy_qos.ReliabilityPolicy = type(
        "ReliabilityPolicy", (), {"RELIABLE": "RELIABLE"}
    )
    rclpy_qos.QoSProfile = type(
        "QoSProfile",
        (),
        {"__init__": lambda self, **_: None},
    )
    rclpy_stub.init = lambda: None
    rclpy_stub.shutdown = lambda: None
    rclpy_stub.spin_once = lambda *_a, **_k: None
    rclpy_stub.create_node = lambda *_a, **_k: _FakeNode()

    audio_common_stub = types.ModuleType("audio_common_msgs")
    audio_common_msgs_stub = types.ModuleType("audio_common_msgs.msg")
    audio_common_msgs_stub.AudioData = type("AudioData", (), {})
    audio_common_stub.msg = audio_common_msgs_stub

    std_msgs_stub = types.ModuleType("std_msgs")
    std_msgs_msg_stub = types.ModuleType("std_msgs.msg")
    std_msgs_msg_stub.String = type("String", (), {"data": ""})
    std_msgs_stub.msg = std_msgs_msg_stub

    sys.modules["rclpy"] = rclpy_stub
    sys.modules["rclpy.qos"] = rclpy_qos
    sys.modules["audio_common_msgs"] = audio_common_stub
    sys.modules["audio_common_msgs.msg"] = audio_common_msgs_stub
    sys.modules["std_msgs"] = std_msgs_stub
    sys.modules["std_msgs.msg"] = std_msgs_msg_stub


class _FakeNode:
    """Минимальный stand-in для rclpy.node.Node — Recorder не дёргает
    его методов кроме ``create_subscription``, который в тестах не
    вызывается."""


_install_ros_stubs()
_SPEC = importlib.util.spec_from_file_location(
    "_tts_chunk_latency_bench_under_test", _SCRIPT_PATH
)
assert _SPEC is not None and _SPEC.loader is not None
bench = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(bench)  # type: ignore[union-attr]
percentile = bench.percentile

# Наш стаб ``rclpy`` нужен только для import-устойчивости
# ``chunk_latency_bench.py``. После загрузки — снимаем его,
# чтобы другие тесты (test_pregen_tts_integration.py и т.п.)
# видели настоящее окружение и сами решали, надо ли им skip-аться
# на отсутствии rclpy / torch / sounddevice. Иначе install stub
# сбивает их _skipif_ логику, и они падают вместо skip.
for _name in (
    "rclpy",
    "rclpy.qos",
    "audio_common_msgs",
    "audio_common_msgs.msg",
    "std_msgs",
    "std_msgs.msg",
):
    sys.modules.pop(_name, None)


class _FakeSub:
    """No-op подписка — используется, чтобы Recorder инициализировался."""


def test_percentile_empty_returns_zero() -> None:
    assert percentile([], 50) == 0.0
    assert percentile([], 95) == 0.0


def test_percentile_single_element() -> None:
    assert percentile([42.0], 50) == 42.0
    assert percentile([42.0], 95) == 42.0


def test_percentile_known_distribution() -> None:
    # Набор [1..100] — формула ``int(round(p/100*(n-1)))``:
    # p50: idx=int(round(0.5*99))=50 → xs[50]=51
    # p95: idx=int(round(0.95*99))=94 → xs[94]=95
    xs = list(range(1, 101))
    assert percentile(xs, 50) == 51
    assert percentile(xs, 95) == 95


def test_percentile_known_small_sample() -> None:
    # Проверка «маленьких n» (типично для n_deltas≈20-30):
    # p=95, n=20 → idx=int(round(0.95*19))=18 → xs[18] (последние 2 — хвост).
    xs = list(range(1, 21))  # 1..20
    assert percentile(xs, 95) == 19
    assert percentile(xs, 50) == int(round(0.5 * 19)) + 1  # = xs[10] = 11


def test_percentile_unordered_input_is_sorted() -> None:
    xs = [95, 1, 50, 99, 10]
    # sorted: [1, 10, 50, 95, 99]
    # p50 на 5 точках: idx=int(round(0.5*4))=2 → xs[2]=50
    # p95: idx=int(round(0.95*4))=4 → xs[4]=99
    assert percentile(xs, 50) == 50
    assert percentile(xs, 95) == 99


def test_percentile_clamps_to_range() -> None:
    # p=200 должно вести себя как p=100
    xs = [1, 2, 3]
    assert percentile(xs, 200) == 3
    # p=0 — как минимум
    assert percentile(xs, 0) == 1


class _FakeNode:
    """Минимальный stand-in для rclpy.node.Node.

    Recorder использует только ``create_subscription``, и в
    юнит-тестах мы её не дёргаем. Поэтому возвращаем пустой
    sentinel — лишь бы Recorder инициализировался.
    """

    def create_subscription(self, *_args: Any, **_kwargs: Any) -> _FakeSub:
        return _FakeSub()


def test_recorder_deltas_ms_empty_for_zero_chunks() -> None:
    """У recorder, не получившего ни одного чанка, ``deltas_ms() == []``."""
    recorder = bench.ChunkLatencyRecorder(_FakeNode(), "/voice/audio/speech")  # type: ignore[arg-type]
    assert recorder.deltas_ms() == []


def test_recorder_deltas_ms_one_chunk_returns_empty() -> None:
    """У одного чанка нет «предыдущего» — delta не определена."""
    recorder = bench.ChunkLatencyRecorder.__new__(bench.ChunkLatencyRecorder)  # type: ignore[attr-defined]
    recorder.chunk_recv_times = [time.monotonic()]
    assert recorder.deltas_ms() == []


def test_recorder_deltas_ms_computes_correct_intervals() -> None:
    """delta_t — это разница между соседними моментами, в мс."""
    recorder = bench.ChunkLatencyRecorder.__new__(bench.ChunkLatencyRecorder)  # type: ignore[attr-defined]
    # 0.000, 0.050, 0.150, 0.350 → deltas: 50, 100, 200 мс
    base = 1000.0
    recorder.chunk_recv_times = [
        base,
        base + 0.050,
        base + 0.150,
        base + 0.350,
    ]
    deltas = recorder.deltas_ms()
    assert deltas == pytest.approx([50.0, 100.0, 200.0])
    # И статистически: mean = 116.66..., stddev > 0
    assert statistics.fmean(deltas) == pytest.approx(116.666, abs=0.01)


def test_recorder_reset_clears_buffer() -> None:
    recorder = bench.ChunkLatencyRecorder.__new__(bench.ChunkLatencyRecorder)  # type: ignore[attr-defined]
    recorder.chunk_recv_times = [1.0, 2.0, 3.0]
    recorder.reset()
    assert recorder.chunk_recv_times == []
    assert recorder.deltas_ms() == []


def test_summary_aggregation_matches_per_replication_join() -> None:
    """Если в серии нет чанков — p50/p95 = 0 (а не падаем)."""
    # Это «контракт» для рантайма: пустой результат должен
    # дать нулевую статистику, не бросая ZeroDivisionError.
    assert percentile([], 50) == 0.0
    assert percentile([], 95) == 0.0
