"""Unit-тесты audio_reactive_animation_node (issue #1050).

Проверяет:
- chunk_size default 4096 (issue #1050: 1024 → 4096 против paInputOverflow);
- обработку PyAudio status=2 (paInputOverflow) в audio_callback:
  (а) не падает на коротком in_data, (б) логирует rate-limited,
  (в) сообщает сколько байт потеряно, (г) продолжает публикации.

Тест НЕ требует железа/PyAudio: скрипт загружается из scripts/ с
замоканными rclpy/std_msgs/std_srvs/pyaudio, узел создаётся через
__new__ без __init__ (как в test_audio_node_echo.py для rob_box_voice).
"""

from __future__ import annotations

import importlib.util
import sys
import time
from pathlib import Path
from unittest.mock import MagicMock

import pytest

_SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts" / "audio_reactive_animation_node.py"
)


@pytest.fixture(scope="module")
def node_module():
    """Загружает scripts/audio_reactive_animation_node.py с ROS2-моками."""
    # Мокаем зависимости, которых нет в dev-окружении (в CI реальные
    # rclpy/std_msgs/std_srvs уже стоят, setdefault их не перезапишет).
    _install_dep_mocks()

    spec = importlib.util.spec_from_file_location(
        "audio_reactive_animation_node", _SCRIPT_PATH
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _install_dep_mocks():
    """Регистрирует MagicMock-заглушки для импортов скрипта.

    Используем setdefault, как в conftest.py: реальные модули (ROS2
    Humble в CI) выигрывают, в dev-окружении работают моки.
    """
    if "rclpy" not in sys.modules:
        class FakeNode:
            def __init__(self, *a, **kw):
                self._logger = MagicMock()

            def get_logger(self):
                return self._logger

            def create_publisher(self, *a, **kw):
                return MagicMock()

            def create_subscription(self, *a, **kw):
                return MagicMock()

        mock_rclpy = MagicMock()
        mock_rclpy_node = MagicMock()
        mock_rclpy_node.Node = FakeNode
        sys.modules.setdefault("rclpy", mock_rclpy)
        sys.modules.setdefault("rclpy.node", mock_rclpy_node)

    if "std_msgs.msg" not in sys.modules:
        msg = MagicMock()
        msg.String = MagicMock
        msg.Float32 = MagicMock
        sys.modules.setdefault("std_msgs", MagicMock())
        sys.modules.setdefault("std_msgs.msg", msg)

    if "std_srvs.srv" not in sys.modules:
        srv = MagicMock()
        srv.Trigger = MagicMock
        sys.modules.setdefault("std_srvs", MagicMock())
        sys.modules.setdefault("std_srvs.srv", srv)

    if "pyaudio" not in sys.modules:
        pa = MagicMock()
        pa.paContinue = 0
        pa.paInt16 = 8
        sys.modules.setdefault("pyaudio", pa)


def _make_node(node_module, **attrs):
    """Узел без __init__: только атрибуты, нужные audio_callback."""
    node = node_module.AudioReactiveAnimationNode.__new__(
        node_module.AudioReactiveAnimationNode
    )
    node.audio_reactive_enabled = False
    node.audio_threshold = 0.3
    node.audio_smoothing = 0.2
    node.smoothed_volume = 0.0
    node.audio_level_pub = MagicMock()
    node.animation_trigger_pub = MagicMock()
    node._overflow_count = 0
    node._overflow_last_logged = 0.0
    node._overflow_log_window_s = 60.0
    node.get_logger = lambda: MagicMock(
        info=lambda *a, **kw: None,
        warning=lambda *a, **kw: None,
        warn=lambda *a, **kw: None,
        error=lambda *a, **kw: None,
        debug=lambda *a, **kw: None,
    )
    for k, v in attrs.items():
        setattr(node, k, v)
    return node


class TestChunkSizeDefault:
    """Issue 1050: frames_per_buffer 1024 → 4096 (23ms → ~93ms @44.1kHz)."""

    def test_chunk_size_declared_4096(self, node_module):
        """В __init__ параметр chunk_size объявлен со значением 4096."""
        src = _SCRIPT_PATH.read_text(encoding="utf-8")
        assert "declare_parameter('chunk_size', 4096)" in src
        assert "declare_parameter('chunk_size', 1024)" not in src

    def test_chunk_size_used_as_frames_per_buffer(self, node_module):
        """chunk_size передаётся в PyAudio как frames_per_buffer."""
        src = _SCRIPT_PATH.read_text(encoding="utf-8")
        assert "frames_per_buffer=self.chunk_size" in src


class TestInputOverflowHandling:
    """Issue 1050: paInputOverflow (PyAudio status=2) — rate-limited лог."""

    @staticmethod
    def _capture_warnings(node):
        warnings = []

        def _logger():
            return MagicMock(
                info=lambda *a, **kw: None,
                warning=lambda *a, **kw: warnings.append(a[0] if a else ""),
                warn=lambda *a, **kw: None,
                error=lambda *a, **kw: None,
                debug=lambda *a, **kw: None,
            )

        node.get_logger = _logger
        return warnings

    def test_overflow_short_chunk_no_crash(self, node_module):
        """Короткий in_data при overflow (потеря кадров) — без исключений."""
        node = _make_node(node_module)
        short_data = b"\x00\x00" * 256  # 512 байт вместо ожидаемых 2048
        result = node.audio_callback(short_data, 1024, {}, 2)
        assert result[1] == 0  # paContinue
        assert node._overflow_count == 1

    def test_overflow_no_log_when_status_0(self, node_module):
        node = _make_node(node_module)
        warnings = self._capture_warnings(node)
        node.audio_callback(b"\x00\x00" * 1024, 1024, {}, 0)
        assert node._overflow_count == 0
        assert warnings == []

    def test_overflow_rate_limited_log(self, node_module):
        """100 переполнений подряд → одна строка в лог, счётчик растёт."""
        node = _make_node(node_module)
        warnings = self._capture_warnings(node)
        for _ in range(100):
            node.audio_callback(b"\x00\x00" * 1024, 1024, {}, 2)
        assert node._overflow_count == 100
        assert len(warnings) == 1
        assert "paInputOverflow" in warnings[0]

    def test_overflow_log_again_after_window(self, node_module):
        """После окончания окна следующее переполнение снова логируется."""
        node = _make_node(node_module)
        warnings = self._capture_warnings(node)
        node.audio_callback(b"\x00\x00" * 1024, 1024, {}, 2)
        assert len(warnings) == 1
        node._overflow_last_logged = 0.0  # «окно» прошло
        node.audio_callback(b"\x00\x00" * 1024, 1024, {}, 2)
        assert node._overflow_count == 2
        assert len(warnings) == 2

    def test_overflow_log_reports_lost_bytes(self, node_module):
        """В логе видно сколько байт ожидалось/пришло/потеряно."""
        node = _make_node(node_module)
        warnings = self._capture_warnings(node)
        node.audio_callback(b"\x00\x00" * 256, 1024, {}, 2)
        assert len(warnings) == 1
        # channels=2, 16-bit: 1024 фрейма = 4096 байт; пришло 512
        assert "512/4096" in warnings[0]
        assert "потеряно ~3584" in warnings[0]

    def test_callback_publishes_when_enabled_and_no_overflow(self, node_module):
        """Без overflow callback публикует уровень и не трогает счётчик."""
        node = _make_node(node_module, audio_reactive_enabled=True)
        warnings = self._capture_warnings(node)
        data = b"\x00\x10" * 1024  # ненулевой сигнал, полный чанк
        node.audio_callback(data, 1024, {}, 0)
        assert node._overflow_count == 0
        assert warnings == []
        node.audio_level_pub.publish.assert_called_once()
