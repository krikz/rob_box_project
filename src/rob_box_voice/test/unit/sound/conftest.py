"""conftest.py — моки ROS2/audio-зависимостей для unit-тестов SoundNode.

``sound_node.py`` на импорте тянет rclpy, std_msgs, audio_common_msgs,
sounddevice, pyaudio (через utils.audio_utils → pydub/ReSpeaker) и numpy.
На dev-машине / CI нет ни ALSA, ни ReSpeaker, ни ROS2-рантайма — этот
conftest ставит минимальные заглушки, чтобы модуль импортировался, а
``SoundNode.__init__`` доходил до конца без ошибок (issue #1133 — sounddevice
опционален для CI).

Заглушки НЕ эмулируют реальный звук: они дают проверить контракты
голосового passthrough (подписка, стрим, координация с эффектами).
"""

import sys
import types
from unittest.mock import MagicMock

_INSTALLED = False


def _install_all_mocks():
    global _INSTALLED
    if _INSTALLED:
        return
    _INSTALLED = True

    # ── rclpy.node.Node ──────────────────────────────────────────────────
    class FakeNode:
        """Минимальная заглушка rclpy.node.Node: записывает подписки/таймеры."""

        def __init__(self, name, **kwargs):
            self._name = name
            self._logger = MagicMock()
            self._declared = {}
            self._subscriptions = []
            self._publishers = []
            self._timers = []

        def get_logger(self):
            return self._logger

        def declare_parameter(self, name, default=None):
            p = MagicMock()
            p.value = default
            self._declared[name] = p
            return p

        def get_parameter(self, name):
            return self._declared.get(name, MagicMock(value=None))

        def has_parameter(self, name):
            return name in self._declared

        def create_publisher(self, *args, **kwargs):
            pub = MagicMock()
            self._publishers.append((args, kwargs))
            return pub

        def create_subscription(self, *args, **kwargs):
            sub = MagicMock()
            self._subscriptions.append((args, kwargs))
            return sub

        def create_timer(self, *args, **kwargs):
            timer = MagicMock()
            self._timers.append((args, kwargs))
            return timer

        def add_on_set_parameters_callback(self, cb):
            return MagicMock()

        def get_name(self):
            return self._name

    mock_rclpy_node = types.ModuleType("rclpy.node")
    mock_rclpy_node.Node = FakeNode

    # ── rclpy.qos ────────────────────────────────────────────────────────
    class _Policy:
        BEST_EFFORT = "best_effort"
        RELIABLE = "reliable"
        VOLATILE = "volatile"
        KEEP_LAST = "keep_last"

    class FakeQoSProfile:
        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)

    mock_rclpy_qos = types.ModuleType("rclpy.qos")
    mock_rclpy_qos.QoSProfile = FakeQoSProfile
    mock_rclpy_qos.ReliabilityPolicy = _Policy
    mock_rclpy_qos.DurabilityPolicy = _Policy
    mock_rclpy_qos.HistoryPolicy = _Policy

    # ── std_msgs.msg ─────────────────────────────────────────────────────
    mock_std_msgs = types.ModuleType("std_msgs")
    mock_std_msgs_msg = types.ModuleType("std_msgs.msg")
    mock_std_msgs_msg.String = type("String", (), {"data": ""})
    sys.modules.setdefault("std_msgs", mock_std_msgs)
    sys.modules.setdefault("std_msgs.msg", mock_std_msgs_msg)

    # ── audio_common_msgs.msg ────────────────────────────────────────────
    mock_audio_pkg = types.ModuleType("audio_common_msgs")
    mock_audio_msg = types.ModuleType("audio_common_msgs.msg")
    mock_audio_msg.AudioData = type("AudioData", (), {})
    sys.modules.setdefault("audio_common_msgs", mock_audio_pkg)
    sys.modules.setdefault("audio_common_msgs.msg", mock_audio_msg)

    # ── rcl_interfaces.msg (lazy-import в parameters_callback) ───────────
    mock_rcl_ifaces = types.ModuleType("rcl_interfaces")
    mock_rcl_ifaces_msg = types.ModuleType("rcl_interfaces.msg")
    mock_rcl_ifaces_msg.SetParametersResult = MagicMock
    sys.modules.setdefault("rcl_interfaces", mock_rcl_ifaces)
    sys.modules.setdefault("rcl_interfaces.msg", mock_rcl_ifaces_msg)

    # ── audio deps: sounddevice / pyaudio / usb / pydub ──────────────────
    sys.modules.setdefault("sounddevice", MagicMock())
    sys.modules.setdefault("pyaudio", MagicMock())
    sys.modules.setdefault("usb", MagicMock())
    sys.modules.setdefault("usb.core", MagicMock())
    sys.modules.setdefault("usb.util", MagicMock())
    mock_pydub = types.ModuleType("pydub")
    mock_pydub.AudioSegment = type("AudioSegment", (), {})
    sys.modules.setdefault("pydub", mock_pydub)

    # ── rclpy (корневой) ─────────────────────────────────────────────────
    mock_rclpy = MagicMock()
    mocks = {
        "rclpy": mock_rclpy,
        "rclpy.node": mock_rclpy_node,
        "rclpy.qos": mock_rclpy_qos,
    }
    for name, mock in mocks.items():
        sys.modules.setdefault(name, mock)


_install_all_mocks()
