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

import types
from unittest.mock import MagicMock

import sys as _sys
from pathlib import Path as _Path

_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))
from ros_stubs import install_ros_stubs  # noqa: E402

_INSTALLED = False


def _install_all_mocks():
    global _INSTALLED
    if _INSTALLED:
        return
    _INSTALLED = True

    # rclpy / rclpy.node / rclpy.qos / std_msgs come from the shared set, and
    # ``install_ros_stubs`` is additive: whichever directory loads first
    # registers them, the rest only fill in names that are missing. A private
    # `std_msgs.msg` here carried `String` but not `Bool`, so running
    # `unit/sound` before `unit/node` broke the *import* of 18 dialogue-node
    # test modules that each passed on their own — the same failure shape
    # `rclpy.qos` had before it was shared.
    audio_msg = types.ModuleType("audio_common_msgs.msg")
    audio_msg.AudioData = type("AudioData", (), {})

    rcl_ifaces_msg = types.ModuleType("rcl_interfaces.msg")
    rcl_ifaces_msg.SetParametersResult = MagicMock

    pydub = types.ModuleType("pydub")
    pydub.AudioSegment = type("AudioSegment", (), {})

    install_ros_stubs(extra={
        "audio_common_msgs": types.ModuleType("audio_common_msgs"),
        "audio_common_msgs.msg": audio_msg,
        "rcl_interfaces": types.ModuleType("rcl_interfaces"),
        "rcl_interfaces.msg": rcl_ifaces_msg,
        # Audio backends: no ALSA, no ReSpeaker on a dev box or in CI
        # (issue #1133 — sounddevice is optional).
        "sounddevice": MagicMock(),
        "pyaudio": MagicMock(),
        "usb": MagicMock(),
        "usb.core": MagicMock(),
        "usb.util": MagicMock(),
        "pydub": pydub,
    })


_install_all_mocks()
