"""conftest.py — mocks for TTS node tests.

TTSNode pulls in a heavy mix of imports at module load (rclpy,
audio_common_msgs, sounddevice, grpc for yandex gRPC, torch for silero,
pyaudio for respeaker detection). On a developer host (no ReSpeaker /
USB-audio stack) importing ``rob_box_voice.tts_node`` fails — this
conftest installs minimal stubs so the module can be imported and tested.

The mocks are NOT meant to exercise the real providers — they let
``TTSNode.__init__`` finish without error so we can poke at the methods
we care about (retry classification, transcode integration, streaming
chunk publication). For unit-testing individual methods we replace
``TTSNode.__init__`` with a tiny stub via ``MagicMock(spec=...)``.
"""

import sys
import types
from unittest.mock import MagicMock


def _install_all_mocks():
    """Install ALL mocks needed to import ``rob_box_voice.tts_node``."""

    # ── rclpy ─────────────────────────────────────────────────────────────
    mock_rclpy = MagicMock()

    class FakeNode:
        """Minimal stub of rclpy.node.Node. TTSNode.__init__ calls.
        declare_parameter (for ~30 params), get_parameter,
        create_publisher / create_subscription, and
        add_on_set_parameters_callback.
        """

        def __init__(self, name, **kwargs):
            self._name = name
            self._logger = MagicMock()
            self._declared = {}

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
            return MagicMock()

        def create_subscription(self, *args, **kwargs):
            return MagicMock()

        def add_on_set_parameters_callback(self, cb):
            return MagicMock()

        def get_name(self):
            return self._name

    mock_rclpy_node = types.ModuleType("rclpy.node")
    mock_rclpy_node.Node = FakeNode

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

    # ── std_msgs.msg ──────────────────────────────────────────────────────
    mock_std_msgs_pkg = types.ModuleType("std_msgs")
    mock_std_msgs_msg = types.ModuleType("std_msgs.msg")
    mock_std_msgs_msg.String = type("String", (), {"data": ""})
    sys.modules.setdefault("std_msgs", mock_std_msgs_pkg)
    sys.modules.setdefault("std_msgs.msg", mock_std_msgs_msg)

    # ── audio_common_msgs.msg ─────────────────────────────────────────────
    mock_audio_pkg = types.ModuleType("audio_common_msgs")
    mock_audio_msg = types.ModuleType("audio_common_msgs.msg")
    mock_audio_msg.AudioData = type("AudioData", (), {})
    sys.modules.setdefault("audio_common_msgs", mock_audio_pkg)
    sys.modules.setdefault("audio_common_msgs.msg", mock_audio_msg)

    # ── audio deps (sounddevice, pyaudio) ─────────────────────────────────
    sys.modules.setdefault("sounddevice", MagicMock())
    sys.modules.setdefault("pyaudio", MagicMock())

    # ── grpc ──────────────────────────────────────────────────────────────
    # Use a real-ish stub: MagicMock for everything *except* ``RpcError``,
    # which must be a real ``BaseException`` subclass so that
    # ``except grpc.RpcError as e:`` in production code actually catches
    # raised errors. Without this, ``except grpc.RpcError as e:`` raises
    # ``TypeError: catching classes that do not inherit from BaseException``
    # because MagicMock instances are not BaseException subclasses.
    grpc_mock = MagicMock()

    class _FakeRpcError(Exception):
        """Stand-in for ``grpc.RpcError`` (must inherit BaseException)."""

        def __init__(self, code=None, details=""):
            self._code = code
            self._details = details
            super().__init__(f"{code} - {details}" if code else details)

        def code(self):
            return self._code

        def details(self):
            return self._details

    grpc_mock.RpcError = _FakeRpcError
    sys.modules.setdefault("grpc", grpc_mock)

    # ── torch (Silero uses it lazily) ─────────────────────────────────────
    sys.modules.setdefault("torch", MagicMock())

    # ── USB / ReSpeaker helpers used in utils/__init__.py ─────────────────
    sys.modules.setdefault("usb", MagicMock())
    sys.modules.setdefault("usb.core", MagicMock())
    sys.modules.setdefault("usb.util", MagicMock())

    # Stub ``rob_box_voice.utils.audio_transcode`` ONLY. We keep the
    # utils package as a real module so its submodules are still
    # importable, but route the helpers through our stub. The other
    # utils submodules (audio_utils → pyaudio, respeaker_interface → pyusb)
    # are NOT loaded — neither tts_node nor our tests need them at runtime.
    #
    # This lets us bypass the utils/__init__.py which would otherwise pull
    # in pyaudio + pyusb. The audio_transcode stub here is the real one
    # loaded via importlib below; we wire it up so that the
    # `from .utils.audio_transcode import ...` at the top of tts_node.py
    # succeeds and yields the real (not MagicMocked) implementation.

    # Pre-load the transcode module into sys.modules under the right key
    # so tts_node's `from .utils.audio_transcode import ...` finds it
    # without triggering utils/__init__.py.
    import importlib.util as _ilu

    # conftest.py lives at:
    #   <root>/rob_box_voice/test/unit/tts/conftest.py
    # The real transcode helper is at:
    #   <root>/rob_box_voice/rob_box_voice/utils/audio_transcode.py
    _ROOT = __file__.rsplit("/test/unit/tts/conftest.py", 1)[0]
    _TRANSCODE_PATH = _ROOT + "/rob_box_voice/utils/audio_transcode.py"
    _spec = _ilu.spec_from_file_location("rob_box_voice.utils.audio_transcode", _TRANSCODE_PATH)
    _transcode_mod = _ilu.module_from_spec(_spec)
    sys.modules["rob_box_voice.utils.audio_transcode"] = _transcode_mod
    _spec.loader.exec_module(_transcode_mod)

    # Set up the utils package with the attribute access that
    # tts_node uses (find_respeaker_device_sounddevice); the real
    # __init__.py is NOT imported.
    mock_utils_pkg = types.ModuleType("rob_box_voice.utils")
    mock_utils_pkg.__path__ = [_ROOT + "/rob_box_voice/utils"]
    mock_utils_pkg.find_respeaker_device_sounddevice = MagicMock()
    sys.modules["rob_box_voice.utils"] = mock_utils_pkg

    # ── registry ──────────────────────────────────────────────────────────
    mocks = {
        "rclpy": mock_rclpy,
        "rclpy.node": mock_rclpy_node,
        "rclpy.qos": mock_rclpy_qos,
    }
    for name, mock in mocks.items():
        sys.modules[name] = mock


_install_all_mocks()
