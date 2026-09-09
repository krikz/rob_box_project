"""rclpy stub for the TTS audio test bench.

The bench runs on a minimal host that may NOT have ROS2 Humble installed.
The official ``packages.ros.org/ros2/ubuntu jammy`` builds are
ABI-linked to ``libpython3.10.so.1.0``, ``libspdlog.so.1`` (legacy
SONAME), and ``libfmt.so.8``; Debian 13 trixie only ships
``libpython3.13``, ``libspdlog1.15`` (so version) and ``libfmt10`` —
so a force-install of the jammy ``.deb`` packages loads the Python
C-extension but fails on the C++ ABI in
``librcl_logging_spdlog.so`` (``undefined symbol`` on
``spdlog::sinks::basic_file_sink``). Production deploys use the
project's ``ghcr.io/krikz/rob_box_base:rtabmap`` Docker image (which
already includes a full ROS2 Humble + audio_common_msgs install —
see ``docker/main/ros2_control/Dockerfile`` for an example of
extending that base).

To still exercise the real :class:`rob_box_voice.tts_node.TTSNode`
code path on a host without ROS2, the bench installs a minimal stub
package that fulfils the surface used by ``tts_node``:

* ``rclpy.init()`` / ``rclpy.shutdown()`` / ``rclpy.ok()``
* ``rclpy.node.Node`` with ``declare_parameter``, ``get_parameter``,
  ``has_parameter``, ``get_logger``, ``get_name``, ``destroy_node``,
  ``add_on_set_parameters_callback``, ``create_subscription``,
  ``create_publisher``, and ``publish_state`` (the last is a project
  extension — we just record the last state on the stub).
* ``rclpy.qos.{QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy}``
* ``rcl_interfaces.msg.SetParametersResult``

Every ``create_publisher`` is intercepted so the bench can attach an
``AudioSink`` to ``tts_node.audio_pub`` and verify the messages that
flow over ``/voice/audio/speech``. ``create_subscription`` records
handlers but does not deliver anything (the bench calls the handlers
directly via :func:`dispatch_dialogue_message`).

The stub is intentionally narrow — only the surface the production
``TTSNode`` uses. The bench also runs a **separate real subprocess
subscriber** (:mod:`tts_audio_bench.scripts.real_subscriber`) on every
scenario that publishes AudioData; that subprocess receives the same
``AudioData`` payloads over a stdio pipe and writes its own WAV file
that ``ffprobe`` cross-checks, which is the bench's end-to-end
demonstration that what ``tts_node`` publishes is what a real ROS
subscriber would consume on ``/voice/audio/speech``.

When ``tts_node`` runs inside the project's Docker image the same
production code is exercised, but the subscriber is a real
``rclpy`` node subscribed on ``audio_common_msgs.msg.AudioData``.
The wire contract (``uint8[] data``, int16 LE, mono, configured
sample rate) is identical, and ``real_subscriber.py`` is structured
so swapping it for a real ``rclpy`` subscriber is a one-line change.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import Any, Callable, Dict, List, Optional


@dataclass
class _Parameter:
    name: str
    value: Any
    type_: str = "string"


class _Node:
    """Drop-in replacement for :class:`rclpy.node.Node`."""

    _params: Dict[str, _Parameter] = {}
    _subs: Dict[str, "_Subscription"] = {}
    _pubs: Dict[str, "_Publisher"] = {}
    _param_callback: Optional[Callable] = None

    def __init__(self, name: str = "tts_node", *, context: Any = None):
        self._name = name
        self._params = {}
        self._subs = {}
        self._pubs = {}
        self._param_callback = None
        self._logger = logging.getLogger(f"rclstub.{name}")
        # Surfaced for tests that want to assert destroy() happened.
        self.destroyed = False

    # Identity -------------------------------------------------------------
    def get_name(self) -> str:
        return self._name

    def destroy_node(self) -> None:
        self.destroyed = True

    # Parameters -----------------------------------------------------------
    def declare_parameter(self, name: str, default: Any = None) -> _Parameter:
        # Auto-detect type from default for get_parameter compatibility.
        type_ = type(default).__name__ if default is not None else "string"
        # rclpy normalises Python ``bool`` to type "bool", ``int`` to "int",
        # ``float`` to "double", ``str`` to "string". Map closely enough for
        # tts_node's get_parameter("...").value usage.
        if type_ == "bool":
            t = "bool"
        elif type_ in ("int",):
            t = "int"
        elif type_ in ("float",):
            t = "double"
        else:
            t = "string"
        p = _Parameter(name=name, value=default, type_=t)
        self._params[name] = p
        return p

    def has_parameter(self, name: str) -> bool:
        return name in self._params

    def get_parameter(self, name: str) -> SimpleNamespace:
        if name not in self._params:
            raise KeyError(f"parameter {name!r} not declared")
        p = self._params[name]
        # Mimic rclpy's ``Parameter`` shape: ``.value``, ``.type_``.
        return SimpleNamespace(value=p.value, type_=p.type_)

    def add_on_set_parameters_callback(self, cb: Callable) -> None:
        self._param_callback = cb

    def set_parameters_atomically(self, params: List[Any]) -> Any:
        for p in params:
            self._params[p.name] = _Parameter(name=p.name, value=p.value)
        # Match the production return shape (SetParametersResult with .successful).
        return SimpleNamespace(successful=True)

    # Pub/sub --------------------------------------------------------------
    def create_subscription(self, msg_type: Any, topic: str, callback: Callable, qos: int) -> "_Subscription":
        s = _Subscription(msg_type=msg_type, topic=topic, callback=callback)
        self._subs[topic] = s
        return s

    def create_publisher(self, msg_type: Any, topic: str, qos: Any) -> "_Publisher":
        # Normalise QoSProfile to the (reliability, history, depth) tuple
        # the bench cares about.
        p = _Publisher(msg_type=msg_type, topic=topic, qos=qos)
        self._pubs[topic] = p
        return p

    # Logging --------------------------------------------------------------
    def get_logger(self) -> logging.Logger:
        return self._logger


@dataclass
class _Subscription:
    msg_type: Any
    topic: str
    callback: Callable


@dataclass
class _Publisher:
    msg_type: Any
    topic: str
    qos: Any
    captured: List[Any] = field(default_factory=list)

    def publish(self, msg: Any) -> None:
        self.captured.append(msg)


# ---------------------------------------------------------------------------
# QoS helpers
# ---------------------------------------------------------------------------


class _Policy:
    BEST_EFFORT = "best_effort"
    RELIABLE = "reliable"
    KEEP_LAST = "keep_last"
    VOLATILE = "volatile"


ReliabilityPolicy = SimpleNamespace(BEST_EFFORT=_Policy.BEST_EFFORT, RELIABLE=_Policy.RELIABLE)
HistoryPolicy = SimpleNamespace(KEEP_LAST=_Policy.KEEP_LAST)
DurabilityPolicy = SimpleNamespace(VOLATILE=_Policy.VOLATILE)


@dataclass
class QoSProfile:
    history: str = _Policy.KEEP_LAST
    depth: int = 10
    reliability: str = _Policy.RELIABLE
    durability: str = _Policy.VOLATILE


# ---------------------------------------------------------------------------
# rcl_interfaces stand-in
# ---------------------------------------------------------------------------


@dataclass
class SetParametersResult:
    successful: bool = True


_rcl_interfaces_msg = SimpleNamespace(SetParametersResult=SetParametersResult)


@dataclass
class AudioData:
    """Minimal stand-in for ``audio_common_msgs.msg.AudioData``.

    The production message carries ``uint8[] data`` (raw int16 LE PCM
    bytes). We expose the same ``.data`` slot so the bench can replay
    the same payload through ``wave`` for verification.
    """

    data: List[int] = field(default_factory=list)


def install() -> None:
    """Install the stub into ``sys.modules`` under ``rclpy`` and friends.

    Idempotent: safe to call multiple times. Returns once the stub is in
    place; downstream ``import rclpy`` statements pick it up.
    """
    import sys
    import types

    if "rclpy" in sys.modules and getattr(sys.modules["rclpy"], "_is_stub", False):
        return

    rclpy = types.ModuleType("rclpy")
    rclpy._is_stub = True
    rclpy.init = lambda args=None: None
    rclpy.shutdown = lambda: None
    rclpy.ok = lambda: True

    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = _Node

    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.QoSProfile = QoSProfile
    rclpy_qos.ReliabilityPolicy = ReliabilityPolicy
    rclpy_qos.HistoryPolicy = HistoryPolicy
    rclpy_qos.DurabilityPolicy = DurabilityPolicy

    rcl_interfaces = types.ModuleType("rcl_interfaces")
    rcl_interfaces_msg = types.ModuleType("rcl_interfaces.msg")
    rcl_interfaces_msg.SetParametersResult = SetParametersResult

    # audio_common_msgs stand-in. Production AudioData is a generated
    # ROS message with a single ``uint8[] data`` field; the stub is a
    # Python dataclass with the same surface.
    audio_common_msgs = types.ModuleType("audio_common_msgs")
    audio_common_msgs_msg = types.ModuleType("audio_common_msgs.msg")
    audio_common_msgs_msg.AudioData = AudioData

    # std_msgs stand-in (String only — that's all tts_node uses).
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")

    @dataclass
    class String:
        data: str = ""

    std_msgs_msg.String = String

    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.node"] = rclpy_node
    sys.modules["rclpy.qos"] = rclpy_qos
    sys.modules["rcl_interfaces"] = rcl_interfaces
    sys.modules["rcl_interfaces.msg"] = rcl_interfaces_msg
    sys.modules["audio_common_msgs"] = audio_common_msgs
    sys.modules["audio_common_msgs.msg"] = audio_common_msgs_msg
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg


# ---------------------------------------------------------------------------
# Helpers consumed by run_bench.py
# ---------------------------------------------------------------------------


class AudioSink:
    """Listener attached to the ``audio_pub`` publisher.

    The bench calls :meth:`attach` immediately after :class:`TTSNode`
    construction so every subsequent :func:`_Publisher.publish` lands in
    :attr:`messages`. We also track ``first_arrival`` so the bench can
    compute time-to-first-AudioData (TTFA) without monkey-patching the
    node's internal timer.
    """

    def __init__(self) -> None:
        self.messages: List[Any] = []
        self.first_arrival: Optional[float] = None

    def attach(self, publisher: _Publisher) -> None:
        original_publish = publisher.publish
        sink = self

        def wrapper(msg: Any) -> None:
            import time as _time
            if sink.first_arrival is None:
                sink.first_arrival = _time.monotonic()
            sink.messages.append(msg)
            # Note: we intentionally do NOT call original_publish — that
            # would also append to publisher.captured, which is fine, but
            # keeps the wrapping path simple and side-effect-free.
            original_publish(msg)

        publisher.publish = wrapper  # type: ignore[assignment]


def dispatch_dialogue_message(node: _Node, topic: str, payload: Any) -> None:
    """Synchronously invoke the subscription callback for ``topic``."""
    sub = node._subs.get(topic)
    if sub is None:
        raise KeyError(f"no subscription on {topic}")
    sub.callback(payload)