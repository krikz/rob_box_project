"""One ROS2 stub set, shared by every test that runs without a ROS runtime.

Why this exists
---------------
The voice tests each grew their own hand-rolled ``rclpy`` stub — fourteen of
them at last count, defining the same names in four different styles
(``MagicMock``, ``SimpleNamespace``, ad-hoc classes, module attributes). They
were installed with ``sys.modules.setdefault``, so whichever test directory
pytest reached first won and the rest were silently ignored.

That is not a cosmetic problem. ``unit/greeting/conftest.py`` registered an
``rclpy.qos`` without ``DurabilityPolicy``; in a full run it got there before
``unit/node/conftest.py``, and **19 test modules failed to import** with
``cannot import name 'DurabilityPolicy'`` — including
``test_system_prompt_tools_check.py``. Each of them passed in isolation, so
the breakage looked like flakiness rather than a missing stub attribute.

:func:`install_ros_stubs` is *additive*: it installs missing modules and
fills in missing attributes on modules another stub already registered. That
makes it order-independent — a partial stub installed first is completed
rather than left to break an unrelated import.
"""

from __future__ import annotations

import sys
import types
from unittest.mock import MagicMock

__all__ = ["FakeNode", "FakeQoSProfile", "install_ros_stubs", "qos_stub"]


class FakeNode:
    """Stand-in for ``rclpy.node.Node`` — no sockets, no executor.

    One class for every test directory, because only one can win.
    ``rclpy.node`` is installed with ``sys.modules.setdefault``, so in a full
    run the first conftest to load supplies the base class for *all* the
    others. Four directories each had their own ``FakeNode`` and the winner
    changed with the selection: ``unit/sound`` asserts on ``_subscriptions``,
    which ``unit/node``'s copy never recorded, so
    ``test_subscribes_to_avatar_voice_in_best_effort`` failed in a full run
    and passed on its own; ``unit/node``'s copy also answered every
    ``get_parameter`` with ``None``, so ``SoundNode.__init__`` died on
    ``sound_pack_dir.startswith`` when it happened to load first.

    So this records in every shape the suites read:

    ``_parameters``    name -> declared default (greeting)
    ``_declared``      name -> parameter object (tts, sound)
    ``_publishers``    ``(topic, fake)`` (greeting)
    ``_subscribers``   ``(topic, fake)`` (greeting)
    ``_subscriptions`` ``(args, kwargs)`` (sound)
    ``_timers``        timer objects with ``period``/``callback``/``cancel``

    ``get_parameter`` returns the declared default rather than ``None``:
    that is what ``rclpy`` does, and a stub that forgets defaults turns every
    node constructor into a ``NoneType`` crash.
    """

    def __init__(self, name, **kwargs):
        self._name = name
        self._logger = MagicMock()
        self._logger.info = MagicMock()
        self._logger.warning = MagicMock()
        self._logger.warn = MagicMock()
        self._logger.error = MagicMock()
        self._logger.debug = MagicMock()
        self._parameters: dict = {}
        self._declared: dict = {}
        self._publishers: list = []
        self._subscribers: list = []
        self._subscriptions: list = []
        self._timers: list = []

    # ── identity ────────────────────────────────────────────────────────
    def get_name(self):
        return self._name

    def get_namespace(self):
        return ""

    def get_logger(self):
        return self._logger

    # ── parameters ──────────────────────────────────────────────────────
    def declare_parameter(self, name, default=None):
        if name not in self._parameters:
            self._parameters[name] = default
        parameter = MagicMock()
        parameter.value = self._parameters[name]
        self._declared[name] = parameter
        return parameter

    def get_parameter(self, name):
        if name in self._declared:
            return self._declared[name]
        parameter = MagicMock()
        parameter.value = None
        return parameter

    def has_parameter(self, name):
        return name in self._declared

    def set_parameters_atomically(self, params):
        return MagicMock(successful=True)

    def add_on_set_parameters_callback(self, callback):
        return MagicMock()

    # ── publishers / subscriptions / timers ─────────────────────────────
    def create_publisher(self, msg_type=None, topic=None, *args, **kwargs):
        fake = MagicMock()
        fake.topic = topic
        fake.msg_type = msg_type
        self._publishers.append((topic, fake))
        return fake

    def create_subscription(self, msg_type=None, topic=None, callback=None,
                            *args, **kwargs):
        fake = MagicMock()
        fake.topic = topic
        fake.callback = callback
        self._subscribers.append((topic, fake))
        # Sound's assertions index this positionally: args[1] is the topic.
        self._subscriptions.append(
            ((msg_type, topic, callback, *args), kwargs)
        )
        return fake

    def create_timer(self, period=None, callback=None, *args, **kwargs):
        fake = MagicMock()
        fake.period = period
        fake.callback = callback
        fake.cancel = MagicMock()
        self._timers.append(fake)
        return fake

    def create_service(self, *args, **kwargs):
        return MagicMock()

    def create_client(self, *args, **kwargs):
        return MagicMock()

    # ── introspection (startup_greeting_node waits on real subscribers) ──
    def get_publisher_names_and_types_by_node(self, name, namespace):
        return [(topic, ["std_msgs/String"]) for topic, _ in self._publishers]

    def get_subscriber_names_and_types_by_node(self, name, namespace):
        return [(topic, ["std_msgs/String"]) for topic, _ in self._subscribers]

    def count_subscribers(self, topic):
        return sum(1 for t, _ in self._subscribers if t == topic)

    def destroy_node(self):
        self._publishers.clear()
        self._subscribers.clear()
        self._subscriptions.clear()
        for timer in self._timers:
            try:
                timer.cancel()
            except Exception:  # noqa: BLE001 — a MagicMock cannot fail, but
                pass          # a test may swap in something that can.
        self._timers.clear()


class FakeQoSProfile:
    """Records the QoS kwargs so tests can assert on them.

    ``rclpy``'s real ``QoSProfile`` exposes ``reliability`` / ``durability``
    / ``history`` attributes, and the sound tests check them; a bare
    ``MagicMock`` silently satisfies any assertion, which is worse than
    useless in a stub.
    """

    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class _Policy:
    """All QoS policy members the voice nodes reference, as plain strings.

    Values are lower-case because that is what the sound and TTS suites
    already assert against. Real ``rclpy`` policies are IntEnums and would
    not compare equal to any string, so the exact spelling is a stub
    convention — it just has to be *one* convention, which it was not:
    four directories spelled the same enums four different ways.
    """

    RELIABLE = "reliable"
    BEST_EFFORT = "best_effort"
    TRANSIENT_LOCAL = "transient_local"
    VOLATILE = "volatile"
    KEEP_LAST = "keep_last"
    KEEP_ALL = "keep_all"
    AUTOMATIC = "automatic"


def _qos_module() -> types.SimpleNamespace:
    """``rclpy.qos`` — every policy the voice nodes actually import.

    ``DurabilityPolicy`` is required by ``dialogue_node`` (latched
    ``barge_in_policy`` topic, issue #1734), ``sound_node`` and the latched
    ``/mcp/tools`` catalog. Missing it is an ImportError at module scope, so
    it takes out every test in the directory rather than one assertion.
    """
    return types.SimpleNamespace(
        HistoryPolicy=_Policy,
        ReliabilityPolicy=_Policy,
        DurabilityPolicy=_Policy,
        LivelinessPolicy=_Policy,
        QoSProfile=FakeQoSProfile,
    )


def _build_stubs() -> dict[str, object]:
    mock_rclpy_node = MagicMock()
    mock_rclpy_node.Node = FakeNode

    mock_std_msgs_msg = MagicMock()
    # Each message must be a *class*, not a MagicMock instance: calling a
    # MagicMock returns the same `return_value` object every time, so
    # `String()` handed every publisher the one shared message and a test
    # reading `publish.call_args[0][0].data` saw whatever was written last
    # («thinking» where it asserted «STOP», batch_index 2 where it asserted
    # 1). A plain class with the field defaulted is what a real msg behaves
    # like here.
    for message, fields in (
        ("String", {"data": ""}),
        ("Bool", {"data": False}),
        ("Float32", {"data": 0.0}),
        ("Int32", {"data": 0}),
        ("Empty", {}),
    ):
        setattr(mock_std_msgs_msg, message, type(message, (), dict(fields)))
    mock_std_msgs = MagicMock()
    mock_std_msgs.msg = mock_std_msgs_msg

    return {
        "rclpy": MagicMock(),
        "rclpy.node": mock_rclpy_node,
        "rclpy.qos": _qos_module(),
        "rclpy.callback_groups": types.SimpleNamespace(
            ReentrantCallbackGroup=type("ReentrantCallbackGroup", (), {}),
            MutuallyExclusiveCallbackGroup=type(
                "MutuallyExclusiveCallbackGroup", (), {}
            ),
        ),
        "rclpy.executors": MagicMock(),
        "std_msgs": mock_std_msgs,
        "std_msgs.msg": mock_std_msgs_msg,
    }


def install_ros_stubs(
    extra: dict[str, object] | None = None,
    names: tuple[str, ...] | None = None,
) -> None:
    """Install the shared ROS2 stubs, completing any partial ones already there.

    Safe to call repeatedly and from any number of conftests: a module that
    is already registered keeps its own attributes, and only names it is
    missing are copied across. Pass *extra* for package-specific stubs (audio
    backends, vendor SDKs) that only one test tree needs.

    *names* restricts the install to specific modules. The root conftest uses
    it to publish only ``rclpy.qos``: that module is pure data, so completing
    it is always safe, whereas ``rclpy.node`` carries *behaviour* — several
    test directories assert against their own ``FakeNode`` (recording
    publishers, capturing parameters), and claiming that name from the root
    would replace the fake those tests are written against.
    """
    stubs = {**_build_stubs(), **(extra or {})}
    if names is not None:
        stubs = {name: stub for name, stub in stubs.items() if name in names}
    for name, stub in stubs.items():
        existing = sys.modules.get(name)
        if existing is None:
            sys.modules[name] = stub
            continue
        # Another stub got here first (or a real ROS2 is installed). Fill in
        # only what it lacks, so partial stubs stop breaking unrelated imports.
        for attribute in dir(stub):
            if attribute.startswith("__"):
                continue
            if not hasattr(existing, attribute):
                try:
                    setattr(existing, attribute, getattr(stub, attribute))
                except (AttributeError, TypeError):
                    # Real modules can be read-only; nothing to do.
                    pass


def qos_stub() -> types.SimpleNamespace:
    """The shared ``rclpy.qos`` stub, for conftests that build their own set."""
    return _qos_module()
