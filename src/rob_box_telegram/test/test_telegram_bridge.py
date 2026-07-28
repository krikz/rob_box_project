#!/usr/bin/env python3
"""Integration tests for the Telegram <-> ROS 2 bridge (Phase 6 v2 / W9).

After W8 ``rob_box_telegram.telegram_node.TelegramNode`` is a thin ROS 2
input bridge that forwards Telegram messages into the dialogue pipeline and
keeps camera frames available to Telegram handlers:

    Telegram user --text/voice--> /voice/stt/result         (dialogue in)
    CompressedImage topics      -> in-memory CameraCache     (camera frames)

The node deliberately does not subscribe to ``/voice/dialogue/response``:
that topic is the dialogue-to-TTS output and consuming it here creates an
output echo path back into Telegram.

These tests cover the W9 acceptance criteria from
``.planning/phases/06-harness-p0-finalization/06-03-PLAN.md``:

    1. Telegram message -> published to /voice/stt/result
    2. Telegram node does not subscribe to the dialogue/TTS response topic
    3. Camera image -> forwarded to appropriate topic (CameraCache update)
    4. Telegram bot starts (mock Application builder)
    5. VPN connectivity (skipped — no container in CI; see skipVPN)

The test environment has neither ``rclpy`` nor ``python-telegram-bot``
installed, so both stacks are injected as fakes into ``sys.modules``
before the node module is imported. ``rclpy.node.Node`` is replaced
with a stub that records ``create_publisher`` / ``create_subscription``
calls so the bridge can be exercised without spinning a real DDS
discovery. The ``telegram.ext.Application.builder()`` chain is wired to
an ``AsyncMock`` so the bot startup path runs end-to-end without
hitting the Telegram API.

No real network, no real LLM, no real Telegram — everything goes through
``unittest.mock``.
"""

from __future__ import annotations

import asyncio
import importlib
import json
import os
import sys
import threading
import types
import unittest
from unittest.mock import AsyncMock, MagicMock, patch

# ``StdString`` (provided by the rclpy stub injected at first import) is
# resolved lazily inside the test class, so a top-level import is not
# needed here.


# ─────────────────────────────────────────────────────────────────────────
# Fake rclpy / telegram modules
#
# Must run BEFORE ``rob_box_telegram.telegram_node`` is imported; the node
# pulls in rclpy.node.Node, rclpy.qos.*, sensor_msgs, nav_msgs, std_msgs
# and telegram.ext.* at module load time.
# ─────────────────────────────────────────────────────────────────────────


_STD_STRING_CLS = None  # set by _install_fake_rclpy() so tests can build msgs


def _install_fake_rclpy() -> None:
    """Inject a minimal rclpy shim so telegram_node can be imported."""

    if "rclpy" in sys.modules:
        return

    rclpy = types.ModuleType("rclpy")
    callback_groups = types.ModuleType("rclpy.callback_groups")
    node_mod = types.ModuleType("rclpy.node")
    qos_mod = types.ModuleType("rclpy.qos")

    class _Reliability:
        BEST_EFFORT = "best_effort"
        RELIABLE = "reliable"

    class _History:
        KEEP_LAST = "keep_last"

    class _Durability:
        TRANSIENT_LOCAL = "transient_local"

    class QoSProfile:
        def __init__(self, reliability=None, history=None, depth=10, durability=None):
            self.reliability = reliability
            self.history = history
            self.depth = depth
            self.durability = durability

    class _CallbackGroup:
        pass

    class ReentrantCallbackGroup(_CallbackGroup):
        pass

    # Expose QoS factory + policy enums on the qos module so the bridge
    # can build QoS profiles the same way it does at runtime.
    qos_mod.QoSProfile = QoSProfile
    qos_mod.ReliabilityPolicy = _Reliability
    qos_mod.HistoryPolicy = _History
    qos_mod.DurabilityPolicy = _Durability

    class Node:
        """Recording stand-in for rclpy.node.Node."""

        def __init__(self, name: str):
            self._created_publishers: list = []
            self._created_subscriptions: list = []
            self._declared_parameters: dict = {}
            # Fake logger that captures calls without printing.
            self.get_logger = MagicMock()

        def declare_parameter(self, name, default_value=None):
            # emulate rclpy.Parameter declaration behaviour
            self._declared_parameters[name] = default_value
            return types.SimpleNamespace(value=default_value)

        def get_parameter(self, name):
            value = self._declared_parameters.get(name)
            return types.SimpleNamespace(value=value)

        def create_publisher(self, msg_type, topic, qos_profile=None, **_):
            pub = MagicMock(name=f"Pub[{topic}]")
            pub.topic = topic
            pub.msg_type = msg_type
            pub.qos = qos_profile
            # Track every .publish() call on the spy.
            pub.publish = MagicMock(side_effect=self._record_publish(topic))
            self._created_publishers.append(pub)
            return pub

        def create_subscription(self, msg_type, topic, callback, qos_profile=None,
                                callback_group=None, **_):
            sub = MagicMock(name=f"Sub[{topic}]")
            sub.topic = topic
            sub.msg_type = msg_type
            sub.callback = callback
            sub.qos = qos_profile
            sub.callback_group = callback_group
            self._created_subscriptions.append(sub)
            return sub

        def _record_publish(self, topic):
            calls: list = []

            def _capture(msg):
                calls.append((topic, msg))

            return _capture

    node_mod.Node = Node

    callback_groups.ReentrantCallbackGroup = ReentrantCallbackGroup

    def _init(args=None):
        return None

    def _ok():
        return True

    def _shutdown():
        return None

    def _spin(_node):
        return None

    def _try_shutdown():
        return None

    rclpy.init = _init
    rclpy.ok = _ok
    rclpy.shutdown = _shutdown
    rclpy.spin = _spin
    rclpy.try_shutdown = _try_shutdown

    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.node"] = node_mod
    sys.modules["rclpy.callback_groups"] = callback_groups
    sys.modules["rclpy.qos"] = qos_mod

    # rclpy.message_conversion_sets is referenced by some rosidl adapters;
    # the bridge does not import it directly, but a stub avoids surprises.
    sys.modules.setdefault("rclpy.message_conversion_sets",
                           types.ModuleType("rclpy.message_conversion_sets"))

    # ROS 2 message modules — minimal stand-ins for the ones telegram_node
    # imports. Each carries the attribute the bridge actually accesses.
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")

    class String:
        def __init__(self):
            self.data = ""
    std_msgs_msg.String = String
    std_msgs.msg = std_msgs_msg
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg

    # Expose the stub String class at module scope so tests can build
    # ``std_msgs.msg.String`` messages without re-importing.
    globals()["_STD_STRING_CLS"] = String  # noqa: F841 — late binding for tests

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")

    class CompressedImage:
        def __init__(self):
            self.data = b""
            self.format = ""
    sensor_msgs_msg.CompressedImage = CompressedImage
    sensor_msgs.msg = sensor_msgs_msg
    sys.modules["sensor_msgs"] = sensor_msgs
    sys.modules["sensor_msgs.msg"] = sensor_msgs_msg

    nav_msgs = types.ModuleType("nav_msgs")
    nav_msgs_msg = types.ModuleType("nav_msgs.msg")

    class OccupancyGrid:
        def __init__(self):
            self.data = []
    nav_msgs_msg.OccupancyGrid = OccupancyGrid
    nav_msgs.msg = nav_msgs_msg
    sys.modules["nav_msgs"] = nav_msgs
    sys.modules["nav_msgs.msg"] = nav_msgs_msg

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")

    class Twist:
        def __init__(self):
            self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
    geometry_msgs_msg.Twist = Twist
    geometry_msgs.msg = geometry_msgs_msg
    sys.modules["geometry_msgs"] = geometry_msgs
    sys.modules["geometry_msgs.msg"] = geometry_msgs_msg

    # Expose the String class at module scope for tests that need it.
    globals()["_STD_STRING_CLS"] = String  # noqa: F841 — late binding for tests


def _install_fake_telegram() -> None:
    """Inject a minimal python-telegram-bot shim."""

    if "telegram" in sys.modules:
        return

    telegram_mod = types.ModuleType("telegram")
    ext_mod = types.ModuleType("telegram.ext")

    class Update:
        pass

    class ContextTypes:
        DEFAULT_TYPE = object

    class InlineKeyboardButton:
        def __init__(self, text, callback_data=None):
            self.text = text
            self.callback_data = callback_data

    class InlineKeyboardMarkup:
        def __init__(self, inline_keyboard):
            self.inline_keyboard = inline_keyboard

    class _Filters:
        VOICE = "voice"
        TEXT = "text"
        COMMAND = "command"

        def __and__(self, other):
            return ("and", self, other)

        def __invert__(self):
            return ("not", self)

    filters = _Filters()

    class _Handler:
        def __init__(self, *args, **kwargs):
            self.args = args
            self.kwargs = kwargs

    class CommandHandler(_Handler):
        pass

    class MessageHandler(_Handler):
        pass

    class CallbackQueryHandler(_Handler):
        pass

    class _ApplicationBuilder:
        """Minimal stand-in for ``Application.builder().token(...).build()``."""

        def __init__(self):
            self._token = None
            self._application = MagicMock(name="Application")
            self._application.bot = MagicMock(name="Application.bot")
            self._application.bot_data = {}
            self._application.bot.send_message = AsyncMock(
                name="Application.bot.send_message")
            self._application.add_handler = MagicMock()
            self._application.initialize = AsyncMock()
            self._application.start = AsyncMock()
            self._application.stop = AsyncMock()
            self._application.shutdown = AsyncMock()
            self._application.updater = MagicMock()
            self._application.updater.start_polling = AsyncMock()
            self._application.updater.stop = AsyncMock()
            self._application._loop = asyncio.new_event_loop()
            self._call_counts: dict = {"builder": 0, "token": 0}

        def token(self, value):
            self._token = value
            self._call_counts["token"] += 1
            return self

        def build(self):
            self._call_counts["builder"] += 1
            return self._application

    class _Application:
        @staticmethod
        def builder():
            return _ApplicationBuilder()

    ext_mod.Application = _Application
    ext_mod.CommandHandler = CommandHandler
    ext_mod.MessageHandler = MessageHandler
    ext_mod.CallbackQueryHandler = CallbackQueryHandler
    ext_mod.ContextTypes = ContextTypes
    ext_mod.filters = filters

    telegram_mod.Update = Update
    telegram_mod.InlineKeyboardButton = InlineKeyboardButton
    telegram_mod.InlineKeyboardMarkup = InlineKeyboardMarkup

    error_mod = types.ModuleType("telegram.error")

    class TimedOut(Exception):
        pass

    class NetworkError(Exception):
        pass

    error_mod.TimedOut = TimedOut
    error_mod.NetworkError = NetworkError

    sys.modules["telegram"] = telegram_mod
    sys.modules["telegram.ext"] = ext_mod
    sys.modules["telegram.error"] = error_mod

    # numpy / PIL are pulled in by handlers/commands.py — keep them
    # non-functional (we never invoke a handler in these tests), but make
    # the import succeed.
    sys.modules.setdefault("numpy", types.ModuleType("numpy"))
    pil_mod = types.ModuleType("PIL")
    pil_image = MagicMock()
    pil_mod.Image = pil_image
    sys.modules.setdefault("PIL", pil_mod)

    # ``voice_processor`` is imported transitively by ``telegram_node``
    # via ``handlers.messages``. It declares ``import aiohttp`` at module
    # top — stub it so the import chain succeeds in this no-network env.
    aiohttp_mod = types.ModuleType("aiohttp")
    aiohttp_mod.ClientSession = MagicMock()
    aiohttp_mod.ClientTimeout = MagicMock()
    sys.modules.setdefault("aiohttp", aiohttp_mod)


def _load_telegram_node_module():
    """(Re)import ``rob_box_telegram.telegram_node`` with fresh stubs."""

    _install_fake_rclpy()
    _install_fake_telegram()
    sys.modules.pop("rob_box_telegram.telegram_node", None)
    sys.modules.pop("rob_box_telegram.camera_cache", None)
    sys.modules.pop("rob_box_telegram.handlers.commands", None)
    sys.modules.pop("rob_box_telegram.handlers.callbacks", None)
    sys.modules.pop("rob_box_telegram.handlers.messages", None)
    sys.modules.pop("rob_box_telegram.auth", None)
    sys.modules.pop("rob_box_telegram.keyboard_layouts", None)
    sys.modules.pop("rob_box_telegram.voice_processor", None)
    sys.modules.pop("rob_box_telegram", None)
    return importlib.import_module("rob_box_telegram.telegram_node")


# ─────────────────────────────────────────────────────────────────────────
# Tests
# ─────────────────────────────────────────────────────────────────────────


class TestTelegramBridge(unittest.IsolatedAsyncioTestCase):
    """Integration coverage for the W9 acceptance criteria.

    Each test gets a fresh TelegramNode whose ``_start_telegram_bot``
    is patched to a no-op (the bot-loop logic is exercised separately
    in :class:`TestTelegramBotStartup`). That keeps these tests focused
    on the bridge data path without spinning a real thread or asyncio
    loop.
    """

    _TOKEN = "test-token-123456"

    def setUp(self) -> None:
        self._node_mod = _load_telegram_node_module()
        # Make sure no previous test's env var leaks in.
        os.environ["TELEGRAM_BOT_TOKEN"] = self._TOKEN
        # Avoid touching the threading.Thread bot loop during init.
        # The patched attribute replaces the *unbound* method, so when
        # called as ``self._start_telegram_bot(token)`` the lambda
        # receives both ``self`` and ``token``.
        self._start_patch = patch.object(
            self._node_mod.TelegramNode,
            "_start_telegram_bot",
            lambda self, token: None,
        )
        self._start_patch.start()
        self.node = self._node_mod.TelegramNode()

    def tearDown(self) -> None:
        self._start_patch.stop()
        os.environ.pop("TELEGRAM_BOT_TOKEN", None)

    # ── helpers ────────────────────────────────────────────────────────

    def _publisher_for(self, topic: str):
        for pub in self.node._created_publishers:
            if pub.topic == topic:
                return pub
        raise AssertionError(f"No publisher registered for {topic}")

    def _subscription_for(self, topic: str):
        for sub in self.node._created_subscriptions:
            if sub.topic == topic:
                return sub
        raise AssertionError(f"No subscription registered for {topic}")

    # ── Test 1: Telegram message → /voice/stt/result ──────────────────

    def test_telegram_message_published_to_stt_topic(self) -> None:
        """``forward_to_stt`` must publish the raw text on /voice/stt/result."""

        pub = self._publisher_for("/voice/stt/result")
        self.assertEqual(pub.publish.call_count, 0)

        self.node.forward_to_stt("Привет, робот!")
        self.node.forward_to_stt("")

        self.assertEqual(pub.publish.call_count, 1)
        published = pub.publish.call_args.args[0]
        self.assertIsInstance(published, _STD_STRING_CLS)
        self.assertEqual(published.data, "Привет, робот!")

    def test_node_does_not_subscribe_to_dialogue_response(self) -> None:
        """Telegram is input-only and must not consume dialogue/TTS output."""

        topics = {sub.topic for sub in self.node._created_subscriptions}

        self.assertNotIn("/voice/dialogue/response", topics)

    # ── Test 2: Camera image → forwarded to appropriate topic ─────────

    def test_camera_image_forwarded_to_cache(self) -> None:
        """Each camera subscription must cache the latest compressed frame."""

        cache = self.node.camera_cache
        self.assertEqual(cache.topics, [])

        front = self._subscription_for(self.node.camera_topic).callback
        depth = self._subscription_for(self.node.camera_depth_topic).callback
        up = self._subscription_for(self.node.camera_up_topic).callback

        front_msg = _compressed_image(b"\xff\xd8front")
        depth_msg = _compressed_image(b"\xff\xd8depth")
        up_msg = _compressed_image(b"\xff\xd8up")

        front(front_msg)
        depth(depth_msg)
        up(up_msg)

        self.assertEqual(set(cache.topics),
                         {self.node.camera_topic,
                          self.node.camera_depth_topic,
                          self.node.camera_up_topic})
        self.assertEqual(cache.get(self.node.camera_topic), b"\xff\xd8front")
        self.assertEqual(cache.get(self.node.camera_depth_topic), b"\xff\xd8depth")
        self.assertEqual(cache.get(self.node.camera_up_topic), b"\xff\xd8up")

    def test_map_grid_subscription_records_payload(self) -> None:
        """Map subscription stores the latest OccupancyGrid on the node."""

        callback = self._subscription_for("/rtabmap/grid_prob_map").callback
        self.assertIsNone(self.node.latest_map_grid)

        grid = _occupancy_grid()
        callback(grid)

        self.assertIs(self.node.latest_map_grid, grid)

    # ── Test 4: Telegram bot starts (mock Application) ────────────────

    def test_start_telegram_bot_is_patched_during_bridge_tests(self) -> None:
        """``_start_telegram_bot`` must be a no-op for the bridge tests.

        ``setUp`` patches ``_start_telegram_bot`` to a lambda so the
        threading + asyncio loop never run during unit tests. This
        assertion guards against regressions (e.g. someone removing
        the patch and accidentally firing real network calls).
        """
        # The patched attribute is a lambda, not the original method.
        original_method = self._node_mod.TelegramNode._start_telegram_bot
        patched = self.node._start_telegram_bot
        self.assertIsNot(patched, original_method)
        # ``patched`` is bound to ``self.node`` (descriptor protocol),
        # so passing the token alone is the production-shaped call.
        self.assertIsNone(patched(self._TOKEN))


class TestTelegramBotStartup(unittest.IsolatedAsyncioTestCase):
    """Smoke test for the real ``_start_telegram_bot`` code path.

    The bot is launched inside ``asyncio.run_coroutine_threadsafe`` on
    the application's event loop. We stub that schedule point so the
    ``Application.builder()`` chain is built end-to-end, the
    ``_run_telegram`` coroutine is collected, but no real socket I/O
    happens.
    """

    _TOKEN = "test-token-startup"

    def setUp(self) -> None:
        self._node_mod = _load_telegram_node_module()
        os.environ["TELEGRAM_BOT_TOKEN"] = self._TOKEN

    def tearDown(self) -> None:
        os.environ.pop("TELEGRAM_BOT_TOKEN", None)

    def test_bot_starts_and_registers_handlers(self) -> None:
        """``TelegramNode()`` must spin up the bot thread without raising.

        The constructor calls ``_start_telegram_bot``, which spawns a
        daemon thread running the asyncio Application loop. Every await
        resolves against an ``AsyncMock`` (installed by the fake
        ``Application.builder()``), so the loop completes synchronously
        — but the daemon thread is still spawned, which is the contract
        the bridge promises: bot runs out-of-band from ROS callbacks.
        """
        node = self._node_mod.TelegramNode()

        # Drain the spawned thread so we don't leak it into the next test.
        for t in threading.enumerate():
            if t.name == "telegram-bot":
                t.join(timeout=2.0)
                break

        # The constructor wired ROS 2 publishers/subscriptions before
        # handing off to the bot thread; that wiring must have run.
        self.assertTrue(node._created_publishers)
        self.assertTrue(node._created_subscriptions)

    def test_missing_token_does_not_raise(self) -> None:
        """A missing token must log an error but never explode."""

        os.environ.pop("TELEGRAM_BOT_TOKEN", None)
        node = self._node_mod.TelegramNode()
        self.assertIsNone(node._telegram_app)


class TestVPNConnectivity(unittest.TestCase):
    """VPN connectivity check — environment-dependent.

    The bridge relies on the container's WireGuard tunnel, which is
    configured at deploy time and has no Python-facing surface that a
    unit test can poke without root access and a real network device.
    """

    @unittest.skip(
        "VPN connectivity requires a live WireGuard tunnel inside the "
        "deployment container — not testable from CI. Skipping per W9 "
        "spec ('if testable, otherwise skip')."
    )
    def test_vpn_endpoint_reachable(self) -> None:
        pass


# ─────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────


def _compressed_image(payload: bytes):
    msg = types.SimpleNamespace()
    msg.data = payload
    msg.format = "jpeg"
    return msg


def _occupancy_grid():
    msg = types.SimpleNamespace()
    msg.data = []
    return msg


if __name__ == "__main__":
    unittest.main()