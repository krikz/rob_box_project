#!/usr/bin/env python3
"""Issue #5061 — a swallowed Conflict must not leave polling dead.

python-telegram-bot catches ``telegram.error.Conflict`` inside its own polling
task and never raises it to the caller, so the node's loop used to sleep
forever on a channel that accepts nothing. These tests drive the real
``_run_telegram`` / ``_run_telegram_loop`` with a fake updater:

* the polling task dies with Conflict — the loop logs it in plain words and
  starts polling again;
* a second instance in the same container hits the file lock and never calls
  ``start_polling``.
"""

from __future__ import annotations

import asyncio
import os
import sys
import types
import unittest
from unittest.mock import AsyncMock, MagicMock, patch

# The bridge tests install fakes for rclpy/telegram at import time and import
# the node module once. Reuse that shim instead of a second, divergent one.
sys.path.insert(0, os.path.dirname(__file__))
import test_telegram_bridge as bridge  # noqa: E402

bridge._install_fake_rclpy()
bridge._install_fake_telegram()

# The bridge's telegram.error stub has no Conflict; add the one the guard
# recognises. The class name is what polling_guard.is_conflict matches on,
# same as the real telegram.error.Conflict.
_error_mod = sys.modules["telegram.error"]


class Conflict(Exception):
    """Stand-in for ``telegram.error.Conflict`` (HTTP 409)."""


_error_mod.Conflict = Conflict


# The bridge stub's filters know VOICE/TEXT/COMMAND only; the node also
# registers PHOTO and Document.ALL. Fill them in rather than forking the stub.
class _Filter:
    """Stand-in for the operators the node combines filters with (``|``, ``~``)."""

    def __or__(self, _other):
        return self

    def __invert__(self):
        return self

    def __and__(self, _other):
        return self


_filters = sys.modules["telegram.ext"].filters
_filters.PHOTO = _Filter()
_filters.Document = types.SimpleNamespace(ALL=_Filter())
_filters.COMMAND = _Filter()
_filters.TEXT = _Filter()

import rob_box_telegram.telegram_node as node_mod  # noqa: E402
from rob_box_telegram import polling_guard  # noqa: E402
from rob_box_telegram.telegram_node import TelegramNode  # noqa: E402


class _DyingUpdater:
    """Updater whose polling task dies the way the library's does on Conflict.

    ``start_polling`` creates the task and returns, as python-telegram-bot's
    ``Updater.start_polling`` does. The task then raises — the failure the
    live bot saw on 21.11, which the library logs and swallows.
    """

    def __init__(self, deaths):
        self._deaths = list(deaths)
        self.start_calls = 0
        self._polling_task = None

    async def start_polling(self, *args, **kwargs):
        self.start_calls += 1
        death = self._deaths.pop(0)

        async def die():
            raise death

        self._polling_task = asyncio.create_task(die())

    async def stop(self):
        return None


class _QuietUpdater:
    """Polling task that ends cleanly — the library gave up without raising."""

    def __init__(self):
        self.start_calls = 0
        self._polling_task = None

    async def start_polling(self, *args, **kwargs):
        self.start_calls += 1
        self._polling_task = asyncio.create_task(asyncio.sleep(0))

    async def stop(self):
        return None


class _FiniteOk:
    """``rclpy.ok()`` that turns false after a fixed number of checks."""

    def __init__(self, checks: int):
        self._left = checks

    def __call__(self):
        self._left -= 1
        return self._left >= 0


def _app_with(updater):
    app = MagicMock(name="Application")
    app.bot = MagicMock(name="bot")
    app.bot_data = {}
    app.initialize = AsyncMock()
    app.start = AsyncMock()
    app.stop = AsyncMock()
    app.shutdown = AsyncMock()
    app.add_handler = MagicMock()
    app.updater = updater
    return app


def _builder_for(app):
    builder = MagicMock()
    builder.token.return_value = builder
    builder.build.return_value = app
    return builder


def _logged(node) -> str:
    return " ".join(str(call) for call in node.get_logger.return_value.error.call_args_list)


def _node_for_run():
    """A real node with everything ``_run_telegram`` touches stubbed.

    ``TelegramNode.__new__`` skips ``Node.__init__``: the coroutine only needs
    the logger, the echo worker and the attributes it sets itself.
    """
    node = TelegramNode.__new__(TelegramNode)
    node.get_logger = MagicMock(return_value=MagicMock())
    node._retry_sleep = lambda seconds: 0.0

    async def _quiet_worker():
        await asyncio.sleep(3600)

    node._chat_echo_worker = _quiet_worker
    return node


class ConflictRecoveryTest(unittest.TestCase):
    def test_swallowed_conflict_retries_and_logs(self):
        updater = _DyingUpdater([Conflict("terminated by other getUpdates request")] * 5)
        node = _node_for_run()
        # The lock must be free: this drives the real outer loop, which takes it.
        with patch.object(polling_guard, "POLLING_LOCK_PATH", f"/tmp/lock5061_retry_{os.getpid()}"), patch.object(
            node_mod.Application, "builder", return_value=_builder_for(_app_with(updater))
        ), patch.object(node_mod, "publish_menu", AsyncMock()), patch.object(node_mod.rclpy, "ok", _FiniteOk(4)):
            TelegramNode._run_telegram_loop(node, "token")

        self.assertGreaterEqual(updater.start_calls, 2, "polling was not restarted after Conflict")
        logged = _logged(node)
        self.assertIn("Conflict (409)", logged)
        self.assertIn("another getUpdates", logged)
        self.assertLess(len(updater._deaths), 5, "retried forever instead of stopping when rclpy stops")

    def test_clean_task_end_also_restarts(self):
        updater = _QuietUpdater()
        node = _node_for_run()
        with patch.object(polling_guard, "POLLING_LOCK_PATH", f"/tmp/lock5061_quiet_{os.getpid()}"), patch.object(
            node_mod.Application, "builder", return_value=_builder_for(_app_with(updater))
        ), patch.object(node_mod, "publish_menu", AsyncMock()), patch.object(node_mod.rclpy, "ok", _FiniteOk(4)):
            TelegramNode._run_telegram_loop(node, "token")

        self.assertGreaterEqual(updater.start_calls, 2, "a dead polling task left the channel mute")
        self.assertIn("ended on its own", _logged(node))

    def test_no_polling_task_keeps_idling(self):
        """An updater that hides its task (older library) must not spin the restart."""
        updater = MagicMock()
        updater.start_polling = AsyncMock()
        updater.stop = AsyncMock()
        del updater._polling_task
        node = _node_for_run()
        with patch.object(node_mod.Application, "builder", return_value=_builder_for(_app_with(updater))), patch.object(
            node_mod, "publish_menu", AsyncMock()
        ), patch.object(node_mod.rclpy, "ok", _FiniteOk(3)):
            asyncio.run(TelegramNode._run_telegram(node, "token"))
        self.assertEqual(updater.start_polling.await_count, 1)
        node.get_logger.return_value.error.assert_not_called()


class PollingLockTest(unittest.TestCase):
    def setUp(self):
        self._dir = os.path.join("/tmp", f"lock5061_{os.getpid()}_{id(self)}")
        os.makedirs(self._dir, exist_ok=True)
        self._path = os.path.join(self._dir, "polling.lock")

    def tearDown(self):
        for name in os.listdir(self._dir):
            os.remove(os.path.join(self._dir, name))
        os.rmdir(self._dir)

    def test_second_instance_gets_the_reason(self):
        first = polling_guard.PollingLock(self._path)
        first.acquire()
        second = polling_guard.PollingLock(self._path)
        with self.assertRaises(polling_guard.PollingLockHeld) as caught:
            second.acquire()
        self.assertIn("will NOT call getUpdates", str(caught.exception))
        self.assertIn(self._path, str(caught.exception))
        first.release()
        second.acquire()  # free again once the holder releases it
        second.release()

    def test_second_node_never_builds_an_application(self):
        held = polling_guard.PollingLock(self._path)
        held.acquire()
        self.addCleanup(held.release)
        node = MagicMock()
        node.get_logger.return_value = MagicMock()
        with patch.object(polling_guard, "POLLING_LOCK_PATH", self._path), patch.object(
            node_mod.Application, "builder"
        ) as builder:
            TelegramNode._run_telegram_loop(node, "token")
        builder.assert_not_called()
        self.assertIn("will NOT call getUpdates", _logged(node))


class GuardHelpersTest(unittest.TestCase):
    def test_backoff_grows_and_caps(self):
        self.assertEqual(polling_guard.conflict_backoff_s(1), 5.0)
        self.assertEqual(polling_guard.conflict_backoff_s(2), 10.0)
        self.assertEqual(polling_guard.conflict_backoff_s(4), 40.0)
        self.assertEqual(polling_guard.conflict_backoff_s(8), 60.0)

    def test_is_conflict_sees_wrapped_cause(self):
        outer = RuntimeError("polling task died")
        outer.__cause__ = Conflict("409")
        self.assertTrue(polling_guard.is_conflict(outer))
        self.assertFalse(polling_guard.is_conflict(RuntimeError("other")))
        self.assertFalse(polling_guard.is_conflict(None))

    def test_watch_polling_reports_death_and_survives_a_quiet_tick(self):
        async def scenario():
            async def die():
                raise Conflict("409")

            dead = asyncio.create_task(die())
            await asyncio.sleep(0)
            quiet = asyncio.create_task(asyncio.sleep(5))
            died = await polling_guard.watch_polling(dead, poll_s=1.0)
            alive = await polling_guard.watch_polling(quiet, poll_s=0.01)
            quiet.cancel()
            return died, alive

        died, alive = asyncio.run(scenario())
        self.assertTrue(died)
        self.assertFalse(alive)


if __name__ == "__main__":
    unittest.main()
