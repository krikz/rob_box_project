#!/usr/bin/env python3
"""Tests for TelegramNode ↔ SupervisorClient integration (AV-10, ADR-0028 §4.4).

Verifies the *acquire → publish → release* contract for both
``teleop`` and ``voice`` floors without requiring a live ROS 2
environment. ``SupervisorClient`` exposes ``set_mock_response`` and
``set_test_mode`` hooks exactly for this kind of unit test; we use
them to script deterministic service-call responses.

The tests are deliberately permissive about *how* the floor is held
(monitor mode grants locally, active mode would service-call) — what
matters is:

* every ``publish_*_with_floor`` invocation contacts the supervisor,
* if the floor is denied, no ROS publication happens,
* if granted, exactly one publication happens,
* release runs whether or not the callback raised.

NOTE: the actual ``publish_*_with_floor`` wrappers live in
``telegram_node.py`` (they import rclpy). They are not exercised
directly here — the floor-gating logic is encapsulated in
``SupervisorClient.with_floor``, and the wrappers are thin enough
that smoke-test on a real device (W8 deploy) is sufficient. Full
coverage of the wrapper semantics would require installing rclpy +
python-telegram-bot in CI; we keep this file dependency-free.
"""

from __future__ import annotations

import unittest

from rob_box_telegram.supervisor_client import (
    AcquireResult,
    AvatarState,
    Floor,
    SupervisorClient,
)


class _StubNode:
    """Minimal stand-in for a ``rclpy.node.Node``.

    ``SupervisorClient`` only uses ``self._node.create_publisher`` /
    ``create_subscription`` / ``create_timer`` when ``mode="active"``
    *and* rclpy is importable; these tests run with ``mode="monitor"``
    so we never touch ROS. The stubs below raise clearly if a test
    accidentally crosses the boundary.
    """

    def create_publisher(self, *args, **kwargs):  # pragma: no cover - safety
        raise AssertionError("monitor mode must not touch ROS publishers")

    def create_subscription(self, *args, **kwargs):  # pragma: no cover
        raise AssertionError("monitor mode must not touch ROS subscriptions")

    def create_timer(self, *args, **kwargs):  # pragma: no cover
        raise AssertionError("monitor mode must not touch ROS timers")


class TestSupervisorClientMonitor(unittest.TestCase):
    """Phase 1 default behaviour: monitor mode grants locally."""

    def setUp(self) -> None:
        self.node = _StubNode()
        self.client = SupervisorClient(
            node=self.node, client_id="telegram", mode="monitor"
        )

    def tearDown(self) -> None:
        self.client.reset_test_hooks()

    def test_monitor_mode_grants_locally(self) -> None:
        """monitor mode = Phase 1 default (ADR-0028 §4.5)."""
        result = self.client.acquire_floor(Floor.TELEOP)
        self.assertTrue(result.granted)
        self.assertFalse(result.contacted_service)
        self.client.release_floor(Floor.TELEOP)

    def test_monitor_mode_state_has_no_holders(self) -> None:
        """В monitor state.teleop_floor == None → UI gate не блокирует."""
        st = self.client.state
        self.assertIsNone(st.teleop_floor)
        self.assertIsNone(st.voice_floor)


class TestSupervisorClientActiveMode(unittest.TestCase):
    """Phase 2 behaviour: active mode contacts a (mocked) service."""

    def setUp(self) -> None:
        self.node = _StubNode()
        self.client = SupervisorClient(
            node=self.node, client_id="telegram", mode="active"
        )

    def tearDown(self) -> None:
        self.client.reset_test_hooks()

    def test_active_mode_falls_back_when_service_missing(self) -> None:
        """Without real service, active mode logs a warning and grants.

        This is the behaviour we ship *until* ``avatar_supervisor``
        lands (ADR-0028 §4.5). Keeps the bot working during the
        supervisor rollout.
        """
        result = self.client.acquire_floor(Floor.TELEOP)
        self.assertTrue(result.granted)
        self.assertFalse(result.contacted_service)
        self.client.release_floor(Floor.TELEOP)

    def test_set_test_mode_always_deny(self) -> None:
        """``always_deny`` short-circuits the service for tests."""
        self.client.set_test_mode("always_deny")
        result = self.client.acquire_floor(Floor.TELEOP)
        self.assertFalse(result.granted)
        self.assertEqual(result.denied_reason, "held_by_other")
        self.assertEqual(result.held_by, "quest")

    def test_set_test_mode_always_grant(self) -> None:
        self.client.set_test_mode("always_grant")
        result = self.client.acquire_floor(Floor.VOICE)
        self.assertTrue(result.granted)
        self.assertFalse(result.contacted_service)
        self.client.release_floor(Floor.VOICE)

    def test_mock_acquire_deny_prevents_callback(self) -> None:
        """When acquire is denied the inner callback must NOT run.

        Verifies the contract of ``with_floor``: acquire→publish→release
        is short-circuited on denied. TelegramNode relies on this to
        suppress ROS publications when the floor is held by another
        client (e.g. Quest).
        """
        self.client.set_mock_response(
            "acquire",
            lambda **kw: AcquireResult(
                granted=False,
                denied_reason="held_by_other",
                held_by="quest",
            ),
        )
        invoked = []

        def _inner() -> None:
            invoked.append(1)

        result = self.client.with_floor(Floor.TELEOP, _inner)
        self.assertFalse(result.granted)
        self.assertEqual(invoked, [])

    def test_mock_acquire_grant_runs_callback_then_release(self) -> None:
        """Grant → callback → release sequence with mock service.

        Primary acceptance test (AV-10): verify that on granted the
        callback runs *exactly once* and the release path is invoked
        with the correct floor.
        """
        released = []

        def _mock_acquire(**kw):
            return AcquireResult(granted=True, contacted_service=True)

        def _mock_release(**kw):
            released.append(kw["floor"])

        self.client.set_mock_response("acquire", _mock_acquire)
        self.client.set_mock_response("release", _mock_release)

        invoked = []

        def _inner() -> None:
            invoked.append("ok")

        result = self.client.with_floor(Floor.TELEOP, _inner)
        self.assertTrue(result.granted)
        self.assertEqual(invoked, ["ok"])
        self.assertEqual(released, [Floor.TELEOP])

    def test_with_floor_releases_even_on_callback_failure(self) -> None:
        """Exception inside callback still triggers release_floor."""
        released = []

        self.client.set_mock_response(
            "acquire",
            lambda **kw: AcquireResult(granted=True, contacted_service=True),
        )
        self.client.set_mock_response(
            "release", lambda **kw: released.append(kw["floor"])
        )

        def _boom() -> None:
            raise RuntimeError("simulated publish failure")

        with self.assertRaises(RuntimeError):
            self.client.with_floor(Floor.VOICE, _boom)
        self.assertEqual(released, [Floor.VOICE])


class TestSupervisorClientStateSubscription(unittest.TestCase):
    """State subscription: latched /avatar/state → UI gate."""

    def setUp(self) -> None:
        self.node = _StubNode()
        self.client = SupervisorClient(
            node=self.node, client_id="telegram", mode="monitor"
        )

    def tearDown(self) -> None:
        self.client.reset_test_hooks()

    def test_subscribe_state_initial_dispatch(self) -> None:
        """subscribe_state immediately calls listener with current state."""
        seen = []

        def _listener(state: AvatarState) -> None:
            seen.append(state)

        unsubscribe = self.client.subscribe_state(_listener)
        self.assertEqual(len(seen), 1)
        # В monitor режиме initial state имеет None holders — UI gate
        # не должен блокировать кнопки.
        self.assertIsNone(seen[0].teleop_floor)
        unsubscribe()

    def test_subscribe_state_unsubscribe_removes_listener(self) -> None:
        seen = []

        def _listener(state: AvatarState) -> None:
            seen.append(state)

        unsubscribe = self.client.subscribe_state(_listener)
        # initial dispatch → 1 вызов
        self.assertEqual(len(seen), 1)
        unsubscribe()
        # После unsubscribe initial dispatch не происходит для нового
        # подписчика, и старый listener не должен реагировать на новые
        # события. Тут только проверяем, что unsubscribe не падает.
        self.assertEqual(len(seen), 1)


class TestSupervisorClientShutdown(unittest.TestCase):
    """shutdown() releases held floors and stops heartbeat timer."""

    def setUp(self) -> None:
        self.node = _StubNode()
        self.client = SupervisorClient(
            node=self.node, client_id="telegram", mode="monitor"
        )

    def tearDown(self) -> None:
        self.client.reset_test_hooks()

    def test_shutdown_releases_held_floors(self) -> None:
        self.client.acquire_floor(Floor.TELEOP)
        self.client.acquire_floor(Floor.VOICE)
        self.assertTrue(self.client._is_holding(Floor.TELEOP))
        self.assertTrue(self.client._is_holding(Floor.VOICE))
        self.client.shutdown()
        self.assertFalse(self.client._is_holding(Floor.TELEOP))
        self.assertFalse(self.client._is_holding(Floor.VOICE))


if __name__ == "__main__":
    unittest.main()
