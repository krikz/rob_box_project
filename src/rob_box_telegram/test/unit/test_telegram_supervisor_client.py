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

import json
import logging
import threading
import time
import unittest
from typing import Any, Callable, List, Optional, Tuple
from unittest import mock

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


# ──────────────────────────────────────────────────────────────────────
# Fake ROS types for active-mode tests (AV-15 acceptance, ADR-0028 §4.4).
#
# Active-mode service-call tests need to bypass ``import rclpy`` /
# ``import std_srvs.srv`` (CI may not have those installed). The
# helpers below mimic the minimal surface that ``SupervisorClient``
# actually touches:
#
# * ``_FakeTrigger`` — type-like class with ``.Request()`` / ``.Response()``
#   factories. Setting arbitrary attributes on Request is allowed
#   (real ROS msg classes permit that).
# * ``_FakeRosString`` — type-like class with ``.data`` attribute; used
#   by _setup_active_mode for ``/avatar/state`` subscription.
# * ``_FakeFuture`` — wraps a callback that fires ``add_done_callback``
#   synchronously (or after a delay, or never). Supports the
#   ``future.result()`` / ``add_done_callback(cb)`` shape rclpy gives.
# * ``_FakeClient`` — mock of a ROS Client returned by
#   ``Node.create_client``; implements ``wait_for_service(timeout)``
#   and ``call_async(req)``.
# * ``_ActiveNode`` — full stand-in for a rclpy node that records
#   publishers/subscriptions/timers and serves service clients via
#   the supplied factory.
# ──────────────────────────────────────────────────────────────────────


class _FakeTriggerRequest:
    """Mimics ``std_srvs/srv/Trigger.Request``: arbitrary attrs allowed."""

    __slots__ = ("_data", "_attrs")

    def __init__(self) -> None:
        self._data: str = ""
        self._attrs: dict[str, Any] = {}

    @property
    def data(self) -> str:
        # Prefer explicit attribute (json fallback), then _attrs.data.
        return self._data if self._data else self._attrs.get("data", "")

    @data.setter
    def data(self, value: str) -> None:
        self._data = value
        self._attrs["data"] = value

    def __getattr__(self, name: str) -> Any:
        return self._attrs.get(name)

    def __setattr__(self, name: str, value: Any) -> None:
        # data is special; the rest go to _attrs for arbitrary attrs
        if name in ("_data", "_attrs"):
            object.__setattr__(self, name, value)
        else:
            self._attrs[name] = value


class _FakeTriggerResponse:
    """Mimics ``std_srvs/srv/Trigger.Response``."""

    def __init__(self, success: bool = True, message: str = "") -> None:
        self.success = success
        self.message = message


class _FakeTrigger:
    """Stand-in for ``std_srvs.srv.Trigger`` (type-like)."""

    Request = _FakeTriggerRequest
    Response = _FakeTriggerResponse


class _FakeRosString:
    """Stand-in for ``std_msgs.msg.String``."""

    def __init__(self) -> None:
        self.data: str = ""


class _FakeFuture:
    """Mimics ``rclpy.task.Future`` API."""

    def __init__(
        self,
        resolver: Callable[[], Tuple[Optional[Any], Optional[BaseException]]],
        schedule: str = "sync",
        delay_s: float = 0.0,
    ) -> None:
        self._resolver = resolver
        self._schedule = schedule
        self._delay_s = delay_s
        self._cbs: List[Callable[["_FakeFuture"], None]] = []
        self._done = False
        self._result: Optional[Any] = None
        self._exc: Optional[BaseException] = None
        self._timer: Optional[threading.Timer] = None

    def add_done_callback(self, cb: Callable[["_FakeFuture"], None]) -> None:
        self._cbs.append(cb)
        if self._done:
            cb(self)
            return
        if self._schedule == "sync":
            self._resolve()
        elif self._schedule == "delay":
            self._timer = threading.Timer(self._delay_s, self._resolve)
            self._timer.daemon = True
            self._timer.start()
        elif self._schedule == "never":
            return  # simulate a hanging service

    def _resolve(self) -> None:
        if self._done:
            return
        self._done = True
        try:
            resp, exc = self._resolver()
        except Exception as exc:  # noqa: BLE001
            self._exc = exc
        else:
            self._result = resp
            self._exc = exc
        for cb in list(self._cbs):
            try:
                cb(self)
            except Exception:  # noqa: BLE001
                pass

    def result(self) -> Any:
        if self._exc is not None:
            raise self._exc
        return self._result


class _FakeClient:
    """Stand-in for an rclpy Service client returned by ``create_client``."""

    def __init__(
        self,
        service_name: str,
        response_factory: Optional[Callable[[Any], _FakeFuture]] = None,
        wait_result: bool = True,
    ) -> None:
        self.service_name = service_name
        self.calls: List[Any] = []
        self._response_factory = response_factory or self._default_response
        self._wait_result = wait_result

    def wait_for_service(self, timeout_s: float) -> bool:
        return self._wait_result

    def call_async(self, request: Any) -> _FakeFuture:
        self.calls.append(request)
        return self._response_factory(request)

    def _default_response(self, _request: Any) -> _FakeFuture:
        return _FakeFuture(lambda: (_FakeTriggerResponse(True, '{"granted": true, "applied": true}'), None))


class _ActiveNode:
    """Full stand-in for a rclpy.node.Node used in active-mode tests.

    ``create_publisher`` / ``create_subscription`` / ``create_timer``
    capture the call but don't touch real ROS. ``create_client``
    returns a ``_FakeClient`` whose behaviour is controlled by the
    ``client_factory`` argument.
    """

    def __init__(
        self,
        client_factory: Optional[Callable[[str], _FakeClient]] = None,
    ) -> None:
        self.publishers: List[Tuple[Any, str, int]] = []
        self.subscriptions: List[Tuple[Any, str, Any, Any]] = []
        self.timers: List[Tuple[float, Callable[[], None], Any]] = []
        self.clients: List[_FakeClient] = []
        self._client_factory = client_factory or (lambda name: _FakeClient(name))

    def create_publisher(self, msg_type: Any, topic: str, qos: int) -> Any:
        self.publishers.append((msg_type, topic, qos))
        return mock.MagicMock()

    def create_subscription(self, msg_type: Any, topic: str, cb: Any, qos: Any) -> Any:
        self.subscriptions.append((msg_type, topic, cb, qos))
        return mock.MagicMock()

    def create_timer(self, period_s: float, cb: Callable[[], None]) -> Any:
        timer = mock.MagicMock()
        timer.cancel = mock.MagicMock()
        self.timers.append((period_s, cb, timer))
        return timer

    def create_client(self, srv_type: Any, service_name: str) -> _FakeClient:
        client = self._client_factory(service_name)
        self.clients.append(client)
        return client


class TestSupervisorClientMonitor(unittest.TestCase):
    """Phase 1 default behaviour: monitor mode grants locally."""

    def setUp(self) -> None:
        self.node = _StubNode()
        self.client = SupervisorClient(node=self.node, client_id="telegram", mode="monitor")

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
        self.client = SupervisorClient(node=self.node, client_id="telegram", mode="active")

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
        self.client.set_mock_response("release", lambda **kw: released.append(kw["floor"]))

        def _boom() -> None:
            raise RuntimeError("simulated publish failure")

        with self.assertRaises(RuntimeError):
            self.client.with_floor(Floor.VOICE, _boom)
        self.assertEqual(released, [Floor.VOICE])


class TestSupervisorClientStateSubscription(unittest.TestCase):
    """State subscription: latched /avatar/state → UI gate."""

    def setUp(self) -> None:
        self.node = _StubNode()
        self.client = SupervisorClient(node=self.node, client_id="telegram", mode="monitor")

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
        self.client = SupervisorClient(node=self.node, client_id="telegram", mode="monitor")

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


# ---------------------------------------------------------------------------
# AV-14 (issue #1906): wire-contract regression. The consumer's
# ``_on_state_msg`` MUST accept what the supervisor's
# ``encode_for_ros_string`` produces — and MUST NOT silently default on
# a malformed payload (that was the original bug, see
# docs/adr/0028-avatar-supervisor.md §4.7). These tests use ``monitor``
# mode so they don't need rclpy; they exercise the production callback
# directly with a fake ``std_msgs/String``-like object.
# ---------------------------------------------------------------------------


def _try_import_supervisor_codec():
    """Return (encode, decode, AvatarState, FloorState, AvatarEvent) or skip."""
    try:
        from rob_box_supervisor.core.state import (  # noqa: PLC0415
            AvatarEvent,
            AvatarState,
            FloorState,
            decode_from_ros_string,
            encode_for_ros_string,
        )
    except Exception:  # noqa: BLE001
        return None
    return (encode_for_ros_string, decode_from_ros_string, AvatarState, FloorState, AvatarEvent)


class TestOnStateMsgDecodeContract(unittest.TestCase):
    """Anti-regression: ``_on_state_msg`` must round-trip the real codec."""

    def setUp(self) -> None:
        self.node = _StubNode()
        self.client = SupervisorClient(node=self.node, client_id="telegram", mode="monitor")
        # Reset the module-level rate-limit clock so previous tests
        # cannot suppress this test's WARN.
        import rob_box_telegram.supervisor_client as sc

        sc._decode_warn_last_ts = 0.0

    def test_decodes_supervisor_wire_string_into_state(self) -> None:
        codec = _try_import_supervisor_codec()
        if codec is None:
            self.skipTest("rob_box_supervisor.core.state not importable here")
        encode, _decode, _AState, FloorState, AvatarEvent = codec
        state = _AState(
            mode="mixed",
            teleop_floor=FloorState(client_id="квест-1", since_ms=10, last_heartbeat_ms=20),
            voice_floor=None,
            last_event=AvatarEvent(timestamp_ms=9, client_id="тест", kind="dead_man_trip", args={"n": 1}),
            since_ms=7,
        )
        wire = encode(state)

        class _Msg:
            data = wire

        self.client._on_state_msg(_Msg())
        self.assertEqual(self.client.state.mode, "mixed")
        # FloorState (dataclass) is bridged to str-client_id for the
        # existing Telegram UI contract.
        self.assertEqual(self.client.state.teleop_floor, "квест-1")
        self.assertIsNone(self.client.state.voice_floor)
        self.assertEqual(self.client.state.since_ms, 7)
        self.assertEqual(self.client.state.raw["last_event"]["kind"], "dead_man_trip")
        self.assertEqual(self.client.state.raw["last_event"]["client_id"], "тест")

    def test_garbage_payload_does_not_corrupt_state(self) -> None:
        codec = _try_import_supervisor_codec()
        if codec is None:
            self.skipTest("rob_box_supervisor.core.state not importable here")
        encode, _decode, _AState, FloorState, _AEvent = codec
        good = _AState(
            mode="active",
            teleop_floor=FloorState("quest", 1, 2),
            voice_floor=None,
            last_event=None,
            since_ms=1,
        )
        good_wire = encode(good)

        class _MsgGood:
            data = good_wire

        self.client._on_state_msg(_MsgGood())
        self.assertEqual(self.client.state.teleop_floor, "quest")

        class _MsgGarbage:
            data = "this is not msgpack"

        # Must not raise, must not reset state.
        self.client._on_state_msg(_MsgGarbage())
        self.assertEqual(self.client.state.teleop_floor, "quest")

    def test_empty_payload_does_not_corrupt_state(self) -> None:
        class _MsgEmpty:
            data = ""

        self.client._on_state_msg(_MsgEmpty())
        self.assertIsNone(self.client.state.teleop_floor)
        self.assertEqual(self.client.state.mode, "off")


# ──────────────────────────────────────────────────────────────────────
# AV-15 acceptance tests — real service-call path (no test_mode hook).
#
# These tests use ``_ActiveNode`` + a fake rclpy/std_srvs stack to
# exercise the same code path that runs on the robot (AV-15). They
# monkey-patch ``_try_import_rclpy`` and ``_try_import_trigger`` inside
# ``rob_box_telegram.supervisor_client`` so that the imports succeed
# even on CI images that don't ship ROS 2 Python.
# ──────────────────────────────────────────────────────────────────────


@mock.patch("rob_box_telegram.supervisor_client._try_import_trigger")
@mock.patch("rob_box_telegram.supervisor_client._try_import_rclpy")
class TestSupervisorClientActiveServiceCall(unittest.TestCase):
    """Acceptance §1-3, §5-6: real service-call path in active mode."""

    def setUp(self) -> None:
        # Shared between patched helpers and test bodies.
        self._acquire_calls: List[Any] = []
        self._release_calls: List[Any] = []
        # Inject fake ``rclpy.qos`` so that ``_setup_active_mode``'s
        # bare ``from rclpy.qos import ...`` succeeds even on CI images
        # that don't ship ROS 2 Python. ModuleType technically forbids
        # arbitrary attribute assignment at type-check time, but at
        # runtime Python happily accepts it (this is the standard
        # sys.modules injection pattern).
        import sys as _sys
        import types as _types

        qos_mod = _types.ModuleType("rclpy.qos")
        for _name in (
            "QoSProfile",
            "ReliabilityPolicy",
            "HistoryPolicy",
            "DurabilityPolicy",
        ):
            setattr(qos_mod, _name, mock.MagicMock())
        _sys.modules["rclpy.qos"] = qos_mod

        # Add a fake ``spin_until_future_complete`` so we can assert it's
        # never called (for the non-blocking-handler test).
        rclpy_mod = _sys.modules.get("rclpy")
        if rclpy_mod is None:
            rclpy_mod = _types.ModuleType("rclpy")
            _sys.modules["rclpy"] = rclpy_mod
        rclpy_mod.spin_until_future_complete = mock.MagicMock()  # type: ignore[attr-defined]

    def _build(self, mock_rclpy, mock_trigger, *, acquire_resp=None, release_resp=None,
               acquire_wait=True, release_wait=True):
        """Wire fakes and create a SupervisorClient in active mode."""
        mock_rclpy.return_value = _FakeRosString
        mock_trigger.return_value = _FakeTrigger

        def make_acquire_client(name: str) -> _FakeClient:
            def factory(_request: Any) -> _FakeFuture:
                self._acquire_calls.append(name)
                if acquire_resp is None:
                    payload = json.dumps({"granted": True, "applied": True})
                    resp = _FakeTriggerResponse(True, payload)
                else:
                    resp = acquire_resp
                return _FakeFuture(lambda: (resp, None))

            return _FakeClient(
                service_name=name,
                response_factory=factory,
                wait_result=acquire_wait,
            )

        def make_release_client(name: str) -> _FakeClient:
            def factory(_request: Any) -> _FakeFuture:
                self._release_calls.append(name)
                if release_resp is None:
                    resp = _FakeTriggerResponse(True, '{"released": true}')
                else:
                    resp = release_resp
                return _FakeFuture(lambda: (resp, None))

            return _FakeClient(
                service_name=name,
                response_factory=factory,
                wait_result=release_wait,
            )

        def factory(name: str) -> _FakeClient:
            if name.endswith("acquire_floor"):
                return make_acquire_client(name)
            if name.endswith("release_floor"):
                return make_release_client(name)
            return _FakeClient(name)  # pragma: no cover - safety

        node = _ActiveNode(client_factory=factory)
        client = SupervisorClient(
            node=node,
            client_id="telegram",
            mode="active",
            acquire_timeout_s=0.5,
        )
        return node, client

    # ── §1: _acquire_via_service really calls the service and parses response ──

    def test_acquire_via_service_calls_svc_and_returns_granted(self, mock_rclpy, mock_trigger) -> None:
        node, client = self._build(mock_rclpy, mock_trigger)
        result = client.acquire_floor(Floor.TELEOP)
        self.assertTrue(result.granted)
        self.assertTrue(result.contacted_service)
        self.assertIsNone(result.denied_reason)
        # Lazy: both service-clients are created together on first acquire
        # (release client is needed for the release path that follows).
        self.assertEqual(len(node.clients), 2)
        acquire_client = next(c for c in node.clients if c.service_name.endswith("acquire_floor"))
        self.assertEqual(len(acquire_client.calls), 1)
        # Payload carries client_id + floor both as attrs and as JSON.
        req = acquire_client.calls[0]
        self.assertEqual(req.client_id, "telegram")
        self.assertEqual(req.floor, "teleop")
        body = json.loads(req.data)
        self.assertEqual(body, {"client_id": "telegram", "floor": "teleop"})
        client.shutdown()

    def test_acquire_via_service_parses_held_by_on_denial(self, mock_rclpy, mock_trigger) -> None:
        denial = _FakeTriggerResponse(
            True,
            json.dumps(
                {
                    "granted": False,
                    "applied": True,
                    "reason": "conflict:held_by=quest",
                    "held_by": "quest",
                }
            ),
        )
        _, client = self._build(mock_rclpy, mock_trigger, acquire_resp=denial)
        result = client.acquire_floor(Floor.VOICE)
        self.assertFalse(result.granted)
        self.assertEqual(result.denied_reason, "held_by_other")
        self.assertEqual(result.held_by, "quest")
        self.assertTrue(result.contacted_service)
        client.shutdown()

    # ── §2: _release_via_service really calls ReleaseFloor, clears local state in any case ──

    def test_release_via_service_calls_svc_and_clears_state(self, mock_rclpy, mock_trigger) -> None:
        node, client = self._build(mock_rclpy, mock_trigger)
        client.acquire_floor(Floor.TELEOP)
        self.assertTrue(client._is_holding(Floor.TELEOP))
        client.release_floor(Floor.TELEOP)
        self.assertFalse(client._is_holding(Floor.TELEOP))
        # Lazy: second service-client created for release on first release.
        self.assertEqual(len(node.clients), 2)
        release_client = next(c for c in node.clients if c.service_name.endswith("release_floor"))
        self.assertEqual(len(release_client.calls), 1)
        req = release_client.calls[0]
        self.assertEqual(req.client_id, "telegram")
        self.assertEqual(req.floor, "teleop")

    def test_release_clears_local_state_even_if_service_fails(self, mock_rclpy, mock_trigger) -> None:
        # Server returns success=False (treated as failure for release).
        bad = _FakeTriggerResponse(False, "")
        _, client = self._build(mock_rclpy, mock_trigger, release_resp=bad)
        client.acquire_floor(Floor.TELEOP)
        self.assertTrue(client._is_holding(Floor.TELEOP))
        client.release_floor(Floor.TELEOP)
        # Local state cleared unconditionally (release must not «залипнуть»).
        self.assertFalse(client._is_holding(Floor.TELEOP))

    # ── §3: handler does not block when service hangs (timeout 0.5s) ──

    def test_acquire_returns_within_timeout_when_service_hangs(self, mock_rclpy, mock_trigger) -> None:
        mock_rclpy.return_value = _FakeRosString
        mock_trigger.return_value = _FakeTrigger

        def factory(name: str) -> _FakeClient:
            return _FakeClient(
                service_name=name,
                response_factory=lambda _req: _FakeFuture(lambda: (None, None), schedule="never"),
            )

        node = _ActiveNode(client_factory=factory)
        client = SupervisorClient(
            node=node, client_id="telegram", mode="active", acquire_timeout_s=0.2
        )
        start = time.monotonic()
        result = client.acquire_floor(Floor.TELEOP)
        elapsed = time.monotonic() - start
        # The handler returns; with supervisor_required=False (default)
        # we fall back to grant after the timeout window.
        self.assertLess(elapsed, 0.5, f"handler took {elapsed:.2f}s, must be < 0.5s")
        self.assertTrue(result.granted)
        self.assertFalse(result.contacted_service)
        client.shutdown()

    def test_acquire_does_not_spin_rclpy_executor(self, mock_rclpy, mock_trigger) -> None:
        """Verify SupervisorClient never calls rclpy.spin_until_future_complete.

        The contract is ``add_done_callback`` + Event.wait; spinning
        would freeze the Telegram handler thread.
        """
        mock_rclpy.return_value = _FakeRosString
        mock_trigger.return_value = _FakeTrigger
        node = _ActiveNode()
        client = SupervisorClient(
            node=node, client_id="telegram", mode="active", acquire_timeout_s=0.2
        )
        with mock.patch("rclpy.spin_until_future_complete") as forbidden:
            client.acquire_floor(Floor.TELEOP)
            forbidden.assert_not_called()
        client.shutdown()

    # ── §4-5: supervisor_required param + WARN rate-limit ──

    def test_supervisor_required_true_denies_when_service_unavailable(self, mock_rclpy, mock_trigger) -> None:
        mock_rclpy.return_value = _FakeRosString
        mock_trigger.return_value = _FakeTrigger
        node = _ActiveNode(client_factory=lambda name: _FakeClient(name, wait_result=False))
        client = SupervisorClient(
            node=node, client_id="telegram", mode="active", supervisor_required=True
        )
        result = client.acquire_floor(Floor.TELEOP)
        self.assertFalse(result.granted)
        self.assertEqual(result.denied_reason, "supervisor_unavailable")
        self.assertFalse(result.contacted_service)
        client.shutdown()

    def test_supervisor_required_false_grants_with_warn_when_unavailable(self, mock_rclpy, mock_trigger) -> None:
        mock_rclpy.return_value = _FakeRosString
        mock_trigger.return_value = _FakeTrigger
        node = _ActiveNode(client_factory=lambda name: _FakeClient(name, wait_result=False))
        fake_now = [1000.0]
        client = SupervisorClient(
            node=node,
            client_id="telegram",
            mode="active",
            supervisor_required=False,
            now_fn=lambda: fake_now[0],
        )

        # Capture WARN logs through a custom handler so we can count them
        # across multiple calls (assertLogs fails if there are zero
        # records, which makes "rate-limit suppresses this WARN" hard
        # to assert cleanly).
        class _WarnCollector(logging.Handler):
            def __init__(self) -> None:
                super().__init__(level=logging.WARNING)
                self.records: List[logging.LogRecord] = []

            def emit(self, record: logging.LogRecord) -> None:
                self.records.append(record)

        collector = _WarnCollector()
        target_logger = logging.getLogger("rob_box_telegram.supervisor_client")
        target_logger.addHandler(collector)
        try:
            # First call → fallback grant + 1 WARN.
            result = client.acquire_floor(Floor.TELEOP)
            self.assertTrue(result.granted)
            self.assertFalse(result.contacted_service)
            self.assertEqual(
                len(collector.records), 1, f"expected one WARN, got {collector.records}"
            )
            # Second call within rate-limit window → grant, but no additional WARN.
            collector.records.clear()
            fake_now[0] += 5.0  # 5s later, still inside 60s window
            client.acquire_floor(Floor.VOICE)
            self.assertEqual(
                len(collector.records),
                0,
                f"expected rate-limit suppression, got {collector.records}",
            )
            # After the rate-limit window → fresh WARN.
            collector.records.clear()
            fake_now[0] += 60.0
            client.acquire_floor(Floor.VOICE)
            self.assertEqual(
                len(collector.records),
                1,
                f"expected fresh WARN after 60s, got {collector.records}",
            )
        finally:
            target_logger.removeHandler(collector)
            client.shutdown()

    # ── §7: heartbeat starts only when holding teleop floor ──

    def test_heartbeat_starts_only_when_holding_teleop(self, mock_rclpy, mock_trigger) -> None:
        mock_rclpy.return_value = _FakeRosString
        mock_trigger.return_value = _FakeTrigger
        node = _ActiveNode()
        client = SupervisorClient(
            node=node, client_id="telegram", mode="active", heartbeat_period_s=0.5
        )
        # After construction: no heartbeat yet (no floor held).
        self.assertEqual(len(node.timers), 0)
        self.assertEqual(len(node.publishers), 0)
        # Voice floor alone doesn't start heartbeat.
        client.acquire_floor(Floor.VOICE)
        self.assertEqual(len(node.timers), 0)
        self.assertEqual(len(node.publishers), 0)
        # Teleop starts it.
        client.acquire_floor(Floor.TELEOP)
        self.assertEqual(len(node.timers), 1)
        self.assertEqual(len(node.publishers), 1)
        self.assertTrue(node.publishers[0][1].endswith("teleop_heartbeat"))
        timer = node.timers[0][2]
        self.assertEqual(timer.cancel.call_count, 0)
        # Release teleop stops heartbeat (cancel() called exactly once).
        client.release_floor(Floor.TELEOP)
        self.assertEqual(timer.cancel.call_count, 1)
        # Release voice is a no-op for heartbeat.
        client.release_floor(Floor.VOICE)
        self.assertEqual(timer.cancel.call_count, 1)
        client.shutdown()


if __name__ == "__main__":
    unittest.main()
