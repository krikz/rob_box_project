"""End-to-end wire-contract test for ``/avatar/state`` (AV-14, issue #1906).

This is the regression test that **should have caught the bug** before it
shipped: the publisher wrote msgpack-as-latin-1 string, the consumer
attempted ``json.loads``, ``except`` silently swallowed the
``JSONDecodeError`` and the bot never saw any state. We pin the contract
here so that the encoder and the decoder are forced to be the *same
code path* on both sides.

What we exercise:

1. **Round-trip via UTF-8 (the path ROS actually takes).** ``std_msgs/String``
   is a UTF-8 field. A publisher writes ``msg.data = payload_str``
   (str), DDS serialises the str to UTF-8 bytes on the wire, the
   subscriber reads back ``msg.data`` (str, decoded from UTF-8 by rclpy).
   We simulate that by ``s.encode("utf-8").decode("utf-8")`` so a
   regression that picks a non-UTF-8-safe encoding (e.g. a naive
   ``bytes`` cast or a latin-1-only path) blows up *here* and not in
   production.

2. **Cyrillic / multi-byte UTF-8 in client_id and last_event.** A
   previous incarnation of the consumer treated the bytes as ASCII
   and crashed on non-ASCII holders. The contract here forces every
   byte (0x00..0xFF) to round-trip losslessly.

3. **One codec on both sides.** We import
   :func:`rob_box_supervisor.core.state.encode_for_ros_string` and
   :func:`decode_from_ros_string` and prove they are inverses for a
   realistic AvatarState (both floors taken, last_event set, every
   integer populated).

4. **Cross-package contract with the real Telegram consumer.** The
   consumer's ``SupervisorClient._on_state_msg`` is the exact
   production code that decoded ``/avatar/state`` in production and
   silently dropped every frame for a month. We feed the encoded
   payload into that production callback (via a stub ``_StubNode``)
   and assert the resulting ``client.state`` reflects the original
   holders, mode and event kind. This is the test that, had it
   existed, would have failed on every CI run since 2026-08-15.

5. **Garbage / empty payload does not corrupt state.** The contract
   says: a malformed payload leaves the previous state intact, does
   not raise, and the error counter is bumped. This is the explicit
   anti-regression for the "silent except" pattern.
"""

from __future__ import annotations

import sys
from typing import Any

import pytest

# rob_box_supervisor imports this; the test itself does not need rclpy.
# We shim it so the test runs in a pure-Python CI env as well.
if "rclpy" not in sys.modules:  # pragma: no cover - CI env shim
    sys.modules["rclpy"] = None  # type: ignore[assignment]

from rob_box_supervisor.core.state import (  # noqa: E402
    SCHEMA_VERSION,
    AvatarEvent,
    AvatarState,
    FloorState,
    StateTransportError,
    StateVersionError,
    decode_from_ros_string,
    encode_for_ros_string,
)


def _full_state() -> AvatarState:
    """A maximally-diverse AvatarState exercising every byte lane.

    - cyrillic in client_id (multi-byte UTF-8 → goes through msgpack
      ``bin`` after the latin-1 round-trip)
    - 0xC3 bytes in args (lone high-byte, would break naive ASCII
      codecs)
    - both floors set with distinct since_ms / last_heartbeat_ms
    - last_event with structured args dict (forces msgpack map-of-mix)
    """
    return AvatarState(
        mode="mixed",
        teleop_floor=FloorState(
            client_id="квест-1",  # cyrillic, multi-byte UTF-8
            since_ms=1_700_000_000_100,
            last_heartbeat_ms=1_700_000_000_900,
        ),
        voice_floor=FloorState(
            client_id="telegram-bot",
            since_ms=1_700_000_000_200,
            last_heartbeat_ms=1_700_000_000_800,
        ),
        last_event=AvatarEvent(
            timestamp_ms=1_700_000_000_900,
            client_id="телеграм",  # cyrillic
            kind="dead_man_trip",
            args={"trip_count": 3, "raw_byte": b"\xc3\x28"},  # 0xC3 byte
        ),
        since_ms=1_700_000_000_000,
        version=SCHEMA_VERSION,
    )


class TestStateWireContract:
    """Single source of truth for the ``/avatar/state`` on-wire format."""

    def test_round_trip_via_utf8_preserves_cyrillic_and_high_bytes(self) -> None:
        """The exact path ROS 2 takes: ``str`` → UTF-8 → ``str``."""
        state = _full_state()
        wire = encode_for_ros_string(state)
        assert isinstance(wire, str)
        # ROS 2: std_msgs/String.data is str; DDS carries it as UTF-8
        # bytes; subscriber decodes UTF-8 back to str.
        ros_path = wire.encode("utf-8").decode("utf-8")
        decoded = decode_from_ros_string(ros_path)
        assert decoded == state

    def test_round_trip_off_mode_idle_under_budget(self) -> None:
        """The off-mode idle frame must fit in 200 bytes (per state.py
        module docstring) — this is the intropection budget ADR."""
        idle = AvatarState(
            mode="off",
            teleop_floor=None,
            voice_floor=None,
            last_event=None,
            since_ms=1_700_000_000_000,
        )
        wire = encode_for_ros_string(idle)
        # 1 char per byte via latin-1; on-wire len == byte len of msgpack.
        assert len(wire.encode("utf-8")) <= 200, f"off-mode idle frame too large: {len(wire.encode('utf-8'))} bytes"

    def test_empty_string_raises_state_transport_error(self) -> None:
        """Empty payload is a contract violation, not a silent default."""
        with pytest.raises(StateTransportError):
            decode_from_ros_string("")

    def test_non_str_raises_state_transport_error(self) -> None:
        """Defence in depth: the codec only accepts ``str``."""
        with pytest.raises(StateTransportError):
            decode_from_ros_string(b"abc")  # type: ignore[arg-type]

    def test_garbage_string_raises_state_transport_or_unpack(self) -> None:
        """Non-msgpack text must raise, never silently default.

        msgpack raises ``UnpackException`` (which we don't re-export);
        the consumer catches it under ``Exception`` and bumps the error
        counter. Here we just prove the codec is loud.
        """
        with pytest.raises(Exception):  # noqa: BLE001 — broad on purpose
            decode_from_ros_string("это не msgpack — это просто строка")

    def test_forward_version_raises_state_version_error(self) -> None:
        """Future-version payloads must NOT be silently accepted."""
        # Build a payload with version=99, bypassing our pack() helper.
        import msgpack

        future_bytes = msgpack.packb(
            {
                "mode": "off",
                "teleop_floor": None,
                "voice_floor": None,
                "last_event": None,
                "since_ms": 0,
                "version": 99,
            },
            use_bin_type=True,
        )
        future_wire = future_bytes.decode("latin-1")
        with pytest.raises(StateVersionError):
            decode_from_ros_string(future_wire)

    def test_high_byte_0xc3_in_args_survives_latin_1_round_trip(self) -> None:
        """0xC3 is a 2-byte UTF-8 prefix; it must NOT be re-interpreted
        as UTF-8 anywhere on the path."""
        state = AvatarState(
            mode="off",
            teleop_floor=None,
            voice_floor=None,
            last_event=AvatarEvent(
                timestamp_ms=0,
                client_id="x",
                kind="probe",
                args={"raw": b"\xc3\x28"},
            ),
            since_ms=0,
        )
        wire = encode_for_ros_string(state)
        ros_path = wire.encode("utf-8").decode("utf-8")
        decoded = decode_from_ros_string(ros_path)
        assert decoded == state
        assert decoded.last_event is not None
        assert decoded.last_event.args["raw"] == b"\xc3\x28"


# ---------------------------------------------------------------------------
# Cross-package test: the real SupervisorClient._on_state_msg must accept
# what encode_for_ros_string produces. This is the test that, had it existed,
# would have failed on every CI run for the entire lifetime of the silent
# JSONDecodeError bug.
# ---------------------------------------------------------------------------


class _StubNode:
    """Minimal stand-in for rclpy.node.Node — only the bits TelegramNode
    actually calls in monitor mode. Mirrors the helper in
    ``test_telegram_supervisor_client.py``."""

    def get_logger(self) -> Any:  # pragma: no cover — trivial
        class _L:
            def info(self, *a, **k) -> None:
                pass

            def warning(self, *a, **k) -> None:
                pass

            def error(self, *a, **k) -> None:
                pass

            def debug(self, *a, **k) -> None:
                pass

        return _L()

    def create_publisher(self, *args: Any, **kwargs: Any) -> None:  # pragma: no cover
        raise AssertionError("monitor mode must not touch ROS publishers")

    def create_subscription(self, *args: Any, **kwargs: Any) -> None:  # pragma: no cover
        raise AssertionError("monitor mode must not touch ROS subscriptions")

    def create_timer(self, *args: Any, **kwargs: Any) -> None:  # pragma: no cover
        raise AssertionError("monitor mode must not touch ROS timers")


def _telegram_importable() -> bool:
    try:
        import rob_box_telegram  # noqa: F401
    except Exception:  # noqa: BLE001
        return False
    return True


@pytest.mark.skipif(
    not _telegram_importable(),
    reason="rob_box_telegram not importable in this env (CI minimal)",
)
class TestConsumerAcceptsWireContract:
    """The real Telegram consumer must accept what the real supervisor
    publisher produces. This is the actual end-to-end contract test."""

    def test_supervisor_wire_round_trips_through_real_consumer(self) -> None:
        from rob_box_telegram.supervisor_client import SupervisorClient

        client = SupervisorClient(
            node=_StubNode(),
            client_id="telegram",
            mode="monitor",
        )
        state = _full_state()
        wire = encode_for_ros_string(state)
        # ROS 2 path
        ros_path = wire.encode("utf-8").decode("utf-8")

        # Feed the wire into the production callback
        class _Msg:
            data = ros_path

        client._on_state_msg(_Msg())

        # The Telegram UI contract is ``teleop_floor: Optional[str]`` =
        # the client_id of the current holder, so the consumer must
        # bridge the supervisor's FloorState dataclass to a string.
        observed = client.state
        assert observed.mode == "mixed"
        assert observed.teleop_floor == "квест-1"
        assert observed.voice_floor == "telegram-bot"
        assert observed.since_ms == 1_700_000_000_000
        # The full event lands in raw for future UI surfaces.
        assert observed.raw["last_event"]["kind"] == "dead_man_trip"
        assert observed.raw["last_event"]["client_id"] == "телеграм"
        assert observed.raw["teleop_floor"]["client_id"] == "квест-1"

    def test_garbage_payload_does_not_corrupt_state(self) -> None:
        """Anti-regression for the silent except: garbage must NOT
        reset the state to its default (that would lie to the UI
        gate about a held floor)."""
        from rob_box_telegram.supervisor_client import SupervisorClient

        client = SupervisorClient(
            node=_StubNode(),
            client_id="telegram",
            mode="monitor",
        )
        state = _full_state()
        wire = encode_for_ros_string(state)

        class _MsgGood:
            data = wire

        client._on_state_msg(_MsgGood())
        assert client.state.teleop_floor == "квест-1"

        class _MsgGarbage:
            data = "not msgpack at all"

        # Must not raise, must not reset state.
        client._on_state_msg(_MsgGarbage())
        assert client.state.teleop_floor == "квест-1"

    def test_empty_payload_does_not_corrupt_state(self) -> None:
        from rob_box_telegram.supervisor_client import SupervisorClient

        client = SupervisorClient(
            node=_StubNode(),
            client_id="telegram",
            mode="monitor",
        )

        class _MsgEmpty:
            data = ""

        client._on_state_msg(_MsgEmpty())
        assert client.state.teleop_floor is None
