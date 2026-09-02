"""Dataclasses and msgpack (de)serialization for /avatar/state (ADR-0028 §4.3).

Schema is encoded as msgpack and wrapped in ``std_msgs/String`` on the wire
to avoid introducing a new IDL for a high-frequency introspection topic.

This is the **single** place where the transport format of ``/avatar/state``
is defined. The supervisor (``supervisor_node``) and any client (currently
``supervisor_client`` in ``rob_box_telegram``) MUST round-trip via
:func:`encode_for_ros_string` / :func:`decode_from_ros_string` and MUST NOT
hand-roll their own msgpack packb + JSON/utf-8 conversion. Centralising the
codec here is the fix for the silent desync where the publisher
(msgpack-as-latin-1-string) and the consumer (json.loads) spoke different
languages and Telegram never saw any state (issue #1906, audit
``docs/plans/2026-09-02-avatar-epic-state-audit.md`` §1.2 G3).

Forward-compatibility policy (v1):
    If a payload arrives with ``version > SCHEMA_VERSION`` we **raise**
    ``StateVersionError``. Rationale: silently dropping unknown fields on a
    safety-critical control topic (floors) is worse than rejecting the frame
    and letting the supervisor surface the mismatch in logs/metrics. Callers
    that prefer "ignore and continue" must catch ``StateVersionError``
    themselves; this module never swallows it.

Backward-compatibility policy (v1):
    Missing fields fall back to dataclass defaults (e.g. older clients that
    predate ``last_event`` produce ``last_event=None``).

Typical payload budget:
    The off-mode idle frame (mode='off', all floors None, no event) must
    serialize to <=200 bytes (1 Hz publication leaves headroom under the
    ROS2 default 1 KB/s introspection budget). See ``test_state.py::5``.

Why msgpack in a ``std_msgs/String`` (and not base64 / custom IDL):
    ``std_msgs/String`` carries UTF-8. Arbitrary bytes must be hidden in
    text — we use ``latin-1`` because its 256 code points map 1:1 onto
    bytes (0x00..0xFF each become one char), giving us a lossless
    round-trip for any msgpack bytes; on the consumer side we just
    ``.encode("latin-1")`` back to bytes. base64 was an option but would
    inflate 1 Hz monitoring traffic by 33 %; introducing a custom IDL
    (``.msg`` + rosidl generation) for a single high-frequency topic is
    overkill until Phase 2 — the codec lives entirely in Python anyway.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Dict, Optional, cast

import msgpack

SCHEMA_VERSION: int = 1


class StateVersionError(ValueError):
    """Raised when an incoming payload has a higher schema version than we know.

    Forward-compatibility policy: refuse to interpret unknown future payloads
    rather than silently dropping fields on a safety-relevant control topic.
    """


@dataclass(frozen=True)
class FloorState:
    """Holder of a single resource floor (teleop or voice)."""

    client_id: str
    since_ms: int
    last_heartbeat_ms: int


@dataclass(frozen=True)
class AvatarEvent:
    """Last meaningful event observed by the supervisor."""

    timestamp_ms: int
    client_id: str
    kind: str
    args: Dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class AvatarState:
    """Top-level /avatar/state payload published at 1 Hz (latched, transient_local).

    Fields are intentionally flat (msgpack-encoded as a single dict) so external
    ``msgpack.unpackb`` callers can read it without any helper code — see
    ``test_state.py::6`` for the external-decoder contract test.
    """

    mode: str
    teleop_floor: Optional[FloorState]
    voice_floor: Optional[FloorState]
    last_event: Optional[AvatarEvent]
    since_ms: int
    version: int = SCHEMA_VERSION


# ---------------------------------------------------------------------------
# Serialization
# ---------------------------------------------------------------------------


def _floor_to_dict(f: Optional[FloorState]) -> Optional[Dict[str, Any]]:
    return None if f is None else asdict(f)


def _event_to_dict(e: Optional[AvatarEvent]) -> Optional[Dict[str, Any]]:
    return None if e is None else asdict(e)


def pack(state: AvatarState) -> bytes:
    """Serialize ``AvatarState`` to msgpack bytes.

    Uses ``use_bin_type=True`` so any ``bytes`` values land as msgpack ``bin``
    (not the legacy ``str`` type) — keeps the payload self-describing for
    future fields.
    """
    payload: Dict[str, Any] = {
        "mode": state.mode,
        "teleop_floor": _floor_to_dict(state.teleop_floor),
        "voice_floor": _floor_to_dict(state.voice_floor),
        "last_event": _event_to_dict(state.last_event),
        "since_ms": state.since_ms,
        "version": state.version,
    }
    return cast(bytes, msgpack.packb(payload, use_bin_type=True))


def _dict_to_floor(d: Optional[Dict[str, Any]]) -> Optional[FloorState]:
    if d is None:
        return None
    return FloorState(
        client_id=str(d.get("client_id", "")),
        since_ms=int(d.get("since_ms", 0)),
        last_heartbeat_ms=int(d.get("last_heartbeat_ms", 0)),
    )


def _dict_to_event(d: Optional[Dict[str, Any]]) -> Optional[AvatarEvent]:
    if d is None:
        return None
    return AvatarEvent(
        timestamp_ms=int(d.get("timestamp_ms", 0)),
        client_id=str(d.get("client_id", "")),
        kind=str(d.get("kind", "")),
        args=dict(d.get("args") or {}),
    )


def unpack(data: bytes) -> AvatarState:
    """Deserialize msgpack bytes back into an ``AvatarState``.

    Raises:
        StateVersionError: if ``data.version > SCHEMA_VERSION``.
        msgpack.exceptions.*: on malformed payloads (propagated).
    """
    raw = msgpack.unpackb(data, raw=False, strict_map_key=False)
    if not isinstance(raw, dict):
        raise ValueError(f"avatar/state: expected msgpack map, got {type(raw).__name__}")

    version = int(raw.get("version", 0))
    if version > SCHEMA_VERSION:
        raise StateVersionError(
            f"avatar/state: payload version={version} > known "
            f"SCHEMA_VERSION={SCHEMA_VERSION}; refusing to interpret future "
            f"payload (forward-compat policy: raise)"
        )

    return AvatarState(
        mode=str(raw.get("mode", "off")),
        teleop_floor=_dict_to_floor(raw.get("teleop_floor")),
        voice_floor=_dict_to_floor(raw.get("voice_floor")),
        last_event=_dict_to_event(raw.get("last_event")),
        since_ms=int(raw.get("since_ms", 0)),
        version=version,
    )


# ---------------------------------------------------------------------------
# Transport (publisher / ROS String consumer)
# ---------------------------------------------------------------------------


class StateTransportError(ValueError):
    """Raised by :func:`decode_from_ros_string` for malformed / unsupportable payloads.

    Distinguishes wire-level problems (latin-1 decode failure, missing
    msgpack dependency, non-dict payload) from schema-level
    :class:`StateVersionError`. Callers in production code (e.g.
    ``supervisor_client``) should catch this AND log at WARN (rate-limited),
    not silently swallow — silent failure is exactly what issue #1906 was.
    """


def encode_for_ros_string(state: AvatarState) -> str:
    """Serialize ``state`` to the on-wire string carried by ``std_msgs/String``.

    Pipeline: ``AvatarState`` → msgpack bytes → ``decode("latin-1")``. The
    reverse trip is :func:`decode_from_ros_string`. This is the **only**
    sanctioned way to put an ``AvatarState`` on the wire — see the module
    docstring for rationale (issue #1906, audit G3).

    Raises:
        msgpack.exceptions.PackException: if the state is not serializable
            (should never happen for a well-formed dataclass; surfaces
            immediately so the publisher can decide whether to skip the
            tick or fall back — see :func:`is_ros_string_safe`).
    """
    raw_bytes: bytes = pack(state)
    # latin-1 is byte-for-byte identity (0x00..0xFF → same single char), so
    # no character ever falls outside the 256 code points. The string then
    # travels through std_msgs/String which is UTF-8 by spec, but latin-1
    # round-trips losslessly through UTF-8 because every byte 0x00..0xFF
    # encodes as itself in latin-1 *and* is a valid single-byte UTF-8 token
    # (msgpack never produces invalid UTF-8 sequences).
    return raw_bytes.decode("latin-1")


def decode_from_ros_string(data: str) -> AvatarState:
    """Parse the on-wire string from ``/avatar/state`` into an ``AvatarState``.

    Pipeline: ``str`` → ``encode("latin-1")`` → msgpack bytes →
    :func:`unpack` → :class:`AvatarState`. Symmetric inverse of
    :func:`encode_for_ros_string`.

    This function does **not** fall back to JSON or any other encoding — if
    the bytes are not a valid msgpack frame, :class:`StateTransportError`
    is raised (or :class:`StateVersionError` if the message itself is a
    msgpack frame we cannot interpret by version policy).

    Each call goes through ``latin-1`` then msgpack. A more permissive
    variant that first tries msgpack and only then falls back could be
    added later — but the constraint is "one codec, one place", and silent
    fallback on the consumer side is exactly the bug we are fixing.
    """
    if not isinstance(data, str):
        raise StateTransportError(f"avatar/state: expected str, got {type(data).__name__}")
    if not data:
        # Empty payload is a wire-contract violation (latched topic should
        # at minimum carry a msgpack frame). Raise the typed error so the
        # consumer counter can categorise it instead of catching the bare
        # msgpack ValueError downstream.
        raise StateTransportError("avatar/state: empty payload (no msgpack frame)")
    try:
        payload_bytes = data.encode("latin-1")
    except UnicodeEncodeError as exc:
        raise StateTransportError(
            f"avatar/state: latin-1 encode of {len(data)}-char string "
            f"failed at pos {exc.start} (non-latin-1 codepoint) — wire format "
            f"contract violation, refusing to interpret"
        ) from exc
    try:
        return unpack(payload_bytes)
    except (StateVersionError, ValueError, msgpack.exceptions.UnpackException):
        # Schema-level and wire-level errors propagate so the caller can
        # rate-limit its WARN log without silently dropping the payload.
        raise


def is_ros_string_safe(state: AvatarState) -> bool:
    """Quick gate for callers that prefer to skip a publication rather than raise.

    Returns True iff :func:`encode_for_ros_string` would round-trip the
    given state cleanly. Used by supervisors that publish at 1 Hz and
    would rather skip the tick than crash the publisher callback.
    """
    try:
        roundtrip = decode_from_ros_string(encode_for_ros_string(state))
    except (StateTransportError, StateVersionError, msgpack.exceptions.PackException):
        return False
    return roundtrip == state


__all__ = (
    "SCHEMA_VERSION",
    "StateVersionError",
    "StateTransportError",
    "FloorState",
    "AvatarEvent",
    "AvatarState",
    "pack",
    "unpack",
    "encode_for_ros_string",
    "decode_from_ros_string",
    "is_ros_string_safe",
)
