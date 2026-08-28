"""Dataclasses and msgpack (de)serialization for /avatar/state (ADR-0028 §4.3).

Schema is encoded as msgpack and wrapped in ``std_msgs/String`` on the wire
to avoid introducing a new IDL for a high-frequency introspection topic.

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
        raise ValueError(
            f"avatar/state: expected msgpack map, got {type(raw).__name__}"
        )

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
