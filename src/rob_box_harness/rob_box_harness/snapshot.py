"""SessionSnapshot — JSON-serialisable snapshot of a harness's state.

Used by:

* Tests — assert "did the harness end up in this state?".
* Replay — restore a harness from a recorded snapshot.
* Dashboards — render the current state for an operator.

The snapshot is intentionally generic: it captures three things:

  1. The harness name / kind (so a dashboard can label the row).
  2. The state mapping (the dataclass a concrete harness keeps in
     ``self.state``).
  3. A user-supplied ``extensions`` dict for harness-specific data
     (e.g. tool-call counts, last-input timestamp).

The format is plain JSON; we don't try to be clever with binary
serialisation because the volumes are tiny (one snapshot per turn
or per test).
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Mapping


@dataclass(frozen=True)
class SessionSnapshot:
    """A point-in-time view of a harness.

    ``captured_at`` is set in :meth:`Harness.snapshot` to the harness's
    :class:`Clock` value, not :func:`datetime.now` directly, so tests
    using a :class:`MockClock` get deterministic timestamps.
    """

    harness_name: str
    harness_kind: str
    captured_at: datetime
    state: Mapping[str, Any]
    extensions: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-serialisable dict representation."""
        return {
            "harness_name": self.harness_name,
            "harness_kind": self.harness_kind,
            "captured_at": self.captured_at.isoformat(),
            "state": dict(self.state),
            "extensions": dict(self.extensions),
        }

    def to_json(self) -> str:
        """Return a JSON string. ``indent=2`` for human readability."""
        return json.dumps(self.to_dict(), indent=2, sort_keys=True)

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "SessionSnapshot":
        """Inverse of :meth:`to_dict`."""
        if not isinstance(data, Mapping):
            raise ValueError("snapshot payload must be a mapping")
        captured = data.get("captured_at")
        if isinstance(captured, str):
            captured_at = datetime.fromisoformat(captured)
            if captured_at.tzinfo is None:
                captured_at = captured_at.replace(tzinfo=timezone.utc)
        elif isinstance(captured, datetime):
            captured_at = captured
        else:
            raise ValueError(
                "snapshot.captured_at must be ISO-8601 string or datetime"
            )
        return cls(
            harness_name=str(data.get("harness_name", "")),
            harness_kind=str(data.get("harness_kind", "")),
            captured_at=captured_at,
            state=dict(data.get("state", {})),
            extensions=dict(data.get("extensions", {})),
        )


__all__ = ["SessionSnapshot"]
