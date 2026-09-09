"""delta.py — S3.1 MERGE delta model (issue #968, scheduler-segments-merge).

Pure Python, no ROS, no scheduler import — mirrors the validation style
of :mod:`rob_box_voice.scheduler.decision` (frozen dataclasses,
``__post_init__`` validation). Describes *what* the LLM wants changed
about a segment group; :meth:`TaskScheduler.update` (S3.2) is what
actually applies it against live scheduler state and is the place
that knows whether a given ``seg_idx`` still exists.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any, Mapping, Optional


class DeltaOpKind(str, Enum):
    """The four ways a MERGE can touch a segment group."""

    REWRITE = "rewrite"
    REPLACE = "replace"
    APPEND = "append"
    DROP = "drop"


@dataclass(frozen=True)
class DeltaOp:
    """A single operation within a :class:`TaskDelta`.

    ``rewrite``/``replace`` require both ``seg_idx`` and ``args``.
    ``append`` requires ``args`` and must NOT carry a ``seg_idx`` — it
    always targets the tail of the group. ``drop`` requires ``seg_idx``
    and must NOT carry ``args``.

    A negative ``seg_idx`` is rejected here as structurally invalid —
    segment indices are 0-based, so it can never exist. Whether a
    *non-negative* ``seg_idx`` refers to a segment that is still
    PENDING (as opposed to RUNNING/terminal/never-existed) is scheduler
    state, checked by :meth:`TaskScheduler.update`, not here.
    """

    kind: DeltaOpKind
    seg_idx: Optional[int] = None
    args: Optional[Mapping[str, Any]] = None

    def __post_init__(self) -> None:
        if self.kind is DeltaOpKind.APPEND:
            if self.seg_idx is not None:
                raise ValueError("append must not reference a seg_idx")
            if self.args is None:
                raise ValueError("append requires args")
        elif self.kind is DeltaOpKind.DROP:
            if self.seg_idx is None:
                raise ValueError("drop requires seg_idx")
            if self.args is not None:
                raise ValueError("drop must not carry args")
        else:  # REWRITE / REPLACE
            if self.seg_idx is None:
                raise ValueError(f"{self.kind.value} requires seg_idx")
            if self.args is None:
                raise ValueError(f"{self.kind.value} requires args")
        if self.seg_idx is not None and self.seg_idx < 0:
            raise ValueError(f"seg_idx {self.seg_idx} does not exist (must be >= 0)")


def rewrite(seg_idx: int, args: Mapping[str, Any]) -> DeltaOp:
    """Build a ``rewrite`` op — change the payload of a PENDING segment."""
    return DeltaOp(kind=DeltaOpKind.REWRITE, seg_idx=seg_idx, args=dict(args))


def replace(seg_idx: int, args: Mapping[str, Any]) -> DeltaOp:
    """Build a ``replace`` op — same shape as ``rewrite``.

    Kept as a distinct op kind (not an alias) because §4 of the design
    gives REPLACE and MERGE/REWRITE different verdict semantics
    upstream — the delta model preserves that distinction even though
    the scheduler-level effect on a PENDING segment is identical.
    """
    return DeltaOp(kind=DeltaOpKind.REPLACE, seg_idx=seg_idx, args=dict(args))


def append(args: Mapping[str, Any]) -> DeltaOp:
    """Build an ``append`` op — add a new segment to the tail of the group."""
    return DeltaOp(kind=DeltaOpKind.APPEND, seg_idx=None, args=dict(args))


def drop(seg_idx: int) -> DeltaOp:
    """Build a ``drop`` op — remove a PENDING segment."""
    return DeltaOp(kind=DeltaOpKind.DROP, seg_idx=seg_idx, args=None)


@dataclass(frozen=True)
class TaskDelta:
    """A batch of :class:`DeltaOp` targeting one segment group.

    Produced by the LLM (via the ``task_delta`` MCP tool, S6) or by
    ``quick_decide`` (S4) and applied by
    :meth:`TaskScheduler.update` (S3.2).
    """

    group_id: str
    ops: tuple[DeltaOp, ...] = ()

    def __post_init__(self) -> None:
        if not self.group_id:
            raise ValueError("group_id must not be empty")
        if not self.ops:
            raise ValueError("TaskDelta must contain at least one op")
