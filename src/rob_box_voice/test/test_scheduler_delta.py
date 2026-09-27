"""Unit tests for S3.1 :mod:`rob_box_voice.scheduler.delta` (issue #968).

Pure Python model of a MERGE delta — no scheduler, no ROS. Validates
structural correctness only; whether a ``seg_idx`` matches a *live*
segment is checked later, against real scheduler state, by
``TaskScheduler.update()`` (S3.2).
"""

from __future__ import annotations

import pytest

from rob_box_voice.scheduler.delta import (
    DeltaOp,
    DeltaOpKind,
    TaskDelta,
    append,
    drop,
    replace,
    rewrite,
)


# ---------------------------------------------------------------------------
# Op builders
# ---------------------------------------------------------------------------


class TestOpBuilders:
    def test_rewrite_builds_op(self):
        op = rewrite(2, {"text": "про енота"})
        assert op.kind is DeltaOpKind.REWRITE
        assert op.seg_idx == 2
        assert op.args == {"text": "про енота"}

    def test_replace_builds_op(self):
        op = replace(1, {"text": "новый текст"})
        assert op.kind is DeltaOpKind.REPLACE
        assert op.seg_idx == 1
        assert op.args == {"text": "новый текст"}

    def test_append_builds_op_without_seg_idx(self):
        op = append({"text": "куплет 3"})
        assert op.kind is DeltaOpKind.APPEND
        assert op.seg_idx is None
        assert op.args == {"text": "куплет 3"}

    def test_drop_builds_op_without_args(self):
        op = drop(3)
        assert op.kind is DeltaOpKind.DROP
        assert op.seg_idx == 3
        assert op.args is None


# ---------------------------------------------------------------------------
# Op-level validation
# ---------------------------------------------------------------------------


class TestOpValidation:
    def test_negative_seg_idx_does_not_exist(self):
        with pytest.raises(ValueError):
            rewrite(-1, {"text": "x"})

    def test_rewrite_requires_args(self):
        with pytest.raises(ValueError):
            DeltaOp(kind=DeltaOpKind.REWRITE, seg_idx=0, args=None)

    def test_replace_requires_args(self):
        with pytest.raises(ValueError):
            DeltaOp(kind=DeltaOpKind.REPLACE, seg_idx=0, args=None)

    def test_append_rejects_seg_idx(self):
        with pytest.raises(ValueError):
            DeltaOp(kind=DeltaOpKind.APPEND, seg_idx=0, args={"text": "x"})

    def test_append_requires_args(self):
        with pytest.raises(ValueError):
            DeltaOp(kind=DeltaOpKind.APPEND, seg_idx=None, args=None)

    def test_drop_rejects_args(self):
        with pytest.raises(ValueError):
            DeltaOp(kind=DeltaOpKind.DROP, seg_idx=0, args={"text": "x"})

    def test_drop_requires_seg_idx(self):
        with pytest.raises(ValueError):
            DeltaOp(kind=DeltaOpKind.DROP, seg_idx=None, args=None)


# ---------------------------------------------------------------------------
# TaskDelta — bundles ops for one group_id
# ---------------------------------------------------------------------------


class TestTaskDelta:
    def test_builds_with_multiple_ops(self):
        delta = TaskDelta(
            group_id="t_001",
            ops=(rewrite(2, {"text": "про енота"}), drop(3)),
        )
        assert delta.group_id == "t_001"
        assert len(delta.ops) == 2

    def test_empty_delta_rejected(self):
        with pytest.raises(ValueError):
            TaskDelta(group_id="t_001", ops=())

    def test_empty_group_id_rejected(self):
        with pytest.raises(ValueError):
            TaskDelta(group_id="", ops=(append({"text": "x"}),))
