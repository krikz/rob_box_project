"""test_scheduler_delta.py — S6.1 TaskDeltaTool (issue #968).

mcp_server runs as its own ROS2 node/process, separate from
dialogue_node — it has NO access to dialogue_node's in-process
TaskScheduler. TaskDeltaTool exists so ``discover()`` advertises
``task_delta``'s name/schema to the LLM; the real execution path is
SchedulerToolExecutor intercepting the call in-process (S6.2, lives in
rob_box_voice, not tested here). If ``execute()`` is ever actually
reached here it must fail HONESTLY (``capability-honest`` —
docs/architecture/tts-extension-points.md) rather than silently
pretending success — that's what these tests pin.
"""

import sys
from unittest.mock import Mock

# Mock std_msgs перед импортом dialogue — локальный паттерн для test_tools/.
sys.modules['std_msgs'] = Mock()
sys.modules['std_msgs.msg'] = Mock()

# Заглушаем тяжёлые соседние тулы, которые тянет tools/__init__.py.
sys.modules.setdefault("rob_box_mcp_tools.tools.navigation", Mock())
sys.modules.setdefault("rob_box_mcp_tools.tools.system", Mock())
sys.modules.setdefault("rob_box_mcp_tools.tools.perception", Mock())
sys.modules.setdefault("rob_box_mcp_tools.tools.mapping", Mock())
sys.modules.setdefault("rob_box_mcp_tools.tools.memory", Mock())
sys.modules.setdefault("rob_box_mcp_tools.tools.music", Mock())

import pytest  # noqa: E402

from rob_box_mcp_tools.tools.scheduler import TaskDeltaTool  # noqa: E402


@pytest.mark.unit
class TestTaskDeltaToolContract:
    def test_tool_contract(self, mock_node):
        tool = TaskDeltaTool(mock_node)
        assert tool.name == "task_delta"
        assert tool.destructive is False
        assert tool.execution_type.name == "FAST"

        params = {p.name: p for p in tool.parameters}
        assert "group_id" in params
        assert params["group_id"].required is True
        assert "ops" in params
        assert params["ops"].required is True
        assert params["ops"].type == "array"


@pytest.mark.unit
class TestTaskDeltaToolValidation:
    def test_empty_group_id_rejected(self, mock_node):
        tool = TaskDeltaTool(mock_node)
        result = tool.execute(group_id="", ops=[{"kind": "drop", "seg_idx": 1}])
        assert result.success is False
        assert result.error != "scheduler_unavailable"

    def test_empty_ops_rejected(self, mock_node):
        tool = TaskDeltaTool(mock_node)
        result = tool.execute(group_id="t_001", ops=[])
        assert result.success is False
        assert result.error != "scheduler_unavailable"

    def test_unknown_op_kind_rejected(self, mock_node):
        tool = TaskDeltaTool(mock_node)
        result = tool.execute(group_id="t_001", ops=[{"kind": "nonsense"}])
        assert result.success is False
        assert result.error != "scheduler_unavailable"

    def test_rewrite_without_seg_idx_rejected(self, mock_node):
        """Delegates to DeltaOp's own __post_init__ validation (S3.1)."""
        tool = TaskDeltaTool(mock_node)
        result = tool.execute(
            group_id="t_001",
            ops=[{"kind": "rewrite", "args": {"text": "x"}}],
        )
        assert result.success is False
        assert result.error != "scheduler_unavailable"

    def test_negative_seg_idx_rejected(self, mock_node):
        tool = TaskDeltaTool(mock_node)
        result = tool.execute(
            group_id="t_001",
            ops=[{"kind": "drop", "seg_idx": -1}],
        )
        assert result.success is False
        assert result.error != "scheduler_unavailable"


@pytest.mark.unit
class TestTaskDeltaToolHonestFailure:
    def test_well_formed_ops_still_fail_honestly(self, mock_node):
        """capability-honest: mcp_server has no TaskScheduler of its own —
        a structurally valid call must still return success=False, never
        a fabricated success."""
        tool = TaskDeltaTool(mock_node)
        result = tool.execute(
            group_id="t_001",
            ops=[{"kind": "rewrite", "seg_idx": 2, "args": {"text": "про енота"}}],
        )
        assert result.success is False
        assert result.error == "scheduler_unavailable"

    def test_multiple_valid_ops(self, mock_node):
        tool = TaskDeltaTool(mock_node)
        result = tool.execute(
            group_id="t_001",
            ops=[
                {"kind": "rewrite", "seg_idx": 2, "args": {"text": "про енота"}},
                {"kind": "drop", "seg_idx": 3},
                {"kind": "append", "args": {"text": "куплет 3"}},
            ],
        )
        assert result.success is False
        assert result.error == "scheduler_unavailable"
