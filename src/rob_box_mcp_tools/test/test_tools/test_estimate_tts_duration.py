"""test_estimate_tts_duration.py — #949 AC2: EstimateTtsDurationTool.

The tool must return a reasonable TTS playback estimate (seconds) BEFORE
synthesis so the LLM can plan a music arrangement timed to the rap/poem.

Checks:
  * default calibrated rate (30 chars/sec, Russian TTS + chipmunk 2x);
  * explicit ``chars_per_second`` override;
  * safety floor (never below 1 cps);
  * tool contract: name, parameters, non-destructive, FAST execution.

Mocked ROS2: see conftest.py MockNode. ``std_msgs`` and heavy sibling tools
are stubbed before importing dialogue (same pattern as
test_dialogue_speak_text_batch.py).
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

from rob_box_mcp_tools.tools.dialogue import EstimateTtsDurationTool  # noqa: E402


@pytest.mark.unit
class TestEstimateTtsDurationTool:
    """Issue #949 Phase 2 — duration estimation tool."""

    def test_default_rate_estimates_text_length(self, mock_node):
        """Default 30 cps: 90 chars → ~3.0s."""
        tool = EstimateTtsDurationTool(mock_node)
        result = tool.execute(text="а" * 90)
        assert result.success is True
        assert result.data["estimate_sec"] == pytest.approx(3.0)
        assert result.data["text_length"] == 90
        assert result.data["chars_per_second"] == 30.0

    def test_custom_chars_per_second_override(self, mock_node):
        """Slower rate → longer estimate."""
        tool = EstimateTtsDurationTool(mock_node)
        result = tool.execute(text="а" * 60, chars_per_second=20.0)
        assert result.success is True
        assert result.data["estimate_sec"] == pytest.approx(3.0)
        assert result.data["chars_per_second"] == 20.0

    def test_chars_per_second_floor(self, mock_node):
        """cps below 1 is clamped to 1.0 (no division blow-up)."""
        tool = EstimateTtsDurationTool(mock_node)
        result = tool.execute(text="а" * 10, chars_per_second=0.01)
        assert result.success is True
        assert result.data["chars_per_second"] == 1.0
        assert result.data["estimate_sec"] == pytest.approx(10.0)

    def test_empty_text_returns_zero_estimate(self, mock_node):
        """Empty text → 0.0s, still success (LLM can plan safely)."""
        tool = EstimateTtsDurationTool(mock_node)
        result = tool.execute(text="")
        assert result.success is True
        assert result.data["estimate_sec"] == 0.0

    def test_estimate_within_20_percent_of_actual_for_medium_text(self, mock_node):
        """AC2 sanity: a 30-char phrase at default rate ≈ 1s playback.

        The calibrated constant (30 cps incl. chipmunk 2x) is what makes
        estimates land within ±20% of real Russian TTS playback.
        """
        tool = EstimateTtsDurationTool(mock_node)
        result = tool.execute(text="Привет, я робот, читаю рэп!")
        # 28 chars / 30 cps = 0.933 → rounded 0.9
        assert result.data["estimate_sec"] == pytest.approx(0.9, abs=0.2)

    def test_tool_contract(self, mock_node):
        """Name, parameters, execution type, destructive flag."""
        tool = EstimateTtsDurationTool(mock_node)
        assert tool.name == "estimate_tts_duration"
        assert tool.destructive is False
        assert tool.execution_type.name == "FAST"

        params = {p.name: p for p in tool.parameters}
        assert "text" in params
        assert params["text"].required is True
        assert "chars_per_second" in params
        assert params["chars_per_second"].required is False
