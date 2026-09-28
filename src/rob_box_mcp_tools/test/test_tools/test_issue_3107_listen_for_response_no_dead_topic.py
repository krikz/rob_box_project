"""Issue #3107: ListenForResponseTool must not publish on a dead topic.

The live ROS graph (L: Architecture Audit run 36422274238) showed
``/voice/stt/request`` with a publisher (mcp_server) and no subscriber
anywhere; ``git log -S'/voice/stt/request'`` finds only mcp_tools commits,
so no node ever consumed it. The tool's real effect is in dialogue_node,
which leaves the agent loop on a ``listen_for_response`` result.

Mocked ROS2: see conftest.py MockNode; ``std_msgs`` and heavy sibling tools
are stubbed before importing dialogue (same pattern as
test_estimate_tts_duration.py).
"""

import sys
from unittest.mock import Mock

sys.modules["std_msgs"] = Mock()
sys.modules["std_msgs.msg"] = Mock()

sys.modules.setdefault("rob_box_mcp_tools.tools.navigation", Mock())
sys.modules.setdefault("rob_box_mcp_tools.tools.system", Mock())
sys.modules.setdefault("rob_box_mcp_tools.tools.perception", Mock())
sys.modules.setdefault("rob_box_mcp_tools.tools.mapping", Mock())
sys.modules.setdefault("rob_box_mcp_tools.tools.memory", Mock())
sys.modules.setdefault("rob_box_mcp_tools.tools.music", Mock())

from rob_box_mcp_tools.tools.dialogue import ListenForResponseTool  # noqa: E402


def test_no_publisher_on_dead_stt_request_topic(mock_node):
    ListenForResponseTool(mock_node)
    assert "/voice/stt/request" not in mock_node._publishers


def test_execute_publishes_nothing_and_reports_success(mock_node):
    tool = ListenForResponseTool(mock_node)
    result = tool.execute(timeout_seconds=15, prompt_text="как тебя зовут?")
    assert result.success is True
    assert result.data == {"timeout_seconds": 15, "prompt_text": "как тебя зовут?"}
    assert mock_node._publishers == {}


def test_tool_name_unchanged_for_dialogue_node_short_circuit(mock_node):
    # dialogue_node matches on this exact name to leave the agent loop.
    assert ListenForResponseTool(mock_node).name == "listen_for_response"
