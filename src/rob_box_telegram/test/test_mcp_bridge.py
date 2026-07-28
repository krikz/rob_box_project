#!/usr/bin/env python3
"""Tests for rob_box_telegram.mcp_bridge module."""

import asyncio
import json
import sys
import unittest
from unittest.mock import MagicMock

from rob_box_core.ports import ToolTimeout

# Mock ROS 2 modules not available in dev environment
_std_msgs_mock = MagicMock()


class _StringMsg:
    """Mock std_msgs.msg.String for testing without ROS 2."""

    def __init__(self, data: str = ""):
        self.data = data


_std_msgs_mock.msg.String = _StringMsg
sys.modules.setdefault("std_msgs", _std_msgs_mock)
sys.modules.setdefault("std_msgs.msg", _std_msgs_mock.msg)

from rob_box_telegram.mcp_bridge import MCPBridge, MCPBridgeToolProvider  # noqa: E402


class TestMCPBridge(unittest.IsolatedAsyncioTestCase):
    """Tests for MCPBridge request/response correlation."""

    def setUp(self):
        self.pub = MagicMock()
        self.bridge = MCPBridge(self.pub, timeout=1.0)

    async def test_execute_publishes_request(self):
        """execute() should publish JSON to /mcp/execute."""
        # Start execute but don't wait for result (it will timeout)
        task = asyncio.create_task(self.bridge.execute("test_tool", {"key": "val"}))

        # Give it a moment to publish
        await asyncio.sleep(0.05)

        self.pub.publish.assert_called_once()
        call_msg = self.pub.publish.call_args[0][0]
        data = json.loads(call_msg.data)
        self.assertEqual(data["tool_name"], "test_tool")
        self.assertEqual(data["parameters"]["key"], "val")
        self.assertIn("request_id", data)

        # Cancel the task (it would timeout)
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass

    async def test_execute_receives_correlated_result(self):
        """on_result() with matching request_id should resolve the future."""
        task = asyncio.create_task(self.bridge.execute("list_waypoints", {}))
        await asyncio.sleep(0.05)

        # Extract the request_id from the published message
        call_msg = self.pub.publish.call_args[0][0]
        request_data = json.loads(call_msg.data)
        request_id = request_data["request_id"]

        # Simulate MCP server response
        result_msg = _StringMsg()
        result_msg.data = json.dumps({
            "tool_name": "list_waypoints",
            "request_id": request_id,
            "result": {"success": True, "message": "2 waypoints"},
        })
        self.bridge.on_result(result_msg)

        result = await asyncio.wait_for(task, timeout=2.0)
        self.assertEqual(result["result"]["message"], "2 waypoints")

    async def test_execute_timeout(self):
        """The canonical provider should translate transport timeout."""
        provider = MCPBridgeToolProvider(self.bridge)
        with self.assertRaises(ToolTimeout):
            await provider.invoke("slow_tool", {})

    async def test_execute_simple(self):
        """execute_simple() should return message string."""
        task = asyncio.create_task(self.bridge.execute_simple("get_status", {}))
        await asyncio.sleep(0.05)

        call_msg = self.pub.publish.call_args[0][0]
        request_data = json.loads(call_msg.data)

        result_msg = _StringMsg()
        result_msg.data = json.dumps({
            "tool_name": "get_status",
            "request_id": request_data["request_id"],
            "result": {"success": True, "message": "Battery: 85%"},
        })
        self.bridge.on_result(result_msg)

        message = await asyncio.wait_for(task, timeout=2.0)
        self.assertEqual(message, "Battery: 85%")

    def test_on_result_ignores_invalid_json(self):
        """on_result() should handle invalid JSON gracefully."""
        msg = _StringMsg()
        msg.data = "not valid json"
        self.bridge.on_result(msg)  # Should not raise

    def test_on_result_ignores_missing_request_id(self):
        """on_result() should ignore messages without request_id."""
        msg = _StringMsg()
        msg.data = json.dumps({"tool_name": "test"})
        self.bridge.on_result(msg)  # Should not raise


if __name__ == "__main__":
    unittest.main()
