#!/usr/bin/env python3
"""
mcp_bridge.py — Bridge between Telegram bot and MCP tool execution system.

Publishes tool-call requests to /mcp/execute and collects results from /mcp/result.
Supports async request/response with timeout and request_id correlation.
"""

import asyncio
import json
import logging
import threading
import uuid
from typing import Any, Callable, Dict, Optional

from std_msgs.msg import String

logger = logging.getLogger(__name__)


class MCPBridge:
    """Async bridge for executing MCP tools via ROS 2 topics.

    Usage::

        bridge = MCPBridge(execute_pub, node_logger)
        # In ROS callback:
        bridge.on_result(msg)
        # From Telegram handler:
        result = await bridge.execute("get_robot_status", {})

    Args:
        execute_pub: ROS 2 publisher for /mcp/execute topic.
        logger: ROS 2 logger instance.
        timeout: Max seconds to wait for result.
    """

    def __init__(self, execute_pub, ros_logger=None, timeout: float = 10.0):
        self._execute_pub = execute_pub
        self._logger = ros_logger or logger
        self._timeout = timeout

        # Pending requests: request_id -> asyncio.Future
        self._pending: Dict[str, asyncio.Future] = {}
        self._lock = threading.Lock()

    def on_result(self, msg: String) -> None:
        """ROS 2 callback for /mcp/result messages.

        Resolves the corresponding pending Future if request_id matches.
        """
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self._logger.warning(f"Invalid JSON in /mcp/result: {msg.data[:200]}")
            return

        request_id = data.get("request_id")
        if not request_id:
            return

        with self._lock:
            future = self._pending.pop(request_id, None)

        if future and not future.done():
            # Thread-safe resolution — the future belongs to the Telegram asyncio loop
            loop = future.get_loop()
            loop.call_soon_threadsafe(future.set_result, data)

    async def execute(self, tool_name: str, parameters: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """Execute an MCP tool and wait for the result.

        Args:
            tool_name: Name of the MCP tool (e.g. "get_robot_status").
            parameters: Tool parameters dict.

        Returns:
            Result dict with keys: tool_name, request_id, result.

        Raises:
            asyncio.TimeoutError: If result not received within timeout.
        """
        request_id = str(uuid.uuid4())[:8]
        request = {
            "tool_name": tool_name,
            "parameters": parameters or {},
            "request_id": request_id,
        }

        # Create a future in the current event loop
        loop = asyncio.get_running_loop()
        future: asyncio.Future = loop.create_future()

        with self._lock:
            self._pending[request_id] = future

        # Publish the request
        msg = String()
        msg.data = json.dumps(request, ensure_ascii=False)
        self._execute_pub.publish(msg)

        try:
            result = await asyncio.wait_for(future, timeout=self._timeout)
            return result
        except asyncio.TimeoutError:
            with self._lock:
                self._pending.pop(request_id, None)
            self._logger.warning(f"MCP tool '{tool_name}' timed out (request_id={request_id})")
            return {
                "tool_name": tool_name,
                "request_id": request_id,
                "result": {"success": False, "message": f"Timeout waiting for tool '{tool_name}'"},
            }

    async def execute_simple(self, tool_name: str, parameters: Optional[Dict[str, Any]] = None) -> str:
        """Execute an MCP tool and return a human-readable message.

        Convenience wrapper that extracts the message string from the result.
        """
        result = await self.execute(tool_name, parameters)
        inner = result.get("result", {})
        if isinstance(inner, dict):
            return inner.get("message", json.dumps(inner, ensure_ascii=False, indent=2))
        return str(inner)
