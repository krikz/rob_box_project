"""Canonical ToolProvider adapter for Telegram's ROS2 MCP bridge.

The existing :class:`MCPBridge` remains responsible for request correlation on
``/mcp/execute`` and ``/mcp/result``.  :class:`MCPBridgeToolProvider` owns the
LLM-facing catalogue and maps bridge replies to the shared core contract.
"""

from __future__ import annotations

import asyncio
import json
import logging
import threading
import uuid
from typing import Any, Dict, Mapping, Optional, cast

from rob_box_core.ports import (
    ToolContext,
    ToolDescriptor,
    ToolNotFound,
    ToolProvider,
    ToolResult,
    ToolTimeout,
    ToolValidationError,
    ValidationResult,
)
from std_msgs.msg import String

logger = logging.getLogger(__name__)


class MCPBridge:
    """Async request/response transport over the existing MCP ROS2 topics."""

    _NO_TIMEOUT_TOOLS = frozenset(
        {
            "navigate_to_waypoint",
            "navigate_to_coordinates",
            "move_direction",
            "finish_mapping",
            "start_mapping",
        }
    )

    def __init__(self, execute_pub: Any, ros_logger: Any = None, timeout: float = 10.0):
        if timeout <= 0:
            raise ValueError("timeout must be > 0")
        self._execute_pub = execute_pub
        self._logger = ros_logger or logger
        self._timeout = timeout
        self._pending: Dict[str, asyncio.Future[Dict[str, Any]]] = {}
        self._lock = threading.Lock()
        self._provider = MCPBridgeToolProvider(self)

    @property
    def provider(self) -> "MCPBridgeToolProvider":
        """Return the canonical provider backed by this transport."""

        return self._provider

    def update_tools(self, tools: list[Mapping[str, Any]]) -> None:
        """Replace the provider catalogue received from ``/mcp/tools``."""

        self._provider.update_tools(tools)

    def on_result(self, msg: String) -> None:
        """Resolve the pending request matching an MCP result message."""

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
            future.get_loop().call_soon_threadsafe(future.set_result, data)

    async def execute(
        self,
        tool_name: str,
        parameters: Optional[Dict[str, Any]] = None,
        *,
        timeout: float | None = None,
    ) -> Dict[str, Any]:
        """Publish one MCP request and return its correlated raw response.

        ``timeout=None`` uses the bridge default. Long physical-action tools
        preserve their historical no-timeout behaviour unless a caller supplies
        an explicit timeout through :class:`ToolContext`.
        """

        request_id = str(uuid.uuid4())[:8]
        request = {
            "tool_name": tool_name,
            "parameters": parameters or {},
            "request_id": request_id,
        }
        loop = asyncio.get_running_loop()
        future: asyncio.Future[Dict[str, Any]] = loop.create_future()

        with self._lock:
            self._pending[request_id] = future

        msg = String()
        msg.data = json.dumps(request, ensure_ascii=False)
        self._execute_pub.publish(msg)

        effective_timeout = timeout
        if effective_timeout is None and tool_name not in self._NO_TIMEOUT_TOOLS:
            effective_timeout = self._timeout
        try:
            if effective_timeout is None:
                return await future
            return await asyncio.wait_for(future, timeout=effective_timeout)
        except asyncio.TimeoutError:
            with self._lock:
                self._pending.pop(request_id, None)
            self._logger.warning(
                f"MCP tool '{tool_name}' timed out (request_id={request_id})"
            )
            raise
        except asyncio.CancelledError:
            with self._lock:
                self._pending.pop(request_id, None)
            raise

    async def execute_simple(
        self,
        tool_name: str,
        parameters: Optional[Dict[str, Any]] = None,
    ) -> str:
        """Backward-compatible human-readable execution helper."""

        return _human_message(await self._provider.invoke(tool_name, parameters or {}))


class MCPBridgeToolProvider(ToolProvider):
    """Expose Telegram's MCP topic bridge through the canonical core port."""

    name = "telegram-mcp-bridge"

    def __init__(self, bridge: MCPBridge) -> None:
        self._bridge = bridge
        self._tools: dict[str, ToolDescriptor] = {}

    def _descriptor(self, name: str) -> ToolDescriptor | None:
        """Return a descriptor, allowing legacy calls before discovery.

        Until ``/mcp/tools`` is received, the bridge cannot distinguish a
        valid tool from a typo, so preserve the old permissive behaviour. Once
        discovery has completed, unknown names are rejected locally.
        """

        descriptor = self._tools.get(name)
        if descriptor is not None or not self._tools:
            return descriptor or ToolDescriptor(
                name=name,
                parameters={"type": "object", "properties": {}},
            )
        return None

    def update_tools(self, tools: list[Mapping[str, Any]]) -> None:
        """Atomically replace descriptors parsed from OpenAI tool definitions."""

        descriptors: dict[str, ToolDescriptor] = {}
        for raw in tools:
            function = raw.get("function", raw)
            if not isinstance(function, Mapping):
                continue
            name = function.get("name")
            if not isinstance(name, str) or not name:
                continue
            parameters = function.get("parameters", {})
            if not isinstance(parameters, Mapping):
                parameters = {}
            annotations = raw.get("annotations", {})
            if not isinstance(annotations, Mapping):
                annotations = {}
            descriptors[name] = ToolDescriptor(
                name=name,
                description=str(function.get("description", "")),
                parameters=cast(Mapping[str, Any], parameters),
                idempotent=bool(annotations.get("idempotentHint", False)),
                read_only=bool(annotations.get("readOnlyHint", False)),
            )
        self._tools = descriptors

    def list_tools(self) -> list[ToolDescriptor]:
        return list(self._tools.values())

    def validate_args(
        self,
        name: str,
        args: Mapping[str, Any],
    ) -> ValidationResult:
        """Validate arguments without making a network or ROS call."""

        descriptor = self._descriptor(name)
        if descriptor is None:
            return ValidationResult(valid=False, errors=(f"unknown tool: {name}",))
        return _validate_json_schema(descriptor.parameters, args)

    async def invoke(
        self,
        name: str,
        args: Mapping[str, Any],
        ctx: ToolContext | None = None,
    ) -> ToolResult:
        validation = self.validate_args(name, args)
        if not validation.valid:
            if validation.errors[0].startswith("unknown tool"):
                raise ToolNotFound(
                    validation.errors[0], provider=self.name, tool_name=name
                )
            raise ToolValidationError(
                "; ".join(validation.errors),
                errors=validation.errors,
                provider=self.name,
                tool_name=name,
            )

        timeout = ctx.timeout if ctx is not None else None
        try:
            raw = await self._bridge.execute(name, dict(args), timeout=timeout)
        except asyncio.TimeoutError as exc:
            effective_timeout = timeout if timeout is not None else self._bridge._timeout
            raise ToolTimeout(
                f"tool {name!r} timed out after {effective_timeout:.3f}s",
                provider=self.name,
                tool_name=name,
            ) from exc
        return _tool_result(raw)


def _validate_json_schema(
    schema: Mapping[str, Any],
    args: Mapping[str, Any],
) -> ValidationResult:
    errors: list[str] = []
    required = schema.get("required", ())
    if isinstance(required, list):
        for key in required:
            if key not in args:
                errors.append(f"missing required argument: {key}")
    properties = schema.get("properties", {})
    if schema.get("additionalProperties") is False and isinstance(properties, Mapping):
        for key in args:
            if key not in properties:
                errors.append(f"unexpected argument: {key}")
    if isinstance(properties, Mapping):
        for key, value in args.items():
            field_schema = properties.get(key)
            if not isinstance(field_schema, Mapping):
                continue
            expected = field_schema.get("type")
            if isinstance(expected, str) and not _matches_json_type(value, expected):
                errors.append(f"{key} must be {expected}")
    return ValidationResult(valid=not errors, errors=tuple(errors))


def _matches_json_type(value: Any, expected: str) -> bool:
    table: dict[str, tuple[type[Any], ...]] = {
        "string": (str,),
        "integer": (int,),
        "number": (int, float),
        "boolean": (bool,),
        "object": (Mapping,),
        "array": (list, tuple),
        "null": (type(None),),
    }
    expected_types = table.get(expected)
    if expected_types is None:
        return True
    if expected in {"integer", "number"} and isinstance(value, bool):
        return False
    return isinstance(value, expected_types)


def _tool_result(raw: Mapping[str, Any]) -> ToolResult:
    inner = raw.get("result", raw)
    if not isinstance(inner, Mapping):
        return ToolResult(value=inner)
    success = bool(inner.get("success", True))
    value = inner.get("data", inner.get("message"))
    error = inner.get("error") or (None if success else inner.get("message"))
    return ToolResult(value=value, error=str(error) if error else None)


def _human_message(result: ToolResult) -> str:
    if result.error is not None:
        return result.error
    if isinstance(result.value, str):
        return result.value
    return json.dumps(result.value, ensure_ascii=False, indent=2)


__all__ = ["MCPBridge", "MCPBridgeToolProvider"]
