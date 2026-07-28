"""Production adapter from the existing ROS2 LLM bridge to ToolProvider."""

from __future__ import annotations

import asyncio
from typing import Any, Mapping, Protocol

from rob_box_core.ports import (
    ToolContext,
    ToolDescriptor,
    ToolNotFound,
    ToolProvider,
    ToolProviderError,
    ToolResult,
    ToolTimeout,
    ToolValidationError,
    ValidationResult,
)


class ROSMCPBridge(Protocol):
    """Subset of ``LLMToolCallAdapter`` required by this adapter."""

    def execute_tool_call_sync(
        self,
        tool_name: str,
        parameters: dict[str, Any],
        timeout: float | None = None,
    ) -> Mapping[str, Any]: ...


class ROSMCPToolProvider(ToolProvider):
    """Route DialogueNode tool calls through the shared provider contract."""

    name = "ros-mcp-bridge"

    def __init__(
        self,
        bridge: ROSMCPBridge,
        *,
        default_timeout: float = 10.0,
    ) -> None:
        if not callable(getattr(bridge, "execute_tool_call_sync", None)):
            raise TypeError("bridge must define execute_tool_call_sync")
        if default_timeout <= 0:
            raise ValueError("default_timeout must be > 0")
        self._bridge = bridge
        self._default_timeout = default_timeout
        self._tools: dict[str, ToolDescriptor] = {}

    def update_tools(self, tools: list[Mapping[str, Any]]) -> None:
        """Replace descriptors parsed from OpenAI-compatible tool definitions."""

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
                parameters=parameters,
                idempotent=bool(annotations.get("idempotentHint", False)),
                read_only=bool(annotations.get("readOnlyHint", False)),
            )
        self._tools = descriptors

    def register_tool(self, descriptor: ToolDescriptor) -> None:
        """Register one generated DialogueNode function-tool descriptor."""

        self._tools[descriptor.name] = descriptor

    def list_tools(self) -> list[ToolDescriptor]:
        return list(self._tools.values())

    def validate_args(
        self,
        name: str,
        args: Mapping[str, Any],
    ) -> ValidationResult:
        descriptor = self._tools.get(name)
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

        timeout = ctx.timeout if ctx is not None and ctx.timeout is not None else self._default_timeout
        if timeout <= 0:
            raise ToolTimeout(
                f"tool {name!r} has a non-positive timeout",
                provider=self.name,
                tool_name=name,
            )
        try:
            raw = await asyncio.to_thread(
                self._bridge.execute_tool_call_sync,
                name,
                dict(args),
                timeout,
            )
        except TimeoutError as exc:
            raise ToolTimeout(
                f"tool {name!r} timed out after {timeout:.3f}s",
                provider=self.name,
                tool_name=name,
            ) from exc
        except Exception as exc:
            # The legacy bridge reports transport failures as a structured
            # response; unexpected bridge exceptions need the same typed
            # provider-level taxonomy rather than leaking implementation errors.
            raise ToolProviderError(
                f"ROS MCP execution failed for {name!r}: {exc}",
                provider=self.name,
                tool_name=name,
            ) from exc
        return _result(raw)


def descriptor_from_function_tool(tool: Any) -> ToolDescriptor:
    """Extract a descriptor from OpenAI Agents SDK ``FunctionTool`` metadata."""

    name = getattr(tool, "name", None)
    if not isinstance(name, str) or not name:
        raise TypeError("function tool is missing a name")
    description = str(getattr(tool, "description", ""))
    schema = getattr(tool, "params_json_schema", None)
    if schema is None:
        schema = getattr(tool, "parameters", {})
    if not isinstance(schema, Mapping):
        schema = {}
    return ToolDescriptor(
        name=name,
        description=description,
        parameters=schema,
        idempotent=False,
    )


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
    return ValidationResult(valid=not errors, errors=tuple(errors))


def _result(raw: Mapping[str, Any]) -> ToolResult:
    success = bool(raw.get("success", True))
    value = raw.get("data", raw.get("message"))
    error = raw.get("error") or (None if success else raw.get("message"))
    return ToolResult(value=value, error=str(error) if error else None)


__all__ = ["ROSMCPBridge", "ROSMCPToolProvider", "descriptor_from_function_tool"]
