"""MCP registry adapter implementing the canonical core ``ToolProvider`` port."""

from __future__ import annotations

import asyncio
import inspect
import time
from dataclasses import dataclass, field
from typing import Any, Awaitable, Mapping, Protocol, cast

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


class MCPTransport(Protocol):
    """Minimal transport surface required by :class:`MCPBridgeExecutor`."""

    def list_tools(self) -> Any: ...

    def get_tool(self, name: str) -> Any: ...

    def execute(self, name: str, **kwargs: Any) -> Any: ...


class MCPTransportError(ToolProviderError):
    """The MCP registry/bridge failed before producing a tool result."""


class MCPBackpressureError(ToolProviderError):
    """The per-provider request budget was exhausted."""


@dataclass(frozen=True)
class MCPRetryPolicy:
    """Bounded exponential retry policy for idempotent tools only."""

    attempts: int = 2
    base_delay: float = 0.05
    max_delay: float = 2.0

    def __post_init__(self) -> None:
        if self.attempts < 1:
            raise ValueError("attempts must be >= 1")
        if self.base_delay < 0:
            raise ValueError("base_delay must be >= 0")
        if self.max_delay < self.base_delay:
            raise ValueError("max_delay must be >= base_delay")

    def delay_for(self, attempt: int) -> float:
        """Return the delay before a 1-indexed attempt."""

        if attempt <= 1:
            return 0.0
        return float(min(self.base_delay * 2.0 ** (attempt - 2), self.max_delay))


@dataclass
class MCPRateLimit:
    """Async token-bucket limiter; ``None`` disables limiting."""

    max_per_second: float | None = None
    burst: float | None = None
    wait_timeout: float = 5.0
    _tokens: float = field(init=False, repr=False, default=0.0)
    _updated_at: float = field(init=False, repr=False, default_factory=time.monotonic)
    _lock: asyncio.Lock | None = field(init=False, repr=False, default=None)

    def __post_init__(self) -> None:
        if self.max_per_second is None:
            return
        if self.max_per_second <= 0:
            raise ValueError("max_per_second must be > 0 or None")
        if self.burst is None:
            self.burst = float(self.max_per_second)
        if self.burst <= 0:
            raise ValueError("burst must be > 0")
        if self.wait_timeout < 0:
            raise ValueError("wait_timeout must be >= 0")
        self._tokens = float(self.burst)

    async def acquire(self) -> None:
        """Consume one token, waiting no longer than ``wait_timeout``."""

        rate = self.max_per_second
        burst = self.burst
        if rate is None or burst is None:
            return
        if self._lock is None:
            self._lock = asyncio.Lock()

        deadline = time.monotonic() + self.wait_timeout
        while True:
            async with self._lock:
                now = time.monotonic()
                elapsed = now - self._updated_at
                self._updated_at = now
                self._tokens = min(burst, self._tokens + elapsed * rate)
                if self._tokens >= 1.0:
                    self._tokens -= 1.0
                    return
                wait = (1.0 - self._tokens) / rate

            remaining = deadline - time.monotonic()
            if wait > remaining:
                raise MCPBackpressureError(
                    "MCP rate limit exceeded",
                    provider="mcp-bridge",
                )
            await asyncio.sleep(wait)


def _maybe_await(value: Any) -> Awaitable[Any]:
    async def resolve() -> Any:
        if inspect.isawaitable(value):
            return await value
        return value

    return resolve()


class MCPBridgeExecutor(ToolProvider):
    """Map LLM-style named calls onto an existing MCP registry or bridge."""

    name = "mcp-bridge"

    def __init__(
        self,
        transport: MCPTransport,
        *,
        retry_policy: MCPRetryPolicy | None = None,
        rate_limit: MCPRateLimit | None = None,
        default_timeout: float = 10.0,
    ) -> None:
        for method in ("list_tools", "get_tool", "execute"):
            if not callable(getattr(transport, method, None)):
                raise TypeError(f"MCP transport is missing callable {method}")
        if default_timeout <= 0:
            raise ValueError("default_timeout must be > 0")
        self._transport = transport
        self._retry = retry_policy or MCPRetryPolicy()
        self._rate_limit = rate_limit or MCPRateLimit()
        self._default_timeout = default_timeout
        self._tools: tuple[ToolDescriptor, ...] | None = None
        self._idempotency_results: dict[tuple[str, str], ToolResult] = {}
        self._idempotency_locks: dict[tuple[str, str], asyncio.Lock] = {}

    def list_tools(self) -> list[ToolDescriptor]:
        """Synchronously expose every registered MCP tool."""

        if self._tools is None:
            raw_names = self._transport.list_tools()
            if inspect.isawaitable(raw_names):
                if inspect.iscoroutine(raw_names):
                    raw_names.close()
                raise TypeError("ToolProvider.list_tools requires a synchronous MCP catalogue")
            descriptors: list[ToolDescriptor] = []
            for raw in raw_names:
                name, tool = self._resolve_tool(raw)
                if tool is not None:
                    descriptors.append(self._descriptor(name, tool))
            self._tools = tuple(descriptors)
        return list(self._tools)

    async def invoke(
        self,
        name: str,
        args: Mapping[str, Any],
        ctx: ToolContext | None = None,
    ) -> ToolResult:
        """Validate, rate-limit and execute one MCP tool call."""

        validation = self.validate_args(name, args)
        if not validation.valid:
            if any(error.startswith("unknown tool") for error in validation.errors):
                raise ToolNotFound(
                    validation.errors[0], provider=self.name, tool_name=name
                )
            raise ToolValidationError(
                "; ".join(validation.errors),
                errors=validation.errors,
                provider=self.name,
                tool_name=name,
            )

        tool = self._transport.get_tool(name)
        if inspect.isawaitable(tool):
            tool = await tool
        assert tool is not None
        idempotent = bool(getattr(tool, "idempotent", False))
        cache_key = self._cache_key(name, ctx) if idempotent else None
        if cache_key is None:
            return await self._invoke_uncached(name, args, tool, ctx)

        cached = self._idempotency_results.get(cache_key)
        if cached is not None:
            return cached
        lock = self._idempotency_locks.setdefault(cache_key, asyncio.Lock())
        async with lock:
            cached = self._idempotency_results.get(cache_key)
            if cached is not None:
                return cached
            result = await self._invoke_uncached(name, args, tool, ctx)
            self._idempotency_results[cache_key] = result
            return result

    def validate_args(
        self,
        name: str,
        args: Mapping[str, Any],
    ) -> ValidationResult:
        """Delegate to the MCP tool's existing parameter validator."""

        if not isinstance(args, Mapping):
            return ValidationResult(valid=False, errors=("arguments must be a mapping",))
        tool = self._transport.get_tool(name)
        if inspect.isawaitable(tool):
            if inspect.iscoroutine(tool):
                tool.close()
            return ValidationResult(
                valid=False,
                errors=("async get_tool is incompatible with synchronous validation",),
            )
        if tool is None:
            return ValidationResult(valid=False, errors=(f"unknown tool: {name}",))
        validator = getattr(tool, "validate_parameters", None)
        if callable(validator):
            valid, error = validator(**dict(args))
            return ValidationResult(
                valid=bool(valid),
                errors=() if valid else (str(error or "invalid arguments"),),
            )
        return self._validate_json_schema(self._descriptor(name, tool).parameters, args)

    async def _invoke_uncached(
        self,
        name: str,
        args: Mapping[str, Any],
        tool: Any,
        ctx: ToolContext | None,
    ) -> ToolResult:
        idempotent = bool(getattr(tool, "idempotent", False))
        attempts = self._retry.attempts if idempotent else 1
        timeout = ctx.timeout if ctx and ctx.timeout is not None else self._default_timeout
        last_error: BaseException | None = None

        for attempt in range(1, attempts + 1):
            delay = self._retry.delay_for(attempt)
            if delay:
                await asyncio.sleep(delay)
            try:
                await self._rate_limit.acquire()
                raw = await asyncio.wait_for(
                    _maybe_await(self._transport.execute(name, **dict(args))),
                    timeout=timeout,
                )
            except asyncio.TimeoutError as exc:
                last_error = exc
                if attempt == attempts:
                    raise ToolTimeout(
                        f"tool {name!r} timed out after {timeout:.3f}s",
                        provider=self.name,
                        tool_name=name,
                    ) from exc
            except MCPBackpressureError as exc:
                last_error = exc
                if attempt == attempts:
                    raise
            except Exception as exc:
                last_error = exc
                if attempt == attempts:
                    raise MCPTransportError(
                        f"MCP execution failed for {name!r}: {exc}",
                        provider=self.name,
                        tool_name=name,
                    ) from exc
            else:
                return self._result(raw)

        raise MCPTransportError(
            f"MCP execution exhausted {attempts} attempts for {name!r}",
            provider=self.name,
            tool_name=name,
        ) from last_error

    def _resolve_tool(self, raw: Any) -> tuple[str, Any | None]:
        if isinstance(raw, str):
            tool = self._transport.get_tool(raw)
            if inspect.isawaitable(tool):
                if inspect.iscoroutine(tool):
                    tool.close()
                raise TypeError("ToolProvider.list_tools requires synchronous get_tool")
            return raw, tool
        name = getattr(raw, "name", None)
        if not isinstance(name, str) or not name:
            raise TypeError("MCP list_tools entries must be names or tool objects")
        return name, raw

    @staticmethod
    def _descriptor(name: str, tool: Any) -> ToolDescriptor:
        formatter = getattr(tool, "to_openai_tool_format", None) or getattr(
            tool, "to_deepseek_function", None
        )
        if callable(formatter):
            formatted = formatter()
            function = formatted.get("function", formatted)
            description = str(function.get("description", ""))
            parameters = cast(Mapping[str, Any], function.get("parameters", {}))
            annotations = formatted.get("annotations", {})
            idempotent = bool(
                getattr(tool, "idempotent", annotations.get("idempotentHint", False))
            )
            read_only = bool(
                getattr(tool, "read_only", annotations.get("readOnlyHint", False))
            )
        else:
            description = str(getattr(tool, "description", ""))
            parameters = {}
            idempotent = bool(getattr(tool, "idempotent", False))
            read_only = bool(getattr(tool, "read_only", False))
        return ToolDescriptor(
            name=name,
            description=description,
            parameters=parameters,
            idempotent=idempotent,
            read_only=read_only,
        )

    @staticmethod
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
        if isinstance(properties, Mapping):
            for key, value in args.items():
                field_schema = properties.get(key)
                if isinstance(field_schema, Mapping):
                    expected = field_schema.get("type")
                    if expected and not _matches_json_type(value, str(expected)):
                        errors.append(f"{key} must be {expected}")
        if schema.get("additionalProperties") is False and isinstance(properties, Mapping):
            for key in args:
                if key not in properties:
                    errors.append(f"unexpected argument: {key}")
        return ValidationResult(valid=not errors, errors=tuple(errors))

    @staticmethod
    def _result(raw: Any) -> ToolResult:
        if isinstance(raw, ToolResult):
            return raw
        if isinstance(raw, Mapping):
            inner = raw.get("result", raw)
            if isinstance(inner, Mapping):
                success = bool(inner.get("success", True))
                value = inner.get("data", inner.get("message"))
                error = inner.get("error") or (None if success else inner.get("message"))
                return ToolResult(value=value, error=str(error) if error else None)
            return ToolResult(value=inner)
        success = bool(getattr(raw, "success", True))
        error = getattr(raw, "error", None)
        value = getattr(raw, "data", None)
        if value is None:
            value = getattr(raw, "message", None)
        return ToolResult(
            value=value,
            error=str(error) if error else (None if success else "tool execution failed"),
        )

    @staticmethod
    def _cache_key(name: str, ctx: ToolContext | None) -> tuple[str, str] | None:
        if ctx is None or not ctx.idempotency_key:
            return None
        return name, ctx.idempotency_key


MCPBridgeProvider = MCPBridgeExecutor


def build_default_mcp_bridge_provider(
    registry: MCPTransport,
    *,
    max_per_second: float | None = None,
    wait_timeout: float = 5.0,
    retry_attempts: int = 2,
    retry_base_delay: float = 0.05,
    default_timeout: float = 10.0,
) -> MCPBridgeExecutor:
    return MCPBridgeExecutor(
        registry,
        retry_policy=MCPRetryPolicy(
            attempts=retry_attempts,
            base_delay=retry_base_delay,
        ),
        rate_limit=MCPRateLimit(
            max_per_second=max_per_second,
            wait_timeout=wait_timeout,
        ),
        default_timeout=default_timeout,
    )


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


__all__ = [
    "MCPBackpressureError",
    "MCPBridgeExecutor",
    "MCPBridgeProvider",
    "MCPRateLimit",
    "MCPRetryPolicy",
    "MCPTransportError",
    "build_default_mcp_bridge_provider",
]
