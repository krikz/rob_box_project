"""Read-only, in-process tool provider for harness introspection."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Callable, Mapping

from rob_box_core.ports import (
    ToolContext,
    ToolDescriptor,
    ToolNotFound,
    ToolProvider,
    ToolResult,
    ToolValidationError,
    ValidationResult,
)

Accessor = Callable[[], Any]


@dataclass(frozen=True)
class HarnessIntrospection:
    """Optional, pure accessors exposed by :class:`LocalToolProvider`."""

    snapshot: Accessor | None = None
    recent_turns: Accessor | None = None
    tool_count: Accessor | None = None
    provider_names: Accessor | None = None
    recovery_status: Accessor | None = None

    def __post_init__(self) -> None:
        for name in (
            "snapshot",
            "recent_turns",
            "tool_count",
            "provider_names",
            "recovery_status",
        ):
            value = getattr(self, name)
            if value is not None and not callable(value):
                raise TypeError(f"{name} must be callable or None")


_TOOL_DESCRIPTORS: tuple[ToolDescriptor, ...] = tuple(
    ToolDescriptor(
        name=name,
        description=description,
        parameters={
            "type": "object",
            "properties": {},
            "additionalProperties": False,
        },
        idempotent=True,
        read_only=True,
    )
    for name, description in (
        ("harness.snapshot", "Return the current in-memory session snapshot."),
        ("harness.recent_turns", "Return recent in-memory conversation turns."),
        ("harness.tool_count", "Return the current tool catalogue size."),
        ("harness.provider_names", "Return registered provider names."),
        ("harness.recovery_status", "Return read-only recovery diagnostics."),
        ("harness.utc_now", "Return the current UTC timestamp."),
    )
)


class LocalToolProvider(ToolProvider):
    """Network-free provider whose handlers only read in-process state."""

    name = "local"

    def __init__(
        self,
        introspection: HarnessIntrospection | None = None,
        *,
        now: Callable[[], datetime] | None = None,
    ) -> None:
        self._introspection = introspection or HarnessIntrospection()
        self._now = now or (lambda: datetime.now(timezone.utc))

    def list_tools(self) -> list[ToolDescriptor]:
        return list(_TOOL_DESCRIPTORS)

    def validate_args(
        self,
        name: str,
        args: Mapping[str, Any],
    ) -> ValidationResult:
        if name not in {descriptor.name for descriptor in _TOOL_DESCRIPTORS}:
            return ValidationResult(valid=False, errors=(f"unknown tool: {name}",))
        if args:
            return ValidationResult(
                valid=False,
                errors=(f"{name} does not accept arguments",),
            )
        return ValidationResult(valid=True)

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
                validation.errors[0],
                errors=validation.errors,
                provider=self.name,
                tool_name=name,
            )

        handlers: dict[str, Callable[[], Any]] = {
            "harness.snapshot": lambda: self._read("snapshot", {"available": False}),
            "harness.recent_turns": lambda: self._read("recent_turns", []),
            "harness.tool_count": lambda: self._read("tool_count", 0),
            "harness.provider_names": lambda: sorted(
                str(value) for value in self._read("provider_names", [])
            ),
            "harness.recovery_status": lambda: self._read(
                "recovery_status", {"healthy": True}
            ),
            "harness.utc_now": lambda: self._now().isoformat(),
        }
        return ToolResult(value=handlers[name]())

    def _read(self, name: str, default: Any) -> Any:
        accessor = getattr(self._introspection, name)
        return default if accessor is None else accessor()


LocalSkillProvider = LocalToolProvider


def build_default_local_skill_provider(
    introspection: HarnessIntrospection | None = None,
    *,
    now: Callable[[], datetime] | None = None,
) -> LocalToolProvider:
    return LocalToolProvider(introspection=introspection, now=now)


__all__ = [
    "HarnessIntrospection",
    "LocalSkillProvider",
    "LocalToolProvider",
    "build_default_local_skill_provider",
]
