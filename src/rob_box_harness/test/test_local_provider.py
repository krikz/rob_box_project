"""Contract and safety tests for the in-process local tool provider."""

from __future__ import annotations

from datetime import datetime, timezone

import pytest

from rob_box_core.ports import ToolNotFound, ToolProvider, ToolValidationError
from rob_box_harness.executors import HarnessIntrospection, LocalToolProvider


def _provider() -> LocalToolProvider:
    return LocalToolProvider(
        HarnessIntrospection(
            snapshot=lambda: {"turn": 3},
            recent_turns=lambda: ["one", "two"],
            tool_count=lambda: 42,
            provider_names=lambda: ["mcp", "local"],
            recovery_status=lambda: {"healthy": True, "attempts": 0},
        ),
        now=lambda: datetime(2026, 7, 27, 12, 30, tzinfo=timezone.utc),
    )


def test_local_provider_implements_canonical_contract() -> None:
    provider = _provider()

    assert isinstance(provider, ToolProvider)
    assert provider.name == "local"


def test_catalogue_contains_only_read_only_idempotent_tools() -> None:
    provider = _provider()

    descriptors = provider.list_tools()

    assert {descriptor.name for descriptor in descriptors} == {
        "harness.snapshot",
        "harness.recent_turns",
        "harness.tool_count",
        "harness.provider_names",
        "harness.recovery_status",
        "harness.utc_now",
    }
    assert all(descriptor.idempotent for descriptor in descriptors)
    assert all(descriptor.parameters["additionalProperties"] is False for descriptor in descriptors)
    assert descriptors is not provider.list_tools()


@pytest.mark.parametrize(
    ("name", "expected"),
    [
        ("harness.snapshot", {"turn": 3}),
        ("harness.recent_turns", ["one", "two"]),
        ("harness.tool_count", 42),
        ("harness.provider_names", ["local", "mcp"]),
        ("harness.recovery_status", {"healthy": True, "attempts": 0}),
        ("harness.utc_now", "2026-07-27T12:30:00+00:00"),
    ],
)
async def test_each_local_tool_reads_in_process_state(name: str, expected: object) -> None:
    result = await _provider().invoke(name, {})

    assert result.is_error is False
    assert result.value == expected


async def test_missing_accessors_return_stable_safe_defaults() -> None:
    provider = LocalToolProvider(
        now=lambda: datetime(2026, 7, 27, tzinfo=timezone.utc)
    )

    assert (await provider.invoke("harness.snapshot", {})).value == {"available": False}
    assert (await provider.invoke("harness.recent_turns", {})).value == []
    assert (await provider.invoke("harness.tool_count", {})).value == 0
    assert (await provider.invoke("harness.provider_names", {})).value == []
    assert (await provider.invoke("harness.recovery_status", {})).value == {"healthy": True}


def test_validation_rejects_arguments() -> None:
    validation = _provider().validate_args("harness.snapshot", {"network": True})

    assert validation.valid is False
    assert validation.errors == ("harness.snapshot does not accept arguments",)


async def test_unknown_and_invalid_calls_raise_typed_errors() -> None:
    provider = _provider()

    with pytest.raises(ToolNotFound):
        await provider.invoke("harness.missing", {})
    with pytest.raises(ToolValidationError):
        await provider.invoke("harness.snapshot", {"unexpected": True})


def test_introspection_rejects_non_callable_accessors() -> None:
    with pytest.raises(TypeError, match="snapshot"):
        HarnessIntrospection(snapshot={})  # type: ignore[arg-type]
