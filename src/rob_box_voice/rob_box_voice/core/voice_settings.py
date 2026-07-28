"""Voice-settings tool factory — single source for ``voice_settings`` tool.

ADR-0001 §2.7 D12: ``DialogueNode`` had two parallel MCP tools —
``set_volume(action)`` and ``set_pitch(pitch)`` — each a one-liner
wrapping a different MCP call. This module collapses them into one
:func:`make_voice_settings_tool` that builds a unified tool with
``action`` ∈ {"louder", "quieter", "max", "normal", "pitch"} and
an optional ``value`` for the numeric cases.

The factory accepts a caller-provided async ``caller`` (an MCP
adapter wrapper) so it works both inside the ROS2 ``DialogueNode``
and the headless ``DialogHarness``. The dispatch table is data, not
code — adding a new setting means adding one row, not another
function_tool.

Public API
----------

* :func:`make_voice_settings_tool` — returns a single
  ``@function_tool``-decorated callable bound to the supplied caller.
* :data:`SUPPORTED_ACTIONS` — the canonical action vocabulary.
* :data:`ACTION_REQUIRES_VALUE` — set of actions that need a ``value``
  parameter.

Examples
--------

In a ``DialogueNode``-style caller::

    from rob_box_voice.core.voice_settings import make_voice_settings_tool

    async def _call(tool_name, params, timeout=10.0):
        return await self.tool_provider.invoke(tool_name, params, ...)

    set_voice = make_voice_settings_tool(caller=_call)
    tools = [set_voice]

In tests::

    async def fake_caller(name, params, timeout=10.0):
        return f"{name}({params})"

    set_voice = make_voice_settings_tool(caller=fake_caller)
    # Use as a coroutine: await set_voice.on_invoke_tool(ctx, '{"action": "max"}')
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Awaitable, Callable, Optional

# ``agents`` (OpenAI Agents SDK) is an optional dep — only required at
# factory-call time, not at import time. This keeps the pure parser
# importable in test environments where ``agents`` isn't installed.
try:  # pragma: no cover - exercised via the factory, not the parser
    from agents import function_tool as _function_tool  # type: ignore
except ImportError:  # pragma: no cover - same
    _function_tool = None

# Action vocabulary. ``pitch`` is the only numeric one; the rest are
# symbolic volume presets. Keep the set small and explicit — each
# addition must come with a docstring update in the tool below.
SUPPORTED_ACTIONS: frozenset[str] = frozenset(
    {"louder", "quieter", "max", "normal", "pitch"}
)

# Actions that need a ``value`` parameter (currently just ``pitch``).
ACTION_REQUIRES_VALUE: frozenset[str] = frozenset({"pitch"})


# ── Async caller contract ─────────────────────────────────────────────
# A caller is an async function taking a tool name and a params dict,
# returning the tool's textual result (matches the
# ``DialogueNode._call`` shape).
ToolCaller = Callable[[str, dict, float], Awaitable[str]]


@dataclass(frozen=True)
class VoiceSettingsCall:
    """A single, validated request to the voice-settings tool.

    Normalised form of the LLM-supplied JSON payload:

    * ``action`` ∈ :data:`SUPPORTED_ACTIONS`
    * ``value`` required iff ``action`` ∈ :data:`ACTION_REQUIRES_VALUE`
    * ``mcp_tool`` is the resolved MCP tool name to dispatch to.
    """

    action: str
    value: Optional[float]
    mcp_tool: str
    mcp_params: dict

    def __str__(self) -> str:  # pragma: no cover - debug helper
        return f"VoiceSettingsCall(action={self.action!r}, value={self.value!r})"


def parse_voice_settings(payload: dict) -> VoiceSettingsCall:
    """Parse and validate an LLM-supplied payload.

    Args:
        payload: ``{"action": str, "value": float | None}``

    Returns:
        A :class:`VoiceSettingsCall` ready to dispatch.

    Raises:
        ValueError: When ``action`` is missing/unknown, or when a
            required ``value`` is absent or non-numeric.
    """
    action = payload.get("action")
    if not isinstance(action, str) or not action:
        raise ValueError("voice_settings: 'action' must be a non-empty string")
    if action not in SUPPORTED_ACTIONS:
        raise ValueError(
            f"voice_settings: unknown action {action!r}. "
            f"Supported: {sorted(SUPPORTED_ACTIONS)}"
        )
    raw_value = payload.get("value")
    value: Optional[float] = None
    if action in ACTION_REQUIRES_VALUE:
        if raw_value is None:
            raise ValueError(f"voice_settings: action {action!r} requires 'value'")
        try:
            value = float(raw_value)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"voice_settings: action {action!r} requires numeric value"
            ) from exc
    elif raw_value is not None:
        # Value provided for a symbolic action — keep it ignored but
        # don't error. Sympathetic LLM behaviour is "let it through".
        try:
            value = float(raw_value)
        except (TypeError, ValueError):
            value = None

    if action == "pitch":
        mcp_tool = "set_pitch"
        mcp_params = {"pitch": value if value is not None else 1.0}
    else:
        mcp_tool = "set_volume"
        mcp_params = {"action": action}

    return VoiceSettingsCall(
        action=action,
        value=value,
        mcp_tool=mcp_tool,
        mcp_params=mcp_params,
    )


def make_voice_settings_tool(
    caller: ToolCaller,
    *,
    timeout: float = 10.0,
):
    """Build the unified ``voice_settings`` ``@function_tool``.

    Args:
        caller: Async dispatcher ``(tool_name, params, timeout) -> str``.
            Must accept the legacy ``set_volume`` / ``set_pitch`` MCP
            calls (this tool is a thin wrapper around them).
        timeout: Default per-call timeout in seconds.

    Returns:
        An ``@function_tool``-decorated callable. The function name is
        ``voice_settings``; ``action`` and optional ``value`` are
        passed via JSON.
    """
    if caller is None:
        raise ValueError("make_voice_settings_tool: caller is required")
    if _function_tool is None:
        raise RuntimeError(
            "make_voice_settings_tool: 'agents' SDK is not installed. "
            "Install openai-agents to build voice_settings tools."
        )

    @_function_tool
    async def voice_settings(action: str, value: float = 1.0) -> str:
        """Изменить настройки голоса одним вызовом.

        Поддерживаемые действия (action):
            * ``louder``   — сделать громче
            * ``quieter``  — сделать тише
            * ``max``      — максимальная громкость
            * ``normal``   — нормальная громкость
            * ``pitch``    — установить высоту голоса (нужен ``value`` 0.5-2.0)

        Для ``action="pitch"`` параметр ``value`` обязателен (0.5..2.0).
        Для остальных действий ``value`` игнорируется.
        """
        try:
            parsed = parse_voice_settings({"action": action, "value": value})
        except ValueError as exc:
            return f"ERROR: {exc}"
        return await caller(parsed.mcp_tool, parsed.mcp_params, timeout)

    voice_settings.__name__ = "voice_settings"
    return voice_settings


__all__ = [
    "ACTION_REQUIRES_VALUE",
    "SUPPORTED_ACTIONS",
    "ToolCaller",
    "VoiceSettingsCall",
    "make_voice_settings_tool",
    "parse_voice_settings",
]