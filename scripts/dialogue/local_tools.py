"""Simulated tool provider for the local (ROS2-free) dialogue chat.

On the robot the LLM's tool calls travel ``DialogCore → LLMToolCallAdapter
→ /mcp/execute → MCPServer``, which needs a ROS2 graph, a motor bus, a
speaker and a LED matrix. None of that exists on a laptop, but the *tool
catalog* does: :mod:`rob_box_core.tool_catalog` is generated from the very
``MCPTool`` classes the robot runs, so the model can be shown exactly the
same 51 tools with exactly the same JSON schemas.

This provider therefore answers ``discover()`` from the real catalog and
fakes ``execute()``:

* ``speak_text`` is surfaced to the terminal — it is how РОББОКС talks,
  so in a text chat it *is* the reply,
* the memory tools run for real against the same
  :class:`~rob_box_harness.memory.MemoryStore` ``DialogCore`` is using,
  so «запомни, что…» → «что ты обо мне помнишь?» works across restarts,
* ``get_current_time`` returns the real clock,
* everything else returns ``{"status": "ok", "simulated": true}`` with a
  note saying the hardware is absent.

The stub answers are deliberately *successful*: the point of the local
chat is to exercise the prompt and the tool-choice behaviour, and an
error result would send the model down its retry paths instead.
"""

from __future__ import annotations

import json
import time
from datetime import datetime
from typing import Any, Callable, Mapping

from rob_box_harness.core.tool_registry import ToolRegistry
from rob_box_harness.memory import Fact, MemoryStore
from rob_box_harness.tools import ToolProvider, ToolSpec
from rob_box_llm.provider import ToolCall, ToolResult

__all__ = ["SimulatedToolProvider", "ToolEvent"]


#: Free-form callback: ``(tool_name, arguments, rendered_result)``.
ToolEvent = Callable[[str, Mapping[str, Any], str], None]

_SIM_NOTE = (
    "локальный текстовый режим: железа робота нет, действие не выполнено "
    "физически — считай, что оно прошло успешно"
)


class SimulatedToolProvider(ToolProvider):
    """Catalog-backed tool provider that fakes execution locally."""

    name = "local-sim"

    def __init__(
        self,
        *,
        memory: MemoryStore | None = None,
        user_id: str = "default",
        on_speak: Callable[[str, Mapping[str, Any]], None] | None = None,
        on_tool: ToolEvent | None = None,
    ) -> None:
        self._specs: tuple[ToolSpec, ...] = tuple(ToolRegistry().list_tools())
        self._memory = memory
        self._user_id = user_id
        self._on_speak = on_speak
        self._on_tool = on_tool
        #: Names of tools handled for real rather than stubbed.
        self._real: dict[str, Callable[[Mapping[str, Any]], Any]] = {
            "get_current_time": self._t_current_time,
            "memory_save": self._t_memory_save,
            "memory_context": self._t_memory_context,
            "memory_search": self._t_memory_search,
        }

    # ── ToolProvider contract ────────────────────────────────────────

    async def discover(self) -> tuple[ToolSpec, ...]:
        """Return the robot's real LLM-visible tool catalog."""
        return self._specs

    async def execute(self, call: ToolCall) -> ToolResult:
        """Run ``call`` locally: real handler, speech, or stub."""
        args: Mapping[str, Any] = dict(call.arguments or {})

        if call.name == "speak_text":
            payload = self._t_speak_text(args)
        else:
            handler = self._real.get(call.name)
            if handler is not None:
                try:
                    payload = handler(args)
                    if hasattr(payload, "__await__"):
                        payload = await payload
                except Exception as exc:  # noqa: BLE001 — tool-level failure
                    payload = {
                        "status": "error",
                        "error": f"{type(exc).__name__}: {exc}",
                    }
            else:
                payload = {
                    "status": "ok",
                    "simulated": True,
                    "tool": call.name,
                    "note": _SIM_NOTE,
                }

        content = json.dumps(payload, ensure_ascii=False)
        if self._on_tool is not None and call.name != "speak_text":
            self._on_tool(call.name, args, content)
        return ToolResult(tool_call_id=call.id, content=content, is_error=False)

    # ── Handlers ─────────────────────────────────────────────────────

    def _t_speak_text(self, args: Mapping[str, Any]) -> dict[str, Any]:
        """Surface the robot's speech to the terminal."""
        text = str(args.get("text") or "").strip()
        if not text:
            # Mirrors the live validation: an empty phrase never reaches TTS.
            return {"status": "error", "error": "empty text", "simulated": True}
        if self._on_speak is not None:
            self._on_speak(text, args)
        return {
            "status": "ok",
            "simulated": True,
            "voice_used": str(args.get("voice") or "local-text"),
            "animation": args.get("animation"),
        }

    def _t_current_time(self, args: Mapping[str, Any]) -> dict[str, Any]:
        _ = args
        now = datetime.now().astimezone()
        return {
            "status": "ok",
            "iso": now.isoformat(timespec="seconds"),
            "time": now.strftime("%H:%M"),
            "date": now.strftime("%d.%m.%Y"),
            "weekday": now.strftime("%A"),
            "timezone": str(now.tzinfo),
        }

    async def _t_memory_save(self, args: Mapping[str, Any]) -> dict[str, Any]:
        if self._memory is None:
            return {"status": "error", "error": "memory disabled", "simulated": True}
        fact_text = str(args.get("fact") or "").strip()
        if not fact_text:
            return {"status": "error", "error": "empty fact"}
        category = str(args.get("category") or "general")
        await self._memory.save_fact(
            self._user_id,
            Fact(
                key=f"{category}:{int(time.time() * 1000)}",
                value=fact_text,
                tags=(category, "local-chat"),
            ),
        )
        return {"status": "ok", "saved": fact_text, "category": category}

    async def _t_memory_context(self, args: Mapping[str, Any]) -> dict[str, Any]:
        if self._memory is None:
            return {"status": "error", "error": "memory disabled", "simulated": True}
        limit = _as_int(args.get("limit"), default=10, lo=1, hi=50)
        query = str(args.get("query") or "").strip()
        if query:
            facts = await self._memory.search_facts(
                self._user_id, query, top_k=limit
            )
            return {
                "status": "ok",
                "query": query,
                "facts": [_fact_json(f) for f in facts],
            }
        turns = await self._memory.load_recent(self._user_id, limit=limit)
        facts = await self._memory.list_facts(self._user_id, limit=limit)
        return {
            "status": "ok",
            "recent_turns": [
                {"role": t.role, "content": t.content} for t in turns
            ],
            "facts": [_fact_json(f) for f in facts],
        }

    async def _t_memory_search(self, args: Mapping[str, Any]) -> dict[str, Any]:
        if self._memory is None:
            return {"status": "error", "error": "memory disabled", "simulated": True}
        query = str(args.get("query") or "").strip()
        if not query:
            return {"status": "error", "error": "empty query"}
        limit = _as_int(args.get("limit"), default=5, lo=1, hi=20)
        facts = await self._memory.search_facts(self._user_id, query, top_k=limit)
        return {
            "status": "ok",
            "query": query,
            "results": [_fact_json(f) for f in facts],
        }


def _fact_json(fact: Fact) -> dict[str, Any]:
    return {"key": fact.key, "value": fact.value, "tags": list(fact.tags)}


def _as_int(value: Any, *, default: int, lo: int, hi: int) -> int:
    """Coerce an LLM-supplied number into ``[lo, hi]``."""
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        return default
    return max(lo, min(hi, parsed))
