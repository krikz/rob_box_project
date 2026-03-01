"""
memory_skill.py — MemorySkill: sub-agent for long-term memory operations.

Tools exposed to the sub-agent:
  memory_save    — persist a fact or event
  memory_search  — semantic search in past turns
  memory_context — retrieve recent conversation context
"""

from agents import function_tool

from .base_skill import BaseSkill


class MemorySkill(BaseSkill):
    """Sub-agent that handles long-term memory read and write operations."""

    def _make_tools(self) -> list:
        async def _call(name, params, timeout=10.0):
            return await self._call(name, params, timeout)

        @function_tool
        async def memory_save(content: str, tags: str = "") -> str:
            """Save important information to long-term memory.

            Args:
                content: The fact or event to remember.
                tags:    Optional comma-separated tags (e.g. 'user,preference').
            """
            return await _call("memory_save", {"content": content, "tags": tags})

        @function_tool
        async def memory_search(query: str, limit: int = 5) -> str:
            """Search long-term memory for relevant past information.

            Args:
                query: Natural-language search query.
                limit: Max number of results to return (default 5).
            """
            return await _call("memory_search", {"query": query, "limit": limit})

        @function_tool
        async def memory_context(limit: int = 10) -> str:
            """Retrieve recent conversation context from long-term memory.

            Args:
                limit: Max number of recent turns to include (default 10).
            """
            return await _call("memory_context", {"limit": limit})

        return [memory_save, memory_search, memory_context]
