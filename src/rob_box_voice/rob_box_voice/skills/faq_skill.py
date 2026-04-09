"""FAQ skill for event-specific question answering."""

from __future__ import annotations

import json

from agents import function_tool

from .base_skill import BaseSkill


class FAQSkill(BaseSkill):
    """Sub-agent that retrieves FAQ entries for the active event."""

    def __init__(self, store, event_id: str, *args, **kwargs) -> None:
        super().__init__(adapter=None, *args, **kwargs)
        self._store = store
        self._event_id = event_id

    def _make_tools(self) -> list:
        @function_tool
        async def faq_search(query: str, limit: int = 3) -> str:
            """Search the active event FAQ and return the best matching entries.

            Args:
                query: Natural-language event question or reformulated search query.
                limit: Max number of FAQ matches to return.
            """
            results = self._store.search(
                query=query, event_id=self._event_id, limit=limit
            )
            return json.dumps(results, ensure_ascii=False)

        return [faq_search]
