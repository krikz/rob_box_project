#!/usr/bin/env python3
"""web_search_skill.py — General web search via DuckDuckGo (issue #1101).

Provides a single tool ``search_web`` for general-purpose queries:
weather, news, facts, prices, sports scores. Reuses the DuckDuckGo
wrapper that's already used by ``music_skill.search_artist_style``
(issue #1000) — no API key required.

The tool returns up to N result snippets (~250 chars each) so the LLM
can synthesize an answer. For music-specific queries, the LLM should
still call ``search_artist_style`` (which does genre/BPM/instruments
research); ``search_web`` is the fallback for everything else.
"""

from __future__ import annotations

import json
import os
from typing import Any

from .base_skill import BaseSkill
from ..function_tool import function_tool

try:
    from ddgs import DDGS

    _DDGS_AVAILABLE = True
except ImportError:
    try:
        from duckduckgo_search import DDGS  # legacy name

        _DDGS_AVAILABLE = True
    except ImportError:
        _DDGS_AVAILABLE = False


class WebSearchSkill(BaseSkill):
    """General-purpose web search tool."""

    def __init__(self) -> None:
        super().__init__()
        self._enabled: bool = bool(
            os.environ.get("ROBBOX_WEB_SEARCH_ENABLED", "1") != "0"
        )

    def skill_name(self) -> str:
        return "web_search"

    def get_tools(self) -> list[Any]:
        if not self._enabled:
            return []
        if not _DDGS_AVAILABLE:
            # Log once at construction time
            self._log_once(
                "⚠️ web_search disabled — install `pip install duckduckgo-search`"
            )
            return []

        @function_tool
        def search_web(query: str, max_results: int = 5) -> str:
            """Search the web (DuckDuckGo) and return top result snippets.

            USE THIS for any factual question that requires fresh/current
            information that you don't have in training data:
            - **Weather**: «какая погода в Батайске сегодня»
            - **News**: «что нового в мире ИИ», «последние новости Tesla»
            - **Prices/currency**: «курс доллара сейчас», «сколько стоит iPhone»
            - **Sports**: «счёт матча Спартак-Зенит», «кто выиграл UFC»
            - **Local info**: «где поесть в Сочи», «работает ли метро в Москве»

            DO NOT use for:
            - Music research — use ``search_artist_style`` instead
              (genre/BPM/instruments/structure)
            - Personal facts — use ``memory_search`` / ``memory_context``

            Args:
                query: Search query in Russian or English. Keep it specific
                    (city name, date, exact topic) for best results.
                max_results: How many top results to return (default 5, max 10).
                    Each result includes title and snippet (~250 chars).

            Returns:
                JSON with ``results`` list (title + body) and a short
                ``hint`` field. Empty list when nothing found.
            """
            if not query.strip():
                return json.dumps(
                    {"error": "empty query", "results": []}, ensure_ascii=False
                )

            max_results = max(1, min(int(max_results or 5), 10))

            try:
                with DDGS() as ddgs:
                    results = list(
                        ddgs.text(query.strip(), max_results=max_results, region="wt-wt")
                    )
            except Exception as exc:  # noqa: BLE001
                return json.dumps(
                    {
                        "error": f"{type(exc).__name__}: {exc}",
                        "query": query,
                        "results": [],
                        "hint": "DuckDuckGo недоступен — проверь сеть на роботе",
                    },
                    ensure_ascii=False,
                )

            snippets = []
            for r in results:
                title = (r.get("title") or "").strip()
                body = (r.get("body") or "").strip()
                if not body:
                    continue
                # Trim body to keep token usage bounded.
                if len(body) > 280:
                    body = body[:280].rstrip() + "..."
                href = (r.get("href") or "").strip()
                snippets.append(
                    {"title": title, "body": body, "url": href}
                )

            if not snippets:
                return json.dumps(
                    {
                        "query": query,
                        "results": [],
                        "hint": "Ничего не найдено — попробуй переформулировать запрос",
                    },
                    ensure_ascii=False,
                )

            return json.dumps(
                {"query": query, "results": snippets},
                ensure_ascii=False,
            )

        return [search_web]
