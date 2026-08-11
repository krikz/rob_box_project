#!/usr/bin/env python3
"""web_search.py — MCP tool for DuckDuckGo web search (issue #1101).

Wraps the DuckDuckGo instant-answer API so the LLM can get fresh/current
information: weather, news, prices, sports scores, local info, facts —
anything that requires data newer than the LLM's training cutoff.

The tool runs a synchronous HTTP call inside ``execute()`` — no async, no
ROS topics (unlike speak_text / register_speaker). Returns a JSON blob with
up to ``max_results`` snippets (~280 chars each).

Usage (from LLM tool-call):
    search_web(query="погода в Батайске сегодня", max_results=3)
    → {"query": "...", "results": [{"title":"...","body":"...","url":"..."}], ...}
"""

from __future__ import annotations

import json
from typing import List

from ..base import MCPTool, MCPToolParameter, MCPToolResult

class SearchWebTool(MCPTool):
    """DuckDuckGo general-purpose web search tool."""

    _MAX_RESULTS: int = 10
    _SNIPPET_CHARS: int = 280

    def __init__(self, node) -> None:
        super().__init__(node)
        self._ddgs_available: bool | None = None  # lazy init in execute()
        self.log_info("🔍 SearchWebTool registered (ddgs import deferred)")

    # ── Tool metadata ────────────────────────────────────────────────────

    @property
    def name(self) -> str:
        return "search_web"

    @property
    def description(self) -> str:
        return (
            "Поиск в интернете через DuckDuckGo. Возвращает до N сниппетов "
            "(title, text, url) по запросу. Используй для: погоды, новостей, "
            "курсов валют, спортивных результатов, фактов, локальной "
            "информации — всего, что требует свежих данных. НЕ используй "
            "для музыкального ресёрча (genre/BPM — есть search_artist_style)."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="query",
                type="string",
                description=(
                    "Поисковый запрос на русском или английском. "
                    "Будь конкретным: укажи город, дату, тему."
                ),
                required=True,
            ),
            MCPToolParameter(
                name="max_results",
                type="integer",
                description="Сколько результатов вернуть (1-10, по умолчанию 5).",
                required=False,
            ),
        ]

    @property
    def execution_type(self):
        from ..base import ToolExecutionType

        return ToolExecutionType.FAST

    @property
    def destructive(self) -> bool:
        return False

    # ── Execution ────────────────────────────────────────────────────────

    def execute(
        self,
        query: str,
        max_results: int = 5,
    ) -> MCPToolResult:
        # Issue #1101 — lazy ddgs import. Module-level import
        # sometimes fails at mcp_server startup (race with pip
        # install, stale .pyc, or sys.path ordering). Retry once
        # per execute() invocation so the tool self-heals without
        # a process restart.
        if self._ddgs_available is None:
            try:
                from ddgs import DDGS

                self._ddgs_available = True
            except ImportError:
                try:
                    from duckduckgo_search import DDGS  # noqa: F811

                    self._ddgs_available = True
                except ImportError:
                    self._ddgs_available = False
                    self.log_error(
                        "[search_web] ddgs/duckduckgo_search import"
                        " failed — web search will return an error."
                    )
        DDGS_cls = DDGS if self._ddgs_available else None

        if not self._ddgs_available or DDGS_cls is None:
            return MCPToolResult(
                success=False,
                data={"error": "duckduckgo_search not installed"},
                message=(
                    "Поиск в интернете недоступен — duckduckgo-search не "
                    "установлен на роботе. Попроси оператора выполнить "
                    "`pip install duckduckgo-search` в Docker-образе."
                ),
            )

        query = (query or "").strip()
        if not query:
            return MCPToolResult(
                success=False,
                data={"error": "empty_query", "results": []},
                message="Пустой поисковый запрос — уточни, что искать.",
            )

        max_results = max(1, min(int(max_results or 5), self._MAX_RESULTS))

        try:
            with DDGS_cls() as ddgs:
                raw = list(
                    ddgs.text(query, max_results=max_results, region="wt-wt")
                )
        except Exception as exc:  # noqa: BLE001
            self.log_error(f"[search_web] DDGS error: {type(exc).__name__}: {exc}")
            return MCPToolResult(
                success=False,
                data={
                    "error": f"{type(exc).__name__}: {exc}",
                    "query": query,
                    "results": [],
                },
                message=(
                    f"DuckDuckGo недоступен — проверь, есть ли интернет "
                    f"на роботе. Ошибка: {type(exc).__name__}"
                ),
            )

        snippets: list[dict] = []
        for r in raw:
            title = (r.get("title") or "").strip()
            body = (r.get("body") or "").strip()
            if not body:
                continue
            if len(body) > self._SNIPPET_CHARS:
                body = body[:self._SNIPPET_CHARS].rstrip() + "..."
            href = (r.get("href") or "").strip()
            snippets.append({"title": title, "body": body, "url": href})

        if not snippets:
            return MCPToolResult(
                success=True,
                data={"query": query, "results": []},
                message=(
                    "Ничего не найдено по запросу. "
                    "Попробуй переформулировать или уточнить."
                ),
            )

        result_json = json.dumps(
            {"query": query, "results": snippets},
            ensure_ascii=False,
        )
        self.log_info(
            f"[search_web] query={query[:60]!r} → {len(snippets)} results"
        )
        return MCPToolResult(
            success=True,
            data={"query": query, "results": snippets},
            message=result_json,
        )
