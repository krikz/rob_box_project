#!/usr/bin/env python3
"""
memory.py — MCP tools for accessing VoiceMemory from the LLM agent.

Tools:
  - MemorySaveTool:    Save a user fact / preference to persistent memory.
  - MemorySearchTool:  Hybrid search (FTS5 + optional vector) over conversation history.
  - MemoryContextTool: Retrieve recent conversation turns from previous sessions.

The tools read `node.voice_memory` (a VoiceMemory instance set on MCPServer).
If voice_memory is not initialised yet, all tools return an informative error
instead of crashing.
"""

from __future__ import annotations

from typing import List

from ..base import MCPTool, MCPToolParameter, MCPToolResult


class MemorySaveTool(MCPTool):
    """
    Сохранить факт о пользователе в долгосрочную память.

    Используй, когда узнаёшь что-то важное о пользователе:
    предпочтения, привычки, имена близких, настройки.
    """

    @property
    def name(self) -> str:
        return "memory_save"

    @property
    def description(self) -> str:
        return (
            "Сохранить факт или предпочтение пользователя в долгосрочную память. "
            "Используй после того как узнал что-то важное: имя, предпочтение, привычку. "
            "Данные сохраняются между сессиями и помогут лучше обслуживать пользователя в будущем."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="fact",
                type="string",
                description=(
                    "Текст факта для сохранения. "
                    "Пример: 'Пользователя зовут Алексей', 'Предпочитает краткие ответы'."
                ),
                required=True,
            ),
            MCPToolParameter(
                name="category",
                type="string",
                description="Категория факта для организации памяти.",
                required=False,
                enum=["preference", "habit", "name", "general"],
            ),
        ]

    def execute(self, **kwargs) -> MCPToolResult:
        memory = getattr(self.node, "voice_memory", None)
        if memory is None:
            return MCPToolResult(
                success=False,
                data=None,
                message="VoiceMemory не инициализирована (MCPServer ещё не запустился).",
            )

        fact = kwargs.get("fact", "").strip()
        category = kwargs.get("category", "general")

        if not fact:
            return MCPToolResult(success=False, data=None, message="Параметр fact не может быть пустым.")

        try:
            fact_id = memory.save_fact(fact, category=category)
            self.log_info(f"[memory_save] Saved fact #{fact_id}: {fact[:60]}...")
            return MCPToolResult(
                success=True,
                data={"fact_id": fact_id, "fact": fact, "category": category},
                message=f"Факт сохранён (id={fact_id}).",
            )
        except Exception as e:
            self.log_error(f"[memory_save] Error: {e}")
            return MCPToolResult(success=False, data=None, message=f"Ошибка сохранения: {e}")


class MemorySearchTool(MCPTool):
    """
    Гибридный поиск по истории разговоров (FTS5 + семантический поиск).

    Помогает вспомнить что обсуждалось ранее — даже в прошлых сессиях.
    При наличии Ollama использует векторный поиск для смысловых запросов.
    """

    @property
    def name(self) -> str:
        return "memory_search"

    @property
    def description(self) -> str:
        return (
            "Поиск по долгосрочной памяти (история разговоров со всех сессий). "
            "Используй когда нужно вспомнить: 'где мы остановились', 'что я просил раньше', "
            "'какие были настройки'. "
            "Возвращает релевантные фрагменты из прошлых разговоров."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="query",
                type="string",
                description="Поисковый запрос на русском или английском.",
                required=True,
            ),
            MCPToolParameter(
                name="limit",
                type="integer",
                description="Максимальное количество результатов (по умолчанию 5, максимум 20).",
                required=False,
            ),
        ]

    def execute(self, **kwargs) -> MCPToolResult:
        memory = getattr(self.node, "voice_memory", None)
        if memory is None:
            return MCPToolResult(
                success=False,
                data=None,
                message="VoiceMemory не инициализирована.",
            )

        query = kwargs.get("query", "").strip()
        limit = min(int(kwargs.get("limit", 5)), 20)

        if not query:
            return MCPToolResult(success=False, data=None, message="Параметр query не может быть пустым.")

        try:
            results = memory.search(query, limit=limit)
            self.log_info(
                f"[memory_search] query={query[:40]!r} → {len(results)} results "
                f"(vec={'yes' if memory.embedder.is_available() else 'no'})"
            )

            # Format for LLM readability
            formatted = []
            for r in results:
                formatted.append(
                    {
                        "role": r["role"],
                        "content": r["content"],
                        "session": r["session_id"],
                        "score": round(r.get("score", 0), 4),
                        "source": r.get("source", "fts"),
                    }
                )

            return MCPToolResult(
                success=True,
                data={
                    "results": formatted,
                    "total": len(formatted),
                    "query": query,
                    "limit": limit,
                    "has_more": len(formatted) == limit,
                    "next_offset": limit if len(formatted) == limit else None,
                },
                message=f"Найдено {len(formatted)} результатов.",
            )
        except Exception as e:
            self.log_error(f"[memory_search] Error: {e}")
            return MCPToolResult(success=False, data=None, message=f"Ошибка поиска: {e}")


class MemoryContextTool(MCPTool):
    """
    Получить контекст из предыдущих сессий для инжекции в разговор.

    Возвращает последние реплики из прошлых сессий + сохранённые факты.
    Используй в начале разговора чтобы восстановить контекст.
    """

    @property
    def name(self) -> str:
        return "memory_context"

    @property
    def description(self) -> str:
        return (
            "Получить контекст памяти из предыдущих сессий: "
            "последние реплики + известные факты о пользователе. "
            "Используй в начале разговора для восстановления контекста, "
            "или чтобы напомнить себе что знаешь о пользователе."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="limit",
                type="integer",
                description="Количество последних реплик для загрузки (по умолчанию 10).",
                required=False,
            ),
            MCPToolParameter(
                name="query",
                type="string",
                description=(
                    "Опциональный поисковый запрос — если задан, возвращает релевантные "
                    "реплики вместо хронологических последних."
                ),
                required=False,
            ),
        ]

    def execute(self, **kwargs) -> MCPToolResult:
        memory = getattr(self.node, "voice_memory", None)
        if memory is None:
            return MCPToolResult(
                success=False,
                data=None,
                message="VoiceMemory не инициализирована.",
            )

        limit = min(int(kwargs.get("limit", 10)), 30)
        query = kwargs.get("query", "").strip() or None

        try:
            ctx = memory.get_context(limit=limit, query=query)
            facts_block = memory.format_facts_for_prompt()
            stats = memory.get_stats()

            self.log_info(
                f"[memory_context] turns={len(ctx['recent_turns'])} facts={len(ctx['facts'])} "
                f"total_sessions={ctx['sessions']} vec={ctx['vec_enabled']}"
            )

            return MCPToolResult(
                success=True,
                data={
                    "recent_turns": [
                        {"role": t["role"], "content": t["content"], "session": t["session_id"]}
                        for t in ctx["recent_turns"]
                    ],
                    "facts_block": facts_block,
                    "facts": ctx["facts"],
                    "stats": {
                        "total_turns": ctx["total_turns"],
                        "total_sessions": ctx["sessions"],
                        "vec_enabled": ctx["vec_enabled"],
                        "db_size_kb": stats["db_size_kb"],
                    },
                    "current_session": ctx["current_session"],
                },
                message=(
                    f"Контекст: {len(ctx['recent_turns'])} реплик из прошлых сессий, "
                    f"{len(ctx['facts'])} фактов о пользователе."
                ),
            )
        except Exception as e:
            self.log_error(f"[memory_context] Error: {e}")
            return MCPToolResult(success=False, data=None, message=f"Ошибка получения контекста: {e}")
