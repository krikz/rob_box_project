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

import asyncio
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

    async def aexecute(self, **kwargs) -> MCPToolResult:
        """Async variant — awaits the async search so the ROS event loop is not blocked."""
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
            search = getattr(memory, "asearch", None)
            if search is not None:
                results = await search(query, limit=limit)
            else:
                results = await asyncio.to_thread(memory.search, query, limit=limit)
            self.log_info(
                f"[memory_search] query={query[:40]!r} → {len(results)} results "
                f"(vec={'yes' if memory.embedder.is_available() else 'no'})"
            )

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

    async def aexecute(self, **kwargs) -> MCPToolResult:
        """Async variant — awaits the async context so the ROS event loop is not blocked."""
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
            get_ctx = getattr(memory, "aget_context", None)
            if get_ctx is not None:
                ctx = await get_ctx(limit=limit, query=query)
            else:
                ctx = await asyncio.to_thread(memory.get_context, limit=limit, query=query)
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


# ---------------------------------------------------------------------------
# FaqSearchTool — event FAQ retrieval
# ---------------------------------------------------------------------------


class FaqSearchTool(MCPTool):
    """Search the active event's FAQ for answers to visitor questions.

    Uses ``node.faq_store`` (a :class:`FAQStore` instance set on MCPServer
    during ``_init_faq_store``). When event mode is not active or the FAQ
    file was not loaded, returns a clear "FAQ not available" message so the
    LLM can fall back gracefully.
    """

    @property
    def name(self) -> str:
        return "faq_search"

    @property
    def description(self) -> str:
        return (
            "Поиск по FAQ активного мероприятия. "
            "Используй когда пользователь спрашивает о программе, локации, "
            "поступлении, организации или других деталях мероприятия. "
            "Возвращает вопрос-ответные пары из загруженного FAQ-файла."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="query",
                type="string",
                description="Поисковый запрос — вопрос или ключевые слова о мероприятии.",
                required=True,
            ),
            MCPToolParameter(
                name="limit",
                type="integer",
                description="Максимальное количество результатов (по умолчанию 3, максимум 10).",
                required=False,
            ),
        ]

    @property
    def execution_type(self):
        from ..base import ToolExecutionType
        return ToolExecutionType.FAST

    @property
    def read_only(self) -> bool:
        return True

    @property
    def destructive(self) -> bool:
        return False

    def execute(self, query: str, limit: int = 3) -> MCPToolResult:
        faq_store = getattr(self.node, "faq_store", None)
        if faq_store is None:
            return MCPToolResult(
                success=False,
                error="FAQ-хранилище не инициализировано. Режим мероприятия не активен или FAQ-файл не загружен.",
            )

        event_profile = getattr(self.node, "event_profile", None)
        event_id = None
        if event_profile is not None:
            event_id = getattr(event_profile, "event_id", None)

        if not event_id:
            return MCPToolResult(
                success=False,
                error="Не задан event_id — профиль мероприятия не загружен.",
            )

        limit = max(1, min(int(limit), 10))
        try:
            results = faq_store.search(query=query.strip(), event_id=event_id, limit=limit)
        except Exception as exc:
            self.log_error(f"[faq_search] search failed: {exc}")
            return MCPToolResult(
                success=False,
                error=f"Ошибка поиска по FAQ: {exc}",
            )

        if not results:
            return MCPToolResult(
                success=True,
                data={"query": query, "event_id": event_id, "found": 0},
                message=f"По запросу '{query}' в FAQ мероприятия ничего не найдено.",
            )

        self.log_info(f"[faq_search] query={query!r} event={event_id} → {len(results)} matches")
        formatted = []
        for r in results:
            formatted.append({
                "question": r.get("question", ""),
                "answer": r.get("answer", ""),
                "category": r.get("category", ""),
            })

        return MCPToolResult(
            success=True,
            data={
                "query": query,
                "event_id": event_id,
                "found": len(results),
                "results": formatted,
            },
            message=f"Найдено {len(results)} совпадений в FAQ мероприятия.",
        )

    async def aexecute(self, **kwargs) -> MCPToolResult:
        """Async variant — awaits the async FAQ search so the ROS event loop is not blocked."""
        faq_store = getattr(self.node, "faq_store", None)
        if faq_store is None:
            return MCPToolResult(
                success=False,
                error="FAQ-хранилище не инициализировано. Режим мероприятия не активен или FAQ-файл не загружен.",
            )

        event_profile = getattr(self.node, "event_profile", None)
        event_id = None
        if event_profile is not None:
            event_id = getattr(event_profile, "event_id", None)

        if not event_id:
            return MCPToolResult(
                success=False,
                error="Не задан event_id — профиль мероприятия не загружен.",
            )

        query = str(kwargs.get("query", "")).strip()
        limit = max(1, min(int(kwargs.get("limit", 3)), 10))
        try:
            search = getattr(faq_store, "asearch", None)
            if search is not None:
                results = await search(query=query, event_id=event_id, limit=limit)
            else:
                results = await asyncio.to_thread(
                    faq_store.search, query=query, event_id=event_id, limit=limit
                )
        except Exception as exc:
            self.log_error(f"[faq_search] search failed: {exc}")
            return MCPToolResult(
                success=False,
                error=f"Ошибка поиска по FAQ: {exc}",
            )

        if not results:
            return MCPToolResult(
                success=True,
                data={"query": query, "event_id": event_id, "found": 0},
                message=f"По запросу '{query}' в FAQ мероприятия ничего не найдено.",
            )

        self.log_info(f"[faq_search] query={query!r} event={event_id} → {len(results)} matches")
        formatted = []
        for r in results:
            formatted.append({
                "question": r.get("question", ""),
                "answer": r.get("answer", ""),
                "category": r.get("category", ""),
            })

        return MCPToolResult(
            success=True,
            data={
                "query": query,
                "event_id": event_id,
                "found": len(results),
                "results": formatted,
            },
            message=f"Найдено {len(results)} совпадений в FAQ мероприятия.",
        )
