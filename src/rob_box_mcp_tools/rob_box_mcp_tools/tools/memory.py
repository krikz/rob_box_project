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

Память о людях - всегда чья-то (#1770, #4023): ``speaker_id`` вызова или «кто
сейчас» (``EncounterSeam``, только подтверждённое имя). Нет ни того, ни другого -
отказ ``rejected{no_person}`` (:func:`_no_person`): общих фактов не пишем, по
памяти всех людей не ищем и контекст всех не отдаём.
"""

from __future__ import annotations

import re
from typing import Any, List, Mapping, Optional

from rob_box_dialog import knowledge as kn

from ..base import MCPTool, MCPToolParameter, MCPToolResult

#: Отказ тула памяти без собеседника (#4023): ни ``speaker_id`` в вызове, ни «кто сейчас». Раньше запись шла в общие
#: факты, а поиск и контекст - по памяти ВСЕХ людей (#1770: Telegram/оператор, v1 без id получали чужие факты).
NO_PERSON_MESSAGE = "Не знаю, кто говорит: память о людях без собеседника не читаю и не пишу."


def _current_encounter_speaker_id(node: object) -> Optional[str]:
    """Issue #2442 — единственный путь к «кто сейчас», вместо трёх копий.

    Раньше все три инструмента ниже независимо повторяли один и тот же
    ``kwargs.get("speaker_id") or getattr(self.node, "current_speaker_id",
    None)`` — фоллбэк на приватное поле mcp_server, синхронизированное
    вручную в его ``_on_speaker_result``. Теперь mcp_server кормит тот же
    сигнал в ``EncounterSeam`` (``rob_box_harness.encounter``, общий шов
    «Встреча») и хранит его в ``node._encounter_seam``; эта функция — одна
    точка чтения ``EncounterSeam.current()`` для всех трёх тулов ниже.

    Не импортирует ``mcp_server`` напрямую (тот импортирует этот модуль —
    цикл), поэтому читает ``_encounter_seam`` через ``getattr`` по
    контракту, а не по типу. Отсутствие шва на узле (старый fake-node в
    тестах, узел без speaker_id_enabled) — молчаливый ``None``, как и
    раньше при отсутствии атрибута.
    """
    seam = getattr(node, "_encounter_seam", None)
    if seam is None:
        return None
    encounter = seam.current()
    if encounter is None or encounter.who is None:
        return None
    return encounter.who.id


def _person(node: object, kwargs: Mapping[str, Any]) -> Optional[str]:
    """Чья память: ``speaker_id`` вызова (диалог v2 кладёт собеседника снимка, #3705), иначе «кто сейчас»
    (:func:`_current_encounter_speaker_id`: ``mcp_server`` ставит его только по подтверждённому имени, #4023).
    ``None`` - собеседника нет: тул отвечает :func:`_no_person` и в память не идёт."""
    return kwargs.get("speaker_id") or _current_encounter_speaker_id(node)


def _no_person() -> MCPToolResult:
    """Отказ без собеседника (#4023): «всем» не пишем и по всей памяти не ищем; причина - ключ диалога
    ``knowledge.NO_PERSON_REASON`` (фраза отказа - у диалога, ``*.rejected.no_person``)."""
    return MCPToolResult(success=False, data={"status": "rejected", "reason": kn.NO_PERSON_REASON},
                         message=NO_PERSON_MESSAGE)


#: Подсказка при пустом ответе на многословный запрос (#3890). Тул ничего не
#: подгоняет под запрос: честно говорит «пусто» и называет общее правило.
EMPTY_MULTIWORD_HINT = (
    "Ничего не найдено по запросу из нескольких слов. Ищи 1–2 ключевыми "
    "словами в начальной форме, без перечислений и описаний; пусто — "
    "попробуй другое одно слово."
)
#: Слово короче этого — не «значимое» (предлоги, частицы), общее правило длины.
_MIN_SIGNIFICANT_WORD_LEN = 3


def _empty_result_hint(query: str, found: int) -> Optional[str]:
    """Hint для пустой выдачи при запросе из >= 2 значимых слов, иначе None."""
    if found:
        return None
    words = [w for w in re.findall(r"\w+", query) if len(w) >= _MIN_SIGNIFICANT_WORD_LEN]
    return EMPTY_MULTIWORD_HINT if len(words) >= 2 else None


class MemorySaveTool(MCPTool):
    """
    Сохранить факт о пользователе в долгосрочную память.

    Используй, когда узнаёшь что-то важное о пользователе:
    предпочтения, привычки, имена близких, настройки.
    """

    #: saved{fact_id} - ответ VoiceMemory.save_fact
    event_source = "inproc:voice_memory"

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
            MCPToolParameter(
                name="speaker_id",
                type="string",
                description=(
                    "Опционально: voice-biometric id текущего спикера (из "
                    "<system_context>/<speaker_id>). Факт сохраняется ТОЛЬКО "
                    "этому пользователю; без speaker_id и без узнанного "
                    "собеседника факт не сохраняется (общих фактов нет)."
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
                message="VoiceMemory не инициализирована (MCPServer ещё не запустился).",
            )

        fact = kwargs.get("fact", "").strip()
        category = kwargs.get("category", "general")
        # Issue #1770/#4023 — факт только конкретному человеку (``_person``); некому - не пишем вовсе.
        speaker_id = _person(self.node, kwargs)

        if not fact:
            return MCPToolResult(success=False, data=None, message="Параметр fact не может быть пустым.")
        if speaker_id is None:
            return _no_person()

        try:
            fact_id = memory.save_fact(fact, category=category, speaker_id=speaker_id)
            if not fact_id:  # saved{fact_id} без id - не запись: «Запомнил» по нему не говорится (ADR-0148)
                return MCPToolResult(success=False, data=None, message="Память не вернула id записи.")
            self.log_info(f"[memory_save] Saved fact #{fact_id} speaker={speaker_id}: {fact[:60]}...")
            return MCPToolResult(
                success=True,
                data={"event": "saved", "fact_id": fact_id, "fact": fact, "category": category,
                      "speaker_id": speaker_id},
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
            "Поиск по долгосрочной памяти: сохранённые факты о пользователе "
            "(memory_save) И история разговоров со всех сессий. "
            "Используй когда нужно вспомнить что-то про пользователя "
            "('что я говорил про...', 'что ты обо мне знаешь') или сверить "
            "факт, который только что сохранил через memory_save — включая "
            "в этом же разговоре, сразу после сохранения. "
            "Возвращает и факты, и фрагменты прошлых разговоров, помечая "
            "каждый результат полем kind: 'fact' | 'turn'."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="query",
                type="string",
                description=(
                    "Поисковый запрос на русском или английском: 1–2 ключевых "
                    "слова в начальной форме, без перечислений и описаний."
                ),
                required=True,
            ),
            MCPToolParameter(
                name="limit",
                type="integer",
                description="Максимальное количество результатов (по умолчанию 5, максимум 20).",
                required=False,
            ),
            MCPToolParameter(
                name="speaker_id",
                type="string",
                description=(
                    "Опционально: voice-biometric id текущего спикера (из "
                    "<system_context>/<speaker_id>). Ищется только память "
                    "этого человека; без speaker_id и без узнанного "
                    "собеседника поиск не идёт (по всей памяти не ищем)."
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

        query = kwargs.get("query", "").strip()
        limit = min(int(kwargs.get("limit", 5)), 20)
        # Issue #1770/#4023 — поиск только по памяти конкретного человека (``_person``); по всей памяти - никогда.
        speaker_id = _person(self.node, kwargs)

        if not query:
            return MCPToolResult(success=False, data=None, message="Параметр query не может быть пустым.")
        if speaker_id is None:
            return _no_person()

        try:
            results = memory.search(query, limit=limit, speaker_id=speaker_id)
            self.log_info(
                f"[memory_search] query={query[:40]!r} speaker={speaker_id} "
                f"→ {len(results)} results "
                f"(vec={'yes' if memory.embedder.is_available() else 'no'})"
            )

            # Format for LLM readability. Issue #2793 — ``kind`` tells the
            # LLM (and any test) whether a hit is a saved fact
            # (``memory_save``, source="fact") or a historical conversation
            # line (source="fts"/"vec"/"hybrid"); ``.get`` defaults keep this
            # compatible with any ``VoiceMemory`` stand-in that only ever
            # returned turns (no "kind"/"category" keys).
            formatted = []
            for r in results:
                entry = {
                    "kind": r.get("kind", "turn"),
                    "role": r["role"],
                    "content": r["content"],
                    "session": r.get("session_id"),
                    "score": round(r.get("score", 0), 4),
                    "source": r.get("source", "fts"),
                }
                if r.get("kind") == "fact":
                    entry["category"] = r.get("category")
                formatted.append(entry)

            data = {
                "results": formatted,
                "total": len(formatted),
                "query": query,
                "limit": limit,
                "speaker_id": speaker_id,
                "has_more": len(formatted) == limit,
                "next_offset": limit if len(formatted) == limit else None,
            }
            hint = _empty_result_hint(query, len(formatted))
            if hint:
                data["hint"] = hint
            return MCPToolResult(
                success=True,
                data=data,
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
            MCPToolParameter(
                name="speaker_id",
                type="string",
                description=(
                    "Опционально: voice-biometric id текущего спикера (из "
                    "<system_context>/<speaker_id>). Контекст только этого "
                    "человека; без speaker_id и без узнанного собеседника "
                    "контекста нет (реплики и факты всех не отдаём)."
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
        # Issue #1770/#4023 — контекст только конкретного человека (``_person``): реплики и факты всех - никогда.
        speaker_id = _person(self.node, kwargs)
        if speaker_id is None:
            return _no_person()

        try:
            ctx = memory.get_context(limit=limit, query=query, speaker_id=speaker_id)
            facts_block = memory.format_facts_for_prompt(speaker_id=speaker_id)
            stats = memory.get_stats()

            self.log_info(
                f"[memory_context] turns={len(ctx['recent_turns'])} facts={len(ctx['facts'])} "
                f"speaker={speaker_id} "
                f"total_sessions={ctx['sessions']} vec={ctx['vec_enabled']}"
            )

            return MCPToolResult(
                success=True,
                data={
                    "recent_turns": [
                        {
                            "role": t["role"],
                            "content": t["content"],
                            "session": t["session_id"],
                        }
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
                    "speaker_id": speaker_id,
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
