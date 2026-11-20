"""Fallback-каталог фактов, когда подстрока и реплики пусты (issue #4956).

Марафон #4950, шаг n1008: после сброса сессии запрос «еда» не находил
«Совсем не ест лук, даже жареный» — «еда» не подстрока факта, окно реплик
пустое, вектор на стенде мёртв (Ollama нет). Каталог отдаёт только
``search(..., browse_when_empty=True)`` — его просит инструмент
``memory_search``. Дефолт ``False`` сохраняет контракт #3890: запрос
про то, чего нет, остаётся пустым.
"""

from __future__ import annotations

import time

from rob_box_dialog_host.voice_memory import VoiceMemory

ONION = "Совсем не ест лук, даже жареный"
TEA = "Пьёт только зелёный чай без сахара"
SASHA = "0da4521c-0b50-422b-be4e-402417f25659"


def _memory(tmp_path, session_id: str = "issue_4956") -> VoiceMemory:
    return VoiceMemory(
        db_path=str(tmp_path / "voice_memory.db"),
        session_id=session_id,
    )


def test_general_query_returns_fact_catalog(tmp_path) -> None:
    """«еда» не подстрока ни одного факта — оба возвращаются как каталог."""
    memory = _memory(tmp_path)
    try:
        memory.save_fact(ONION, category="preference", speaker_id=SASHA)
        time.sleep(0.01)
        memory.save_fact(TEA, category="preference", speaker_id=SASHA)

        hits = memory.search("еда", limit=5, speaker_id=SASHA, browse_when_empty=True)

        assert [h["content"] for h in hits] == [TEA, ONION]
        assert all(h["source"] == "fact_browse" for h in hits)
        assert all(h["kind"] == "fact" for h in hits)
        assert all(h["score"] == 0.0 for h in hits)
        assert all(h["role"] == "fact" for h in hits)
        assert all(h["session_id"] is None for h in hits)
        assert all(h["speaker_id"] == SASHA for h in hits)
        assert all(h["category"] == "preference" for h in hits)
    finally:
        memory.close()


def test_substring_hit_keeps_fact_source(tmp_path) -> None:
    """Точное подстрочное попадание не размывается каталогом."""
    memory = _memory(tmp_path)
    try:
        memory.save_fact(ONION, category="preference", speaker_id=SASHA)
        memory.save_fact(TEA, category="preference", speaker_id=SASHA)

        hits = memory.search("чай", limit=5, speaker_id=SASHA)

        assert len(hits) == 1
        assert hits[0]["content"] == TEA
        assert hits[0]["source"] == "fact"
        assert hits[0]["kind"] == "fact"
        assert hits[0]["score"] == 1.0
    finally:
        memory.close()


def test_matching_turn_is_not_replaced_by_catalog(tmp_path) -> None:
    """Совпавшая реплика возвращается; каталог фактов не подмешивается.

    Фактов нет, реплика содержит токен запроса целиком (FTS5 unicode61
    не стеммит «еду» → «еда», поэтому токен берём как в реплике).
    """
    memory = _memory(tmp_path)
    try:
        # Факт есть, но токена запроса в нём нет — каталог не должен
        # вытеснить или дополнить найденную реплику.
        memory.save_fact(ONION, category="preference", speaker_id=SASHA)
        memory.save_turn(
            "user",
            "Вчера на кухне готовили ужин",
            session_id="past_session",
            speaker_id=SASHA,
        )

        hits = memory.search("кухне", limit=5, speaker_id=SASHA)

        assert len(hits) == 1
        assert "ужин" in hits[0]["content"]
        assert hits[0]["source"] != "fact_browse"
        assert hits[0].get("kind") == "turn"
    finally:
        memory.close()


def test_facts_survive_session_reset(tmp_path) -> None:
    """new_session в API нет: новый VoiceMemory с тем же db_path — факты живы."""
    db_path = tmp_path / "voice_memory.db"
    first = VoiceMemory(db_path=str(db_path), session_id="before_reset")
    try:
        first.save_fact(ONION, category="preference", speaker_id=SASHA)
        first.save_fact(TEA, category="preference", speaker_id=SASHA)
    finally:
        first.close()

    reset = VoiceMemory(db_path=str(db_path), session_id="after_reset")
    try:
        hits = reset.search("еда", limit=5, speaker_id=SASHA, browse_when_empty=True)
        assert {h["content"] for h in hits} == {ONION, TEA}
        assert all(h["source"] == "fact_browse" for h in hits)
    finally:
        reset.close()


def test_browse_hides_other_speaker_keeps_legacy_null(tmp_path) -> None:
    """Чужой speaker_id не отдаётся; legacy NULL виден, как в get_facts."""
    memory = _memory(tmp_path)
    try:
        memory.save_fact(ONION, category="preference", speaker_id=SASHA)
        memory.save_fact("Денчик ест всё подряд", speaker_id="denchik")
        memory.save_fact("глобальный факт про еду")  # speaker_id IS NULL

        hits = memory.search("еда", limit=10, speaker_id=SASHA, browse_when_empty=True)
        contents = [h["content"] for h in hits]

        assert ONION in contents
        assert "глобальный факт про еду" in contents
        assert "Денчик ест всё подряд" not in contents
        assert all(h["source"] == "fact_browse" for h in hits)
    finally:
        memory.close()


def test_browse_respects_limit(tmp_path) -> None:
    memory = _memory(tmp_path)
    try:
        for i in range(4):
            memory.save_fact(f"факт номер {i} без общих слов", speaker_id=SASHA)
            time.sleep(0.01)

        hits = memory.search("еда", limit=2, speaker_id=SASHA, browse_when_empty=True)

        assert len(hits) == 2
        assert [h["content"] for h in hits] == [
            "факт номер 3 без общих слов",
            "факт номер 2 без общих слов",
        ]
        assert all(h["source"] == "fact_browse" for h in hits)
    finally:
        memory.close()


def test_default_search_on_miss_stays_empty(tmp_path) -> None:
    """Дефолт browse_when_empty=False: несовпадение остаётся [], как #3890."""
    memory = _memory(tmp_path)
    try:
        memory.save_fact(ONION, category="preference", speaker_id=SASHA)
        memory.save_fact(TEA, category="preference", speaker_id=SASHA)

        assert memory.search("еда", limit=5, speaker_id=SASHA) == []
        assert memory.search("еда", limit=5, speaker_id=SASHA, browse_when_empty=False) == []
    finally:
        memory.close()


def test_memory_search_tool_requests_catalog(tmp_path) -> None:
    """Инструмент memory_search зовёт search с browse_when_empty=True (#4956).

    Прямой search() без флага на «еда» пуст (#3890). Тул обязан просить каталог,
    иначе робот снова скажет «в памяти ничего нет».
    """
    import importlib.util
    import sys
    import types
    from pathlib import Path

    from rob_box_mcp_tools.base import MCPTool, MCPToolParameter, MCPToolResult

    pkg = "rob_box_mcp_tools"
    tools = f"{pkg}.tools"
    pkg_root = (
        Path(__file__).resolve().parents[5]
        / "src"
        / "rob_box_mcp_tools"
        / "rob_box_mcp_tools"
    )
    if pkg not in sys.modules:
        parent = types.ModuleType(pkg)
        parent.__path__ = [str(pkg_root)]
        sys.modules[pkg] = parent
    if tools not in sys.modules:
        tools_pkg = types.ModuleType(tools)
        tools_pkg.__path__ = [str(pkg_root / "tools")]
        sys.modules[tools] = tools_pkg
    if f"{tools}.base" not in sys.modules:
        base = types.ModuleType(f"{tools}.base")
        base.MCPTool = MCPTool
        base.MCPToolParameter = MCPToolParameter
        base.MCPToolResult = MCPToolResult
        sys.modules[f"{tools}.base"] = base

    mod_name = f"{tools}.memory"
    if mod_name not in sys.modules:
        spec = importlib.util.spec_from_file_location(
            mod_name, str(pkg_root / "tools" / "memory.py")
        )
        module = importlib.util.module_from_spec(spec)
        module.__package__ = tools
        sys.modules[mod_name] = module
        spec.loader.exec_module(module)
    MemorySearchTool = sys.modules[mod_name].MemorySearchTool

    memory = _memory(tmp_path)
    try:
        memory.save_fact(ONION, category="preference", speaker_id=SASHA)
        logger = types.SimpleNamespace(info=lambda *_a, **_k: None, error=lambda *_a, **_k: None)
        node = types.SimpleNamespace(voice_memory=memory, get_logger=lambda: logger)
        result = MemorySearchTool(node=node).execute(query="еда", speaker_id=SASHA)

        assert result.success
        contents = [r["content"] for r in result.data["results"]]
        assert ONION in contents
        assert all(r["source"] == "fact_browse" for r in result.data["results"])
    finally:
        memory.close()
