"""Поиск факта по форме слова и каталог, когда совпадения нет (issue #4956).

Марафон #4950, шаг n1008: после сброса сессии запрос «еда» не находил
«Совсем не ест лук, даже жареный» — «еда» не подстрока факта, окно реплик
пустое, вектор на стенде мёртв (Ollama нет, ``vec=no``). Ту же дыру давал
запрос «пить» по факту «Пьёт только зелёный чай без сахара». Семьи форм
сводит ``same_inflection`` в коде поиска, без эмбеддингов (ADR-0148).
Каталог (``browse_when_empty=True``) остаётся для запроса, которого нет
ни в одном факте. Дефолт ``False`` сохраняет контракт #3890: запрос про
то, чего нет, остаётся пустым.
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


def test_eat_query_finds_onion_fact_not_tea(tmp_path) -> None:
    """«еда» находит «не ест лук» формой слова, не каталогом и не чаем."""
    memory = _memory(tmp_path)
    try:
        memory.save_fact(ONION, category="preference", speaker_id=SASHA)
        memory.save_fact(TEA, category="preference", speaker_id=SASHA)

        hits = memory.search("еда", limit=5, speaker_id=SASHA, browse_when_empty=True)

        assert [h["content"] for h in hits] == [ONION]
        assert hits[0]["source"] == "fact"
        assert hits[0]["kind"] == "fact"
        assert hits[0]["score"] == 1.0
        assert hits[0]["role"] == "fact"
        assert hits[0]["session_id"] is None
        assert hits[0]["speaker_id"] == SASHA
        assert hits[0]["category"] == "preference"
    finally:
        memory.close()


def test_drink_query_finds_tea_fact(tmp_path) -> None:
    """«пить» находит «Пьёт … чай»; лук в выдачу не попадает."""
    memory = _memory(tmp_path)
    try:
        memory.save_fact(ONION, category="preference", speaker_id=SASHA)
        memory.save_fact(TEA, category="preference", speaker_id=SASHA)

        hits = memory.search("пить", limit=5, speaker_id=SASHA)

        assert [h["content"] for h in hits] == [TEA]
        assert hits[0]["source"] == "fact"
        assert hits[0]["score"] == 1.0
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


def test_facts_found_after_session_reset_with_empty_turn_window(tmp_path) -> None:
    """После new_session окно текущей сессии пустое, а факт находится формой слова.

    ``new_session`` сбрасывает эпоху диалога, не строки ``voice_turns``.
    Пустое окно здесь — новая сессия без реплик: поиск не может опереться
    на FTS и всё равно возвращает факт (``source="fact"``, не каталог).
    """
    db_path = tmp_path / "voice_memory.db"
    first = VoiceMemory(db_path=str(db_path), session_id="before_reset")
    try:
        first.save_fact(ONION, category="preference", speaker_id=SASHA)
        first.save_fact(TEA, category="preference", speaker_id=SASHA)
    finally:
        first.close()

    reset = VoiceMemory(db_path=str(db_path), session_id="after_reset")
    try:
        assert reset.load_recent_turns(speaker_id=SASHA) == []
        hits = reset.search("еда", limit=5, speaker_id=SASHA, browse_when_empty=True)
        assert [h["content"] for h in hits] == [ONION]
        assert hits[0]["source"] == "fact"
        drink = reset.search("пить", limit=5, speaker_id=SASHA)
        assert [h["content"] for h in drink] == [TEA]
        assert drink[0]["source"] == "fact"
    finally:
        reset.close()


def test_inflection_hides_other_speaker_keeps_legacy_null(tmp_path) -> None:
    """Форма «ест» чужого спикера не отдаётся; legacy NULL с «еда» виден."""
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
        assert all(h["source"] == "fact" for h in hits)
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


def test_unrelated_query_stays_empty_without_browse(tmp_path) -> None:
    """Дефолт browse_when_empty=False: запрос не из семьи и не подстрока — [], как #3890."""
    memory = _memory(tmp_path)
    try:
        memory.save_fact(ONION, category="preference", speaker_id=SASHA)
        memory.save_fact(TEA, category="preference", speaker_id=SASHA)

        assert memory.search("кофе", limit=5, speaker_id=SASHA) == []
        assert (
            memory.search("кофе", limit=5, speaker_id=SASHA, browse_when_empty=False)
            == []
        )
    finally:
        memory.close()


def test_false_friend_is_not_an_inflection_hit(tmp_path) -> None:
    """«если» не семья «есть», «письмо» не семья «пить» — подстрока их не спасает."""
    memory = _memory(tmp_path)
    try:
        memory.save_fact("Если дождь — сидит дома", speaker_id=SASHA)
        memory.save_fact("Ждёт письмо от брата", speaker_id=SASHA)

        assert memory.search("еда", limit=5, speaker_id=SASHA) == []
        assert memory.search("пить", limit=5, speaker_id=SASHA) == []
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
        logger = types.SimpleNamespace(
            info=lambda *_a, **_k: None, error=lambda *_a, **_k: None
        )
        node = types.SimpleNamespace(voice_memory=memory, get_logger=lambda: logger)
        result = MemorySearchTool(node=node).execute(query="еда", speaker_id=SASHA)

        assert result.success
        contents = [r["content"] for r in result.data["results"]]
        assert contents == [ONION]
        assert all(r["source"] == "fact" for r in result.data["results"])
    finally:
        memory.close()
