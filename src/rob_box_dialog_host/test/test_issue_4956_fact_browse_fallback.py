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
        assert memory.search("кофе", limit=5, speaker_id=SASHA, browse_when_empty=False) == []
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

    Прямой search() без флага на незнакомое слово пуст (#3890). Тул обязан
    просить каталог, иначе робот снова скажет «в памяти ничего нет». «еда»
    теперь находится формой слова (``source="fact"``), поэтому каталог здесь
    проверяется запросом, которого нет ни в одном факте.
    """
    import importlib.util
    import sys
    import types
    from pathlib import Path

    # Зеркало монорепо, не ядро: ``rob_box_mcp_tools.tools`` импортирует навигацию,
    # а та — rclpy. На роботе ROS есть, в прогоне пакета здесь его нет. Грузим
    # только tools/memory.py, как src/rob_box_voice/test/unit/core/test_issue_4956_fact_browse_fallback.py.
    # Контракт ядра v0.7.31 сохранён: каталог проверяется словом, которого нет в фактах.
    from rob_box_mcp_tools.base import MCPTool, MCPToolParameter, MCPToolResult

    pkg = "rob_box_mcp_tools"
    tools = f"{pkg}.tools"
    pkg_root = Path(__file__).resolve().parents[2] / "rob_box_mcp_tools" / "rob_box_mcp_tools"
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
        spec = importlib.util.spec_from_file_location(mod_name, str(pkg_root / "tools" / "memory.py"))
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
        result = MemorySearchTool(node=node).execute(query="погода", speaker_id=SASHA)

        assert result.success
        contents = [r["content"] for r in result.data["results"]]
        assert contents == [ONION]
        assert all(r["source"] == "fact_browse" for r in result.data["results"])
    finally:
        memory.close()


def test_generalized_word_finds_fact_by_substring(tmp_path) -> None:
    """Обобщённое слово вне семей форм находит факт подстрокой слова (#4956).

    «лук» не семья «еда»/«пить», но целая подстрока слова «луковицу». Это
    поиск (``source="fact"``), не каталог: чай в выдачу не попадает.
    Регистр схлопывается. «ё» схлопывается с «е»: «зеленый» находит
    «зелёный» — слово факта после схлопывания равно токену.
    """
    memory = _memory(tmp_path)
    try:
        memory.save_fact("Любит луковицу в салате", category="preference", speaker_id=SASHA)
        memory.save_fact(TEA, category="preference", speaker_id=SASHA)

        hits = memory.search("лук", limit=5, speaker_id=SASHA, browse_when_empty=True)

        assert [h["content"] for h in hits] == ["Любит луковицу в салате"]
        assert hits[0]["source"] == "fact"
        # «лук» — подстрока слова «луковицу», не равенство слов.
        assert hits[0]["score"] == 0.5

        folded = memory.search("ЛуК", limit=5, speaker_id=SASHA)
        assert [h["content"] for h in folded] == ["Любит луковицу в салате"]

        yo_hit = memory.search("зеленый", limit=5, speaker_id=SASHA)
        assert [h["content"] for h in yo_hit] == [TEA]
        assert yo_hit[0]["source"] == "fact"
        assert yo_hit[0]["score"] == 1.0
    finally:
        memory.close()


def test_whole_token_outranks_nothing_and_reverse_is_empty(tmp_path) -> None:
    """Целое слово факта сильнее подстроки; длинный запрос короткое не находит.

    «луковицу» находит свой факт (1.0). Обратное «луковица» не находит «лук»:
    подстрока только «короткое внутри длинного слова факта». Иначе «кухне»
    находила бы «не». Другая форма («луковица» при факте «луковицу») тоже
    пуста — это не стеммер, контракт #3890 («пицца» не «пиццу»).
    """
    memory = _memory(tmp_path)
    try:
        memory.save_fact("Ест лук сырым", speaker_id=SASHA)
        memory.save_fact("Любит луковицу в салате", speaker_id=SASHA)

        hits = memory.search("луковицу", limit=5, speaker_id=SASHA)

        assert [h["content"] for h in hits] == ["Любит луковицу в салате"]
        assert [h["score"] for h in hits] == [1.0]
        assert hits[0]["source"] == "fact"
        assert memory.search("луковица", limit=5, speaker_id=SASHA) == []
    finally:
        memory.close()


def test_substring_does_not_false_match(tmp_path) -> None:
    """Нет ложных совпадений: чужой корень, короткое слово, чужой спикер."""
    memory = _memory(tmp_path)
    try:
        memory.save_fact(ONION, category="preference", speaker_id=SASHA)
        memory.save_fact("Если дождь — сидит дома", speaker_id=SASHA)
        memory.save_fact("Ждёт письмо от брата", speaker_id=SASHA)
        memory.save_fact("Денчик любит луковицу", speaker_id="denchik")

        # «еда» находит лук семьёй форм — это не ложь. Ложь — чужой корень.
        eat = memory.search("еда", limit=5, speaker_id=SASHA)
        assert [h["content"] for h in eat] == [ONION]
        assert memory.search("кофе", limit=5, speaker_id=SASHA) == []
        assert memory.search("а", limit=5, speaker_id=SASHA) == []
        # «лук» в факте Саши есть целиком — это попадание, не ложь.
        assert [h["content"] for h in memory.search("лук", limit=5, speaker_id=SASHA)] == [ONION]
        denchik = memory.search("лук", limit=5, speaker_id="denchik")
        assert [h["content"] for h in denchik] == ["Денчик любит луковицу"]
        # Другая форма — не стеммер: «пицца» не находит «пиццу» (#3890).
        memory.save_fact("Борис всегда приносит пиццу", speaker_id=SASHA)
        assert memory.search("пицца", limit=5, speaker_id=SASHA) == []
        assert [h["content"] for h in memory.search("пиццу", limit=5, speaker_id=SASHA)] == [
            "Борис всегда приносит пиццу",
        ]
        assert memory.search("   ", limit=5, speaker_id=SASHA) == []
    finally:
        memory.close()


def test_empty_db_and_vec_unavailable(tmp_path, monkeypatch) -> None:
    """Пустая БД → [], даже с каталогом. Эмбеддер недоступен: поиск не падает."""
    memory = _memory(tmp_path)
    try:
        monkeypatch.setattr(memory.embedder, "is_available", lambda: False)
        assert memory.embedder.is_available() is False

        assert memory.search("еда", limit=5, speaker_id=SASHA) == []
        assert memory.search("еда", limit=5, speaker_id=SASHA, browse_when_empty=True) == []
        assert memory.search("лук", limit=5, speaker_id=SASHA, browse_when_empty=True) == []

        memory.save_fact("Любит луковицу в салате", speaker_id=SASHA)
        hits = memory.search("лук", limit=5, speaker_id=SASHA)
        assert [h["content"] for h in hits] == ["Любит луковицу в салате"]
        assert hits[0]["source"] == "fact"
        assert memory.embedder.is_available() is False
    finally:
        memory.close()
