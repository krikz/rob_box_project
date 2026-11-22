"""memory_search не падает на ядре без browse_when_empty (issue #4961).

Живой тул на роботе отвечал success=false на любой запрос: инструмент из
develop звал ``search(..., browse_when_empty=True)``, а установленное ядро
v0.7.10 этого аргумента не знало и бросало TypeError. Адаптер просит каталог
только у ядра, которое флаг принимает, и честно сообщает, что каталог
недоступен, вместо того чтобы ронять поиск.
"""

from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path


def _load_memory_module():
    """tools/memory.py без package __init__ (тот тянет rclpy/nav2_msgs)."""
    pkg = "rob_box_mcp_tools"
    tools = f"{pkg}.tools"
    root = Path(__file__).resolve().parents[1] / "rob_box_mcp_tools"
    if pkg not in sys.modules:
        parent = types.ModuleType(pkg)
        parent.__path__ = [str(root)]
        sys.modules[pkg] = parent
    if tools not in sys.modules:
        tools_pkg = types.ModuleType(tools)
        tools_pkg.__path__ = [str(root / "tools")]
        sys.modules[tools] = tools_pkg
    if f"{pkg}.base" not in sys.modules:
        base_mod = types.ModuleType(f"{pkg}.base")
        sys.modules[f"{pkg}.base"] = base_mod
        base_src = (root / "base.py").read_text(encoding="utf-8")
        exec(compile(base_src, str(root / "base.py"), "exec"), base_mod.__dict__)  # noqa: S102
    if f"{tools}.base" not in sys.modules:
        shim = types.ModuleType(f"{tools}.base")
        for attr in ("MCPTool", "MCPToolParameter", "MCPToolResult"):
            setattr(shim, attr, getattr(sys.modules[f"{pkg}.base"], attr))
        sys.modules[f"{tools}.base"] = shim

    name = f"{tools}.memory"
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(name, str(root / "tools" / "memory.py"))
        module = importlib.util.module_from_spec(spec)
        module.__package__ = tools
        sys.modules[name] = module
        spec.loader.exec_module(module)
    return sys.modules[name]


memory_mod = _load_memory_module()
MemorySearchTool = memory_mod.MemorySearchTool

SASHA = "0da4521c-0b50-422b-be4e-402417f25659"
HIT = {
    "kind": "fact",
    "role": "fact",
    "content": "Пьёт только зелёный чай без сахара",
    "session_id": None,
    "score": 1.0,
    "source": "fact",
    "category": "preference",
}


class _OldCore:
    """Ядро v0.7.10: search() не знает browse_when_empty (сигнатура из карточки)."""

    def __init__(self, hits):
        self.hits = hits
        self.calls = []
        self.embedder = types.SimpleNamespace(is_available=lambda: False)

    def search(self, query, limit=5, speaker_id=None):
        self.calls.append((query, limit, speaker_id))
        return list(self.hits)


class _NewCore(_OldCore):
    """Ядро v0.7.11+: флаг принят, каталог отдаётся только с ним."""

    def search(self, query, limit=5, speaker_id=None, browse_when_empty=False):
        self.calls.append((query, limit, speaker_id, browse_when_empty))
        if browse_when_empty and not self.hits:
            return [{**HIT, "content": "каталог", "source": "fact_browse", "score": 0.0}]
        return list(self.hits)


def _tool(memory):
    logger = types.SimpleNamespace(info=lambda *_a, **_k: None, error=lambda *_a, **_k: None)
    node = types.SimpleNamespace(voice_memory=memory, get_logger=lambda: logger)
    return MemorySearchTool(node=node)


def test_old_core_search_does_not_fail():
    """Попадание на ядре без флага — success, а не «Ошибка поиска» (#4961)."""
    memory = _OldCore([HIT])

    result = _tool(memory).execute(query="чай", speaker_id=SASHA)

    assert result.success is True, result.message
    assert result.data["results"][0]["content"] == HIT["content"]
    assert result.data["results"][0]["source"] == "fact"
    assert memory.calls == [("чай", 5, SASHA)]


def test_old_core_miss_is_honest_about_missing_catalog():
    """Пусто на старом ядре — честный ответ, не падение и не выдуманный каталог."""
    memory = _OldCore([])

    result = _tool(memory).execute(query="еда", speaker_id=SASHA)

    assert result.success is True, result.message
    assert result.data["results"] == []
    assert result.data["catalog"] == "unavailable"
    assert "browse_when_empty" not in (result.message or "")


def test_new_core_still_asks_for_catalog():
    """Ядро с флагом по-прежнему получает browse_when_empty=True (#4956)."""
    memory = _NewCore([])

    result = _tool(memory).execute(query="еда", speaker_id=SASHA)

    assert result.success is True
    assert result.data["results"][0]["source"] == "fact_browse"
    assert "catalog" not in result.data
    assert memory.calls == [("еда", 5, SASHA, True)]


def test_adapter_asks_catalog_only_when_core_accepts_flag():
    """Решение в адаптере: флаг уходит только ядру, которое его принимает."""
    search_memory = memory_mod.search_memory
    old = _OldCore([HIT])
    new = _NewCore([HIT])

    assert search_memory(old, "чай", 5, SASHA) == ([HIT], False)
    assert old.calls == [("чай", 5, SASHA)]
    assert search_memory(new, "чай", 5, SASHA) == ([HIT], True)
    assert new.calls == [("чай", 5, SASHA, True)]
