"""Smoke test: import MemoryContextTool/MemorySaveTool/MemorySearchTool
without triggering rob_box_mcp_tools/tools/__init__.py (which loads
nav2_msgs, action_msgs, etc. — ROS2-only).

Strategy: import the module file directly via importlib + spec_from_file_location,
bypassing the package __init__. This lets us test the speaker_id plumbing
on CI builders that don't have rclpy installed.
"""
import importlib.util
import os
import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest


# ---------------------------------------------------------------------------
# Import bootstrap: load ``tools/memory.py`` without loading the package
# __init__.py (which transitively imports rclpy/nav2_msgs).
# ---------------------------------------------------------------------------

# Build a fake ``rob_box_mcp_tools`` package namespace and a sibling
# ``..base`` module.
_PKG_NAME = "rob_box_mcp_tools"
_TOOLS_NAME = f"{_PKG_NAME}.tools"
_BASE_NAME = f"{_TOOLS_NAME}.base"


def _install_base_module() -> None:
    """Provide a minimal ``rob_box_mcp_tools.tools.base`` shim."""
    base_src = (
        "from dataclasses import dataclass\n"
        "from enum import Enum\n"
        "from typing import Any, Dict, List, Optional\n"
        "from abc import ABC, abstractmethod\n"
        "class ToolExecutionType(Enum):\n"
        "    INSTANT='instant'; FAST='fast'; MEDIUM='medium'; LONG='long'\n"
        "@dataclass\n"
        "class MCPToolParameter:\n"
        "    name: str; type: str; description: str\n"
        "    required: bool = True\n"
        "    enum: Optional[List[str]] = None\n"
        "    properties: Optional[Dict[str, Any]] = None\n"
        "    items: Any = None\n"
        "    default: Any = None\n"
        "    def to_json_schema(self) -> Dict[str, Any]:\n"
        "        out = {'type': self.type, 'description': self.description}\n"
        "        if self.enum is not None: out['enum'] = self.enum\n"
        "        if self.default is not None: out['default'] = self.default\n"
        "        return out\n"
        "@dataclass\n"
        "class MCPToolResult:\n"
        "    success: bool\n"
        "    data: Optional[Dict[str, Any]] = None\n"
        "    error: Optional[str] = None\n"
        "    message: Optional[str] = None\n"
        "    def to_dict(self) -> Dict[str, Any]:\n"
        "        r = {'success': self.success}\n"
        "        if self.data is not None: r['data'] = self.data\n"
        "        if self.error is not None: r['error'] = self.error\n"
        "        if self.message is not None: r['message'] = self.message\n"
        "        return r\n"
        "class MCPTool(ABC):\n"
        "    def __init__(self, node=None): self.node = node\n"
        "    @property\n"
        "    @abstractmethod\n"
        "    def name(self): pass\n"
        "    @property\n"
        "    @abstractmethod\n"
        "    def description(self): pass\n"
        "    @property\n"
        "    @abstractmethod\n"
        "    def parameters(self): pass\n"
        "    @abstractmethod\n"
        "    def execute(self, **kw): pass\n"
        "    def log_info(self, msg): pass\n"
        "    def log_error(self, msg): pass\n"
    )
    base = types.ModuleType(_BASE_NAME)
    exec(base_src, base.__dict__)
    sys.modules[_BASE_NAME] = base


def _install_memory_module() -> None:
    """Load the actual ``tools/memory.py`` source bypassing the package."""
    memory_path = (
        Path(__file__).resolve().parent.parent.parent
        / "rob_box_mcp_tools"
        / "tools"
        / "memory.py"
    )
    spec = importlib.util.spec_from_file_location(
        f"{_TOOLS_NAME}.memory", str(memory_path)
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Failed to load spec for {memory_path}")
    module = importlib.util.module_from_spec(spec)
    # Set ``__package__`` so relative imports ``from ..base import ...`` work
    # even though we loaded the file directly (not via package __init__).
    module.__package__ = _TOOLS_NAME
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _ensure_packages() -> None:
    """Register the package namespace so ``from ..base import ...`` resolves.

    The real ``rob_box_mcp_tools/__init__.py`` and
    ``rob_box_mcp_tools/tools/__init__.py`` import ROS2-only modules
    (``nav2_msgs``, ``action_msgs`` …) at module load time, so we cannot
    import them in CI. We exec the ``__init__.py`` *sources* manually
    (they have no side-effect imports beyond docstrings/version), then
    wire up a parent package with a real ``__path__`` so relative
    imports (``from ..base``) work.
    """
    pkg_root = (
        Path(__file__).resolve().parent.parent.parent
        / "rob_box_mcp_tools"
    )
    tools_dir = pkg_root / "tools"

    # Parent package: rob_box_mcp_tools (needs __path__ for ``..`` imports).
    if _PKG_NAME not in sys.modules:
        parent = types.ModuleType(_PKG_NAME)
        parent.__path__ = [str(pkg_root)]
        sys.modules[_PKG_NAME] = parent
    # Sibling: rob_box_mcp_tools.tools.
    if _TOOLS_NAME not in sys.modules:
        tools_pkg = types.ModuleType(_TOOLS_NAME)
        tools_pkg.__path__ = [str(tools_dir)]
        sys.modules[_TOOLS_NAME] = tools_pkg

    # Exec the parent ``__init__.py`` (has only docstring + version).
    parent_init = pkg_root / "__init__.py"
    if parent_init.exists():
        with open(parent_init, encoding="utf-8") as fh:
            code = fh.read()
        exec(compile(code, str(parent_init), "exec"), sys.modules[_PKG_NAME].__dict__)

    # The ``tools/__init__.py`` would import every tool module
    # (navigation, music, …) and most of them require ``rclpy``. We need
    # only ``tools.memory`` for these tests, so leave the tools package
    # namespace empty (no exec, no per-tool imports).

    # ``rob_box_mcp_tools.base`` — real file, no ROS deps.
    base_src = (pkg_root / "base.py").read_text(encoding="utf-8")
    base_mod = types.ModuleType(f"{_PKG_NAME}.base")
    exec(compile(base_src, str(pkg_root / "base.py"), "exec"), base_mod.__dict__)
    sys.modules[f"{_PKG_NAME}.base"] = base_mod

    # ``rob_box_mcp_tools.tools.base`` (relative import target).
    _install_base_module()

    # Finally: ``rob_box_mcp_tools.tools.memory`` — the module under test.
    _install_memory_module()


_ensure_packages()
memory_mod = sys.modules[f"{_TOOLS_NAME}.memory"]

MemorySaveTool = memory_mod.MemorySaveTool
MemorySearchTool = memory_mod.MemorySearchTool
MemoryContextTool = memory_mod.MemoryContextTool


# ---------------------------------------------------------------------------
# In-memory fake of VoiceMemory: same surface as the real one but no SQLite.
# ---------------------------------------------------------------------------


class _FakeMemory:
    def __init__(self) -> None:
        self._turns: list[dict] = []
        self._facts: list[dict] = []
        self._next_turn = 0
        self._next_fact = 0
        self.embedder = type(
            "E", (), {"is_available": staticmethod(lambda: False)}
        )()

    def save_turn(self, role, content, *, session_id=None, speaker_id=None) -> int:
        self._next_turn += 1
        self._turns.append(
            {
                "id": self._next_turn, "role": role, "content": content,
                "session_id": session_id, "speaker_id": speaker_id,
            }
        )
        return self._next_turn

    def save_fact(self, fact, category="general", *, speaker_id=None) -> int:
        self._next_fact += 1
        self._facts.append(
            {
                "id": self._next_fact, "fact": fact, "category": category,
                "speaker_id": speaker_id,
            }
        )
        return self._next_fact

    def get_facts(self, category=None, limit=20, *, speaker_id=None):
        out = []
        for f in self._facts:
            if category and f["category"] != category:
                continue
            if speaker_id and f["speaker_id"] != speaker_id and f["speaker_id"] is not None:
                continue
            out.append(f)
            if len(out) >= limit:
                break
        return out

    def search(self, query, limit=5, *, speaker_id=None):
        out = []
        for t in self._turns:
            if (
                speaker_id
                and t["speaker_id"] != speaker_id
                and t["speaker_id"] is not None
            ):
                continue
            if query.lower() in t["content"].lower():
                out.append(
                    {
                        "id": t["id"], "role": t["role"],
                        "content": t["content"],
                        "session_id": t["session_id"],
                        "speaker_id": t["speaker_id"],
                        "score": 1.0, "source": "fts",
                    }
                )
                if len(out) >= limit:
                    break
        return out

    def get_context(self, limit=10, query=None, *, speaker_id=None):
        return {
            "recent_turns": [
                {
                    "id": t["id"], "role": t["role"],
                    "content": t["content"],
                    "session_id": t["session_id"],
                    "speaker_id": t["speaker_id"],
                }
                for t in self._turns
            ][:limit],
            "facts": self.get_facts(speaker_id=speaker_id),
            "total_turns": len(self._turns),
            "sessions": 1,
            "vec_enabled": False,
            "current_session": "issue_1770",
        }

    def format_facts_for_prompt(self, *, speaker_id=None) -> str:
        facts = self.get_facts(speaker_id=speaker_id)
        if not facts:
            return ""
        return "Known user facts:\n" + "\n".join(f"- {f['fact']}" for f in facts)

    def get_stats(self) -> dict:
        return {
            "turn_count": len(self._turns),
            "session_count": 1,
            "fact_count": len(self._facts),
            "vec_count": 0,
            "db_size_kb": 0,
            "vec_enabled": False,
            "ollama_available": False,
            "current_session": "issue_1770",
        }


class _FakeNode:
    def __init__(self, current_speaker_id=None) -> None:
        self.voice_memory = _FakeMemory()
        self.current_speaker_id = current_speaker_id
        # Real ``MCPTool.log_info``/``log_error`` calls ``self.node.get_logger()``.
        self._logs: list[str] = []

    def get_logger(self):
        logger = MagicMock()
        logger.info.side_effect = self._logs.append
        logger.warning.side_effect = self._logs.append
        logger.error.side_effect = self._logs.append
        logger.debug.side_effect = self._logs.append
        return logger


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


class TestMemorySaveSpeakerScope:
    def test_save_persists_explicit_speaker_id(self):
        node = _FakeNode()
        tool = MemorySaveTool(node)
        result = tool.execute(
            fact="Денчик любит лыжи", speaker_id="denchik-uuid"
        )
        assert result.success
        assert result.data["speaker_id"] == "denchik-uuid"
        assert node.voice_memory._facts[0]["speaker_id"] == "denchik-uuid"

    def test_save_falls_back_to_node_current_speaker_id(self):
        node = _FakeNode(current_speaker_id="denchik-uuid")
        tool = MemorySaveTool(node)
        result = tool.execute(fact="Денчик любит лыжи")
        assert result.success
        assert result.data["speaker_id"] == "denchik-uuid"
        assert node.voice_memory._facts[0]["speaker_id"] == "denchik-uuid"

    def test_save_without_any_speaker_stays_global(self):
        node = _FakeNode()
        tool = MemorySaveTool(node)
        result = tool.execute(fact="глобальный факт")
        assert result.success
        assert result.data["speaker_id"] is None
        assert node.voice_memory._facts[0]["speaker_id"] is None


class TestMemorySearchSpeakerScope:
    def test_search_filters_by_explicit_speaker_id(self):
        node = _FakeNode()
        node.voice_memory.save_turn("user", "Денчик про лыжи", speaker_id="denchik")
        node.voice_memory.save_turn("user", "Саша про музыку", speaker_id="sasha")
        tool = MemorySearchTool(node)
        result = tool.execute(query="лыжи", speaker_id="denchik")
        assert result.success
        assert result.data["speaker_id"] == "denchik"
        contents = [r["content"] for r in result.data["results"]]
        assert any("лыжи" in c for c in contents)
        assert not any("Саша" in c for c in contents)

    def test_search_falls_back_to_node_current_speaker_id(self):
        node = _FakeNode(current_speaker_id="denchik")
        node.voice_memory.save_turn("user", "Денчик про лыжи", speaker_id="denchik")
        node.voice_memory.save_turn("user", "Саша про музыку", speaker_id="sasha")
        tool = MemorySearchTool(node)
        result = tool.execute(query="про")
        assert result.success
        assert result.data["speaker_id"] == "denchik"
        contents = [r["content"] for r in result.data["results"]]
        assert all("Саша" not in c for c in contents)


class TestMemoryContextSpeakerScope:
    def test_context_filters_facts_by_explicit_speaker_id(self):
        node = _FakeNode()
        node.voice_memory.save_fact("Денчик любит лыжи", speaker_id="denchik")
        node.voice_memory.save_fact("Саша любит музыку", speaker_id="sasha")
        tool = MemoryContextTool(node)
        result = tool.execute(speaker_id="denchik")
        assert result.success
        fact_texts = [f["fact"] for f in result.data["facts"]]
        assert any("лыжи" in t for t in fact_texts)
        assert not any("Саша" in t for t in fact_texts)

    def test_context_falls_back_to_node_current_speaker_id(self):
        node = _FakeNode(current_speaker_id="denchik")
        node.voice_memory.save_fact("Денчик любит лыжи", speaker_id="denchik")
        node.voice_memory.save_fact("Саша любит музыку", speaker_id="sasha")
        tool = MemoryContextTool(node)
        result = tool.execute()
        assert result.success
        assert result.data["speaker_id"] == "denchik"
        fact_texts = [f["fact"] for f in result.data["facts"]]
        assert not any("Саша" in t for t in fact_texts)
