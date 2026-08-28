"""
test_system_prompt_tools_check.py — Issue #1409 / ARCH-review #1405

SSoT tools-vs-prompt validation in ``DialogueNode._load_system_prompt``.

The music skill prompt is a static contract the LLM reads verbatim at
startup. If an MCP tool gets registered but never mentioned in the
prompt, the LLM silently degrades to «нет такой функции» fallback
(see issue #1403). This test suite locks down the runtime guard that
warns about the drift.

Не требует ROS2 — rclpy замокан в conftest.py.
"""

import json
from unittest.mock import MagicMock, patch

import pytest

from rob_box_voice.dialogue_node import DialogueNode


# ─────────────────────────────────────────────────────────────────────────────
#  Fixtures
# ─────────────────────────────────────────────────────────────────────────────


@pytest.fixture
def node():
    """Минимальная DialogueNode без __init__ (как в test_pure_methods)."""
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n._logger = logger
    n.get_logger = lambda: logger

    # SSoT set (заполняется в __init__ из ToolRegistry().list_tools())
    n._mcp_tool_names = set()

    # Параметры (get_parameter stub)
    n._declared_params = {"system_prompt_file": "music_skill_prompt.txt"}

    def _get_parameter(name):
        return type("P", (), {"value": n._declared_params.get(name)})()

    n.get_parameter = _get_parameter

    return n


# ─────────────────────────────────────────────────────────────────────────────
#  _collect_mcp_tool_names — SSoT source
# ─────────────────────────────────────────────────────────────────────────────


class TestCollectMcpToolNames:
    """``_collect_mcp_tool_names`` returns the canonical tool set."""

    def test_returns_names_from_tool_registry(self, node):
        # Stub ToolRegistry().list_tools() to return two specs.
        fake_spec_a = MagicMock(name="ToolSpecA")
        fake_spec_a.name = "generate_music"
        fake_spec_b = MagicMock(name="ToolSpecB")
        fake_spec_b.name = "search_library"
        with patch(
            "rob_box_voice.dialogue_node.ToolRegistry"
        ) as fake_registry:
            fake_registry.return_value.list_tools.return_value = (
                fake_spec_a,
                fake_spec_b,
            )
            names = node._collect_mcp_tool_names()
        assert names == {"generate_music", "search_library"}

    def test_returns_empty_set_on_registry_failure(self, node):
        """If ToolRegistry() raises, return an empty set (don't crash)."""
        with patch(
            "rob_box_voice.dialogue_node.ToolRegistry",
            side_effect=RuntimeError("registry not importable"),
        ):
            names = node._collect_mcp_tool_names()
        assert names == set()
        # Operator sees a warning explaining the skip.
        node.get_logger().warning.assert_called()

    def test_returns_empty_set_when_registry_empty(self, node):
        """An empty registry (e.g. fake deployment) yields an empty set."""
        with patch(
            "rob_box_voice.dialogue_node.ToolRegistry"
        ) as fake_registry:
            fake_registry.return_value.list_tools.return_value = ()
            names = node._collect_mcp_tool_names()
        assert names == set()


# ─────────────────────────────────────────────────────────────────────────────
#  _validate_tools_in_prompt — guard logic
# ─────────────────────────────────────────────────────────────────────────────


class TestValidateToolsInPrompt:
    """The validation only fires for music-domain prompts (ARCH #1405/B)."""

    PROMPT_FULL = (
        "You are the РОББОКС music module.\n"
        "STEP 1. execute_music_code() to play.\n"
        "STEP 2. set_dj_mode(enabled=True).\n"
        "Optional: generate_music(prompt=...) to compose songs.\n"
        "Search: search_library(query=...) returns matching tracks.\n"
    )
    PROMPT_MISSING_MUSIC_TOOLS = (
        "You are the РОББОКС music module.\n"
        "STEP 1. execute_music_code() to play.\n"
        # NO generate_music, NO search_library, NO list_library
    )

    def test_music_prompt_missing_tools_logs_warning(self, node):
        """AC.A — registry has [generate_music, search_library], prompt
        mentions neither → warning."""
        node._mcp_tool_names = {"generate_music", "search_library"}
        node._validate_tools_in_prompt(
            "music_skill_prompt.txt", self.PROMPT_MISSING_MUSIC_TOOLS
        )
        node.get_logger().warning.assert_called()
        msg = node.get_logger().warning.call_args[0][0]
        assert "[issue 1409]" in msg
        assert "generate_music" in msg
        assert "search_library" in msg

    def test_music_prompt_partial_coverage_lists_only_missing(self, node):
        """AC.C-2 — prompt mentions generate_music but NOT search_library
        → warning only for search_library."""
        node._mcp_tool_names = {"generate_music", "search_library"}
        prompt = (
            "You are the РОББОКС music module.\n"
            "Use generate_music(prompt=...) to compose songs.\n"
            # NO search_library
        )
        node._validate_tools_in_prompt("music_skill_prompt.txt", prompt)
        node.get_logger().warning.assert_called()
        msg = node.get_logger().warning.call_args[0][0]
        assert "search_library" in msg
        # generate_music IS mentioned — should NOT appear in the warning
        # alongside search_library (sorted list of missing).
        assert "generate_music" not in msg.split("not described")[1]

    def test_music_prompt_full_coverage_no_warning(self, node):
        """AC.C-3 — all tools mentioned → no warning, debug log only."""
        node._mcp_tool_names = {"generate_music", "search_library"}
        node._validate_tools_in_prompt(
            "music_skill_prompt.txt", self.PROMPT_FULL
        )
        node.get_logger().warning.assert_not_called()
        # Debug log confirms the check ran.
        node.get_logger().debug.assert_called()
        msg = node.get_logger().debug.call_args[0][0]
        assert "[issue 1409]" in msg
        assert "2 MCP tools" in msg

    def test_empty_tool_set_skips_silently(self, node):
        """AC.C-3 — registry empty → no warning (don't spam the log)."""
        node._mcp_tool_names = set()
        node._validate_tools_in_prompt(
            "music_skill_prompt.txt", self.PROMPT_MISSING_MUSIC_TOOLS
        )
        node.get_logger().warning.assert_not_called()

    def test_non_music_prompt_skipped(self, node):
        """AC.B — only music_skill_prompt.txt triggers the check.

        Other domain prompts (FAQ, navigation, web_search) stay
        unchecked for now — that's a future-cycle TODO.
        """
        node._mcp_tool_names = {"generate_music"}
        node._validate_tools_in_prompt(
            "master_prompt_compact.txt",
            "Ты ROBBOX — умный робот-ассистент. Без упоминаний tools.",
        )
        node.get_logger().warning.assert_not_called()
        node.get_logger().debug.assert_not_called()

    def test_case_insensitive_match(self, node):
        """The check is case-insensitive (AC.A says case-insensitive)."""
        node._mcp_tool_names = {"generate_music", "set_dj_mode"}
        prompt = (
            "Use Generate_Music() or SET_DJ_MODE() to control playback."
        )
        node._validate_tools_in_prompt("music_skill_prompt.txt", prompt)
        node.get_logger().warning.assert_not_called()

    def test_missing_attribute_skips_silently(self, node):
        """If ``_mcp_tool_names`` is missing (e.g. early init failure),
        the guard must not crash."""
        if hasattr(node, "_mcp_tool_names"):
            del node._mcp_tool_names
        # Should NOT raise — getattr with default handles it.
        node._validate_tools_in_prompt(
            "music_skill_prompt.txt", self.PROMPT_MISSING_MUSIC_TOOLS
        )
        node.get_logger().warning.assert_not_called()


# ─────────────────────────────────────────────────────────────────────────────
#  _on_mcp_tools_update — keeps _mcp_tool_names in sync
# ─────────────────────────────────────────────────────────────────────────────


class TestOnMcpToolsUpdateSyncsMcpToolNames:
    """Issue #1409 — /mcp/tools refresh updates ``_mcp_tool_names`` too."""

    def _make_msg(self, data: str):
        msg = MagicMock()
        msg.data = data
        return msg

    def test_valid_payload_populates_mcp_tool_names(self, node):
        tools = [
            {"function": {"name": "generate_music"}, "type": "function"},
            {"function": {"name": "search_library"}, "type": "function"},
            {"function": {"name": "list_library"}, "type": "function"},
        ]
        node._on_mcp_tools_update(self._make_msg(json.dumps(tools)))
        assert node.mcp_tools_available is True
        assert node._mcp_tool_names == {
            "generate_music",
            "search_library",
            "list_library",
        }

    def test_empty_list_clears_mcp_tool_names(self, node):
        """An empty refresh → ``_mcp_tool_names`` becomes empty too
        (mirror of ``available_tools`` behaviour)."""
        node._mcp_tool_names = {"stale_tool"}
        node._on_mcp_tools_update(self._make_msg("[]"))
        assert node._mcp_tool_names == set()
        assert node.available_tools == []

    def test_invalid_json_does_not_clobber_existing_names(self, node):
        """If /mcp/tools sends garbage, we don't lose the SSoT set — the
        operator should fix the bridge, not have the LLM lose tools."""
        node._mcp_tool_names = {"generate_music"}
        node._on_mcp_tools_update(self._make_msg("{invalid json"))
        # _mcp_tool_names untouched (no overwrite with garbage).
        assert node._mcp_tool_names == {"generate_music"}
        assert node.mcp_tools_available is False

    def test_skips_entries_without_function_name(self, node):
        """Malformed tool dicts (no ``function.name``) are filtered out
        rather than polluting the set with empty strings."""
        tools = [
            {"function": {"name": "generate_music"}, "type": "function"},
            {"function": {}, "type": "function"},  # no name
            {"type": "function"},  # no function block
            "not a dict",  # not a dict
        ]
        node._on_mcp_tools_update(self._make_msg(json.dumps(tools)))
        assert node._mcp_tool_names == {"generate_music"}
