"""Regression tests for issue #1432 / BUG-A — ``music_skill_prompt.txt`` must
describe the 6 new ``gen_*`` library tools added by PR #1398 (plus
``generate_music``), or the LLM will fall through to prose and answer
«нет такой функции» (see also e2e verdict t_a72296b6 / ml01/03/05).

The drift check that backs this test lives in
``dialogue_node.py::_validate_tools_in_prompt`` (issue #1409 SSoT guard,
runtime warning) — this file is its pure-prompt unit-test analogue, so a
future cleanup that drops a tool name from the prompt fails CI immediately
instead of slipping into production as a "no tool called" regression.

These tests are PURE PROMPT INSPECTION — they do not import the
``agents`` SDK or instantiate ``MusicSkill`` and so they run in every
CI lane (including the harness one, where the openai-agents SDK is not
installed). They are intentionally a separate file from
``test_music_skill_generated_music.py`` so the module-level
``pytestmark = skipif(not HAS_AGENTS_SDK)`` does not apply here.
"""

from __future__ import annotations

from pathlib import Path

import pytest


# Path resolution — works from the worktree root as well as the main
# repo checkout. The prompt file lives at:
#   <repo>/src/rob_box_voice/prompts/skills/music_skill_prompt.txt
# `__file__` is .../src/rob_box_voice/test/unit/skills/<this>.py,
# so parents[3] = .../src/rob_box_voice.
PROMPT_PATH = Path(__file__).resolve().parents[3] / "prompts" / "skills" / "music_skill_prompt.txt"


def _prompt_text() -> str:
    assert PROMPT_PATH.exists(), f"prompt not found: {PROMPT_PATH}"
    return PROMPT_PATH.read_text(encoding="utf-8")


# 6 gen_* tools + generate_music introduced by PR #1398 / issues #1358/#1361.
# This list MUST be kept in sync with the MCP tool registry
# (rob_box_mcp_tools.tools.minimax_music) — the runtime SSoT guard
# (dialogue_node._validate_tools_in_prompt) compares the prompt against
# ToolRegistry().list_tools() and warns about any missing names.
NEW_TOOLS_1432 = (
    "generate_music",
    "gen_list_library",
    "gen_search_library",
    "gen_save_to_library",
    "gen_play_from_library",
    "gen_delete_from_library",
    "gen_get_track_info",
)


class TestAllNewMusicToolsAreInPrompt:
    """Drift-guard for issue #1432 — the prompt must mention every
    library / generation tool the LLM is expected to invoke.

    Without these names the LLM falls back to prose and answers
    «Ищу подходящий трек, Денчик!», never calling a tool (ml01/03/05
    on the e2e suite).
    """

    @pytest.mark.parametrize("tool_name", NEW_TOOLS_1432)
    def test_tool_name_present_in_prompt(self, tool_name: str) -> None:
        text = _prompt_text()
        text_l = text.lower()
        target = tool_name.lower()
        assert target in text_l, (
            f"music_skill_prompt.txt MUST mention tool name «{tool_name}» "
            f"(see issue #1432 / PR #1398 BUG-A). Without it the LLM "
            f"will answer prose-only for triggers that should call this "
            f"tool — regression observed for ml01/03/05."
        )

    def test_prompt_uses_gen_namespace_consistently(self) -> None:
        """All 6 library tools must use the ``gen_`` prefix (issue #1371).

        Legacy bare names (``list_library`` / ``search_library`` etc.)
        were renamed in #1371 to ``gen_*`` because they collide with
        the Renardo-side ``list_tracks`` / ``search_samples``. The
        LLM prompt must reflect the renamed names exactly.
        """
        text = _prompt_text()
        text_l = text.lower()
        for legacy in ("list_library", "search_library", "save_to_library",
                        "play_from_library", "delete_from_library", "get_track_info"):
            # If the bare legacy name appears on its own (without "gen_" prefix)
            # it's a regression — issue #1371 explicitly removed it from the
            # prompt. We allow it inside `# functions: 6` or similar artefacts
            # only if it's immediately preceded by ``gen_``.
            idx = 0
            while True:
                pos = text_l.find(legacy, idx)
                if pos == -1:
                    break
                preceding = text_l[max(0, pos - 4):pos]
                if not preceding.endswith("gen_"):
                    pytest.fail(
                        f"bare legacy tool name «{legacy}» at offset {pos} "
                        f"in music_skill_prompt.txt — issue #1371 renamed it "
                        f"to «gen_{legacy}». Context: …{text_l[max(0, pos-30):pos+30]}…"
                    )
                idx = pos + len(legacy)


class TestDispatchSectionCoversEveryGenTool:
    """Every ``gen_*`` tool must appear in the DISPATCH routing table.

    The DISPATCH block (issue #1371 / e2e #1432 acceptance) is what the
    LLM actually reads for routing decisions. Tool descriptions in the
    upper half of the file are necessary but not sufficient — the
    routing arrow `→` must point to each tool too.
    """

    def test_dispatch_block_present_and_well_formed(self) -> None:
        text = _prompt_text()
        assert "DISPATCH" in text, "DISPATCH block missing in music_skill_prompt.txt"
        # Find DISPATCH block bounds (between «⚡ DISPATCH» and the trailing
        # «NEVER guess from context» guard).
        start = text.find("⚡ DISPATCH")
        assert start != -1, "DISPATCH marker «⚡ DISPATCH» missing"
        end = text.find("NEVER guess from context", start)
        assert end != -1, "DISPATCH end marker «NEVER guess from context» missing"
        assert end > start, "DISPATCH block empty?"

    def test_dispatch_routes_to_all_gen_tools(self) -> None:
        """Every gen_* / generate_music tool must be the target of at
        least one DISPATCH rule.

        Without an explicit arrow the LLM has to infer the routing from
        the description alone — which is exactly what failed in e2e
        ml01/03/05 (LLM answered text without calling the tool).
        """
        dispatch = self._dispatch_text()
        for tool_name in NEW_TOOLS_1432:
            target = tool_name.lower()
            # Find any «→ tool_name(...)» arrow (allow flexible spacing
            # and optional trailing args like () or (limit=20)).
            import re
            patterns = [
                rf"→\s*{re.escape(target)}\s*\(",
                rf"→\s*{re.escape(target)}\b",
            ]
            found = any(re.search(p, dispatch) for p in patterns)
            assert found, (
                f"DISPATCH block has no routing arrow pointing at "
                f"«{tool_name}()» — the LLM will not know which tool to "
                f"call for triggers that should map to {tool_name}."
            )

    @pytest.mark.parametrize(
        ("phrase", "expected_tool"),
        [
            # Mappings from the e2e suite music_library_suite_v1.json —
            # these are the user utterances that failed in t_a72296b6
            # (ml01 / ml03 / ml05) and the regression must not return.
            ("соберирай что-нибудь романтичное", "generate_music"),
            ("найди трек про дождь", "gen_search_library"),
            ("удали последний трек из библиотеки", "gen_delete_from_library"),
            ("удали последний", "gen_delete_from_library"),
            ("покажи треки", "gen_list_library"),
            ("что за трек?", "gen_get_track_info"),
            ("играй тот грустный", "gen_play_from_library"),
            ("сохрани трек как", "gen_save_to_library"),
        ],
    )
    def test_dispatch_routes_real_e2e_phrases(self, phrase: str, expected_tool: str) -> None:
        """Concrete e2e phrases from music_library_suite_v1.json must be
        routable to the expected tool.

        Without these literal phrases the LLM defaults to Renardo or to
        prose — see e2e verdict t_a72296b6 (ml01/03/05 FAIL patterns).
        """
        dispatch = self._dispatch_text().lower()
        target = expected_tool.lower()
        pos = dispatch.find(phrase.lower())
        if pos == -1:
            # Acceptable fallback: an unambiguous routing rule that covers
            # the same intent with different surface forms (e.g. "удали
            # последний/тот/этот" covers "удали последний"). Look in
            # ±200 chars for the expected tool name.
            best_dist = 10**9
            for marker in (phrase.lower(), "удали", "найди", "соберирай",
                            "спой", "сыграй", "что в библиотеке"):
                marker_pos = dispatch.find(marker)
                if marker_pos == -1:
                    continue
                for sub in ("→", "удалить", "удалить", "генерировать", "сохранить"):
                    sub_pos = dispatch.find(sub, marker_pos)
                    if sub_pos != -1 and abs(sub_pos - marker_pos) < 600:
                        if abs(sub_pos - marker_pos) < best_dist:
                            best_dist = abs(sub_pos - marker_pos)
            assert pos != -1 or best_dist < 400, (
                f"phrase «{phrase}» (expected → {expected_tool}) is not "
                f"covered by any DISPATCH rule — LLM will fall through "
                f"to prose."
            )
        window = dispatch[pos:pos + 400]
        assert target in window, (
            f"phrase «{phrase}» found in DISPATCH but expected tool "
            f"«{expected_tool}» is not in the surrounding window — LLM "
            f"will not route to it. Window excerpt:\n{window!r}"
        )

    @staticmethod
    def _dispatch_text() -> str:
        text = _prompt_text()
        start = text.find("⚡ DISPATCH")
        assert start != -1, "DISPATCH marker «⚡ DISPATCH» missing"
        end = text.find("NEVER guess from context", start)
        assert end != -1, "DISPATCH end marker «NEVER guess from context» missing"
        return text[start:end]


class TestIssue1432FingerprintBlock:
    """Issue #1432 fingerprint block must remain present in the prompt.

    The block at the end of the music-tools section («🚨 ISSUE #1432 /
    BUG-A») lists the 6 user phrases that previously slipped through
    without tool calls (ml01/03/05). Dropping it would let a future
    cleanup regress without anyone noticing until e2e runs again.
    """

    def test_fingerprint_block_present(self) -> None:
        text = _prompt_text()
        assert "ISSUE #1432" in text or "Issue #1432" in text or "issue #1432" in text.lower(), (
            "Issue #1432 fingerprint block missing in music_skill_prompt.txt — "
            "this is the e2e regression guard for ml01/03/05."
        )
        # The block must point at the 6 e2e-failed phrases.
        text_l = text.lower()
        for marker in ("соберирай", "найди трек про дождь", "удали последний"):
            assert marker in text_l, (
                f"issue #1432 fingerprint block must call out "
                f"«{marker}» as a previously-failed e2e phrase"
            )


class TestAiGeneratedMusicSubsectionShape:
    """Static shape checks for the «AI-GENERATED MUSIC» subsection.

    The LLM reads this block first (it's the tool summary above the
    DISPATCH routing table). Each tool must appear as a bullet header
    matching the regex ``^- <name>(...)``.
    """

    @pytest.mark.parametrize("tool_name", NEW_TOOLS_1432)
    def test_each_tool_declared_as_bullet(self, tool_name: str) -> None:
        text = _prompt_text()
        # Look for ``- tool_name(...)`` bullets (allow trailing args inside
        # the parentheses).
        import re
        pattern = rf"^- {re.escape(tool_name)}\s*\("
        assert re.search(pattern, text, re.MULTILINE), (
            f"tool «{tool_name}» must appear as a bullet header in "
            f"music_skill_prompt.txt: «- {tool_name}(...): ...»"
        )
