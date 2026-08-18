#!/usr/bin/env python3
"""Static regression guard for issue #1219 (voice-change feature, MiniMax live).

Issue #1219 — ``set_voice`` tool is correctly registered in mcp_server.py
and LLM receives the live voice list every turn via
``<tts_context>[TTS] voices: ...``. BUT the system prompt
``master_prompt_compact.txt`` (which dialogue_node.py loads by default —
see ``system_prompt_file`` param) had NO instruction telling the LLM how
to use the tool. MiniMax-M3, when asked "говори ему это женским голосом",
silently answered "Ок." with zero tool calls; tts_node then reported:

    Голос 'None' недоступен у MiniMax — использую дефолтный 'male-qn-qingse'

and the user heard the same default male voice — feature looked broken.

Fix: explicit ``RULE #VOICE`` block in master_prompt_compact.txt with
mapping hints (female-/male- prefixes) and the fallback policy on
``voice_unavailable``.

These tests pin the *wording* of the prompt so a future merge/cleanup
can never silently drop the rule again. Root cause of this regression
chain was exactly that — nobody re-checked the prompt after the harness
refactor (W1-W7, commits 6c4eaa98 / 77760f19).

Run with::

    python3 -m pytest src/rob_box_voice/test/unit/test_issue_1219_set_voice_rule.py
"""

from __future__ import annotations

from pathlib import Path
import re

MASTER_PROMPT = (
    Path(__file__).resolve().parents[2]
    / "prompts"
    / "master_prompt_compact.txt"
)


def _read(prompt_path: Path) -> str:
    return prompt_path.read_text(encoding="utf-8")


# ── master_prompt_compact.txt ─────────────────────────────────────────


def test_master_prompt_contains_voice_rule() -> None:
    """The RULE #VOICE anchor must be present (regression guard)."""
    content = _read(MASTER_PROMPT)
    assert "RULE #VOICE" in content, (
        "master_prompt_compact.txt lost RULE #VOICE — LLM will ignore "
        "voice-change requests on MiniMax-M3 (live, 18.08.2026). "
        "Restore the block per docs/design/LLM_VOICE_SELECTION_PROPOSAL.md "
        "(Q8: 'tell LLM HOW to change voice')."
    )


def test_voice_rule_directs_set_voice_tool() -> None:
    """The rule must explicitly mention set_voice as the way to change voice."""
    content = _read(MASTER_PROMPT)
    # Find RULE #VOICE block (single line/paragraph anchor up to next 🚨 RULE).
    match = re.search(
        r"RULE #VOICE.*?(?=🚨 \*\*RULE #)",
        content,
        re.DOTALL,
    )
    assert match, "RULE #VOICE block not delimited properly"
    block = match.group(0)
    assert "set_voice" in block, (
        "RULE #VOICE does not mention set_voice tool — LLM has no way to "
        "learn it exists"
    )
    # Must instruct LLM to call it FIRST, then speak — not the other way.
    assert re.search(r"set_voice.{0,80}FIRST", block, re.DOTALL), (
        "RULE #VOICE must direct LLM to call set_voice FIRST (otherwise "
        "the user hears the old voice before the new one kicks in)"
    )
    assert "speak_text" in block, (
        "RULE #VOICE must mention speak_text — the answer should still be "
        "spoken (just in the new voice)"
    )


def test_voice_rule_documents_gender_mapping() -> None:
    """The rule must map Russian gender words to voice id prefixes.

    Without explicit prefix hints (female-/male-) the LLM may still try
    to invent a name like 'женский' or 'alena' (Yandex-only, not in
    minimax catalogue) → voice_unavailable → same default voice played.
    """
    content = _read(MASTER_PROMPT)
    match = re.search(
        r"RULE #VOICE.*?(?=🚨 \*\*RULE #)",
        content,
        re.DOTALL,
    )
    assert match
    block = match.group(0)
    assert "female-shaonv" in block, (
        "RULE #VOICE must list at least one minimax female voice id so "
        "LLM picks a valid name when the user says 'женским голосом'"
    )
    assert "male-qn-qingse" in block, (
        "RULE #VOICE must list at least one minimax male voice id (default "
        "for fallback when user asks for the regular voice back)"
    )


def test_voice_rule_explains_voice_unavailable_fallback() -> None:
    """When set_voice returns voice_unavailable, LLM must self-recover."""
    content = _read(MASTER_PROMPT)
    match = re.search(
        r"RULE #VOICE.*?(?=🚨 \*\*RULE #)",
        content,
        re.DOTALL,
    )
    assert match
    block = match.group(0)
    assert "voice_unavailable" in block, (
        "RULE #VOICE must explain how to handle voice_unavailable "
        "response (pick another voice from [TTS] voices: ... list), "
        "otherwise LLM gives up after 1 retry"
    )
    assert "default_voice" in block, (
        "RULE #VOICE must point LLM at [TTS] default_voice: as the "
        "ultimate fallback (so user hears SOMETHING instead of silence)"
    )


def test_master_prompt_default_matches_registry() -> None:
    """The voice ids listed in RULE #VOICE must exist in tts_voice_registry.

    Catches the case where the prompt lists a stale voice id (e.g. after
    MiniMax renames their catalogue) — LLM tries it, gets
    voice_unavailable, has no fallback mapped in the prompt.
    """
    # Lazy import — registry is pure Python, no ROS dependencies.
    from rob_box_voice.tts_voice_registry import voices_for

    content = _read(MASTER_PROMPT)
    match = re.search(
        r"RULE #VOICE.*?(?=🚨 \*\*RULE #)",
        content,
        re.DOTALL,
    )
    assert match
    block = match.group(0)

    # Extract every voice id mentioned in the rule block (looks like
    # `female-shaonv`, `male-qn-qingse` — lowercase word with hyphen).
    mentioned = set(re.findall(r"\b(?:female|male)-(?:\w+-?)+\w+\b", block))

    # Registry is the source of truth (issue #1219 contract).
    registry_voices = set(voices_for("minimax"))

    missing_in_registry = mentioned - registry_voices
    assert not missing_in_registry, (
        f"RULE #VOICE lists voice ids not in tts_voice_registry.PROVIDER_VOICES"
        f"['minimax']: {sorted(missing_in_registry)}. "
        f"Either add them to the registry or remove from the prompt. "
        f"Current registry voices: {sorted(registry_voices)}."
    )
