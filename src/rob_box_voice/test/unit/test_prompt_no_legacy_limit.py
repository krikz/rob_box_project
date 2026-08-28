"""Static regression guard for issue #1377.

Issue #1377: the LLM prompts (master_prompt_compact.txt, compositor_prompt.txt)
told the model "no more than 150 characters" per speak_text call. This is a
legacy hint that pre-dates the runtime contract:

* Runtime chunker (`src/rob_box_voice/rob_box_voice/tts_chunking.py`)
  - ``CHUNK_LIMITS["yandex_grpc_v3"] = 250`` (real API hard limit, ~250 chars
    SSML; >250 → INVALID_ARGUMENT).
  - All other providers are at least as generous (silero_v5=800, MiniMax=5000).
* Runtime LLM budget (`src/rob_box_mcp_tools/.../tools/dialogue.py`)
  - ``_MAX_CHUNK_CHARS = 200`` is the hard cap the agentic LLM is being fed
    through the MCP tool description.

LLM was reading the prompt-internal "150" and micro-splitting answers into
fragments that re-triggered TTS batching unnecessarily. The prompt must
agree with the runtime — we use **200** as the single source of truth
(matches ``_MAX_CHUNK_CHARS`` and stays well below the Yandex 250 ceiling).

These tests pin the wording so a future merge can never silently roll the
prompts back to the old "150 characters" model again.

Run with::

    python3 -m pytest src/rob_box_voice/test/unit/test_prompt_no_legacy_limit.py
"""

from __future__ import annotations

import re
from pathlib import Path


MASTER_PROMPT = (
    Path(__file__).resolve().parents[2]
    / "prompts"
    / "master_prompt_compact.txt"
)

COMPOSITOR_PROMPT = (
    Path(__file__).resolve().parents[2]
    / "prompts"
    / "compositor_prompt.txt"
)


# Pattern of the legacy "150 characters" hint that previously confused the LLM.
# We accept "150ms" (SSML break time) and other 150 occurrences — only the
# character-limit phrasing is the regression we're guarding.
_LEGACY_150_CHAR_LIMIT = re.compile(
    r"150\s*(characters?|символов?|знаков)",
    flags=re.IGNORECASE,
)


def _read(prompt_path: Path) -> str:
    return prompt_path.read_text(encoding="utf-8")


def _legacy_150_hits(content: str) -> list[str]:
    """Return the matched substrings (for diagnostics) so a future failure
    is debuggable without re-reading the file."""
    return _LEGACY_150_CHAR_LIMIT.findall(content)


# ── master_prompt_compact.txt ─────────────────────────────────────────


def test_master_prompt_has_no_legacy_150_character_limit() -> None:
    """The pre-#1377 wording "150 characters" must be gone from the master prompt."""
    content = _read(MASTER_PROMPT)
    hits = _legacy_150_hits(content)
    assert hits == [], (
        f"master_prompt_compact.txt still contains legacy '150 characters' "
        f"hint(s): {hits!r}. Runtime contract is _MAX_CHUNK_CHARS=200 "
        f"(tts_chunking.py CHUNK_LIMITS['yandex_grpc_v3']=250). "
        f"Use 200 to match the runtime cap."
    )


def test_master_prompt_documents_200_character_limit() -> None:
    """The prompt must explicitly tell the LLM to use 200 chars as the limit."""
    content = _read(MASTER_PROMPT)
    # The exact wording introduced by the #1377 fix (two places: BREVITY rule
    # and the speak_text tool-card section).
    assert "NO MORE than 200 characters" in content, (
        "master_prompt_compact.txt missing the BREVITY rule rewrite "
        "'NO MORE than 200 characters' (see issue #1377)."
    )
    assert "Max **200 characters** per speak_text" in content, (
        "master_prompt_compact.txt missing the speak_text tool-card "
        "'Max **200 characters** per speak_text' (see issue #1377)."
    )


# ── compositor_prompt.txt ─────────────────────────────────────────────


def test_compositor_prompt_has_no_legacy_150_character_limit() -> None:
    """Compositor prompt must not echo the legacy 150 hint."""
    content = _read(COMPOSITOR_PROMPT)
    hits = _legacy_150_hits(content)
    assert hits == [], (
        f"compositor_prompt.txt still contains legacy '150 characters' "
        f"hint(s): {hits!r}. Use 200 to match _MAX_CHUNK_CHARS."
    )


def test_compositor_prompt_documents_200_character_limit() -> None:
    """Compositor prompt must use 200 as the single source of truth."""
    content = _read(COMPOSITOR_PROMPT)
    assert "no more than 200 characters" in content, (
        "compositor_prompt.txt missing the BREVITY rule rewrite "
        "'no more than 200 characters' (see issue #1377)."
    )
