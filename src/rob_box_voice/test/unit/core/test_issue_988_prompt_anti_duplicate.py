"""test_issue_988_prompt_anti_duplicate.py — the master/compositor prompts
must keep the anti-duplicate contract for ``speak_text`` (issue #988).

E2E v43/v44 regression: the LLM correctly called ``speak_text(song)`` and the
song was voiced, but the final text response re-printed the whole lyrics
(«Вот и песенка про енотика! 🦝🎶 **Повторю слова:** …») — the robot read the
song a SECOND time. The fix was two-fold:

1. Prompt (commit 841f41bc): explicit ANTI-DUPLICATE rule + few-shot
   correct/incorrect examples so the LLM returns exactly ``done`` after the
   LAST ``speak_text``.
2. Code (commits 345dc38f, ec4c3b10): ``dialogue_node`` skips auto-TTS of
   the final text when ``speak_text`` was called in the cycle; Markdown is
   stripped before chunking so a lone ``*`` is not read as «звёздочка».

These tests guard the PROMPT side: if somebody later trims the prompt and
removes the rule or the few-shot examples, CI catches it. The code side is
covered by ``test_issue_988_anti_duplicate.py`` and
``test/unit/tts/test_issue_988_markdown.py``.

Run with::

    python3 -m pytest src/rob_box_voice/test/unit/core/test_issue_988_prompt_anti_duplicate.py
"""

from __future__ import annotations

from pathlib import Path


# Resolve repo root by walking up the tree until we find the ``docker/`` and
# ``src/`` siblings. This avoids brittle parents[N] indexing that breaks when
# the test is copied into a colcon workspace (test_ws/src/rob_box_voice/...).
def _resolve_repo_root(start: Path) -> Path:
    for parent in [start, *start.parents]:
        if (parent / "docker").is_dir() and (parent / "src").is_dir():
            return parent
    # Fallback: original semantics (5 parents up from this test file).
    return start.parents[5]


_REPO_ROOT = _resolve_repo_root(Path(__file__).resolve())
MASTER_PROMPT_PATH = (
    _REPO_ROOT / "src" / "rob_box_voice" / "prompts" / "master_prompt_compact.txt"
)
COMPOSITOR_PROMPT_PATH = (
    _REPO_ROOT / "src" / "rob_box_voice" / "prompts" / "compositor_prompt.txt"
)


def test_master_prompt_has_explicit_anti_duplicate_rule() -> None:
    """Acceptance #1: the prompt explicitly forbids re-printing spoken text."""
    content = MASTER_PROMPT_PATH.read_text(encoding="utf-8")

    assert "ANTI-DUPLICATE" in content
    # The core contract: final response after LAST speak_text == "done",
    # no repeat / summarize / quote of already-spoken content.
    assert "your final response MUST be exactly `done`" in content
    assert "do NOT repeat, summarize, quote, or re-print any text" in content
    # Explicitly banned post-amble phrasings observed in E2E v43/v44.
    assert "Вот и песенка!" in content
    assert "Повторю слова:" in content


def test_master_prompt_has_few_shot_correct_and_wrong_examples() -> None:
    """Acceptance #2: few-shot examples of correct/incorrect responses."""
    content = MASTER_PROMPT_PATH.read_text(encoding="utf-8")

    # ✅ Correct: speak_text(song) → done, no re-print.
    assert "✅ SONG / RAP — lyrics via speak_text, final answer = \"done\":" in content
    assert "Текст уже озвучен через speak_text, повторять НЕ нужно." in content
    # ❌ Wrong: post-amble duplication — full example with the banned phrase.
    assert "⛔ WRONG — post-amble duplication (NEVER do this!):" in content
    assert "← ДУБЛЬ! Весь текст уже озвучен." in content
    # The music section repeats the same anti-duplicate for BACKING.
    assert "NO \"Повторю слова:\" / \"Вот и песенка!\" post-ambles!" in content


def test_compositor_prompt_has_anti_duplicate_rule() -> None:
    """The compositor (skills orchestration) also forbids lyrics echo."""
    content = COMPOSITOR_PROMPT_PATH.read_text(encoding="utf-8")

    assert "ANTI-DUPLICATE" in content
    assert "no \"Вот и песенка!\"" in content
    assert "no \"Повторю слова:\"" in content
    assert "never echo or summarize the lyrics in your final response" in content


def test_master_prompt_done_marker_rule_is_adjacent_to_speak_text() -> None:
    """The 'return done after LAST speak_text' rule sits in the same section
    as the ANTI-DUPLICATE rule, so a reader sees both together."""
    content = MASTER_PROMPT_PATH.read_text(encoding="utf-8")

    anti_dup_idx = content.index("ANTI-DUPLICATE")
    window = content[anti_dup_idx : anti_dup_idx + 2000]
    # The immediate neighbourhood must contain both the termination rule and
    # the post-amble ban.
    assert "After the LAST speak_text" in window
    assert "MUST be exactly `done`" in window
    assert "post-amble" in window
