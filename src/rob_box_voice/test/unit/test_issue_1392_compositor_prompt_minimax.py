#!/usr/bin/env python3
"""Static regression guard for issue #1392 (MiniMax-music visibility at Compositor).

Live evidence 18.08.2026 on ``z-{e2e}/test-round-143``: юзер сказал
«робокс сгенерируй настоящую музыку не синтезатором а сгенерирую ее
специальная функция которая у тебя есть генерит музыку», LLM-M3
(Compositor) ответил: «Это невозможно. У меня нет такой функции. Я
умею играть мелодии только через синтезатор Renardo».

Root cause: ``compositor_prompt.txt`` (the prompt the **outer** LLM
sees) had no mention of ``MiniMax`` / ``generate_music`` / AI-generated
tracks. The ``MusicSkill`` sub-agent had a 69 KB prompt that did mention
``generate_music``, but Compositor never told the outer LLM that
``handle_music`` could call it — so it invented "I only have Renardo".

These tests pin the wording so a future prompt cleanup can never silently
re-introduce that gap. Related: #1358 (PR #1361), #1371 (gen_* split),
#1373 (return-to-work), #1384 (e2e harness scenarios).
"""

from __future__ import annotations

from pathlib import Path

COMPOSITOR_PROMPT = (
    Path(__file__).resolve().parents[2]
    / "prompts"
    / "compositor_prompt.txt"
)


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


# ── Compositor prompt must mention MiniMax / generate_music ────────────────


def test_compositor_prompt_mentions_minimax_music() -> None:
    """The outer LLM must know that MiniMax powers music generation."""
    content = _read(COMPOSITOR_PROMPT)
    assert "MiniMax" in content, (
        "compositor_prompt.txt has no mention of MiniMax — the outer LLM "
        "will answer 'I only have Renardo' even after PR #1392 wired "
        "generate_music through MusicSkill."
    )


def test_compositor_prompt_documents_generate_music_route() -> None:
    """The rule block must explicitly route to generate_music."""
    content = _read(COMPOSITOR_PROMPT)
    assert "generate_music" in content, (
        "compositor_prompt.txt has no `generate_music` reference — "
        "Compositor cannot delegate 'сгенерируй песню...' to the "
        "generate_music tool that lives inside MusicSkill."
    )
    assert "gen_play_from_library" in content or "gen_search_library" in content, (
        "compositor_prompt.txt must show how to play back a saved track "
        "via gen_* library tools. Without it the LLM never loops the "
        "user back to a previously generated track."
    )


def test_compositor_prompt_does_not_invent_unavailable_function() -> None:
    """Pin the anti-fabrication rule for the music domain."""
    content = _read(COMPOSITOR_PROMPT)
    # The block pinned in #1392 fixes the "I have no such function" lie.
    assert "Не выдумывай" in content or "не выдумывай" in content, (
        "compositor_prompt.txt has no anti-fabrication line about the "
        "music generation tools — LLM-M3 may revert to inventing "
        "'у меня нет такой функции' on edge phrases."
    )


def test_compositor_prompt_pre_call_warning_rule() -> None:
    """generate_music takes 40-160s. Compositor must warn before calling."""
    content = _read(COMPOSITOR_PROMPT)
    # Either an inline warning rule OR the speed hint is referenced.
    assert ("40" in content and "160" in content) or "минут" in content.lower(), (
        "Compositor must know generate_music is slow so it pre-warns the "
        "user. If the latency hint is removed, users will think the robot "
        "froze during the 1-2 minute generation wait."
    )


def test_compositor_prompt_handles_handle_music_sk_dual_path() -> None:
    """`handle_music` MUST be marked as capable of BOTH engines."""
    content = _read(COMPOSITOR_PROMPT)
    # Find the AI-GENERATED block anchor (added in #1392).
    assert "AI-GENERATED" in content or "GENERATED MUSIC" in content, (
        "compositor_prompt.txt lost the GENERATED MUSIC rule block that "
        "tells LLM there are two engines behind `handle_music` (Renardo + "
        "MiniMax). Without this block the LLM collapses to 'only Renardo'."
    )
