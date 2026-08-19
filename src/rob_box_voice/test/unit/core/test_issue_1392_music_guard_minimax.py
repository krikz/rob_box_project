#!/usr/bin/env python3
"""Regression guard for issue #1392 — music_guard keyword filter must
match AI-generation triggers.

Live evidence 18.08.2026 17:05 MSK: юзер сказал «робот сгенерируй
песню про дождь», dialogue_node пошёл в LLM-Compositor, но
``user_wants_music`` returned False (``"MUSIC_GUARD_KEYWORDS →
wants_music=False (возможно, нужно добавить keyword в
MUSIC_GUARD_KEYWORDS)"``). Без этого guard не запускается Bug C
retry — LLM-M3 уходит в голосовой текст «🎤 Меняю роль!» вместо
вызова handle_music.
"""

from __future__ import annotations

from rob_box_voice.core.dialogue_guards import (
    MUSIC_GUARD_KEYWORDS,
    build_music_retry_prompt,
    user_wants_music,
)


# ── MUSIC_GUARD_KEYWORDS must include AI-generation triggers ────────────────


def test_music_guard_has_sgeneriruy_pesnyu() -> None:
    """«сгенерируй песню…» — primary AI-generation trigger."""
    assert "сгенерируй песн" in MUSIC_GUARD_KEYWORDS, (
        "MUSIC_GUARD_KEYWORDS has no 'сгенерируй песн' trigger — юзер "
        "'сгенерируй песню про дождь' пройдёт без guard, LLM не вызовет "
        "handle_music и Bug C retry не сработает."
    )


def test_music_guard_has_generation_variants() -> None:
    """Other variants юзера: сочини, сделай музыку, генератор, генерация."""
    required = (
        "сочини песн",
        "сочини музык",
        "сделай музык",
        "генератор музык",
        "генерация музык",
        "сгенерируй мелоди",
        "сгенерируй трек",
        "сгенерируй музык",
    )
    missing = [kw for kw in required if kw not in MUSIC_GUARD_KEYWORDS]
    assert not missing, (
        f"MUSIC_GUARD_KEYWORDS is missing AI-generation triggers: {missing}. "
        f"Live regression 18.08: 'сгенерируй песню про дождь' bypassed "
        f"music guard entirely."
    )


def test_user_wants_music_sgeneriruy_pesnyu() -> None:
    """End-to-end: user_wants_music('сгенерируй песню про дождь') → True."""
    assert user_wants_music("сгенерируй песню про дождь") is True
    assert user_wants_music("сгенерируй мелодию для дождя") is True
    assert user_wants_music("сочини песню про кота") is True
    assert user_wants_music("сделай музыку под настроение") is True
    assert user_wants_music("я хочу сгенерируем музыку кайфовую") is True


# ── build_music_retry_prompt must point at handle_music, not execute_music_code alone


def test_music_retry_prompt_mentions_handle_music() -> None:
    """Retry prompt must NOT just say 'execute_music_code' — LLM must
    know that handle_music is the correct tool and sub-agent picks the
    engine (Renardo OR MiniMax)."""
    prompt = build_music_retry_prompt("сгенерируй песню про дождь")
    assert "handle_music" in prompt, (
        "build_music_retry_prompt does not mention 'handle_music' — "
        "Bug C retry asks for 'execute_music_code' alone, which maps "
        "to Renardo and ignores AI-generation tools. Live regression "
        "18.08: LLM after retry was still empty-handed (tools=[])."
    )
    assert "generate_music" in prompt, (
        "build_music_retry_prompt does not mention 'generate_music' — "
        "LLM must be told about both engines (Renardo AND MiniMax)"
        " when retrying on AI-generation requests."
    )
