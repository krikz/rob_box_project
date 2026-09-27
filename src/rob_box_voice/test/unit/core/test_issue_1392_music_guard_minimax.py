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
    MUSIC_RETRY_PROMPT_PREFIX,
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


def test_music_retry_prompt_names_only_reachable_engines() -> None:
    """Retry prompt must name engines the LLM can ACTUALLY call.

    Исходная посылка теста (19.08.2026) верна и не поменялась: указывать в
    CRITICAL-ретрае на незарегистрированный тул — значит гарантированно
    сжечь цикл, потому что модель физически не сможет его вызвать. Тогда
    фантомом был ``handle_music`` (фасад старого Compositor-скилла).

    Что поменялось: 20.08.2026 MiniMax Music API отключили (410 Gone), и
    ``mcp_server`` перестал регистрировать ``generate_music`` — фантомом
    стал он сам. Живьём 30.08 это стоило пользователю ответа «Я тут
    растерялся»: на «спой песню про денчика» гуард требовал generate_music,
    модель дважды отвечала прозой, бюджет ретраев кончался.

    Поэтому проверка развёрнута: раньше требовали упоминание MiniMax,
    теперь требуем его ОТСУТСТВИЕ и наличие живых Renardo-путей.
    Связь с реальной регистрацией пинится отдельно в
    ``test_generate_music_is_gone.py``.
    """
    prompt = build_music_retry_prompt("сгенерируй песню про дождь")
    assert "execute_music_code" in prompt, (
        "build_music_retry_prompt does not mention 'execute_music_code' — "
        "Renardo (bit/DJ/ambient) path must be reachable on Bug C retry."
    )
    assert "compose_music" in prompt, (
        "build_music_retry_prompt does not mention 'compose_music' — это "
        "единственный оставшийся путь для «сочини/спой про X»."
    )
    assert "generate_music" not in prompt, (
        "build_music_retry_prompt снова зовёт 'generate_music' — тул не "
        "зарегистрирован с 20.08.2026 (MiniMax 410 Gone), вызвать его "
        "нельзя, и ретрай гарантированно уйдёт в прозу."
    )
    assert "handle_music" not in prompt, (
        "build_music_retry_prompt still mentions 'handle_music' — that "
        "skill facade is not registered anymore and would make the LLM "
        "call a phantom tool ('handle_music не найден')."
    )


def test_music_retry_prompt_starts_with_shared_prefix() -> None:
    """Regression 19.08.2026 — retry prompt must start with the shared
    ``MUSIC_RETRY_PROMPT_PREFIX``.

    ``dialogue_node._run_turn`` matches ``startswith(MUSIC_RETRY_PROMPT_PREFIX)``
    to avoid resetting the Bug C retry budget on the synthetic retry. When
    the prompt was renamed (execute_music_code → handle_music) without
    updating the guard, every retry counted as a brand-new user request,
    the budget reset each iteration and Bug C looped forever re-speaking
    the refusal."""
    prompt = build_music_retry_prompt("зачитай репчик про колонку")
    assert prompt.startswith(MUSIC_RETRY_PROMPT_PREFIX), (
        "build_music_retry_prompt must start with MUSIC_RETRY_PROMPT_PREFIX; "
        "otherwise _run_turn's budget-reset guard (startswith) won't match "
        "and Bug C retries loop forever."
    )
