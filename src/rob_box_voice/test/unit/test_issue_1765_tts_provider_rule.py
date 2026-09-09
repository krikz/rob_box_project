"""Static regression guard for issue #1765 — cross-provider TTS switching.

Issue #1765 — LLM никогда не переключал активного TTS-провайдера:
* На фразу «Робот, переключи голос на Яндекс Артём» LLM отвечала «голоса
  «Артём» в списке нет» (хотя Артём — yandex-голос, а активный был minimax).
* Юзер не мог попросить «давай говорить Яндексом» — не было tool'а для
  смены провайдера.

Fix: новый RULE #TTS-PROVIDER в master_prompt_compact.txt с инструкциями
по использованию ``set_tts_provider(provider='yandex')`` + ``set_voice(voice,
provider=...)`` для cross-provider кейса.

Эти тесты пинят wording правила, чтобы будущий merge/cleanup не выкинул
его снова. Подробнее — test_issue_1219_set_voice_rule.py (этот файл —
прямое расширение для #1765).

Run with::

    python3 -m pytest src/rob_box_voice/test/unit/test_issue_1765_tts_provider_rule.py
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


def _tts_provider_rule_block() -> str:
    """Body of ``RULE #TTS-PROVIDER`` block.

    Анкорим на ``🚨 **RULE #TTS-PROVIDER — `` (с эмодзи), чтобы не
    спутать с вхождениями ``TTS-PROVIDER`` в routing-таблице (issue с
    аналогичным bug в test_issue_1219_set_voice_rule.py, где bare
    ``RULE #VOICE`` ловил QUICK ROUTING вместо настоящего блока).
    """
    content = _read(MASTER_PROMPT)
    match = re.search(
        r"🚨 \*\*RULE #TTS-PROVIDER — .*?(?=🚨 \*\*RULE #)",
        content,
        re.DOTALL,
    )
    assert match, (
        "RULE #TTS-PROVIDER block not found in master_prompt_compact.txt — "
        "expected a line starting with '🚨 **RULE #TTS-PROVIDER — ' followed "
        "by another '🚨 **RULE #'"
    )
    return match.group(0)


# ── master_prompt_compact.txt ─────────────────────────────────────────


def test_master_prompt_contains_tts_provider_rule() -> None:
    """The RULE #TTS-PROVIDER anchor must be present (regression guard)."""
    content = _read(MASTER_PROMPT)
    assert "🚨 **RULE #TTS-PROVIDER" in content, (
        "master_prompt_compact.txt lost RULE #TTS-PROVIDER — LLM won't know "
        "how to switch TTS provider on user requests like «давай говорить "
        "Яндексом» (live 31.08.2026). Restore the block per issue #1765."
    )


def test_tts_provider_rule_mentions_set_tts_provider_tool() -> None:
    """Правило должно явно ссылаться на set_tts_provider как способ смены."""
    block = _tts_provider_rule_block()
    assert "set_tts_provider" in block, (
        "RULE #TTS-PROVIDER must mention set_tts_provider tool — "
        "otherwise LLM has no way to learn it exists"
    )


def test_tts_provider_rule_mentions_set_voice_with_provider() -> None:
    """Cross-provider set_voice(voice, provider=...) должен быть упомянут."""
    block = _tts_provider_rule_block()
    # Правило ссылается на set_voice с параметром provider.
    assert "set_voice" in block and "provider" in block, (
        "RULE #TTS-PROVIDER must reference set_voice(voice, provider=...) "
        "for cross-provider voice switch (RULE #VOICE-CROSS-PROVIDER)"
    )


def test_tts_provider_rule_enumerates_supported_providers() -> None:
    """Список допустимых провайдеров (yandex/minimax/silero) должен быть в правиле."""
    block = _tts_provider_rule_block()
    for p in ("yandex", "minimax", "silero"):
        assert p in block, (
            f"RULE #TTS-PROVIDER must list {p!r} in supported providers "
            f"enum — otherwise LLM won't know which providers are allowed"
        )


def test_tts_provider_rule_distinguishes_set_voice_vs_set_tts_provider() -> None:
    """set_tts_provider — для смены провайдера, set_voice — для голоса.

    Без явного разделения LLM будет путать: «поменяй голос» →
    set_tts_provider вместо set_voice, или наоборот.
    """
    block = _tts_provider_rule_block()
    assert "set_voice" in block, (
        "RULE #TTS-PROVIDER must reference set_voice to clarify "
        "the distinction between provider switch and voice change"
    )


def test_tts_provider_rule_teaches_list_tts_voices() -> None:
    """Для вопросов «какие голоса на Яндексе?» LLM должна звать list_tts_voices."""
    block = _tts_provider_rule_block()
    assert "list_tts_voices" in block, (
        "RULE #TTS-PROVIDER must reference list_tts_voices tool — "
        "otherwise LLM hallucinates voice names when user asks "
        "«какие голоса на yandex?» (live 20.08.2026: answered 'татьяна')"
    )


def test_cross_provider_rule_uses_set_voice_with_provider_param() -> None:
    """RULE #VOICE-CROSS-PROVIDER должен явно учить set_voice(voice, provider=...).

    Без этого LLM не поймёт, что для кросс-провайдерного кейса нужно
    передать ОБА параметра одним вызовом, а не делать два вызова
    (set_tts_provider → set_voice) с риском race condition.
    """
    content = _read(MASTER_PROMPT)
    cross_block = re.search(
        r"🚨 \*\*RULE #VOICE-CROSS-PROVIDER — .*?(?=🚨 \*\*RULE #)",
        content,
        re.DOTALL,
    )
    assert cross_block, "RULE #VOICE-CROSS-PROVIDER block not found"
    block = cross_block.group(0)
    # Должен явно требовать set_voice с параметром provider.
    # Формат в правиле: ``set_voice(voice="<имя>", provider="<имя>")`` —
    # regex ищет set_voice(...) c упоминанием provider= ВНУТРИ скобок.
    assert re.search(r"set_voice\([^)]*provider=", block), (
        "RULE #VOICE-CROSS-PROVIDER must teach set_voice(voice=..., "
        "provider=...) one-shot pattern — issue #1765 closes the live "
        "bug where LLM tried two separate tool calls."
    )
