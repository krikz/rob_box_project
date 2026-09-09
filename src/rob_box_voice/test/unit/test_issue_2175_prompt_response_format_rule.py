#!/usr/bin/env python3
"""Static regression guard for issue #2175 — master_prompt RULE #RESPONSE-FORMAT.

Issue #2175 (live 08.09): MiniMax-M3 regurgitates ``<system>...</system>``
template after ``set_voice`` switch + multi-voice user_input + new DJ-skill
context. The raw template leak (e.g. «получатель ответа забыл указать
антропоморфные атрибуты») was voiced through Yandex→MiniMax fallback.

Three layers of defense landed in this fix:

1. Prompt-level (this test) — explicit ``RULE #RESPONSE-FORMAT`` block in
   ``master_prompt_compact.txt`` telling the model to never copy the
   contents of internal XML blocks into its reply.
2. dialogue_node guard (issue #2175 acceptance #1) — regex detector +
   one-shot CRITICAL retry.
3. tts_node refusal (issue #2175 acceptance #2) — defence-in-depth at the
   synthesis chokepoint.

These tests pin the *wording* of the prompt rule so a future merge/cleanup
can never silently drop it again. Same regression-prevention pattern as
``test_issue_1219_set_voice_rule.py`` and ``test_issue_1765_tts_provider_rule.py``.

Run with::

    PYTHONPATH=src/rob_box_voice:src/rob_box_llm:src/rob_box_core:src/rob_box_harness \\
        python3 -m pytest src/rob_box_voice/test/unit/test_issue_2175_prompt_response_format_rule.py
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


def _response_format_rule_block() -> str:
    """Body of ``RULE #RESPONSE-FORMAT`` block, anchored on its header.

    Тест test_issue_1219_set_voice_rule.py показал что bare ``RULE #VOICE``
    ловит routing-таблицу вместо настоящего блока. Здесь anchor
    ``🚨 **RULE #RESPONSE-FORMAT — `` (с эмодзи) чтобы не путать с
    вхождениями в QUICK ROUTING.
    """
    content = _read(MASTER_PROMPT)
    match = re.search(
        r"🚨 \*\*RULE #RESPONSE-FORMAT — .*?(?=🚨 \*\*RULE #)",
        content,
        re.DOTALL,
    )
    assert match, (
        "RULE #RESPONSE-FORMAT block not found in master_prompt_compact.txt — "
        "expected a line starting with '🚨 **RULE #RESPONSE-FORMAT — ' "
        "followed by another '🚨 **RULE #'"
    )
    return match.group(0)


# ── master_prompt_compact.txt ─────────────────────────────────────────


def test_master_prompt_contains_response_format_rule() -> None:
    """Regression guard — rule must be present."""
    content = _read(MASTER_PROMPT)
    assert "🚨 **RULE #RESPONSE-FORMAT" in content, (
        "master_prompt_compact.txt lost RULE #RESPONSE-FORMAT — MiniMax-M3 may "
        "regurgitate <system>...</system> template after set_voice switch. "
        "Restore the block per issue #2175."
    )


def test_response_format_rule_mentions_forbidden_xml_tags() -> None:
    """Правило должно явно называть ЗАПРЕЩЁННЫЕ XML-теги, чтобы LLM
    знала ЧТО именно нельзя копировать в ответ."""
    block = _response_format_rule_block()
    required_tags = [
        "<system>",  # канонический пример из живого лога 08.09
        "<system_context>",  # второй по частоте
        "<hardware>",  # часто появляется в контексте
    ]
    for tag in required_tags:
        assert tag in block, (
            f"RULE #RESPONSE-FORMAT must explicitly name forbidden tag "
            f"{tag!r} so LLM knows what NOT to regurgitate"
        )


def test_response_format_rule_explains_clarify_question_alternative() -> None:
    """Правило должно предлагать альтернативу — задать уточняющий вопрос
    обычным языком вместо regurgitates (см. acceptance карточки #4)."""
    block = _response_format_rule_block()
    # Должно быть явное упоминание «уточняющ*» или «clarif*»
    # (в английском варианте) или «Уточни» в русском.
    has_clarify = bool(
        re.search(r"уточн", block, re.IGNORECASE)
        or re.search(r"clarif", block, re.IGNORECASE)
    )
    assert has_clarify, (
        "RULE #RESPONSE-FORMAT must offer the alternative of asking a "
        "clarifying question (instead of regurgitating the template). "
        "Otherwise LLM has no idea what to do when it doesn't know the answer."
    )


def test_response_format_rule_warns_about_live_bug() -> None:
    """Правило должно ссылаться на issue #2175 — оператор/ревьюер
    должны видеть в коде, ЗАЧЕМ оно появилось."""
    block = _response_format_rule_block()
    assert "#2175" in block or "2175" in block, (
        "RULE #RESPONSE-FORMAT must reference issue #2175 so a future "
        "cleanup knows the root cause and won't drop the rule again"
    )


def test_response_format_rule_in_quick_routing() -> None:
    """Правило должно быть перечислено в QUICK ROUTING-таблице в начале
    промпта (по аналогии с RULE #LANG / #SYSCTX / etc.)."""
    content = _read(MASTER_PROMPT)
    # В начале есть строка вида ``§1 CORE INVARIANTS (...#RESPONSE-FORMAT...)``
    assert "#RESPONSE-FORMAT" in content[:1500], (
        "RULE #RESPONSE-FORMAT должен быть в QUICK ROUTING (первые 1500 "
        "символов), чтобы LLM видел правило сразу"
    )


def test_response_format_rule_is_short_and_actionable() -> None:
    """Правило не должно быть раздутым — иначе LLM его пропустит.
    1500 chars — мягкий лимит (для сравнения, RULE #VOICE ~1100 chars)."""
    block = _response_format_rule_block()
    assert len(block) < 1500, (
        f"RULE #RESPONSE-FORMAT block is {len(block)} chars — слишком длинное, "
        "LLM может его пропустить. Сократи до <1500 chars."
    )
