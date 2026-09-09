#!/usr/bin/env python3
"""
test_mv01_voice_by_name_cross_provider.py — prompt-contract test для
guardrail'а Fix #1 (mv01 fail-streak fix-path, retro t_5a7b6fe3).

Контекст:
* E2E voice scenario mv01_set_voice_alena падает 5+ раундов потому что
  LLM вызывает ``set_voice(voice="alena")`` на minimax-провайдере, где
  "alena" — yandex-only голос, и SetVoiceTool возвращает
  ``voice_unavailable`` → success=False → ``voice_changed=false``.
* Фикс: RULE #VOICE в master_prompt_compact.txt должен явно говорить,
  что если юзер просит голос ПО ИМЕНИ (Алёна, Антон, Захар, ...) и
  активный провайдер этот голос НЕ содержит — НЕ вызывать
  ``set_voice(yandex-имя)`` (получишь voice_unavailable), а сказать
  «этот голос недоступен» и предложить аналогичный из текущего
  ``[TTS] voices: ...``.

Этот тест фиксирует wording RULE #VOICE как prompt-contract (как
test_issue_1219_set_voice_rule.py для исходного issue #1219). Если
кто-то сократит RULE #VOICE или удалит cross-provider guidance — тест
сломается, что и нужно.

Refs:
* t_5a7b6fe3 — ретро-карточка с этим fix-path
* PR #1636 / aa1612fe — repro-тест для set_voice(alena) на minimax
* issue #1219 — multi-voice canon (исходный RULE #VOICE)
* docs/analysis/mv01-fail-streak-fix-path-verdict-2026-08-26.md — verdict
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


def _voice_rule_block(content: str) -> str:
    """Извлечь текст RULE #VOICE (включая RULE #VOICE-MULTI, если
    соседствует — это часть того же semantic block)."""
    # RULE #VOICE-MULTI — это продолжение семантики voice management,
    # относим к тому же блоку для проверок.
    match = re.search(
        r"RULE #VOICE.*?(?=🚨 \*\*RULE #MUSIC|🚨 \*\*RULE #SEARCH|$)",
        content,
        re.DOTALL,
    )
    assert match, "RULE #VOICE block not delimited properly"
    return match.group(0)


class TestMv01VoiceByNameCrossProvider:
    """Prompt-contract guard для cross-provider voice requests."""

    def test_voice_rule_mentions_alena_anton_zahar_by_name(self) -> None:
        """RULE #VOICE должна явно упоминать «Алёна», «Антон», «Захар»
        как примеры имён, завязанных на yandex-провайдер.

        Это промпт-якорь, чтобы LLM при запросе «голосом Алены» НЕ
        автоматически вызывала ``set_voice(voice="alena")``, а
        сначала проверила активный провайдер.
        """
        content = _read(MASTER_PROMPT)
        block = _voice_rule_block(content)

        # Хотя бы одно из имён должно быть упомянуто как пример.
        assert any(
            name in block
            for name in ("алён", "Алён", "anton", "Антон", "zahar", "Захар")
        ), (
            "RULE #VOICE must mention at least one cross-provider voice name "
            "(Алёна/Антон/Захар) so LLM has explicit guidance for "
            "'говори голосом Алены' on a non-yandex provider. "
            "Current block has none of these markers."
        )

    def test_voice_rule_warns_against_calling_set_voice_with_unavailable_name(
        self,
    ) -> None:
        """RULE #VOICE должна явно говорить НЕ вызывать
        ``set_voice(unavailable_name)`` на текущем провайдере.

        Без этого LLM (MiniMax-M3) будет тупо вызывать set_voice("alena")
        на minimax и получать voice_unavailable (repro PR #1636).
        """
        content = _read(MASTER_PROMPT)
        block = _voice_rule_block(content)

        # Должна быть инструкция в духе «не вызывай set_voice с голосом,
        # которого нет в [TTS] voices:» или «сначала проверь [TTS] voices:».
        forbidden_patterns = [
            r"НЕ\s+(?:вызывай|нужно\s+вызывать)\s+set_voice",
            r"do\s+NOT\s+call\s+set_voice",
            r"не\s+передавай.*set_voice.*если.*(?:нет|недоступ)",
            r"check\s+\[TTS\]\s*voices:?\s*first",
            r"сначала.*проверь.*\[TTS\]\s*voices:?",
        ]
        assert any(
            re.search(p, block, re.IGNORECASE | re.DOTALL)
            for p in forbidden_patterns
        ), (
            "RULE #VOICE must explicitly warn against calling "
            "set_voice with a voice name not in [TTS] voices: list. "
            "Without this, LLM hallucinates 'alena' (yandex voice) "
            "even on minimax provider (mv01 fail-streak round-227..231)."
        )

    def test_voice_rule_teaches_honest_unavailable_message(self) -> None:
        """RULE #VOICE должна учить LLM честно сказать «этот голос
        недоступен» вместо попытки всё равно вызвать set_voice или
        молча fall-back'нуть.

        Текущая RULE #VOICE имеет fallback на default_voice, но не
        запрещает попытку вызвать set_voice с yandex-именем.
        """
        content = _read(MASTER_PROMPT)
        block = _voice_rule_block(content)

        # Любая формулировка «скажи что голос недоступен» / «сообщи юзеру»
        honest_patterns = [
            r"недоступен",
            r"не\s+доступен",
            r"unavailable",
            r"скажи.*(?:юзеру|робот|голос).*(?:недоступ|нет)",
        ]
        assert any(
            re.search(p, block, re.IGNORECASE | re.DOTALL)
            for p in honest_patterns
        ), (
            "RULE #VOICE must teach the LLM to honestly report "
            "'voice unavailable' when the user asks for a voice not in "
            "the current [TTS] voices: list."
        )

    def test_voice_rule_suggests_alternative_from_current_list(self) -> None:
        """RULE #VOICE должна говорить «предложи аналогичный голос из
        текущего [TTS] voices:» — иначе LLM после «недоступен» просто
        замолчит или скажет default без выбора.
        """
        content = _read(MASTER_PROMPT)
        block = _voice_rule_block(content)

        # Должна быть формулировка «выбери/предложи из текущего списка»
        # или аналог.
        suggest_patterns = [
            r"предлож[ии].*\[TTS\]\s*voices:",
            r"выбер[ии].*\[TTS\]\s*voices:",
            r"из\s+(?:текущего|списка).*\[TTS\]\s*voices:",
            r"pick.*(?:from|in).*\[TTS\]\s*voices:",
            r"analog.*from.*\[TTS\]\s*voices:",
            r"filter.*by\s+gender.*\[TTS\]\s*voices:",
        ]
        assert any(
            re.search(p, block, re.IGNORECASE | re.DOTALL)
            for p in suggest_patterns
        ), (
            "RULE #VOICE must instruct the LLM to propose an alternative "
            "voice from the current [TTS] voices: list (e.g. by gender) "
            "when the requested name is unavailable."
        )
