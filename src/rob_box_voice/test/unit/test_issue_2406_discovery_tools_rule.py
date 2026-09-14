"""Static regression guard for issue #2406 — discovery-tool enforcement.

Issue #2406 (umbrella) — LLM отвечает verbal-only на discovery/inquiry-шаги
вместо вызова read-only tool. Воспроизводилось 3 раза подряд на develop HEAD
4ab3a0a (runs 363/364/365, fail-streak продолжался до ручного фикса):

- Run 363 / act2 / n201_sasha_intro_long — missing ``register_speaker`` в
  expected-списке (юзер назвал имя «Саша» в intro).
- Run 364 / act3 / n301_wake_open (n313_silence_restored) — missing
  ``get_music_state`` (тот же баг, что и в issue #2347).
- Run 365 / act4 / n401_list_voices — missing ``list_tts_voices`` (юзер
  спрашивал «какие голоса?»).

Все три — discovery-шаги («расскажи / перечисли / что играет / как тебя
зовут»), и у всех трёх LLM пропустила РОВНО ОДИН tool — первый
discovery-тул в expected-списке. Остальные (``set_voice``, ``set_volume``,
``set_speed``, ``stop_music``) были вызваны штатно.

Fix: новый ``RULE #DISCOVERY-TOOLS`` в ``master_prompt_compact.txt`` —
короткий enforcement-блок сразу после ``RULE #LANG``, который явно
требует tool-call **первым** для каждого из 3 discovery-тулов и запрещает
verbal-only ответ.

Run with::

    python3 -m pytest src/rob_box_voice/test/unit/test_issue_2406_discovery_tools_rule.py
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


def _discovery_rule_block() -> str:
    """Body of ``RULE #DISCOVERY-TOOLS`` block.

    Анкорим на ``🚨 **RULE #DISCOVERY-TOOLS — `` (с эмодзи), чтобы не
    спутать с другими вхождениями. Заканчиваем на следующем ``🚨 **RULE #``.
    """
    content = _read(MASTER_PROMPT)
    match = re.search(
        r"🚨 \*\*RULE #DISCOVERY-TOOLS — .*?(?=🚨 \*\*RULE #)",
        content,
        re.DOTALL,
    )
    assert match, (
        "RULE #DISCOVERY-TOOLS block not found in master_prompt_compact.txt — "
        "expected a line starting with '🚨 **RULE #DISCOVERY-TOOLS — ' followed "
        "by another '🚨 **RULE #'. Issue #2406 fix depends on this enforcement "
        "block (live 13.09.2026, 3 fail-runs подряд 363/364/365)."
    )
    return match.group(0)


# ── master_prompt_compact.txt ─────────────────────────────────────────


def test_master_prompt_contains_discovery_tools_rule() -> None:
    """The RULE #DISCOVERY-TOOLS anchor must be present (regression guard)."""
    content = _read(MASTER_PROMPT)
    assert "🚨 **RULE #DISCOVERY-TOOLS" in content, (
        "master_prompt_compact.txt lost RULE #DISCOVERY-TOOLS — LLM will "
        "answer discovery questions verbally (issue #2406 bug). Restore "
        "the block — see test_issue_2406_discovery_tools_rule.py for context."
    )


def test_discovery_rule_requires_list_tts_voices_first() -> None:
    """«какие у тебя голоса?» → CALL list_tts_voices() ДО speak_text.

    Run 365 / n401_list_voices — LLM отвечал «у меня только anton» потому
    что в [TTS] voices: в <tts_context> только текущий голос. Без явного
    enforcement rule это будет повторяться.
    """
    block = _discovery_rule_block()
    assert "list_tts_voices" in block, (
        "RULE #DISCOVERY-TOOLS must mention list_tts_voices — otherwise LLM "
        "won't call it on «какие голоса?» queries (issue #2406 run 365)"
    )
    assert "BEFORE" in block, (
        "RULE #DISCOVERY-TOOLS must use BEFORE — emphasize that the tool "
        "call must precede any speak_text, not follow it"
    )


def test_discovery_rule_requires_get_music_state_first() -> None:
    """«что играет?» → CALL get_music_state() ДО speak_text.

    Run 364 / n313_silence_restored — LLM опирался на stale <music_state>
    XML-тег. Правило должно явно требовать tool-call, иначе e2e-гейт
    падает на missing tool-call в трейсе.
    """
    block = _discovery_rule_block()
    assert "get_music_state" in block, (
        "RULE #DISCOVERY-TOOLS must mention get_music_state — otherwise "
        "LLM answers «Сейчас тишина» from stale <music_state> tag "
        "(issue #2406 run 364, same bug as #2347)"
    )


def test_discovery_rule_requires_register_speaker_first() -> None:
    """«зовут Саша/Петя/...» / intro → CALL register_speaker(name=) ДО speak.

    Run 363 / n201_sasha_intro_long — юзер назвал имя, LLM звал set_voice/
    set_volume/set_speed, но пропустил register_speaker. Правило должно
    явно требовать tool-call первым.
    """
    block = _discovery_rule_block()
    assert "register_speaker" in block, (
        "RULE #DISCOVERY-TOOLS must mention register_speaker — otherwise "
        "LLM skips it on intro scenarios like «давай знакомиться» "
        "(issue #2406 run 363)"
    )


def test_discovery_rule_bans_verbal_only_answers() -> None:
    """Запрет verbal-only ответа на discovery-вопросы.

    Без явного BANNED примера LLM продолжает отвечать текстом «Сейчас
    тишина» / «У меня только anton» / «Приятно познакомиться, Саша!».
    """
    block = _discovery_rule_block()
    assert "BANNED" in block and "verbal-only" in block, (
        "RULE #DISCOVERY-TOOLS must explicitly BANNED verbal-only answers "
        "on discovery questions — that's the exact bug from issue #2406"
    )


def test_discovery_rule_references_issue_2406_run_numbers() -> None:
    """Правило ссылается на issue #2406 и конкретные run-номера для трассировки.

    Это помогает будущим ревьюерам / cleaning-агентам связать правило с
    исходным bug-report и не удалить его «как избыточное».
    """
    block = _discovery_rule_block()
    assert "#2406" in block, (
        "RULE #DISCOVERY-TOOLS must reference issue #2406 — otherwise "
        "future cleanup agents may treat it as orphaned"
    )
    for run in ("363", "364", "365"):
        assert run in block, (
            f"RULE #DISCOVERY-TOOLS must reference run {run} — these are "
            f"the 3 fail-runs that motivated issue #2406"
        )


def test_discovery_rule_placed_after_lang_rule() -> None:
    """RULE #DISCOVERY-TOOLS должен идти СРАЗУ после RULE #LANG.

    Существующие RULE #VOICE / #TTS-PROVIDER / #MUSIC-STATE / #SYSCTX
    покрывают каждый tool в отдельности, но LLM их «забывает» —
    enforcement-блок в начале §1 максимизирует шанс быть прочитанным.
    """
    content = _read(MASTER_PROMPT)
    lang_match = re.search(r"🚨 \*\*RULE #LANG — SPEAK RUSSIAN ALWAYS\*\*:", content)
    assert lang_match, "RULE #LANG anchor missing — cannot check ordering"
    discovery_match = re.search(
        r"🚨 \*\*RULE #DISCOVERY-TOOLS — FIRST TOOL CALL", content
    )
    assert discovery_match, "RULE #DISCOVERY-TOOLS anchor missing"
    assert discovery_match.start() > lang_match.start(), (
        "RULE #DISCOVERY-TOOLS must come AFTER RULE #LANG — it's the "
        "second rule in §1 to maximize LLM attention (issue #2406)"
    )