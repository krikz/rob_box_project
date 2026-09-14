#!/usr/bin/env python3
"""Static regression guard for kanban t_83916b3e — voice-tools prompt completeness.

Root cause (live 13.09–14.09.2026, runs 34781633844 etc.):
L: E2E Voice Test on develop HEAD (4ab3a0a) stable-fails at two steps of
night marathon act 4:

    >>> ACCEPTANCE[n401_list_voices]: ❌ expected tool calls not invoked: ['list_tts_voices']
    >>> ACCEPTANCE[n410_speed_up]:     ❌ expected tool calls not invoked: ['set_speed']

Both tools are registered (``mcp_server.py:885`` / ``mcp_server.py:844``)
and tested in ``test_issue_1765_tts_provider_tools.py`` /
``test_acceptance_gate.py`` (set_speed is 🟡 notify). The gap is in the
LLM prompt that ``dialogue_node`` ships to the LLM every turn
(``config/dialogue_node.yaml:30`` → ``master_prompt_compact.txt``):

1. **n401** (``Робот, какими голосами ты умеешь говорить?``) — RULE #VOICE
   told the LLM to ENUMERATE ``[TTS] voices: ...`` verbatim. But
   ``[TTS] voices: ...`` only carries the CURRENT voice id (1 name), not
   the full catalogue. The LLM correctly enumerated "anton" and answered
   "у меня голос антон", failing GATE-1 because it didn't call
   ``list_tts_voices()``. ``RULE #TTS-PROVIDER`` already mentioned
   ``list_tts_voices()`` (issue #1765 fix), but the routing table
   earlier in the prompt sends "какие голоса" to ``§5 RULE #VOICE`` —
   that path skipped the tool. Fix: add ``list_tts_voices()`` to the
   enumeration branch of RULE #VOICE itself.

2. **n410** (``Робот, говори быстрее / тараторь``) — ``master_prompt.txt``
   documents ``set_speed`` (line 630), but ``master_prompt_compact.txt``
   dropped it during the W1-W7 compaction (commit 84ad604, "split
   master-prompt behind a flag"). The compact prompt only lists
   ``set_volume`` and ``set_pitch`` in ``RULE #VOICE-SETTINGS``. The LLM
   never knew the tool existed for the active provider (yandex). Fix:
   add ``set_speed(action)`` to ``RULE #VOICE-SETTINGS`` alongside
   ``set_volume`` / ``set_pitch``.

These tests pin the wording so a future cleanup can't silently re-drop
either tool from the prompt again. Run::

    python3 -m pytest src/rob_box_voice/test/unit/test_t_83916b3e_voice_tools_prompt.py
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


def _voice_settings_rule_block() -> str:
    """Body of ``RULE #VOICE-SETTINGS`` block.

    Anchored on ``🚨 **RULE #VOICE-SETTINGS — `` to avoid catching the
    SSML cheat sheet or the closing paragraph that reuses the rule name.
    """
    content = _read(MASTER_PROMPT)
    match = re.search(
        r"🚨 \*\*RULE #VOICE-SETTINGS — .*?(?=🚨 \*\*RULE #|<<<SKILL-OFF>>>)",
        content,
        re.DOTALL,
    )
    assert match, (
        "RULE #VOICE-SETTINGS block not found in master_prompt_compact.txt — "
        "expected a line starting with '🚨 **RULE #VOICE-SETTINGS — '."
    )
    return match.group(0)


def _voice_rule_block() -> str:
    """Body of ``RULE #VOICE`` (the canonical set_voice enumeration block).

    Anchored on ``🚨 **RULE #VOICE — `` to avoid catching the routing
    table mention ``→ §5 RULE #VOICE``.
    """
    content = _read(MASTER_PROMPT)
    match = re.search(
        r"🚨 \*\*RULE #VOICE — .*?(?=🚨 \*\*RULE #)",
        content,
        re.DOTALL,
    )
    assert match, (
        "RULE #VOICE block not found in master_prompt_compact.txt — "
        "expected a line starting with '🚨 **RULE #VOICE — '."
    )
    return match.group(0)


# ── n410 — set_speed in RULE #VOICE-SETTINGS ────────────────────────────────


def test_voice_settings_rule_renamed_to_include_speed() -> None:
    """Header of RULE #VOICE-SETTINGS must advertise SPEED alongside PITCH/VOLUME.

    Без переименования заголовка LLM, ищущий «где правило про скорость»,
    пройдёт мимо. Live 13.09.2026: LLM не нашёл set_speed в промпте →
    n410 stable FAIL.
    """
    content = _read(MASTER_PROMPT)
    # Учитываем, что граница "PITCH/VOLUME/SPEED" могла бы стать
    # "PITCH/VOLUME/SPEED-RELATED" в будущем; фиксируем именно слово SPEED.
    match = re.search(
        r"🚨 \*\*RULE #VOICE-SETTINGS — ([^*]+)\*\*:",
        content,
    )
    assert match, "RULE #VOICE-SETTINGS header not found"
    header = match.group(1)
    assert "SPEED" in header.upper(), (
        f"RULE #VOICE-SETTINGS header missing SPEED — got {header!r}. "
        "LLM looking for 'скорость речи' rule will skip this block, "
        "n410 keeps failing in night-marathon act4 (issue t_83916b3e)."
    )


def test_voice_settings_rule_teaches_set_speed_tool() -> None:
    """RULE #VOICE-SETTINGS должен явно перечислять ``set_speed(action)``.

    Без этого LLM, получив «говори быстрее», не знает что инструмент
    существует. До фикса set_speed был только в master_prompt.txt, но
    не в master_prompt_compact.txt (default для dialogue_node.yaml).
    """
    block = _voice_settings_rule_block()
    assert "set_speed" in block, (
        "RULE #VOICE-SETTINGS must reference set_speed tool — "
        "otherwise LLM silently answers «говори быстрее» with no tool "
        "call (live 13.09–14.09.2026, n410 e2e FAIL, issue t_83916b3e)."
    )
    # И enums действия — иначе LLM может выдумать action="быстро".
    assert "faster" in block and "slower" in block and "normal" in block, (
        "RULE #VOICE-SETTINGS must enumerate the set_speed action enum "
        "{faster, slower, normal} so the LLM picks the right tool arg."
    )


def test_voice_settings_rule_speed_is_independent_of_pitch_volume() -> None:
    """Правило должно подчеркнуть: speed не трогает pitch/volume.

    LLM иначе может ошибочно подставить «говори басом и быстрее» в
    один вызов или вообще заменить set_pitch на set_speed. Это regression
    для будущих cleanup'ов промпта.
    """
    block = _voice_settings_rule_block()
    # Допустим формулировки "independent", "не трогает", "не меняет" ИЛИ
    # явное "Does NOT change set_pitch or set_volume" — все они сообщают
    # LLM, что speed и pitch/volume — разные оси.
    has_independence_hint = re.search(
        r"(independent|не\s*трогает|не\s*меняет|does\s+not\s+change\s+(set_pitch|set_volume)|не\s*влияет\s+на)",
        block,
        re.IGNORECASE,
    )
    assert has_independence_hint, (
        "RULE #VOICE-SETTINGS must call out that set_speed is independent "
        "of set_pitch/set_volume — prevents the LLM from conflating them."
    )


def test_voice_settings_rule_cheat_sheet_has_speed_column() -> None:
    """Provider cheat sheet должен включать колонку speed control.

    Без колонки правило SSML-таблицы теряет третье измерение — LLM не
    видит, что set_speed работает на всех провайдерах.
    """
    block = _voice_settings_rule_block()
    assert "speed control" in block, (
        "RULE #VOICE-SETTINGS SSML cheat sheet must include 'speed control' "
        "column with set_speed(action) for every provider."
    )
    # В таблице 7 ячеек на строку: provider, <break>, <prosody rate>,
    # <prosody pitch>, volume, pitch, speed. Упрощённо: ищем в block
    # 3 строки `| `provider` ... set_speed` (одна на провайдера).
    for provider in ("minimax", "yandex", "silero"):
        # Берём широкий non-greedy match по всей строке таблицы.
        pattern = (
            r"\|\s*`?" + provider + r"`?[^|]*(?:\|[^|]*){5}\|\s*`?set_speed\(action\)`?"
        )
        assert re.search(pattern, block), (
            f"RULE #VOICE-SETTINGS cheat sheet row for provider {provider!r} "
            "must include set_speed(action) in the speed-control column."
        )


# ── n401 — list_tts_voices() в RULE #VOICE ────────────────────────────────


def test_voice_rule_teaches_list_tts_voices_for_enumeration() -> None:
    """RULE #VOICE (НЕ только #TTS-PROVIDER) должен явно учить
    ``list_tts_voices()`` для вопроса «какие у тебя голоса?».

    Live 13.09.2026: routing-таблица в начале промпта отправляла запрос
    «какие голоса?» в §5 RULE #VOICE, где правило советовало ENUMERATE
    ``[TTS] voices: ...`` VERBATIM — но эта строка содержит только
    ТЕКУЩИЙ голос (1 имя), не весь каталог. LLM отвечал «у меня голос
    антон» без вызова tool → n401 FAIL.
    """
    block = _voice_rule_block()
    assert "list_tts_voices" in block, (
        "RULE #VOICE must reference list_tts_voices() tool — "
        "the routing table at the top of the prompt sends 'какие голоса?' "
        "to this rule (n401 e2e FAIL, issue t_83916b3e)."
    )
    assert "CALL" in block and "FIRST" in block, (
        "RULE #VOICE must instruct the LLM to CALL list_tts_voices() FIRST "
        "before speaking — not just mention the tool name."
    )


def test_voice_rule_explains_voices_context_is_only_current() -> None:
    """Правило должно объяснить, что ``[TTS] voices: ...`` ≠ каталог.

    Без явного «это ТОЛЬКО текущий голос» LLM продолжает верить, что
    строка содержит все доступные голоса, и отвечает «у меня голос
    антон» на «какие у тебя голоса?».
    """
    block = _voice_rule_block()
    # В самой правке промпта фраза: "that line only contains the CURRENT
    # voice (1 name), not the catalogue". Разрешаем основные варианты —
    # любая фраза, явно говорящая LLM, что строка содержит только current.
    pattern = (
        r"\[TTS\] voices:[^[]*?"  # до следующего блока
        r"(only\s+contains?\s+the\s+current|"
        r"only\s+the\s+current\s+voice|"
        r"contains?\s+only\s+the\s+current|"
        r"contains?\s+the\s+CURRENT\s+voice)"
    )
    assert re.search(pattern, block, re.IGNORECASE | re.DOTALL), (
        "RULE #VOICE must warn the LLM that '[TTS] voices: ...' contains "
        "ONLY the current voice, not the full catalogue — otherwise the "
        "LLM enumerates one name and skips list_tts_voices()."
    )


# ── Cross-check: ничего не сломали ───────────────────────────────────────


def test_voice_settings_rule_still_teaches_set_volume_and_set_pitch() -> None:
    """Regression guard: патч не выкинул set_volume / set_pitch.

    Если в будущем кто-то почистит промпт и уберёт set_volume /
    set_pitch — n407/n408/n411 начнут падать.
    """
    block = _voice_settings_rule_block()
    assert "set_volume" in block, "set_volume lost from RULE #VOICE-SETTINGS"
    assert "set_pitch" in block, "set_pitch lost from RULE #VOICE-SETTINGS"


def test_tts_provider_rule_still_teaches_list_tts_voices() -> None:
    """Regression guard: патч не сломал ``RULE #TTS-PROVIDER``
    (тест из test_issue_1765_tts_provider_rule.py).

    Дублируем тут, чтобы регрессия n401 не была замаскирована.
    """
    content = _read(MASTER_PROMPT)
    match = re.search(
        r"🚨 \*\*RULE #TTS-PROVIDER — .*?(?=🚨 \*\*RULE #)",
        content,
        re.DOTALL,
    )
    assert match, "RULE #TTS-PROVIDER block not found"
    block = match.group(0)
    assert "list_tts_voices" in block, (
        "RULE #TTS-PROVIDER lost list_tts_voices reference — "
        "issue #1765 regression must not happen again."
    )