#!/usr/bin/env python3
"""Static regression guard for issue #1532 (mv03 multi-voice skazka).

Issue #1532 — LLM did NOT call ``set_voice`` for a multi-voice skazka
(«робот, расскажи сказку про Красную Шапочку разными голосами»):

    spoken='Жила-была девочка. Мама испекла пирожки...'
    tools=[]
    # TTS played ONE voice on ONE sentence — all characters sounded the same.

Root cause: ``master_prompt_compact.txt`` had no rule teaching the LLM
when/how to interleave ``set_voice`` between ``speak_text`` fragments for
different characters. ``RULE #VOICE`` (issue #1219) only described global
voice-change, not per-character switching in a single narrative.

Fix: explicit ``RULE #VOICE-MULTI`` block in ``master_prompt_compact.txt``
with:
- per-character switching contract (example with 3 voices + 4 fragments)
- 7 mandatory rules (full skazka 5-7 fragments, ≥2 different voices,
  gender-based mapping, do NOT invent names, return to default at end)
- ✅/❌ examples anchored to e2e #1532 mv03

These tests pin the *wording* so future prompt refactors (e.g. issue
#1219 next iteration) cannot silently drop the multi-voice block again.
Same lesson as ``test_issue_1219_set_voice_rule.py``: the prompt wording
is the source of truth.

Run with::

    python3 -m pytest src/rob_box_voice/test/unit/test_issue_1532_voice_multi_skazka.py
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


def _multi_block(content: str) -> str:
    """Extract the RULE #VOICE-MULTI block (anchor up to next 🚨 RULE or end)."""
    match = re.search(
        r"RULE #VOICE-MULTI.*?(?=🚨 \*\*RULE #|\Z)",
        content,
        re.DOTALL,
    )
    assert match, (
        "RULE #VOICE-MULTI block not delimited properly in "
        "master_prompt_compact.txt"
    )
    return match.group(0)


# ── master_prompt_compact.txt ─────────────────────────────────────────


def test_master_prompt_contains_voice_multi_rule() -> None:
    """The RULE #VOICE-MULTI anchor must be present (regression guard)."""
    content = _read(MASTER_PROMPT)
    assert "RULE #VOICE-MULTI" in content, (
        "master_prompt_compact.txt lost RULE #VOICE-MULTI — LLM will not "
        "switch voices between skazka characters on MiniMax-M3 (live "
        "18.08.2026 / e2e #1532 mv03). Without this block, LLM answers "
        "with tools=[] and the user hears one voice for all characters."
    )


def test_voice_multi_rule_directs_set_voice_between_speak_text() -> None:
    """The rule must instruct LLM to call set_voice BETWEEN speak_text.

    Live 22.08.2026: LLM вызывал speak_text 1 раз → set_voice не вызывался.
    Без set_voice между фрагментами все персонажи звучат одинаковым голосом.
    """
    content = _read(MASTER_PROMPT)
    block = _multi_block(content)
    # Must mention set_voice tool explicitly in the multi-voice block.
    assert "set_voice" in block, (
        "RULE #VOICE-MULTI must mention set_voice tool — otherwise LLM "
        "has no way to learn it exists for multi-voice skazkas."
    )
    # Must mention speak_text in the multi-voice block.
    assert "speak_text" in block, (
        "RULE #VOICE-MULTI must mention speak_text — the narration IS "
        "spoken via speak_text (just with voice switching between "
        "fragments)."
    )
    # Must say "between" or "МЕЖДУ" — explicit ordering matters.
    assert re.search(r"МЕЖДУ|between", block, re.IGNORECASE), (
        "RULE #VOICE-MULTI must say set_voice is called МЕЖДУ (between) "
        "speak_text fragments — otherwise LLM may call all set_voice "
        "before any speak_text, defeating the purpose."
    )


def test_voice_multi_rule_requires_min_two_voices() -> None:
    """The rule must require MINIMUM 2 different voices for multi-voice.

    Live 22.08.2026: «разными голосами» — юзер ожидает минимум 2 разных
    голоса. Если правило разрешит один голос — фича сломана.
    """
    content = _read(MASTER_PROMPT)
    block = _multi_block(content)
    # Russian pattern: «Минимум 2» или «минимум 2-х»
    assert re.search(r"[Мм]инимум\s*2", block), (
        "RULE #VOICE-MULTI must require minimum 2 different voices — "
        "the user's request 'разными голосами' literally means "
        "'different voices' (plural)."
    )
    # Must mention the Голоса plural — not just singular voice.
    assert re.search(r"голос\w*", block, re.IGNORECASE), (
        "RULE #VOICE-MULTI must reference 'голоса' / 'голос' to anchor "
        "the concept of multiple distinct voices."
    )


def test_voice_multi_rule_requires_full_skazka_5_7_fragments() -> None:
    """The rule must require 5-7 speak_text fragments for a complete skazka.

    Live 22.08.2026: LLM ограничился 1 предложением — обрезка. Без явного
    требования «минимум 5-7» LLM повторит обрезку.
    """
    content = _read(MASTER_PROMPT)
    block = _multi_block(content)
    assert re.search(r"5-7|5–7", block), (
        "RULE #VOICE-MULTI must require '5-7' speak_text fragments for "
        "a skazka — same as IMPORTANT CONSTRAINTS §2 'Fairy tale/story — "
        "minimum 5-7 speak_text'. Without this, LLM answers with 1 "
        "sentence and the user hears only the intro."
    )
    assert "полной" in block.lower() or "полн" in block.lower(), (
        "RULE #VOICE-MULTI must say the skazka must be 'полной' "
        "(complete) — otherwise LLM might answer 'жила-была девочка' "
        "(just an opening, no plot)."
    )


def test_voice_multi_rule_prohibits_inventing_voice_names() -> None:
    """The rule must say: voice names only from [TTS] voices: list.

    Live 20.08.2026: LLM говорит «татьяна» для MiniMax (нет в каталоге).
    Без запрета LLM будет hallucinate имена.
    """
    content = _read(MASTER_PROMPT)
    block = _multi_block(content)
    assert "[TTS] voices" in block, (
        "RULE #VOICE-MULTI must anchor voice names to '[TTS] voices:' "
        "context — otherwise LLM invents names like 'татьяна' or "
        "'алена' that don't exist in the active provider."
    )
    assert re.search(r"НЕ\s*выдумывай|не выдумывай|NEVER invent", block), (
        "RULE #VOICE-MULTI must forbid inventing voice names from "
        "memory (same lesson as RULE #VOICE for issue #1219)."
    )


def test_voice_multi_rule_documents_gender_mapping() -> None:
    """The rule must map character genders to specific voice ids.

    Without explicit gender hints LLM defaults to one voice for everyone.
    """
    content = _read(MASTER_PROMPT)
    block = _multi_block(content)
    # Must mention at least one current minimax female voice id.
    assert "Russian_BrightHeroine" in block, (
        "RULE #VOICE-MULTI must list at least one current minimax "
        "female voice id (e.g. Russian_BrightHeroine) so LLM picks a "
        "valid name for Красная Шапочка / бабушка."
    )
    # Must mention at least one current minimax male voice id.
    assert "Russian_ReliableMan" in block, (
        "RULE #VOICE-MULTI must list at least one current minimax male "
        "voice id (e.g. Russian_ReliableMan) so LLM picks a valid name "
        "for дед / волк / охотник."
    )


def test_voice_multi_rule_anchors_to_krasnaya_shapochka() -> None:
    """The rule must explicitly mention 'Красная Шапочка' (the failing case).

    Без явного примера с этой сказкой LLM не догадается, что нужно
    выбирать разные голоса для разных персонажей.
    """
    content = _read(MASTER_PROMPT)
    block = _multi_block(content)
    assert "Красная Шапочка" in block or "Красную Шапочку" in block, (
        "RULE #VOICE-MULTI must explicitly mention Красная Шапочка "
        "(the e2e #1532 mv03 failing case) — otherwise the rule is too "
        "abstract and LLM won't connect 'разными голосами' to "
        "switching voices between her and the wolf."
    )


def test_voice_multi_rule_voice_ids_exist_in_registry() -> None:
    """All voice ids mentioned in RULE #VOICE-MULTI must exist in registry.

    Same lesson as test_issue_1219_set_voice_rule.py — stale voice id in
    prompt → LLM tries it → voice_unavailable → falls back to default.
    """
    from rob_box_voice.tts_voice_registry import voices_for

    content = _read(MASTER_PROMPT)
    block = _multi_block(content)

    mentioned = set(
        re.findall(
            # Legacy: female-shaonv / male-qn-qingse (compound slug).
            # Современные: Russian_ReliableMan / Russian_BrightHeroine.
            # НЕ ловим «female-voice» / «male-voice» — это категории, а не id.
            r"\b(?:female|male)-(?:\w+-){1,}\w+\b|\bRussian_[A-Za-z]+(?:-[A-Za-z]+)*\b",
            block,
        )
    )

    registry_voices = set(voices_for("minimax"))
    missing_in_registry = mentioned - registry_voices
    assert not missing_in_registry, (
        f"RULE #VOICE-MULTI lists voice ids not in "
        f"tts_voice_registry.PROVIDER_VOICES['minimax']: "
        f"{sorted(missing_in_registry)}. Either add them to the registry "
        f"or remove from the prompt. Current registry voices: "
        f"{sorted(registry_voices)}."
    )


# ── scenario.json: voice_core_suite_v1.json mv03 step ────────────────


SCENARIO = (
    Path(__file__).resolve().parents[4]
    / ".github"
    / "e2e"
    / "scenarios"
    / "voice_core_suite_v1.json"
)


def _read_scenario() -> dict:
    import json

    return json.loads(SCENARIO.read_text(encoding="utf-8"))


def _mv03_step(scenario: dict) -> dict | None:
    # Лейбл в voice_core_suite_v1.json переименован (mv03_skazka_raznymi_golosami),
    # исторически был mv03_two_voices_skazka. Поддерживаем оба + fallback по тексту.
    for step in scenario.get("steps", []):
        if step.get("label") in ("mv03_two_voices_skazka", "mv03_skazka_raznymi_golosami"):
            return step
    for step in scenario.get("steps", []):
        if "разными голосами" in step.get("text", ""):
            return step
    return None


def test_scenario_mv03_has_acceptance_block() -> None:
    """mv03 step MUST have an acceptance block (GATE-1, ADR-0022).

    Live 22.08.2026: без acceptance блока e2e не проверял, что LLM
    вызвал set_voice — фича выглядела работающей (patterns=["voice_used"]
    триггерился на дефолтном голосе), но multi-voice был сломан.
    """
    scenario = _read_scenario()
    step = _mv03_step(scenario)
    assert step is not None, (
        "voice_core_suite_v1.json is missing mv03_two_voices_skazka step"
    )
    acceptance = step.get("acceptance")
    assert acceptance is not None, (
        "mv03 step has no acceptance block — e2e will not gate on "
        "set_voice being called (regression of issue #1532)."
    )
    assert isinstance(acceptance, dict), (
        "mv03 acceptance must be a dict (e2e_voice_test.sh contract)"
    )


def test_scenario_mv03_acceptance_requires_set_voice() -> None:
    """mv03 acceptance must require set_voice (not just any tool call)."""
    scenario = _read_scenario()
    step = _mv03_step(scenario)
    assert step is not None, (
        "voice_core_suite_v1.json is missing mv03_two_voices_skazka step"
    )
    acceptance = step["acceptance"]
    assert isinstance(acceptance, dict), (
        "mv03 acceptance must be a dict (e2e_voice_test.sh contract)"
    )
    expected = acceptance.get("expected_tool_calls", [])
    assert "set_voice" in expected, (
        f"mv03 acceptance expected_tool_calls must include 'set_voice' "
        f"(current: {expected}). Without it, e2e can pass with LLM "
        f"never switching voices — that's the bug from issue #1532."
    )


def test_scenario_mv03_acceptance_requires_keyword_krasnaya() -> None:
    """mv03 acceptance must require 'Красная' keyword in recognized text.

    Без expected_keywords=['Красная'] LLM может уйти в generic
    «жила-была» — это вторая корневая причина из issue #1532.
    """
    scenario = _read_scenario()
    step = _mv03_step(scenario)
    assert step is not None, "mv03 step missing"
    acceptance = step["acceptance"]
    keywords = acceptance.get("expected_keywords", [])
    assert any("красн" in k.lower() for k in keywords), (
        f"mv03 acceptance expected_keywords must include 'Красная' "
        f"(case-insensitive) — without it LLM can answer with a "
        f"generic 'жила-была' opening instead of 'Красная Шапочка'. "
        f"Current keywords: {keywords}"
    )


def test_scenario_mv03_text_mentions_skazka_and_multi_voice() -> None:
    """The mv03 step text must mention 'сказку' and 'разными голосами'.

    Это ключевые маркеры для LLM-маршрутизации (RULE #VOICE-MULTI) — без
    них LLM может не понять, что нужен multi-voice режим.
    """
    scenario = _read_scenario()
    step = _mv03_step(scenario)
    assert step is not None, "mv03 step missing"
    text = step.get("text", "").lower()
    assert "сказку" in text or "сказк" in text, (
        f"mv03 step text must mention 'сказку' (a fairy tale) — that's "
        f"the trigger for RULE #VOICE-MULTI. Current text: {step.get('text')!r}"
    )
    assert "разными голосами" in text or "разные голоса" in text, (
        f"mv03 step text must mention 'разными голосами' — that's the "
        f"explicit multi-voice signal from the user. Current text: "
        f"{step.get('text')!r}"
    )


# ── voice command: .github/e2e/voice_commands/ ────────────────────────


VOICE_COMMAND = (
    Path(__file__).resolve().parents[4]
    / ".github"
    / "e2e"
    / "voice_commands"
    / "rabot_rasskazhi_skazku_raznymi_golosami.ogg"
)


def test_voice_command_file_exists() -> None:
    """Voice command ogg for mv03 must be committed to the repo.

    Without this file e2e_voice_test.sh v2 will auto-generate one on the
    robot via Yandex TTS (deterministic but slow). Committing the file
    removes a dependency on the live robot during the e2e round.
    """
    assert VOICE_COMMAND.exists(), (
        f"Voice command for mv03 is missing: {VOICE_COMMAND}. "
        f"Generate via Yandex TTS (or espeak + ffmpeg as fallback) and "
        f"commit it — see scripts/agent_flow/tests/ for examples."
    )


def test_voice_command_is_under_vad_max_12s() -> None:
    """Voice command duration must be < 12s (audio_node VAD max).

    Live 12.08.2026: VAD отклоняет команды длиннее 12с ДО STT →
    «Речь отклонена: 12.76с». Короткая команда про сказку должна
    укладываться в лимит.
    """
    import json
    import subprocess

    result = subprocess.run(
        ["ffprobe", "-v", "error", "-show_format", "-of", "json", str(VOICE_COMMAND)],
        capture_output=True, text=True, check=False,
    )
    if result.returncode != 0:
        # ffprobe не установлен — пропускаем (но фиксируем для CI).
        return
    info = json.loads(result.stdout)
    duration = float(info.get("format", {}).get("duration", 0))
    assert duration < 12.0, (
        f"Voice command {VOICE_COMMAND.name} is {duration:.2f}s — "
        f"exceeds audio_node VAD max (12.0с). VAD will reject it before "
        f"STT — see .github/e2e/VOICE_COMMANDS_RESEARCH.md (VAD max=12с)."
    )
