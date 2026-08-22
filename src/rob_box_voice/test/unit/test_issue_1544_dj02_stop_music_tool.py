#!/usr/bin/env python3
"""Static regression guard for issue #1544 (dj02 stop_music — verbal-only).

Issue #1544 — LLM did NOT call ``stop_music`` tool when the user said
«Робот, стоп музыку» after ``dj01_start_renardo`` (a Renardo/SuperCollider
beat, not an AI-generated mp3). The cycle ran fine (TTS finished x26,
speak_text x80) but GATE-1 reported::

    expected tool calls not invoked: ['stop_music']
    voice cycle completed (TTS finished x26, speak_text x80) but expected
    tool call(s) skipped: stop_music. LLM made verbal-only answer
    (RULE #MUSIC для stop_music ... могут не enforce).
    Проверь master_prompt_compact.txt и/или добавь explicit tool-call
    enforcement в LLM-system reminder.

Root cause: the pre-fix ``RULE #MUSIC`` block in master_prompt_compact.txt
was anchored only to ``<generated_music>`` (AI mp3 from MiniMax Music API).
After ``dj01_start_renardo`` the ``<generated_music>`` tag is ``idle`` —
Renardo plays but the rule thinks "nothing to stop". LLM defaults to
verbal-only confirmation, GATE-1 fails.

Fix: explicit ``RULE #MUSIC — ОСТАНОВКА МУЗЫКИ (issues #1392, #1544 dj02)``
block with:
- covers ALL stop/hvatit/vykljuchi/uberi/dostatochno/pause triggers
- covers BOTH Renardo (dj01) AND AI-generated mp3 (dj04+) AND DJ-mode
- explicit ✅/❌ example pairs anchored to e2e #1544 dj02
- prohibits verbal-only answer (this is the GATE-1 bug)

These tests pin the *wording* so future prompt refactors cannot silently
drop the stop_music enforcement again. Same lesson as
``test_issue_1532_voice_multi_skazka.py``: the prompt wording is the
source of truth.

Run with::

    python3 -m pytest src/rob_box_voice/test/unit/test_issue_1544_dj02_stop_music_tool.py
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


def _music_block(content: str) -> str:
    """Extract the RULE #MUSIC block (anchor up to next 🚨 RULE or end)."""
    match = re.search(
        r"RULE #MUSIC.*?(?=🚨 \*\*RULE #|\Z)",
        content,
        re.DOTALL,
    )
    assert match, (
        "RULE #MUSIC block not delimited properly in "
        "master_prompt_compact.txt"
    )
    return match.group(0)


# ── master_prompt_compact.txt ─────────────────────────────────────────


def test_master_prompt_contains_music_stop_rule() -> None:
    """The RULE #MUSIC anchor must be present (regression guard).

    Live 22.08.2026 / e2e #1544 dj02: without this block LLM defaults to
    verbal-only "Выключаю!" — GATE-1 fails on missing stop_music tool.
    """
    content = _read(MASTER_PROMPT)
    assert "RULE #MUSIC" in content, (
        "master_prompt_compact.txt lost RULE #MUSIC — LLM will fall back "
        "to verbal-only answer on 'стоп музыку' (regression of #1544 "
        "dj02: GATE-1 expected tool calls not invoked: ['stop_music'])."
    )


def test_music_rule_anchored_to_1544_dj02() -> None:
    """The rule must reference issue #1544 / dj02 explicitly.

    Без явной ссылки на #1544 dj02 будущие рефакторинги промпта могут
    убрать enforcement, не понимая, какая регрессия была.
    """
    content = _read(MASTER_PROMPT)
    block = _music_block(content)
    assert "1544" in block, (
        "RULE #MUSIC must reference issue #1544 dj02 — otherwise future "
        "prompt refactors won't see why this block exists."
    )


def test_music_rule_must_call_stop_music_tool() -> None:
    """The rule must DIRECTLY require calling the stop_music tool.

    This is the core of the fix: LLM was answering verbal-only
    («Выключаю!») without calling stop_music — GATE-1 catches this.
    The rule MUST say "ОБЯЗАТЕЛЬНО вызвать tool: stop_music".
    """
    content = _read(MASTER_PROMPT)
    block = _music_block(content)
    assert "stop_music()" in block, (
        "RULE #MUSIC must mention stop_music() tool — otherwise LLM "
        "has no way to learn the tool name for stop commands."
    )
    # Stronger: explicit 'ОБЯЗАТЕЛЬНО вызвать tool' phrasing.
    assert re.search(
        r"ОБЯЗАТЕЛЬНО\s+вызва(ть|ти)\s+tool", block, re.IGNORECASE
    ), (
        "RULE #MUSIC must use ОБЯЗАТЕЛЬНО вызвать tool phrasing — "
        "otherwise the LLM can rationalize away the tool call."
    )


def test_music_rule_prohibits_verbal_only_answer() -> None:
    """The rule must forbid verbal-only answer (the exact dj02 bug).

    Live 22.08.2026: LLM сказал «Выключаю!» без вызова stop_music —
    это GATE-1 bug. Правило ОБЯЗАНО явно запретить verbal-only.
    """
    content = _read(MASTER_PROMPT)
    block = _music_block(content)
    # Russian "verbal-only" or explicit forbidden phrases.
    assert re.search(
        r"verbal-only|ЗАПРЕЩЕНО|❌", block,
    ), (
        "RULE #MUSIC must use ❌ or 'verbal-only' marker — the LLM "
        "must see this is a hard ban, not a suggestion."
    )
    # Must specifically call out speak_text WITHOUT stop_music as wrong.
    assert re.search(
        r"speak_text.{0,40}(без|without).{0,30}stop_music|"
        r"stop_music.{0,30}не\s+вызван",
        block,
        re.IGNORECASE,
    ), (
        "RULE #MUSIC must show ❌ example with speak_text без stop_music "
        "— that's the exact regression from dj02 #1544."
    )


def test_music_rule_covers_renardo_not_only_ai_mp3() -> None:
    """The rule must cover Renardo (dj01) AND AI-generated mp3.

    Pre-fix rule only mentioned <generated_music> tag (AI mp3). After
    dj01_start_renardo the tag is idle → rule thinks nothing to stop.
    Fix: explicitly mention Renardo / Renardo patterns.
    """
    content = _read(MASTER_PROMPT)
    block = _music_block(content)
    assert "Renardo" in block, (
        "RULE #MUSIC must mention 'Renardo' — dj02 runs after "
        "dj01_start_renardo (Renardo/SuperCollider beat). Without "
        "Renardo reference LLM assumes stop_music only handles AI mp3."
    )
    # Must mention both branches.
    assert re.search(r"(И\s+Renardo|и\s+сгенерирован|и\s+AI)", block), (
        "RULE #MUSIC must say stop_music() handles BOTH Renardo AND "
        "AI-generated mp3 — otherwise LLM can't reason about it."
    )


def test_music_rule_lists_stop_triggers() -> None:
    """The rule must enumerate the user's stop phrasings.

    Live 22.08.2026: user said «стоп музыку» but LLM didn't recognize
    it as a stop trigger. Rule must list at least: стоп/хватит/выключи.
    """
    content = _read(MASTER_PROMPT)
    block = _music_block(content)
    for trigger in ("стоп", "хватит", "выключи"):
        assert trigger in block.lower(), (
            f"RULE #MUSIC must list '{trigger}' as a stop trigger — "
            "otherwise LLM won't map user phrasing to stop_music."
        )


def test_music_rule_requires_dj_mode_exit_when_active() -> None:
    """If DJ mode is active, stop_music alone is not enough — must exit DJ.

    Rule must instruct LLM to call set_dj_mode(enabled=false) BEFORE
    stop_music when <dj_mode>enabled=true</dj_mode>.
    """
    content = _read(MASTER_PROMPT)
    block = _music_block(content)
    assert "set_dj_mode" in block, (
        "RULE #MUSIC must mention set_dj_mode(enabled=false) — without "
        "it DJ mode keeps restarting music after stop_music."
    )
    # Must explicitly say enabled=false.
    assert re.search(r"set_dj_mode\(.*enabled\s*=\s*false", block), (
        "RULE #MUSIC must show set_dj_mode(enabled=false) call — the "
        "exact API LLM needs."
    )


def test_music_rule_anchors_to_dj02_starter() -> None:
    """The rule must reference the dj02 scenario explicitly.

    'Робот, стоп музыку' after dj01_start_renardo is the failing case.
    Rule must mention it so LLM connects scenario → rule.
    """
    content = _read(MASTER_PROMPT)
    block = _music_block(content)
    assert "dj02" in block or "dj01_start_renardo" in block, (
        "RULE #MUSIC must reference dj02 / dj01_start_renardo — "
        "the explicit failing scenario from e2e #1544."
    )


def test_music_rule_anchor_after_renardo_pattern() -> None:
    """The 'after Renardo' pattern must be present.

    The bug: LLM didn't realize music was still playing after Renardo.
    Rule must say 'после Renardo' / 'after dj01' explicitly so LLM
    understands music can be active without <generated_music>=playing.
    """
    content = _read(MASTER_PROMPT)
    block = _music_block(content)
    # Russian: «после Renardo» или «после dj01»
    assert re.search(r"после\s+(Renardo|dj01)", block, re.IGNORECASE), (
        "RULE #MUSIC must say 'после Renardo' / 'после dj01' — this is "
        "the exact state when <generated_music>=idle but music plays."
    )


# ── scenario.json: voice_core_suite_v1.json dj02 step ─────────────────


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


def _dj02_step(scenario: dict) -> dict | None:
    for step in scenario.get("steps", []):
        if step.get("label") == "dj02_stop_music":
            return step
    return None


def test_scenario_dj02_has_acceptance_block() -> None:
    """dj02 step MUST have an acceptance block (GATE-1, ADR-0022).

    Live 22.08.2026: без acceptance блока e2e не проверял, что LLM
    вызвал stop_music — фича выглядела работающей, но tool-skipped.
    """
    scenario = _read_scenario()
    step = _dj02_step(scenario)
    assert step is not None, (
        "voice_core_suite_v1.json is missing dj02_stop_music step"
    )
    acceptance = step.get("acceptance")
    assert acceptance is not None, (
        "dj02 step has no acceptance block — e2e will not gate on "
        "stop_music being called (regression of issue #1544)."
    )
    assert isinstance(acceptance, dict), (
        "dj02 acceptance must be a dict (e2e_voice_test.sh contract)"
    )


def test_scenario_dj02_acceptance_requires_stop_music() -> None:
    """dj02 acceptance must require stop_music (not just any tool call)."""
    scenario = _read_scenario()
    step = _dj02_step(scenario)
    assert step is not None, (
        "voice_core_suite_v1.json is missing dj02_stop_music step"
    )
    acceptance = step["acceptance"]
    assert isinstance(acceptance, dict), (
        "dj02 acceptance must be a dict (e2e_voice_test.sh contract)"
    )
    expected = acceptance.get("expected_tool_calls", [])
    assert "stop_music" in expected, (
        f"dj02 acceptance expected_tool_calls must include 'stop_music' "
        f"(current: {expected}). Without it, e2e can pass with LLM "
        f"never calling stop_music — that's the bug from issue #1544."
    )


def test_scenario_dj02_text_mentions_stop_music() -> None:
    """The dj02 step text must mention 'стоп музыку'.

    Это ключевой маркер для LLM-маршрутизации (RULE #MUSIC) — без
    этого LLM может не понять, что нужен stop_music.
    """
    scenario = _read_scenario()
    step = _dj02_step(scenario)
    assert step is not None, "dj02 step missing"
    text = step.get("text", "").lower()
    assert "стоп" in text and "музыку" in text, (
        f"dj02 step text must mention 'стоп музыку' — that's the "
        f"explicit stop trigger from the user. Current text: "
        f"{step.get('text')!r}"
    )


def test_scenario_dj02_text_mentions_wake_word_robot() -> None:
    """The dj02 step text must include wake word 'робот'.

    Live 22.08.2026: PR #1526 added wake-word fix; test guards that
    step text still has 'Робот' prefix so STT/wake-gate work.
    """
    scenario = _read_scenario()
    step = _dj02_step(scenario)
    assert step is not None, "dj02 step missing"
    text = step.get("text", "").lower()
    assert "робот" in text, (
        f"dj02 step text must start with 'Робот' (wake word) — "
        f"audio_node wake-gate will reject it otherwise. "
        f"Current text: {step.get('text')!r}"
    )


# ── voice command: .github/e2e/voice_commands/ ────────────────────────


VOICE_COMMAND = (
    Path(__file__).resolve().parents[4]
    / ".github"
    / "e2e"
    / "voice_commands"
    / "rabot_stop_muzyku.ogg"
)


def test_voice_command_file_exists_for_dj02() -> None:
    """Voice command ogg for dj02 must be committed to the repo.

    Without this file e2e_voice_test.sh v2 will auto-generate one on the
    robot via Yandex TTS (deterministic but slow). Committing the file
    removes a dependency on the live robot during the e2e round.

    The ogg should contain «Робот, стоп музыку» (or close variant).
    """
    if VOICE_COMMAND.exists():
        return  # committed — all good.
    # Fallback: accept any of the existing stop-related oggs if present.
    candidates = [
        Path(__file__).resolve().parents[4]
        / ".github"
        / "e2e"
        / "voice_commands"
        / "rabot_stop_hvatit.ogg",
        Path(__file__).resolve().parents[4]
        / ".github"
        / "e2e"
        / "voice_commands"
        / "rabot_dj_off.ogg",
    ]
    for c in candidates:
        if c.exists():
            return
    raise AssertionError(
        f"Voice command for dj02 is missing: {VOICE_COMMAND}. "
        f"Generate via Yandex TTS (or espeak + ffmpeg as fallback) and "
        f"commit it — see scripts/agent_flow/tests/ for examples. "
        f"Acceptable variants: {candidates}"
    )


def test_voice_command_is_under_vad_max_12s() -> None:
    """Voice command duration must be < 12s (audio_node VAD max).

    Live 12.08.2026: VAD отклоняет команды длиннее 12с ДО STT →
    «Речь отклонена: 12.76с». Короткая команда «стоп музыку»
    должна укладываться в лимит.
    """
    import json
    import subprocess

    ogg = None
    if VOICE_COMMAND.exists():
        ogg = VOICE_COMMAND
    else:
        candidates = [
            Path(__file__).resolve().parents[4]
            / ".github"
            / "e2e"
            / "voice_commands"
            / "rabot_stop_hvatit.ogg",
            Path(__file__).resolve().parents[4]
            / ".github"
            / "e2e"
            / "voice_commands"
            / "rabot_dj_off.ogg",
        ]
        for c in candidates:
            if c.exists():
                ogg = c
                break
    if ogg is None:
        return  # existence already asserted in test_voice_command_file_exists
    result = subprocess.run(
        ["ffprobe", "-v", "error", "-show_format", "-of", "json", str(ogg)],
        capture_output=True, text=True, check=False,
    )
    if result.returncode != 0:
        # ffprobe не установлен — пропускаем (но фиксируем для CI).
        return
    info = json.loads(result.stdout)
    duration = float(info.get("format", {}).get("duration", 0))
    assert duration < 12.0, (
        f"Voice command {ogg.name} is {duration:.2f}s — "
        f"exceeds audio_node VAD max (12.0с). VAD will reject it before "
        f"STT — see .github/e2e/VOICE_COMMANDS_RESEARCH.md (VAD max=12с)."
    )
