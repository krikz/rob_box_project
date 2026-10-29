#!/usr/bin/env python3
"""Static regression guard for issue #1532 (mv03 multi-voice skazka).

Issue #1532 — LLM did NOT call ``set_voice`` for a multi-voice skazka
(«робот, расскажи сказку про Красную Шапочку разными голосами»):

    spoken='Жила-была девочка. Мама испекла пирожки...'
    tools=[]
    # TTS played ONE voice on ONE sentence — all characters sounded the same.

Root cause: ``master_prompt_compact.txt`` had no rule teaching the LLM
when/how to interleave ``set_voice`` between ``speak_text`` fragments for
different characters. The voice rule (issue #1219) only described global
voice-change, not per-character switching in a single narrative.

Fix: an explicit multi-voice block in ``master_prompt_compact.txt``
with:
- per-character switching contract (example with 3 voices + 4 fragments)
- 7 mandatory rules (full skazka 5-7 fragments, ≥2 different voices,
  gender-based mapping, do NOT invent names, return to default at end)
- ✅/❌ examples anchored to e2e #1532 mv03

#3696 (ADR-0148, ADR-0150 A5): the prompt-wording tests were removed with the
hand-written multi-voice block; per-reply voices are the ``voice``
parameter of ``speak_text`` (its schema) and the ``perform`` rule of ``knowledge.PROMPT_RULES``.
What stays here is the e2e scenario contract (mv03) and the voice command file.

Run with::

    python3 -m pytest src/rob_box_voice/test/unit/test_issue_1532_voice_multi_skazka.py
"""

from __future__ import annotations

from pathlib import Path

import pytest

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

    Это ключевые маркеры для LLM-маршрутизации (мульти-голос) — без
    них LLM может не понять, что нужен multi-voice режим.
    """
    scenario = _read_scenario()
    step = _mv03_step(scenario)
    assert step is not None, "mv03 step missing"
    text = step.get("text", "").lower()
    assert "сказку" in text or "сказк" in text, (
        f"mv03 step text must mention 'сказку' (a fairy tale) — that's "
        f"the trigger for the multi-voice route. Current text: {step.get('text')!r}"
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


def _vad_max_duration() -> float:
    """`speech_max_duration` from the config the robot actually loads.

    Was hard-coded 12.0 here. The limit had since been raised to 15.0 in
    `audio_node.py` and in the deployed YAML, but not in the package one,
    so this test enforced a number no running node used. Read it instead.
    """
    import yaml

    config = (
        Path(__file__).resolve().parents[4]
        / "docker" / "sound" / "config" / "voice_assistant" / "audio_node.yaml"
    )
    with config.open(encoding="utf-8") as fh:
        loaded = yaml.safe_load(fh)
    return float(loaded["audio_node"]["ros__parameters"]["speech_max_duration"])


def test_voice_command_is_under_vad_max() -> None:
    """Voice command duration must be under audio_node's VAD maximum.

    Live 12.08.2026: VAD отклоняет слишком длинные команды ДО STT →
    «Речь отклонена: 12.76с». Короткая команда про сказку должна
    укладываться в лимит.
    """
    import json
    import shutil
    import subprocess

    # The `returncode != 0` guard below never ran when ffprobe was absent:
    # `subprocess.run` raises FileNotFoundError before returning anything,
    # so the CI runner (no ffmpeg installed) got an error, not a skip.
    if shutil.which("ffprobe") is None:
        pytest.skip("ffprobe is not installed")

    result = subprocess.run(
        ["ffprobe", "-v", "error", "-show_format", "-of", "json", str(VOICE_COMMAND)],
        capture_output=True, text=True, check=False,
    )
    if result.returncode != 0:
        pytest.skip(f"ffprobe could not read {VOICE_COMMAND.name}")
    info = json.loads(result.stdout)
    duration = float(info.get("format", {}).get("duration", 0))
    vad_max = _vad_max_duration()
    assert duration < vad_max, (
        f"Voice command {VOICE_COMMAND.name} is {duration:.2f}s — "
        f"exceeds audio_node VAD max ({vad_max}с). VAD will reject it "
        f"before STT — see .github/e2e/VOICE_COMMANDS_RESEARCH.md."
    )
