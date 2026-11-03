# -*- coding: utf-8 -*-
"""#4077 (остаток, эпик #4236): ``requires`` (П4), узкие подстроки (П7), ответ n908, параметры тулов (П6).

Судят те же функции, что у живого харнесса: ``e2e_events`` (requires), ``e2e_tool_match.step_verdict`` (acceptance),
``patterns_missing`` (patterns). Логи -- синтетические, строки в формате робота.

Run:
    python -m pytest tests/unit/e2e_scripts/test_issue_4077_requires_and_narrow.py -v -o addopts=""
"""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict

import pytest

ROOT = Path(__file__).resolve().parents[3]
for _dir in (ROOT / "scripts" / "e2e", ROOT / ".github" / "workflows" / "scripts"):
    if str(_dir) not in sys.path:
        sys.path.insert(0, str(_dir))

import e2e_events as eev  # noqa: E402
import e2e_tool_match as etm  # noqa: E402
import gen_night_marathon as gen  # noqa: E402

EVENTS = ROOT / ".github" / "workflows" / "scripts" / "e2e_events.py"
NIGHT = ROOT / ".github" / "e2e" / "scenarios" / "night"
DONE = "📊 [turn] turn_id=t1 epoch=1 agent=main stage=done status=finished latency_ms=900 llm_calls=2 llm_provider=x tokens=10"


def _tts(text: str) -> str:
    return f"[tts_node] 🔊 TTS: speech_id=76400, dialogue_id=None, batch=None None/None, voice=default, lang=default, text='{text}'"


def _steps() -> Dict[str, Dict[str, Any]]:
    return {s["label"]: s for act in gen.ACTS for s in act["steps"]}


def _verdict(label: str, *lines: str) -> Dict[str, Any]:
    step = _steps()[label]
    log = "\n".join(lines)
    missing = etm.patterns_missing(log, step["patterns"])
    acc = etm.step_verdict(step.get("acceptance", {}), log, label=label)
    return {"missing": missing, "acc": acc, "pass": not missing and acc["pass"]}


# ── П4: requires ─────────────────────────────────────────────────────────────────────────────────────────────


def test_requires_skips_when_source_failed_in_this_act():
    jsonl = '{"label": "n602", "status": "FAIL"}\n{"label": "n602", "status": "FAIL", "detail": "x"}\n'
    got = eev.requires_verdict(("n602",), eev.step_statuses(jsonl), {})
    assert got["skip"] == "requires:n602" and "FAIL" in got["evidence"]


def test_requires_last_record_wins_so_retry_ok_is_ok():
    jsonl = '{"label": "n602", "status": "FAIL"}\n{"label": "n602", "status": "OK", "detail": "recovered_no_wake"}\n'
    assert eev.requires_verdict(("n602",), eev.step_statuses(jsonl), {})["skip"] == "-"


def test_requires_skipped_source_cascades():
    got = eev.requires_verdict(("n805",), {"n805": "SKIP"}, {})
    assert got["skip"] == "requires:n805"


def test_requires_prior_act_fail_skips_and_current_beats_prior():
    prior = eev.prior_statuses("n203_sasha_memory_tea=FAIL,n204=OK")
    assert eev.requires_verdict(("n203_sasha_memory_tea",), {}, prior)["skip"] == "requires:n203_sasha_memory_tea"
    assert eev.requires_verdict(("n203_sasha_memory_tea",), {"n203_sasha_memory_tea": "OK"}, prior)["skip"] == "-"


def test_requires_unknown_source_is_not_proven_step_plays_loudly():
    got = eev.requires_verdict(("n203",), {}, {})
    assert got["skip"] == "-" and got["unknown"] == "n203"


def test_requires_schema_errors():
    for bad in ([], "n1", [1], ["bad label"]):
        with pytest.raises(ValueError):
            eev.requires({"label": "x", "requires": bad})
    with pytest.raises(ValueError):
        eev.requires({"label": "x", "requires": ["x"]})
    with pytest.raises(ValueError):  # источник стоит позже зависимого
        eev.requires_rules({"steps": [{"label": "a", "requires": ["b"]}, {"label": "b"}]})


def test_generator_writes_requires_into_the_scenarios():
    steps = _steps()
    gen._apply_requires()
    assert steps["n603_confirm_dj_playing"]["requires"] == ["n602_dj_on"]
    assert steps["n908_search_borsch_memory"]["requires"] == ["n203_sasha_memory_tea"]
    on_disk = {}
    for path in NIGHT.glob("night_marathon_act*_v1.json"):
        if "acceptance" in path.name:
            continue
        for step in json.loads(path.read_text(encoding="utf-8")).get("steps", []):
            on_disk[step["label"]] = step.get("requires")
    assert {k: v for k, v in on_disk.items() if v} == gen.REQUIRES


def test_cli_require_reads_steps_file_and_prior(tmp_path):
    steps = tmp_path / "steps.jsonl"
    steps.write_text('{"label": "n805_hard_stop", "status": "FAIL"}\n', encoding="utf-8")
    out = subprocess.run([sys.executable, str(EVENTS), "require", "--label", "n805b", "--labels", "n805_hard_stop",
                          "--steps", str(steps), "--prior", ""], capture_output=True, text=True, encoding="utf-8")
    assert "skip=requires:n805_hard_stop" in out.stdout, out
    out = subprocess.run([sys.executable, str(EVENTS), "require", "--label", "n908", "--labels", "n203",
                          "--steps", str(tmp_path / "none.jsonl"), "--prior", "n203=OK"],
                         capture_output=True, text=True, encoding="utf-8")
    assert "skip=-" in out.stdout and "unknown=-" in out.stdout, out


# ── П7: узкие подстроки ──────────────────────────────────────────────────────────────────────────────────────


@pytest.mark.parametrize("label", ["n601_silence_check", "n609_silence_after_dj", "n701_silence_check"])
def test_silence_question_denial_is_red(label):
    got = _verdict(label, _tts("Нет, не тихо, музыка ещё играет."), DONE)
    assert not got["pass"] and got["acc"]["forbidden_said"], got


@pytest.mark.parametrize("label", ["n601_silence_check", "n609_silence_after_dj", "n701_silence_check"])
def test_silence_question_honest_answer_is_green(label):
    assert _verdict(label, _tts("Да, тихо, ничего не играет."), DONE)["pass"]


def test_n1005_not_switched_off_is_red_and_switched_off_is_green():
    red = _verdict("n1005_music_state_final", _tts("Нет, музыка не выключена."), DONE)
    assert not red["pass"], red
    assert _verdict("n1005_music_state_final", _tts("Да, музыка выключена."), DONE)["pass"]


@pytest.mark.parametrize("said", ["Забит в стену бит.", "Это был урок.", "Идёт речь о ремонте.", "Треков нет."])
def test_playing_answer_is_not_fooled_by_substrings(said):
    got = _verdict("n603_confirm_dj_playing", _tts(said), DONE)
    assert not got["pass"], said


def test_playing_answer_accepts_the_state_verb():
    assert _verdict("n603_confirm_dj_playing", _tts("Сейчас играет спокойный сет."), DONE)["pass"]


# ── n908: ретрай по существу ─────────────────────────────────────────────────────────────────────────────────


def test_n908_attempt_with_beet_and_onion_exclusion_in_other_words_is_green():
    said = "Борщ со свёклой и капустой, лука ни грамма, как ты просил."
    assert _verdict("n908_search_borsch_memory", _tts(said), DONE)["pass"]


def test_n908_recipe_without_beet_is_red():
    said = "Борщ без лука: капуста, картошка, мясо на бульон. Варить два часа."
    got = _verdict("n908_search_borsch_memory", _tts(said), DONE)
    assert not got["pass"] and got["acc"]["missing_keywords"] == [gen.RECIPE_ANSWER], got


def test_n908_recipe_commanding_onion_is_red():
    said = "Свёкла, капуста, лука ни грамма, но обжарьте лук до золота."
    got = _verdict("n908_search_borsch_memory", _tts(said), DONE)
    assert not got["pass"] and got["acc"]["forbidden_said"], got


# ── П6: параметры/успех тулов ────────────────────────────────────────────────────────────────────────────────


def test_n907_search_needs_success_not_request_line():
    request = "📥 Запрос выполнения: search_web с параметрами {'query': 'космос'}"
    assert _verdict("n907_search_space", request, DONE)["missing"]
    ok = "[mcp_server]: ✅ Инструмент search_web выполнен успешно"
    assert not _verdict("n907_search_space", request, ok, DONE)["missing"]


def test_n903_animation_must_be_police_lights():
    done = "[mcp_server]: ✅ Инструмент play_animation выполнен успешно"
    snd = "[mcp_server]: ✅ Инструмент play_sound выполнен успешно"
    wrong = "📥 Запрос выполнения: play_animation с параметрами {'animation': 'happy'}"
    right = "📥 Запрос выполнения: play_animation с параметрами {'animation': 'police_lights'}"
    assert _verdict("n903_sound_siren_anim", snd, wrong, done, DONE)["missing"]
    assert not _verdict("n903_sound_siren_anim", snd, right, done, DONE)["missing"]


def test_n1009_green_tea_fact_not_question_echo():
    mem = "[mcp_server]: ✅ Инструмент memory_search выполнен успешно"
    assert not _verdict("n1009_memory_context_after_reset", mem, _tts("Какой чай ты пьёшь, не помню."), DONE)["pass"]
    assert _verdict("n1009_memory_context_after_reset", mem, _tts("Ты пьёшь зелёный чай."), DONE)["pass"]


def test_n810_rap_needs_performance_and_no_deck_stop():
    start = "🎤 выступление старт kind=rap id=1"
    speech = "🗣️ SpeechEvent started speech_id=aa agent=tars epoch=1 text_len=9"
    assert _verdict("n810_rap_over_music", speech, DONE)["missing"]
    assert not _verdict("n810_rap_over_music", speech, start, DONE)["missing"]
    stopped = _verdict("n810_rap_over_music", speech, start, "🎵 [music v2] stop track_id=x reason=user_stop", DONE)
    assert not stopped["pass"]
