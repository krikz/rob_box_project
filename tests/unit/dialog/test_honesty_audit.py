"""Тесты scripts/dialog/honesty_audit.py (ADR-0150 PR-0, #3684, A3).

На реальном фрагменте лога 20.10 аудит обязан найти ровно два позорных хода: ложный отказ «Ничего не заиграло» на
смену голоса (строки 3534/3536 полного лога) и выдуманный отказ «Сгенерить … не может» без generate_music (4174/4176).
Ход с «Как тебя зовут?» (play_animation выполнен, вопрос о личности) отчётом об исполнении не является.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts" / "dialog"))

import honesty_audit  # noqa: E402
import logparse  # noqa: E402

FIXTURE = Path(__file__).resolve().parent / "fixtures" / "turns_2026-10-20.log"


@pytest.fixture(scope="module")
def findings():
    return honesty_audit.audit(logparse.load_turns(FIXTURE))


def test_finds_false_denial_on_set_voice_turn(findings):
    f = next(x for x in findings if x.rule == "music_denied")
    assert f.verdict == "denial_without_attempt"
    assert f.tools_executed == ("set_voice",)
    assert "Ничего не заиграло" in f.text
    # строки внутри фикстуры: WARN «фраза кода вместо ответа модели» и 🔊 TTS
    assert len(f.lines) == 2


def test_finds_invented_generation_refusal(findings):
    f = next(x for x in findings if x.rule == "generate_denied")
    assert f.verdict == "denial_without_attempt"
    assert "generate_music" not in f.tools_executed
    assert set(f.tools_executed) <= {"music_library"}


def test_exactly_two_findings(findings):
    assert sorted(f.rule for f in findings) == ["generate_denied", "music_denied"]


def _turn(spoken: str, executed=(), failed=()):
    t = logparse.Turn(index=1, anchor_line=1, start_line=1, end_line=9)
    t.tools_executed, t.tools_failed = list(executed), list(failed)
    t.speeches = [logparse.Speech(spoken, (5,), ("tts",))]
    return t


def test_launch_claim_without_music_tool_is_flagged():
    found = honesty_audit.audit_turn(_turn("Включаю «Воробушек на ладони» из библиотеки."))
    assert [f.verdict for f in found] == ["claim_without_tool"]


def test_launch_claim_backed_by_music_tool_passes():
    assert honesty_audit.audit_turn(_turn("Включаю «Воробушек на ладони».", executed=["music_library"])) == []


def test_runtime_launch_stems_are_part_of_the_vocabulary():
    # словарь основ — из media_phrases.LAUNCH_PHRASE_STEMS (единая таблица знания)
    found = honesty_audit.audit_turn(_turn("Токкату ре минор Баха ставлю мелодией."))
    assert [f.rule for f in found] == ["music_launch"]


def test_denial_after_real_attempt_is_not_flagged():
    t = _turn("Сгенерить новый трек сейчас не могу.", failed=["generate_music"])
    t.speeches = [logparse.Speech("Сгенерить новый трек облако сейчас не может.", (5,), ("tts",))]
    assert honesty_audit.audit_turn(t) == []


def test_memory_claim_without_memory_save_is_flagged():
    found = honesty_audit.audit_turn(_turn("Записала: Саша любит зелёный чай."))
    assert [f.rule for f in found] == ["memory_saved"]


def test_chatter_is_not_flagged():
    assert honesty_audit.audit_turn(_turn("Как тебя зовут?")) == []
