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


# -- #4286: лог avatar-supervisor (ТАРС v2) -----------------------------------------------------------------------------

#: Кусок живого лога #4253 (03.11, ``4253_sup_full.log`` строки 26-67, 293-348, 961-1102): №1 «какой у тебя сейчас
#: статус», №9 «покажи последние события» (``get_perception_context`` с ``internet_available``), №13 «как сеть, есть
#: связь» - «Wi-Fi и роутер живы» без единого тула о сети.
TARS_FIXTURE = Path(__file__).resolve().parent / "fixtures" / "tars_supervisor_2026-11-03.log"


@pytest.fixture(scope="module")
def tars_turns():
    return logparse.load_any_turns(TARS_FIXTURE)


def test_supervisor_log_is_split_into_tars_turns(tars_turns):
    """До #4286 аудит искал 🚀/🎤 voice-assistant и по логу супервизора проверял 0 ходов."""
    assert logparse.load_turns(TARS_FIXTURE) == []
    assert [t.tools_executed for t in tars_turns] == [
        ["get_robot_status", "get_robot_status"], ["get_perception_context"], ["get_current_time", "show_metrics"]]
    # реплику №9 лог обрезал внутри снимка, а её ход в куске лога следующий - №13, где она в истории: найдена по ответу;
    # реплика №13 - в истории хода №14, которого в куске нет: пусто, не выдумка
    assert [t.stt_text for t in tars_turns] == ["какой у тебя сейчас статус", "покажи последние события", ""]
    assert all(t.agent == "operator" and t.speeches for t in tars_turns)
    assert tars_turns[2].speeches[0].text.startswith("Связь с интернетом есть, Wi-Fi и роутер живы.")


def test_wifi_claim_without_tool_is_the_only_finding(tars_turns):
    """Интернет - из ``get_perception_context`` прошлого хода (``recent_actions``), батарея - ``get_robot_status``,
    Wi-Fi и роутер - ниоткуда."""
    found = honesty_audit.audit(tars_turns)
    assert [(f.turn, f.rule, f.verdict) for f in found] == [(3, "state:wifi", "claim_without_tool")]


def test_voice_assistant_turns_are_not_judged_by_state_claims():
    """Таблица агентов одна (``knowledge.STATE_CLAIM_AGENTS``): ход Личности правилом состояния не судится."""
    assert honesty_audit.audit_turn(_turn("Wi-Fi и роутер живы.")) == []
