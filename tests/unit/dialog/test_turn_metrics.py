"""Тесты scripts/dialog/turn_metrics.py на фрагменте реального лога 20.10 (ADR-0150 PR-0, #3684).

Фикстура ``fixtures/turns_2026-10-20.log`` — три куска лога voice-assistant (длинные строки обрезаны):
«покажи анимацию сон» (ход + вопрос «Как тебя зовут?»), «включи голос надежного мужика» (ложный отказ),
«робот» + бэклог (выдуманный отказ).
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts" / "dialog"))

import logparse  # noqa: E402
import turn_metrics  # noqa: E402

FIXTURE = Path(__file__).resolve().parent / "fixtures" / "turns_2026-10-20.log"


@pytest.fixture(scope="module")
def turns():
    return logparse.load_turns(FIXTURE)


def test_three_turns_found(turns):
    assert [t.stt_text for t in turns] == [
        "робот покажи анимацию сон",
        "робот включи голос надежного мужика",
        "робот",
    ]


def test_llm_calls_counted_per_turn(turns):
    # по числу «🔎 LLM REQUEST START» между 🚀 и ✅ (сверено с grep по полному логу)
    assert [t.llm_calls for t in turns] == [3, 3, 3]


def test_tools_visible_taken_from_tools_line(turns):
    assert [t.tools_visible for t in turns] == [48, 48, 48]


def test_stt_to_first_tts_ms(turns):
    # 1792499884.657972345 - 1792499880.518573133
    assert turns[0].stt_to_first_tts_ms == pytest.approx(4139.4, abs=0.1)
    assert turns[1].stt_to_first_tts_ms == pytest.approx(4110.2, abs=0.1)
    assert turns[2].stt_to_first_tts_ms == pytest.approx(10161.4, abs=0.1)


def test_identity_question_counted_once_in_first_turn(turns):
    assert [len(turn_metrics.identity_questions(t)) for t in turns] == [1, 0, 0]


def test_backlog_entries_flushed_into_last_turn(turns):
    assert [t.backlog_entries for t in turns] == [0, 0, 1]


def test_tools_requested_parsed_from_turn_result(turns):
    assert turns[0].tools_requested == ["play_animation", "speak_text"]
    assert turns[1].tools_requested == ["set_voice"]
    assert turns[2].tools_requested == ["music_library"]


def test_percentile_linear_interpolation():
    assert turn_metrics.percentile([1, 2, 3, 4], 0.5) == 2.5
    assert turn_metrics.percentile([10], 0.95) == 10.0
    assert turn_metrics.percentile([], 0.5) is None
    assert turn_metrics.percentile([0, 100], 0.95) == pytest.approx(95.0)


def test_render_contains_a1_a2_a5_rows(turns):
    lines = logparse.read_lines(FIXTURE)
    text = turn_metrics.render(turns, lines)
    assert "A1 `llm_calls`" in text and "A2 `stt_to_first_tts_ms`" in text and "A5-прокси" in text
    assert "`identity_questions`" in text and "`backlog_entries`" in text


def test_retry_turn_inherits_stt_anchor(tmp_path):
    log = tmp_path / "retry.log"
    log.write_text(
        "\n".join(
            [
                "[command_node-8] [INFO] [100.000000000] [command_node]: 🎤 STT: робот включи свет",
                "[dialogue_node-4] [INFO] [100.100000000] [dialogue_node]: 🚀 [turn] calling process_input: user_input='a'",
                "[dialogue_node-4] 🔎 LLM REQUEST START provider=minimax",
                "[dialogue_node-4] [INFO] [101.000000000] [dialogue_node]: ✅ [turn] process_input returned: tools=[]",
                "[dialogue_node-4] [INFO] [101.100000000] [dialogue_node]: 🚀 [turn] calling process_input: user_input='b'",
                "[dialogue_node-4] 🔎 LLM REQUEST START provider=minimax",
                "[tts_node-5] [INFO] [103.000000000] [tts_node]: 🔊 TTS: speech_id=x, voice=default, text='Готово'",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    first, retry = logparse.load_turns(log)
    assert not first.retry and retry.retry
    assert first.stt_to_first_tts_ms is None
    assert retry.stt_to_first_tts_ms == pytest.approx(3000.0)


def test_tier1_and_disputed_stop_share(tmp_path):
    """#3696: доля спорных стопов (В19) и исход Chooser — из маркеров лога, без рантайма робота."""
    log = tmp_path / "tier1.log"
    log.write_text(
        "\n".join(
            [
                "[dialogue_node-4] [INFO] [1.0] [dialogue_node]: 🧩 [tier1] kind=stop_all disputed=False args={} rest='' "
                "silenced=False text='стоп'",
                "[dialogue_node-4] [INFO] [2.0] [dialogue_node]: 🧩 [tier1] kind=stop_all disputed=True args={} rest='' "
                "silenced=False text='спой песню про стоп'",
                "[dialogue_node-4] [INFO] [3.0] [dialogue_node]: 🤔 [stop-chooser] verdict=part_of_command stage=model "
                "ms=640 text='спой песню про стоп'",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    line = turn_metrics._tier1_line(logparse.read_lines(log))
    assert "команды кодом, 0 LLM): 2" in line and "спорных стопов (В19): 1 (50 %)" in line
    assert "part_of_command 1" in line and "stop 0" in line
