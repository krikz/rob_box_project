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
    assert (
        "A1 `llm_calls`" in text
        and "A2 `stt_to_first_tts_ms`" in text
        and "A5 `prompt_tokens`" in text
    )
    assert "A5 `tools_visible`" in text
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
    """#3696: доля спорных стопов (В19) — из маркеров лога, без рантайма робота."""
    log = tmp_path / "tier1.log"
    log.write_text(
        "\n".join(
            [
                "[dialogue_node-4] [INFO] [1.0] [dialogue_node]: 🧩 [tier1] kind=stop_all disputed=False args={} rest='' "
                "silenced=False text='стоп'",
                "[dialogue_node-4] [INFO] [2.0] [dialogue_node]: 🧩 [tier1] kind=stop_all disputed=True args={} rest='' "
                "silenced=False text='спой песню про стоп'",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    line = turn_metrics._tier1_line(logparse.read_lines(log))
    assert (
        "команды кодом, 0 LLM): 2" in line and "спорных стопов (В19): 1 (50 %)" in line
    )
    assert "Chooser" not in line


def _prompt_line(ts, tokens, over):
    return (
        f"[dialogue_node-4] [INFO] [{ts}] [dialogue_node]: 🧮 [prompt] tokens={tokens} estimated=False "
        f"provider=deepseek view=status system=3100 facts=0 window=400 snapshot=300 turn=20 tools=3500 over={over}"
    )


def test_prompt_tokens_per_llm_call_a5(tmp_path):
    """#3696: A5 — токены каждого вызова LLM хода из строки ``🧮 [prompt]`` и блоки сверх бюджета."""
    log = tmp_path / "a5.log"
    log.write_text(
        "\n".join(
            [
                "[command_node-8] [INFO] [100.0] [command_node]: 🎤 STT: робот который час",
                "[dialogue_node-4] [INFO] [100.1] [dialogue_node]: 🚀 [turn] calling process_input: user_input='a'",
                _prompt_line(100.2, 9000, "-"),
                _prompt_line(101.0, 13000, "system,tools"),
                "[dialogue_node-4] [INFO] [102.0] [dialogue_node]: ✅ [turn] process_input returned: tools=[]",
                "[command_node-8] [INFO] [110.0] [command_node]: 🎤 STT: робот привет",
                "[dialogue_node-4] [INFO] [110.1] [dialogue_node]: 🚀 [turn] calling process_input: user_input='b'",
                _prompt_line(110.2, 11000, "tools"),
                "[dialogue_node-4] [INFO] [111.0] [dialogue_node]: ✅ [turn] process_input returned: tools=[]",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    lines = logparse.read_lines(log)
    turns = logparse.split_turns(lines)
    assert [t.prompt_tokens for t in turns] == [[9000, 13000], [11000]]
    assert [t.prompt_over for t in turns] == [["system", "tools"], ["tools"]]
    text = turn_metrics.render(turns, lines)
    assert (
        "| A5 `prompt_tokens` на вызов LLM (порог p50 ≤ 12000, max ≤ 20000) | 3 | 11000 | 12800 | 13000 |"
        in text
    )
    assert "блоки сверх бюджета (`over=`, вызовов): system 1, tools 2." in text
    assert "9000,13000" in turn_metrics.render_per_turn(turns)


def test_prompt_mark_is_the_runtime_mark():
    """Маркер в скрипте — тот же, что пишет рантайм (``rob_box_dialog.context``): две копии строки сверены."""
    context = pytest.importorskip("rob_box_dialog.context")
    assert logparse.PROMPT_TOKENS_MARK == context.PROMPT_TOKENS_MARK


# -- v2-агрегат 📊 [turn] (#4354 / #4694): split_turns режет живой dialog_node без process_input -----------------

_V2_AGG = (
    "📊 [turn] turn_id=aa11bb22cc33 epoch=2 agent=personality stage=done status=ok "
    "latency_ms=5900 llm_calls=1 llm_provider=deepseek tokens=5200 reason=-"
)


def test_v2_aggregate_single_line_is_one_turn(tmp_path):
    """Одна строка ``📊 [turn]`` (формат turn_log.TurnAggregate) — один ход; process_input не нужен."""
    log = tmp_path / "v2_one.log"
    log.write_text(
        f"[dialog_node-3] [INFO] [1792750753.000000000] [dialog_node]: {_V2_AGG}\n",
        encoding="utf-8",
    )
    turns = logparse.split_turns(logparse.read_lines(log))
    assert len(turns) == 1
    assert turns[0].llm_calls == 1
    assert turns[0].agent == "personality"
    assert turns[0].start_line == turns[0].end_line == 1


def test_v2_aggregate_window_includes_stt_and_llm(tmp_path):
    """Окно v2: STT и LLM REQUEST до агрегата входят в ход; llm_calls берётся из строки 📊."""
    log = tmp_path / "v2_window.log"
    log.write_text(
        "\n".join(
            [
                "[stt_node-5] [INFO] [100.0] [stt_node]: 🎤 STT: робот который час",
                "[dialog_node-3] [INFO] [100.1] [dialog_node]: 🔎 LLM REQUEST START provider=deepseek",
                "[dialog_node-3] tools(12): get_current_time, say",
                "[dialog_node-3] [INFO] [100.2] [dialog_node]: Инструмент get_current_time выполнен успешно",
                f"[dialog_node-3] [INFO] [101.0] [dialog_node]: {_V2_AGG}",
                "[stt_node-5] [INFO] [110.0] [stt_node]: 🎤 STT: робот пока",
                "[dialog_node-3] [INFO] [110.1] [dialog_node]: 🔎 LLM REQUEST START provider=deepseek",
                (
                    "[dialog_node-3] [INFO] [111.0] [dialog_node]: 📊 [turn] turn_id=bb22cc33dd44 "
                    "epoch=2 agent=personality stage=done status=finished latency_ms=10 "
                    "llm_calls=2 llm_provider=deepseek tokens=100 reason=-"
                ),
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    turns = logparse.split_turns(logparse.read_lines(log))
    assert len(turns) == 2
    assert turns[0].stt_text == "робот который час"
    assert (
        turns[0].llm_calls == 1
    )  # из агрегата, не «сколько REQUEST в окне после end-mark»
    assert turns[0].tools_visible == 12
    assert turns[0].tools_executed == ["get_current_time"]
    assert turns[1].stt_text == "робот пока"
    assert turns[1].llm_calls == 2


def test_v1_process_input_fixture_still_splits(turns):
    """Старая фикстура process_input по-прежнему даёт ходы (регрессия v1-пути)."""
    assert len(turns) == 3
    assert all(t.stt_text for t in turns)


def test_aggregate_mark_matches_runtime():
    """Копия маркера в logparse — та же строка, что turn_log.AGGREGATE_MARK."""
    turn_log = pytest.importorskip("rob_box_dialog.turn_log")
    assert logparse.TURN_AGGREGATE_MARK == turn_log.AGGREGATE_MARK
