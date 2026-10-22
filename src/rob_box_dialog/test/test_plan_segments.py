"""Реплики выступления (ADR-0150 §5.6.3, §5.6.4; PR-7c #3700 ч.1а): нарезка сегментов кодом, граница сегмента (О7),
правка не начатых сегментов (``edit_pending``, X3) с запасной версией, прогноз конца речи по тактам."""

import itertools

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import Expression, Performance
from rob_box_dialog.plan.estimate import SpeechRate
from rob_box_dialog.plan.segments import FIXED, HEARD, Line, SegmentList, cut, segment_rows
from rob_box_dialog.plan.sync import BeatGrid

GRID = BeatGrid(bpm=120.0, beat_at=100.0, bar_beats=4.0)  # такт 2 с


def make(texts=("раз", "два", "три", "четыре")):
    ids = itertools.count(1)
    return SegmentList([(t, "", "") for t in texts], lambda: f"s{next(ids)}", SpeechRate())


def play(line, start, duration):
    line.state, line.heard_at, line.duration_s, line.ready = "active", start, duration, True


# --- нарезка -------------------------------------------------------------------------------------------------------

def test_cut_keeps_short_segments_and_splits_long_ones_by_sentences():
    assert cut("  куплет  ") == ("куплет",)
    assert cut("   ") == ()
    sentence = "Слово " * 30 + "конец."  # ~186 символов
    text = " ".join([sentence] * 4)
    pieces = cut(text)
    assert len(pieces) >= 2 and all(len(p) <= kn.SEGMENT_MAX_CHARS for p in pieces)
    assert " ".join(pieces).split() == text.split()  # ничего не потеряно и не переставлено
    endless = "ля" * 300  # без пробелов и точек — режется по лимиту
    assert all(len(p) <= kn.SEGMENT_MAX_CHARS for p in cut(endless)) and "".join(cut(endless)) == endless


def test_cut_splits_at_lines_too():
    verse = "\n".join(["строчка куплета номер такая-то"] * 20)
    assert all(len(p) <= kn.SEGMENT_MAX_CHARS for p in cut(verse)) and len(cut(verse)) >= 2


def test_segment_rows_carry_voice_and_emotion_to_every_piece():
    long = "Раз. " * 100
    perf = Performance(kind="tale", segments=("Жил-был кот.", long), voice_per_segment=("", "Wise_Woman"),
                       expression_per_segment=(Expression("happy"), Expression("neutral")))
    rows = segment_rows(perf)
    assert rows[0] == ("Жил-был кот.", "", "happy")
    assert len(rows) > 2 and {r[1:] for r in rows[1:]} == {("Wise_Woman", "")}
    assert segment_rows(perf, skip=1) == rows[1:]


# --- кто где -------------------------------------------------------------------------------------------------------

def test_numbers_and_lookup():
    lines = make()
    assert [line.no for line in lines.lines] == [1, 2, 3, 4] and lines.top(FIXED) == 0
    lines.lines[0].state, lines.lines[1].state = "done", "committed"
    assert lines.top(FIXED) == 2 and lines.top(HEARD) == 1 and lines.live()
    assert lines.find("s3") is lines.lines[2] and lines.find("чужая") is None


def test_code_phrase_goes_before_the_first_not_committed_line():
    lines = make()
    lines.lines[0].state = "committed"
    phrase = lines.insert(Line(0, "Бит не завёлся", "p1", key="perform.no_backing.error", state="committed"))
    assert [line.no for line in lines.lines] == [1, 0, 2, 3, 4]
    lead = lines.insert(Line(0, "Поехали!", "p2"))
    assert lines.lines[2] is lead and lines.find("p1") is phrase  # после отданных — до первого куплета в очереди
    assert [line.no for line in lines.segments()] == [1, 2, 3, 4]


# --- граница сегмента (О7) -----------------------------------------------------------------------------------------

def test_first_line_is_due_as_soon_as_it_is_synthesized():
    lines = make()
    assert lines.due(0.0) is None  # пред-синтез не готов — ждать
    lines.lines[0].ready = True
    assert lines.due(0.0) == (lines.lines[0], 0.0)


def test_next_line_is_due_ahead_of_the_end_of_the_sounding_one():
    lines = make()
    play(lines.lines[0], start=100.0, duration=8.0)
    lines.lines[1].ready = True
    assert lines.due(105.0) is None  # до конца 3 с > AHEAD
    assert lines.due(108.0 - kn.PERFORM_COMMIT_AHEAD_S) == (lines.lines[1], 108.0)


def test_committed_not_started_line_blocks_the_next():
    lines = make()
    lines.lines[0].state, lines.lines[1].ready = "committed", True
    assert lines.due(1000.0) is None
    lines.lines[0].state = "done"
    assert lines.due(1000.0)[0] is lines.lines[1]


def test_unknown_duration_leaves_the_order_to_the_speech_owner():
    lines = make()
    lines.lines[0].state, lines.lines[0].heard_at = "active", 100.0  # длительность не сообщена
    lines.lines[1].ready = True
    assert lines.due(100.1) == (lines.lines[1], 0.0)


# --- правка не начатых (X3) и запасная версия (О7) ------------------------------------------------------------------

def test_replace_tail_keeps_ready_old_versions_as_spares():
    lines = make()
    lines.lines[0].state = "active"
    lines.lines[1].ready = lines.lines[2].ready = True  # 2 и 3 синтезированы, 4 — нет
    old2, old3, old4 = lines.lines[1:]
    new, numbers = lines.replace_tail(2, [("енот 2", "", ""), ("енот 3", "", ""), ("енот 4", "", ""),
                                          ("енот 5", "", "")])
    assert numbers == [2, 3, 4, 5]
    assert [line.text for line in lines.lines] == ["раз", "енот 2", "енот 3", "енот 4", "енот 5"]
    assert new[0].fallback is old2 and new[1].fallback is old3 and new[2].fallback is None
    assert old4.state == "replaced" and old4 in lines.gone
    assert lines.find(old2.speech_id) is old2  # события запасной версии узнаются


def test_shorter_edit_drops_the_old_tail():
    lines = make()
    new, numbers = lines.replace_tail(3, [("финал", "", "")])
    assert numbers == [3, 4] and [line.text for line in lines.lines] == ["раз", "два", "финал"]


def test_at_the_boundary_the_synthesized_edit_wins():
    lines = make()
    play(lines.lines[0], 100.0, 4.0)
    lines.lines[1].ready = True
    (new, *_), _ = lines.replace_tail(2, [("енот", "", "")])
    new.ready = True
    line, _ = lines.due(103.0)
    assert line is new and new.fallback is None and lines.gone[-1].state == "replaced"


def test_o7_edit_not_ready_at_the_boundary_yields_to_the_old_version():
    lines = make()
    play(lines.lines[0], 100.0, 4.0)
    old2 = lines.lines[1]
    old2.ready = True
    (new, *_), _ = lines.replace_tail(2, [("енот", "", "")])
    line, _ = lines.due(103.0)
    assert line is old2 and old2.state == "pending" and lines.lines[1] is old2
    assert new.state == "late" and new in lines.gone


def test_edit_before_anything_sounded_waits_for_the_new_version():
    """Ещё ничего не звучало — границы нет: ждём синтеза правки, а не играем прежнюю."""
    lines = make()
    lines.lines[0].ready = True
    lines.replace_tail(1, [("енот", "", "")])
    assert lines.due(0.0) is None


# --- прогноз и учёт ------------------------------------------------------------------------------------------------

def test_predicted_end_lays_out_on_bars():
    lines = make(("а" * 22, "б" * 33))  # 2 с и 3 с при 11 симв/с
    assert lines.predicted_end(100.0, None) == pytest.approx(105.0)
    # первая на такте 100 до 102, вторая — на такте 102, кончится в 105
    assert lines.predicted_end(100.0, GRID) == pytest.approx(105.0)
    assert lines.predicted_end(100.5, GRID) == pytest.approx(107.0)  # первая — с такта 102


def test_predicted_end_uses_the_sounding_line_fact():
    lines = make(("а" * 22, "б" * 22))
    play(lines.lines[0], start=100.0, duration=3.0)  # синтез сказал 3 с, а не оценка 2 с
    assert lines.predicted_end(101.0, None) == pytest.approx(105.0)


def test_overdue_and_heard_stats():
    lines = make()
    play(lines.lines[0], 100.0, 2.0)
    assert lines.overdue(102.0 + kn.PLAN_WAIT_SLACK_S - 0.1) == []
    assert lines.overdue(102.0 + kn.PLAN_WAIT_SLACK_S + 0.1) == [lines.lines[0]]
    lines.lines[0].state, lines.lines[0].spoken_chars, lines.lines[0].synced = "done", 3, True
    lines.lines[1].state, lines.lines[1].spoken_chars, lines.lines[1].synced = "cancelled", 1, True
    assert lines.heard_stats() == (1, 4, True)
    lines.lines[1].synced = False
    assert lines.heard_stats() == (1, 4, False)
