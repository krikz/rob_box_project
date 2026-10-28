"""Старт на такте (ADR-0150 §5.6.3, X2; PR-7c #3700): формула ``beat_at``, ``next_bar`` и раскладка на синтетических
``started``; подложка выступления — попытки, срок, поздний ``started``, продление."""

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.plan.backing import Backing, BackingTrack, grid_of
from rob_box_dialog.plan.sync import (BeatGrid, Slot, bar_at_or_after, beat_heard_at, beat_offset_ms, earliest, layout,
                                      next_bar)

# 120 BPM, 4/4: такт 2 с; доля старта подложки звучит в t=100
GRID = BeatGrid(bpm=120.0, beat_at=100.0, bar_beats=4.0, start_beat=16.0)
LAT_MS = 550


def test_beat_heard_at_formula():
    """Колбэк клока опоздал на 0.25 доли (0.125 с при 120 BPM), ноты звучат на Clock.latency 0.5 с позже доли."""
    assert beat_heard_at(ts=50.0, late_beats=0.25, bpm=120.0, latency_s=0.5) == pytest.approx(50.375)
    assert beat_heard_at(ts=50.0, late_beats=0.0, bpm=90.0, latency_s=0.0) == 50.0


def test_grid_geometry():
    assert GRID.bar_s == pytest.approx(2.0)
    assert GRID.beat_time(16.0) == pytest.approx(100.0)
    assert GRID.beat_time(48.0) == pytest.approx(116.0)  # 32 доли = 16 с
    with pytest.raises(ValueError):
        BeatGrid(bpm=0.0, beat_at=1.0, bar_beats=4.0)


def test_beat_offset_is_signed_ms_from_the_nearest_beat():
    """A19: доля = 0.5 с при 120 BPM; позже ближайшей доли — плюс, раньше — минус; сетки нет — ``None``."""
    assert beat_offset_ms(GRID, 108.0) == 0
    assert beat_offset_ms(GRID, 108.03) == 30
    assert beat_offset_ms(GRID, 107.97) == -30
    assert beat_offset_ms(GRID, 108.2) == 200  # ближайшая доля 108.0, а не 108.5
    assert beat_offset_ms(GRID, 108.3) == -200  # ближайшая — 108.5
    assert beat_offset_ms(GRID, 95.8) == -200  # раньше старта подложки сетка продолжается назад (ближайшая — 96.0)
    assert beat_offset_ms(None, 108.0) is None


def test_no_grid_means_right_after_previous_and_honest_unsynced():
    assert next_bar(None, now=10.0, output_latency_ms=LAT_MS) == Slot(0.0, False)


def test_first_segment_on_the_nearest_bar_after_latency():
    """now=99: самый ранний звук 99 + 0.55 + lead 0.3 = 99.85 → такт 100 (первая доля подложки)."""
    assert earliest(99.0, LAT_MS) == pytest.approx(99.0 + 0.55 + kn.PERFORM_COMMIT_LEAD_S)
    assert next_bar(GRID, now=99.0, output_latency_ms=LAT_MS) == Slot(pytest.approx(100.0), True)
    # 100.5 + 0.85 = 101.35 → 102
    assert next_bar(GRID, now=100.5, output_latency_ms=LAT_MS).at == pytest.approx(102.0)


def test_never_before_music_start():
    """Подложка ещё не зазвучала (now сильно раньше beat_at): тактов раньше её доли старта нет."""
    assert next_bar(GRID, now=80.0, output_latency_ms=0).at == pytest.approx(100.0)


def test_next_segment_on_the_bar_after_previous_end():
    """Предыдущий кончится в 107.3 → следующий в 108, а не раньше; ровно на такте — тот же такт."""
    assert next_bar(GRID, now=105.0, output_latency_ms=LAT_MS, after=107.3).at == pytest.approx(108.0)
    assert next_bar(GRID, now=105.0, output_latency_ms=LAT_MS, after=108.0).at == pytest.approx(108.0)
    assert bar_at_or_after(GRID, 108.0 + 1e-9) == pytest.approx(108.0)


def test_layout_on_bars_and_back_to_back():
    assert layout([3.0, 1.5, 4.0], GRID, 100.0) == pytest.approx((100.0, 104.0, 106.0))
    assert layout([3.0, 1.5, 4.0], None, 100.0) == pytest.approx((100.0, 103.0, 104.5))
    assert layout([], GRID, 100.0) == ()


def test_grid_of_started_fields():
    assert grid_of({"bpm": 120, "beat_at": 100.0, "bar_beats": 4, "start_beat": 16}) == GRID
    assert grid_of({"bpm": 120, "bar_beats": 4}) is None  # старый плеер без beat_at — чтение без тактов
    assert grid_of({"bpm": "x", "beat_at": 1, "bar_beats": 4}) is None


# --- подложка ------------------------------------------------------------------------------------------------------

def test_backing_attempts_by_mood_and_deadline():
    track = BackingTrack("beat")
    assert track.wanted and track.moods == kn.PERFORM_BACKING_MOODS["beat"]
    first = track.request(10.0, 42.04)
    assert first == Backing(kn.PERFORM_BACKING_MOODS["beat"][0], 42.0)
    assert track.deadline == 10.0 + kn.PERFORM_BACKING_START_S
    assert not track.timed_out(15.9) and track.timed_out(16.0)
    assert track.can_retry()
    second = track.request(11.0, 40.0)
    assert second.mood == kn.PERFORM_BACKING_MOODS["beat"][1]
    assert not track.can_retry()


def test_backing_none_is_not_wanted():
    assert not BackingTrack("none").wanted
    assert BackingTrack("none").mood == ""


def test_failure_reason_buckets():
    for reason, phrase in (("quota", "quota"), ("network", "network"), ("timeout", "timeout"),
                           ("compose_error", "error"), ("renardo_unavailable", "error")):
        track = BackingTrack("beat")
        track.request(0.0, 10.0)
        assert track.fail(reason, 1.0) == phrase


def test_verdicts_follow_the_backing_state():
    track = BackingTrack("beat")
    assert track.verdict("started", "t1", 0.0) == "ignore"  # не просили
    track.request(0.0, 30.0)
    assert track.verdict("nearly_finished", "x", 0.5) == "ignore"
    assert track.verdict("started", "t1", 0.7) == "ignore"  # событие плеера до исхода запроса — не наше (#4085)
    assert track.verdict("rejected", None, 0.5, own=True) == "rejected"
    track.request(0.6, 30.0)  # отказ - вторая попытка: ещё один запрос, ещё один исход в пути
    assert track.verdict("started", "t1", 1.0, own=True) == "started"  # исход НАШЕГО запроса — ответ тула
    track.started("t1", {"bpm": 120, "beat_at": 2.0, "bar_beats": 4})
    assert track.grid is not None
    assert track.verdict("nearly_finished", "t1", 3.0) == "nearly"
    assert track.verdict("nearly_finished", "other", 3.0) == "ignore"
    assert track.verdict("finished", "t1", 3.0) == "ended"
    assert track.verdict("idle", "t1", 3.0) == "ended"
    assert track.verdict("started", "t1", 3.0) == "ignore"  # свой started повторно — не новый старт
    assert track.verdict("started", "users_track", 3.0) == "ended"  # деку заняла другая музыка — тактов нет


def test_late_started_is_ours_only_as_the_outcome_of_our_own_request():
    """Сняли подложку до её started (таймаут, «хватит»): исход нашего запроса (ответ тула) ещё в пути - его поздний
    started наш (снять по id), сколько бы времени ни прошло; второй «исход» без запроса - чужой трек (музыку, которую
    человек поставил потом, выступление не трогает). Хвост ревью PR #4111: окно по времени заменено счётчиком
    запросов в пути."""
    track = BackingTrack("beat")
    track.request(0.0, 30.0)
    track.fail("timeout", 6.0)
    assert track.awaiting == 1
    assert track.verdict("started", "users_track", 7.0) == "ignore"  # чужой трек от плеера — не «поздний» (#4085)
    assert track.awaiting == 1, "событие плеера исходом запроса не считается"
    assert track.verdict("started", "late", 60.0, own=True) == "late"  # поздно - но это исход нашего запроса
    assert track.awaiting == 0
    assert track.verdict("started", "users_track", 61.0, own=True) == "ignore"  # запроса в пути больше нет
    rejected = BackingTrack("beat")
    rejected.request(0.0, 30.0)
    rejected.stopped(1.0)
    assert rejected.verdict("rejected", "pending", 9.0, own=True) == "late"  # поставлен, не встал: отменить по id
    nobody = BackingTrack("beat")
    nobody.request(0.0, 30.0)
    nobody.stopped(1.0)
    assert nobody.verdict("rejected", None, 9.0, own=True) == "ignore"  # отказ без id - снимать нечего
    playing = BackingTrack("beat")
    playing.request(0.0, 30.0)
    playing.started("t1", {})
    playing.stopped(5.0)  # сняли играющую — id известен, чужие треки не наши
    assert playing.verdict("started", "users_track", 5.5) == "ignore"


def test_extension_only_when_speech_outlasts_the_form():
    track = BackingTrack("beat")
    track.request(0.0, 30.0)
    track.started("t1", {"bpm": 120, "beat_at": 100.0, "bar_beats": 4, "start_beat": 16})
    fields = {"form_end_beat": 16 + 64}  # 64 доли = 32 с → конец формы 132
    assert track.extension(120.0, fields, speech_end=127.0, min_s=20.0) is None  # 127 + outro 4 ≤ 132
    ext = track.extension(120.0, fields, speech_end=140.0, min_s=30.04)
    assert ext == Backing(kn.PERFORM_BACKING_MOODS["beat"][0], 30.0, extend=True)


def test_moods_and_backings_are_one_table():
    assert set(kn.PERFORM_BACKING_MOODS) | {"none"} == set(kn.PERFORM_BACKINGS)
    assert all(kn.PERFORM_BACKING_MOODS.values())
