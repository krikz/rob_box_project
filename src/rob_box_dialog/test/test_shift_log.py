"""#4076: журнал смены — приходы, треки, исходы действий, выступления, тишина; фраза-итог из записей кода.

Марафон 27.10, n1001 «подведи итог ночи: кто к нам приходил, что мы слушали»: источника фактов смены не было, и модель
сказала «никто, кроме тебя, не подходил, музыку мы не слушали». Здесь — поведение журнала: события → итог их содержит;
событий нет → честная фраза кода; окно времени и кольцо; раздел снимка только на вопрос о смене.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping, Optional

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.context import build_context
from rob_box_dialog.music_state import MusicStateMemory
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session, SpeakerRef
from rob_box_dialog.shift_log import (ACTION, FACE, ONGOING, SECTION, VISIT, ShiftLog, shift_asked, span_words,
                                      summary)
from rob_box_dialog.turn_log import PERFORM_STARTED, TurnLog

T0 = 1_800_000_000.0


class _Clock:
    def __init__(self, t: float = T0) -> None:
        self.t = t

    def __call__(self) -> float:
        return self.t

    def skip(self, minutes: float) -> float:
        self.t += minutes * 60.0
        return self.t


@dataclass(frozen=True)
class _Player:
    """Снимок ``/voice/music/state`` (поля ``MusicPlayerState``, которые читают журнал и ``MusicStateMemory``)."""

    state: str
    track_id: Optional[str] = None
    ts: Optional[float] = None
    dj_info: Mapping[str, Any] = field(default_factory=dict)
    stops_at: Optional[float] = None
    finished_track_id: Optional[str] = None
    dj: bool = False

    def is_playing(self, now: Optional[float] = None) -> bool:
        return self.state == "playing"


def _journal(clock: _Clock):
    log = ShiftLog(clock=clock)
    session = Session("personality", clock=clock)
    session.subscribe(log.observe)
    return log, session


def _confirmed(name: str, ref: str) -> SpeakerRef:
    return SpeakerRef(person_ref=ref, name=name, name_state="confirmed")


def _play(log: ShiftLog, memory: MusicStateMemory, snap: _Player) -> None:
    """Как ``dialog_node._on_music_state``: память плеера, затем журнал с её ``last_ended``."""
    memory.observe_state(snap)
    log.music(snap, memory.last_ended)


def _result(ref: str, tool: str, status: str, cls: str = "action.media", reason: Optional[str] = None):
    return ActionResult(ref, 0, "personality", tool, cls, status, reason)


def test_events_of_the_night_are_in_the_summary_and_in_the_section():
    clock = _Clock()
    log, session = _journal(clock)
    memory, turns = MusicStateMemory(), TurnLog(on_outcome=log.outcome)
    clock.skip(10)
    session.set_speaker(_confirmed("Борис", "spk-boris"))
    clock.skip(1)
    log.seen("Гриша", FACE)
    session.set_speaker(SpeakerRef(name_state="unknown"))
    _play(log, memory, _Player("playing", "t1", clock(), {"name": "Песня про енота", "source": "synth"}))
    clock.skip(3)
    _play(log, memory, _Player("idle", None, clock()))
    turns.begin("turn-1", 0, "расскажи сказку и включи музыку")
    turns.result("turn-1", _result("turn-1/0", "request_music", "done"))
    turns.result("turn-1", _result("turn-1/1", "dj_set", "rejected", reason="busy"))
    turns.performed("turn-1", "сказка", PERFORM_STARTED)
    turns.performance_ended("turn-1", "done", segments_done=4)
    session.push_scene("Silence", "user")
    clock.skip(5)
    session.pop_scene("Silence", "user")
    clock.skip(2)

    view = log.view()

    assert [(p["name"], p["via"]) for p in view["people"]] == [("Гриша", "face"), ("Борис", "voice")]
    assert view["unknown"] == {"voice": 1}
    assert view["music"] == [{"track": "Песня про енота", "min_ago": 10, "ended": "stopped", "source": "synth",
                              "played_min": 3}]
    assert view["actions"] == {"request_music": {"done": 1}, "dj_set": {"rejected": 1}}
    assert view["performances"] == [{"kind": "сказка", "status": "done", "min_ago": 7}]
    assert view["silence"] == [{"min_ago": 7, "for_min": 5, "ended": "ended"}]
    assert view["covers_min"] == 21
    assert view["summary"] == (
        "Журнал смены за последние 21 мин: узнал: Борис, Гриша; незнакомых приходов: 1; звучало треков: 1, "
        "последний — «Песня про енота»; выступления: сказка; действий выполнено: 1, не вышло: 1; тишину просили: 1.")


def test_empty_journal_says_honestly_that_nothing_is_recorded():
    """Событий нет — фраза кода «ничего не записано» за то время, что журнал ведётся, а не «никто не приходил»."""
    clock = _Clock()
    log, _session = _journal(clock)
    clock.skip(25)
    assert log.view() == {"summary": "В журнале смены за последние 25 мин ничего не записано.", "covers_min": 25}
    assert summary([], 0.0) == "В журнале смены за последние 1 мин ничего не записано."


def test_window_drops_events_older_than_the_shift_and_covers_at_most_the_window():
    clock = _Clock()
    log, session = _journal(clock)
    session.set_speaker(_confirmed("Борис", "spk-boris"))
    clock.skip(kn.SHIFT_LOG_WINDOW_S / 60 + 30)
    session.set_speaker(_confirmed("Нина", "spk-nina"))
    view = log.view()
    assert [p["name"] for p in view["people"]] == ["Нина"]
    assert view["covers_min"] == int(kn.SHIFT_LOG_WINDOW_S / 60)
    assert "Борис" not in view["summary"]


def test_restart_journal_covers_only_the_time_since_start():
    """Рестарт процесса обнуляет журнал — и раздел говорит, что покрывает только время с рестарта."""
    clock = _Clock()
    log, session = _journal(clock)
    clock.skip(5)
    session.set_speaker(_confirmed("Борис", "spk-boris"))
    assert log.view()["summary"] == "Журнал смены за последние 5 мин: узнал: Борис."


def test_same_person_within_the_gap_is_one_visit_after_the_gap_a_new_one():
    clock = _Clock()
    log, session = _journal(clock)
    session.set_speaker(_confirmed("Борис", "spk-boris"))
    clock.skip(kn.SHIFT_VISIT_GAP_S / 60 - 1)
    session.set_speaker(_confirmed("Борис", "spk-boris"))
    log.seen("борис", FACE)  # лицо того же человека — тот же приход
    assert [(e.kind, e.detail) for e in log.events()] == [(VISIT, "voice,face")]
    clock.skip(kn.SHIFT_VISIT_GAP_S / 60 + 1)
    session.set_speaker(_confirmed("Борис", "spk-boris"))
    clock.skip(1)
    assert log.view()["people"] == [{"name": "Борис", "visits": 2, "first_min_ago": 31, "last_min_ago": 1,
                                     "via": "face,voice"}]


@pytest.mark.parametrize("state", ["tentative", "contested", "unknown"])
def test_unconfirmed_voice_is_an_unknown_visit_not_a_name_a30(state):
    clock = _Clock()
    log, session = _journal(clock)
    session.set_speaker(SpeakerRef(person_ref="spk-boris" if state == "tentative" else None, name="Борис",
                                   name_state=state))
    view = log.view()
    assert "people" not in view and view["unknown"] == {"voice": 1}
    assert "Борис" not in view["summary"]


def test_new_session_of_the_dialog_does_not_clear_the_shift():
    """«Забудь всё» (``Session.reset``, ``TurnLog.reset``) — новая сессия разговора, журнал смены остаётся."""
    clock = _Clock()
    log, session = _journal(clock)
    turns = TurnLog(on_outcome=log.outcome)
    session.set_speaker(_confirmed("Борис", "spk-boris"))
    turns.begin("turn-1", 0, "включи музыку")
    turns.result("turn-1", _result("turn-1/0", "request_music", "done"))
    session.reset()
    turns.reset()
    view = log.view()
    assert [p["name"] for p in view["people"]] == ["Борис"] and view["actions"] == {"request_music": {"done": 1}}


def test_late_outcome_replaces_the_action_in_place_even_after_the_window_dropped_it():
    clock = _Clock()
    log, _session = _journal(clock)
    turns = TurnLog(on_outcome=log.outcome)
    turns.begin("turn-1", 0, "включи музыку")
    turns.result("turn-1", _result("turn-1/0", "request_music", "timeout"))
    assert log.view()["actions"] == {"request_music": {"timeout": 1}}
    turns.reset()  # ход выпал из окна (новая сессия) — журнал смены всё равно получает поздний итог
    turns.late(_result("turn-1/0", "request_music", "done"), lambda before: None)
    assert [(e.kind, e.what, e.status) for e in log.events()] == [(ACTION, "request_music", "done")]


def test_reads_promises_and_animations_are_not_events_of_the_shift():
    clock = _Clock()
    log, _session = _journal(clock)
    turns = TurnLog(on_outcome=log.outcome)
    turns.begin("turn-1", 0, "поезжай на кухню и покажи радость")
    turns.result("turn-1", _result("turn-1/0", "memory_search", "done", cls="query"))
    turns.result("turn-1", _result("turn-1/1", "music_library", "done", cls="query"))  # list — чтение по аргументу
    turns.result("turn-1", _result("turn-1/2", "play_animation", "done", cls="action.local"))
    turns.result("turn-1", _result("turn-1/3", "navigate_to_waypoint", "pending", cls="action.motion"))
    assert log.events() == []
    turns.late(_result("turn-1/3", "navigate_to_waypoint", "done", cls="action.motion"), lambda before: None)
    assert log.view()["actions"] == {"navigate_to_waypoint": {"done": 1}}


def test_tracks_replaced_and_finished_by_the_player_rule():
    """Как трек кончился — правило ``MusicStateMemory`` (сменён, доиграл, остановлен), название — из снимка плеера."""
    clock = _Clock()
    log, _session = _journal(clock)
    memory = MusicStateMemory()
    _play(log, memory, _Player("playing", "t1", clock(), {"title": "ночной сет · трек 1 из 3", "source": "generated"}))
    clock.skip(4)
    _play(log, memory, _Player("playing", "t2", clock(), {"name": "Очень странные дела", "engine": "renardo"}))
    clock.skip(2)
    _play(log, memory, _Player("idle", None, clock(), finished_track_id="t2"))
    clock.skip(1)
    _play(log, memory, _Player("playing", "t3", clock(), {}))
    music = log.view()["music"]
    assert [(m["track"], m["ended"], m.get("source"), m.get("played_min")) for m in music] == [
        ("ночной сет · трек 1 из 3", "replaced", "generated", 4),
        ("Очень странные дела", "finished", "renardo", 2),
        ("без названия", ONGOING, None, None),
    ]


@pytest.mark.parametrize("text", [
    "подведи итог ночи: кто к нам приходил, что мы слушали и что ты про нас запомнил",
    "кто сегодня заходил?", "что было, пока меня не было", "чем ты занимался за смену", "что вы слушали",
    "как прошла ночь",
])
def test_section_goes_to_the_model_on_a_question_about_the_shift(text):
    clock = _Clock()
    log, session = _journal(clock)
    session.set_speaker(_confirmed("Борис", "spk-boris"))
    assert shift_asked(text)
    sections = log.section(text)
    assert list(sections) == [SECTION] and "Борис" in sections[SECTION]["summary"]
    assert "<shift_log>" in build_context(sections)  # раздел из таблицы ``CONTEXT_SECTIONS``


@pytest.mark.parametrize("text", ["включи ночной билд", "какая погода", "спой песню про енота", "сколько заряда"])
def test_other_turns_do_not_carry_the_journal(text):
    log, _session = _journal(_Clock())
    assert not shift_asked(text)
    assert log.section(text) == {}


def test_ring_keeps_at_most_max_events_newest_first_out():
    clock = _Clock()
    log = ShiftLog(clock=clock, max_events=3)
    for name in ("Борис", "Гриша", "Нина", "Тамара", "Валера"):
        log.seen(name)
        clock.skip(1)
    assert [e.who for e in log.events()] == ["Нина", "Тамара", "Валера"]


def test_section_lists_at_most_the_newest_items_but_the_summary_counts_all():
    clock = _Clock()
    log, _session = _journal(clock)
    memory = MusicStateMemory()
    for n in range(kn.SHIFT_SECTION_MAX_ITEMS + 2):
        _play(log, memory, _Player("playing", f"t{n}", clock(), {"name": f"трек {n}"}))
        clock.skip(1)
    view = log.view()
    assert len(view["music"]) == kn.SHIFT_SECTION_MAX_ITEMS and view["music"][-1]["track"] == f"трек {n}"
    assert f"звучало треков: {n + 1}, последний — «трек {n}»" in view["summary"]


@pytest.mark.parametrize("seconds,words", [(0, "1 мин"), (59 * 60, "59 мин"), (3600, "1 ч"), (200 * 60, "3 ч 20 мин")])
def test_span_words(seconds, words):
    assert span_words(seconds) == words
