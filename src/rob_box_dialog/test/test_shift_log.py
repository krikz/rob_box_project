"""#4076: журнал смены — приходы, треки, исходы действий, выступления, тишина; фраза-итог из записей кода.

Марафон 27.10, n1001 «подведи итог ночи: кто к нам приходил, что мы слушали»: источника фактов смены не было, и модель
сказала «никто, кроме тебя, не подходил, музыку мы не слушали». Здесь — поведение журнала: события → итог их содержит;
событий нет → честная фраза кода; окно времени и кольцо; раздел снимка только на вопрос о смене.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping, Optional

import pytest

from rob_box_core.token_estimate import estimate_tokens
from rob_box_dialog import knowledge as kn
from rob_box_dialog import shift_log as sl
from rob_box_dialog.context import build_context
from rob_box_dialog.music_state import MusicStateMemory
from rob_box_dialog.respond import ActionResult, asks, failed
from rob_box_dialog.session import Session, SpeakerRef
from rob_box_dialog.shift_log import (ACTION, FACE, ONGOING, PERFORM, SECTION, VISIT, ShiftLog, shift_asked,
                                      span_words, summary)
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
    assert view["unnamed"] == {"voice": 1}
    assert view["music"] == [{"track": "Песня про енота", "min_ago": 10, "ended": "stopped", "source": "synth",
                              "played_min": 3}]
    assert view["actions"] == {"request_music": {"done": 1}, "dj_set": {"failed": 1}}
    assert view["performances"] == [{"kind": "сказка", "status": "done", "min_ago": 7}]
    assert view["silence"] == {"times": 1, "total_min": 5, "last_min_ago": 7}
    assert view["covers_min"] == 21
    assert view["summary"] == (
        "Журнал смены за последние 21 мин: узнал: Борис, Гриша; появлений без подтверждённого имени (не число людей): "
        "1; звучало треков: 1, последний — «Песня про енота»; выступления: сказка; действия — выполнено: 1, не вышло: "
        "1; тишину просили: 1.")


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
def test_unconfirmed_voice_is_an_unnamed_appearance_not_a_name_a30(state):
    """Гипотеза, спор голоса и неузнанный — «без подтверждённого имени», не имя (A30) и не «незнакомец»: спор —
    голос, похожий на знакомого (``speaker_adapter``)."""
    clock = _Clock()
    log, session = _journal(clock)
    session.set_speaker(SpeakerRef(person_ref="spk-boris" if state == "tentative" else None, name="Борис",
                                   name_state=state))
    view = log.view()
    assert "people" not in view and view["unnamed"] == {"voice": 1}
    assert "Борис" not in view["summary"] and "незнаком" not in view["summary"]
    assert "без подтверждённого имени" in view["summary"] and "не число людей" in view["summary"]


def test_unnamed_voices_of_one_channel_within_the_gap_are_one_appearance():
    """Два неопознанных голоса подряд — одно появление, а не «двое незнакомцев»: журнал не умеет их различить."""
    clock = _Clock()
    log, session = _journal(clock)
    session.set_speaker(SpeakerRef(name_state="unknown"))
    clock.skip(2)
    session.set_speaker(SpeakerRef(name_state="contested", name="Борис"))
    assert log.view()["unnamed"] == {"voice": 1}


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
    assert log.view()["actions"] == {"request_music": {"failed": 1}}
    turns.reset()  # ход выпал из окна (новая сессия) — журнал смены всё равно получает поздний итог
    turns.late(_result("turn-1/0", "request_music", "done"), lambda before: None)
    assert [(e.kind, e.what, e.status) for e in log.events()] == [(ACTION, "request_music", "done")]


def test_question_of_a_tool_is_not_a_failure_and_every_recorded_status_is_in_the_summary():
    """Ревью PR #4110: «провал» — одно понятие (``respond.failed``/``asks``). Отказ-вопрос ``music_library`` («Удалить
    «X»?», ``needs_confirmation``, #4062) — не «не вышло», а «спросил подтверждения»; ``cancelled`` движения — в итоге
    «отменено», а не пропадает между «выполнено» и «не вышло»."""
    clock = _Clock()
    log, _session = _journal(clock)
    turns = TurnLog(on_outcome=log.outcome)
    question = _result("turn-1/0", "music_library", "rejected", cls="action.local", reason="needs_confirmation")
    cancelled = _result("turn-1/1", "navigate_to_waypoint", "cancelled", cls="action.motion")
    assert asks(question) and not failed(question) and not failed(cancelled)  # правило respond, которому следует журнал
    turns.begin("turn-1", 0, "удали трек и поезжай на кухню")
    turns.result("turn-1", question)
    turns.result("turn-1", cancelled)
    clock.skip(10)
    view = log.view()
    assert view["actions"] == {"music_library": {"asked": 1}, "navigate_to_waypoint": {"cancelled": 1}}
    assert view["summary"] == "Журнал смены за последние 10 мин: действия — спросил подтверждения: 1, отменено: 1."
    assert "не вышло" not in view["summary"] and "выполнено" not in view["summary"]


def test_summary_counts_done_failed_question_and_other_outcomes_each_once():
    clock = _Clock()
    log = ShiftLog(clock=clock)
    log.outcome(_result("t1/0", "request_music", "done"))
    log.outcome(_result("t2/0", "dj_set", "timeout"))
    log.outcome(_result("t3/0", "music_library", "rejected", cls="action.local", reason="needs_confirmation"))
    log.outcome(_result("t4/0", "move_direction", "cancelled", cls="action.motion"))
    log.outcome(_result("t5/0", "set_volume", "rejected", cls="action.local", reason="busy"))
    clock.skip(3)
    assert log.view()["summary"] == ("Журнал смены за последние 3 мин: действия — выполнено: 1, не вышло: 2, "
                                     "спросил подтверждения: 1, отменено: 1.")


def test_the_same_call_id_in_another_iteration_of_the_turn_is_another_call():
    """Итерации хода нумеруют вызовы заново (``<ход>/0`` дважды): другой тул под тем же id — новая запись, а не замена
    (ревью PR #4110: ``request_music`` пропадал под ``dj_set``); замена — только поздний итог после ``timeout``."""
    clock = _Clock()
    log = ShiftLog(clock=clock)
    log.outcome(_result("turn-1/0", "request_music", "done"))
    log.outcome(_result("turn-1/0", "dj_set", "rejected", reason="busy"))
    log.outcome(_result("turn-1/0", "dj_set", "done"))  # не после timeout — тоже другой вызов
    assert [(e.what, e.status) for e in log.events()] == [("request_music", "done"), ("dj_set", "rejected"),
                                                          ("dj_set", "done")]


def test_end_of_a_performance_closes_the_shift_record_even_after_the_turn_left_the_window():
    """Ревью PR #4110: «забудь всё» посреди сказки (``TurnLog.reset``) — конец выступления всё равно доходит до журнала
    смены, запись не висит «идёт» вечно."""
    clock = _Clock()
    log, _session = _journal(clock)
    turns = TurnLog(on_outcome=log.outcome)
    turns.begin("turn-1", 0, "расскажи сказку")
    turns.performed("turn-1", "сказка", PERFORM_STARTED)
    turns.reset()
    assert turns.performance_ended("turn-1", "done", segments_done=3) is False  # в окне хода уже нет
    assert [(e.kind, e.what, e.status) for e in log.events()] == [(PERFORM, "сказка", "done")]


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


def test_repeated_snapshot_of_the_same_track_refines_its_name_and_source():
    """Владелец плеера публикует снимок до слушателя старта (``player_owner.track_started``): имя мелодии сета и
    источник приходят повторным снимком того же ``track_id`` — журнал берёт их, а не замораживает общий заголовок."""
    clock = _Clock()
    log, _session = _journal(clock)
    memory = MusicStateMemory()
    _play(log, memory, _Player("playing", "t1", clock(), {"title": "ночной сет · трек 1 из 3"}))
    clock.skip(1)
    _play(log, memory, _Player("playing", "t1", clock(), {"title": "ночной сет · трек 1 из 3", "name": "Енот в ночи",
                                                          "source": "generated"}))
    clock.skip(1)
    _play(log, memory, _Player("playing", "t1", clock(), {"title": "ночной сет · трек 1 из 3"}))  # имя не теряется
    assert [(m["track"], m.get("source")) for m in log.view()["music"]] == [("Енот в ночи", "generated")]


def test_a_failing_journal_does_not_break_the_owner_callback(monkeypatch):
    """Журнал — наблюдатель: его сбой не ломает ``Session.set_speaker`` для остальных подписчиков, колбэк плеера и
    журнал ходов (ревью PR #4110)."""
    def broken(_ref):
        raise RuntimeError("журнал сломан")

    monkeypatch.setattr(sl, "speaker_block", broken)
    clock = _Clock()
    log, session = _journal(clock)
    heard = []
    session.subscribe(heard.append)
    session.set_speaker(_confirmed("Борис", "spk-boris"))
    log.music(object())  # снимок без полей плеера
    log.outcome(object())  # исход без полей ``ActionResult``
    assert len(heard) == 1 and log.events() == []


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
    # ревью PR #4110: ложные отрицания — между вопросом и глаголом встают «ещё», «сегодня», «к нам», «интересного»
    "Кто к нам пришёл?", "Кто ещё был?", "Кто сегодня был в гостях", "Что интересного было сегодня?", "Что ты видел?",
    "Робот, а что ты делал ночью?", "расскажи про смену",
])
def test_section_goes_to_the_model_on_a_question_about_the_shift(text):
    clock = _Clock()
    log, session = _journal(clock)
    session.set_speaker(_confirmed("Борис", "spk-boris"))
    assert shift_asked(text)
    sections = log.section(text)
    assert list(sections) == [SECTION] and "Борис" in sections[SECTION]["summary"]
    assert "<shift_log>" in build_context(sections)  # раздел из таблицы ``CONTEXT_SECTIONS``


@pytest.mark.parametrize("text", [
    "включи ночной билд", "какая погода", "спой песню про енота", "сколько заряда",
    # ревью PR #4110: прощание — не вопрос о смене (марафон n1011), «сказка на ночь» — не вопрос о ночи
    "Робот, спокойной ночи, смена окончена, спасибо за компанию.", "доброй ночи", "расскажи сказку на ночь",
    "Робот, скажи по-китайски «всем спокойной ночи».", "Робот, о чём мы с тобой только что говорили?",
    "что сейчас играет?",
])
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


NAMES = ("Александра", "Константин", "Борис", "Григорий", "Валентина", "Станислав", "Екатерина", "Владимир",
         "Маргарита")


def test_heavy_night_section_stays_within_sixty_percent_of_the_snapshot_budget():
    """Раздел живёт в бюджете снимка (``CONTEXT_BUDGET_TOKENS["snapshot"]``) рядом с другими разделами. Тяжёлая ночь из
    ревью PR #4110 (9 узнанных, 8 появлений без имени, 10 треков, 24 исхода разных тулов, выступления, тишина) - не
    больше 60 % бюджета: на этих данных 552 оценочных токена при 4 строках списков и 678 при прежних 6."""
    clock = _Clock()
    log, session = _journal(clock)
    tools = [t for t, c in kn.TOOL_CLASSES.items() if c not in ("query", "action.perform")
             and t not in kn.SHIFT_SKIP_TOOLS]
    for n in range(24):
        if n < len(NAMES):
            log.seen(NAMES[n])
        if n < 8:
            log.seen(None, FACE if n % 2 else sl.VOICE)
        log.outcome(_result(f"t{n}/0", tools[n % len(tools)], ("done", "rejected", "cancelled")[n % 3],
                            reason="busy" if n % 3 == 1 else None))
        if n % 6 == 0:
            log.outcome(ActionResult(f"t{n}/perform", 0, "", "perform", "action.perform", "done",
                                     data={"kind": kn.PERFORM_KINDS[n % len(kn.PERFORM_KINDS)]}))
            session.push_scene("Silence", "user")
        else:
            session.pop_scene("Silence", "user")
        if n < 10:
            track = {"name": f"Ночной сет про енота и луну, мелодия {n}", "source": "generated"}
            log.music(_Player("playing", f"tr{n}", clock(), track))
        clock.skip(kn.SHIFT_VISIT_GAP_S / 60 + 1)
    view = log.view()
    tokens = estimate_tokens(build_context(log.section("подведи итог ночи")))
    assert tokens <= kn.CONTEXT_BUDGET_TOKENS["snapshot"] * 6 // 10, tokens
    assert all(len(view[k]) <= kn.SHIFT_SECTION_MAX_ITEMS for k in ("people", "music", "performances", "actions"))
    assert "Маргарита" in view["summary"] and "Александра" in view["summary"]  # итог называет всех, списки - новых


@pytest.mark.parametrize("seconds,words", [(0, "1 мин"), (59 * 60, "59 мин"), (3600, "1 ч"), (200 * 60, "3 ч 20 мин")])
def test_span_words(seconds, words):
    assert span_words(seconds) == words
