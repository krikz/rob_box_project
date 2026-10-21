"""Планировщик ресурсов (ADR-0150 §5.6, PR-7b #3699): compose() на всех парах таблицы, Tier-1, занятость по событиям
владельцев, PlanEvent и фраза «Поставлю после …»; эталонные сценарии X4 / X11 / «стой» на фейковых владельцах."""

import pathlib
import re

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.plan.composition import compose, kind_of
from rob_box_dialog.plan.estimate import SpeechRate, remaining_s
from rob_box_dialog.plan.plan_events import PLAN_EVENT_KINDS, PlanEvent, pending_data, queued, queued_phrase, rejected
from rob_box_dialog.plan.resources import ResourceState
from rob_box_dialog.words import policy_word

REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
PLAN_MSG = REPO_ROOT / "src" / "rob_box_dialog_msgs" / "msg" / "PlanEvent.msg"


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


@pytest.fixture
def clock():
    return Clock()


@pytest.fixture
def state(clock):
    return ResourceState(clock)


# --- compose: таблица ----------------------------------------------------------------------------------------------

@pytest.mark.parametrize("new,busy", sorted(kn.COMPOSITION))
def test_every_pair_follows_the_table(new, busy):
    rule = kn.COMPOSITION[(new, busy)]
    decision = compose(new, [busy])
    (cell,) = decision.cells
    assert (cell.busy, cell.policy, cell.boundary, cell.ask, cell.source) == (busy, rule.policy, rule.boundary,
                                                                              rule.ask, "table")
    assert decision.policy == rule.policy


def test_table_is_complete_and_known():
    assert set(kn.COMPOSITION) == {(n, b) for n in kn.COMPOSITION_NEW for b in kn.COMPOSITION_BUSY}
    assert {r.policy for r in kn.COMPOSITION.values()} <= set(kn.POLICIES)
    assert {r.boundary for r in kn.COMPOSITION.values() if r.boundary} <= set(kn.BOUNDARIES)
    assert set(kn.POLICY_ORDER) == set(kn.POLICIES)


def test_face_column_never_conflicts():
    """Столбец без события конца: в таблице только голый MERGE — ResourceState его не ведёт."""
    assert kn.COMPOSITION_NO_END == ("face",)
    assert {kn.COMPOSITION[(n, "face")] for n in kn.COMPOSITION_NEW} == {kn.Rule("MERGE")}


def test_nothing_busy_is_merge():
    decision = compose("music", [])
    assert (decision.policy, decision.cells, decision.waits_for, decision.replaces) == ("MERGE", (), (), ())


def test_unknown_row_or_column_is_loud():
    with pytest.raises(KeyError):
        compose("dance", ["speech"])
    with pytest.raises(KeyError):
        compose("speech", ["kitchen"])


def test_strictest_cell_wins():
    decision = compose("perform", ["speech", "track"])
    assert decision.policy == "REJECT" and decision.ask == "perform.ask_replace_music"
    assert decision.waits_for == ("speech",)
    music = compose("music", ["perform", "track", "speech"])
    assert music.policy == "REPLACE" and music.replaces == ("perform", "track") and music.boundary == "bar"


def test_every_tool_has_a_row_and_rows_occupy_known_columns():
    assert {kind_of(tool) for tool in kn.TOOL_CLASSES} <= set(kn.COMPOSITION_NEW)
    assert set(kn.COMPOSITION_KIND_BY_CLASS) == set(kn.ACTION_CLASSES)
    assert (kind_of("play_sound"), kind_of("play_animation"), kind_of("speak_text")) == ("sfx", "expression", "speech")
    assert (kind_of("request_music"), kind_of("navigate_to_waypoint"), kind_of("memory_save")) == (
        "music", "motion", "query")
    assert set(kn.COMPOSITION_OCCUPIES.values()) <= set(kn.COMPOSITION_BUSY)
    assert set(kn.RESOURCE_BOUNDARY) == set(kn.COMPOSITION_BUSY)


# --- compose: Tier-1 и when ----------------------------------------------------------------------------------------

def test_halt_takes_the_halt_row_whatever_comes():
    decision = compose("speech", ["speech", "motion", "track", "face"], tier1="halt")
    assert decision.replaces == ("speech", "motion", "track")
    assert {c.boundary for c in decision.cells if c.policy == "REPLACE"} == {"now"}
    assert all(c.source == "tier1" for c in decision.cells)


def test_stop_replaces_voice_on_sentence_but_not_motion_nor_plain_merge():
    decision = compose("speech", ["speech", "motion", "track"], tier1="stop")
    cells = {c.busy: c for c in decision.cells}
    assert (cells["speech"].policy, cells["speech"].boundary, cells["speech"].source) == ("REPLACE", "sentence", "tier1")
    assert cells["motion"].policy == "MERGE" and cells["motion"].source == "table"
    assert cells["track"].policy == "MERGE"  # «хватит, расскажи анекдот» музыку не трогает


def test_now_keeps_music_boundary_and_uninterruptible_motion():
    music = compose("music", ["track"], tier1="now")
    assert (music.cells[0].policy, music.cells[0].boundary) == ("REPLACE", "phrase")
    assert compose("motion", ["motion"], tier1="now").cells[0].boundary == "now"
    assert compose("motion", ["motion"], tier1="stop").cells[0].source == "table"


def test_later_appends_after_everything_with_an_end():
    decision = compose("music", ["speech", "face", "motion"], tier1="later")
    cells = {c.busy: c for c in decision.cells}
    assert (cells["speech"].policy, cells["speech"].boundary) == ("APPEND", "block")
    assert cells["motion"].policy == "APPEND"
    assert cells["face"].policy == "MERGE"
    assert decision.waits_for == ("speech", "motion")


def test_tier1_never_answers_the_ask_cell():
    for hint in ("stop", "now", "later"):
        assert compose("perform", ["track"], tier1=hint).policy == "REJECT"


def test_llm_when_now_is_like_tier1_now_and_tier1_wins():
    llm = compose("speech", ["speech"], when="now")
    assert (llm.policy, llm.boundary, llm.cells[0].source) == ("REPLACE", "sentence", "llm")
    assert compose("speech", ["speech"], when="after_current").cells[0].source == "table"
    assert compose("music", ["speech"], tier1="later", when="now").policy == "APPEND"


def test_later_does_not_delay_the_reply_itself():
    """«расскажи анекдот, а потом включи музыку» при играющем треке: реплика не ждёт конца трека."""
    for row in kn.TIER1_LATER_SKIP_ROWS:
        assert compose(row, ["track", "motion"], tier1="later").waits_for == ()


@pytest.mark.parametrize("text,hint", [
    ("Робот, стой", "halt"),
    ("робот стойте", "halt"),
    ("Робот, хватит, расскажи анекдот", "stop"),
    ("робот стоп-стоп", "stop"),
    ("Робот, расскажи анекдот, а потом включи что-нибудь", "later"),
    ("робот, когда закончишь, включи музыку", "later"),
    ("Робот, сейчас спой", "now"),
    ("робот, спой прямо сейчас", "now"),
    ("Робот, а который сейчас час", None),
    ("робот, включи последний трек", None),
    ("робот, спой песню про стоп", None),
    ("робот, направо", None),
])
def test_policy_word(text, hint):
    assert policy_word(text) == hint


# --- занятость по событиям владельцев --------------------------------------------------------------------------------

def test_speech_is_busy_from_owner_queue_to_finished(state, clock):
    """Голос занят, пока у владельца речи есть живая реплика: его очередь (``queued``) — тоже занятость."""
    state.on_speech("queued", "s1", turn_id="t1", epoch=3, text_len=140)
    assert set(state.busy()) == {"speech"}
    state.on_speech("started", "s1", turn_id="t1", epoch=3, text_len=140)
    block = state.busy()["speech"]
    assert (block.block_id, block.turn_id, block.epoch, block.boundary) == ("s1", "t1", 3, "sentence")
    assert state.eta_s("speech") == pytest.approx(140 / kn.SPEECH_CHARS_PER_S_GUESS)
    clock.t += 4
    state.on_speech("progress", "s1", spoken_chars=60, seconds=4.0)
    assert state.rate.measured and state.rate.chars_per_s == pytest.approx(15.0)
    assert state.eta_s("speech") == pytest.approx(80 / 15.0)
    state.on_speech("finished", "s0")  # чужая реплика не освобождает
    assert "speech" in state.busy()
    state.on_speech("finished", "s1", spoken_chars=140, seconds=9.0)
    assert state.busy() == {} and state.eta_s("speech") == 0.0


def test_synthesized_duration_beats_the_rate(state, clock):
    state.on_speech("synthesized", "s1", duration_ms=12000)
    state.on_speech("started", "s1", text_len=30)
    assert state.eta_s("speech") == pytest.approx(12.0)
    clock.t += 5
    state.on_speech("progress", "s1", spoken_chars=10, seconds=5.0)
    assert state.eta_s("speech") == pytest.approx(7.0)


def test_cancelled_speech_frees_voice(state):
    state.on_speech("started", "s1", text_len=50, perform=True)
    assert set(state.busy()) == {"perform"}
    state.on_speech("cancelled", "s1", perform=True)
    assert state.busy() == {}


def test_sound_music_motion(state, clock):
    state.on_sound("started", "r1")
    state.on_sound("skipped", "r2")  # пропуск другого запроса текущий звук не освобождает
    assert state.busy()["sfx"].block_id == "r1" and state.eta_s("sfx") is None
    state.on_sound("finished", "r1")
    state.on_music(True, "trk", remaining_s=30.0)
    state.on_music(True, "trk", remaining_s=20.0)
    assert state.busy()["track"].started_at == 1000.0 and state.eta_s("track") == pytest.approx(20.0)
    state.on_music(True, "bk", backing=True)
    assert set(state.busy()) == {"backing"}
    state.on_music(False)
    state.on_motion(True, "g1", remaining_s=40.0)
    assert state.busy()["motion"].boundary == "now" and state.eta_s("motion") == pytest.approx(40.0)
    state.on_motion(False, "g1")
    assert state.busy() == {}


def test_lost_end_event_does_not_block_forever(state, clock, caplog):
    state.on_speech("started", "s1", text_len=20)
    state.on_music(True, "trk")
    clock.t += kn.RESOURCE_STALE_S + 1
    assert set(state.busy()) == {"track"}  # музыка из переиздаваемого снимка не стареет
    assert "нет события конца" in caplog.text


def test_speech_rate_and_total():
    rate = SpeechRate()
    assert not rate.measured and rate.chars_per_s == kn.SPEECH_CHARS_PER_S_GUESS
    rate.observe(5, 0.5)  # слишком короткий замер
    assert not rate.measured
    rate.observe(100, 10.0)
    rate.observe(200, 10.0)
    assert rate.chars_per_s == pytest.approx(10 + 0.3 * 10)
    assert remaining_s(0, 2500, 0, 1.0, rate) == 1.5 and remaining_s(0, 0, 0, 0.0, rate) is None
    assert remaining_s(130, 0, 0, 0.0, rate) == pytest.approx(10.0)


def test_chunks_of_one_reply_keep_voice_busy_until_the_last(state):
    """speak_text режет реплику на чанки (speech_id на каждый): голос свободен, когда кончился последний."""
    state.on_speech("queued", "c1", text_len=28)
    state.on_speech("queued", "c2", text_len=28)
    state.on_speech("started", "c1", text_len=28)
    assert state.eta_s("speech") == pytest.approx(56 / kn.SPEECH_CHARS_PER_S_GUESS)
    state.on_speech("finished", "c1")
    assert state.busy()["speech"].block_id == "c2"
    state.on_speech("finished", "c2")
    assert state.busy() == {}


def test_reserved_speech_counts_until_the_owner_ends_it(state):
    """Хост отдал реплику (id из ответа speak_text) раньше, чем пришёл ``queued`` владельца — голос уже занят;
    конец, пришедший до ``reserve``, реплику не «оживляет»."""
    state.reserve(["s1"])
    assert "speech" in state.busy() and state.eta_s("speech") is None
    state.on_speech("finished", "s1")
    assert state.busy() == {}
    state.on_speech("finished", "s2")
    state.reserve(["s2"])
    assert state.busy() == {}


# --- PlanEvent ---------------------------------------------------------------------------------------------------

def test_kinds_match_the_message_constants():
    consts = re.findall(r"^uint8 KIND_(\w+)=(\d+)$", PLAN_MSG.read_text(encoding="utf-8"), re.M)
    assert [name.lower() for name, _ in sorted(consts, key=lambda c: int(c[1]))] == list(PLAN_EVENT_KINDS)
    assert PlanEvent("queued", "b").code == int(dict(consts)["QUEUED"])
    with pytest.raises(ValueError):
        PlanEvent("started", "b")


def test_plan_phrases_exist():
    assert [k for k in kn.PLAN_PHRASE_KEYS if k not in phrases.catalog()] == []


def test_nothing_to_wait_for_is_not_queued(state):
    state.on_speech("started", "s1", text_len=50)
    assert queued("b1", compose("motion", state.busy()), state) is None
    assert queued("b1", compose("speech", ["speech"]), ResourceState()) is None  # решение по устаревшему снимку


def test_rejected_carries_the_ask():
    event = rejected("b1", compose("perform", ["track"]), turn_id="t")
    assert (event.kind, event.reason, event.turn_id) == ("rejected", "perform.ask_replace_music", "t")
    assert rejected("b2", compose("sfx", [])).reason == "busy"


# --- эталонные сценарии на фейковых владельцах (§5.6.4) -----------------------------------------------------------

def test_x4_then_sing_a_lullaby_appends_after_the_performance(state):
    state.on_speech("started", "rap-2", turn_id="t1", text_len=280, perform=True)
    decision = compose("perform", state.busy(), tier1=policy_word("Робот, а потом спой колыбельную"))
    assert decision.policy == "APPEND" and decision.waits_for == ("perform",)
    event = queued("perform_2", decision, state, turn_id="t2", epoch=1)
    assert (event.kind, event.after_block, event.after_resource, event.eta_ms) == ("queued", "rap-2", "perform", 20000)
    assert queued_phrase(event, "perform") == "Начну после выступления — примерно через 20 с."
    assert pending_data(event) == {"status": "pending", "after": "perform", "after_block": "rap-2",
                                   "start_in_s": 20.0}


def test_x11_turn_right_during_a_song_runs_in_parallel(state):
    state.on_speech("started", "song-1", text_len=200, perform=True)
    state.on_music(True, "bk", backing=True)
    decision = compose(kind_of("move_direction"), state.busy(), tier1=policy_word("Робот, направо"))
    assert decision.policy == "MERGE" and decision.waits_for == () and decision.replaces == ()


def test_halt_during_speech_and_motion_stops_both_now(state):
    state.on_speech("started", "s1", text_len=100)
    state.on_motion(True, "g1")
    state.on_music(True, "trk")
    decision = compose("speech", state.busy(), tier1=policy_word("Робот, стой"))
    assert set(decision.replaces) == {"speech", "motion", "track"} and decision.boundary == "now"


def test_b_anecdote_then_music_starts_after_speech_with_a_code_phrase(state):
    state.on_speech("started", "joke", text_len=28)  # 2 с по оценке — ETA не называется
    decision = compose(kind_of("request_music"), state.busy(),
                       tier1=policy_word("Робот, расскажи анекдот, а потом включи что-нибудь"))
    event = queued("music_1", decision, state)
    assert event.after_resource == "speech" and event.after_block == "joke"
    assert queued_phrase(event, "music") == "Поставлю после того, как договорю."


def test_c_question_during_speech_waits_for_the_reply_not_cancels(state):
    state.on_speech("started", "long", text_len=200)
    decision = compose("speech", state.busy(), tier1=policy_word("Робот, а который час"))
    assert decision.policy == "MERGE" and decision.waits_for == ("speech",) and decision.replaces == ()


def test_e_sound_on_busy_sfx_is_appended_not_silently_dropped(state):
    state.on_sound("started", "r1")
    decision = compose(kind_of("play_sound"), state.busy())
    assert decision.policy == "APPEND" and decision.waits_for == ("sfx",)
    event = queued("r2", decision, state)
    assert event.eta_ms == 0 and queued_phrase(event, "sfx") == "Сыграю звук после этого звука."
