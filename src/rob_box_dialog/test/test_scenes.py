"""Кадры Silence/Persona/DJ/OperatorHold по событиям владельцев (ADR-0150 §6.3, §7.2; приёмка A9, A10, PR-10 #3704)."""

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import HELD, NO_WAKE, NotAddressed, decide
from rob_box_dialog.backlog import Heard
from rob_box_dialog.scenes import HOLD, RESET, RESUME, SceneDirector, apply_control
from rob_box_dialog.session import Session

DEFAULT = ("yandex", "anton")


class Clock:
    now = 1000.0

    def __call__(self):
        return self.now


@pytest.fixture
def clock():
    return Clock()


@pytest.fixture
def session(clock):
    return Session("personality", clock=clock)


@pytest.fixture
def restores():
    return []


@pytest.fixture
def director(session, restores):
    return SceneDirector(session, lambda provider, voice: restores.append((provider, voice)), DEFAULT)


def _set_voice(session, provider, voice):
    session.set_voice(requested=voice, applied=voice, provider=provider)


SET = {"set_id": "s1", "persona": "Дядя Стёпа"}


# -- Silence: A9 --------------------------------------------------------------------------------------------------

def test_user_silence_ends_by_itself_after_ten_minutes(session, clock):
    session.push_scene("Silence", "user")
    clock.now += kn.SILENCE_TTL_S - 1
    assert session.expire() == () and session.snapshot.top("Silence") is not None
    clock.now += 1
    assert [f.kind for f in session.expire()] == ["Silence"]
    assert session.snapshot.top("Silence") is None


def test_operator_resume_does_not_lift_user_silence_and_vice_versa(session):
    session.push_scene("Silence", "user")
    assert apply_control(session, HOLD, "operator", 60.0)
    assert apply_control(session, RESUME, "operator")
    assert session.snapshot.top("Silence") is not None and session.snapshot.top("OperatorHold") is None
    assert apply_control(session, HOLD, "tars")
    session.pop_scene("Silence", "user")  # «говори»
    assert session.snapshot.top("OperatorHold") is not None


def test_hold_twice_does_not_stack_and_one_resume_frees(session):
    assert apply_control(session, HOLD, "operator", 10.0) and apply_control(session, HOLD, "operator", 20.0)
    assert [f.kind for f in session.snapshot.scenes] == ["OperatorHold"]
    apply_control(session, RESUME, "operator")
    assert session.snapshot.scenes == ()


def test_hold_expires_by_ttl_and_without_ttl_lives_until_resume(session, clock):
    apply_control(session, HOLD, "operator", 30.0)
    clock.now += 31
    assert [f.kind for f in session.expire()] == ["OperatorHold"]
    apply_control(session, HOLD, "operator", 0.0)
    clock.now += 100000
    assert session.expire() == () and session.snapshot.top("OperatorHold") is not None


@pytest.mark.parametrize("set_by,action", [("user", HOLD), ("code", RESUME), ("", RESET), ("operator", "dance")])
def test_foreign_set_by_or_unknown_action_is_refused_and_changes_nothing(session, set_by, action):
    before = session.snapshot
    assert apply_control(session, action, set_by) is False
    assert session.snapshot is before


def test_reset_through_control_pops_everything_and_bumps_epoch(session):
    session.push_scene("Silence", "user")
    apply_control(session, HOLD, "operator")
    assert apply_control(session, RESET, "operator")
    assert session.snapshot.scenes == () and session.snapshot.epoch == 1


def test_operator_hold_makes_the_voice_unaddressed_but_not_telegram():
    session = Session("personality")
    session.push_scene("OperatorHold", "operator")
    hold = session.snapshot.top("OperatorHold")
    voice = Heard(text="робот, который час", t=1.0, source="respeaker")
    assert decide(voice, None, ["робот"], hold=hold) == NotAddressed(HELD)
    assert decide(voice, None, ["робот"]).text == "который час"
    telegram = Heard(text="робот, который час", t=1.0, source="telegram")
    assert decide(telegram, None, ["робот"], hold=hold).text == "который час"
    assert HELD != NO_WAKE


# -- DJ и Persona: A10 --------------------------------------------------------------------------------------------

def test_set_start_pushes_dj_and_persona_and_idle_pops_both(session, director):
    director.on_music(True, SET)
    assert session.snapshot.top("DJ").payload["set_id"] == "s1"
    assert session.snapshot.top("Persona").payload["name"] == "Дядя Стёпа"
    director.on_music(False, {})
    assert session.snapshot.scenes == ()


def test_same_set_state_repeated_does_not_pile_frames_up(session, director):
    for _ in range(3):
        director.on_music(True, SET)
    assert [f.kind for f in session.snapshot.scenes] == ["DJ", "Persona"]


def test_replaced_set_changes_dj_but_keeps_first_voice_to_return_to(session, director, restores):
    _set_voice(session, *DEFAULT[:1], "kirill")
    director.on_music(True, SET)
    _set_voice(session, "yandex", "alena")
    director.on_music(True, {"set_id": "s2"})
    assert session.snapshot.top("DJ").payload["set_id"] == "s2"
    assert session.snapshot.top("Persona").payload["voice"] == "kirill"
    director.on_music(False, {})
    assert restores == [("yandex", "kirill")]


def test_voice_changed_during_set_is_given_back_when_set_ends(session, director, restores):
    _set_voice(session, "minimax", "Reliable_Man")
    director.on_music(True, SET)
    _set_voice(session, "yandex", "alena")  # LLM сета сменила голос (акт 4 марафона)
    director.on_music(False, {})
    assert restores == [("minimax", "Reliable_Man")]


def test_unchanged_voice_is_not_requested_again(session, director, restores):
    _set_voice(session, "minimax", "Reliable_Man")
    director.on_music(True, SET)
    director.on_music(False, {})
    assert restores == []


def test_unknown_voice_before_set_returns_to_robot_default(session, director, restores):
    director.on_music(True, SET)
    _set_voice(session, "yandex", "alena")
    director.on_music(False, {})
    assert restores == [DEFAULT]


def test_reset_returns_default_voice_and_drops_persona(session, director, restores):
    director.on_music(True, SET)
    _set_voice(session, "yandex", "alena")
    session.reset()
    assert session.snapshot.scenes == () and restores == [DEFAULT]


def test_reset_with_default_voice_asks_nothing(session, director, restores):
    _set_voice(session, *DEFAULT)
    session.reset()
    session.reset()
    assert restores == []


def test_persona_ttl_or_explicit_pop_also_gives_the_voice_back(session, director, restores):
    _set_voice(session, "minimax", "Reliable_Man")
    director.on_music(True, SET)
    _set_voice(session, "yandex", "alena")
    session.pop_scene("Persona")
    assert restores == [("minimax", "Reliable_Man")]
    assert session.snapshot.top("DJ") is not None  # персона снята явной командой, сет играет дальше
