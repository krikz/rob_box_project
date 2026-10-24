"""Сессия с одним владельцем: стек сцен с TTL и set_by, события, снимок только для чтения (ADR-0150 §7.1–7.2)."""

import dataclasses

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import Ask
from rob_box_dialog.session import Session, SpeakerRef


class Clock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now


@pytest.fixture
def clock():
    return Clock()


@pytest.fixture
def session(clock):
    return Session("personality", clock=clock)


def test_restart_is_a_new_session():
    a, b = Session("personality"), Session("personality")
    assert a.snapshot.session_id != b.snapshot.session_id
    assert a.snapshot.epoch == b.snapshot.epoch == 0
    assert len(a.snapshot.session_id) == 32


def test_unknown_agent_or_scene_is_rejected(session):
    with pytest.raises(ValueError):
        Session("tv")
    with pytest.raises(ValueError):
        session.push_scene("Party", "user")
    with pytest.raises(ValueError):
        session.push_scene("Silence", "tv")


def test_snapshot_is_read_only(session):
    with pytest.raises(dataclasses.FrozenInstanceError):
        session.snapshot.epoch = 5  # type: ignore[misc]


def test_user_silence_lives_ten_minutes(session, clock):
    frame = session.push_scene("Silence", "user")
    assert frame.ttl_until == clock.now + kn.SILENCE_TTL_S
    clock.now += kn.SILENCE_TTL_S - 1
    assert session.expire() == ()
    clock.now += 1
    assert session.expire() == (frame,)
    assert session.snapshot.top("Silence") is None


def test_operator_hold_has_no_ttl_and_resume_does_not_lift_user_silence(session):
    user = session.push_scene("Silence", "user")
    hold = session.push_scene("OperatorHold", "operator")
    assert hold.ttl_until is None
    assert session.pop_scene("Silence", set_by="operator") is None
    assert session.snapshot.top("Silence") == user
    assert session.pop_scene("OperatorHold", set_by="operator") == hold
    assert session.pop_scene("Silence") == user
    assert session.pop_scene("Silence") is None


def test_identity_question_expires_with_its_frame(session, clock):
    session.set_pending_ask(Ask(expects="yes_no", key="identity.confirm"))
    session.push_scene("Identity", "code", payload={"person_ref": "spk_1"})
    clock.now += kn.IDENTITY_TTL_S
    (gone,) = session.expire()
    assert gone.payload["person_ref"] == "spk_1"
    assert session.snapshot.pending_ask is None


def test_persona_stack_returns_previous_frame(session):
    session.push_scene("Persona", "code", payload={"name": "robbox"})
    dj = session.push_scene("Persona", "code", payload={"name": "dj"})
    assert session.snapshot.top("Persona") == dj
    session.pop_scene("Persona")
    assert session.snapshot.top("Persona").payload["name"] == "robbox"


def test_reset_bumps_epoch_and_keeps_voice_speaker_music(session):
    session.set_voice(requested="ermil", applied="ermil", provider="yandex")
    session.set_speaker(SpeakerRef(person_ref="spk_1", name="Саша", name_state="confirmed"))
    session.push_scene("Silence", "user")
    session.set_pending_ask(Ask(expects="name", text="Как тебя зовут?"))
    session.reset()
    snap = session.snapshot
    assert snap.epoch == 1 and snap.scenes == () and snap.pending_ask is None
    assert snap.voice.applied == "ermil" and snap.speaker.name == "Саша"


def test_voice_requested_and_applied_are_separate(session):
    session.set_voice(requested="alena")
    session.set_voice(applied="ermil", provider="silero")
    voice = session.snapshot.voice
    assert (voice.requested, voice.applied, voice.provider) == ("alena", "ermil", "silero")


def test_speaker_name_state_is_checked(session):
    with pytest.raises(ValueError):
        session.set_speaker(SpeakerRef(name_state="sure"))
    session.set_speaker(None)
    assert session.snapshot.speaker is None


def test_every_change_is_an_event(session):
    events = []
    session.subscribe(events.append)
    session.push_scene("DJ", "code")
    session.pop_scene("DJ")
    session.pop_scene("DJ")  # нечего снимать — события нет
    session.set_voice(requested="x")
    session.reset()
    assert [e.change for e in events] == ["push_scene", "pop_scene", "set_voice", "reset"]
    assert events[-1].snapshot is session.snapshot
