"""A29 (ADR-0150 §7.5): вопрос об имени — счётчик в коде, не в промпте.

≤ 1 «Как тебя зовут?» / «это ты?» на собеседника за сессию; 0 — в ходе с действием. Число 1 — из ADR.
"""

import asyncio

from rob_box_dialog import identity_ask as ia
from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Ask, Command, ToolCall
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session, SpeakerRef
from rob_box_dialog.turn import Answer, TurnEngine

NEW = SpeakerRef(person_ref="spk-new", name=None, name_state="unknown")
OTHER = SpeakerRef(person_ref="spk-other", name=None, name_state="unknown")
TRACKED = SpeakerRef(track_ref="trk-1", name=None, name_state="unknown")
ANIM = ToolCall("play_animation", {"animation": "happy"})
NAME_Q = Command(speech="Как тебя зовут?")
THIS_YOU = Command(speech="Саша, это ты?")
ASK_NAME = Command(ask=Ask("name", "Как тебя зовут?"))


class Ports:
    def __init__(self, answers):
        self.answers, self.asked, self.said, self.executed = list(answers), [], [], []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append(list(messages))
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        self.executed.append([(a.tool, dict(a.args)) for a in actions])
        return [
            ActionResult(
                f"{turn_id}/{n}",
                snapshot.epoch,
                snapshot.agent,
                a.tool,
                kn.TOOL_CLASSES[a.tool],
                "done",
                None,
                dict(a.args),
            )
            for n, a in enumerate(actions)
        ]

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append(text)
        return f"sp{len(self.said)}"

    def stage(self, *args, **kwargs):
        pass

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        pass

    def stop_perform(self, how):
        return False

    def intent(self, turn_id, epoch, kind):
        pass

    def perform(self, turn_id, epoch, perf, edit):
        return None


def _run(answers, speaker=NEW, session=None):
    session = session or Session("personality")
    session.set_speaker(speaker)
    ports = Ports([Answer(cmd, "deepseek") for cmd in answers])
    asyncio.run(TurnEngine(session, ports, system_prompt="Ты робот.").run(Addressed("робот привет", VOICE)))
    return ports, session


def test_max_is_one_from_adr():
    assert ia.MAX_PER_INTERLOCUTOR == 1


def test_both_a29_phrases_are_identity_questions():
    assert ia.asks(NAME_Q) and ia.asks(THIS_YOU) and ia.asks(ASK_NAME)
    assert not ia.asks(Command(speech="Привет."))
    assert not ia.asks(Command(ask=Ask("yes_no", "Выключить и зачитать?")))


def test_key_prefers_person_ref_then_track_ref():
    both = SpeakerRef(person_ref="p1", track_ref="t1")
    assert ia.interlocutor_key(both) == "p1"
    assert ia.interlocutor_key(TRACKED) == "trk-1"
    assert ia.interlocutor_key(None) == ""
    assert ia.interlocutor_key(SpeakerRef()) == ""


def test_second_question_for_the_same_speaker_is_stripped():
    asked = {}
    first = ia.limited(NAME_Q, asked, NEW)
    ia.note(first, asked, NEW)
    second = ia.limited(Command(speech="Привет. Как тебя зовут?"), asked, NEW)
    assert first.speech == "Как тебя зовут?"
    assert asked[ia.interlocutor_key(NEW)] == 1
    assert second.speech == "Привет."
    assert not ia.asks(second)


def test_question_beside_an_action_is_stripped_and_does_not_spend_the_quota():
    asked = {}
    cmd = Command(speech="Как тебя зовут?", actions=(ANIM,))
    out = ia.limited(cmd, asked, NEW)
    ia.note(out, asked, NEW)
    assert out.speech is None and out.actions == (ANIM,)
    assert asked == {}
    kept = ia.limited(NAME_Q, asked, NEW)
    assert kept.speech == "Как тебя зовут?"


def test_different_speakers_have_separate_counters():
    asked = {}
    ia.note(ia.limited(NAME_Q, asked, NEW), asked, NEW)
    other = ia.limited(THIS_YOU, asked, OTHER)
    assert ia.asks(other)
    ia.note(other, asked, OTHER)
    assert asked[ia.interlocutor_key(NEW)] == 1
    assert asked[ia.interlocutor_key(OTHER)] == 1


def test_replace_epoch_keeps_the_counter_reset_clears_it():
    session = Session("personality")
    session.identity_asked["spk-new"] = 1
    session.replace_epoch()
    assert session.identity_asked == {"spk-new": 1}
    session.reset()
    assert session.identity_asked == {}


def test_first_name_question_is_spoken_the_second_is_not():
    session = Session("personality")
    session.set_speaker(NEW)
    first, session = _run([NAME_Q], NEW, session)
    assert first.said == ["Как тебя зовут?"]
    assert session.identity_asked[ia.interlocutor_key(NEW)] == 1
    second, session = _run([Command(speech="Привет. Как тебя зовут?")], NEW, session)
    assert second.said == ["Привет."]
    assert session.identity_asked[ia.interlocutor_key(NEW)] == 1


def test_name_question_with_animation_does_not_sound_and_later_ask_still_may():
    session = Session("personality")
    busy, session = _run([Command(speech="Показываю. Как тебя зовут?", actions=(ANIM,))], NEW, session)
    heard = " ".join(busy.said)
    assert "Как тебя зовут?" not in heard
    assert "Показываю" in heard
    assert busy.executed == [[("play_animation", {"animation": "happy"})]]
    assert session.identity_asked == {}
    later, session = _run([NAME_Q], NEW, session)
    assert later.said == ["Как тебя зовут?"]
