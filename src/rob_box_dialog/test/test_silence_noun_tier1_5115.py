"""#5115: «Робот, тишина» — Tier-1, та же дверь, что «молчи».

Живой прогон: существительное «тишина» не в ``SILENCE_WORDS``, ``_stop`` возвращал ``None`` до проверки тишины, ход
уходил в LLM и робот говорил. Теперь ``words.silence_noun_asked`` (таблица ``knowledge.SILENCE_NOUN_FORMS`` /
``SILENCE_NOUN_FILLER``) читает ``grammar._stop`` до LLM: сцена Silence, 0 вызовов, 0 TTS. Упоминание — не тишина.
При играющей музыке та же фраза — ``stop_all`` (плеер гаснет), не сцена Silence: сцена тишины музыку не останавливает.
«Молчи»/«замолчи» эта ветка не трогает: у них вердикт стопа ясный.
"""

import asyncio

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command
from rob_box_dialog.grammar import Tier1Command, match
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import MusicStateRef, Session, SessionSnapshot
from rob_box_dialog.turn import Answer, TurnEngine

WAKE = ("робот",)
SAID = frozenset(phrases.catalog()[kn.MUSIC_STOP_SAID])


def _snap(**kw):
    return SessionSnapshot(agent="personality", session_id="t", epoch=0, started_at=0.0, wake_words=WAKE, **kw)


IDLE = _snap()
PLAYING = _snap(music=MusicStateRef("playing", track_id="req:01:A", title="Бит"))


class Ports:
    """Транспорт хода: LLM считает вызовы, стоп музыки отвечает ``was_playing``."""

    def __init__(self, answers=()):
        self.answers = list(answers)
        self.playing = True
        self.executed, self.said = [], []
        self.asked = 0

    async def ask(self, messages, tools, timeout_s):
        self.asked += 1
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        out = []
        for n, call in enumerate(actions):
            data = dict(call.args)
            if call.tool in kn.MUSIC_STOP_TOOLS:
                data["was_playing"], self.playing = self.playing, False
            self.executed.append((call.tool, dict(call.args)))
            out.append(
                ActionResult(
                    f"{turn_id}/{n}",
                    snapshot.epoch,
                    snapshot.agent,
                    call.tool,
                    kn.TOOL_CLASSES[call.tool],
                    "done",
                    None,
                    data,
                )
            )
        return out

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


def _engine(ports, music=False):
    ids = iter(f"t{n}" for n in range(1, 100))
    session = Session("personality", wake_words=WAKE)
    if music:
        session.set_music(MusicStateRef("playing", track_id="req:01:A"))
    return TurnEngine(session, ports, system_prompt="Ты робот.", new_id=lambda: next(ids)), session


def _run(ports, text, music=False):
    engine, session = _engine(ports, music)
    agg = asyncio.run(engine.run(Addressed(text, VOICE)))
    return agg, session


@pytest.mark.parametrize(
    "text",
    [
        "Робот, тишина",
        "Робот, тишину, пожалуйста",
        "тишина, пожалуйста",
        "полная тишина",
        "Робот, полная тишина",
    ],
)
def test_silence_noun_is_tier1_silence(text):
    assert match(text, IDLE) == Tier1Command("silence"), text


@pytest.mark.parametrize(
    "text",
    [
        "Робот, расскажи стих про тишину",
        "что такое тишина",
        "включи тишину",
        "тишина в эфире",
    ],
)
def test_a_mention_of_silence_is_not_tier1(text):
    command = match(text, IDLE)
    assert command is None or command.kind != "silence", text


def test_do_not_be_silent_stays_the_disputed_verb_not_the_noun():
    """«Не молчи» — спорный глагол (``SILENCE_EXACT_FORMS``), как до #5115, а не существительное и не ясная тишина."""
    command = match("Робот, не молчи", IDLE)
    assert command == Tier1Command("silence", disputed=True)


def test_silence_noun_turn_sets_scene_without_llm_or_speech_then_speak_clears_it():
    """«Робот, тишина» — сцена Silence, 0 LLM, речи нет; «Робот, говори» снимает сцену (``unsilence``)."""
    ports = Ports([Answer(Command(speech="Я тут."), "deepseek")])
    engine, session = _engine(ports)
    agg = asyncio.run(engine.run(Addressed("Робот, тишина", VOICE)))
    assert agg.llm_calls == 0 and ports.asked == 0
    assert ports.said == []
    assert session.snapshot.top("Silence") is not None
    assert (agg.stage, agg.status) == ("done", "finished")

    before = session.snapshot
    assert match("Робот, говори", before).kind == "unsilence"
    agg = asyncio.run(engine.run(Addressed("Робот, говори", VOICE)))
    assert agg.llm_calls == 0 and ports.asked == 0
    assert session.snapshot.top("Silence") is None


def test_please_and_full_silence_turns_are_tier1_too():
    for text in ("тишина, пожалуйста", "полная тишина"):
        ports = Ports([Answer(Command(speech="Говорю."), "deepseek")])
        agg, session = _run(ports, text)
        assert agg.llm_calls == 0 and ports.said == [], text
        assert session.snapshot.top("Silence") is not None, text


def test_a_verse_about_silence_still_goes_to_the_model():
    ports = Ports([Answer(Command(speech="Стих про тишину."), "deepseek")])
    agg, session = _run(ports, "Робот, расскажи стих про тишину")
    assert agg.llm_calls == 1
    assert session.snapshot.top("Silence") is None
    assert ports.said == ["Стих про тишину."]


@pytest.mark.parametrize("text", ["Робот, тишина", "тишина, пожалуйста", "полная тишина", "Робот, тишину"])
def test_silence_noun_while_music_plays_is_a_music_stop_not_a_silence_scene(text):
    """При играющей музыке существительное — ``stop_all``, не сцена Silence: сцена тишины плеер не гасит."""
    assert match(text, PLAYING) == Tier1Command("stop_all"), text
    assert match("Робот, замолчи", PLAYING).kind == "silence"  # глагол при музыке не стал стопом музыки


def test_silence_noun_under_music_stops_the_player_and_does_not_set_silence():
    ports = Ports([Answer(Command(speech="Тишина."), "deepseek")])
    agg, session = _run(ports, "Робот, тишина", music=True)
    assert ports.executed == [("stop_music", {})]
    assert agg.llm_calls == 0 and ports.asked == 0
    assert session.snapshot.top("Silence") is None
    assert len(ports.said) == 1 and ports.said[0] in SAID
