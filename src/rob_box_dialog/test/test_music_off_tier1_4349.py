"""#4349, марафон М10 (run 41829742325, акт 8, n812): «Робот, теперь и музыку выключай.» под играющим битом.

STT «роботы пере музыку выключай»: искажённый вейк будил (ворота ``WAKE_SLIP_STOP_FORMS``), но «выключай музыку» не
Tier-1, ход уходил в LLM, она отвечала по прошлому шагу «бит оставил на месте» и музыку не выключала. Теперь, пока
музыка ИГРАЕТ (снимок плеера), форма ``MUSIC_OFF_WORDS`` рядом с объектом музыки - Tier-1 ``music_off``: ``stop_music``
кодом, фраза кода по исходу, 0 вызовов LLM. «Выключи свет», «выключи музыку» без музыки и «музыку не выключай» - как
раньше (не Tier-1).
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


def _snap(**kw):
    return SessionSnapshot(agent="personality", session_id="t", epoch=0, started_at=0.0, **kw)


N812 = "роботы пере музыку выключай"  # STT М10 n812
OK = phrases.text("music.stop_ok")

#: Варианты фразы Tier-1 стопа музыки (решение Шифу 05.11: вариант выбирает код, без повтора подряд).
SAID = frozenset(phrases.catalog()[kn.MUSIC_STOP_SAID])
PLAYING = _snap(music=MusicStateRef("playing", track_id="req95689:01:A"))
IDLE = _snap()

OFF_UNDER_MUSIC = [N812, "пере музыку выключай", "Робот, теперь и музыку выключай.", "теперь и музыку выключай",
                   "выключи её", "выключи это", "выключи музыку", "отключи бит", "выключи пожалуйста эту песню",
                   "выключай сет"]
NOT_OFF = ["выключи свет", "музыку не выключай", "не выключай музыку", "выключи сеть", "расскажи сказку и выключи музыку",
           "выключи свет и музыку оставь"]


@pytest.mark.parametrize("text", OFF_UNDER_MUSIC)
def test_music_off_under_playing_music_is_tier1_without_rest(text):
    assert match(text, PLAYING) == Tier1Command(kn.MUSIC_OFF_KIND), text


@pytest.mark.parametrize("text", NOT_OFF)
def test_light_negation_and_foreign_requests_are_not_music_off(text):
    command = match(text, PLAYING)
    assert command is None or command.kind != kn.MUSIC_OFF_KIND, text


@pytest.mark.parametrize("text", ["выключи музыку", "выключи её", "теперь и музыку выключай", N812])
def test_without_playing_music_nothing_changes(text):
    assert match(text, IDLE) is None, text


def test_new_request_after_music_off_is_the_rest():
    assert match("выключи бит и спой песню", PLAYING) == Tier1Command(kn.MUSIC_OFF_KIND, rest="спой песню")


class Ports:
    """Транспорт хода: стоп музыки отвечает ``was_playing`` по состоянию плеера; LLM считает вызовы."""

    def __init__(self, answers=()):
        self.answers = list(answers)
        self.playing = True
        self.executed, self.said, self.asked = [], [], 0

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
            out.append(ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, call.tool,
                                    kn.TOOL_CLASSES[call.tool], "done", None, data))
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


#: Ответ LLM М10 n812: по контексту прошлого шага, без стопа.
N812_LLM = Answer(Command(speech="Читать перестал, бит оставил на месте, Саша."), "deepseek")


def _run(ports, text, command, music="playing", dj=False):
    ids = iter(f"t{n}" for n in range(1, 100))
    session = Session("personality")
    session.set_music(MusicStateRef(music, track_id="req95689:01:A" if music == "playing" else None))
    if dj:
        session.push_scene("DJ", "user")
    engine = TurnEngine(session, ports, system_prompt="Ты робот.", new_id=lambda: next(ids))
    return asyncio.run(engine.run(Addressed(text, VOICE, command=command)))


@pytest.mark.parametrize("text,command", [(N812, "пере музыку выключай"),
                                          ("Робот, теперь и музыку выключай.", "теперь и музыку выключай"),
                                          ("Робот, выключи её.", "выключи её")])
def test_n812_turn_stops_music_by_code_and_says_the_code_phrase(text, command):
    ports = Ports([N812_LLM])
    agg = _run(ports, text, command)
    assert ports.executed == [("stop_music", {})]
    assert ports.asked == 0, "стоп музыки решает код, LLM не зовётся"
    assert len(ports.said) == 1 and ports.said[0] in SAID, ports.said
    assert (agg.stage, agg.status) == ("done", "finished")


def test_dj_set_is_stopped_with_the_deck():
    ports = Ports()
    _run(ports, N812, "пере музыку выключай", dj=True)
    assert ports.executed == [("dj_set", {"action": "stop"}), ("stop_music", {})]
    assert ports.asked == 0


@pytest.mark.parametrize("text", ["Робот, выключи свет.", "Робот, музыку не выключай."])
def test_other_requests_under_music_still_go_to_the_model(text):
    ports = Ports([Answer(Command(speech="Хорошо."), "deepseek")])
    _run(ports, text, text.split(", ", 1)[1].rstrip("."))
    assert ports.asked == 1
    assert ("stop_music", {}) not in ports.executed


def test_music_off_without_music_still_goes_to_the_model():
    ports = Ports([Answer(Command(speech="Музыка не играет."), "deepseek")])
    _run(ports, "Робот, выключи музыку.", "выключи музыку", music="idle")
    assert ports.asked == 1
