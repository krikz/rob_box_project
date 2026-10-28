"""#4173: фраза стопа музыки - правда о ходе целиком (ADR-0148), а не о последнем туле.

Марафон-2 28.10, акт 6, n608 («робот хватит диджей выключай», 16:44:05): ясный стоп (Tier-1 ``stop_all``) выключил
музыку молча, остаток фразы «диджей выключай» ушёл модели (``Tier1Command.rest``), её ``dj_set stop`` застал тишину
(``was_playing=False``) и честно так ответил - а робот сказал «Сейчас ничего не играет — на всякий случай всё
остановил.», хотя ход музыку только что выключил. Теперь стоп, заставший тишину после стопа ЭТОГО ЖЕ хода, - «Выключил
музыку.» (``respond.reconcile_stops``); первый стоп хода, заставший тишину, по-прежнему «ничего не играет».
"""

import asyncio

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.respond import ActionResult, reconcile_stops
from rob_box_dialog.session import MusicStateRef, Session
from rob_box_dialog.turn import Answer, TurnEngine

OK, IDLE = phrases.text("music.stop_ok"), phrases.text("music.stop_idle")


class Ports:
    """Транспорт хода: музыка играет, пока её не погасит первый стоп-тул; стоп на тишине отвечает ``was_playing=False``
    (как настоящие ``stop_music`` и ``dj_set stop``)."""

    def __init__(self, answers=(), playing=True):
        self.answers = list(answers)
        self.playing = playing
        self.executed = []
        self.said = []

    async def ask(self, messages, tools, timeout_s):
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        out = []
        for n, call in enumerate(actions):
            data = dict(call.args)
            if call.tool in kn.MUSIC_STOP_TOOLS:
                data["was_playing"], self.playing = self.playing, False
            self.executed.append(call.tool)
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


def _run(ports, text, music="playing"):
    ids = iter(f"t{n}" for n in range(1, 100))
    session = Session("personality")
    session.set_music(MusicStateRef(music))
    engine = TurnEngine(session, ports, system_prompt="Ты робот.", new_id=lambda: next(ids))
    return asyncio.run(engine.run(Addressed(text, VOICE)))


def _result(tool, **data):
    return ActionResult("t1/0", 0, "personality", tool, kn.TOOL_CLASSES[tool], "done", None, data)


def test_n608_dj_off_after_a_clear_stop_in_the_same_turn_says_the_music_is_off():
    """«Хватит, диджей выключай»: Tier-1 стоп гасит музыку, модель зовёт ``dj_set stop`` - «Выключил музыку.»."""
    ports = Ports([Answer(Command(actions=(ToolCall("dj_set", {"action": "stop"}),)), "deepseek")])
    _run(ports, "Робот, хватит диджей выключай")
    assert ports.executed == ["stop_music", "dj_set"], "Tier-1 стоп первым, затем вызов модели"
    assert ports.said == [OK] and IDLE not in ports.said


def test_stop_music_after_the_clear_stop_in_the_same_turn_is_not_idle_either():
    ports = Ports([Answer(Command(actions=(ToolCall("stop_music", {"boundary": "bar"}),)), "deepseek")])
    _run(ports, "Робот, хватит выключай")
    assert ports.said == [OK]


def test_the_first_stop_of_the_turn_that_finds_silence_still_says_nothing_was_playing():
    ports = Ports([Answer(Command(actions=(ToolCall("dj_set", {"action": "stop"}),)), "deepseek")], playing=False)
    _run(ports, "Робот, выключи диджея", music="idle")
    assert ports.said == [IDLE]


def test_two_stops_in_one_command_say_one_outcome_the_music_is_off():
    """[dj_set stop, stop_music] одной командой: второй застаёт тишину после первого - не «ничего не играет»."""
    calls = (ToolCall("dj_set", {"action": "stop"}), ToolCall("stop_music", {}))
    ports = Ports([Answer(Command(actions=calls), "deepseek")])
    _run(ports, "Робот, выключи диджея и музыку", music="idle")
    assert ports.said == [OK]


def test_reconcile_marks_only_a_stop_that_found_silence_after_a_stop_of_the_turn():
    first, second = _result("stop_music", was_playing=True), _result("dj_set", action="stop", was_playing=False)
    stopped, (a, b) = reconcile_stops(False, [first, second])
    assert stopped and a is first and b.data[kn.STOPPED_EARLIER_FIELD] is True and b.data["was_playing"] is False
    stopped, (idle,) = reconcile_stops(False, [second])
    assert not stopped and idle is second and kn.STOPPED_EARLIER_FIELD not in idle.data
    stopped, (after,) = reconcile_stops(True, [second])
    assert stopped and after.data[kn.STOPPED_EARLIER_FIELD] is True


@pytest.mark.parametrize("result", [
    _result("dj_set", action="start", theme="космос", was_playing=False),  # старт сета - не стоп
    _result("play_sound", sound="ok"),
])
def test_reconcile_leaves_other_tools_untouched(result):
    stopped, (same,) = reconcile_stops(True, [result])
    assert stopped and same is result


def test_failed_stop_does_not_count_as_a_stop_of_the_turn():
    failed = ActionResult("t1/0", 0, "personality", "stop_music", kn.TOOL_CLASSES["stop_music"], "rejected", "error", {})
    idle = _result("stop_music", was_playing=False)
    stopped, (_, later) = reconcile_stops(False, [failed, idle])
    assert not stopped and kn.STOPPED_EARLIER_FIELD not in later.data
