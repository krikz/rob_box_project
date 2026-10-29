"""#4173, марафон-4 (28.10 22:24:09, акт 6, n608): «робот хватит диджей выключай» под диджей-сетом.

Ясный стоп (Tier-1 ``stop_all``) кодом гасит сет: ``dj_set stop`` + ``stop_music``, молча. Остаток фразы «диджей
выключай» уходит модели, она зовёт тот же ``dj_set {"action": "stop"}`` - повтор исполненного в этом ходе не
исполняется (``turn.unrepeated``), а ход падал ``stage=failed`` (``nothing_to_do``): earcon fail, красное кольцо, речи
нет. Повтор исполненного - не провал: ход сделал, что просили, и код говорит исход по результату Tier-1 («Выключил
музыку.»), ход ``done``.
"""

import asyncio

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import MusicStateRef, Session
from rob_box_dialog.turn import Answer, TurnEngine

N608 = "робот хватит диджей выключай"  # STT марафона-3 28.10 19:50:42 (va_m3.log), тот же шаг в марафоне-4
OK = phrases.text("music.stop_ok")


class Ports:
    """Транспорт хода: идёт сет, первый стоп гасит музыку, стоп на тишине отвечает ``was_playing=False``."""

    def __init__(self, answers):
        self.answers = list(answers)
        self.playing = True
        self.executed = []
        self.said = []
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
            out.append(ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, call.tool,
                                    kn.TOOL_CLASSES[call.tool], "done", None, data))
        return out

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append((text, emotion))
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


def _run(ports, text):
    ids = iter(f"t{n}" for n in range(1, 100))
    session = Session("personality")
    session.set_music(MusicStateRef("playing", track_id="gen26164:00:A"))
    session.push_scene("DJ", "user")  # идёт диджей-сет: Tier-1 стоп гасит и сет, и деку
    engine = TurnEngine(session, ports, system_prompt="Ты робот.", new_id=lambda: next(ids))
    return asyncio.run(engine.run(Addressed(text, VOICE)))


def test_n608_model_repeating_the_tier1_dj_stop_closes_the_turn_with_the_code_phrase_not_a_failure(caplog):
    ports = Ports([Answer(Command(actions=(ToolCall("dj_set", {"action": "stop"}),)), "deepseek")])
    with caplog.at_level("INFO"):
        agg = _run(ports, N608)
    # Tier-1 исполнил оба стопа, повтор модели не исполнен (сет не трогается дважды)
    assert ports.executed == [("dj_set", {"action": "stop"}), ("stop_music", {})]
    assert ports.asked == 1
    assert "повтор исполненного в этом ходе не исполняется: ['dj_set']" in caplog.text
    # ход не провален: фраза кода по исходу Tier-1, одна, без грусти
    assert (agg.stage, agg.status) == ("done", "finished")
    assert [text for text, _emotion in ports.said] == [OK] and ports.said[0][1] != "sad"


def test_repeat_of_a_voiced_outcome_closes_done_without_saying_it_again():
    """Исход, который код уже сказал, повтор модели второй раз не озвучивает (``_Turn.unvoiced`` пуст), ход - не
    провал."""
    from rob_box_dialog.turn import _close_unread, _Turn  # noqa: PLC0415

    turn = _Turn.__new__(_Turn)
    turn.queried, turn.unvoiced, turn.acted = (), (), (ToolCall("dj_set", {"action": "stop"}),)
    turn.stage, turn.status, turn.reason = "done", "finished", ""
    assert _close_unread(turn, ()) == ("", None)
    assert (turn.stage, turn.status) == ("done", "finished")


def test_nothing_done_in_the_turn_and_only_a_dropped_repeat_is_still_a_failure_path_untouched():
    """Без исполненного в ходе путь ``nothing_to_do`` прежний: повторять было нечего - модель ничего не дала."""
    from rob_box_dialog.turn import _close_unread, _Turn  # noqa: PLC0415

    turn = _Turn.__new__(_Turn)
    turn.queried, turn.unvoiced, turn.acted = (), (), ()
    turn.stage, turn.status, turn.reason = "done", "finished", ""
    assert _close_unread(turn, ()) == ("", None)
    assert (turn.stage, turn.status, turn.reason) == ("failed", "failed", "nothing_to_do")
