"""#4320: повтор «посчитай до сорока» посреди идущего счёта — счёт строит код, речь модели посреди выступления не звучит.

Живой Sound Pi 04.11 (образ после #4319): второй запрос пришёл, пока звучал третий сегмент счёта. Модель ответила не
``perform``, а речью с ``play_animation``: «Считаю до сорока. Один, два, … десять.» Код (#4317, ``counting``) счёт не
достроил: «до сорока» в её же речи он принял за досчитанное «сорок» — счёт «полный». Речь встала в очередь голоса и
прозвучала между «тридцать» и «тридцать один» старого счёта. Граница «до N» / «от N» — это просьба, а не счёт.
"""

import asyncio

from rob_box_dialog import counting
from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session
from rob_box_dialog.turn import Answer, TurnEngine

TEXT = "посчитай до сорока"
#: Дословно ответ модели из живого лога 04.11 (``[command] … speech=``), без сокращения «…».
SPEECH = "Считаю до сорока. Один, два, три, четыре, пять, шесть, семь, восемь, девять, десять."
ANIMATION = ToolCall("play_animation", {"animation": "thinking"})


class Ports:
    """Транспорт хода: ответы LLM по очереди; выступление владелец берёт (``pending``)."""

    def __init__(self, answers):
        self.answers = list(answers)
        self.said, self.performed = [], []

    async def ask(self, messages, tools, timeout_s):
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        return [ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, a.tool, kn.TOOL_CLASSES[a.tool],
                             "done", None, dict(a.args)) for n, a in enumerate(actions)]

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
        self.performed.append(perf)
        return {"status": "pending", "block_id": f"perform-{turn_id}"}


def _numbers(perf):
    return [n for segment in perf.segments for n in counting.said_numbers(segment)]


def test_bound_of_the_request_in_model_speech_is_not_a_reached_number():
    """«Считаю до сорока.» — пересказ просьбы: «сорок» в нём не досчитано."""
    assert counting.reached(Command(speech=SPEECH)) == tuple(range(1, 11))
    assert counting.reached(Command(speech="Считаю от пяти до десяти. Пять, шесть.")) == (5, 6)


def test_live_answer_is_a_short_count_built_by_code():
    full = counting.full_count(Command(speech=SPEECH, actions=(ANIMATION,)), TEXT)
    assert full is not None and full.speech is None and full.perform is not None
    assert _numbers(full.perform) == list(range(1, 41))
    assert full.actions == (ANIMATION,)  # тихое действие модели остаётся, слова — нет


def test_counted_numbers_are_still_counted():
    """Числа самого счёта (не граница) считаются как раньше: «сорок» в счёте — досчитано."""
    assert counting.reached(Command(speech="Тридцать девять. Сорок.")) == (39, 40)
    assert counting.full_count(Command(speech="Раз, два, три… тридцать девять и сорок!"), TEXT) is None


def test_repeat_mid_count_turn_performs_the_code_count_and_model_speech_is_silent():
    """Ход целиком, как в живом логе: первый счёт идёт (владелец взял выступление), повтор той же просьбы — модель
    отвечает речью с числами и ``play_animation``. Звучит не её речь, а выступление кода до сорока."""
    first = counting.full_count(Command(speech="Один, два."), TEXT)
    ports = Ports([Answer(first, "deepseek"), Answer(Command(speech=SPEECH, actions=(ANIMATION,)), "deepseek")])
    engine = TurnEngine(Session("personality"), ports, system_prompt="Ты робот.")
    asyncio.run(engine.run(Addressed(TEXT, VOICE)))
    assert engine.log.performing()
    asyncio.run(engine.run(Addressed(TEXT, VOICE)))
    assert SPEECH not in ports.said
    assert not any(counting.said_numbers(s) for s in ports.said), ports.said  # чисел речью посреди счёта нет
    assert len(ports.performed) == 2 and ports.performed[1].segments == ports.performed[0].segments
