"""#4339: «посчитай до семидесяти» → «Считаю до семидесяти, слушай внимательно.» - ни одного числа, счёта нет.

Живой лог 04.11 09:24 UTC (dev-4d43068): ``verdict=speech actions=[]``, TTS сказал только обещание, 45 с тишины. Модель
пообещала счёт и не начала - числа строит код до ``min(N, COUNT_MAX)`` (``counting.full_count``), при урезании - с
фразой кода ``COUNT_CAPPED_PHRASE``; слова модели («до семидесяти») не звучат. Отказ и переспрос остаются (ADR-0148).
"""

import asyncio

import pytest

from rob_box_dialog import counting
from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command
from rob_box_dialog.session import Session
from rob_box_dialog.turn import Answer, TurnEngine

LIVE_TEXT = "робот посчитай до семидесяти"
LIVE_SPEECH = "Считаю до семидесяти, слушай внимательно."


def _all_numbers(cmd):
    parts = [cmd.speech or "", *(cmd.perform.segments if cmd.perform is not None else ())]
    return [n for part in parts for n in counting.said_numbers(part, bounds=False)]


def test_live_promise_above_the_ceiling_is_counted_by_code_to_fifty():
    full = counting.full_count(Command(speech=LIVE_SPEECH), LIVE_TEXT)
    assert full is not None
    assert _all_numbers(full) == list(range(1, kn.COUNT_MAX + 1))
    first = full.speech or full.perform.segments[0]
    assert first.startswith(phrases.text(kn.COUNT_CAPPED_PHRASE, top="пятидесяти"))
    spoken = " ".join([full.speech or "", *(full.perform.segments if full.perform else ())])
    assert "семидесяти" not in spoken and "слушай" not in spoken  # граница модели выше предела не звучит


def test_promise_within_speech_limit_stays_plain_speech():
    full = counting.full_count(Command(speech="Сейчас посчитаю до десяти."), "посчитай до десяти")
    assert full is not None and full.perform is None
    assert counting.said_numbers(full.speech) == tuple(range(1, 11))
    assert not full.speech.startswith(phrases.text(kn.COUNT_CAPPED_PHRASE, top="пятидесяти"))


@pytest.mark.parametrize("speech", [
    "Считаю до двадцати. Один.",            # одно число - не начатый счёт
    "Начинаю считать!",
    "Хорошо, досчитаю до двадцати.",
])
def test_promise_forms_are_counted(speech):
    full = counting.full_count(Command(speech=speech), "посчитай до двадцати")
    assert full is not None and counting.said_numbers(full.speech) == tuple(range(1, 21))


@pytest.mark.parametrize("cmd", [
    Command(speech="Не буду считать, это скучно."),                     # отказ
    Command(speech="До скольки считать?"),                              # переспрос
    Command(speech="Считаю до двадцати?", ask="До двадцати точно?"),    # переспрос полем ask
    Command(speech="Я не досчитаю до двадцати, извини."),               # отрицание перед глаголом
    Command(speech="Считаю, что это слишком долго."),                   # мнение, не счёт
])
def test_refusal_or_question_stays_model_answer(cmd):
    assert counting.full_count(cmd, "посчитай до двадцати") is None


def test_promise_without_asked_range_stays():
    assert counting.full_count(Command(speech="Считаю!"), "посчитай, сколько нас") is None


class Ports:
    def __init__(self, answers):
        self.answers, self.said, self.performed = list(answers), [], []

    async def ask(self, messages, tools, timeout_s):
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        return []

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
        return None


def test_live_turn_counts_to_fifty_instead_of_the_bare_promise():
    """Ход целиком: ответ модели - одно обещание, звучит счёт кода до пятидесяти с фразой о пределе."""
    ports = Ports([Answer(Command(speech=LIVE_SPEECH), "deepseek")])
    engine = TurnEngine(Session("personality"), ports, system_prompt="Ты робот.")
    asyncio.run(engine.run(Addressed(LIVE_TEXT, VOICE)))
    assert LIVE_SPEECH not in ports.said
    # фейковый ``perform`` возвращает None - ход говорит сегменты речью; считаем один из двух путей, не оба
    spoken = " ".join([s for p in ports.performed for s in p.segments] or ports.said)
    assert counting.said_numbers(spoken, bounds=False) == tuple(range(1, kn.COUNT_MAX + 1))
    assert phrases.text(kn.COUNT_CAPPED_PHRASE, top="пятидесяти") in spoken
