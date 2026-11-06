"""#4207 (решение Шифу 04.11): правка звучащего счёта продолжает его в новой форме, а не считает заново.

Живой лог 04.11 04:07Z (образ с #4325): «посчитай до двадцати» - модель ответила речью «Один, два, … двадцать» (192
знака). Через ~15 с «и после каждого числа добавляй слово шаг» - модель видела звучащий счёт в окне, но прислала
«Один шаг, два шага, …» с единицы, и он прозвучал после старого, досказанного целиком.

Нужно: «1, 2, 3» → правка → «4 шаг, 5 шаг, …»; текущее число договаривается, повтора с единицы нет. Решает код
(ADR-0148): счёт речью - по предложению на число и с ``boundary=sentence``; правка режет звучащий счёт по его
``speech_id`` на границе числа; сколько прозвучало, говорит ``spoken_chars`` владельца речи, а от модели берётся только
форма элемента («шаг»). Здесь - чистые функции ``counting`` и ход целиком на двойнике портов; с настоящим журналом
владельца речи - ``rob_box_voice/test/unit/core/test_count_edit_owner_4207.py``.
"""

import asyncio

from rob_box_dialog import counting
from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session
from rob_box_dialog.turn import Answer, TurnEngine

N806 = "посчитай вслух от одного до двадцати спокойно"
N807 = "и после каждого числа добавляй слово шаг"
_FORMS = {1: "шаг", 2: "шага", 3: "шага", 4: "шага"}
#: Ответы модели из живого лога 04.11: счёт через запятую и правка с единицы.
COUNT = ", ".join(counting.say(n) for n in range(1, 21)).capitalize() + "."
EDIT = ", ".join(f"{counting.say(n)} {_FORMS.get(n, 'шагов')}" for n in range(1, 21)).capitalize() + "."
SENTENCED = counting.count_text(range(1, 21))
#: Продолжение после «Четыре.»: числа 5-20 в форме правки.
REST = " ".join(f"{counting.say(n).capitalize()} шагов." for n in range(5, 21))


# -- counting: ряд и его форма ----------------------------------------------------------------------------------------


def test_series_splits_a_count_into_numbers_and_their_form():
    lead, items, trailer = counting.series("Считаю, Саша: один шаг, два шага. Все дошли.")
    assert lead == "Считаю, Саша:" and trailer == "Все дошли."
    assert [(i.number, i.form) for i in items] == [(1, ("шаг",)), (2, ("шага",))]
    assert counting.is_count(COUNT) and counting.is_count(EDIT) and counting.is_count(SENTENCED)
    assert not counting.is_count("Считаю до двадцати.") and not counting.is_count("Мне пять лет, брату семь.")


def test_count_speech_is_one_sentence_per_number_lead_and_trailer_kept():
    cmd = counting.sentenced(Command(speech="Хорошо, Саша. Один, два, три. Вот и всё."), "посчитай до трёх")
    assert cmd is not None and cmd.speech == "Хорошо, Саша. Один. Два. Три. Вот и всё."
    assert counting.sentenced(Command(speech=SENTENCED), N806) is None  # уже так
    assert counting.sentenced(Command(speech=COUNT), "как дела") is None  # счёт не просили


def test_continuation_takes_numbers_from_progress_and_form_from_the_edit():
    spoken = len("Один. Два. Три. Четыре.")  # владелец речи дочитал «Четыре» и встал
    assert counting.continued(SENTENCED, spoken, EDIT) == REST


def test_continuation_of_an_edit_shorter_than_the_count_uses_its_common_form():
    """Модель прислала правку до трёх - ряд всё равно старый, до пяти: форма - самая частая в правке."""
    assert counting.continued("Один. Два. Три. Четыре. Пять.", len("Один. Два."), "Один шаг, два шага, три шага.") == \
        "Три шага. Четыре шага. Пять шага."


def test_nothing_left_after_the_whole_count_sounded():
    assert counting.continued(SENTENCED, len(SENTENCED), EDIT) == ""


# -- ход целиком: что уходит владельцу речи ---------------------------------------------------------------------------


class Ports:
    """Двойник транспорта: реплики и отмены записываются; конец реплики тест присылает сам (``speech_ended``)."""

    def __init__(self, answers):
        self.answers, self.said, self.cancels = list(answers), [], []

    async def ask(self, messages, tools, timeout_s):
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        return [ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, a.tool, kn.TOOL_CLASSES[a.tool],
                             "done", None, dict(a.args)) for n, a in enumerate(actions)]

    def say(self, turn_id, epoch, text, voice=None, emotion=None, boundary="none"):
        self.said.append((text, boundary))
        return f"sp{len(self.said)}"

    def stage(self, *args, **kwargs):
        pass

    def cancel_speech(self, epoch, reason, boundary="as_requested", speech_id=""):
        self.cancels.append((speech_id, boundary))

    def stop_perform(self, how):
        return False

    def intent(self, turn_id, epoch, kind):
        pass

    def perform(self, turn_id, epoch, perf, edit):
        return None


def _engine(*answers):
    ports = Ports([Answer(cmd, "deepseek") for cmd in answers])
    ids = iter(f"t{n}" for n in range(1, 100))
    return TurnEngine(Session("personality"), ports, system_prompt="Ты робот.", new_id=lambda: next(ids)), ports


def test_count_goes_to_the_speech_owner_by_sentences_with_sentence_boundary():
    engine, ports = _engine(Command(speech=COUNT))
    asyncio.run(engine.run(Addressed(N806, VOICE)))
    assert ports.said == [(SENTENCED, "sentence")]


def test_edit_of_a_sounding_count_cuts_it_and_continues_in_the_new_form():
    """n806 → n807: правка пришла, пока звучит «Четыре». Счёт с единицы не звучит; старый режется по ``speech_id``,
    продолжение - когда владелец речи скажет, где встал."""
    engine, ports = _engine(Command(speech=COUNT), Command(speech=EDIT))
    asyncio.run(engine.run(Addressed(N806, VOICE)))
    engine.speech_started("sp1")
    asyncio.run(engine.run(Addressed(N807, VOICE)))
    assert ports.cancels == [("sp1", "as_requested")]
    assert len(ports.said) == 1  # правка с единицы не ушла
    engine.speech_ended("sp1", len("Один. Два. Три. Четыре."))
    assert ports.said[1] == (REST, "sentence")
    spoken = [n for text, _ in ports.said[:1] for n in counting.said_numbers(text[:len("Один. Два. Три. Четыре.")])]
    assert spoken + list(counting.said_numbers(ports.said[1][0])) == list(range(1, 21))  # каждое число один раз


def test_count_that_ended_before_the_cut_is_not_repeated():
    """Гонка: старый счёт договорился до конца раньше отмены (``finished``) - продолжать нечего, повтора нет."""
    engine, ports = _engine(Command(speech=COUNT), Command(speech=EDIT))
    asyncio.run(engine.run(Addressed(N806, VOICE)))
    asyncio.run(engine.run(Addressed(N807, VOICE)))
    engine.speech_ended("sp1", len(SENTENCED))
    assert len(ports.said) == 1


def test_new_count_request_while_counting_is_not_an_edit():
    engine, ports = _engine(Command(speech=COUNT), Command(speech="Один. Два. Три."))
    asyncio.run(engine.run(Addressed(N806, VOICE)))
    asyncio.run(engine.run(Addressed("посчитай до трех", VOICE)))
    assert ports.cancels == [] and ports.said[1] == ("Один. Два. Три.", "sentence")


def test_count_beside_loud_action_is_not_an_edit():
    """Речь рядом с громким действием не звучит (§4.1) - и звучащий счёт не режет."""
    sing = ToolCall("play_sound", {"sound": "robot_alert"})
    engine, ports = _engine(Command(speech=COUNT), Command(speech=EDIT, actions=(sing,)))
    asyncio.run(engine.run(Addressed(N806, VOICE)))
    asyncio.run(engine.run(Addressed(N807, VOICE)))
    assert ports.cancels == []
