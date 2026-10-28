"""#4179: голое «Робот» подхватывает бэклог, а ход отвечает одним лицом без речи (живое 28.10 17:55: «Мы тут чайник
поставили…» → «Робот.» → ``verdict=command actions=[]``; марафон-2 n301 - шаг FAIL no_speech).

Подхват бывает только по голому «робот» - человек обратился к роботу: ``turn.words_owed`` истинно, голый неслышимый
ответ - тот же ``VIOLATION_SILENT_ANSWER``/повтор, что у любой адресованной реплики (не второй механизм). Исключение -
подхваченная фраза-просьба слушать и не встревать (``knowledge.LISTEN_ONLY_FORMS`` + ``SILENCE_WORDS``): код
говорит «Слушаю», LLM не зовётся.
"""

import asyncio

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import BACKLOG_PICKUP, Addressed, decide
from rob_box_dialog.backlog import Backlog, Heard
from rob_box_dialog.command import Command, Expression
from rob_box_dialog.session import Session
from rob_box_dialog.turn import Answer, TurnEngine
from rob_box_dialog.words import asks_to_listen

KETTLE = "Мы тут чайник поставили, минут через пять будет готов."
LISTEN = "Мы тут сейчас немного пошумим, ты просто слушай и не вылезай."
FACE = Expression(emotion="happy")  # живой лог: лицо без речи (verdict=command actions=[])


class Ports:
    """Транспорт хода: ответы LLM по очереди, сказанное - в ``said``."""

    def __init__(self, answers=()):
        self.answers, self.asked, self.said = list(answers), [], []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append(list(messages))
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
        return None


def _picked(background: str) -> Addressed:
    """Фоновая фраза без вейка в бэклог, через 20 с голое «Робот.» - решение адресации, как у ``dialog_node``."""
    backlog = Backlog()
    first = decide(Heard(background, t=100.0), backlog, kn.DEFAULT_WAKE_WORDS)
    assert not isinstance(first, Addressed), "фоновая фраза без вейка роботу не адресована"
    backlog.add(Heard(background, t=100.0))
    picked = decide(Heard("Робот.", t=120.0), backlog, kn.DEFAULT_WAKE_WORDS)
    assert isinstance(picked, Addressed) and picked.pickup and picked.source == BACKLOG_PICKUP
    return picked


def _run(ports, addressed):
    engine = TurnEngine(Session("personality"), ports, system_prompt="Ты робот.")
    return asyncio.run(engine.run(addressed))


def test_bare_robot_over_a_kettle_phrase_the_face_only_answer_is_retried_and_words_are_heard():
    """Живое 17:55: подхват «чайник поставили», ответ модели - одно лицо. Теперь - нарушение и повтор за словами."""
    words = Answer(Command(speech="Отлично, чай через пять минут — подожду."), "deepseek")
    ports = Ports([Answer(Command(expression=FACE), "deepseek"), words])
    _run(ports, _picked(KETTLE))
    assert len(ports.asked) == 2 and kn.VIOLATION_SILENT_ANSWER in ports.asked[1][-1]["content"]
    assert ports.said == ["Отлично, чай через пять минут — подожду."]


def test_bare_robot_over_a_kettle_phrase_silent_twice_the_code_speaks_not_silence():
    """И после повтора одно лицо: говорит код (``SILENT_ANSWER_PHRASE``), а не тишина - речь есть."""
    face = Answer(Command(expression=FACE), "deepseek")
    ports = Ports([face, face])
    agg = _run(ports, _picked(KETTLE))
    assert len(ports.said) == 1 and ports.said[0] in phrases.catalog()[kn.SILENT_ANSWER_PHRASE]
    assert agg.stage == "done"


def test_bare_robot_over_a_listen_only_request_says_listening_by_code_without_the_model():
    """n301: подхвачена просьба «ты просто слушай и не вылезай» - «Слушаю» кодом, LLM не зовётся."""
    ports = Ports()
    agg = _run(ports, _picked(LISTEN))
    assert ports.asked == [] and ports.said == [phrases.text(kn.LISTEN_ONLY_PHRASE)] == ["Слушаю."]
    assert agg.status == "finished"


def test_a_listen_only_request_with_a_wake_word_goes_to_the_model_as_usual():
    """«Робот, ты просто слушай» с вейком - не подхват: решает модель, как раньше (правило - только для подхвата)."""
    ports = Ports([Answer(Command(speech="Хорошо, слушаю."), "deepseek")])
    _run(ports, Addressed("ты просто слушай и не влезай", "voice"))
    assert len(ports.asked) == 1 and ports.said == ["Хорошо, слушаю."]


@pytest.mark.parametrize("text, expected", [
    (LISTEN, True),
    ("ты просто слушай и не влезай", True),
    ("не встревай, мы разговариваем", True),
    ("помолчи немного", True),
    (KETTLE, False),
    ("не молчи, скажи что-нибудь", False),
    ("слушай, а как варить борщ", False),
])
def test_asks_to_listen_is_one_table_with_the_silence_words(text, expected):
    assert asks_to_listen(text) is expected
