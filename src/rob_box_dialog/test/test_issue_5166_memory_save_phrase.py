"""#5166: фраза успеха memory_save строится из результата, не из слов модели (ADR-0148).

Живой n206 (проба на origin/develop, 22.11): модель не вызвала тул и сказала
«Запомнил, Борис: ты болеешь за Спартак с девяносто восьмого года…». Код сохранил
продиктованный факт, а человек услышал выдуманный год. Каталог уже знает фразу
``memory.saved`` — «Запомнил: {fact}.»; её слот — факт из события владельца.
"""

import asyncio

from rob_box_dialog import fact_dictation
from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session
from rob_box_dialog.turn import Answer, TurnEngine

N206 = "робот запомни про меня борис болеет за спартак и всегда приносит пиццу"
N206_FACT = "борис болеет за спартак и всегда приносит пиццу"
INVENTED = "Запомнил, Борис: ты болеешь за Спартак с девяносто восьмого года и по пятницам приносишь пиццу."
INVENTED_YEAR = "девяносто восьмого"


class Ports:
    """Ответ модели и исход тула; сказанное человеку пишется, чтобы тест мог его проверить."""

    def __init__(self, answers, outcomes=None):
        self.answers = list(answers)
        self.outcomes = outcomes or {}
        self.executed = []
        self.said = []

    async def ask(self, messages, tools, timeout_s):
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        self.executed.append([(a.tool, dict(a.args)) for a in actions])
        out = []
        for n, action in enumerate(actions):
            status, data = self.outcomes.get(action.tool, ("done", dict(action.args)))
            out.append(
                ActionResult(
                    f"{turn_id}/{n}",
                    snapshot.epoch,
                    snapshot.agent,
                    action.tool,
                    kn.TOOL_CLASSES[action.tool],
                    status,
                    None,
                    data,
                )
            )
        return out

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append(text)
        return "sp"

    def stage(
        self, turn_id, epoch, stage, status, reason="", cls=None, llm_calls=0, provider="", source="", expression=None
    ):
        pass

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        pass

    def stop_perform(self, how):
        return False

    def intent(self, turn_id, epoch, kind):
        pass

    def perform(self, turn_id, epoch, perf, edit):
        return None


def _run(text, speech, actions=(), outcomes=None):
    ports = Ports([Answer(Command(speech=speech, actions=actions), "deepseek")], outcomes)
    asyncio.run(TurnEngine(Session("personality"), ports, system_prompt="Ты робот.").run(Addressed(text, VOICE)))
    return ports


def test_n206_says_the_catalog_phrase_from_the_saved_fact_not_the_model():
    """Модель тул не вызвала и дописала год. Звучит каталог с фактом из результата, год — нет."""
    ports = _run(N206, INVENTED)
    saved = [args for batch in ports.executed for tool, args in batch if tool == kn.MEMORY_SAVE_TOOL]
    assert saved == [{"fact": N206_FACT, "category": "general"}]
    assert ports.said == [phrases.text("memory.saved", fact=N206_FACT)]
    assert INVENTED_YEAR not in " ".join(ports.said)
    assert INVENTED not in ports.said


def test_a_save_the_model_called_itself_is_still_said_from_the_result():
    """Тул уже в команде — код второй не добавляет, но фраза всё равно из факта результата, не из речи."""
    model_fact = "модель записала другое"
    call = ToolCall(kn.MEMORY_SAVE_TOOL, {"fact": model_fact, "category": "general"})
    ports = _run(N206, INVENTED, actions=(call,), outcomes={kn.MEMORY_SAVE_TOOL: ("done", {"fact": model_fact})})
    assert ports.said == [phrases.text("memory.saved", fact=model_fact)]
    assert INVENTED_YEAR not in " ".join(ports.said)


def test_a_greeting_beside_an_honest_save_is_not_doubled_by_remembered():
    """Не диктовка: «Привет, Саша!» рядом с memory_save не дублируется «Запомнил» (n701)."""
    greeting = "Привет, Саша! Приятно познакомиться."
    call = ToolCall(kn.MEMORY_SAVE_TOOL, {"fact": "Пользователя зовут Саша"})
    ports = _run("здравствуй я Саша чиню тут технику", greeting, actions=(call,))
    assert ports.said == [greeting]
    assert all("Запомнил" not in line for line in ports.said)


def test_a_rejected_dictation_still_says_the_failure_and_the_model():
    """Отказ записи — не успех диктовки: отказ говорит код, слова модели остаются."""
    ports = _run(N206, "Запомнил!", outcomes={kn.MEMORY_SAVE_TOOL: ("rejected", {})})
    assert ports.said == [phrases.text("local.rejected"), "Запомнил!"]


def test_saved_is_none_without_a_dictation_or_a_fact_in_the_result():
    """Предикат молчит: не диктовка, отказ, успех без факта в данных события."""
    done = ActionResult("t", 1, "voice", kn.MEMORY_SAVE_TOOL, "memory", "done", None, {"fact": N206_FACT})
    no_fact = ActionResult("t", 1, "voice", kn.MEMORY_SAVE_TOOL, "memory", "done", None, {"fact_id": "f1"})
    refused = ActionResult("t", 1, "voice", kn.MEMORY_SAVE_TOOL, "memory", "rejected", "error", {"fact": N206_FACT})
    assert fact_dictation.saved(N206, (done,)) is done
    assert fact_dictation.saved("здравствуй я Саша", (done,)) is None
    assert fact_dictation.saved(N206, (no_fact,)) is None
    assert fact_dictation.saved(N206, (refused,)) is None
