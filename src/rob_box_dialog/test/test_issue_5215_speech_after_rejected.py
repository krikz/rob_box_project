"""#5215: после отказа memory_save речь модели о записи не звучит (ADR-0148).

Живой ход 24.11 (Sound Pi, образ dev-429fbe3, ядро 0.7.31): модель позвала memory_save,
тул закрылся rejected{no_person}, код сказал «Не узнаю твой голос, поэтому не запомню.»,
а следом прозвучало «Записываю: Борис болеет за Спартак и всегда приносит пиццу.» из
speech модели. Фраза об исходе строится кодом из результата; речь, утверждающая
несделанное действие, не озвучивается. Регекса по ответу нет: решает таблица
``knowledge.DONE_CLAIMS`` и исход тула.
"""

import asyncio

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session
from rob_box_dialog.turn import Answer, TurnEngine
from rob_box_dialog.words import done_claims

HEARD = "Робот, запомни про меня борис болеет за спартак и всегда приносит пиццу"
FACT = "Борис болеет за Спартак и всегда приносит пиццу"
RECORDING = f"Записываю: {FACT}"
REFUSAL = phrases.text("local.rejected.no_person")
OTHER = "Хорошо, расскажу, как дойти до кухни."


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
            status, reason, data = self.outcomes.get(action.tool, ("done", None, dict(action.args)))
            out.append(
                ActionResult(
                    f"{turn_id}/{n}",
                    snapshot.epoch,
                    snapshot.agent,
                    action.tool,
                    kn.TOOL_CLASSES[action.tool],
                    status,
                    reason,
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


def _save(fact=FACT):
    return ToolCall(kn.MEMORY_SAVE_TOOL, {"fact": fact, "category": "general"})


def test_present_tense_recording_is_a_remember_fact_claim_and_a_bare_verb_is_not():
    """«Записываю: …» — то же утверждение, что «Запомнил». Голый глагол и отрицание — нет (``words._fact_excluded``)."""
    assert done_claims(RECORDING) == ("remember_fact",)
    assert done_claims("Запоминаю про тебя: любишь чай.") == ("remember_fact",)
    assert done_claims("Сохраняю, что ты приносишь пиццу.") == ("remember_fact",)
    assert done_claims("Записываю") == ()
    assert done_claims("Не записываю.") == ()


def test_a_rejected_dictation_does_not_voice_the_model_recording_claim():
    """Живой ход #5215: memory_save → rejected{no_person}, в TTS только фраза исхода, не «Записываю: …»."""
    ports = _run(
        HEARD,
        RECORDING,
        actions=(_save(),),
        outcomes={kn.MEMORY_SAVE_TOOL: ("rejected", kn.NO_PERSON_REASON, {})},
    )
    assert ports.said == [REFUSAL]
    assert RECORDING not in ports.said
    assert FACT not in " ".join(ports.said)


def test_a_timed_out_dictation_does_not_voice_the_model_recording_claim():
    """Тот же класс: срок вышел — фраза исхода, речь модели о записи не звучит."""
    ports = _run(
        HEARD,
        RECORDING,
        actions=(_save(),),
        outcomes={kn.MEMORY_SAVE_TOOL: ("timeout", None, {})},
    )
    assert ports.said == [phrases.text("local.timeout")]
    assert RECORDING not in ports.said


def test_a_done_dictation_still_says_the_catalog_phrase_from_the_result():
    """Негатив: успех записи звучит как раньше — каталог из факта результата, не речь модели."""
    ports = _run(
        HEARD,
        RECORDING,
        actions=(_save(fact=FACT.lower()),),
        outcomes={kn.MEMORY_SAVE_TOOL: ("done", None, {"fact": FACT.lower()})},
    )
    assert ports.said == [phrases.text("memory.saved", fact=FACT.lower())]
    assert RECORDING not in ports.said


def test_a_refused_tool_keeps_speech_that_does_not_claim_the_action():
    """Отказ тула не глушит реплику, которая не про это действие: человек слышит и отказ, и ответ."""
    ports = _run(
        HEARD,
        OTHER,
        actions=(_save(),),
        outcomes={kn.MEMORY_SAVE_TOOL: ("rejected", kn.NO_PERSON_REASON, {})},
    )
    assert ports.said == [REFUSAL, OTHER]
