"""#5221: после отказа register_speaker речь модели о регистрации голоса не звучит (ADR-0148).

Живой ход 24.11 (Sound Pi, образ dev-c3fd361d8, ядро 0.7.32, ход a04110432633): модель сказала
«Приятно познакомиться, Борис! Голос твой запоминаю.» в том же ходе, где ``register_speaker``
закрылся ``rejected no_utterance_context``. Код сказал «Не смог запомнить голос.», следом
прозвучала речь модели (speech_id=ac065945). ``done_claims`` и ``recording_claims["remember_fact"]``
эту фразу не узнают: таблица факта ключуется на ``memory_save``.

Фразу исхода говорит код из результата. Речь, утверждающая несделанную регистрацию, не озвучивается.
Регекса по ответу нет: вид ``knowledge.RECORDING_CLAIMS["remember_voice"]`` (тул
``knowledge.NAME_OWNER_TOOL``) и исход тула. В ``DONE_CLAIMS`` вида нет — фраза плана «Запоминаю: …»
законна (#4403), фразы кода знакомства валидатор не судит.

Успех регистрации рядом со ``speech`` код не озвучивает (``QUIET_WITH_SPEECH_TOOLS``, как n722):
речь модели о запоминании голоса при ``done`` остаётся, фразу «Запомнил тебя» код не добавляет.
Отказ — другое: код уже сказал исход, и утверждение о несделанном не звучит.
"""

import asyncio

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session
from rob_box_dialog.turn import Answer, TurnEngine
from rob_box_dialog.words import done_claims, recording_claims

HEARD = "меня зовут Борис"
CLAIM = "Приятно познакомиться, Борис! Голос твой запоминаю."
GREETING = "Приятно познакомиться, Борис!"
BARE = "Приятно познакомиться!"
REFUSAL = phrases.text("identity.rejected")
PLAN = phrases.text("chain.plan.save", items="хил-один, хил-два")


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


def _register(name="Борис"):
    return ToolCall(kn.NAME_OWNER_TOOL, {"name": name})


def test_voice_registration_claim_is_not_a_fact_claim_and_not_a_done_claim():
    """«Голос твой запоминаю» — вид голоса, не факт и не ``done_claims``.

    Фраза плана «Запоминаю: …» голоса не несёт и ни тем, ни другим не ловится (#4403).
    Приветствие без утверждения о голосе — не утверждение.
    """
    assert recording_claims(CLAIM) == ("remember_voice",)
    assert recording_claims("Запоминаю твой голос.") == ("remember_voice",)
    assert recording_claims("Записываю голос.") == ("remember_voice",)
    assert done_claims(CLAIM) == ()
    assert recording_claims(PLAN) == ("remember_fact",)  # план факта, не голоса
    assert "remember_voice" not in recording_claims(PLAN)
    assert done_claims(PLAN) == ()
    assert recording_claims(GREETING) == ()
    assert recording_claims("Не запоминаю голос.") == ()
    assert recording_claims("Запоминаю.") == ()


def test_a_rejected_registration_does_not_voice_the_model_voice_claim():
    """Живой ход #5221: register_speaker → rejected, в TTS только фраза исхода, не «Голос твой запоминаю»."""
    ports = _run(
        HEARD,
        CLAIM,
        actions=(_register(),),
        outcomes={kn.NAME_OWNER_TOOL: ("rejected", "no_utterance_context", {"name": "Борис"})},
    )
    assert ports.said == [REFUSAL]
    assert CLAIM not in ports.said
    assert "запоминаю" not in " ".join(ports.said)


def test_a_failed_and_a_timed_out_registration_do_not_voice_the_model_voice_claim():
    """Тот же класс: провал и срок вышли — речь модели о голосе не звучит.

    ``failure`` у ``action.identity`` фразы не имеет (ключи класса — done/rejected/timeout) и в
    ``_FAILED`` хода не входит: код молчит, но утверждение «голос запоминаю» всё равно не звучит.
    Срок — фраза ``identity.timeout``.
    """
    failed = _run(
        HEARD,
        CLAIM,
        actions=(_register(),),
        outcomes={kn.NAME_OWNER_TOOL: ("failure", None, {})},
    )
    assert failed.said == []
    assert CLAIM not in failed.said
    timed = _run(
        HEARD,
        CLAIM,
        actions=(_register(),),
        outcomes={kn.NAME_OWNER_TOOL: ("timeout", None, {})},
    )
    assert timed.said == [phrases.text("identity.timeout")]
    assert CLAIM not in timed.said


def test_a_done_registration_keeps_the_model_speech_and_code_stays_quiet():
    """Успех: код фразу «Запомнил тебя» не добавляет (``QUIET_WITH_SPEECH_TOOLS``), речь модели звучит целиком.

    Как n722: одна реплика — слова модели. Гасить её и говорить успех кодом было бы второй репликой
    об одном, чего путь ``memory_save`` делает только потому, что код сам строит подтверждение факта.
    """
    ports = _run(
        HEARD,
        CLAIM,
        actions=(_register(),),
        outcomes={kn.NAME_OWNER_TOOL: ("done", None, {"name": "Борис"})},
    )
    assert ports.executed == [[(kn.NAME_OWNER_TOOL, {"name": "Борис"})]]
    assert ports.said == [CLAIM]
    assert phrases.text("identity.done", name="Борис") not in ports.said


def test_a_refused_registration_keeps_speech_that_does_not_claim_the_voice():
    """Отказ регистрации не глушит приветствие без утверждения о голосе (#5172)."""
    ports = _run(
        HEARD,
        BARE,
        actions=(_register(),),
        outcomes={kn.NAME_OWNER_TOOL: ("rejected", "no_utterance_context", {"name": "Борис"})},
    )
    assert ports.said == [REFUSAL, BARE]


def test_a_rejected_fact_save_still_drops_only_the_fact_claim():
    """Путь #5215 не изменился: отказ memory_save гасит «Записываю: …», фраза исхода — код."""
    fact = "Борис болеет за Спартак и всегда приносит пиццу"
    recording = f"Записываю: {fact}"
    heard = "Робот, запомни про меня борис болеет за спартак и всегда приносит пиццу"
    save = ToolCall(kn.MEMORY_SAVE_TOOL, {"fact": fact, "category": "general"})
    ports = _run(
        heard,
        recording,
        actions=(save,),
        outcomes={kn.MEMORY_SAVE_TOOL: ("rejected", kn.NO_PERSON_REASON, {})},
    )
    assert ports.said == [phrases.text("local.rejected.no_person")]
    assert recording not in ports.said
