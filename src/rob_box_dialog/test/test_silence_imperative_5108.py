"""#5108: «Робот, молчи» / «Робот, молчите» — просьба тишины, как «помолчи».

Tier-1 знал тишину только основами ``SILENCE_WORDS`` («помолч», «замолч»): «молчи» без приставки проходила мимо,
ход уходил в LLM, и голая ``expression`` становилась долгом слов. Основой «молч» это не чинится — она поймала бы
«молчит», «молчание», «молчаливый». Решает код (ADR-0148): точные формы ``knowledge.SILENCE_EXACT_FORMS`` рядом с
основами, сверка целым словом в ``grammar._stop``. Решения Шифу 23.10 («тишину снимает только "Робот, говори"»)
и 22.10 («тишина — отдельно, по "замолчи"»).

Синтетические реплики (репо публичный: никаких данных людей).
"""

import asyncio

import pytest
from robot_wake_fixture import ROBOT_WAKE

from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command
from rob_box_dialog.grammar import Tier1Command, match
from rob_box_dialog.session import Session, SessionSnapshot
from rob_box_dialog.turn import Answer, TurnEngine
from rob_box_dialog.words import STOP_CLEAR, stop_command


def _snap():
    return SessionSnapshot(
        agent="personality",
        session_id="t",
        epoch=0,
        started_at=0.0,
        wake_words=ROBOT_WAKE,
    )


class Ports:
    """Транспорт хода: ответы LLM по очереди, отмены речи и стадии — наружу."""

    def __init__(self, answers=()):
        self.answers = list(answers)
        self.asked, self.said, self.cancels, self.stages = [], [], [], []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append(list(messages))
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        return []

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append(text)
        return f"sp{len(self.said)}"

    def stage(
        self,
        turn_id,
        epoch,
        stage,
        status,
        reason="",
        cls=None,
        llm_calls=0,
        provider="",
        source="",
        expression=None,
    ):
        self.stages.append((stage, status, reason))

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        self.cancels.append(reason)

    def stop_perform(self, how):
        return False

    def intent(self, turn_id, epoch, kind):
        pass

    def perform(self, turn_id, epoch, perf, edit):
        return None


def _run(ports, text):
    engine = TurnEngine(
        Session("personality", wake_words=ROBOT_WAKE), ports, system_prompt="Ты робот."
    )
    agg = asyncio.run(engine.run(Addressed(text, VOICE)))
    return engine, agg


def test_bare_molchi_is_tier1_silence_and_calls_no_llm():
    """«Робот, молчи» — тишина кодом: LLM не зовётся, речи нет, кадр Silence включён."""
    ports = Ports()
    engine, agg = _run(ports, "Робот, молчи")
    assert match("Робот, молчи", _snap()) == Tier1Command("silence")
    assert ports.asked == [] and ports.said == [] and agg.llm_calls == 0
    assert ports.cancels == ["tier1 silence"]
    assert engine.session.snapshot.top("Silence") is not None
    assert (agg.stage, agg.status) == ("done", "finished")


def test_molchite_with_please_is_still_silence():
    """«Робот, молчите, пожалуйста» — та же тишина: филлер «пожалуйста» просьбу не отменяет."""
    ports = Ports()
    engine, agg = _run(ports, "Робот, молчите, пожалуйста")
    assert match("Робот, молчите, пожалуйста", _snap()) == Tier1Command("silence")
    assert ports.asked == [] and agg.llm_calls == 0
    assert engine.session.snapshot.top("Silence") is not None


@pytest.mark.parametrize("text", ["Робот, он молчит", "Робот, что такое молчание"])
def test_molchit_and_molchanie_are_not_silence(text):
    """«Он молчит» и «что такое молчание» — не просьба: основа «молч» запрещена, ход обычный, LLM зовётся."""
    assert match(text, _snap()) is None
    spoken = Command(speech="Обычный ответ.")
    ports = Ports([Answer(spoken, "deepseek")])
    engine, agg = _run(ports, text)
    assert len(ports.asked) == 1 and agg.llm_calls == 1
    assert engine.session.snapshot.top("Silence") is None
    assert ports.cancels == []
    assert ports.said == ["Обычный ответ."]


def test_molchi_with_a_command_tail_is_stop_all_keeping_the_rest():
    """«Робот, молчи, спой песню» — не чистая тишина: хвост команды снимает тишину, как у «помолчи, спой песню»
    и «замолчи и включи свет» (``grammar._stop``: тишина только без rest). Стоп всего кодом, остаток — в LLM.
    """
    assert match("Робот, помолчи, спой песню", _snap()) == Tier1Command(
        "stop_all", rest="спой песню"
    )
    assert match("Робот, молчи, спой песню", _snap()) == Tier1Command(
        "stop_all", rest="спой песню"
    )
    ports = Ports([Answer(Command(speech="Пою."), "deepseek")])
    engine, agg = _run(ports, "Робот, молчи, спой песню")
    assert ports.cancels == ["tier1 stop_all"]
    assert engine.session.snapshot.top("Silence") is None
    assert len(ports.asked) == 1 and ports.asked[0][-1]["content"].endswith(
        "спой песню"
    )
    assert agg.llm_calls == 1 and ports.said == ["Пою."]


@pytest.mark.parametrize(
    "text, kind",
    [
        ("Робот, помолчи", "silence"),
        ("Робот, замолчи", "silence"),
        ("Робот, стоп", "stop_all"),
    ],
)
def test_old_silence_and_stop_words_are_unchanged(text, kind):
    """«Помолчи», «замолчи», «стоп» работают как раньше: ясный стоп, без LLM."""
    assert match(text, _snap()) == Tier1Command(kind)
    assert stop_command(text, ROBOT_WAKE).verdict == STOP_CLEAR
    ports = Ports()
    engine, agg = _run(ports, text)
    assert ports.asked == [] and agg.llm_calls == 0
    if kind == "silence":
        assert engine.session.snapshot.top("Silence") is not None
    else:
        assert engine.session.snapshot.top("Silence") is None


def test_exact_forms_are_whole_words_not_a_stem():
    """Знание живёт в ``knowledge``: точные формы, не основа. «Молчит» и «молчание» в таблицу не входят."""
    assert kn.SILENCE_EXACT_FORMS == ("молчи", "молчите")
    assert set(kn.SILENCE_EXACT_FORMS) <= set(kn.STOP_WORDS)
    for stem in kn.SILENCE_WORDS:
        assert not any(form.startswith(stem) for form in kn.SILENCE_EXACT_FORMS)
    for lookalike in ("молчит", "молчание", "молчаливый", "молчал"):
        assert (
            lookalike not in kn.STOP_WORDS and lookalike not in kn.SILENCE_EXACT_FORMS
        )
