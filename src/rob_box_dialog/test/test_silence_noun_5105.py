"""#5105: «Робот, тишина» — просьба молчать, а не долг слов.

Живой факт (образ dev-9dd577c, turn ``a2ab8c87d268``, синтетическая реплика «Робот, тишина»): ответ одним лицом
``expression:thinking`` стал нарушением ``silent_answer``, повтор заставил модель сказать «Тишина в эфире. Я тоже
помолчу и постою рядом.». Tier-1 тишины знает только основы ``SILENCE_WORDS`` («помолч», «замолч»), а ``silent_asked``
— «молча»/«без слов», и существительное «тишина» проходило мимо обоих. Решает код (ADR-0148): таблица форм в
``knowledge``, предикат ``tools_view.silence_asked`` снимает вейк хода и считает просьбой фразу, где кроме «тишина»/
«тишину»/«тишины»/«тишине»/«полная тишина» только служебные филлеры.
"""

import asyncio

from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, Expression
from rob_box_dialog.session import Session
from rob_box_dialog.tools_view import needs_words, silence_asked
from rob_box_dialog.turn import Answer, TurnEngine, _Turn, owed_reason

#: Синтетические реплики (репо публичный: никаких данных людей).
N_SILENCE = "Робот, тишина"
N_PLEASE = "Робот, тишина, пожалуйста"
N_VERSE = "Робот, расскажи стих про тишину"
N_SPEAK = "Робот, не молчи"
WAKE = ("робот",)
THINKING = Command(expression=Expression(emotion="thinking"))


class Ports:
    """Транспорт хода: ответы LLM по очереди, речь и стадии — наружу."""

    def __init__(self, answers=()):
        self.answers = list(answers)
        self.asked, self.said, self.stages = [], [], []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append(list(messages))
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        return []

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append(text)
        return f"sp{len(self.said)}"

    def stage(
        self, turn_id, epoch, stage, status, reason="", cls=None, llm_calls=0, provider="", source="", expression=None
    ):
        self.stages.append((stage, status, reason))

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        pass

    def stop_perform(self, how):
        return False

    def intent(self, turn_id, epoch, kind):
        pass

    def perform(self, turn_id, epoch, perf, edit):
        return None


def _run(ports, text):
    engine = TurnEngine(Session("personality", wake_words=WAKE), ports, system_prompt="Ты робот.")
    return asyncio.run(engine.run(Addressed(text, VOICE)))


def test_silence_noun_with_only_an_expression_closes_without_speech():
    """«Робот, тишина» + одно лицо ``thinking`` — не нарушение ``silent_answer``: один вызов LLM, речи нет, ход закрыт."""
    ports = Ports([Answer(THINKING, "deepseek")])
    agg = _run(ports, N_SILENCE)
    assert len(ports.asked) == 1 and agg.llm_calls == 1, "нарушения нет — повторного вызова нет"
    assert ports.said == [], "ход закрывается без речи"
    assert (agg.stage, agg.status) == ("done", "finished")
    assert ports.stages[-1] == ("done", "finished", "")


def test_silence_noun_with_please_is_still_a_silence_ask():
    """«Робот, тишина, пожалуйста» — та же просьба: филлер «пожалуйста» просьбу не отменяет."""
    ports = Ports([Answer(THINKING, "deepseek")])
    agg = _run(ports, N_PLEASE)
    assert len(ports.asked) == 1 and ports.said == []
    assert (agg.stage, agg.status) == ("done", "finished")


def test_a_verse_about_silence_still_owes_words():
    """«Робот, расскажи стих про тишину» — упоминание, не просьба: голая ``expression`` остаётся нарушением
    ``silent_answer``, повтор просит слова."""
    spoken = Command(speech="Стих про тишину.")
    ports = Ports([Answer(THINKING, "deepseek"), Answer(spoken, "deepseek")])
    _run(ports, N_VERSE)
    assert len(ports.asked) == 2, "нарушение есть — модель зовут повторно"
    assert kn.VIOLATION_SILENT_ANSWER in ports.asked[1][-1]["content"]
    assert ports.said == ["Стих про тишину."]


def test_do_not_be_silent_still_expects_words():
    """«Робот, не молчи» — просит говорить, не молчать: голый ответ лицом по-прежнему долг слов."""
    spoken = Command(speech="Говорю.")
    ports = Ports([Answer(THINKING, "deepseek"), Answer(spoken, "deepseek")])
    _run(ports, N_SPEAK)
    assert len(ports.asked) == 2
    assert kn.VIOLATION_SILENT_ANSWER in ports.asked[1][-1]["content"]
    assert ports.said == ["Говорю."]


def test_silence_asked_is_a_request_not_a_mention():
    """Предикат узкий: формы «тишина»/«тишину»/«полная тишина» с филлерами — просьба; упоминание и «не молчи» — нет.
    Вейк снимает таблица хода, не зашитое слово."""
    for phrase in (N_SILENCE, N_PLEASE, "полная тишина", "давай тишину", "ну тишины", "просто режим тишина"):
        assert silence_asked(phrase, WAKE), phrase
    assert not needs_words(N_SILENCE, THINKING, WAKE)
    for phrase in (N_VERSE, "что такое тишина", N_SPEAK, "включи тишину", "тишина в эфире"):
        assert not silence_asked(phrase, WAKE), phrase
    assert needs_words(N_VERSE, THINKING, WAKE) and needs_words(N_SPEAK, THINKING, WAKE)


def _turn(text):
    turn = _Turn("t1", 0, text, VOICE, deadline=None)
    turn.wake_words = WAKE
    return turn


def test_owed_reason_names_no_violation_for_a_silence_ask():
    """Порядок нарушений не поехал: просьба о тишине не даёт ``silent_answer``, упоминание — даёт."""
    session = Session("personality", wake_words=WAKE)
    assert owed_reason(session.snapshot, _turn(N_SILENCE), THINKING) == ""
    assert owed_reason(session.snapshot, _turn(N_VERSE), THINKING) == kn.VIOLATION_SILENT_ANSWER
