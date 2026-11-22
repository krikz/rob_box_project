"""#5105 / #5115: «Робот, тишина» — просьба молчать, и Tier-1 её знает.

Живой факт (#5105, образ dev-9dd577c, turn ``a2ab8c87d268``): ответ одним лицом ``expression:thinking`` стал нарушением
``silent_answer``, повтор заставил модель сказать «Тишина в эфире. Я тоже помолчу и постою рядом.». #5105 снял долг
слов, но ход всё равно звал LLM (ожидание ``llm_calls==1``): существительного «тишина» в ``SILENCE_WORDS`` нет, и
``_stop`` возвращал ``None`` до проверки тишины. Живой прогон #5115: «Робот, тишина» ушло в модель, робот говорил, сцена
Silence не ставилась. Теперь та же таблица (``words.silence_noun_asked``) — дверь Tier-1: 0 вызовов LLM, 0 TTS, сцена
Silence. Старое ожидание ``llm_calls==1`` закрепляло обход, а не тишину.
"""

import asyncio

import rob_box_dialog.phrases as phrases
from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, Expression, Invalid, ToolCall
from rob_box_dialog.respond import ActionResult
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
        return [
            ActionResult(
                f"{turn_id}/{n}",
                snapshot.epoch,
                snapshot.agent,
                a.tool,
                kn.TOOL_CLASSES[a.tool],
                "done",
                None,
                dict(a.args),
            )
            for n, a in enumerate(actions)
        ]

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
    """«Робот, тишина» — Tier-1 (#5115): LLM не зовётся, речи нет, сцена Silence стоит. Старое ожидание ``llm_calls==1``
    закрепляло обход: долг слов снят, а тишину решает модель."""
    ports = Ports([Answer(THINKING, "deepseek")])
    engine = TurnEngine(Session("personality", wake_words=WAKE), ports, system_prompt="Ты робот.")
    agg = asyncio.run(engine.run(Addressed(N_SILENCE, VOICE)))
    assert ports.asked == [] and agg.llm_calls == 0, "тишина решает код, не модель"
    assert ports.said == [], "ход закрывается без речи"
    assert engine.session.snapshot.top("Silence") is not None
    assert (agg.stage, agg.status) == ("done", "finished")
    assert ports.stages[-1] == ("done", "finished", "")


def test_silence_noun_with_please_is_still_a_silence_ask():
    """«Робот, тишина, пожалуйста» — та же Tier-1 тишина: филлер «пожалуйста» просьбу не отменяет, LLM не зовётся."""
    ports = Ports([Answer(THINKING, "deepseek")])
    agg = _run(ports, N_PLEASE)
    assert ports.asked == [] and agg.llm_calls == 0 and ports.said == []
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


#: Спасение невалидной команды одним неслышимым тулом: успех ``play_animation`` молчит (``SILENT_DONE_TOOLS``),
#: результат непуст — и ``_salvaged_phrase`` доходит до ``words_owed``, а не до фразы «исполнять нечего».
BARE_SHOW = (ToolCall("play_animation", {"animation": "thinking"}),)


def test_invalid_command_salvaged_by_a_bare_expression_stays_silent_on_a_silence_ask():
    """«Робот, тишина» до спасения не доходит (#5115): Tier-1 закрывает ход до LLM, сцена Silence стоит, фразы нет.
    Спасение упоминания («стих про тишину») проверяет соседний тест — тот путь в LLM остался."""
    bare = Invalid("bad_json", salvage=BARE_SHOW)
    ports = Ports([Answer(bare, "deepseek", calls=2)])
    engine = TurnEngine(Session("personality", wake_words=WAKE), ports, system_prompt="Ты робот.")
    agg = asyncio.run(engine.run(Addressed(N_SILENCE, VOICE)))
    assert ports.asked == [] and agg.llm_calls == 0
    assert ports.said == [], "просьба о тишине — не долг слов, фразы нарушения нет"
    assert engine.session.snapshot.top("Silence") is not None
    assert (agg.stage, agg.status) == ("done", "finished")


def test_invalid_command_salvaged_by_a_bare_expression_still_speaks_on_a_mention():
    """Контроль того же пути: «Робот, расскажи стих про тишину» — упоминание, не просьба, и код говорит фразу
    нарушения, как и до #5105."""
    bare = Invalid("bad_json", salvage=BARE_SHOW)
    ports = Ports([Answer(bare, "deepseek", calls=2)])
    agg = _run(ports, N_VERSE)
    assert ports.said == [phrases.text(kn.INVALID_COMMAND_PHRASE)]
    assert (agg.stage, agg.status) == ("done", "done")
