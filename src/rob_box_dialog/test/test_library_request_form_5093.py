"""#5093 (марафон #3 акт 7 n705): просьба найти в библиотеке не принимает ``memory_save`` как исполнение.

Модель на «найди у себя в библиотеке что-нибудь про дождь» позвала ``memory_save`` фактом прошлого акта, хотя
``music_library`` был в срезе. Форма хода закрывает ``memory_save`` структурой (ADR-0148 п.2): тул вне среза и вне
команды, нарушение своё (``library_request``), не ``tool_not_available`` — мост по тому срез расширяет. «Запомни,
что…» глагола поиска не содержит: запись памяти остаётся открытой.
"""

import asyncio
import logging

from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, Form, ToolCall, current_form, explain, parse, validate
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session
from rob_box_dialog.tools_view import available_tools, library_asked
from rob_box_dialog.turn import Answer, TurnEngine, _Turn, command_form, form_tools

N705 = "робот найди у себя в библиотеке что нибудь про дождь"  # STT шага n705, без вейка в тексте хода
FIND = "найди у себя в библиотеке что-нибудь про дождь"
REMEMBER = "запомни, что гость пьет только зеленый чай"
SAVE = {"tool": "memory_save", "args": {"fact": "гость любит дождь"}}
SEARCH = {"tool": "music_library", "args": {"action": "search", "query": "дождь"}}
MUSIC = ["memory_save", "memory_search", "music_library", "search_web"]


def _check(raw, form, tools=MUSIC):
    return validate(parse(raw), tools, form=form)


def test_n705_phrase_closes_memory_save_and_keeps_the_library_tool():
    assert library_asked(N705) and library_asked(FIND)
    form = Form(passed=False, library=True)
    closed = form.closed_tools()
    assert "memory_save" in closed and "music_library" not in closed
    assert Form().closed_tools() == ()  # вне хода (v1, тесты) запись памяти открыта


def test_remember_phrase_does_not_close_memory_save():
    assert not library_asked(REMEMBER)
    assert not library_asked("запомни, что в библиотеке есть трек про дождь")
    assert "memory_save" not in Form(passed=False).closed_tools()


def test_memory_save_on_a_library_request_is_library_request_not_tool_not_available():
    result = _check({"actions": [SAVE]}, Form(passed=False, library=True))
    reasons = [v.reason for v in result.all]
    assert kn.VIOLATION_LIBRARY_REQUEST in reasons and "tool_not_available" not in reasons, reasons
    assert not any(c.tool == "memory_save" for c in result.salvage)
    note = explain(result)
    assert kn.VIOLATION_LIBRARY_REQUEST in note and "music_library" in note and "memory_save" in note


def test_music_library_on_a_library_request_stays_a_command():
    cmd = _check({"actions": [SEARCH]}, Form(passed=False, library=True))
    assert isinstance(cmd, Command) and cmd.actions[0].tool == "music_library"


def test_remember_turn_still_accepts_memory_save():
    cmd = _check({"actions": [SAVE]}, Form(passed=False))
    assert isinstance(cmd, Command) and cmd.actions[0].tool == "memory_save"


def test_form_tools_drop_memory_save_from_the_offered_slice():
    snap = Session("personality").snapshot
    offered = available_tools(snap, None, "personality", FIND)
    names = [t.name for t in form_tools(offered, Form(passed=False, library=True), "personality")]
    assert "memory_save" not in names and "music_library" in names
    plain = [t.name for t in form_tools(offered, Form(passed=False), "personality")]
    assert "memory_save" in plain  # «запомни» срез не сужает


def test_command_form_of_the_turn_sets_library_from_the_utterance():
    snap = Session("personality").snapshot
    find = command_form(_Turn("t1", 0, FIND, VOICE, deadline=None), snap)
    assert find.library and "memory_save" in find.closed_tools()
    remember = command_form(_Turn("t1", 0, REMEMBER, VOICE, deadline=None), snap)
    assert not remember.library and "memory_save" not in remember.closed_tools()


class _Ports:
    def __init__(self, answers):
        self.answers, self.asked, self.forms = list(answers), [], []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append([t.name for t in tools])
        self.forms.append(current_form())
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
        return "sp"

    def stage(self, *args, **kwargs):
        pass

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        pass

    def stop_perform(self, how):
        return False

    def intent(self, turn_id, epoch, kind):
        pass


def test_library_turn_logs_the_closed_tool_and_offers_no_memory_save(caplog):
    search = Answer(Command(actions=(ToolCall("music_library", SEARCH["args"]),)))
    ports = _Ports([search, Answer(Command(speech="В библиотеке есть трек про дождь."))])
    engine = TurnEngine(Session("personality"), ports, system_prompt="Ты робот.")
    with caplog.at_level(logging.INFO):
        asyncio.run(engine.run(Addressed(FIND, VOICE)))
    assert ports.forms[0].library and all("memory_save" not in asked for asked in ports.asked)
    assert "music_library" in ports.asked[0]
    assert any(
        "форма команды:" in r.getMessage() and "тулы=['memory_save']" in r.getMessage() and "закрыты=" in r.getMessage()
        for r in caplog.records
    )
