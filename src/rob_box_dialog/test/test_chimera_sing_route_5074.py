"""#5074 — «спой … химерой» доходит до ``chimera_perform`` структурой формы, не текстом ответа (ADR-0148).

Глагол пения без дополнения-звука открывает группу ``music``. Вместе с всегда-видимыми тулами она заполняет
``TOOLS_VISIBLE_MAX``, и ``chimera_perform`` из группы ``sound`` (стебель «химер» её уже открыл) в срез не входит.
Живая семья при этом требует ``perform`` и закрывает генерацию. Форма хода для такой реплики — семья ``chimera``:
``needs_tool="chimera_perform"``, ``needs_perform=False``, тул не в ``closed_tools``. ``form_tools`` добавляет
требуемый тул, даже если срез его срезал.

Голое «спой» и «спой песню» остаются живым пением. «Покажи химеру» пением не становится: путь пуст, срез звука
как был. Новой регулярки нет — стебель тот же, что в ``SOUND_STEMS`` (``knowledge.chimera_named``).
"""

import asyncio

from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, ToolCall, current_form, form_schema, parse, validate
from rob_box_dialog.owners import OWNERS
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session, SessionSnapshot
from rob_box_dialog.tools_view import available_tools
from rob_box_dialog.turn import Answer, TurnEngine, command_form, form_tools

CHIMERA = "Робот, спой химерой песню про енота"
SONG = "Робот, спой песню"
SHOW = "Робот, покажи химеру"
BARE = "Робот, спой"
CALL = {"tool": kn.CHIMERA_TOOL, "args": {"action": "sing", "title": "енот"}}


def _snap() -> SessionSnapshot:
    return SessionSnapshot(agent="personality", session_id="t", epoch=0, started_at=0.0, owners=OWNERS)


def _form(text: str, sing: str):
    from rob_box_dialog.turn import _Turn

    turn = _Turn("t1", 0, text, VOICE, deadline=None)
    return command_form(turn, _snap(), sing=sing)


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

    def perform(self, turn_id, epoch, perf, edit):
        return {"status": "pending"}


class _Router:
    """Хост пути пения, как ``choose_sing`` на голый глагол пения: живое пение, id не смотрит на «химер»."""

    def __call__(self, text, recent, playing):
        return "live:song"


def test_sing_by_chimera_form_requires_the_chimera_tool_not_a_performance():
    """«Спой химерой …»: форма требует тул Химеры, живое пение не обязательно, perform не в required схемы.

    Срез музыки тул вытесняет (12 имён без него); выдача формы его возвращает и не закрывает.
    """
    form = _form(CHIMERA, "live:song")
    sing = form.sing_form
    assert form.text == CHIMERA and form.sing == "live:song"
    assert sing is not None and sing.needs_tool == kn.CHIMERA_TOOL and sing.needs_perform is False
    assert kn.CHIMERA_TOOL not in sing.closed_tools and kn.CHIMERA_TOOL not in form.closed_tools()
    schema = form_schema(kn.command_schema("json"), form)
    assert "perform" not in schema.get("required", [])
    names = [t.name for t in available_tools(_snap(), text=CHIMERA)]
    assert kn.CHIMERA_TOOL not in names  # музыка заполнила срез — без формы тула нет
    offered = [t.name for t in form_tools(available_tools(_snap(), text=CHIMERA), form, "personality", OWNERS)]
    assert kn.CHIMERA_TOOL in offered
    assert isinstance(validate(parse({"actions": [CALL]}), offered, form=form), Command)
    speech = validate(parse({"speech": "Сейчас спою!"}), offered, form=form)
    assert speech.reason == kn.VIOLATION_SING_ROUTE


def test_sing_a_song_stays_live_singing():
    """«Спой песню» без Химеры — живое пение: perform обязателен, тул Химеры форма не требует."""
    form = _form(SONG, "live:song")
    sing = form.sing_form
    assert kn.sing_family(form.sing, text=form.text) == "live"
    assert sing is not None and sing.needs_perform is True and sing.needs_tool == ""
    schema = form_schema(kn.command_schema("json"), form)
    assert "perform" in schema["required"]
    assert schema["properties"]["perform"]["properties"]["kind"]["enum"] == ["song"]
    assert kn.CHIMERA_TOOL not in form.closed_tools()


def test_show_chimera_is_not_a_sing_form_and_the_slice_keeps_the_tool():
    """«Покажи химеру» — не пение: путь пуст, семья пуста, срез звука по-прежнему содержит тул."""
    form = _form(SHOW, "")
    assert form.sing == "" and form.sing_form is None
    assert kn.sing_family(form.sing, text=form.text) == ""
    names = [t.name for t in available_tools(_snap(), text=SHOW)]
    assert kn.CHIMERA_TOOL in names
    offered = [t.name for t in form_tools(available_tools(_snap(), text=SHOW), form, "personality", OWNERS)]
    assert kn.CHIMERA_TOOL in offered  # форма пустого пути тул не закрыла и не потребовала заново


def test_bare_sing_without_objects_stays_live_singing():
    """«Спой» без объектов и без «химер» — живое пение, как до #5074."""
    form = _form(BARE, "live:song")
    assert kn.sing_family(form.sing, text=form.text) == "live"
    assert form.sing_form is not None and form.sing_form.needs_perform is True
    assert form.sing_form.needs_tool == ""
    assert "perform" in form_schema(kn.command_schema("json"), form)["required"]


def test_the_turn_keeps_the_live_route_id_and_offers_chimera_perform():
    """Ход целиком: хост по-прежнему отдаёт ``live:song``, форма по реплике — семья ``chimera``, тул в выдаче модели."""
    ports = _Ports([Answer(Command(actions=(ToolCall(kn.CHIMERA_TOOL, CALL["args"]),)))])
    engine = TurnEngine(Session("personality", owners=OWNERS), ports, system_prompt="Ты робот.", sing=_Router())
    asyncio.run(engine.run(Addressed(CHIMERA, VOICE)))
    form = ports.forms[-1]
    assert form.sing == "live:song" and form.text == CHIMERA
    assert form.sing_form is not None and form.sing_form.needs_tool == kn.CHIMERA_TOOL
    assert form.sing_form.needs_perform is False
    assert kn.CHIMERA_TOOL in ports.asked[-1]
    assert "perform" not in form_schema(kn.command_schema("json"), form).get("required", [])
