"""#4079 — путь пения до основной модели: форма хода сужает команду структурой (ADR-0148 п.2, решение Шифу 27.10).

Путь решает код хоста (``rob_box_music.source.choose_sing``: маркеры слов, чойсер с перевесом живому) и отдаёт ходу id
кандидата; ход кладёт его в форму команды (``command.Form.sing``):

* ``live:song``/``live:rap`` — ``generate_music`` и ``request_music`` вне среза и вне команды, ``perform`` обязателен,
  вид — из пути, подложка — синт (``none`` код меняет на бит, поле в ``dropped``);
* ``generate:…`` — ``generate_music`` обязателен (в срезе, даже если его срезала обрезка), песня выступлением — нарушение;
* пути нет (не пение, хост без пути пения — чат, оператор) — всё как раньше.

Нарушение формы — своё (``sing_route``), не «тула нет в срезе»: мост его срез не расширяет (``tools_view.widen_for``).
"""

import asyncio

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, Form, Performance, ToolCall, current_form, form_schema, parse, validate
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session
from rob_box_dialog.turn import Answer, TurnEngine, form_tools
from rob_box_dialog.tools_view import available_tools

LIVE, RAP, GEN = Form(passed=False, sing="live:song"), Form(passed=False, sing="live:rap"), Form(
    passed=False, sing="generate:robot_song:V6")
MUSIC = ["stop_music", "get_music_state", "request_music", "dj_set", "music_library", "generate_music"]
SONG = {"kind": "song", "segments": ["Енот-полоскун у реки", "моет лапки до луны"]}
GEN_CALL = {"tool": "generate_music", "args": {"kind": "robot_song", "theme": "енот"}}


def _check(raw, form, tools=MUSIC):
    return validate(parse(raw), tools, form=form)


# -- схема: перечни выступления и обязательный perform ------------------------------------------------------------


def test_live_form_schema_requires_perform_of_the_route_kind_on_a_synth_backing():
    schema = form_schema(kn.command_schema("json"), LIVE)
    perform = schema["properties"]["perform"]["properties"]
    assert "perform" in schema["required"]
    assert perform["kind"]["enum"] == ["song"] and perform["backing"]["enum"] == ["beat", "calm"]
    assert form_schema(kn.command_schema("json"), RAP)["properties"]["perform"]["properties"]["kind"]["enum"] == ["rap"]
    line = next(x for x in kn.prompt_fragment(schema).splitlines() if x.startswith("- perform"))
    assert "kind ∈ {song}" in line and "backing ∈ {beat, calm}" in line  # промпт — из той же схемы хода


def test_generate_form_schema_has_no_song_or_rap_performance():
    schema = form_schema(kn.command_schema("json"), GEN)
    assert schema["properties"]["perform"]["properties"]["kind"]["enum"] == ["tale", "poem"]
    assert "perform" not in schema.get("required", [])


def test_no_sing_route_leaves_the_schema_as_it_was():
    plain = kn.without_fields(kn.command_schema("json"), Form(passed=False).closed())
    assert form_schema(kn.command_schema("json"), Form(passed=False)) == plain


# -- валидатор ------------------------------------------------------------------------------------------------------


def test_live_route_takes_the_performance_and_puts_the_synth_under_it():
    cmd = _check({"perform": SONG}, LIVE)
    assert isinstance(cmd, Command) and cmd.perform.backing == "beat" and "perform.backing" in cmd.dropped
    calm = _check({"perform": {**SONG, "backing": "calm"}}, LIVE)
    assert calm.perform.backing == "calm" and "perform.backing" not in calm.dropped


@pytest.mark.parametrize("raw", [
    {"actions": [GEN_CALL]},  # генерация вместо пения
    {"actions": [{"tool": "request_music", "args": {"intent": "track", "text": "спой под синт"}}]},
    {"speech": "Сейчас спою!"},  # обещание без пения
    {"perform": {**SONG, "kind": "rap"}},  # не тот вид
    {"perform": SONG, "actions": [GEN_CALL]},  # пение и генерация разом
])
def test_live_route_violations_are_the_sing_route_not_tool_not_available(raw):
    """Закрытый тул — нарушение формы (``sing_route``), а не ``tool_not_available``: мост по нему срез не расширяет;
    закрытый тул не спасается (``salvage``)."""
    result = _check(raw, LIVE)
    reasons = [v.reason for v in result.all]
    assert kn.VIOLATION_SING_ROUTE in reasons and "tool_not_available" not in reasons, reasons
    assert not any(c.tool in ("generate_music", "request_music") for c in result.salvage)


def test_generate_route_needs_the_generation_tool_and_no_song_performance():
    assert isinstance(_check({"actions": [GEN_CALL]}, GEN), Command)
    for raw in ({"perform": SONG}, {"speech": "Сейчас сгенерирую!"}, {"perform": {**SONG, "kind": "rap"}}):
        assert _check(raw, GEN).reason == kn.VIOLATION_SING_ROUTE, raw
    tale = {"kind": "tale", "segments": ["Жил-был енот", "и пел"], "voice_per_segment": ["", ""]}
    assert isinstance(_check({"perform": tale, "actions": [GEN_CALL]}, GEN), Command)


def test_without_a_sing_route_both_ways_stay_open():
    assert isinstance(_check({"perform": SONG}, Form(passed=False)), Command)
    assert isinstance(_check({"actions": [GEN_CALL]}, Form(passed=False)), Command)


def test_edit_of_a_running_performance_is_not_judged_by_the_sing_route():
    """«И про енота» посреди песни — правка идущего выступления, а не новая просьба спеть."""
    from rob_box_dialog.command import Busy

    edit = {"edit_pending": True, "perform": {"kind": "rap", "segments": ["А вот и енот"]}}
    assert isinstance(validate(parse(edit), MUSIC, busy=Busy(active_segment=1), form=LIVE), Command)


def test_the_violation_explains_the_route_to_the_model():
    from rob_box_dialog.command import explain

    text = explain(_check({"actions": [GEN_CALL]}, LIVE))
    assert "sing_route" in text and "live:song" in text and "generate_music" in text


# -- ход: путь до модели, срез тулов, форма на время вызова LLM -----------------------------------------------------


class Ports:
    def __init__(self, answers):
        self.answers, self.asked, self.forms, self.performed = list(answers), [], [], []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append([t.name for t in tools])
        self.forms.append(current_form())
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        return [ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, a.tool, kn.TOOL_CLASSES[a.tool],
                             "done", None, dict(a.args)) for n, a in enumerate(actions)]

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
        self.performed.append(perf)
        return {"status": "pending"}


class Router:
    def __init__(self, route):
        self.route, self.calls = route, []

    def __call__(self, text, recent, playing):
        self.calls.append((text, tuple(recent), playing))
        return self.route


def _turns(router, answers, *texts, pickup=False):
    ports, engine = Ports(answers), None
    engine = TurnEngine(Session("personality"), ports, system_prompt="Ты робот.", sing=router)
    for text in texts:
        asyncio.run(engine.run(Addressed(text, VOICE, pickup=pickup)))
    return ports


def test_live_route_reaches_the_model_as_the_form_and_a_slice_without_generation():
    router = Router("live:song")
    sing = Command(perform=Performance("song", tuple(SONG["segments"]), "beat"))
    ports = _turns(router, [Answer(Command(speech="Привет!")), Answer(sing)], "Робот, привет",
                   "Робот, спой под синт про енота")
    assert router.calls[-1] == ("Робот, спой под синт про енота", ("Робот, привет",), False)
    assert ports.forms[-1].sing == "live:song"
    assert "generate_music" not in ports.asked[-1] and "request_music" not in ports.asked[-1]
    assert ports.performed and ports.performed[-1].kind == "song"


def test_generate_route_keeps_the_generation_tool_in_the_slice():
    router = Router("generate:robot_song:V6")
    ports = _turns(router, [Answer(Command(actions=(ToolCall("generate_music", GEN_CALL["args"]),)))],
                   "Робот, сгенерируй песню про енота своим голосом")
    assert ports.forms[-1].sing == "generate:robot_song:V6" and "generate_music" in ports.asked[-1]


def test_form_tools_add_the_needed_generation_tool_cut_by_the_slice_limit():
    snap = Session("personality").snapshot
    cut = tuple(t for t in available_tools(snap, None, "personality", "спой") if t.name != "generate_music")
    assert "generate_music" in [t.name for t in form_tools(cut, GEN, "personality")]
    assert "generate_music" not in [t.name for t in form_tools(cut, Form(), "personality")]


def test_no_route_and_no_router_leave_the_turn_as_it_was():
    router = Router("")
    ports = _turns(router, [Answer(Command(speech="Привет!"))], "Робот, привет")
    assert ports.forms[-1].sing == "" and router.calls
    plain = _turns(None, [Answer(Command(speech="Привет!"))], "Робот, спой про енота")
    assert plain.forms[-1].sing == "" and "generate_music" in plain.asked[-1]


def test_backlog_pickup_does_not_ask_for_a_sing_route():
    """Подхват бэклога — не просьба этой реплики: путь пения не спрашивается."""
    router = Router("live:song")
    ports = _turns(router, [Answer(Command(speech="Слышал."))], "Робот, спой под синт", pickup=True)
    assert ports.asked and router.calls == [] and ports.forms[-1].sing == ""
