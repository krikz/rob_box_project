# 5025 say/play schema

import ast
from pathlib import Path

from rob_box_core.tool_catalog import get_tool
from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import VOICE
from rob_box_dialog.chimera_offer import PLAY_FORMS, SAY_FORMS, fields_for, perform_description, verb_action
from rob_box_dialog.command import Form, current_form, parse, tool_schema, turn_form, validate
from rob_box_dialog.owners import OWNERS
from rob_box_dialog.session import SessionSnapshot
from rob_box_dialog.tools_view import available_tools
from rob_box_dialog.turn import command_form, form_tools
from rob_box_dialog_host.dialog_v2_ports import tool_schema as model_schema

SAY = "Робот, скажи химерой привет"
PLAY = "Робот, сыграй химерой Грига"
SHOW = "Робот, покажи химеру"
BOTH = "Робот, скажи и сыграй химерой"
SING = "Робот, спой химерой песню про енота"
_MUSIC = Path(__file__).resolve().parents[2] / "rob_box_music" / "rob_box_music" / "knowledge.py"


def _snap() -> SessionSnapshot:
    return SessionSnapshot(agent="personality", session_id="t", epoch=0, started_at=0.0, owners=OWNERS)


def _form(text: str, sing: str = "") -> Form:
    from rob_box_dialog.turn import _Turn

    return command_form(_Turn("t1", 0, text, VOICE, deadline=None), _snap(), sing=sing)


def _shown(form: Form) -> dict:
    with turn_form(form):
        return model_schema(get_tool(kn.CHIMERA_TOOL))["function"]


def _music_play_forms() -> tuple:
    tree = ast.parse(_MUSIC.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        target = getattr(node, "target", None)
        if isinstance(node, ast.AnnAssign) and getattr(target, "id", "") == "SING_PLAY_VERBS":
            return tuple(ast.literal_eval(node.value))
    raise AssertionError("SING_PLAY_VERBS not found")


def test_say_and_play_forms_match_the_one_verb_table():
    assert set(SAY_FORMS) <= set(kn.SPEECH_ASK_VERBS)
    assert "расскажи" not in SAY_FORMS and "озвучь" not in SAY_FORMS
    assert PLAY_FORMS == _music_play_forms()
    assert verb_action(SAY) == "speak" and verb_action(PLAY) == "play"
    assert verb_action("Робот, не скажи химерой") == ""
    assert verb_action("Робот, не сыграй химерой") == ""
    assert verb_action(BOTH) == ""
    assert verb_action(SHOW) == "" and verb_action(SING) == ""


def test_say_by_chimera_narrows_the_schema_to_speak():
    form = _form(SAY)
    assert form.sing == "" and kn.sing_family("", text=SAY) == "chimera"
    sing = form.sing_form
    assert sing is not None and sing.needs_tool == kn.CHIMERA_TOOL and sing.needs_perform is False
    assert sing.tool_actions == ("speak",) and sing is not kn.SING_FORMS["chimera"]
    shown = _shown(form)
    actions = shown["parameters"]["properties"]["action"]["enum"]
    assert actions == ["speak"] and shown["description"] == perform_description(actions)
    assert set(shown["parameters"]["properties"]) == {"action", *fields_for(actions)}
    props = shown["parameters"]["properties"]
    assert "phrase_id" in props and "title" not in props
    call = {"tool": kn.CHIMERA_TOOL, "args": {"action": "speak", "phrase_id": "hi"}}
    accepted = validate(parse({"actions": [call]}), [kn.CHIMERA_TOOL], form=form)
    assert accepted.actions[0].args["action"] == "speak"
    outside = validate(parse({"actions": [call]}), [kn.CHIMERA_TOOL])
    assert outside.reason == "bad_args"
    catalog = get_tool(kn.CHIMERA_TOOL).parameters["properties"]["action"]["enum"]
    assert catalog == ["call"]
    assert current_form().sing_form is None


def test_play_by_chimera_narrows_the_schema_to_play_when_the_slice_drops_the_tool():
    form = _form(PLAY)
    assert form.sing == "" and form.sing_form is not None and form.sing_form.tool_actions == ("play",)
    names = [tool.name for tool in available_tools(_snap(), text=PLAY)]
    assert kn.CHIMERA_TOOL not in names
    offered = [tool.name for tool in form_tools(available_tools(_snap(), text=PLAY), form, "personality", OWNERS)]
    assert kn.CHIMERA_TOOL in offered
    shown = tool_schema(kn.CHIMERA_TOOL, form)
    assert shown["properties"]["action"]["enum"] == ["play"]
    assert set(shown["properties"]) == {"action", *fields_for(("play",))}
    call = {"tool": kn.CHIMERA_TOOL, "args": {"action": "play", "title": "Григ"}}
    accepted = validate(parse({"actions": [call]}), offered, form=form)
    assert accepted.actions[0].args["action"] == "play"
    speech = validate(parse({"speech": "Сейчас сыграю!"}), offered, form=form)
    assert speech.reason == kn.VIOLATION_SING_ROUTE
    catalog = get_tool(kn.CHIMERA_TOOL).parameters["properties"]["action"]["enum"]
    assert catalog == ["call"]


def test_live_sing_stays_sing_and_a_bare_name_stays_outside_the_family():
    live = _form(SING, "live:song")
    assert live.sing_form is kn.SING_FORMS["chimera"] and live.sing_form.tool_actions == ("sing",)
    mixed = _form("Робот, скажи и спой химерой", "live:song")
    assert mixed.sing_form is not None and mixed.sing_form.tool_actions == ("sing",)
    bare = (SHOW, BOTH, "Робот, расскажи анекдот химерой", "Робот, не сыграй химерой")
    for text in bare:
        form = _form(text)
        assert form.sing_form is None, text
        assert kn.sing_family("", text=text) == ""
        enum = tool_schema(kn.CHIMERA_TOOL, form)["properties"]["action"]["enum"]
        assert enum == ["call"]
