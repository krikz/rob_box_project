"""#4174 (n310, акт 3, марафон-3 28.10): значение перечня у НЕОБЯЗАТЕЛЬНОГО аргумента-подсказки не валит команду.

«Включи какую-нибудь музыку, только не электронную, что-нибудь спокойное» → ``request_music.genre='acoustic'`` вне
перечня схемы → ``invalid:bad_args`` дважды → «Не понял, как это сделать». Просьба была понятна. Одно правило кода
(ADR-0148, ``knowledge.DROPPABLE.soft_args``): мягкий аргумент не из перечня → ``auto`` (если есть в перечне) или выброс,
запись в ``Command.dropped``, команда исполняется. Обязательные аргументы и аргументы, несущие смысл действия, строги."""

import pytest

from rob_box_core.tool_catalog import TOOL_CATALOG, get_tool
from rob_box_dialog import command as cm
from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import Command, Invalid, ToolCall, parse, validate

ALL = [t.name for t in TOOL_CATALOG]


def _check(raw, tools=ALL):
    return validate(parse(raw), tools)


def _ok(raw, tools=ALL):
    res = _check(raw, tools)
    assert isinstance(res, Command), res
    return res


def _bad(raw, reason, tools=ALL):
    res = _check(raw, tools)
    assert isinstance(res, Invalid), res
    assert res.reason == reason, res
    return res


def _call(tool, **args):
    return {"actions": [{"tool": tool, "args": args}]}


def test_4174_n310_unknown_genre_becomes_auto_and_the_music_request_runs():
    cmd = _ok(_call("request_music", intent="track", text="спокойное, не электронное", genre="acoustic"))
    assert dict(cmd.actions[0].args) == {"intent": "track", "text": "спокойное, не электронное", "genre": "auto"}
    assert cmd.dropped == ("actions[0].request_music.genre['acoustic'->auto]",)


def test_4174_engine_without_auto_default_is_not_needed_it_also_falls_to_auto():
    cmd = _ok(_call("request_music", intent="track", text="джаз", engine="suno"))
    assert cmd.actions[0].args["engine"] == "auto"
    assert cmd.dropped == ("actions[0].request_music.engine['suno'->auto]",)


def test_4174_hint_without_neutral_value_in_the_enum_is_dropped():
    """``mood`` без ``auto`` в перечне: выбрасывается, тул берёт своё значение по умолчанию."""
    assert "auto" not in get_tool("request_music").parameters["properties"]["mood"]["enum"]
    cmd = _ok(_call("request_music", intent="track", text="спокойное", mood="chill"))
    assert "mood" not in cmd.actions[0].args
    assert cmd.dropped == ("actions[0].request_music.mood['chill'->]",)


def test_4174_several_bad_hints_in_one_call_and_two_calls_each_get_their_own_index():
    cmd = _ok({"actions": [
        {"tool": "play_animation", "args": {"animation": "happy"}},
        {"tool": "request_music", "args": {"intent": "track", "text": "х", "genre": "acoustic", "mood": "chill"}}]})
    assert sorted(cmd.dropped) == ["actions[1].request_music.genre['acoustic'->auto]",
                                   "actions[1].request_music.mood['chill'->]"]
    assert dict(cmd.actions[0].args) == {"animation": "happy"}


def test_4174_dj_and_generate_style_outside_the_enum_are_softened_too():
    dj = _ok(_call("dj_set", action="start", style="polka"))
    assert dj.actions[0].args["style"] == "auto" and dj.dropped == ("actions[0].dj_set.style['polka'->auto]",)
    gen = _ok(_call("generate_music", kind="instrumental", style="polka"))
    assert "style" not in gen.actions[0].args and gen.dropped == ("actions[0].generate_music.style['polka'->]",)


def test_4174_valid_values_and_absent_hints_are_untouched():
    cmd = _ok(_call("request_music", intent="track", text="рок", genre="rock", mood="calm"))
    assert dict(cmd.actions[0].args) == {"intent": "track", "text": "рок", "genre": "rock", "mood": "calm"}
    assert cmd.dropped == ()
    assert _ok(_call("request_music", intent="track", text="рок")).dropped == ()


@pytest.mark.parametrize("tool,args,named", [
    ("request_music", {"intent": "play", "text": "джаз"}, "request_music.intent"),     # обязательный intent
    ("request_music", {"intent": "track"}, "request_music.text: обязателен"),          # нет обязательного text
    ("set_volume", {"action": "much louder"}, "set_volume.action"),                    # обязательный action
    ("dj_set", {"action": "pause"}, "dj_set.action"),
    ("edit_music", {"change": "weirder"}, "edit_music.change"),         # необязательный, но «что править» - смысл
    ("music_library", {"action": "rate", "verdict": "love"}, "music_library.verdict"),  # оценка - смысл
    ("stop_music", {"boundary": "soon"}, "stop_music.boundary"),
    ("play_animation", {"animation": "sadness"}, "play_animation.animation"),
])
def test_4174_required_and_meaning_arguments_stay_strict(tool, args, named):
    res = _bad(_call(tool, **args), "bad_args")
    assert named in res.detail


def test_4174_strict_argument_next_to_a_soft_one_still_rejects_and_names_only_the_strict_one():
    res = _bad(_call("request_music", intent="play", text="джаз", genre="acoustic"), "bad_args")
    assert "intent" in res.detail and "genre" not in res.detail


def test_4174_validate_is_idempotent_and_the_result_round_trips():
    first = _ok(_call("request_music", intent="track", text="спокойное", genre="acoustic", mood="chill"))
    again = validate(first, ALL)
    assert isinstance(again, Command)
    assert again.actions == first.actions and again.dropped == first.dropped
    assert _ok(cm.to_args(first)).actions == first.actions, "к исполнителю уходит команда без непринятых значений"


def test_4174_native_tool_call_form_is_softened_the_same_way():
    calls = [{"name": "command", "arguments": {}},
             {"name": "request_music", "arguments": {"intent": "track", "text": "х", "genre": "acoustic"}}]
    cmd = _ok(calls)
    assert cmd.actions == (ToolCall("request_music", {"intent": "track", "text": "х", "genre": "auto"}),)


def test_4174_unknown_tool_or_tool_outside_the_slice_is_still_rejected():
    _bad(_call("no_such_tool", genre="acoustic"), "tool_not_available")
    _bad(_call("request_music", intent="track", text="х", genre="acoustic"), "tool_not_available", tools=["memory_save"])


def test_4174_soft_args_table_is_consistent_with_the_schemas():
    """Одна таблица: тулы из каталога, аргументы - необязательные поля с перечнем (обязательный/без перечня в неё
    не нужен), «смысловые» необязательные аргументы в ней не числятся."""
    for tool, names in kn.DROPPABLE.soft_args.items():
        schema = get_tool(tool).parameters
        for name in names:
            assert schema["properties"][name].get("enum"), (tool, name)
            assert name not in schema.get("required", ()), (tool, name)
    meaning = {("edit_music", "change"), ("music_library", "verdict"), ("stop_music", "boundary"),
               ("play_animation", "animation"), ("memory_save", "category")}
    listed = {(t, n) for t, names in kn.DROPPABLE.soft_args.items() for n in names}
    assert not listed & meaning


def test_4174_empty_table_rolls_back_to_the_old_behaviour(monkeypatch):
    monkeypatch.setattr(kn, "DROPPABLE", kn.Droppable(expression=(), tool_args={}))
    res = _bad(_call("request_music", intent="track", text="х", genre="acoustic"), "bad_args")
    assert "acoustic" in res.detail
