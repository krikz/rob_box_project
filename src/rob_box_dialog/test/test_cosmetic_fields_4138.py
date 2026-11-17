"""#4138 (n904, акт 9, 28.10): команда не отвергается целиком из-за косметики.

«Робот, покажи, как тебе грустно» → ``invalid:bad_enum earcon=none``, на повторе ``invalid:bad_args
play_animation.animated: лишнее поле`` → «Не понял, как это сделать», грустного лица не было. Модель оба раза хотела
показать грусть; выброшенное поле смысла не меняло. Теперь решает код (ADR-0148): ``knowledge.DROPPABLE`` - единственная
таблица того, что можно выбросить; выброшенное - в ``Command.dropped`` (лог ``fields_dropped``), действие исполняется.
Отказ целиком остаётся там, где без поля действие теряет смысл (нет аргумента, несущего смысл; неизвестный тул)."""

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


def _show(args=None, **expression):
    """Команда показа на лице: ``play_animation`` + экспрессия (форма по логу n904)."""
    return {"expression": {"emotion": "sad", **expression},
            "actions": [{"tool": "play_animation", "args": {"animation": "sad", **(args or {})}}]}


# -- п.1: earcon не из перечня выбрасывается -------------------------------------------------------------------------


def test_4138_first_n904_answer_earcon_none_is_dropped_and_the_animation_stays():
    cmd = _ok(_show(earcon="none"))
    assert cmd.expression.earcon is None and cmd.expression.emotion == "sad"
    assert cmd.actions == (ToolCall("play_animation", {"animation": "sad"}),)
    assert cmd.dropped == ("expression.earcon",)


def test_4138_valid_earcon_is_kept_and_nothing_is_dropped():
    cmd = _ok(_show(earcon="done"))
    assert cmd.expression.earcon == "done" and cmd.dropped == ()


def test_4138_earcon_of_a_performance_segment_is_dropped_by_its_index():
    perf = {"kind": "rap", "segments": ["куплет один", "куплет два"],
            "expression_per_segment": [{"emotion": "happy"}, {"emotion": "sad", "earcon": "none"}]}
    cmd = _ok({"perform": perf})
    assert cmd.dropped == ("perform.expression_per_segment[1].earcon",)
    assert [e.earcon for e in cmd.perform.expression_per_segment] == [None, None]
    assert [e.emotion for e in cmd.perform.expression_per_segment] == ["happy", "sad"]


def test_4138_bad_face_and_bad_earcon_are_both_dropped_in_one_pass():
    cmd = _ok(_show(earcon="none", face={"eyes": "?"}))
    assert cmd.dropped == ("expression.earcon", "expression.face") and cmd.expression.face is None


def test_4138_emotion_and_gesture_may_be_the_request_itself_so_they_still_reject():
    """«Покажи, как тебе грустно» / «подмигни»: выбросить эмоцию или жест значило бы исполнить не то, что просили."""
    assert [v.reason for v in _bad(_show(emotion="joy", earcon="none"), "bad_enum").all] == ["bad_enum"]
    res = _bad({"expression": {"gesture": "dance", "earcon": "none"}}, "bad_enum")
    assert [(v.detail, v.allowed) for v in res.all] == [("gesture=dance", kn.GESTURES)]


# -- п.1: лишний аргумент play_animation выбрасывается ---------------------------------------------------------------


def test_4138_retry_n904_extra_animated_argument_is_dropped_and_the_animation_stays():
    cmd = _ok(_show({"animated": True}))
    assert cmd.actions == (ToolCall("play_animation", {"animation": "sad"}),)
    assert cmd.dropped == ("actions[0].play_animation.animated",)


def test_4138_extra_argument_next_to_a_face_scene_is_dropped_as_well():
    args = {"face": {"mouth": "frown"}, "animated": True, "color": "blue"}
    cmd = _ok({"actions": [{"tool": "play_animation", "args": args}]})
    assert dict(cmd.actions[0].args) == {"face": {"mouth": "frown"}}
    assert cmd.dropped == ("actions[0].play_animation.animated", "actions[0].play_animation.color")


def test_4138_both_n904_mistakes_in_one_command_are_dropped_together_and_the_command_round_trips():
    cmd = _ok(_show({"animated": True}, earcon="none"))
    assert sorted(cmd.dropped) == ["actions[0].play_animation.animated", "expression.earcon"]
    assert _ok(cm.to_args(cmd)).actions == cmd.actions, "к исполнителю уходит команда без выброшенного"


def test_4138_native_narrow_tool_call_gets_the_same_treatment():
    calls = [{"name": "command", "arguments": {"expression": {"emotion": "sad"}}},
             {"name": "play_animation", "arguments": {"animation": "sad", "animated": True}}]
    cmd = _ok(calls)
    assert cmd.actions == (ToolCall("play_animation", {"animation": "sad"}),)
    assert cmd.dropped == ("actions[0].play_animation.animated",)


def test_4138_dropped_argument_does_not_hide_the_other_violations_and_the_salvage_is_clean():
    """Спасённое действие невалидной команды (#3953) тоже без выброшенного: исполнится так, как исполнилось бы в
    валидной команде."""
    bell = {"tool": "play_sound", "args": {"sound": "ui_bell"}}
    shown = {"tool": "play_animation", "args": {"animation": "sad", "animated": True}}
    res = _bad({"speech": "Вот.", "actions": [bell, shown]}, kn.VIOLATION_SPEECH_WITH_ACTIONS)
    assert ToolCall("play_animation", {"animation": "sad"}) in res.salvage


# -- п.2: отказ целиком там, где без поля действие теряет смысл -------------------------------------------------------


def test_4138_only_the_extra_argument_left_means_no_meaning_so_the_call_is_rejected_as_before():
    res = _bad({"actions": [{"tool": "play_animation", "args": {"animated": True}}]}, "bad_args")
    assert "animated: лишнее поле" in res.detail


def test_4138_wrong_value_of_the_meaning_argument_is_still_rejected():
    res = _bad(_show({"animation": "sadness", "animated": True}), "bad_args")
    assert "sadness" in res.detail and "animated" not in res.detail, "лишнее ушло, enum animation проверен по-прежнему"


def test_4138_unknown_tool_and_tool_outside_the_slice_are_rejected_as_before():
    extra = {"animated": True}
    _bad({"actions": [{"tool": "no_such_tool", "args": extra}]}, "tool_not_available")
    _bad(_show(extra), "tool_not_available", tools=["memory_save"])


@pytest.mark.parametrize("tool,args", [
    ("play_sound", {"sound": "ui_bell", "loop": True}),
    ("set_volume", {"action": "louder", "db": 3}),
    ("request_music", {"intent": "play", "text": "джаз", "channel": "music"}),
])
def test_4138_extra_argument_of_a_tool_outside_the_table_is_still_rejected(tool, args):
    """У громкости, музыки и звуков лишний аргумент (канал, жанр, повтор) молча менял бы действие - не косметика."""
    assert tool not in kn.DROPPABLE.tool_args
    res = _bad({"actions": [{"tool": tool, "args": args}]}, "bad_args")
    assert "лишнее поле" in res.detail


def test_4138_missing_required_argument_is_rejected_as_before():
    _bad({"actions": [{"tool": "set_volume", "args": {"db": 3}}]}, "bad_args")
    _bad({"actions": [{"tool": "save_waypoint", "args": {}}]}, "bad_args")


def test_4138_clean_commands_are_untouched():
    cmd = _ok(_show(earcon="heard"))
    assert cmd.dropped == () and dict(cmd.actions[0].args) == {"animation": "sad"}


# -- таблица «что можно выбросить» -----------------------------------------------------------------------------------


def test_4138_droppable_table_is_consistent_with_the_schemas():
    """Одна таблица: поля экспрессии - с enum и необязательные (эмоция не среди них); тулы - из каталога, аргументы
    смысла - из схемы тула."""
    optional = {name for name in cm._EXPRESSION_ENUMS if name != "emotion"}
    assert set(kn.DROPPABLE.expression) <= optional
    for tool, meaning in kn.DROPPABLE.tool_args.items():
        props = get_tool(tool).parameters["properties"]
        assert meaning and set(meaning) <= set(props), tool
        assert get_tool(tool).parameters.get("additionalProperties") is False, "лишнее поле - из закрытой схемы"
        assert kn.TOOL_CLASSES[tool] == "action.local", "косметика - действие без побочных эффектов вне робота"


def test_4138_droppable_fields_are_in_the_model_schema_so_the_model_is_still_asked_for_valid_values():
    schema = kn.command_schema("json")["properties"]["expression"]["properties"]
    for name in kn.DROPPABLE.expression:
        assert schema[name]["enum"], "модель по-прежнему просим о значении из перечня; выбрасывает только код"
