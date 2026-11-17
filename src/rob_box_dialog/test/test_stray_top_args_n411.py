"""n411 марафона-4 (28.10): ``args`` на уровне команды не валят её (ADR-0148: решает код, не повтор модели).

«Робот, говори ниже» → DeepSeek прислал ``{"args": {"action": "lower"}, "actions": [{"args": {"action": "lower"},
"tool": "set_pitch"}], "speech": ...}``: поля ``args`` в схеме команды нет, ``invalid:unknown_field detail='command:
args'``, ход потратил повтор. Теперь ``args`` переезжают в единственное действие (``moved``) или выбрасываются
(``dropped``); без действия - нарушение ``args_without_action`` и повтор. Обязательные поля и прочие незнакомые поля
команды строгие по-прежнему."""

import json

import pytest

from rob_box_core.tool_catalog import TOOL_CATALOG
from rob_box_dialog import command as cm
from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import Command, Invalid, ToolCall, parse, validate

ALL = [t.name for t in TOOL_CATALOG]

#: Дословно из лога марафона-4 (22:06:45, ``🔎 LLM RESPONSE``, аргументы вызова ``command``): речь - из повтора 22:06:47.
N411 = json.dumps({
    "args": {"action": "lower"},
    "actions": [{"args": {"action": "lower"}, "tool": "set_pitch"}],
    "speech": "Куда же без неё, господа: рыба к Гарри — басом, как диктор старого кино. Саша, теперь я звучу солидно, "
              "правда?"}, ensure_ascii=False)


def _parsed(raw):
    cmd = parse(raw)
    assert isinstance(cmd, Command), cmd
    return cmd


def test_n411_verbatim_tool_call_is_no_longer_unknown_field():
    cmd = _parsed([{"function": {"name": "command", "arguments": N411}}])
    assert cmd.actions == (ToolCall("set_pitch", {"action": "lower"}),)
    # те же значения в действии и сверху: переносить нечего, лишнее выброшено с записью в лог
    assert cmd.dropped == ("command.args.action",) and cmd.moved == ()
    res = validate(cmd, ALL)
    # дальше - ровно то нарушение, что и на повторе в логе (speech рядом с громким действием), а не unknown_field
    assert isinstance(res, Invalid) and [v.reason for v in res.all] == ["speech_with_actions"]
    assert res.salvage == (ToolCall("set_pitch", {"action": "lower"}),)


def test_top_args_of_the_only_action_move_into_it_and_the_action_runs():
    cmd = _parsed({"actions": [{"tool": "set_pitch"}], "args": {"action": "lower"}})
    assert cmd.actions == (ToolCall("set_pitch", {"action": "lower"}),)
    assert cmd.moved == ("command.args.action",) and cmd.dropped == ()
    assert isinstance(validate(cmd, ALL), Command)


def test_action_own_value_wins_over_the_stray_one():
    cmd = _parsed({"actions": [{"tool": "set_pitch", "args": {"action": "higher"}}], "args": {"action": "lower"}})
    assert cmd.actions[0].args == {"action": "higher"} and cmd.dropped == ("command.args.action",)


def test_several_actions_make_the_top_args_ambiguous_so_they_are_dropped():
    cmd = _parsed({"actions": [{"tool": "set_pitch", "args": {"action": "lower"}},
                               {"tool": "set_speed", "args": {"action": "slower"}}], "args": {"action": "x"}})
    assert cmd.dropped == (cm.STRAY_ARGS,) and cmd.moved == ()
    assert [dict(a.args) for a in cmd.actions] == [{"action": "lower"}, {"action": "slower"}]
    assert isinstance(validate(cmd, ALL), Command)


def test_top_args_without_actions_are_dropped_and_the_missing_action_is_a_violation():
    res = validate(parse({"speech": "Говорю ниже.", "args": {"action": "lower"}}), ALL)
    assert isinstance(res, Invalid) and res.reason == kn.VIOLATION_ARGS_WITHOUT_ACTION
    assert kn.VIOLATION_ARGS_WITHOUT_ACTION in cm.explain(res)  # повтор получает подсказку, а не «нет поля»


def test_top_args_beside_a_performance_are_dropped_without_violation():
    perf = {"kind": "rap", "segments": ["куплет один", "куплет два"]}
    res = validate(parse({"perform": perf, "args": {"action": "lower"}}), ALL)
    assert isinstance(res, Command) and res.dropped == (cm.STRAY_ARGS,)


@pytest.mark.parametrize("empty", ["", None, {}])
def test_empty_top_args_are_nothing_to_lose(empty):
    cmd = _parsed({"speech": "привет", "args": empty})
    assert cmd.dropped == () and cmd.moved == ()


def test_non_object_top_args_are_dropped_not_crashing():
    res = validate(parse({"actions": [{"tool": "set_pitch", "args": {"action": "lower"}}], "args": "lower"}), ALL)
    assert isinstance(res, Command) and res.dropped == (cm.STRAY_ARGS,)


def test_other_stray_command_fields_stay_strict():
    res = parse({"speech": "x", "args": {"a": 1}, "mood": "весёлый"})
    assert isinstance(res, Invalid) and res.reason == "unknown_field"


def test_required_action_fields_stay_strict_after_the_move():
    """Перенос не додумывает за модель: аргумента не хватает или он не из перечня - схема тула отказывает, как раньше."""
    res = validate(parse({"actions": [{"tool": "set_pitch"}], "args": {"action": "sideways"}}), ALL)
    assert isinstance(res, Invalid) and res.reason == "bad_args"
    res = validate(parse({"actions": [{"tool": "set_pitch"}], "args": {"junk": 1}}), ALL)
    assert isinstance(res, Invalid) and res.reason == "bad_args"
