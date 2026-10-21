"""Команда LLM: разбор из tool_calls/JSON/текста, валидатор по таблицам знания и схемам каталога (ADR-0150 §4.1)."""

import dataclasses
import inspect
import json
import pathlib
import random
import re

import pytest

from rob_box_core.tool_catalog import TOOL_CATALOG, get_tool
from rob_box_dialog import command as cm
from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import Busy, Command, FaceScene, Invalid, parse, validate
from rob_box_dialog.session import Session

ALL = [t.name for t in TOOL_CATALOG]
OPERATOR = Session("operator").snapshot
PERSONALITY = Session("personality").snapshot


def ok(raw, tools=ALL, **kw):
    res = validate(parse(raw), tools, **kw)
    assert isinstance(res, Command), res
    return res


def bad(raw, reason, tools=ALL, **kw):
    res = validate(parse(raw), tools, **kw)
    assert isinstance(res, Invalid), res
    assert res.reason == reason, res
    assert res.ask.key == "understand.invalid" and res.ask.text is None
    return res


def _perform(n=2, **extra):
    return {"kind": "rap", "segments": [f"куплет {i}" for i in range(n)], **extra}


# --- разбор -------------------------------------------------------------------------------------------------


def test_json_text_and_mapping_give_the_same_command():
    raw = {"speech": "Привет!", "expression": {"emotion": "happy"}}
    assert ok(raw) == ok(json.dumps(raw, ensure_ascii=False))
    assert ok(raw).speech == "Привет!" and ok(raw).expression.emotion == "happy"


def test_free_text_takes_the_first_json_object_and_drops_the_rest():
    cmd = ok('Конечно! {не json} {"speech": "Первая"} и ещё {"speech": "Вторая"}')
    assert cmd.speech == "Первая"


def test_text_without_command_is_invalid():
    bad("Просто поговорю без JSON", "no_command")
    bad("", "no_command")


def test_tool_calls_command_plus_narrow_tools():
    calls = [
        {"id": "1", "type": "function", "function": {"name": "command", "arguments": '{"speech": "Ок"}'}},
        {"id": "2", "type": "function", "function": {"name": "get_current_time", "arguments": ""}},
        {"name": "set_volume", "arguments": {"action": "louder"}},
    ]
    cmd = ok(calls)
    assert cmd.speech == "Ок"
    assert [a.tool for a in cmd.actions] == ["get_current_time", "set_volume"]


def test_tool_calls_need_exactly_one_command_or_some_tools():
    bad([{"name": "command", "arguments": "{}"}, {"name": "command", "arguments": "{}"}], "several_commands")
    bad([], "no_command")
    bad([{"name": "command", "arguments": "{oops"}], "bad_json")
    bad(["не вызов"], "bad_tool_call")
    assert ok([{"name": "get_current_time", "arguments": "{}"}]).speech is None


@pytest.mark.parametrize("raw, reason", [
    ({"speech": 5}, "bad_type"),
    ({"edit_pending": "yes"}, "bad_type"),
    ({"actions": {"tool": "get_current_time"}}, "bad_type"),
    ({"actions": [{"tool": "get_current_time", "argz": {}}]}, "unknown_field"),
    ({"mood": "весёлый"}, "unknown_field"),
    ({"perform": {"kind": "rap", "segments": ["раз", 2]}}, "bad_type"),
    ({"perform": {"kind": "rap", "segments": None}}, "bad_segments"),
    ({"actions": [{"tool": "get_current_time", "args": None}], "speech": None}, None),
    ({"perform": {**_perform(), "from_segment": True}}, "bad_type"),
    ({"expression": "happy"}, "bad_type"),
    (42, "bad_type"),
])
def test_structural_errors(raw, reason):
    if reason is None:  # null там, где ждут объект/строку, — «нет значения», а не ошибка
        ok(raw)
    else:
        bad(raw, reason)


def test_empty_speech_is_no_speech():
    assert ok({"speech": "   "}).speech is None


# --- речь, вопрос, молчание ---------------------------------------------------------------------------------


def test_speech_limit_and_control_chars():
    assert ok({"speech": "а" * kn.SPEECH_MAX_CHARS}).speech
    bad({"speech": "а" * (kn.SPEECH_MAX_CHARS + 1)}, "too_long")
    bad({"speech": "привет\x07"}, "control_chars")
    assert ok({"speech": "две\nстроки"}).speech == "две\nстроки"


def test_ask_from_llm():
    assert ok({"ask": {"expects": "name", "text": "Как тебя зовут?"}}).ask.text == "Как тебя зовут?"
    bad({"ask": {"expects": "maybe", "text": "Да?"}}, "bad_enum")
    bad({"ask": {"expects": "yes_no"}}, "bad_ask")


def test_passed_is_honest_silence_only():
    assert ok({"passed": "not_addressed"}).passed == "not_addressed"
    bad({"passed": "nothing_to_say", "speech": "но скажу"}, "passed_with_content")
    bad({"passed": "bored"}, "bad_enum")


def test_minimax_empty_shapes_normalized():
    """Бейк-офф 21.10 (ADR §18.2 О4): MiniMax пишет ``"passed": false`` и ``"args": ""`` — это «поля нет»."""
    said = ok({"speech": "Привет!", "actions": [], "passed": False})
    assert (said.speech, said.passed) == ("Привет!", None)
    assert ok({"speech": "Привет!", "passed": ""}).passed is None
    assert ok({"speech": "Привет!", "passed": None}).passed is None
    timed = ok({"actions": [{"tool": "get_current_time", "args": ""}]})
    assert dict(timed.actions[0].args) == {}
    assert dict(ok({"actions": [{"tool": "get_current_time", "args": None}]}).actions[0].args) == {}
    found = ok({"actions": [{"tool": "search_web", "args": '{"query": "погода"}'}]})
    assert dict(found.actions[0].args) == {"query": "погода"}
    bad({"speech": "x", "passed": True}, "bad_type")
    bad({"actions": [{"tool": "search_web", "args": "погода"}]}, "bad_type")
    bad({"actions": [{"tool": "search_web", "args": ["погода"]}]}, "bad_type")


# --- действия -----------------------------------------------------------------------------------------------


def test_actions_limit_visibility_and_catalog():
    four = [{"tool": "get_current_time"}] * (kn.ACTIONS_MAX + 1)
    bad({"actions": four}, "too_many_actions")
    bad({"actions": [{"tool": "search_web", "args": {"query": "x"}}]}, "tool_not_available", tools=["get_current_time"])
    bad({"actions": [{"tool": "speak_text", "args": {"text": "x"}}]}, "speech_channel_tool")
    bad({"actions": [{"tool": "no_such_tool"}]}, "tool_not_available")
    bad({"actions": [{"tool": "no_such_tool"}]}, "unknown_tool", tools=ALL + ["no_such_tool"])


@pytest.mark.parametrize("args", [{"action": "оглушительно"}, {}, {"action": "louder", "db": 3}, {"action": 1}])
def test_args_follow_the_catalog_schema(args):
    bad({"actions": [{"tool": "set_volume", "args": args}]}, "bad_args")


def test_number_and_integer_args():
    assert ok({"actions": [{"tool": "move_direction", "args": {"direction": "вперёд", "distance": 1.5}}]})
    bad({"actions": [{"tool": "move_direction", "args": {"direction": "вперёд", "distance": True}}]}, "bad_args")


def test_one_action_per_media_and_motion_class():
    media = {"tool": "request_music", "args": {"intent": "track", "text": "поставь бит"}}
    motion = {"tool": "navigate_to_waypoint", "args": {"waypoint": "кухня"}}
    bad({"actions": [media, {"tool": "stop_music"}]}, "single_action_class")
    bad({"actions": [motion, {"tool": "stop_navigation"}]}, "single_action_class")
    assert len(ok({"actions": [media, motion, {"tool": "get_current_time"}]}).actions) == 3


def test_speak_aloud_is_only_for_operator_and_telegram():
    say = {"actions": [{"tool": "say", "args": {"text": "Привет"}}]}
    bad(say, "class_not_for_agent")
    assert ok(say, session=OPERATOR).actions[0].tool == "say"


# --- выступление, время, правка -----------------------------------------------------------------------------


def test_perform_shape():
    cmd = ok({"perform": _perform(3, backing="beat")})
    assert cmd.perform.segments == ("куплет 0", "куплет 1", "куплет 2") and cmd.perform.backing == "beat"
    bad({"speech": "Сейчас зачитаю", "perform": _perform()}, "perform_with_speech")
    bad({"perform": _perform(1)}, "bad_segments")
    bad({"perform": _perform(kn.SEGMENTS_MAX + 1)}, "bad_segments")
    bad({"perform": {"kind": "rap", "segments": ["а" * (kn.SEGMENT_MAX_CHARS + 1), "б"]}}, "too_long")
    bad({"perform": {"kind": "opera", "segments": ["а", "б"]}}, "bad_enum")
    bad({"perform": _perform(2, expression_per_segment=[{"emotion": "happy"}])}, "bad_segments")


def test_perform_beside_actions_follows_adr_and_excludes_speech():
    """ADR §4.1: ``perform`` взаимоисключает ``speech`` и при действиях; ответ по сути рядом с действием — только
    ``perform`` (правку правила, чтобы невозвучиваемый ``speech`` не валил команду, решает Шифу, #3698 ч.7c)."""
    sound = [{"tool": "play_sound", "args": {"sound": "robot_happy"}}]
    assert ok({"actions": sound, "perform": _perform()}).perform is not None
    bad({"speech": "Включаю звук!", "actions": sound, "perform": _perform()}, "perform_with_speech")


def test_voice_of_a_line_and_voices_of_segments():
    cmd = ok({"speech": "Привет", "voice": " zahar "})
    assert cmd.voice == "zahar"
    tale = ok({"perform": _perform(3, voice_per_segment=["", None, "alena"])})
    assert tale.perform.voice_per_segment == ("", "", "alena")
    bad({"perform": _perform(3, voice_per_segment=["zahar"])}, "bad_segments")
    bad({"perform": _perform(2, voice_per_segment=["a" * (kn.VOICE_MAX_CHARS + 1), ""])}, "too_long")
    bad({"perform": _perform(2, voice_per_segment=[1, ""])}, "bad_type")
    bad({"speech": "Привет", "voice": "z" * (kn.VOICE_MAX_CHARS + 1)}, "too_long")


def test_to_args_round_trips_through_parse():
    raw = {"voice": "zahar", "ask": {"expects": "name", "text": "Как зовут?"},
           "actions": [{"tool": "memory_save", "args": {"fact": "чай"}}],
           "perform": _perform(2, voice_per_segment=["", "alena"], expression_per_segment=[{"emotion": "happy"}] * 2),
           "expression": {"emotion": "happy", "gesture": "wink", "face": {"eyes": "wide"}}}
    cmd = ok(raw)
    assert parse(cm.to_args(cmd)) == cmd
    assert json.loads(json.dumps(cm.to_args(cmd), ensure_ascii=False)) == cm.to_args(cmd)


def test_edit_pending_needs_active_perform_and_a_later_segment():
    edit = {"edit_pending": True, "perform": _perform(4, from_segment=3)}
    bad(edit, "edit_without_active_perform")
    assert ok(edit, busy=Busy(frozenset({"action.perform"}), active_segment=2)).edit_pending
    bad(edit, "edit_started_segment", busy=Busy(active_segment=3))
    bad({"edit_pending": True, "speech": "и про енота"}, "edit_without_perform")
    bad({"perform": _perform(2, from_segment=1)}, "from_segment_without_edit")


def test_when_now_is_cut_against_uninterruptible_busy_class():
    now = {"speech": "сейчас", "when": "now"}
    bad(now, "now_vs_uninterruptible", busy=Busy(frozenset({"action.motion"})))
    assert ok(now, busy=Busy(frozenset({"query"}))).when == "now"
    assert ok({"speech": "потом"}).when == "after_current"
    bad({"when": "tomorrow"}, "bad_enum")


# --- экспрессия ---------------------------------------------------------------------------------------------


def test_expression_enums():
    bad({"expression": {"emotion": "joy"}}, "bad_enum")
    bad({"expression": {"gesture": "dance"}}, "bad_enum")
    bad({"expression": {"earcon": "boop"}}, "bad_enum")
    assert ok({"expression": {"emotion": "sad", "gesture": "wink", "earcon": "heard"}}).expression.earcon == "heard"


def test_valid_face_scene_is_kept():
    face = {"eyes": "wide", "mouth": "teeth", "tint": "red", "rear": "halloween", "duration_s": 10}
    cmd = ok({"speech": "Бу!", "expression": {"emotion": "angry", "face": face}})
    assert cmd.expression.face == FaceScene(eyes="wide", mouth="teeth", tint="red", rear="halloween", duration_s=10)
    assert cmd.dropped == ()


@pytest.mark.parametrize("face", [
    {"tint": "#ff0000"}, {"eyes": "laser"}, {"duration_s": kn.FACE_SCENE_MAX_S + 1}, {"duration_s": "долго"},
    {"pixels": [[1, 0]]}, "scary",
])
def test_bad_face_scene_is_dropped_not_the_command(face):
    cmd = ok({"speech": "Смотри", "expression": {"emotion": "happy", "face": face}})
    assert cmd.expression.face is None and cmd.expression.emotion == "happy" and cmd.speech == "Смотри"
    assert cmd.dropped == ("expression.face",)


def test_bad_face_in_a_segment_is_dropped():
    per = [{"emotion": "happy"}, {"emotion": "sad", "face": {"tint": "neon"}}]
    cmd = ok({"perform": _perform(2, expression_per_segment=per)})
    assert cmd.dropped == ("perform.expression_per_segment[1].face",)
    assert cmd.perform.expression_per_segment[1].emotion == "sad"


# --- контракт -----------------------------------------------------------------------------------------------


def test_schema_fields_match_the_dataclasses():
    schema = kn.command_schema("json")["properties"]

    def names(cls):
        return {f.name for f in dataclasses.fields(cls)}

    assert set(schema) == names(cm.Command) - {"dropped", "moved"}
    assert set(schema["ask"]["properties"]) == names(cm.Ask) - {"key"}
    assert set(schema["perform"]["properties"]) == names(cm.Performance)
    assert set(schema["expression"]["properties"]) == names(cm.Expression)
    assert set(schema["expression"]["properties"]["face"]["properties"]) == names(cm.FaceScene)


def test_validator_has_no_llm_hook_and_no_retry():
    """Проверка — чистая функция без LLM: повтор по нарушениям делает мост режима команд, не валидатор (#3698)."""
    assert list(inspect.signature(validate).parameters) == ["cmd", "available_tools", "session", "busy"]
    assert "retry" not in pathlib.Path(cm.__file__).read_text(encoding="utf-8").lower()
    invalid = Invalid("x")
    assert validate(invalid, ALL) is invalid


def test_schema_validator_against_the_catalog_examples():
    for entry in TOOL_CATALOG:
        assert cm._schema_error(entry.parameters, {"__nope__": 1}, entry.name) is not None, entry.name


# --- случайные команды: ни одного исключения ----------------------------------------------------------------


def _value(rng, depth=0):
    kinds = [None, True, rng.randint(-3, 500), rng.random() * 40, _text(rng)]
    if depth < 2:
        kinds += [[_value(rng, depth + 1) for _ in range(rng.randint(0, 3))],
                  {_text(rng, 1, 6): _value(rng, depth + 1) for _ in range(rng.randint(0, 3))}]
    return rng.choice(kinds)


def _text(rng, low=0, high=40):
    alphabet = "абвгдежзиклмнопрстуфхцчшщыэюя ,.!?{}\"\n\x07" + "abc"
    return "".join(rng.choice(alphabet) for _ in range(rng.randint(low, high)))


def _arg_for(schema, rng):
    if "enum" in schema:
        return rng.choice(list(schema["enum"]))
    kind = schema.get("type")
    return {"string": lambda: _text(rng, 1, 20), "integer": lambda: rng.randint(0, 5),
            "number": lambda: rng.random() * 5, "boolean": lambda: rng.random() < 0.5,
            "array": lambda: [], "object": lambda: {}}.get(kind, lambda: None)()


def _args(tool, rng):
    if tool not in ALL or rng.random() < 0.2:
        return _value(rng)
    props = get_tool(tool).parameters.get("properties", {})
    return {k: _arg_for(v, rng) for k, v in props.items() if rng.random() < 0.8}


def _face(rng):
    face = {slot: rng.choice(values + ("neon",)) for slot, values in kn.FACE_SCENE_SLOTS.items() if rng.random() < 0.4}
    if rng.random() < 0.3:
        face["duration_s"] = rng.choice([1, 5.5, 30, 31, "x"])
    return face


def _random_command(rng):
    d = {}
    if rng.random() < 0.6:
        d["speech"] = _text(rng, 0, 320)
    if rng.random() < 0.35:
        tools = ALL + ["speak_text", "nope"]
        d["actions"] = [{"tool": t, "args": _args(t, rng)} for t in rng.sample(tools, rng.randint(0, 4))]
    if rng.random() < 0.15:
        n = rng.randint(0, 14)
        d["perform"] = {"kind": rng.choice(kn.PERFORM_KINDS + ("opera",)),
                        "segments": [_text(rng, 0, 420) for _ in range(n)],
                        "backing": rng.choice(kn.PERFORM_BACKINGS), "from_segment": rng.randint(0, 4)}
    if rng.random() < 0.5:
        d["expression"] = {"emotion": rng.choice(list(kn.EMOTIONS) + ["joy"]), "face": _face(rng)}
        if rng.random() < 0.3:  # слоты лица уровнем выше, как у DeepSeek (#3698)
            d["expression"].update(d["expression"].pop("face") if rng.random() < 0.7 else _face(rng))
    for key, values in (("when", kn.WHEN + ("soon",)), ("passed", kn.PASSED), ("edit_pending", (True, False))):
        if rng.random() < 0.1:
            d[key] = rng.choice(values)
    if rng.random() < 0.1:
        d["ask"] = {"expects": rng.choice(kn.ASK_EXPECTS + ("maybe",)), "text": _text(rng, 1, 60)}
    if d and rng.random() < 0.15:
        d[rng.choice(list(d))] = _value(rng)
    if rng.random() < 0.05:
        d["mood"] = "лишнее поле"
    return d


def _random_raw(rng):
    d = _random_command(rng)
    form = rng.randrange(6)
    if form == 0:
        return d
    if form == 1:
        return json.dumps(d, ensure_ascii=False)
    if form == 2:
        return f"{_text(rng)} {json.dumps(d, ensure_ascii=False)} {_text(rng)}"
    if form == 3:
        actions = d.pop("actions", [])
        actions = [a for a in actions if isinstance(a, dict)] if isinstance(actions, list) else []
        calls = [{"type": "function", "function": {"name": "command", "arguments": json.dumps(d)}}]
        return calls + [{"function": {"name": a.get("tool"), "arguments": json.dumps(a.get("args"))}} for a in actions]
    if form == 4:
        return _text(rng, 0, 80)
    return _value(rng)


def test_thousands_of_random_commands_never_raise():
    rng = random.Random(3685)
    sessions = [None, PERSONALITY, OPERATOR]
    busies = [Busy(), Busy(frozenset({"action.motion"})), Busy(frozenset({"action.perform"}), active_segment=1)]
    seen = {Command: 0, Invalid: 0}
    for _ in range(3000):
        res = validate(parse(_random_raw(rng)), ALL, rng.choice(sessions), rng.choice(busies))
        seen[type(res)] += 1
        if isinstance(res, Command):
            assert res.speech is None or len(res.speech) <= kn.SPEECH_MAX_CHARS
            assert len(res.actions) <= kn.ACTIONS_MAX
            assert all(a.tool in kn.TOOL_CLASSES for a in res.actions)
            assert not (res.perform and res.speech)
            assert parse(cm.to_args(res)) == dataclasses.replace(res, dropped=(), moved=())  # мост → исполнитель
            assert res.expression.emotion in kn.EMOTIONS
            assert res.expression.face is None or isinstance(res.expression.face, FaceScene)
        else:
            assert res.ask.key == "understand.invalid"
    assert seen[Command] >= 300 and seen[Invalid] >= 300, seen


# --- нарушения словами для модели (повтор команды, #3698) ---------------------------------------------------


def test_validator_reports_every_violation_first_one_is_the_reason():
    res = bad({"speech": "x", "perform": _perform(), "when": "later", "expression": {"earcon": "beep"}},
              "perform_with_speech")
    assert [v.reason for v in res.all] == ["perform_with_speech", "bad_enum", "bad_enum"]
    assert [v.allowed for v in res.all][1:] == [kn.WHEN, kn.EARCONS]


def test_explain_says_what_is_wrong_and_what_is_allowed():
    note = cm.explain(bad({"speech": "x", "expression": {"emotion": "ecstatic"}}, "bad_enum"))
    assert note.splitlines()[0] == cm.INVALID_HEAD
    assert note.splitlines()[1] == (f"- bad_enum: значение не из перечня (emotion=ecstatic). "
                                    f"Допустимо: {', '.join(kn.EMOTIONS)}")
    unknown = cm.explain(bad({"speech": "x", "mood": 1}, "unknown_field"))
    assert "(command: mood)" in unknown and ", ".join(cm._COMMAND_FIELDS) in unknown
    missing = cm.explain(bad({"actions": [{"tool": "drive", "args": {}}]}, "tool_not_available", tools=["memory_save"]))
    assert "Допустимо: memory_save" in missing


def test_every_reason_the_validator_raises_has_words_for_the_model():
    source = pathlib.Path(cm.__file__).read_text(encoding="utf-8")
    raised = set(re.findall(r'_Bad\("([a-z_]+)"', source))
    assert raised and raised <= set(cm.HINTS)
