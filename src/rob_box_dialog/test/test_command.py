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
        {"id": "1", "type": "function", "function": {"name": "command", "arguments": '{"expression": {"emotion": "happy"}}'}},
        {"id": "2", "type": "function", "function": {"name": "get_current_time", "arguments": ""}},
        {"name": "set_volume", "arguments": {"action": "louder"}},
    ]
    cmd = ok(calls)
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


def test_3700_command_mode_cannot_order_the_performance_beat():
    """Бит выступления заказывает код выступления, не LLM: ``intent=backing`` вне схемы тула — ``bad_args``."""
    order = {"tool": "request_music", "args": {"intent": kn.BACKING_INTENT, "text": "бит", "min_seconds": 45}}
    bad({"actions": [order]}, "bad_args")
    ok({"actions": [{"tool": "request_music", "args": {"intent": "melody", "text": "калинку"}}]})


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


def test_quiet_actions_beside_speech_are_valid_loud_ones_are_not_3953():
    """Шифу 23.10: ``memory_save``/``register_speaker``/``play_animation`` живут рядом со ``speech``; звук - нет."""
    fact = {"tool": "memory_save", "args": {"fact": "Пользователя зовут Саша"}}
    sound = {"tool": "play_sound", "args": {"sound": "robot_happy"}}
    assert ok({"speech": "Привет, Саша!", "actions": [fact]}).speech == "Привет, Саша!"
    assert ok({"speech": "Привет!", "actions": [{"tool": "register_speaker", "args": {"name": "Борис"}}]}).speech
    bad({"speech": "Привет, Саша! Вот звук", "actions": [fact, sound]}, kn.VIOLATION_SPEECH_WITH_ACTIONS)


def test_invalid_command_offers_its_individually_valid_actions_for_salvage_3953():
    fact = {"tool": "memory_save", "args": {"fact": "x"}}
    sound = {"tool": "play_sound", "args": {"sound": "robot_happy"}}
    broken = {"tool": "play_animation", "args": {"no_such": 1}}
    query = {"tool": "get_current_time", "args": {}}
    res = bad({"speech": "Вот!", "actions": [fact, sound, broken]}, "bad_args")
    assert [a.tool for a in res.salvage] == ["memory_save", "play_sound"]  # битый по схеме не спасаем
    res = bad({"speech": "Вот!", "actions": [sound, query]}, kn.VIOLATION_SPEECH_WITH_ACTIONS)
    assert [a.tool for a in res.salvage] == ["play_sound"]  # query некому вернуть данные


def test_perform_beside_actions_follows_adr_and_excludes_speech():
    """ADR §4.1: ``perform`` взаимоисключает ``speech`` и при действиях; ответ по сути рядом с действием — только
    ``perform`` (правку правила, чтобы невозвучиваемый ``speech`` не валил команду, решает Шифу, #3698 ч.7c)."""
    sound = [{"tool": "play_sound", "args": {"sound": "robot_happy"}}]
    assert ok({"actions": sound, "perform": _perform()}).perform is not None
    bad({"speech": "Включаю звук!", "actions": sound, "perform": _perform()}, "perform_with_speech")


def test_speech_beside_actions_is_a_violation_with_a_hint_for_the_model():
    """#3698 (решение Шифу 22.10): ``speech`` при actions не звучит - ответ по сути пропал бы молча; нарушение
    идёт модели на повтор, подсказка различает «ответ по сути - в perform» и «слова про действие - убрать»."""
    sound = [{"tool": "play_sound", "args": {"sound": "robot_happy"}}]
    res = bad({"speech": "Вот звук!", "actions": sound}, kn.VIOLATION_SPEECH_WITH_ACTIONS)
    note = cm.explain(res)
    assert "положи его в perform" in note and "убери speech" in note
    assert ok({"speech": "Привет!"}).speech == "Привет!"  # без действий речь законна
    assert ok({"actions": sound, "perform": _perform()}).perform is not None  # ответ по сути рядом с действием
    assert ok({"ask": {"text": "Какой?"}, "actions": sound}).ask is not None  # вопрос - не speech
    chat = Session("telegram").snapshot  # у телеграма speech - текст в чат (§4.1), при действиях законен
    assert ok({"speech": "Записала", "actions": sound}, session=chat).speech == "Записала"


def test_voice_of_a_line_and_resolved_voices_of_segments():
    cmd = ok({"speech": "Привет", "voice": " zahar "})
    assert cmd.voice == "zahar"
    # голоса сегментов уже решённой команды (мост → исполнитель; каталога у перепроверки нет) разбор держит как есть
    poem = ok({"perform": _perform(3, kind="poem", voice_per_segment=["", None, "alena"])})
    assert poem.perform.voice_per_segment == ("", "", "alena")
    bad({"perform": _perform(3, kind="poem", voice_per_segment=["zahar"])}, "bad_segments")
    bad({"perform": _perform(2, kind="poem", voice_per_segment=["a" * (kn.VOICE_MAX_CHARS + 1), ""])}, "too_long")
    bad({"perform": _perform(2, voice_per_segment=[1, ""])}, "bad_type")
    bad({"speech": "Привет", "voice": "z" * (kn.VOICE_MAX_CHARS + 1)}, "too_long")


def test_to_args_round_trips_through_parse():
    raw = {"voice": "zahar", "ask": {"expects": "name", "text": "Как зовут?"},
           "actions": [{"tool": "memory_save", "args": {"fact": "чай"}}],
           "perform": {"kind": "tale", "segments": ["Жила-была лиса.", "Привет!"], "role_per_segment": ["", "Лиса"],
                       "cast": [{"name": "Лиса", "gender": "female"}, {"name": "Ёж"}],
                       "expression_per_segment": [{"emotion": "happy"}] * 2},
           "expression": {"emotion": "happy", "gesture": "wink", "face": {"eyes": "wide"}}}
    cmd = ok(raw, busy=Busy(voices=("zahar", "alena"), voice_genders=(("zahar", "male"), ("alena", "female"))))
    assert cmd.perform.voice_per_segment == ("", "alena") and cmd.dropped == ()
    assert parse(cm.to_args(cmd)) == cmd
    assert json.loads(json.dumps(cm.to_args(cmd), ensure_ascii=False)) == cm.to_args(cmd)


def test_edit_pending_needs_active_perform_and_code_picks_the_segment():
    edit = {"edit_pending": True, "perform": _perform(4)}
    bad(edit, "edit_without_active_perform")
    assert ok(edit, busy=Busy(frozenset({"action.perform"}), active_segment=2)).edit_pending
    bad({"edit_pending": True, "speech": "и про енота"}, "edit_without_perform")
    # номер модели не бракует и не исполняется: с какого править — решает хост выступления (#3700)
    late = ok({"edit_pending": True, "perform": _perform(4, from_segment=1)}, busy=Busy(active_segment=3))
    assert late.perform.from_segment == 0 and late.dropped == ("perform.from_segment",)
    assert ok({"perform": _perform(2, from_segment=1)}).perform.from_segment == 0


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
    # с какого сегмента править и голоса сегментов выбирает код (#3700, #4084)
    assert set(schema["perform"]["properties"]) == names(cm.Performance) - {"from_segment", "voice_per_segment"}
    assert set(schema["perform"]["properties"]["cast"]["items"]["properties"]) == names(cm.Role)
    assert set(schema["expression"]["properties"]) == names(cm.Expression)
    assert set(schema["expression"]["properties"]["face"]["properties"]) == names(cm.FaceScene)


def test_validator_has_no_llm_hook_and_no_retry():
    """Проверка — чистая функция без LLM: повтор по нарушениям делает мост режима команд, не валидатор (#3698).
    ``form`` — форма команды хода (#4021): данные, а не крючок к LLM."""
    assert list(inspect.signature(validate).parameters) == ["cmd", "available_tools", "session", "busy", "form"]
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
    assert kn.VIOLATION_EMPTY_COMMAND in cm.HINTS  # нарушение моста (#3698) — тоже со словами для модели


@pytest.mark.parametrize("raw, empty", [
    ({"expression": {"emotion": "happy"}}, False),  # #3974: «покажи, как тебе грустно» - команда лицом без слов
    ({"expression": {"gesture": "wink"}}, False),
    ({"expression": {"emotion": "neutral"}}, True),
    ({"speech": "   "}, True),
    ({}, True),
    ({"passed": "nothing_to_say"}, False),
    ({"speech": "Привет"}, False),
    ({"ask": {"expects": "name", "text": "Как тебя зовут?"}}, False),
    ({"actions": [{"tool": "get_current_time", "args": {}}]}, False),
])
def test_is_empty_marks_commands_with_nothing_to_do_or_say(raw, empty):
    """#3698: ответ модели без речи, вопроса, действий и выступления (и не passed) мост считает нарушением."""
    assert cm.is_empty(ok(raw)) is empty


def test_tool_outside_the_slice_names_same_class_tools_of_the_slice_first():
    """#3897: play_sound вне среза → в перечне только тулы среза того же класса (action.media); класс неизвестен — весь срез."""
    slice_ = ["request_music", "stop_music", "dj_set", "memory_save", "get_current_time"]
    res = bad({"actions": [{"tool": "play_sound", "args": {}}]}, "tool_not_available", tools=slice_)
    assert res.all[0].allowed == ("dj_set", "request_music", "stop_music")
    assert "Допустимо: dj_set, request_music, stop_music" in cm.explain(res)
    unknown = bad({"actions": [{"tool": "no_such_tool"}]}, "tool_not_available", tools=slice_)
    assert unknown.all[0].allowed == ("dj_set", "get_current_time", "memory_save", "request_music", "stop_music")


def test_explain_after_query_says_data_received_not_nothing_done():
    """#3883: после выполненного запроса подсказка повтора честная: данные получены, речь не длиннее потолка."""
    invalid = cm.Invalid("too_long", "слишком длинно")
    plain, after = cm.explain(invalid).splitlines()[0], cm.explain(invalid, after_query=True).splitlines()[0]
    assert plain == cm.INVALID_HEAD and "ничего не сделано" in plain
    assert after == cm.INVALID_AFTER_QUERY_HEAD and "ничего не сделано" not in after
    assert "получены" in after and str(kn.SPEECH_MAX_CHARS) in after and "без actions" in after


def test_query_data_message_is_recognised_by_the_field_turn_writes():
    from rob_box_dialog import turn as turn_mod
    row = turn_mod.ActionResult("t/0", 1, "personality", "get_current_time", "query", "done", None, {"time": "17:00"})
    assert cm.is_query_data(turn_mod._query_data([row]))
    assert not cm.is_query_data("invalid arguments: ...") and not cm.is_query_data("")


# --- сказка разными голосами (#3959) ---------------------------------------------------------------------------

TALE_SEGMENTS = ["Жила-была девочка.", "Куда идёшь?", "К бабушке."]
VOICES = ("alena", "zahar", "filipp")


GENDERS = (("alena", "female"), ("zahar", "male"), ("filipp", "male"))
POOL = cm.Busy(voices=VOICES, voice_genders=GENDERS)


def test_tale_without_cast_is_a_violation_with_a_hint_3959_4084():
    """Марафон 23.10 акт 4: сказка без ролей прозвучала одним голосом, а валидатор молчал. Теперь сказка обязана
    назвать героев (``cast``, пусто — героев нет) и чья реплика в сегменте; голоса им даёт код."""
    res = bad({"perform": {"kind": "tale", "segments": TALE_SEGMENTS}}, "tale_without_cast", busy=POOL)
    note = cm.explain(res)
    assert "perform.cast обязателен" in note and "реплика одного говорящего" in note and "голоса героям даст код" in note


def test_tale_without_heroes_is_the_robot_voice_4084():
    cmd = ok({"perform": {"kind": "tale", "segments": TALE_SEGMENTS, "cast": []}}, busy=POOL)
    assert cmd.perform.voice_per_segment is None


def test_model_voices_of_a_tale_are_replaced_by_the_code_4084():
    """Голос сегмента модель больше не выбирает: её ``voice_per_segment`` — в ``dropped``, голоса — по ролям."""
    raw = {"perform": {"kind": "tale", "segments": TALE_SEGMENTS, "voice_per_segment": ["", "Wise_Woman", "zahar"],
                       "cast": [{"name": "Волк", "gender": "male"}], "role_per_segment": ["", "волк", "Волк"]}}
    cmd = ok(raw, busy=POOL)
    assert cmd.perform.voice_per_segment == ("", "zahar", "zahar") and cmd.dropped == ("perform.voice_per_segment",)


def test_role_outside_the_cast_is_a_violation_with_the_cast_names_4084():
    raw = {"perform": {"kind": "tale", "segments": TALE_SEGMENTS, "cast": [{"name": "Волк", "gender": "male"}],
                       "role_per_segment": ["", "Волк", "Бабушка"]}}
    res = bad(raw, "bad_cast", busy=POOL)
    assert res.violations[0].allowed == ("Волк", "") and "Бабушка" in res.violations[0].detail
    assert "только name героя из cast" in cm.explain(res)


@pytest.mark.parametrize("perform, detail", [
    ({"cast": [{"name": "Волк", "gender": "male"}]}, "без role_per_segment"),
    ({"cast": [{"name": n, "gender": "male"} for n in "абвгд"], "role_per_segment": ["", "а", "б"]}, "героев 5"),
])
def test_cast_structure_violations_4084(perform, detail):
    res = bad({"perform": {"kind": "tale", "segments": TALE_SEGMENTS, **perform}}, "bad_cast", busy=POOL)
    assert detail in res.violations[0].detail


def test_more_speaking_heroes_than_role_voices_is_a_violation_4084():
    four = ["Жил-был кот.", "Мяу.", "Гав.", "Ква."]
    raw = {"perform": {"kind": "tale", "segments": four, "role_per_segment": ["", "кот", "пёс", "жаба"],
                       "cast": [{"name": "кот"}, {"name": "пёс"}, {"name": "жаба"}]}}
    small = cm.Busy(voices=("alena", "zahar"))
    assert "голосов для них 2" in bad(raw, "bad_cast", busy=small).violations[0].detail
    assert len(set(ok(raw, busy=POOL).perform.voice_per_segment) - {""}) == 3


def test_voices_held_by_the_running_performance_are_not_free_for_new_heroes_4112():
    """Правка идущей сказки (ревью PR #4112): герои выступления (``Busy.cast``) держат свои голоса, и новым героям
    правки каталог считается без них. Каталог из четырёх голосов, два заняты: двое новых героев - можно, трое - нарушение
    (раньше сверка шла со ВСЕМ каталогом, и третий получил бы голос рассказчика)."""
    held = cm.Busy(frozenset({"action.perform"}), active_segment=1, voices=("alena", "zahar", "filipp", "arina"),
                   cast=(("волк", "alena"), ("лиса", "zahar")))
    names = ("волк", "кот", "пёс", "жаба")

    def edit(heroes):
        cast = [{"name": n} for n in heroes]
        roles = ["", *heroes]
        return {"edit_pending": True, "perform": {"kind": "tale", "segments": ["Дальше."] * len(roles), "cast": cast,
                                                  "role_per_segment": roles}}

    res = bad(edit(names), "bad_cast", busy=held)
    assert "свободных голосов для них 2" in res.violations[0].detail and "идущего выступления" in res.violations[0].detail
    cmd = ok(edit(names[:3]), busy=held)  # волк держит свой голос, кот и пёс берут два свободных
    voices = dict(zip(cmd.perform.role_per_segment, cmd.perform.voice_per_segment))
    assert voices["волк"] == "alena" and {voices["кот"], voices["пёс"]} == {"filipp", "arina"} and voices[""] == ""


def test_hero_of_the_running_performance_does_not_ask_for_a_new_voice_4112():
    """Все герои правки уже звучат своими голосами: свободных голосов нет, и это не нарушение."""
    held = cm.Busy(frozenset({"action.perform"}), active_segment=1, voices=("alena", "zahar"),
                   cast=(("волк", "alena"), ("лиса", "zahar")))
    raw = {"edit_pending": True, "perform": {"kind": "tale", "segments": ["Волк: — Я тут.", "Лиса: — И я."],
                                             "cast": [{"name": "Лиса"}, {"name": "Волк"}],
                                             "role_per_segment": ["Волк", "Лиса"]}}
    assert ok(raw, busy=held).perform.voice_per_segment == ("alena", "zahar")


def test_role_list_of_a_wrong_length_names_the_models_field_not_the_codes_4112():
    """Роли не по числу сегментов: нарушение называет ``role_per_segment`` - поле, которое пишет модель. Голоса сегментов
    решает код (``voice_per_segment``), модель такого поля не видит."""
    raw = {"perform": {"kind": "tale", "segments": TALE_SEGMENTS, "cast": [{"name": "Волк", "gender": "male"}],
                       "role_per_segment": ["", "Волк"]}}
    res = bad(raw, "bad_segments", busy=POOL)
    assert res.violations[0].detail == "role_per_segment не по числу сегментов"
    assert "voice_per_segment" not in cm.explain(res) and "role_per_segment" in cm.explain(res)


def test_role_names_length_is_checked_4084():
    long = "я" * (kn.ROLE_NAME_MAX_CHARS + 1)
    raw = {"perform": {"kind": "tale", "segments": TALE_SEGMENTS[:2], "cast": [{"name": long}],
                       "role_per_segment": ["", long]}}
    bad(raw, "too_long", busy=POOL)


def test_cast_items_are_lenient_but_the_model_does_not_pick_a_voice_4084():
    """Герой строкой — без пола; пол вне перечня и голос героя от модели — в ``dropped``, команда не отвергается."""
    raw = {"perform": {"kind": "tale", "segments": TALE_SEGMENTS, "role_per_segment": ["", "Лиса", "Волк"],
                       "cast": ["Волк", {"name": "Лиса", "gender": "child", "voice": "zahar"}]}}
    cmd = ok(raw, busy=POOL)
    assert cmd.perform.cast == (cm.Role("Волк"), cm.Role("Лиса"))
    assert set(cmd.dropped) == {"perform.cast[1].voice", "perform.cast[1].gender"}
    assert cmd.perform.voice_per_segment == ("", "alena", "zahar")
    bad({"perform": {"kind": "tale", "segments": TALE_SEGMENTS, "cast": [{"gender": "male"}]}}, "bad_type")


def test_only_tale_needs_a_cast_and_a_backing_only_edit_does_not_3959():
    ok({"perform": {"kind": "poem", "segments": TALE_SEGMENTS}}, busy=POOL)
    ok({"edit_pending": True, "perform": {"kind": "tale", "backing": "calm", "segments": []}},
       busy=cm.Busy(active_segment=1))


def test_song_and_rap_are_sung_in_the_robot_voice_whatever_the_model_sent_4084():
    """Живой случай 27.10: песня про енота — куплеты через один голосом ReliableMan и HandsomeChildhoodFriend."""
    for kind in ("song", "rap"):
        raw = {"perform": {"kind": kind, "segments": TALE_SEGMENTS, "voice_per_segment": ["alena", "zahar", "alena"],
                           "cast": [{"name": "Енот", "gender": "male"}], "role_per_segment": ["Енот", "", "Енот"]}}
        for busy in (POOL, cm.Busy()):
            cmd = ok(raw, busy=busy)
            assert (cmd.perform.voice_per_segment, cmd.perform.cast, cmd.perform.role_per_segment) == (None,) * 3
            assert set(cmd.dropped) == {"perform.voice_per_segment", "perform.cast", "perform.role_per_segment"}


def test_an_edit_keeps_the_voices_of_the_heroes_already_heard_4084():
    """Правка идущей сказки: ``Busy.cast`` — роли и голоса выступления; новый герой — свободный голос."""
    busy = cm.Busy(frozenset({"action.perform"}), active_segment=1, voices=VOICES, voice_genders=GENDERS,
                   cast=(("лиса", "filipp"),))
    raw = {"edit_pending": True, "perform": {"kind": "tale", "segments": TALE_SEGMENTS, "role_per_segment": [
        "Волк", "Лиса", ""], "cast": [{"name": "Волк", "gender": "male"}, {"name": "Лиса", "gender": "female"}]}}
    assert ok(raw, busy=busy).perform.voice_per_segment == ("zahar", "filipp", "")


def test_voice_catalogue_goes_into_the_schema_as_an_enum_of_the_line_voice_only_4084():
    schema = kn.with_voices(kn.command_schema("json"), VOICES)
    assert schema["properties"]["voice"]["enum"] == ["", "alena", "filipp", "zahar"]
    assert "voice_per_segment" not in schema["properties"]["perform"]["properties"]
    assert kn.with_voices(kn.command_schema("json"), ()) == kn.command_schema("json")


# --- #3974: вложенные лишние поля аргументов тула дропаются, не отказывают ---------------------------------------


def _animation(**face):
    return {"actions": [{"tool": "play_animation", "args": {"animation": "sad", "face": {"mouth": "frown", **face}}}]}


def test_3974_nested_extra_field_in_tool_args_is_dropped_not_rejected():
    """n905: ``play_animation.face.emotion: лишнее поле`` валило команду. Теперь поле уходит в ``dropped``."""
    cmd = ok(_animation(emotion="sad"))
    assert dict(cmd.actions[0].args) == {"animation": "sad", "face": {"mouth": "frown"}}
    assert cmd.dropped == ("actions[0].play_animation.face.emotion",)


def test_3974_dropping_nested_extras_does_not_weaken_the_schema():
    bad({"actions": [{"tool": "play_animation", "args": {"animation": "sad", "face": {"mouth": "frown",
                                                                                           "emotion": "sad", "eyes": "?"}}}]},
        "bad_args")  # лишнее ушло, но enum слота eyes всё ещё проверяется
    bad({"actions": [{"tool": "play_animation", "args": {"animation": "sad", "face": "грустное"}}]}, "bad_args")  # тип
    bad({"actions": [{"tool": "set_volume", "args": {"action": "louder", "db": 3}}]}, "bad_args")  # верхний уровень
    bad({"actions": [{"tool": "set_volume", "args": {"db": 3}}]}, "bad_args")  # обязательное поле


def test_3974_clean_args_are_untouched_and_leave_dropped_empty():
    cmd = ok(_animation())
    assert cmd.dropped == () and dict(cmd.actions[0].args)["face"] == {"mouth": "frown"}
