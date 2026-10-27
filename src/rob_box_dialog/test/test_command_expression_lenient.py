"""Лишние поля экспрессии не бракуют команду (ADR-0150 §5.5.9, решение Шифу 21.10 по #3698).

Корпус — команды DeepSeek из живого прогона 7c (Sound Pi, 21.10 19:37–19:51Z, комментарий #3698): в логе
``🧾 [command] … verdict=invalid:unknown_field detail='expression: …'`` есть только имена полей, полный JSON не
писался — значения слотов ниже подставлены из таблицы ``knowledge.FACE_SCENE_SLOTS``, имена полей — как в логе.
"""

import pytest

from rob_box_core.tool_catalog import TOOL_CATALOG
from rob_box_dialog import command as cm
from rob_box_dialog.command import Command, FaceScene, parse, validate

ALL = [t.name for t in TOOL_CATALOG]
_SOUND = {"tool": "play_sound", "args": {"sound": "robot_happy"}}
_TALE = {"kind": "tale", "segments": ["Жил-был робот.", "И нашёл он лампочку."], "backing": "none",
         "voice_per_segment": ["", "alena"]}

#: (фраза, ответ модели, ожидаемая сцена лица, переложено, отброшено). detail из лога — в комментарии.
LIVE_7C = [
    # 19:39:07 «включи звук радостный» — detail='expression: rear'
    ("звук", {"actions": [_SOUND], "expression": {"emotion": "happy", "rear": "party"}},
     FaceScene(rear="party"), ("expression.rear",), ()),
    # 19:41:43 «включи звук радостный и расскажи шутку» — detail='expression: eyes, mouth, rear, tint'
    ("звук+шутка", {"actions": [_SOUND], "perform": {"kind": "poem", "segments": ["Шутка.", "Смешно!"]},
                    "expression": {"emotion": "happy", "eyes": "wide", "mouth": "grin", "rear": "party",
                                   "tint": "warm"}},
     FaceScene(eyes="wide", mouth="grin", tint="warm", rear="party"),
     ("expression.eyes", "expression.mouth", "expression.rear", "expression.tint"), ()),
    # 19:44:02 «расскажи короткую сказку разными голосами» — detail='expression: duration_s' (сцена уже в face).
    # #4083: ``duration_s`` экспрессии — своё поле (время лица команды), больше не выбрасывается.
    ("сказка", {"perform": _TALE, "expression": {"emotion": "happy", "face": {"eyes": "open"}, "duration_s": 8}},
     FaceScene(eyes="open"), (), ()),
]


@pytest.mark.parametrize("phrase, raw, face, moved, dropped", LIVE_7C, ids=[c[0] for c in LIVE_7C])
def test_live_7c_commands_are_valid_now(phrase, raw, face, moved, dropped):
    cmd = validate(parse(raw), ALL)
    assert isinstance(cmd, Command), (phrase, cmd)
    assert cmd.expression.face == face
    assert cmd.moved == moved and cmd.dropped == dropped
    assert cmd.expression.emotion == "happy"


def test_4083_expression_duration_is_kept_not_dropped():
    """Марафон 23.10 21:53:48 (DeepSeek): ``{"emotion": "sleepy", "face": {…}, "duration_s": 4}`` давал
    ``fields_dropped=['expression.duration_s']`` — время лица команды терялось. Теперь это поле экспрессии."""
    raw = {"speech": "Тихо. Я тоже помолчу — если понадоблюсь, скажи.",
           "expression": {"emotion": "sleepy", "face": {"eyes": "half", "mouth": "flat", "tint": "cold"},
                          "duration_s": 4}}
    cmd = validate(parse(raw), ALL)
    assert isinstance(cmd, Command) and cmd.dropped == () and cmd.moved == ()
    assert cmd.expression.duration_s == 4.0 and cmd.expression.face == FaceScene(eyes="half", mouth="flat", tint="cold")


def test_duration_stays_on_the_expression_when_slots_move():
    cmd = validate(parse({"expression": {"eyes": "closed", "tempo": "slow", "duration_s": 12}}), ALL)
    assert cmd.expression.face == FaceScene(eyes="closed", tempo="slow") and cmd.expression.duration_s == 12.0
    assert cmd.moved == ("expression.eyes", "expression.tempo") and cmd.dropped == ()


def test_duration_alone_is_the_time_of_the_emotion_not_an_empty_scene():
    cmd = validate(parse({"speech": "Ок", "expression": {"emotion": "sad", "duration_s": 3}}), ALL)
    assert cmd.expression.face is None and cmd.expression.duration_s == 3.0
    assert cmd.dropped == () and cmd.moved == ()


@pytest.mark.parametrize("bad", ["долго", True, [5], {"s": 5}])
def test_4083_duration_that_is_not_a_number_is_dropped_and_the_command_stays(bad):
    cmd = validate(parse({"speech": "Ок", "expression": {"emotion": "sad", "duration_s": bad}}), ALL)
    assert isinstance(cmd, Command) and cmd.expression.duration_s is None
    assert cmd.dropped == ("expression.duration_s",)


@pytest.mark.parametrize("unset", [0, 0.0, -3, float("nan")])
def test_4083_duration_zero_or_below_is_not_set(unset):
    """Ревью #4088: ``duration_s=0`` на обычное «привет» давал лицу 8 с показа вместо рефлекса — ноль, отрицательное и
    ``NaN`` значат «время не задано» (``None``), поле не выбрасывается в ``dropped``, команда исполняется."""
    cmd = validate(parse({"speech": "Привет!", "expression": {"emotion": "happy", "duration_s": unset}}), ALL)
    assert isinstance(cmd, Command) and cmd.expression.duration_s is None and cmd.dropped == ()


def test_4083_duration_without_a_face_is_still_an_empty_command():
    """Время без лица (нейтраль) показать нечего — команда пустая, как и без времени (мост: ``empty_command``)."""
    assert cm.is_empty(parse({"expression": {"duration_s": 6}}))
    assert not cm.is_empty(parse({"expression": {"emotion": "sad", "duration_s": 6}}))


def test_slots_next_to_a_given_face_do_not_overwrite_it():
    cmd = validate(parse({"expression": {"face": {"eyes": "wide"}, "eyes": "closed", "tint": "red"}}), ALL)
    assert cmd.expression.face == FaceScene(eyes="wide")
    assert cmd.moved == () and cmd.dropped == ("expression.eyes", "expression.tint")


def test_unknown_expression_fields_are_dropped_and_the_command_stays():
    cmd = validate(parse({"speech": "Привет", "expression": {"emotion": "happy", "mood": "весело", "eyes": "wide"}}),
                   ALL)
    assert cmd.speech == "Привет" and cmd.expression.face == FaceScene(eyes="wide")
    assert cmd.moved == ("expression.eyes",) and cmd.dropped == ("expression.mood",)


def test_moved_scene_with_a_bad_value_is_dropped_by_the_validator_as_before():
    cmd = validate(parse({"speech": "Смотри", "expression": {"tint": "orange"}}), ALL)
    assert cmd.speech == "Смотри" and cmd.expression.face is None
    assert cmd.moved == ("expression.tint",) and cmd.dropped == ("expression.face",)


def test_segment_expressions_are_lenient_too():
    per = [{"emotion": "happy", "mouth": "grin"}, {"emotion": "sad", "color": "#000"}]
    cmd = validate(parse({"perform": {**_TALE, "expression_per_segment": per}}), ALL)
    assert cmd.perform.expression_per_segment[0].face == FaceScene(mouth="grin")
    assert cmd.moved == ("perform.expression_per_segment[0].mouth",)
    assert cmd.dropped == ("perform.expression_per_segment[1].color",)


def test_bridge_args_carry_the_moved_scene_inside_face():
    """Мост v1 отдаёт команду исполнителю через ``to_args`` — сцена уже на своём месте, второй разбор чистый."""
    cmd = parse({"actions": [_SOUND], "expression": {"emotion": "happy", "rear": "party"}})
    args = cm.to_args(cmd)
    assert args["expression"] == {"emotion": "happy", "face": {"rear": "party", "duration_s": 5.0}}
    again = parse(args)
    assert again.moved == () and again.dropped == () and again.expression == cmd.expression


def test_4083_bridge_args_carry_the_expression_duration():
    cmd = parse({"speech": "Смотри", "expression": {"emotion": "sad", "duration_s": 8}})
    args = cm.to_args(cmd)
    assert args["expression"] == {"emotion": "sad", "duration_s": 8.0}
    assert parse(args).expression == cmd.expression


def test_unknown_command_level_field_is_still_invalid():
    """Расширение только для экспрессии (§5.5.9): лишнее поле самой команды по-прежнему невалидно (§4.1)."""
    res = validate(parse({"speech": "Привет", "mood": "весело"}), ALL)
    assert isinstance(res, cm.Invalid) and res.reason == "unknown_field"
